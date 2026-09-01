#![recursion_limit = "256"]

use burn_wgpu::{
    RuntimeOptions, WgpuDevice,
    graphics::{AutoGraphicsApi, GraphicsApi},
};
pub use brush_sfm::{
    BaResult,
    BaState,
    CameraIntrinsics,
    GlobalSfmState,
    GpsPrior,
    ImuRotationPrior,
    LmConfig,
    Observation,
    SlidingWindowConfig,
    axis_angle_to_rotation,
    global_state_to_ply_bytes,
    rotation_log,
    run_levenberg_marquardt,
    run_sliding_window_ba,
    sparse_points_to_ply_bytes,
    write_global_state_ply,
    write_sparse_points_ply,
};
use wgpu::{Adapter, Device, Queue};

pub mod config;
pub mod message;

#[cfg(feature = "training")]
pub mod train_stream;

#[cfg(feature = "training")]
pub mod args_file;

pub mod slot;
// Stage 3.7 BA is always available (pure Rust, no feature gate needed for the logic)
pub mod sfm;

// JNI bridge symbols are only compiled in when the feature is active.
// This keeps the desktop/CI build clean (no jni crate needed there).
#[cfg(feature = "jni-support")]
pub use sfm::stage_3_7_bundle_adjustment::jni_bridge::*;

use std::pin::{Pin, pin};

use anyhow::Error;
use async_fn_stream::try_fn_stream;
use brush_render::MainBackend;
use brush_render::gaussian_splats::{SplatRenderMode, Splats};
use brush_vfs::{DataSource, SendNotWasm};
use burn_cubecl::cubecl::Runtime;
use burn_wgpu::WgpuRuntime;
use tokio_stream::{Stream, StreamExt};

use crate::{message::ProcessMessage, slot::Slot};

pub trait ProcessStream: Stream<Item = Result<ProcessMessage, Error>> + SendNotWasm {}
impl<T> ProcessStream for T where T: Stream<Item = Result<ProcessMessage, Error>> + SendNotWasm {}

pub struct RunningProcess {
    pub stream: Pin<Box<dyn ProcessStream>>,
    pub splat_view: Slot<Splats<MainBackend>>,
}

use tokio::sync::SetOnce;

fn burn_options() -> RuntimeOptions {
    RuntimeOptions {
        tasks_max: 64,
        memory_config: burn_wgpu::MemoryConfiguration::ExclusivePages,
    }
}

pub async fn burn_init_setup() -> WgpuDevice {
    burn_wgpu::init_setup_async::<AutoGraphicsApi>(&WgpuDevice::DefaultDevice, burn_options())
        .await;
    connect_device(WgpuDevice::DefaultDevice);
    WgpuDevice::DefaultDevice
}

static DEVICE: SetOnce<WgpuDevice> = SetOnce::const_new();
static MAX_BUFFER_SIZE: SetOnce<u64> = SetOnce::const_new();

pub(crate) fn connect_device(device: WgpuDevice) {
    DEVICE.set(device).unwrap();
}

pub fn burn_init_device(adapter: Adapter, device: Device, queue: Queue) -> WgpuDevice {
    let adapter_limit = adapter.limits().max_buffer_size;
    let adapter_info = adapter.get_info();

    // Qualcomm Adreno Vulkan drivers report an inflated max_buffer_size to wgpu
    // (e.g. 2 GB) that does not reflect the real per-allocation hardware limit
    // (~128 MB on Adreno 725 / Snapdragon 8 Gen 1). Detect this by vendor ID
    // and clamp only for Qualcomm on Android. Other vendors (Mali, Tensor, etc.)
    // report accurate limits so we leave those untouched.
    let max_buffer_size = {
        const QUALCOMM_VENDOR_ID: u32 = 0x5143;
        // The inflated wgpu default is 2 GB; anything suspicious is > 256 MB.
        const QCOM_ANDROID_CAP: u64 = 128 * 1024 * 1024;
        const INFLATION_THRESHOLD: u64 = 256 * 1024 * 1024;

        #[cfg(target_os = "android")]
        let clamped = if adapter_info.vendor == QUALCOMM_VENDOR_ID
            && adapter_limit > INFLATION_THRESHOLD
        {
            QCOM_ANDROID_CAP
        } else {
            adapter_limit
        };
        #[cfg(not(target_os = "android"))]
        let clamped = adapter_limit;

        clamped
    };

    log::info!(
        "[BRUSH_FLOW] burn_init_device: vendor=0x{:04X} adapter_limit={:.1}MB effective_limit={:.1}MB",
        adapter_info.vendor,
        adapter_limit as f64 / (1024.0 * 1024.0),
        max_buffer_size as f64 / (1024.0 * 1024.0),
    );
    let _ = MAX_BUFFER_SIZE.set(max_buffer_size);

    let setup = burn_wgpu::WgpuSetup {
        instance: wgpu::Instance::new(&wgpu::InstanceDescriptor::default()), // unused... need to fix this in Burn.
        adapter,
        device,
        queue,
        backend: AutoGraphicsApi::backend(),
    };
    let burn = burn_wgpu::init_device(setup, burn_options());
    connect_device(burn.clone());
    burn
}

/// Create a running process from a datasource and args.
///
/// The `config_fn` callback receives the initial config (loaded from args.txt if present,
/// otherwise defaults) and returns the final config to use. This allows the caller to
/// modify or override settings as needed.
pub fn create_process<
    #[cfg(feature = "training")] Fun: FnOnce(crate::config::TrainStreamConfig) -> Fut + Send + 'static,
    #[cfg(feature = "training")] Fut: std::future::Future<Output = crate::config::TrainStreamConfig> + Send,
>(
    source: DataSource,
    #[cfg(feature = "training")] config_fn: Fun,
) -> RunningProcess {
    let splat_view = Slot::default();
    let splat_state_cl = splat_view.clone();
    log::info!("[BRUSH_FLOW] create_process called with source: {:?}", source);

    let stream = try_fn_stream(|emitter| async move {
        log::info!("Starting process with source {source:?}");
        emitter.emit(ProcessMessage::NewProcess).await;

        // Wait until the devise is set.
        log::info!("[BRUSH_FLOW] create_process: Waiting for GPU device...");
        let device = DEVICE.wait().await.clone();
        let vfs = source.clone().into_vfs().await?;
        log::info!("[BRUSH_FLOW] create_process: VFS mounted successfully.");
        let vfs_counts = vfs.file_count();

        if vfs_counts == 0 {
            return Err(anyhow::anyhow!("No files found."));
        }

        let ply_count = vfs.files_with_extension("ply").count();

        log::info!(
            "Mounted VFS with {} files. (plys: {})",
            vfs.file_count(),
            ply_count
        );

        let is_training = vfs_counts != ply_count;

        // Emit source info - just the display name
        let paths: Vec<_> = vfs.file_paths().collect();
        let source_name = if let Some(base_path) = vfs.base_path() {
            base_path
                .file_name()
                .and_then(|n| n.to_str())
                .unwrap_or(if is_training { "dataset" } else { "file" })
                .to_owned()
        } else if paths.len() == 1 {
            paths[0]
                .file_name()
                .and_then(|n| n.to_str())
                .unwrap_or("input.ply")
                .to_owned()
        } else {
            format!("{} files", paths.len())
        };

        let base_path = vfs.base_path();

        // Load initial config from args.txt via VFS if present
        #[cfg(feature = "training")]
        let initial_config = crate::args_file::load_config_from_vfs(&vfs).await;

        emitter
            .emit(ProcessMessage::StartLoading {
                name: source_name,
                source,
                training: is_training,
                base_path,
            })
            .await;

        if !is_training {
            let mut paths: Vec<_> = vfs.file_paths().collect();
            alphanumeric_sort::sort_path_slice(&mut paths);
            let client = WgpuRuntime::client(&device);
            let total_frames = paths.len() as u32;

            for (frame, path) in paths.iter().enumerate() {
                log::info!("[BRUSH_FLOW] create_process: Loading frame {} from ply file: {:?}", frame, path);

                let mut splat_stream = pin!(brush_serde::stream_splat_from_ply(
                    vfs.reader_at_path(path).await?,
                    None,
                    true,
                ));

                while let Some(message) = splat_stream.next().await {
                    let message = message?;
                    log::info!("[BRUSH_FLOW] create_process: Splat frame received and parsed.");

                    {
                        let n_splats = message.data.num_splats() as u64;
                        let means_size = n_splats * 3 * 4;
                        let rot_size = n_splats * 4 * 4;
                        let scale_size = n_splats * 3 * 4;
                        let opac_size = n_splats * 4;
                        let sh_coeffs_len = message
                            .data
                            .sh_coeffs
                            .as_ref()
                            .map_or(n_splats * 3, |c| c.len() as u64);
                        let sh_size = sh_coeffs_len * 4;

                        let max_req = means_size
                            .max(rot_size)
                            .max(scale_size)
                            .max(opac_size)
                            .max(sh_size);

                        // 128 MB fallback: Adreno and other mobile drivers report inflated limits.
                        // burn_init_device already clamps to 128 MB on Android, but use the
                        // same constant here in case MAX_BUFFER_SIZE was never set (race).
                        const SAFE_FALLBACK_LIMIT: u64 = 128 * 1024 * 1024;
                        let max_buf = MAX_BUFFER_SIZE.get().copied().unwrap_or(SAFE_FALLBACK_LIMIT);

                        log::info!(
                            "[BRUSH_FLOW] Buffer check: n_splats={n_splats} \
                             largest={:.1}MB limit={:.1}MB",
                            max_req as f64 / (1024.0 * 1024.0),
                            max_buf as f64 / (1024.0 * 1024.0),
                        );

                        if max_req > max_buf {
                            anyhow::bail!(
                                "Splat too large: needs {:.1} MB but device limit is {:.1} MB. \
                                 Try a smaller or subsampled PLY file.",
                                max_req as f64 / (1024.0 * 1024.0),
                                max_buf as f64 / (1024.0 * 1024.0)
                            );
                        }
                    }

                    let mode = message.meta.render_mode.unwrap_or(SplatRenderMode::Default);
                    let splats = message.data.into_splats(&device, mode);

                    // As loading concatenates splats each time, memory usage tends to accumulate a lot
                    // over time. Clear out memory after each step to prevent this buildup.
                    client.memory_cleanup();

                    // For the first frame of a new file, clear existing frames
                    if frame == 0 {
                        splat_view.clear().await;
                    }

                    // Capture stats before moving splats
                    let num_splats = splats.num_splats();
                    let sh_degree = splats.sh_degree();
                    splat_view.set_at(frame, splats).await;

                    emitter
                        .emit(ProcessMessage::SplatsUpdated {
                            up_axis: message.meta.up_axis,
                            frame: frame as u32,
                            total_frames,
                            num_splats,
                            sh_degree,
                        })
                        .await;
                }
            }

            emitter.emit(ProcessMessage::DoneLoading).await;
        } else {
            #[cfg(feature = "training")]
            {
                // Pass initial config (from args.txt or defaults) to the callback
                let base_config = initial_config.unwrap_or_default();
                let config = config_fn(base_config).await;
                crate::train_stream::train_stream(vfs, config, device, emitter, splat_view).await?;
            }

            #[cfg(not(feature = "training"))]
            anyhow::bail!("Training is not enabled in Brush, cannot load dataset.");
        };

        Ok(())
    });

    RunningProcess {
        stream: Box::pin(stream),
        splat_view: splat_state_cl,
    }
}
