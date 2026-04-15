#![recursion_limit = "256"]

pub mod app;
pub mod camera_controls;

pub mod ui_process;

mod panels;
mod scene;
pub mod splat_backbuffer;
#[cfg(feature = "training")]
mod stats;
mod widget_3d;

#[cfg(feature = "training")]
mod datasets;

#[cfg(feature = "training")]
mod training_panel;

#[cfg(feature = "training")]
mod settings_popup;

use eframe::egui_wgpu::WgpuConfiguration;
use std::sync::Arc;
use wasm_bindgen::prelude::*;
use wgpu::{Adapter, Features};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[wasm_bindgen]
pub enum UiMode {
    // Default UI with data loading, stats view etc.
    #[default]
    Default,
    // Show the splat fullscreen, but allow toggling, and control panels.
    FullScreenSplat,
    // Render as an embedded viewer which does nothing except render the splat.
    EmbeddedViewer,
}

pub fn create_egui_options() -> WgpuConfiguration {
    create_egui_options_with_hints(None)
}

pub fn create_egui_options_with_hints(device_hint: Option<&str>) -> WgpuConfiguration {
    let mut backends = wgpu::Backends::all();

    #[cfg(target_os = "android")]
    {
        // Android stability: Vulkan is extremely fragile on Adreno/Qualcomm (Poco/Xiaomi).
        // GLES is universally stable and thus our default.
        backends = wgpu::Backends::GL;

        if let Some(hint) = device_hint {
            let hint_lower = hint.to_lowercase();
            // Whitelist Pixels/Google devices (Tensor G4 for the demo prototype) for modern Vulkan support.
            if hint_lower.contains("pixel") || hint_lower.contains("google") || hint_lower.contains("tensor") {
                log::info!("High-compliance hardware detected ({}): enabling Vulkan.", hint);
                backends = wgpu::Backends::all();
            } else {
                log::info!("Android hardware ({}) detected: defaulting to stable GLES path.", hint);
            }
        }
    }

    WgpuConfiguration {
        wgpu_setup: eframe::egui_wgpu::WgpuSetup::CreateNew(
            eframe::egui_wgpu::WgpuSetupCreateNew {
                instance_descriptor: wgpu::InstanceDescriptor {
                    backends,
                    ..Default::default()
                },
                power_preference: wgpu::PowerPreference::HighPerformance,
                device_descriptor: Arc::new(|adapter: &Adapter| wgpu::DeviceDescriptor {
                    label: Some("egui+burn"),
                    required_features: adapter
                        .features()
                        .difference(wgpu::Features::MAPPABLE_PRIMARY_BUFFERS | wgpu::Features::TEXTURE_ADAPTER_SPECIFIC_FORMAT_FEATURES),
                    required_limits: adapter.limits(),
                    memory_hints: wgpu::MemoryHints::MemoryUsage,
                    trace: wgpu::Trace::Off,
                }),
                ..Default::default()
            },
        ),
        present_mode: wgpu::PresentMode::Fifo,
        ..Default::default()
    }
}

pub fn draw_checkerboard(ui: &mut egui::Ui, rect: egui::Rect, color: egui::Color32) {
    let id = egui::Id::new("checkerboard");
    let handle = ui
        .ctx()
        .data(|data| data.get_temp::<egui::TextureHandle>(id));

    let handle = handle.unwrap_or_else(|| {
        let color_1 = [190, 190, 190, 255];
        let color_2 = [240, 240, 240, 255];

        let pixels = vec![color_1, color_2, color_2, color_1]
            .into_iter()
            .flatten()
            .collect::<Vec<u8>>();

        let texture_options = egui::TextureOptions {
            magnification: egui::TextureFilter::Nearest,
            minification: egui::TextureFilter::Nearest,
            wrap_mode: egui::TextureWrapMode::Repeat,
            mipmap_mode: None,
        };

        let tex_data = egui::ColorImage::from_rgba_unmultiplied([2, 2], &pixels);

        let handle = ui.ctx().load_texture("checker", tex_data, texture_options);
        ui.ctx().data_mut(|data| {
            data.insert_temp(id, handle.clone());
        });
        handle
    });

    let uv = egui::Rect::from_min_max(
        egui::pos2(0.0, 0.0),
        egui::pos2(rect.width() / 24.0, rect.height() / 24.0),
    );

    ui.painter().image(handle.id(), rect, uv, color);
}
