use crate::shared::startup;
use brush_ui::app::App;
use std::os::raw::c_void;
use std::sync::Arc;

#[allow(non_snake_case)]
#[unsafe(no_mangle)]
pub extern "system" fn JNI_OnLoad(vm: jni::JavaVM, _: *mut c_void) -> jni::sys::jint {
    let vm_ref = Arc::new(vm);
    rrfd::android::jni_initialize(vm_ref);
    jni::sys::JNI_VERSION_1_6
}

#[cfg(target_os = "android")]
fn call_java_static(method: &str) {
    let vm = rrfd::android::get_jvm().expect("JVM not initialized");
    let mut env = vm.attach_current_thread().unwrap();
    let class = env.find_class("com/splats/app/MainActivity").unwrap();
    env.call_static_method(class, method, "()V", &[])
        .unwrap_or_else(|e| panic!("Failed to call {method}: {e:?}"));
}

#[unsafe(no_mangle)]
fn android_main(app: winit::platform::android::activity::AndroidApp) {
    let wgpu_options = brush_ui::create_egui_options();

    startup();

    tokio::runtime::Builder::new_multi_thread()
        .enable_all()
        .build()
        .unwrap()
        .block_on(async {
            android_logger::init_once(
                android_logger::Config::default().with_max_level(log::LevelFilter::Info),
            );

            eframe::run_native(
                "Brush",
                eframe::NativeOptions {
                    // Build app display.
                    viewport: egui::ViewportBuilder::default(),
                    android_app: Some(app),
                    wgpu_options,
                    ..Default::default()
                },
                Box::new(|cc| {
                    let app = App::new(cc, None);

                    #[cfg(target_os = "android")]
                    {
                        let ctx = app.context();
                        ctx.register_platform_action("choose_mp4",
                            Box::new(|| call_java_static("chooseMp4")));
                        ctx.register_platform_action("extract_frames",
                            Box::new(|| call_java_static("extractFrames")));
                        ctx.register_platform_action("choose_csv",
                            Box::new(|| call_java_static("chooseCsv")));
                        ctx.register_platform_action("telemetry",
                            Box::new(|| call_java_static("runTelemetry")));
                        ctx.register_platform_action("pose_estimation",
                            Box::new(|| call_java_static("runPoseEstimation")));
                        ctx.register_platform_action("bundle_alignment",
                            Box::new(|| call_java_static("runBundleAlignment")));
                        ctx.register_platform_action("open_in_viewer",
                            Box::new(|| call_java_static("openInViewer")));
                        ctx.register_platform_action("save_ply",
                            Box::new(|| call_java_static("savePly")));
                    }

                    Ok(Box::new(app))
                }),
            )
            .unwrap();
        });
}
