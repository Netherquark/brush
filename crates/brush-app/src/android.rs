use crate::shared::startup;
use brush_ui::app::App;
use std::os::raw::c_void;
use std::sync::{Arc, RwLock};

#[allow(non_snake_case)]
#[unsafe(no_mangle)]
pub extern "system" fn JNI_OnLoad(vm: jni::JavaVM, _: *mut c_void) -> jni::sys::jint {
    let vm_ref = Arc::new(vm);
    rrfd::android::jni_initialize(vm_ref.clone());

    let mut env = vm_ref.get_env().expect("Cannot get JNIEnv");
    let class = env.find_class("com/splats/app/MainActivity").expect("Failed to find MainActivity");
    *MAIN_ACTIVITY_CLASS.write().unwrap() = Some(env.new_global_ref(class).unwrap());

    jni::sys::JNI_VERSION_1_6
}

use lazy_static::lazy_static;

lazy_static! {
    static ref PLATFORM_EVENT_SENDER: RwLock<Option<tokio::sync::mpsc::UnboundedSender<brush_ui::ui_process::PlatformEvent>>> =
        RwLock::new(None);
    static ref MAIN_ACTIVITY_CLASS: RwLock<Option<jni::objects::GlobalRef>> =
        RwLock::new(None);
}

#[unsafe(no_mangle)]
pub unsafe extern "system" fn Java_com_splats_app_MainActivity_notifyPlatformEvent<'local>(
    mut env: jni::JNIEnv<'local>,
    _class: jni::objects::JClass<'local>,
    event_type: jni::objects::JString<'local>,
    data: jni::objects::JString<'local>,
) {
    let event_type: String = env.get_string(&event_type).unwrap().into();
    let data: String = env.get_string(&data).unwrap().into();

    log::info!("Platform event: {} data: {}", event_type, data);

    let event = if event_type.ends_with("_picked") {
        let name = std::path::Path::new(&data)
            .file_name()
            .and_then(|n| n.to_str())
            .unwrap_or("file")
            .to_string();
        brush_ui::ui_process::PlatformEvent::FileSelected {
            event_type,
            path: data,
            name,
        }
    } else {
        brush_ui::ui_process::PlatformEvent::ProcessComplete {
            event_type,
            success: true,
            data,
        }
    };

    if let Some(sender) = PLATFORM_EVENT_SENDER.read().unwrap().as_ref() {
        let _: Result<(), _> = sender.send(event);
    }
}

#[cfg(target_os = "android")]
fn call_java_static(method: &str) {
    let vm = rrfd::android::get_jvm().expect("JVM not initialized");
    let mut env = vm.attach_current_thread().unwrap();

    let class_ref = MAIN_ACTIVITY_CLASS.read().unwrap();
    let class = class_ref.as_ref().expect("MainActivity class not cached");

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
                        let (sender, mut receiver) = tokio::sync::mpsc::unbounded_channel();
                        *PLATFORM_EVENT_SENDER.write().unwrap() = Some(sender);

                        let ctx_clone = ctx.clone();
                        tokio::spawn(async move {
                            while let Some(event) = receiver.recv().await {
                                ctx_clone.dispatch_platform_event(event);
                            }
                        });

                        ctx.register_platform_action("choose_mp4",
                            Box::new(|| call_java_static("chooseMp4")));
                        ctx.register_platform_action("extract_frames",
                            Box::new(|| call_java_static("extractFrames")));
                        ctx.register_platform_action("choose_csv",
                            Box::new(|| call_java_static("chooseCsv")));
                        ctx.register_platform_action("run_train",
                            Box::new(|| call_java_static("runTrain")));
                    }

                    Ok(Box::new(app))
                }),
            )
            .unwrap();
        });
}
