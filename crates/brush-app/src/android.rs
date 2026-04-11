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
    let vm = match rrfd::android::get_jvm() {
        Some(vm) => vm,
        None => {
            log::error!("JVM not initialized when calling MainActivity.{method}()");
            return;
        }
    };
    let mut env = match vm.attach_current_thread() {
        Ok(env) => env,
        Err(err) => {
            log::error!("Failed to attach JNI thread for MainActivity.{method}(): {err:?}");
            return;
        }
    };

    let class_ref = MAIN_ACTIVITY_CLASS.read().unwrap();
    let Some(class) = class_ref.as_ref() else {
        log::error!("MainActivity class not cached when calling {method}()");
        return;
    };

    let class_obj = env.new_local_ref(class).unwrap();
    let class: &jni::objects::JClass = (&class_obj).into();

    if let Err(err) = env.call_static_method(class, method, "()V", &[]) {
        log::error!("Failed to call MainActivity.{method}(): {err:?}");
        if env.exception_check().unwrap_or(false) {
            let _ = env.exception_describe();
            let _ = env.exception_clear();
        }
    }
}

#[cfg(target_os = "android")]
fn call_java_static_string(method: &str, arg: &str) {
    let vm = match rrfd::android::get_jvm() {
        Some(vm) => vm,
        None => {
            log::error!("JVM not initialized when calling MainActivity.{method}(String)");
            return;
        }
    };
    let mut env = match vm.attach_current_thread() {
        Ok(env) => env,
        Err(err) => {
            log::error!("Failed to attach JNI thread for MainActivity.{method}(String): {err:?}");
            return;
        }
    };

    let class_ref = MAIN_ACTIVITY_CLASS.read().unwrap();
    let Some(class) = class_ref.as_ref() else {
        log::error!("MainActivity class not cached when calling {method}(String)");
        return;
    };

    let jstr = match env.new_string(arg) {
        Ok(s) => s,
        Err(err) => {
            log::error!("Failed to allocate jstring for MainActivity.{method}: {err:?}");
            return;
        }
    };

    let class_obj = match env.new_local_ref(class) {
        Ok(c) => c,
        Err(err) => {
            log::error!("Failed to local-ref MainActivity for {method}: {err:?}");
            return;
        }
    };
    let class: &jni::objects::JClass = (&class_obj).into();

    if let Err(err) = env.call_static_method(
        class,
        method,
        "(Ljava/lang/String;)V",
        &[jni::objects::JValue::Object(&jstr)],
    ) {
        log::error!("Failed to call MainActivity.{method}(String): {err:?}");
        if env.exception_check().unwrap_or(false) {
            let _ = env.exception_describe();
            let _ = env.exception_clear();
        }
    }
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

                        let proc_extract = ctx.clone();
                        ctx.register_platform_action("choose_mp4",
                            Box::new(|| call_java_static("chooseMp4")));
                        ctx.register_platform_action("extract_frames", Box::new(move || {
                            let json = proc_extract
                                .take_platform_action_payload()
                                .unwrap_or_else(|| "{}".to_string());
                            call_java_static_string("extractFrames", &json);
                        }));
                        ctx.register_platform_action("choose_csv",
                            Box::new(|| call_java_static("chooseCsv")));
                        ctx.register_platform_action("choose_config",
                            Box::new(|| call_java_static("chooseConfig")));
                        let proc_train = ctx.clone();
                        ctx.register_platform_action("run_train", Box::new(move || {
                            let json = proc_train
                                .take_platform_action_payload()
                                .unwrap_or_else(|| "{}".to_string());
                            call_java_static_string("runTrain", &json);
                        }));
                    }

                    Ok(Box::new(app))
                }),
            )
            .unwrap();
        });
}
