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
    *MAIN_ACTIVITY_CLASS.write().unwrap() = Some(env.new_global_ref(&class).unwrap());

    // Cache method IDs
    let mut cache = JNI_METHOD_CACHE.write().unwrap();
    cache.choose_mp4 = env.get_static_method_id(&class, "chooseMp4", "()V").ok();
    cache.extract_frames = env.get_static_method_id(&class, "extractFrames", "(Ljava/lang/String;)V").ok();
    cache.choose_csv = env.get_static_method_id(&class, "chooseCsv", "()V").ok();
    cache.choose_config = env.get_static_method_id(&class, "chooseConfig", "()V").ok();
    cache.run_train = env.get_static_method_id(&class, "runTrain", "(Ljava/lang/String;)V").ok();
    cache.get_device_model = env.get_static_method_id(&class, "getDeviceModel", "()Ljava/lang/String;").ok();
    cache.pick_file = env.get_static_method_id(&class, "pickFile", "()V").ok();

    jni::sys::JNI_VERSION_1_6
}

macro_rules! jni_guard {
    ($env:expr, $block:block) => {
        std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            $block
        })).unwrap_or_else(|_| {
            let _ = $env.throw_new("java/lang/RuntimeException", "Native panic in Rust core (android.rs)");
            // Return a default value if necessary, or just rely on the JVM handling the exception
        })
    };
}

use lazy_static::lazy_static;

lazy_static! {
    static ref PLATFORM_EVENT_SENDER: RwLock<Option<tokio::sync::mpsc::UnboundedSender<brush_ui::ui_process::PlatformEvent>>> =
        RwLock::new(None);
    static ref MAIN_ACTIVITY_CLASS: RwLock<Option<jni::objects::GlobalRef>> =
        RwLock::new(None);
    static ref JNI_METHOD_CACHE: RwLock<JniMethodCache> = RwLock::new(JniMethodCache::default());
}

#[derive(Default)]
struct JniMethodCache {
    choose_mp4: Option<jni::objects::JStaticMethodID>,
    extract_frames: Option<jni::objects::JStaticMethodID>,
    choose_csv: Option<jni::objects::JStaticMethodID>,
    choose_config: Option<jni::objects::JStaticMethodID>,
    run_train: Option<jni::objects::JStaticMethodID>,
    get_device_model: Option<jni::objects::JStaticMethodID>,
    pick_file: Option<jni::objects::JStaticMethodID>,
}

#[unsafe(no_mangle)]
pub unsafe extern "system" fn Java_com_splats_app_MainActivity_notifyPlatformEvent<'local>(
    mut env: jni::JNIEnv<'local>,
    _class: jni::objects::JClass<'local>,
    event_type: jni::objects::JString<'local>,
    data: jni::objects::JString<'local>,
) {
    jni_guard!(env, {
        let event_type_jstr: jni::objects::JString = event_type.into();
        let data_jstr: jni::objects::JString = data.into();

        let event_type_raw = match env.get_string(&event_type_jstr) {
            Ok(s) => s,
            Err(_) => return, // Return silently as exception is likely pending
        };
        let data_raw = match env.get_string(&data_jstr) {
            Ok(s) => s,
            Err(_) => return,
        };

        let event_type: String = event_type_raw.into();
        let data: String = data_raw.into();

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
        } else if event_type.starts_with("progress:") {
            let name = event_type.strip_prefix("progress:").unwrap_or(&event_type).to_string();
            let val: f32 = data.parse().unwrap_or(0.0);
            brush_ui::ui_process::PlatformEvent::Progress {
                event_type: name,
                progress: val,
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
    })
}

fn with_attached_env<F, R>(f: F) -> Option<R>
where
    F: FnOnce(&mut jni::JNIEnv, &jni::objects::JClass) -> R,
{
    let vm = rrfd::android::get_jvm()?;
    // Get the env if already attached, or attach as daemon.
    // daemon prevents the JVM from hanging on shutdown waiting for our threads.
    let mut env = match vm.get_env() {
        Ok(env) => env,
        Err(e) => match vm.attach_current_thread_as_daemon() {
            Ok(env) => env,
            Err(attach_err) => {
                log::error!("JNI error: get_env failed ({:?}) and attach failed ({:?})", e, attach_err);
                return None;
            }
        },
    };

    let class_ref = MAIN_ACTIVITY_CLASS.read().unwrap();
    let Some(class_global) = class_ref.as_ref() else {
        log::error!("MainActivity class global ref not found");
        return None;
    };

    let class_obj = match env.new_local_ref(class_global) {
        Ok(c) => c,
        Err(err) => {
            log::error!("Failed to create local ref for MainActivity: {err:?}");
            return None;
        }
    };
    let class: &jni::objects::JClass = (&class_obj).into();

    Some(f(&mut env, class))
}

#[cfg(target_os = "android")]
fn call_java_static(method_name: &str) {
    with_attached_env(|env, class| {
        let method_id = {
            let cache = JNI_METHOD_CACHE.read().unwrap();
            match method_name {
                "chooseMp4" => cache.choose_mp4.as_ref(),
                "chooseCsv" => cache.choose_csv.as_ref(),
                "chooseConfig" => cache.choose_config.as_ref(),
                "pickFile" => cache.pick_file.as_ref(),
                _ => None,
            }.cloned()
        };

        let result = if let Some(mid) = method_id {
            unsafe {
                env.call_static_method_unchecked(
                    class,
                    mid,
                    jni::signature::ReturnType::Primitive(jni::signature::Primitive::Void),
                    &[],
                )
            }
        } else {
            env.call_static_method(class, method_name, "()V", &[])
        };

        if let Err(err) = result {
            log::error!("Failed to call MainActivity.{method_name}(): {err:?}");
            if env.exception_check().unwrap_or(false) {
                let _ = env.exception_describe();
                let _ = env.exception_clear();
            }
        } else {
            log::info!("Successfully called MainActivity.{method_name}()");
        }
    });
}

#[cfg(target_os = "android")]
fn call_java_static_string(method_name: &str, arg: &str) {
    with_attached_env(|env, class| {
        let jstr = match env.new_string(arg) {
            Ok(s) => s,
            Err(err) => {
                log::error!("Failed to allocate jstring for MainActivity.{method_name}: {err:?}");
                return;
            }
        };

        let method_id = {
            let cache = JNI_METHOD_CACHE.read().unwrap();
            match method_name {
                "extractFrames" => cache.extract_frames.as_ref(),
                "runTrain" => cache.run_train.as_ref(),
                _ => None,
            }.cloned()
        };

        let result = if let Some(mid) = method_id {
            let jval = jni::objects::JValue::Object(&jstr);
            unsafe {
                env.call_static_method_unchecked(
                    class,
                    mid,
                    jni::signature::ReturnType::Primitive(jni::signature::Primitive::Void),
                    &[jval.as_jni()],
                )
            }
        } else {
            env.call_static_method(
                class,
                method_name,
                "(Ljava/lang/String;)V",
                &[jni::objects::JValue::Object(&jstr)],
            )
        };

        if let Err(err) = result {
            log::error!("Failed to call MainActivity.{method_name}(String): {err:?}");
            if env.exception_check().unwrap_or(false) {
                let _ = env.exception_describe();
                let _ = env.exception_clear();
            }
        } else {
            log::info!("Successfully called MainActivity.{method_name}(String)");
        }
    });
}

#[cfg(target_os = "android")]
fn call_java_static_return_string(method_name: &str) -> Option<String> {
    with_attached_env(|env, class| {
        let method_id = {
            let cache = JNI_METHOD_CACHE.read().unwrap();
            match method_name {
                "getDeviceModel" => cache.get_device_model.as_ref(),
                _ => None,
            }.cloned()
        };

        let result = if let Some(mid) = method_id {
            unsafe {
                env.call_static_method_unchecked(
                    class,
                    mid,
                    jni::signature::ReturnType::Object,
                    &[],
                )
            }
        } else {
            env.call_static_method(class, method_name, "()Ljava/lang/String;", &[])
        };

        let result = match result {
            Ok(r) => r,
            Err(e) => {
                log::error!("JNI call failed for {method_name}: {e:?}");
                return None;
            }
        };

        let jobj = result.l().ok()?;
        if jobj.is_null() {
            return None;
        }

        let jstr: jni::objects::JString = jobj.into();
        let s: String = env.get_string(&jstr).ok()?.into();
        Some(s)
    })
    .flatten()
}

#[unsafe(no_mangle)]
fn android_main(app: winit::platform::android::activity::AndroidApp) {
    android_logger::init_once(
        android_logger::Config::default().with_max_level(log::LevelFilter::Info),
    );

    let device_model = call_java_static_return_string("getDeviceModel");
    log::info!("Device Identification: {:?}", device_model);

    let wgpu_options = brush_ui::create_egui_options_with_hints(device_model.as_deref());

    startup();

    let runtime = tokio::runtime::Builder::new_multi_thread()
        .enable_all()
        .build()
        .unwrap();

    runtime.block_on(async move {
        eframe::run_native(
            "Brush",
            eframe::NativeOptions {
                // Build app display.
                viewport: egui::ViewportBuilder::default(),
                android_app: Some(app),
                wgpu_options,
                ..Default::default()
            },
            Box::new(move |cc| {
                let app = App::new(cc, None);

                #[cfg(target_os = "android")]
                {
                    let ctx = app.context();
                    let (sender, mut receiver) = tokio::sync::mpsc::unbounded_channel();
                    *PLATFORM_EVENT_SENDER.write().unwrap() = Some(sender);

                    let ctx_clone = ctx.clone();
                    tokio::spawn(async move {
                        log::info!("[BRUSH_FLOW] Platform event handler task started.");
                        while let Some(event) = receiver.recv().await {
                            log::debug!("[BRUSH_FLOW] Received platform event: {:?}", event);
                            ctx_clone.dispatch_platform_event(event);
                        }
                    });

                    let proc_extract = ctx.clone();
                    ctx.register_platform_action("choose_mp4",
                        Arc::new(|| call_java_static("chooseMp4")));
                    ctx.register_platform_action("extract_frames", Arc::new(move || {
                        let json = proc_extract
                            .take_platform_action_payload()
                            .unwrap_or_else(|| "{}".to_string());
                        call_java_static_string("extractFrames", &json);
                    }));
                    ctx.register_platform_action("choose_csv",
                        Arc::new(|| call_java_static("chooseCsv")));
                    ctx.register_platform_action("choose_config",
                        Arc::new(|| call_java_static("chooseConfig")));
                    let proc_train = ctx.clone();
                    ctx.register_platform_action("run_train", Arc::new(move || {
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
