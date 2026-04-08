use std::env;
use std::path::PathBuf;

fn main() {
    let target_os = env::var("CARGO_CFG_TARGET_OS").unwrap_or_default();

    // We only need custom linking logic for Android because opencv pkg-config doesn't work well there
    if target_os == "android" {
        let manifest_dir = env::var("CARGO_MANIFEST_DIR").unwrap();
        let manifest_path = PathBuf::from(manifest_dir);
        let workspace_root = manifest_path.parent().unwrap().parent().unwrap();

        let abi = env::var("ANDROID_ABI").unwrap_or_else(|_| "arm64-v8a".to_string());
        let default_opencv_lib_dir = workspace_root
            .join("crates")
            .join("brush-app")
            .join("app")
            .join("src")
            .join("main")
            .join("jniLibs")
            .join(&abi);

        println!("cargo:rerun-if-env-changed=ANDROID_ABI");
        println!("cargo:rerun-if-env-changed=OPENCV_LINK_PATHS");
        println!("cargo:rerun-if-env-changed=OPENCV_LINK_LIBS");
        println!("cargo:rustc-env=ON_ANDROID=1");

        let link_paths = env::var("OPENCV_LINK_PATHS")
            .ok()
            .map(|paths| split_path_list(&paths))
            .filter(|paths| !paths.is_empty())
            .unwrap_or_else(|| vec![default_opencv_lib_dir]);

        for path in link_paths {
            println!("cargo:rustc-link-search=native={}", path.display());
        }

        let link_libs = env::var("OPENCV_LINK_LIBS")
            .ok()
            .map(|libs| split_lib_list(&libs))
            .filter(|libs| !libs.is_empty())
            .unwrap_or_else(default_opencv_libs);

        for lib in link_libs {
            println!("cargo:rustc-link-lib=dylib={lib}");
        }
    }
}

fn split_path_list(value: &str) -> Vec<PathBuf> {
    env::split_paths(value).collect()
}

fn split_lib_list(value: &str) -> Vec<String> {
    value
        .split([',', ';', ' '])
        .map(str::trim)
        .filter(|entry| !entry.is_empty())
        .map(ToOwned::to_owned)
        .collect()
}

fn default_opencv_libs() -> Vec<String> {
    vec![
        "opencv_core".to_string(),
        "opencv_imgproc".to_string(),
        "opencv_imgcodecs".to_string(),
        "opencv_features2d".to_string(),
        "opencv_flann".to_string(),
        "opencv_calib3d".to_string(),
    ]
}
