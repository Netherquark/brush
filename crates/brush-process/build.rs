use std::env;
use std::path::PathBuf;

fn main() {
    let target_os = env::var("CARGO_CFG_TARGET_OS").unwrap_or_default();
    
    // We only need custom linking logic for Android because opencv pkg-config doesn't work well there
    if target_os == "android" {
        let manifest_dir = env::var("CARGO_MANIFEST_DIR").unwrap();
        let manifest_path = PathBuf::from(manifest_dir);
        let workspace_root = manifest_path.parent().unwrap().parent().unwrap();
        
        let opencv_lib_dir = workspace_root
            .join("crates")
            .join("brush-app")
            .join("app")
            .join("src")
            .join("main")
            .join("jniLibs")
            .join("arm64-v8a");
            
        // Tell cargo to tell rustc to link the custom directory where the opencv libs are
        println!("cargo:rustc-link-search=native={}", opencv_lib_dir.display());
        
        // Ensure cargo links to the right libraries
        println!("cargo:rustc-link-lib=dylib=opencv_core");
        println!("cargo:rustc-link-lib=dylib=opencv_imgproc");
        println!("cargo:rustc-link-lib=dylib=opencv_imgcodecs");
        println!("cargo:rustc-link-lib=dylib=opencv_features2d");
        println!("cargo:rustc-link-lib=dylib=opencv_flann");
        println!("cargo:rustc-link-lib=dylib=opencv_calib3d");
    }
}
