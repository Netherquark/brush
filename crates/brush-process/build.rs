// ============================================================
// FILE PATH: crates/brush-process/build.rs
//
// PROJECT: brush-app — Drone-Driven On-Device Gaussian Splatting
//
// PURPOSE:
//   Tells the Rust linker where to find the prebuilt minimal OpenCV .so.
//   This script is invoked BEFORE cargo compiles the crate.
//   The environment variables it reads are set by the Gradle task
//   `buildRustNativeBa` in crates/brush-app/app/build.gradle.
//
// NOTE:
//   Stage 3.7 (BA module) does NOT link against OpenCV at all.
//   This build.rs only matters for Stages 3.1–3.6.
//   If you are only building/testing Stage 3.7, you can set a dummy
//   OPENCV_LINK_LIBS_DIR or comment out the opencv dep in Cargo.toml.
//
// PLACE THIS FILE AT:
//   <workspace_root>/crates/brush-process/build.rs
// ============================================================

fn main() {
    // Read env vars set by Gradle before invoking cargo.
    // These are NOT set when running `cargo test` on desktop —
    // in that case, we skip OpenCV linkage (BA tests don't need it).
    if let Ok(opencv_lib_dir) = std::env::var("OPENCV_LINK_LIBS_DIR") {
        println!("cargo:rustc-link-search=native={opencv_lib_dir}");
        // Link against the minimal OpenCV AAR .so (core + imgproc + features2d + calib3d only)
        println!("cargo:rustc-link-lib=dylib=opencv_java4");
    }

    // Re-run this script if these env vars change (cache invalidation)
    println!("cargo:rerun-if-env-changed=OPENCV_LINK_LIBS_DIR");
    println!("cargo:rerun-if-env-changed=OPENCV_INCLUDE_PATHS");

    // nalgebra and serde are pure Rust — no special linkage directives needed.
    // jni symbols are resolved at runtime by the Android dynamic linker.
}
