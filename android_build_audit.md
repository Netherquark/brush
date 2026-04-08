# Android Build Audit

This audit summarizes the hardcoded paths, fixed assumptions, and build-time constants currently present in the repository after the `.env` migration work.

## Generalized in this change

- `crates/brush-app/app/build.gradle`
  - `.env` is now read from the workspace root.
  - `ANDROID_SDK_ROOT` / `ANDROID_HOME` can come from `.env`.
  - `ANDROID_NDK_HOME` can come from `.env`.
  - `CARGO_PATH` can come from `.env`.
  - `ANDROID_ABI`, `ANDROID_RUST_TARGET`, `ANDROID_MIN_SDK`, and `ANDROID_NDK_VERSION` can come from `.env`.
  - `LIBCLANG_PATH`, `CLANG_PATH`, `LD_LIBRARY_PATH`, `OPENCV_LINK_PATHS`, and `OPENCV_LINK_LIBS` are forwarded into the Rust build.
  - Cargo/CC/CXX/AR target environment variables are now derived from `ANDROID_RUST_TARGET` instead of being pinned to `aarch64-linux-android`.
- `crates/brush-process/build.rs`
  - OpenCV linker search paths are now configurable through `OPENCV_LINK_PATHS`.
  - OpenCV linked libraries are now configurable through `OPENCV_LINK_LIBS`.
  - The default Android OpenCV search path now depends on `ANDROID_ABI` instead of a fixed `arm64-v8a` string.

## Remaining fixed or semi-fixed assumptions

- `crates/brush-app/app/build.gradle`
  - `compileSdk 34`, `targetSdk 34`, and `minSdk 33` are still fixed Android app targets.
  - `ndkVersion "29.0.14206865"` is still declared in the Android block, even though the Rust build helper now allows `ANDROID_NDK_VERSION` overrides for its own checks.
  - The native output file is still expected to be `libbrush_process.so`.
- `Cargo.toml`
  - `[profile.release]` still fixes:
    - `opt-level = 3`
    - `lto = "thin"`
    - `codegen-units = 16`
    - `panic = "abort"`
    - `incremental = true`
- `.cargo/config.toml`
  - Android rustflags still fix:
    - `target-feature=+neon`
    - `opt-level=3`
    - `-Wl,--allow-shlib-undefined`
    - `-lc++_shared`
- `crates/brush-process/Cargo.toml`
  - OpenCV crate features are fixed to:
    - `imgcodecs`
    - `features2d`
    - `imgproc`
    - `flann`
    - `calib3d`
- `crates/brush-process/build.rs`
  - The fallback OpenCV library names are still fixed:
    - `opencv_core`
    - `opencv_imgproc`
    - `opencv_imgcodecs`
    - `opencv_features2d`
    - `opencv_flann`
    - `opencv_calib3d`

## Git context reviewed

- `bf92c20` `migrate to .env.example`
- `c7a39fd` `optimise build config (one-third time)`
- `e62c8b2` `load opencv lib`
- `2733ef6` `implement stage 3.1`

## Follow-up recommendations

- Unify `android.ndkVersion` with the overridable `ANDROID_NDK_VERSION` path so Android Studio and the Rust helper use the same single source of truth.
- Decide whether release-profile knobs should remain repo-wide defaults or move behind documented CI/local presets.
- Wire the new Stage 3.2–3.6 Rust frontend into the Android telemetry flow once the JNI/API contract is finalized.
