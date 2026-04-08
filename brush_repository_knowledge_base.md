# Brush-SFM Repository Knowledge Base

This knowledge base synthesizes the architecture, data flow, component breakdown, and technical state of the **Brush-SFM (Drone-Driven On-Device Gaussian Splatting)** repository, mapped directly from our codebase analysis.

## 1. High-Level Architecture Overview
The repository is a hybrid workspace bridging an Android application and a highly optimized native Rust computational core. 
- **Frontend / Rendering**: Rust + WGPU using `egui` for the cross-platform UI (`brush-ui`).
- **Telemetry Preprocessor**: Native Android Java (`TelemetrySparseReconstruction.java`) which serves as a pre-processor for video and sensor telemetry mapping.
- **Inter-Process Communication (IPC)**: A structured JNI bridge routing asynchronous string/JSON messages and callback hooks between Rust's tokio runtime and Android's JVM.
- **Computational Math Engine**: Pure Rust optimization logic mapping sparse features into a globally consistent state (`brush-sfm`).

---

## 2. Component Breakdown

### A. The Core UI Layer (`crates/brush-ui`)
- **Location:** `crates/brush-ui/src/scene.rs`
- **Purpose:** Acts as the primary interaction canvas for the user. It is built natively in Rust using `egui`.
- **Mechanics:** 
  - Instead of invoking OS-specific code directly, it emits abstract platform actions via an internal UiProcess state (e.g., `process.call_platform_action("telemetry")`).
  - Features asynchronous polling inside its `on_update` loop to retrieve status events (like `"telemetry_complete"`) via `tokio::mpsc` queues without stalling the render thread.

### B. The JNI Translation API (`crates/brush-app/src/android.rs`)
- **Purpose:** The glue binding the Rust frontend with the Android operating system.
- **Rust -> Java:** Contains static maps binding UI event strings (`"choose_csv"`, `"telemetry"`) to `call_java_static` functions. It uses `rrfd::android::get_jvm()` to access the `MainActivity` JVM class and trigger methods.
- **Java -> Rust:** Exposes the `Java_com_splats_app_MainActivity_notifyPlatformEvent` function, allowing Android tasks to signal the tokenized strings back to the Rust Toki runtime (e.g. providing a path to a generated PLY mesh).

### C. The Android Telemetry Layer (`com.splats.app`)
- **Location:** `crates/brush-app/app/src/main/java/com/splats/app/`
- **MainActivity.java**: Manages intent requests for file-picking (MP4/CSV) and triggers the heavy `TelemetryPreprocessor` on single-thread executors.
- **TelemetrySparseReconstruction.java**: 
  - **The "Fallback SfM":** Currently performs manual feature-patch extraction, image border matching, and initial epipolar triangulation natively in Java. It heavily relies on the Drone's spatial priors (GPS/IMU coordinates) ingested via parsed CSVs.
  - **Payload Generation:** Accumulates massive structural arrays mapping `observations`, `poses`, and `points` and serializes them into large JSON strings.
- **BundleAdjustmentLib.kt**: The Kotlin wrapper abstracting native JNI optimization methods. It hands the compiled JSON payload down to `brush_process`.

### D. The Numerical Optimization Engine (`crates/brush-sfm` & `crates/brush-process`)
- **Location:** `crates/brush-sfm/src/sfm/stage_3_7_bundle_adjustment.rs`
- **Purpose:** Pure mathematical optimization for 3D trajectory alignment, eliminating the need for heavyweight C++ OpenCV binaries in the final packaging.
- **Execution:**
  1. Accepts JSON structs across the JNI bridge and standardizes them using `serde_json`.
  2. Subdivides camera tracks utilizing a "Sliding Window" matrix.
  3. Uses **Levenberg-Marquardt** via `run_lm_core(..)` to minimize the distance between 2D feature observations and triangulated 3D points.
  4. Mathematically computes Schur Complements using `nalgebra` structures to extract camera rotations via Axis-Angle log mapping.
  5. Returns structural refinements as a unified `BaResult` JSON back to the Android JVM stack via the JNI.

---

## 3. Implementation Status & OpenCV Integration

During analysis, major gaps in the pipeline were identified regarding the integration of native SfM implementations. We are actively replacing the old Java-based fallback SfM with a high-performance native pipeline using OpenCV 4.13.0. 

**OpenCV Asset Integration (Completed):**
- **Custom Build Environment**: OpenCV 4.13.0 must be built out-of-tree using a specialized `cmake` command invoking the `android-30` platform and targeting `arm64-v8a`. The build intentionally strips Java/JS wrappers, UI projects, ML modules, and examples (`-DBUILD_opencv_java=OFF`, `-DBUILD_ANDROID_PROJECTS=OFF`, etc.) to significantly reduce the `.so` artifact size, ensuring Android Studio can bundle it without bloat.
- **Dynamic Libraries**: Prebuilt OpenCV 4.13.0 and TBB `.so` files are located in `crates/brush-app/app/src/main/jniLibs/arm64-v8a/`. 
- **JNI Loading**: Due to Android's dynamic linker constraints, these libraries must be explicitly loaded in `MainActivity.java` via `System.loadLibrary` (order: `tbb` -> `opencv_core` -> `opencv_imgproc` -> ... -> `brush_app`).
- **Headers**: C++ headers retrieved during the `cmake` phase are placed in `third_party/opencv/include/`.

**Build System & Sandbox Workarounds (Optimized):**
To address the 30-minute compilation bottleneck and Flatpak sandbox restrictions:
- **Parallel Optimization**: Switched to `lto = "thin"` and `codegen-units = 16` in the release profile.
- **Saturated Linking**: Configured `-Wl,--threads=12` in `.cargo/config.toml`.
- **Flatpak/Gradle Bypass**: The `:app:buildRustNativeBa` task in `build.gradle` is configured to **skip** the Cargo build if the native library already exists. This avoids `libclang.so` linkage errors caused by Flatpak's restricted access to host `/usr/lib64`.
- **NDK Check**: The build uses the **NDK-internal LLVM toolchain** (`LIBCLANG_PATH` and `CLANG_PATH`) for bindgen to maintain environment consistency.

**The Rust SfM Pipeline (In Progress):**
In `crates/brush-process/src/sfm/mod.rs`, the core pipeline is being implemented:
- **Stage 3.1: Feature Extraction (Completed)**: Using native OpenCV ORB with grayscale conversion and `features2d` module.
- **Stage 3.2: Matching (Completed)**: Using `knnMatch` with Lowe's ratio test and Hamming distance thresholding.
- **Stage 3.3: RANSAC (Completed)**: Filtering matches via Epipolar Geometry (`USAC_MAGSAC`).
- **Stage 3.4: Pose Recovery (Completed)**: Recovering `R` and `t` matrices.
- **Stage 3.5: Triangulation (Completed)**: Transforming 2D points into triangulated 3D spatial points.
- **Stage 3.6: Inlier Filtering (Completed)**: Keeping only triangulated points with positive Z coordinates and minimal reprojection error.


**UI Redundancies & Dead Code:**
The application has UI buttons explicitly referencing the missing steps (e.g., `Extract`, `Pose Est.`, `Bundle`, `Save PLY`, `Viewer`). In `MainActivity.java`, these actions map solely to literal String stubs returning a `Toast.makeText(..., "coming soon")`. The execution of the active backend happens exclusively via the singular `Telemetry` execution flow.

**Bugs Noticed:**
Errors inside the Java telemetry loops fail to propagate standard return signals back to the Native Rust token registry, leaving `tokio` listeners perpetually polling a `"Running..."` state if `TelemetryPreprocessorCallback#onComplete` detects a failure. 

---

---

## 5. Development Environment & Toolchain

This project uses a highly specific cross-compilation environment to bridge the Fedora host with the Android Pixel 9a target.

- **Host OS**: Fedora Linux 43 (x86_64)
- **Android NDK**: 29.0.14206865 (Verified)
- **Target Architecture**: `arm64-v8a` (`aarch64-linux-android`)
- **OpenCV SDK**: Native Android SDK 4.13.0 (Modularized `.so` and headers)
- **IDE**: Android Studio (Flatpak version)
  - **Flatpak Workaround**: Uses NDK-internal LLVM and Gradle "Skip if exists" logic to bypass sandbox restrictions.
- **Optimized Workflow**:
  1. **Host CLI**: `cargo ndk -t arm64-v8a -o crates/brush-app/app/src/main/jniLibs/ build --release` (Fastest, full system access).
  2. **IDE**: Hit **Run** in Android Studio (Gradle skips native build and packages pre-compiled `.so`).
- **Rust Build Profile (Release)**:
  - `opt-level = 3`
  - `lto = "thin"`
  - `codegen-units = 16`
  - `panic = "abort"`
  - `incremental = true`
- **Compiler Flags**:
  - `target-feature=+neon`: Enabled for ARM SIMD performance.
  - `-Wl,--threads=12`: Parallel linking enabled for AMD Ryzen 5650U.
  - `--allow-shlib-undefined`: Required for OpenCV system symbol resolution.

