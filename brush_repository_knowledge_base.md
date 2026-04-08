# Brush-SFM Repository Knowledge Base

This knowledge base synthesizes the architecture, data flow, component breakdown, and technical state of the **Brush-SFM (Drone-Driven On-Device Gaussian Splatting)** repository, mapped directly from our codebase analysis.

## 1. High-Level Architecture Overview
The repository is a hybrid workspace bridging an Android application and a highly optimized native Rust computational core. 
- **Frontend / Rendering**: Rust + WGPU using `egui` for the cross-platform UI (`brush-ui`).
- **Telemetry Bridge**: Native Android Java (`TelemetrySparseReconstruction.java`) which prepares video and sensor telemetry mapping for the native pipeline.
- **Inter-Process Communication (IPC)**: A structured JNI bridge routing asynchronous string/JSON messages and callback hooks between Rust's tokio runtime and Android's JVM.
- **Unified Computational Engine**: A single, end-to-end Rust pipeline (`brush-process`) that orchestrates feature extraction, pose recovery, triangulation, and bundle adjustment using OpenCV, graduating to Gaussian Splatting training.

---

## 2. Component Breakdown

### A. The Core UI Layer (`crates/brush-ui`)
- **Location:** `crates/brush-ui/src/scene.rs`
- **Purpose:** Acts as the primary interaction canvas. It uses a consolidated **"One Train Process"** model.
- **Mechanics:** 
  - Instead of individual stage buttons, the UI focuses on a global **"Train"** action (`process.call_platform_action("run_train")`).
  - Implements a unified status listener that polls `tokio::mpsc` queues to provide real-time updates from the native training stages.

### B. The JNI Translation API (`crates/brush-app/src/android.rs`)
- **Purpose:** The glue binding the Rust frontend with the Android operating system.
- **Key Methods:**
  - `run_train`: Consolidated platform action that triggers the full Android telemetry and native SfM flow.
  - `Java_com_splats_app_MainActivity_notifyPlatformEvent`: Shared entry point for Android to signal completion or updates back to Rust.

### C. The Android Telemetry Layer (`com.splats.app`)
- **Location:** `crates/brush-app/app/src/main/java/com/splats/app/`
- **MainActivity.java**: Orchestrates library loading and handles the high-level response from the native pipeline.
- **TelemetrySparseReconstruction.java**: 
  - **The JNI Bridge:** No longer contains SfM logic. It is strictly responsible for preparing input payloads (JSON-serialized frames, GPS priors, and IMU deltas).
  - **Payload Generation:** Maps spatial priors into `gps.json` and `imu.json` formats required by the Rust core.
- **OpenCvFrontendLib.kt**: The modern Kotlin entry point for the **Unified SfM Pipeline**. It exposes `runFullPipelineSync`, which encapsulates the entire native lifecycle from extraction to export.

### D. The Native SfM Pipeline (`crates/brush-process`)
- **Location:** `crates/brush-process/src/sfm/mod.rs`
- **Purpose:** End-to-end sparse reconstruction using native OpenCV 4.13.0, eliminating fragmented Java-to-Rust JNI chatter.
- **Execution Workflow (Stages 3.1 - 3.8):**
  1. **3.1 - 3.2**: Feature Detection (ORB) and KNN Matching.
  2. **3.3 - 3.4**: Geometric Filtering (RANSAC) and Pose Recovery.
  3. **3.5 - 3.6**: Triangulation and Reprojection Inlier Filtering.
  4. **3.7**: Bundle Adjustment (via `brush-sfm`) for global consistency.
  5. **3.8**: **Pose Export**: Generates Nerfstudio-compatible `transforms.json` and `sparse.ply`.

---

## 3. Implementation Status & OpenCV Integration

**OpenCV Asset Integration (Completed):**
- **Dynamic Libraries**: Modular OpenCV 4.13.0 and TBB `.so` files are bundled in `jniLibs/arm64-v8a/`.
- **Optimization**: The core pipeline uses native OpenCV modules (`features2d`, `calib3d`) compiled specifically for Android `arm64-v8a` with NEON acceleration.

**The Unified SfM Pipeline (Completed):**
- **Stages 3.1 - 3.8**: The full pipeline from image extraction to final export is now implemented and verified.
- **Pose Export**: The system successfully exports a visualizable sparse mesh (`sparse.ply`) and the necessary transformation data for the Gaussian Splatting stage.

**UI Consolidation:**
- The previous grid of "Coming Soon" buttons has been replaced by a single, prominent **Train** button in both the Rust UI and Android Java frontend. 
- Error handling has been centralized; failures in the native pipeline now propagate back through JNI to the UI as explicit error states instead of hanging the polling loop.

---

## 4. Development Environment & Toolchain

- **Host OS**: Fedora Linux 43 (x86_64)
- **Target Architecture**: `arm64-v8a` (`aarch64-linux-android`)
- **OpenCV SDK**: Native Android SDK 4.13.0 (Modularized `.so` and headers)
- **NDK/Gradle Bypass Pattern**:
  1. **Host CLI Build**: `cargo ndk -t arm64-v8a -o crates/brush-app/app/src/main/jniLibs/ build --release`
  2. **Android Studio**: Gradle `:app:buildRustNativeBa` skips the Cargo build if the library exists, avoiding Flatpak environment conflicts.

- **Rust Build Profile (Release)**:
  - `opt-level = 3`, `lto = "thin"`, `codegen-units = 16`.
  - Enables SIMD (`target-feature=+neon`) and parallel linking for performance.

---

## 5. Known Integration Points
- **SfM to Training**: The output of Stage 3.8 (`transforms.json`) is the direct input for the `brush-train` Gaussian Splatting core.
- **Telemetry Reliability**: Relies on accurate Enu position mappings from the Drone CSVs to seed the global SfM coordinate system.
