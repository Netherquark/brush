# Brush-SFM Repository Knowledge Base

This knowledge base synthesizes the architecture, data flow, component breakdown, and technical state of the **Brush-SFM (Drone-Driven On-Device Gaussian Splatting)** repository, mapped directly from our codebase analysis.

## 1. High-Level Architecture Overview
The repository is a hybrid workspace bridging an Android application and a highly optimized native Rust computational core. 
- **Frontend / Rendering**: Rust + WGPU using `egui` for the cross-platform UI (`brush-ui`).
- **Telemetry Bridge**: Native Android Java (`TelemetrySparseReconstruction.java`) which prepares video and sensor telemetry mapping for the native pipeline.
- **Inter-Process Communication (IPC)**: A structured JNI bridge routing asynchronous string/JSON messages and callback hooks between Rust's tokio runtime and Android's JVM.
- **Unified Computational Engine**: A single, end-to-end Rust pipeline (`brush-process`) that orchestrates feature extraction, pose recovery, triangulation, and bundle adjustment using OpenCV, graduating to Gaussian Splatting training.
- **Dynamic Configuration**: Hyperparameters for SfM and Bundle Adjustment are passed as JSON from the Android GUI to the Rust native layer, enabling real-time experimentation without recompilation.

---

## 2. Component Breakdown

### A. The Core UI Layer (`crates/brush-ui`)
- **Location:** `crates/brush-ui/src/scene.rs`
- **Purpose:** Acts as the primary interaction canvas.
- **Mechanics:** 
  - Implements a multi-step data selection workflow: **MP4**, **CSV**, and **Config**.
  - Provides dual-mode **Extraction** (Uniform vs. Telemetry-based) and a final **Train** button.
  - Implements a unified status listener that polls `tokio::mpsc` queues to provide real-time updates from the background processing stages.

### B. The JNI Translation API (`crates/brush-app/src/android.rs`)
- **Purpose:** The glue binding the Rust frontend with the Android operating system.
- **Key Methods:**
  - `run_train`: Consolidated platform action that triggers the full Android telemetry and native SfM flow.
  - `Java_com_splats_app_MainActivity_notifyPlatformEvent`: Shared entry point for Android to signal completion or updates back to Rust.

### C. The Android Telemetry Layer (`com.splats.app`)
- **Location:** `crates/brush-app/app/src/main/java/com/splats/app/`
- **MainActivity.java**: Orchestrates library loading and handles the high-level response from the native pipeline.
- **TelemetrySparseReconstruction.java**: 
  - **The Orchestrator:** Prepares the full input payload (frames, intrinsics, GPS, IMU deltas) and invokes the native SfM pipeline via `OpenCvFrontendLib`.
  - **Payload Generation:** Maps spatial priors into `gps.json` and `imu.json` formats required by the Rust core.
- **OpenCvFrontendLib.kt**: The JNI bridge for the **Unified SfM Pipeline**. It exposes `runFullPipelineSync`, which encapsulates the entire native lifecycle from extraction to export.

### D. The Native SfM Pipeline (`crates/brush-process`)
- **Location:** `crates/brush-process/src/sfm/mod.rs`
- **Purpose:** End-to-end sparse reconstruction using native OpenCV 4.13.0, eliminating fragmented Java-to-Rust JNI chatter.
- **Execution Workflow (Stages 3.1 - 3.8):**
  1. **3.1 - 3.2**: Feature Detection (ORB) and KNN Matching.
  2. **3.3 - 3.4**: Geometric Filtering (RANSAC) and Pose Recovery.
  3. **3.5 - 3.6**: Triangulation and Reprojection Inlier Filtering.
  4. **3.7**: Bundle Adjustment (via `brush-sfm`) for global consistency.
  5. **3.8**: **Pose Export**: Generates Nerfstudio-compatible `transforms.json` and `sparse.ply`.
- **Coordinate Space Normalization**: Stage 3.8 implements a critical conversion from OpenCV's right-handed coordinate system (x-right, y-down, z-forward) to the NeRF-standard system (x-right, y-up, z-back) to ensure compatibility with Gaussian Splatting kernels.

---

## 3. Implementation Status & OpenCV Integration

**OpenCV Asset Integration (Completed):**
- **Dynamic Libraries**: Modular OpenCV 4.13.0 and TBB `.so` files are bundled in `jniLibs/arm64-v8a/`.
- **Optimization**: The core pipeline uses native OpenCV modules (`features2d`, `calib3d`) compiled specifically for Android `arm64-v8a` with NEON acceleration.

**The Unified SfM Pipeline (Completed & Integrated):**
- **Stages 3.1 - 3.8**: The full pipeline from image extraction to final export is now implemented, integrated via JNI, and verified.
- **Dynamic Tuning**: The system now supports runtime overrides for matching (Hamming distance, max matches), RANSAC (threshold, probability), and depth filtering parameters.
- **Pose Export**: The system successfully exports a visualizable sparse mesh (`sparse.ply`) and optimized transformation data (`transforms.json`) with correct axis alignment for training.

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
- **SfM to Training**: The output of Stage 3.8 (`transforms.json`) is the direct input for the `brush-train` Gaussian Splatting core, featuring NeRF-compatible axis alignment.
- **Telemetry Reliability**: Relies on accurate Enu position mappings from the Drone CSVs to seed the global SfM coordinate system.
- **JSON Configuration Interface**: The JNI bridge implements a tiered parsing logic that handles both flat configuration payloads (for the mobile UI) and nested structures for advanced pipeline control.
