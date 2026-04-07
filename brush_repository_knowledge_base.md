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

## 3. Known Implementation Gaps & Stubs
During analysis, major gaps in the pipeline were identified regarding the integration of machine-learning native SfM implementations.

**The "Missing" Rust SfM Pipeline:**
In `crates/brush-process/src/sfm/mod.rs`, the core C++/Rust pipeline is intentionally gated as `Placeholder stubs`. 
The following stages **do not exist** in the Rust codebase and are completely unimplemented:
- **Stage 3.1:** Feature Extraction 
- **Stage 3.2:** Matching 
- **Stage 3.3:** RANSAC 
- **Stage 3.4:** Pose Recovery
- **Stage 3.5:** Triangulation 
- **Stage 3.6:** Inlier Filtering
- **Stage 3.8:** Pose Export (handled loosely by Java equivalents presently).

Because these modules are missing, the repository leans entirely on the rudimentary Java `TelemetrySparseReconstruction` layer to fill the gap.

**UI Redundancies & Dead Code:**
The application has UI buttons explicitly referencing the missing steps (e.g., `Extract`, `Pose Est.`, `Bundle`, `Save PLY`, `Viewer`). In `MainActivity.java`, these actions map solely to literal String stubs returning a `Toast.makeText(..., "coming soon")`. The execution of the active backend happens exclusively via the singular `Telemetry` execution flow.

**Bugs Noticed:**
Errors inside the Java telemetry loops fail to propagate standard return signals back to the Native Rust token registry, leaving `tokio` listeners perpetually polling a `"Running..."` state if `TelemetryPreprocessorCallback#onComplete` detects a failure. 

---

## 4. Overall Codebase Context Map
- **UI Management**: `brush-ui`
- **Rendering / WGPU**: `brush-render`, `brush-wgsl`
- **Native Bridges**: `brush-app/android.rs`, `BundleAdjustmentLib.kt`
- **Data File Parsing**: `colmap-reader`, `brush-vfs`, `rrfd` (File dialogs).
- **Core Pipeline Computation**: `brush-process`, `brush-sfm`.
