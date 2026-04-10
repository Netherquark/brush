# Configuration Refactoring Plan

The objective is to expose hardcoded calibration, hyperparameters, and config settings currently deeply embedded in the Rust Core and Android Telemetry Pipeline to the Android app GUI.

Based on an exhaustive codebase sweep spanning `crates/brush-train`, `crates/brush-process`, `crates/brush-dataset`, and Android JNI paths (`com.splats.app`), we've isolated 6 major areas where parameters are hardcoded and need shifting.

## User Review Required

> [!IMPORTANT]
> The Android JNI protocol (`run_train` or specific initialization methods) will need to be updated to accept these new parameters. This involves changes to BOTH the Rust FFI boundary and the Android JNI payload layer.

## Proposed Changes

---

### Android Telemetry Layer (Kotlin)

These defaults dictate which sensor/video frames get selected for the pipeline.
Currently defined as defaults in `KeyframeSelector.kt`.

#### [MODIFY] KeyframeSelector.kt
Update `KeyframeSelectionConfig` to be instantiable from GUI Preferences instead of relying solely on default values.
*   `distanceThresholdM` (Current: 2.0 m)
*   `yawThresholdDeg` (Current: 8.0°)
*   `pitchThresholdDeg` (Current: 5.0°)
*   `timeThresholdUs` (Current: 1s)
*   `minSpeedMs` (Current: 0.2 m/s)

---

### Rust Core: Gaussian Splatting Training

The Gaussian Splatting hyper-parameters reside in `TrainConfig`. Currently, these depend on CLI arguments or default structs which can't be modified by Android.

#### [MODIFY] config.rs (brush-train)
Adjust `TrainConfig` exposed defaults so they easily parse from a JSON struct sent by the Android JNI bridge instead of relying on `clap` CLI arguments.
*   `total_steps` (Current: 30,000)
*   `lr_mean` (Current: 2e-5)
*   `lr_coeffs_dc` (Current: 2e-3)
*   `lr_opac` (Current: 0.012)
*   `lr_scale` (Current: 7e-3)
*   `mean_noise_weight` (Current: 50.0)
*   `max_splats` (Current: 10,000,000)
*   `refine_every` (Current: 200)
*   `growth_grad_threshold` (Current: 0.003)

---

### Rust Core: Structure from Motion (SfM)

SfM hyper-parameters heavily impact trajectory scaling and reprojection filtering.

#### [MODIFY] stage_3_3_ransac.rs (brush-process)
Update `RansacConfig::default()` usage to fetch from JNI settings context.
*   `probability` (Current: 0.999)
*   `threshold_px` (Current: 1.0 px)
*   `max_iters` (Current: 10,000)

#### [MODIFY] stage_3_6_inlier_filtering.rs (brush-process)
Update `InlierFilterConfig::default()` usage to limit max depth of outliers.
*   `min_depth` (Current: 0.01)
*   `max_depth` (Current: 10_000.0)

---

### Rust Core: Process & Dataset Configuration

#### [MODIFY] config.rs (brush-dataset)
*   `sh_degree` (Current: 3)
*   `max_resolution` (Current: 1920)

#### [MODIFY] config.rs (brush-process)
*   `seed` (Current: 42)

---

### JNI Interop Layer (Rust <-> Kotlin)

To shuttle the unified GUI configurations across the native layer securely.

#### [MODIFY] android.rs (brush-app)
Add parsing logic to decode an overarching JSON config string passed from Android's `runFullPipelineSync` and apply it to the respective Rust configs (`TrainConfig`, `RansacConfig`, etc.) before initiating the `brush-process` stages.

#### [MODIFY] OpenCvFrontendLib.kt (or relevant entry point in Android)
Update `runFullPipelineSync` (and the JNI signature) to accept a single JSON string containing all user-specified hyperparameter configurations, allowing the app GUI to easily construct and pass a single payload.

## Verification Plan

### Automated Tests
*   Compile Rust core using `cargo ndk -t arm64-v8a build` to ensure JNI payload struct updates safely compile across the FFI boundary.
*   Validate the JSON deserialization logic in Rust by writing a small unit test for the single-JSON payload.

### Manual Verification
*   Android app build and deploy. Test the GUI sliders, confirming the constructed JSON payload reaches the native side by inspecting Logcat outputs and verifying the settings take effect in the pipeline.
