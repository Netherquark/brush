 # Config Refactor Task List

- `[x]` 1. Android Telemetry Layer (Kotlin)
  - `[x]` Update `KeyframeSelectionConfig` to be instantiable from JSON parsing or parameters.
  - `[x]` Update `OpenCvFrontendLib.kt` JNI signature (`runFullPipelineSync`) to accept a global JSON string.
- `[x]` 2. Rust Core: Training & Dataset configs
  - `[x]` Update `TrainConfig` and `ModelConfig` and `ProcessConfig` struct derivations for optimal JSON deserialization if not fully covered.
- `[x]` 3. Rust Core: Structure from Motion (SfM)
  - `[x]` Expose `RansacConfig` and `InlierFilterConfig` so they can be injected rather than strictly initialized via `.default()`.
- `[x]` 4. JNI Interop Layer (`crates/brush-app/src/android.rs`)
  - `[x]` Support passing GUI strings to android process.
  - `[x]` Apply the parsed values down to the process/training/SfM pipelines.
- `[x]` 5. Verification
  - `[x]` Write small unit test or test parse logic for JSON payload.
  - `[x]` Visual verification of logic flow from Android selection to Rust application.
- `[x]` 6. Documentation
  - `[x]` Update README or docs with the new JSON configuration schema.
