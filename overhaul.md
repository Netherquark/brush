# Post-Fork Overhaul Audit

**Scope**: Audit all changes after stable commit `0e11a521936b7476719be50ed82a38ff63e6b6bd` (fork point).

**Reference**: Complementary to `impl_from_scratch.md` - documents deviations, not original design.

**Platform priority**: Android-first for usage. Desktop used only for compilation/testing.

**Goal**: Course correction toward production-grade proof of concept. Retain valuable engineering contributions. Remove unnecessary complexity while preserving performance improvements.

---

## Executive Summary

Post-fork implementation achieved functional Android-first drone-to-splat pipeline. Core contribution: generating Gaussian splat from CSV+MP4 using telemetry-informed keyframe extraction, providing interactive preview in Brush.

Validated architectural decisions:
- Android-first usage (desktop for compilation only)
- Litchi CSV only (DJI SRT removed by design)
- Hardware detection (Qualcomm 128MB limitation requires adaptation)
- JSON config (necessary for experimental setup, same config across tests)
- Native libraries provided as binary (no stale .so concern)

Areas requiring correction:
- Architecture boundary violations (SfM location, OpenCV dependency)
- Incorrect mathematical implementations (pose chaining, residuals, yaw wraparound)
- Duplicated coordinate conversions
- Missing telemetry-window matching
- Resource budget defaults misaligned
- Some telemetry complexity may be excessive (evaluate case-by-case)

Focus: Convert to solid reliable production-grade proof of concept. Accept specific case-by-case hacks for MVP research paper results.

---

## 1. Architecture Drift

### 1.1 Crate Boundary Violation

**Spec Requirement**:
- Novel SfM/BA work belongs in `brush-sfm` or separate `brush-ba`
- `brush-process` remains training/loading orchestration only
- Android-first for usage; desktop for compilation/testing

**Current Implementation**:
- `brush-process` now contains `pub mod sfm` with full SfM pipeline
- `brush-process/Cargo.toml` adds OpenCV dependency
- Desktop builds may require Android/OpenCV dependencies due to unconditional SfM compilation
- `lib.rs` both declares local `pub mod sfm` and re-exports `brush-sfm` crate, creating namespace collision

**Evidence**:
```diff
+ crates/brush-process/src/sfm/mod.rs                |  511 ++
+ crates/brush-process/Cargo.toml                    |   11 +
```

**Impact**: Desktop compilation may require Android-specific dependencies. Architecture boundary unclear.

**Correction**: Move SfM logic to `brush-sfm` exclusively. Remove OpenCV from `brush-process` if possible. Keep `brush-process` as orchestration-only shell. Consider conditional compilation for Android-specific features.

---

### 1.2 Duplicate SfM Location

**Spec Requirement**: Single location for SfM/BA - either `brush-sfm` or `brush-ba`

**Current Implementation**:
- `crates/brush-process/src/sfm/` - full SfM implementation
- `crates/brush-sfm/src/sfm/` - additional SfM crate with BA
- Both `brush-process` and `brush-sfm` expose SfM modules

**Evidence**:
```diff
+ crates/brush-sfm/Cargo.toml                        |   25 +
+ crates/brush-sfm/src/lib.rs                        |   24 +
+ crates/brush-sfm/src/sfm/mod.rs                    |    3 +
+ crates/brush-sfm/src/sfm/stage_3_7_bundle_adjustment.rs | 1335 +++++
```

**Impact**: Namespace confusion, duplicate code paths, unclear ownership.

**Correction**: Choose single location (`brush-sfm` per spec preference). Remove duplicate pathway. Update all imports to single source.

---

### 1.3 JNI/Native Library Loading

**Spec Requirement**: Single JNI boundary, native library loading names match exports

**Current Implementation**:
- `OpenCvFrontendLib.kt` loads `brush_process`
- `BundleAdjustmentLib.kt` loads `brush_process`
- `brush-process` crate compiled as cdylib (produces libbrush_process.so)
- `brush-app` crate compiled as cdylib (produces libbrush_app.so)
- Single JNI_OnLoad in android.rs (brush-app)

**Evidence**:
```kotlin
// Both Kotlin objects load same library correctly
System.loadLibrary("brush_process")
```

```toml
# brush-process/Cargo.toml
[lib]
crate-type = ["cdylib", "rlib"]

# brush-app/Cargo.toml  
[lib]
name = "brush_app"
crate-type = ["cdylib", "rlib"]
```

**Impact**: JNI library loading is actually consistent. No mismatch found.

**Status**: Verified - JNI library loading names match exports correctly. No duplicate JNI issue.

---

## 2. Telemetry Pipeline Deviations

### 2.1 DJI SRT Support

**Spec Change**: DJI SRT support removed by design. Litchi CSV only.

**Current Implementation**: Only Litchi CSV processing exists.

**Impact**: None - this is intentional scope reduction.

**Status**: Acceptable per revised spec.

---

### 2.2 Telemetry Processing in Kotlin

**Spec Requirement**: Rust core telemetry processing with thin Android shell

**Current Implementation**: Substantial telemetry logic in Kotlin:
- `CsvIngestParser.kt`
- `Interpolator.kt`
- `ImuIntegrator.kt`
- `GapInterpolator.kt`
- `QualityFlagger.kt`
- `KeyframeSelector.kt`
- `TelemetryPreprocessor.kt`
- `EnuConverter.kt`
- `PoseStampEmitter.kt`

**Evidence**:
```diff
+ .../java/com/splats/app/telemetry/CsvIngestParser.kt    |  311 ++
+ .../java/com/splats/app/telemetry/Interpolator.kt  |  121 ++
+ .../java/com/splats/app/telemetry/ImuIntegrator.kt |  183 ++
+ .../java/com/splats/app/telemetry/GapInterpolator.kt    |   57 +
+ .../java/com/splats/app/telemetry/QualityFlagger.kt     |  100 ++
+ .../java/com/splats/app/telemetry/KeyframeSelector.kt   |   135 ++
+ .../com/splats/app/telemetry/TelemetryPreprocessor.kt  |  412 ++
+ .../java/com/splats/app/telemetry/EnuConverter.kt  |   89 ++
+ .../com/splats/app/telemetry/PoseStampEmitter.kt   |  136 ++
```

**Impact**: Core research logic in Android layer. Android-specific ownership of reconstruction logic.

**Assessment**: Some components may provide performance benefits. Evaluate case-by-case:
- Quality flagging: valuable for GPS filtering
- Interpolators: may improve temporal alignment
- IMU integration: may improve pose estimation
- Gap filling: may handle telemetry dropouts

**Correction**: Move core telemetry processing to Rust. Retain components that demonstrate clear performance value. Remove unjustified complexity. Centralize coordinate conversions.

---

### 2.3 Telemetry Components Assessment

**Spec Requirement**: Simple telemetry pipeline - parse, quality gate, WGS84->ENU, timestamp alignment

**Current Implementation**: Additional components:
- Multiple interpolator types (general, gap-specific)
- IMU integrator
- Hardware-tier timeout logic
- Binary `PoseStamp` packing
- Diagnostic reporter
- Activity coroutine scope

**Assessment**: Some components may improve performance:
- Interpolators: may handle temporal gaps better
- IMU integrator: may improve pose estimation between GPS updates
- Diagnostic reporter: valuable for debugging

**Correction**: Evaluate each component for demonstrated performance benefit. Retain valuable contributions. Remove unjustified complexity.

---

### 2.4 Duplicated Coordinate Conversions

**Spec Requirement**: Centralized WGS84->ENU conversion, single OpenCV->NeRF conversion

**Current Implementation**: Conversions duplicated between Kotlin and Rust layers.

**Evidence**: `EnuConverter.kt` in Kotlin plus Rust equivalents.

**Impact**: Inconsistency risk. Violates single-conversion requirement. Known failure modes (mirrored/upside-down output, incorrect scale).

**Correction**: Centralize all coordinate math in Rust. Remove Kotlin conversions. Add unit tests as specified.

---

### 2.5 Hardware Detection

**Spec Change**: Hardware detection required for MVP due to Qualcomm 128MB maxStorageBufferBindingSize limitation.

**Current Implementation**: Hardware SoC detection, platform-specific backend switching (qcom vs pxl)

**Evidence**: Commit "automatic backend switching based on platform (qcom vs pxl)"

**Impact**: Necessary hack for MVP to work across target platforms. Qualcomm and Pixel are significantly different.

**Status**: Acceptable as required workaround for hardware constraints. Document as platform-specific adaptation.

---

## 3. Keyframe Selection Deviations

### 3.1 Implementation in Wrong Language

**Spec Requirement**: Pure Rust keyframe module, no OpenCV/Android dependency

**Current Implementation**: Keyframe selection in Kotlin (`KeyframeSelector.kt`)

**Impact**: Ties core logic to Android layer. However, may be acceptable for Android-first architecture.

**Assessment**: Given Android-first usage model, Kotlin implementation may be pragmatic. Evaluate if moving to Rust provides clear benefit.

**Correction**: Consider moving to Rust for architectural consistency, but acceptable in Kotlin for Android-first MVP.

---

### 3.2 Incorrect Distance Threshold

**Spec Requirement**: `distance > 2.0m` (strict greater-than)

**Current Implementation**: Uses `>=` instead of `>`

**Impact**: Off-by-one semantic error. Keyframe selection logic incorrect.

**Correction**: Change to strict `>` as specified.

---

### 3.3 Incorrect Yaw Wraparound

**Spec Requirement**: Use Rust `.rem_euclid()` for negative-safe floating-point modulo

**Current Implementation**: Uses `%` operator

**Impact**: Incorrect wraparound for negative angles. Keyframe selection bug.

**Correction**: Replace `%` with `.rem_euclid()`.

---

### 3.4 Misnamed Distance Function

**Spec Requirement**: Haversine or ENU distance

**Current Implementation**: `haversineMetres` function reportedly implements equirectangular/ENU-style distance, not haversine

**Impact**: Naming confusion. Potential correctness issue if actual haversine needed.

**Correction**: Rename to match actual implementation or implement true haversine if required.

---

### 3.5 Incorrect First Keyframe Logic

**Spec Requirement**: Select from initial valid/non-hover telemetry according to trigger logic

**Current Implementation**: First keyframe always selected (frame-zero/default-first-keyframe behavior)

**Impact**: Forced keyframe outside physical trigger logic. Invalid initial state.

**Correction**: Implement proper first-keyframe selection from valid telemetry per trigger logic.

---

### 3.6 Incorrect 100-Keyframe Overflow Handling

**Spec Requirement**: Documented reservoir sampling or adaptive threshold rerun

**Current Implementation**: Early truncation at 100 frames

**Impact**: Violates spec. Poor sampling strategy.

**Correction**: Implement reservoir sampling or adaptive threshold rerun as specified.

---

## 4. Vision/Geometric Verification Deviations

### 4.1 Mode A/Mode C Scope

**Spec Change**: MVP telemetry-guided only. Mode A vision-only is future scope.

**Current Implementation**: Telemetry-guided implementation exists without explicit Mode A separation

**Impact**: Acceptable for MVP. Vision-only benchmarking deferred.

**Status**: No correction needed for MVP. Mode A can be future work.

---

### 4.2 Missing solvePnPRansac Seeded Prior

**Spec Requirement**: `solvePnPRansac(... useExtrinsicGuess=true, rvec=prior_r, tvec=prior_t)`

**Current Implementation**: Seeded-prior path absent

**Impact**: Spec violation. Missing telemetry guidance mechanism.

**Correction**: Implement seeded `solvePnPRansac` with telemetry prior.

---

### 4.3 Missing Telemetry-Window Matching

**Spec Requirement**: Match feature pairs only when telemetry-derived positions satisfy baseline threshold

**Current Implementation**: Matching restricted to sequential pairs only

**Impact**: Performance mechanism not implemented. Claimed complexity absent.

**Correction**: Implement telemetry-window matching as specified.

---

### 4.4 Uniform Extraction

**Spec Requirement**: Telemetry-based extraction for MVP

**Current Implementation**: Uniform extraction available as alternative

**Impact**: Provides comparison capability. May be useful for research.

**Assessment**: Acceptable as experimental comparison. Not critical for MVP.

**Correction**: Consider demoting to experimental-only, but acceptable to retain for research flexibility.

---

### 4.5 Unclear Low-Inlier Fallback

**Spec Requirement**: Fallback to vision-only when telemetry prior performs badly

**Current Implementation**: Fallback status insufficiently clear

**Impact**: Failure mode handling unclear.

**Correction**: Implement explicit low-inlier fallback from Mode C to Mode A.

---

## 5. Bundle Adjustment Deviations

### 5.1 Incorrect Window Size

**Spec Requirement**: Sliding window 10–20 frames

**Current Implementation**: Default 8 frames, UI clamps max to 5

**Impact**: Outside specified range. Reduced accuracy potential.

**Correction**: Set default to 10-20 range. Remove UI constraint or align with spec.

---

### 5.2 Marginalization Not Implemented

**Spec Requirement**: Marginalization preferred; simple discard acceptable for MVP if documented

**Current Implementation**: Simple discard without confirmed documentation

**Impact**: Accuracy compromise without explicit decision.

**Correction**: Either implement marginalization or document deliberate MVP discard choice.

---

### 5.3 Solver Executed Twice Per Iteration

**Spec Requirement**: Single LM iteration per loop

**Current Implementation**: `run_lm_core` followed by `apply_result_state` - solver appears to run twice

**Impact**: Performance bug. Incorrect iteration counting.

**Correction**: Ensure solver executes exactly once per LM iteration.

---

### 5.4 IMU Jacobian Issues

**Spec Requirement**: Analytically derived Jacobians, progressing from finite-difference

**Current Implementation**: IMU Jacobian reportedly constant, not analytically derived. Jacobian signs may be inverted.

**Impact**: Mathematical correctness suspect. Convergence issues possible.

**Correction**: Derive proper analytic IMU Jacobian. Verify signs through synthetic tests.

---

### 5.5 Frozen Frames Incorrect

**Spec Requirement**: Frozen frames hold states/constraints fixed

**Current Implementation**: Frozen frames skip observations instead

**Impact**: Changes optimization problem incorrectly.

**Correction**: Fix frozen frame handling to hold states fixed while keeping observations.

---

### 5.6 Schur Complement Verification Needed

**Spec Requirement**: Schur complement required for efficient BA

**Current Implementation**: Schur complement exists but correctness requires validation

**Impact**: Efficiency claim may be invalid if implementation incorrect.

**Correction**: Validate Schur implementation against reference. Add synthetic tests.

---

## 6. Pose/Telemetry Semantics

### 6.1 Incorrect Pose Chaining

**Spec Requirement**: Compose relative poses with previous frames

**Current Implementation**: Absolute poses stored without composition

**Impact**: Pose chain broken. Incorrect trajectory reconstruction.

**Correction**: Implement proper pose composition: `pose_i = pose_{i-1} * relative_pose_i`.

---

### 6.2 Incorrect Initial Camera Pose

**Spec Requirement**: Derive/compose actual initial pose

**Current Implementation**: Uses identity matrix

**Impact**: Wrong coordinate system origin. Downstream errors propagate.

**Correction**: Derive initial pose from first valid telemetry, not identity.

---

### 6.3 Incorrect GPS Residual Semantics

**Spec Requirement**: GPS residuals from GPS translation prior

**Current Implementation**: Compares camera centers against ENU translations incorrectly

**Impact**: Wrong residual definition. Optimization targets wrong quantity.

**Correction**: Fix GPS residual to match spec: `||t_i - t_i^{GPS}||^2`.

---

### 6.4 Incorrect IMU Residual Semantics

**Spec Requirement**: Per-camera orientation residuals against measured IMU orientation

**Current Implementation**: Pairwise relative frame deltas

**Impact**: Wrong residual type. IMU prior not used as specified.

**Correction**: Change to per-camera orientation: `||log(R_i(R_i^{IMU})^{-1})||^2`.

---

### 6.5 Incorrect SO(3) Update

**Spec Requirement**: Proper SO(3) composition/local tangent update

**Current Implementation**: Axis-angle deltas applied incorrectly

**Impact**: Manifold violation. Quaternion renormalization issues.

**Correction**: Implement proper `so(3)` tangent-space update with quaternion exponential map.

---

## 7. Camera/Intrinsics Deviations

### 7.1 Incorrect Focal Length Handling

**Spec Requirement**: Read camera metadata, use proper `fx`/`fy`

**Current Implementation**: `recover_pose` assumes equal focal lengths, ignores `fy`. `estimateIntrinsics` guesses from max image dimension

**Impact**: Violates pinhole model. Incorrect projection for non-square pixels.

**Correction**: Read actual camera metadata. Use distinct `fx` and `fy`. Validate projection.

---

## 8. Point Tracking/Representation

### 8.1 Brittle Point Identity

**Spec Requirement**: Stable feature/point identifiers

**Current Implementation**: Float-bit keys for point tracking

**Impact**: Brittle. Floating precision issues. Unstable identity.

**Correction**: Replace with integer feature/point IDs or structured keys.

---

## 9. Frame/Export Handling

### 9.1 Incorrect Exported Filenames

**Spec Requirement**: Use actual source-file basename/path semantics

**Current Implementation**: Hardcoded frame-name patterns

**Impact**: Breaks downstream dataset references. Non-reproducible exports.

**Correction**: Use source video basename in exported filenames.

---

### 9.2 Resolution Behavior Inconsistent

**Spec Requirement**: 640x360 resolution, 100-frame budget

**Current Implementation**: Fixed 640x360 in one path, UI allows/limits max dimension around 360, defaults to 50 frames

**Impact**: Budget enforcement unclear. Inconsistent behavior.

**Correction**: Enforce 640x360 and 100-frame budget consistently across all paths.

---

## 10. Configuration/UI Deviations

### 10.1 JSON Config

**Spec Change**: JSON config required for experimental setup. Allows same config across tests.

**Current Implementation**: JSON configuration exposed through UI, hyperparameter editing surface

**Impact**: Valuable for experimental research. Enables reproducible testing.

**Status**: Acceptable and necessary. Do not remove.

---

### 10.2 Serde Defaults

**Spec Requirement**: Explicit configuration failure

**Current Implementation**: `serde` defaults on `ModelConfig` silently mask missing configuration

**Impact**: Silent failures. Incorrect configuration undetected.

**Assessment**: For experimental setup, some defaults may be acceptable. However, critical config should fail explicitly.

**Correction**: Remove silent defaults for critical parameters. Allow experimental parameters to have sensible defaults.

---

### 10.3 Race Condition from Config

**Spec Requirement**: Thread-safe design

**Current Implementation**: Commit claims to fix race condition caused by JSON config

**Impact**: Suggests fundamental design flaw in config system.

**Assessment**: Race condition fix is good. Need to verify root cause addressed.

**Correction**: Verify race condition properly resolved. Ensure thread-safe config access.

---

### 10.4 UI Scope

**Spec Requirement**: Choose MP4, Choose CSV, Config (JSON)

**Current Implementation**: UI includes BA window, training parameters, hyperparameters

**Impact**: Beyond minimal prototype but valuable for experimental research.

**Assessment**: Extended UI useful for research experimentation. Acceptable for proof-of-concept.

**Correction**: Consider simplifying for production, but acceptable for research MVP.

---

## 11. Android/JNI Deviations

### 11.1 Original Style Violation

**Spec Requirement**: Preserve original `android.rs` style, minimal JNI boundary

**Current Implementation**: `lazy_static` added, student-added complexity

**Impact**: Diverges from original Brush conventions. Maintenance burden.

**Correction**: Restore original `android.rs` style. Remove unnecessary complexity.

---

### 11.2 Training Wiring Unverified

**Spec Requirement**: JNI training wiring complete

**Current Implementation**: JNI training flow insufficiently verified

**Impact**: Unclear if training actually invoked correctly end-to-end.

**Correction**: Verify full JNI training path from Android UI to `brush-train`.

---

### 11.3 Native Library Build

**Spec Change**: Native libraries provided directly as binary in brush repo. Build independently.

**Current Implementation**: Build process exists but libraries will be provided as pre-built binaries

**Impact**: No stale .so concern. Libraries managed as binary artifacts.

**Status**: Acceptable per revised approach.

---

## 12. Dependencies/Invented Code

### 12.1 Vendored OpenCV Headers

**Spec Requirement**: Use standard dependency integration

**Current Implementation**: Vendored OpenCV headers under `third_party/`

**Evidence**:
```diff
+ third_party/opencv/include/opencv2/calib3d.hpp     | 4453 +++++++++++++++++
+ [283 files changed, 134022 insertions(+)]
```

**Impact**: Unnecessary vendoring. Maintenance burden. Should use standard OpenCV dependency.

**Correction**: Remove vendored headers. Use standard `opencv-rust` or system OpenCV integration.

---

### 12.2 Reimplemented Existing Library Functionality

**Spec Requirement**: Use existing libraries where suitable (`rust-cv/levenberg-marquardt`, `apex-solver`, `factrs`, Ceres/g2o for comparison)

**Current Implementation**: Custom solver reimplemented, telemetry processing reinvented, interpolation reinvented

**Impact**: Reinventing wheel. Bug surface. Lost library ecosystem benefits.

**Correction**: Evaluate and use existing solver libraries. Explicitly justify any custom implementation as research contribution.

---

## 13. Documentation/Repo Hygiene

### 13.1 Inaccurate contributions.md

**Current Issue**: Resume-speak/fluff rather than engineering documentation. Claims tasks verified/completed without implementation evidence.

**Evidence**: `contributions.md` lists achievements not supported by code inspection.

**Impact**: Misleading project state. False completion claims.

**Correction**: Remove or rewrite `contributions.md` to reflect actual verified state.

---

### 13.2 README Build Instructions

**Current Issue**: Student-specific Android OpenCV/CMake instructions that conflict with intended architecture

**Impact**: Confusing build documentation. Encourages wrong architecture.

**Correction**: Clean README to reflect intended architecture. Remove student-specific workarounds.

---

### 13.3 Knowledge Base Claims

**Current Issue**: `brush_repository_knowledge_base.md` claims verification without supporting evidence

**Impact**: False project state documentation.

**Correction**: Audit KB claims against actual implementation. Remove unverified assertions.

---

## 14. Training Integration

### 14.1 Unclear Training Path

**Spec Requirement**: SfM export -> stock `brush-train` -> transforms.json + sparse.ply

**Current Implementation**: Training path after SfM export not clearly wired. Full integration requires verification.

**Impact**: Unclear if end-to-end pipeline actually works.

**Correction**: Verify and document complete training path from SfM export through `brush-train`.

---

### 14.2 Config Default Drift

**Spec Requirement**: 100 frames, 2,000 steps, 100,000 max splats

**Current Implementation**:
- BA window defaults to 8 frames (spec: 10-20)
- Training defaults to 30,000 steps (spec: 2,000)
- Max splats defaults to 10,000,000 (spec: 100,000)
- Frame defaults to 50 in some paths (spec: 100)

**Evidence**:
- Line 156 in `stage_3_7_bundle_adjustment.rs`: `window_size: 8`
- Line 9 in `brush-train/src/config.rs`: `default_value = "30000"`
- Line 52 in `brush-train/src/config.rs`: `default_value = "10000000"`

**Impact**: Resource budget not enforced as specified. Performance impact.

**Assessment**: Current defaults may be tuned for specific hardware/performance. Need to validate against target device constraints.

**Correction**: Align with spec or document rationale for deviation. Ensure resource budget enforced on target device.

---

## 15. Commit History Patterns

### 15.1 Commit Message Analysis

**Pattern**: Many commits claim fixes without addressing root architectural issues:
- "fix wgpu regression" - may be symptom of deeper problem
- "fix race condition due to JSON config" - config design itself questionable
- "harden JNI" - suggests fragile JNI design
- "improve platform detection" - hardware detection is invented complexity
- "allow hyperparameter input via UI using JSON" - feature creep
- "remove legacy java telemetry implementation" - correct direction but incomplete
- "implement OpenCV backend stages" - correct work but in wrong location

**Impact**: Symptom-focused fixes rather than architectural correction. Technical debt accumulated.

**Correction**: Address root architectural issues rather than surface symptoms.

---

## 16. Missing Spec Requirements

### 16.1 Synthetic BA Tests

**Spec Requirement**: Synthetic BA tests with known ground truth

**Current Implementation**: Not evidenced in codebase

**Impact**: No verification of BA correctness. No reproducible evidence.

**Correction**: Implement synthetic BA test suite as specified.

---

### 16.2 Desktop Baseline

**Spec Requirement**: Desktop Brush baseline before Android debugging

**Current Implementation**: Desktop architecture now polluted by Android/OpenCV dependencies

**Impact**: Cannot verify desktop baseline independently.

**Correction**: Remove Android/OpenCV from desktop path. Verify desktop baseline first.

---

### 16.3 Validation Artifacts

**Spec Requirement**: RANSAC iteration logs, timing measurements, spatial accuracy measurements, solver comparisons, determinism tests, memory/thermal profiling

**Current Implementation**: Not evidenced as completed

**Impact**: No validation data. No proof of claims.

**Correction**: Implement full validation plan as specified.

---

## 17. Code Style/Quality Violations

### 17.1 Rust Idiom Violations

**Spec Requirement**: Use `.rem_euclid()` for floating-point modulo

**Current Implementation**: Uses `%` for yaw wraparound

**Impact**: Non-idiomatic Rust. Incorrect behavior for negative values.

**Correction**: Use idiomatic Rust methods as specified.

---

### 17.2 Naming Inconsistencies

**Issue**: `haversineMetres` not actually haversine. Crate naming confusion (`brush-process` vs `crates/brush-process`)

**Impact**: Misleading names. Maintenance confusion.

**Correction**: Rename to match actual implementation. Resolve crate naming.

---

## 18. Resource Budget Violations

### 18.1 Frame Budget

**Spec Requirement**: 100 frames maximum

**Current Implementation**: Defaults to 50 frames, inconsistent enforcement

**Impact**: Resource budget not respected.

**Correction**: Enforce 100-frame cap consistently.

---

### 18.2 Resolution Budget

**Spec Requirement**: 640x360

**Current Implementation**: Fixed 640x360 in one path, UI limits around 360

**Impact**: Unclear actual resolution behavior.

**Correction**: Enforce 640x360 consistently.

---

### 18.3 Memory/Runtime Budget

**Spec Requirement**: 3 GB memory, 30 min runtime

**Current Implementation**: No evidence of enforcement or profiling

**Impact**: Budgets may be exceeded on device.

**Correction**: Add profiling. Enforce budgets.

---

## 19. Corrective Priority

### 19.1 Critical (Must Fix)

1. Fix pose chaining/composition
2. Fix GPS/IMU residual definitions
3. Implement proper SO(3) updates
4. Remove duplicated coordinate conversions
5. Fix yaw wraparound with proper modulo
6. Fix first-keyframe selection logic
7. Implement 100-keyframe overflow handling correctly
8. Replace float-bit point identities with stable IDs
9. Fix exported frame filenames to use source basename
10. Implement telemetry-window matching (missing from current impl)

### 19.2 High (Should Fix)

1. Restore crate boundaries - evaluate SfM location (`brush-sfm` vs `brush-process`)
2. Evaluate telemetry components for performance value, retain valuable ones
3. Implement `solvePnPRansac` seeded prior
4. Fix intrinsics/focal handling (use actual metadata, not guessing)
5. Align resource defaults with spec or document rationale
6. Verify end-to-end training wiring
7. Fix frozen frame handling in BA
8. Validate Schur complement implementation

### 19.3 Medium (Important)

1. Move keyframe selector to Rust (or justify Kotlin location)
2. Centralize coordinate conversions with unit tests
3. Add synthetic BA tests for correctness validation
4. Fix unsafe serde defaults for critical parameters
5. Update documentation to reflect actual implementation
6. Implement validation artifacts for research paper

### 19.4 Low (Clean Up)

1. Remove vendored OpenCV headers if standard dependency works
2. Clean up student-specific README instructions
3. Add unit tests for coordinate conversions
4. Profile and enforce memory/runtime budgets on target device
5. Consider simplifying UI to minimal required controls

### 19.5 Acceptable as MVP Hacks

1. Hardware detection (Qualcomm 128MB limitation requires this)
2. JSON config system (necessary for experimental setup)
3. Native libraries as binary artifacts (build process decision)
4. Some telemetry complexity (evaluate case-by-case for performance value)
5. Android-first architecture (matches usage model)

---

## 20. Verification Required

The following findings require direct verification before final classification:

1. Exact OpenCV desktop build behavior
2. Exact MainActivity/VideoFrameExtractor behavior
3. Training wiring end-to-end
4. Exact Litchi parser correctness against sample CSV
5. Exact BA Jacobian signs
6. Mistaken `brush` directory purpose
7. Exact resolution behavior end-to-end
8. Performance value of telemetry components (interpolators, IMU integration, gap filling)

**Verified and resolved**:
- ~~Duplicate `JNI_OnLoad` symbols~~ - Only one exists in codebase
- ~~JNI library loading names match actual exports~~ - Correctly loads brush_process
- ~~DJI SRT support~~ - Intentionally removed from scope
- ~~Hardware detection~~ - Required for Qualcomm 128MB limitation
- ~~JSON config~~ - Required for experimental setup
- ~~Stale .so~~ - Libraries provided as binary artifacts

These should be verified through direct inspection, testing, or before labeling as confirmed defects.

---

## Appendix: File Changes Summary

**Total Changes**: 283 files changed, 134,022 insertions(+), 239 deletions(-)

**Major Additions**:
- Kotlin telemetry layer: ~2,000 lines
- OpenCV vendored headers: ~40,000 lines
- Rust SfM implementation: ~2,500 lines
- Android UI changes: ~800 lines
- Build configuration: ~300 lines

**Key Structural Changes**:
- `brush-process` converted to cdylib with OpenCV
- `brush-sfm` crate added
- Extensive Kotlin telemetry processing
- JSON config system
- Vendored third-party dependencies

---

## Additional Confirmed Findings from Code Inspection

### 21. Brush Directory Status

**Finding**: `brush` is a directory, not a file

**Evidence**: `file /home/netherquark/Soham/Large_code/sources/forks/brush/brush` returns "directory"

**Impact**: Not a mistaken git submodule artifact as previously suspected. May be legitimate directory or unintentional.

**Correction**: Verify purpose of `brush` directory. Remove if unnecessary artifact.

---

### 22. Confirmed Incorrect Distance Threshold

**Finding**: Line 85 in `KeyframeSelector.kt` uses `>=` instead of `>`

**Evidence**: 
```kotlin
dist   >= config.distanceThresholdM  -> KeyframeTrigger.DISTANCE
```

**Impact**: Spec violation. Off-by-one semantic error.

**Correction**: Change to `>` as specified in impl_from_scratch.md.

---

### 23. Confirmed Incorrect Yaw Wraparound

**Finding**: Line 132 in `KeyframeSelector.kt` uses `%` instead of proper negative-safe modulo

**Evidence**:
```kotlin
var diff = (yaw2 - yaw1 + 360.0) % 360.0
```

**Impact**: Not using `.rem_euclid()` equivalent. Incorrect for negative angles.

**Correction**: Implement proper negative-safe floating-point modulo.

---

### 24. Confirmed First Keyframe Always Selected

**Finding**: Lines 63-72 in `KeyframeSelector.kt` always add first row as keyframe

**Evidence**:
```kotlin
keyframes += KeyframeCandidate(
    rowIndex      = 0,
    timestampUs   = rows.first().timestampUs,
    ...
    triggerReason = KeyframeTrigger.FIRST
)
```

**Impact**: Violates spec requirement to select from initial valid/non-hover telemetry per trigger logic.

**Correction**: Implement proper first-keyframe selection based on trigger conditions.

---

### 25. Confirmed Incorrect 100-Keyframe Handling

**Finding**: Line 75 in `KeyframeSelector.kt` implements early truncation

**Evidence**:
```kotlin
if (keyframes.size >= maxKeyframes) break
```

**Impact**: Simple truncation instead of reservoir sampling or adaptive threshold rerun.

**Correction**: Implement documented overflow handling strategy.

---

### 26. Confirmed Misnamed Distance Function

**Finding**: `haversineMetres` function implements equirectangular approximation, not true haversine

**Evidence**: Lines 120-125 in `KeyframeSelector.kt`:
```kotlin
fun haversineMetres(lat1: Double, lon1: Double, lat2: Double, lon2: Double): Double {
    val latMidRad = Math.toRadians((lat1 + lat2) / 2.0)
    val dLatM = (lat2 - lat1) * 111_320.0
    val dLonM = (lon2 - lon1) * 111_320.0 * cos(latMidRad)
    return sqrt(dLatM.pow(2) + dLonM.pow(2))
}
```

**Impact**: Naming is misleading. Implementation is equirectangular/ENU-style, not haversine formula.

**Correction**: Rename to match implementation (e.g., `equirectangularDistanceMetres`) or implement true haversine if required.

---

### 27. Confirmed Duplicated Coordinate Conversion

**Finding**: `EnuConverter.kt` in Kotlin implements WGS84->ENU conversion

**Evidence**: Entire `EnuConverter.kt` file with ENU conversion logic

**Impact**: Duplicates conversion that should be centralized in Rust per spec.

**Correction**: Remove Kotlin conversion. Centralize in Rust with unit tests.

---

### 28. Confirmed IMU Residual Structure Mismatch

**Finding**: `ImuRotationPrior` struct uses pairwise delta rotation instead of per-camera orientation

**Evidence**: Lines 42-52 in `stage_3_7_bundle_adjustment.rs`:
```rust
pub struct ImuRotationPrior {
    #[serde(default)]
    pub frame_a: usize,
    #[serde(default)]
    pub frame_b: usize,
    #[serde(default)]
    pub delta_rotation: [[f64; 3]; 3],
    ...
}
```

**Impact**: Spec requires per-camera orientation residuals against measured IMU orientation: `||log(R_i(R_i^{IMU})^{-1})||^2`. Current implementation uses pairwise deltas.

**Correction**: Change IMU residual structure to per-camera orientation prior.

---

### 29. Confirmed Point Tracking Using Float Bits

**Finding**: Lines 386-411 in `sfm/mod.rs` use float-bit keys for point identity

**Evidence**:
```rust
let x_bits = (obs_a.observed[0] as f32).to_bits();
let y_bits = (obs_a.observed[1] as f32).to_bits();
let key_prev = (obs_a.frame_idx, x_bits, y_bits);
```

**Impact**: Brittle identity mechanism. Floating precision issues. Unstable tracking.

**Correction**: Replace with integer feature/point IDs.

---

### 30. Confirmed Hardcoded Export Filenames

**Finding**: Lines 70-77 in `stage_3_8_pose_export.rs` use fallback pattern

**Evidence**:
```rust
let file_path = if let Some(path) = frame_paths.get(i) {
    Path::new(path).file_name()
        .and_then(|n| n.to_str())
        .unwrap_or("frame.jpg")
        .to_string()
} else {
    format!("frame_{:05}.jpg", i)
};
```

**Impact**: Hardcoded patterns instead of source-file basename semantics.

**Correction**: Use actual source video basename consistently.

---

### 31. Confirmed Double SfM Module Exposure

**Finding**: `brush-process/src/lib.rs` both declares local `pub mod sfm` and re-exports `brush-sfm`

**Evidence**: Lines 7-25 in `brush-process/src/lib.rs`:
```rust
pub use brush_sfm::{
    BaResult,
    BaState,
    ...
};
...
pub mod sfm;
```

**Impact**: Namespace collision. Duplicate exposure paths.

**Correction**: Remove duplicate. Choose single SfM location.

---

### 32. Confirmed Coordinate Conversion in Export

**Finding**: Lines 55-61 in `stage_3_8_pose_export.rs` implement OpenCV->NeRF conversion

**Evidence**:
```rust
// OpenCV C2W (x-right, y-down, z-forward) -> NeRF C2W (x-right, y-up, z-back)
let mut c2w_nerf = c2w_cv;
for r in 0..3 {
    c2w_nerf[(r, 1)] *= -1.0; // Flip Y column
    c2w_nerf[(r, 2)] *= -1.0; // Flip Z column
}
```

**Impact**: Conversion exists but duplicates logic that should be centralized and unit-tested per spec.

**Correction**: Centralize single OpenCV->NeRF conversion function in Rust with unit tests.

---

#

---

**Next Steps**: Systematic correction starting with Critical priority items, architectural boundary restoration, and verification of mathematical correctness.
