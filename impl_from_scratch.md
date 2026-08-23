# Architecture

## 1. System boundary

Android APK built on Brush renderer. User flies drone, captures footage, receives interactive 3D Gaussian-splat preview for rapid land/terrain surveying.

**Platform priority**: Android-first for usage. Desktop used only for compilation/testing.

Pipeline has three parts:

1. SfM preprocessing, Stages 1–8.
2. 3DGS training, Stage 9.
3. On-device rendering, Stage 10.

Core novelty stays in:

* Stage 1: physical pose-change keyframe selection.
* Stage 4: GPS/IMU/gimbal pose priors for geometric verification.
* Stage 7: lean Levenberg-Marquardt sliding-window Bundle Adjustment.

No new splatting algorithm.

## 2. Pipeline architecture

```text
Video + Telemetry
  |
  |-- telemetry parser
  |-- quality gate
  |-- WGS84 -> ENU
  |
  +-- physical pose-change keyframe selection
  |     distance > 2.0 m
  |     yaw > 8.0°
  |     gimbal pitch > 5.0°
  |     time > 1.0 s
  |
  +-- exact frame extraction
  |
  +-- ORB feature detection
  |
  +-- BFMatcher + Hamming + Lowe ratio test
  |
  +-- telemetry-guided geometric verification
  |     GPS/IMU pose prior
  |     solvePnPRansac useExtrinsicGuess=true
  |     telemetry-window matching
  |     fallback to vision-only when prior performs badly
  |
  +-- pose recovery
  |
  +-- triangulation
  |
  +-- sparse point cloud
  |
  +-- sliding-window LM BA
  |     reprojection residuals
  |     GPS residuals
  |     IMU residuals
  |     Schur complement
  |
  +-- coordinate-frame conversion
  |
  +-- transforms.json
  +-- sparse.ply
        |
        v
   Brush brush-train
        |
        +-- MCMC-style splat growth/pruning
        +-- --max-splats ceiling
        |
        v
   Interactive Brush rendering
```

## 3. Repository architecture

Use Brush as renderer and Android application shell.

Relevant components:

* `Core Splats<B: Backend>`: Gaussian transform storage.
* `brush-render`: forward projection/rasterization.
* `brush-renderbwd`: backward/gradient pass.
* `brush-train`: MCMC-style Gaussian growth/pruning.
* `brush-dataset`: COLMAP and Nerfstudio `transforms.json` loaders.
* `brush-ui`: egui UI.
* `brush-app`: desktop/Android shell.
* `android.rs`: JNI bridge.
* `brush-sfm` / `brush-ba`: SfM and BA location.
* `brushprocess`: processing path.

Do not modify renderer internals unless required by integration.

Novel functionality lives in `brush-sfm` or separate `brush-ba`.

Target output format: Nerfstudio `transforms.json` plus `sparse.ply`.

## 4. Coordinate architecture

Keep coordinate conversions centralized.

```text
GPS:
(lat, lon, alt)

Working frame:
ENU = (east, north, up)

OpenCV camera:
x-right, y-down, z-forward

NeRF:
x-right, y-up, z-back
```

Use:

```text
E = (lon - lon0) * cos(lat0 * π / 180) * 111320
N = (lat - lat0) * 111320
```

Implement:

```text
wgs84_to_enu(lat, lon, alt, origin) -> (e, n, u)
```

Implement OpenCV-to-NeRF conversion once:

```text
(x, y, z)_cv -> (x, -y, -z)_nerf
```

Unit-test both. No manual matrix conversion elsewhere.

## 5. Telemetry architecture

Input:

* Litchi CSV only (DJI SRT support removed).

Parse:

```rust
struct RawTelemetryRecord {
    timestamp,
    lat,
    lon,
    alt,
    yaw,
    pitch,
    roll,
    gimbal_pitch,
    vel_n,
    vel_e,
    vel_d,
    hdop,
    num_sats,
}
```

Quality gate:

* reject `hdop > 3.0`.
* reject non-3D GPS fix.

Timestamp alignment:

* video creation timestamp + frame index.
* linearly align against telemetry timestamps.
* cross-correlation remains research-grade nice-to-have if time permits.
* document deliberate scope decision.

Per-keyframe output:

```text
PoseStamp
  position in ENU
  orientation quaternion
  telemetry timestamp
```

Convert PoseStamp into OpenCV rotation/translation prior.

## 6. Keyframe architecture

Pure Rust module:

```text
Vec<ValidatedRecord> -> Vec<KeyframeCandidate>
```

No OpenCV dependency. No Android dependency.

State:

```text
last_keyframe_pose
last_keyframe_time
```

For each validated telemetry row:

```text
speed = sqrt(vel_n² + vel_e² + vel_d²)

if speed < 0.2 m/s:
    continue

distance = haversine_or_enu_dist(r.pos, last_keyframe_pose.pos)
yaw_delta = shortest_angle_diff(r.yaw, last_keyframe_pose.yaw)
pitch_delta = abs(r.gimbal_pitch - last_keyframe_pose.gimbal_pitch)
time_delta = r.time - last_keyframe_time

if distance > 2.0m
   or yaw_delta > 8.0°
   or pitch_delta > 5.0°
   or time_delta > 1.0s:
    emit KeyframeCandidate(r, reason = argmax trigger)
    last_keyframe_pose, last_keyframe_time = r.pose, r.time
```

Yaw wraparound:

```text
((yaw2 - yaw1 + 180) % 360) - 180
```

Rust implementation must use `.rem_euclid()` for negative floating-point modulo.

Hard cap: 100 keyframes.

Long flights producing more than 100 candidates require documented handling. Candidate approaches already identified:

* reservoir-sample excess;
* scale thresholds adaptively and rerun once.

## 7. Frame extraction architecture

Telemetry chooses frames first.

Use Android `MediaMetadataRetriever` with surviving timestamps to extract exact Bitmaps.

This keeps frame selection separate from video decode cost.

Compare three methods:

1. Physical pose-change trigger.
2. Fixed interval.
3. Mean absolute pixel difference.

Measure:

* keyframe count.
* spatial coverage.
* bounding box.
* path length divided by frame count.
* downstream SfM reprojection error.

## 8. Vision architecture

Use OpenCV:

* ORB extraction.
* BFMatcher.
* Hamming distance.
* Lowe ratio test.
* RANSAC / essential matrix.
* pose recovery.
* triangulation.

Validate vision pipeline first on public SfM data before drone footage.

## 9. Telemetry-guided geometric verification

Build telemetry pose prior from GPS/IMU/gimbal.

Total cost:

[
E_{total}
=========

\sum_{ij}
|x_{ij}-\Pi(R_i,t_i,K_i,X_j)|^2_{\Sigma_{vis}}
+
\sum_i
|t_i-t_i^{GPS}|^2_{\Sigma_{gps}}
+
\sum_i
|\log(R_i(R_i^{IMU})^{-1})|^2_{\Sigma_{imu}}
]

Information matrices are inverse covariances.

GPS quality and IMU noise determine sensor weighting.

**MVP scope**: Telemetry-guided only. Vision-only Mode A is future scope.

Pass telemetry prior into:

```text
cv2.solvePnPRansac(
    ...,
    useExtrinsicGuess=True,
    rvec=prior_r,
    tvec=prior_t
)
```

Use lower `iterationsCount`.

Only match feature pairs when telemetry-derived positions satisfy baseline threshold (telemetry-window matching).

Target effect: reduce matching from approximately `O(N²)` toward `O(N)`.

If telemetry-guided estimation produces low inlier ratio, current implementation may fail gracefully. Bad GPS must be tested explicitly.

## 10. BA architecture

BA minimizes:

[
E =
\sum_{i,j}
|x_{ij}-\Pi(R_i,t_i,K_i,X_j)|^2
]

Extended solver adds GPS and IMU residuals.

Core types:

```rust
struct Camera {
    rotation: UnitQuaternion<f64>,
    translation: Vector3<f64>,
    intrinsics: Mat3
}

struct Point3D {
    position: Vector3<f64>
}

struct Observation {
    camera_idx: usize,
    point_idx: usize,
    pixel: Vector2<f64>
}

struct GpsResidual {
    camera_idx: usize,
    measured_t: Vector3<f64>,
    weight: f64
}

struct ImuResidual {
    camera_idx: usize,
    measured_r: UnitQuaternion<f64>,
    weight: f64
}
```

Projection:

```text
Π(camera, point) -> Vector2<f64>
```

Use standard pinhole projection:

1. transform point into camera frame.
2. perspective divide.
3. apply `K`.

LM:

[
(J^TJ+\lambda,diag(J^TJ))\Delta=-J^Tr
]

Iteration:

1. linearize residuals.
2. compute Jacobian.
3. solve damped system.
4. apply update.
5. evaluate error.
6. accept lower error and shrink `λ`.
7. reject higher error and grow `λ`.
8. repeat until convergence or iteration cap.

Exploit BA block structure.

Use Schur complement to eliminate point blocks first, then solve smaller camera system.

This is required for efficient BA.

## 11. Sliding-window architecture

Maintain:

```text
VecDeque<Camera>
```

Window size: 10–20 frames.

For each new keyframe:

1. add camera.
2. add observations.
3. drop oldest frame when window exceeds bound.
4. rerun LM.

Marginalization is preferred for VIO-grade accuracy.

Simple discarding is acceptable for MVP if documented.

Rotation representation:

* unit quaternions.
* renormalize after LM update, or use local `so(3)` tangent-space update.

## 12. LM implementation choices

Option 1:

`rust-cv/levenberg-marquardt`

Use nalgebra-based LM core. Implement bundle-adjustment residual/Jacobian through `LeastSquaresProblem`.

Contribution remains:

* sliding window.
* multimodal residuals.
* Schur complement.
* mobile memory budget.

Option 2:

Implement damping and trust-region logic on raw nalgebra.

Higher academic ownership. More work. More bug surface. Budget Sprint 3 accordingly.

Read:

* `apex-solver`.
* `factrs`.

Use their benchmark methodology for positioning. Do not depend on them unless selected as solver.

## 13. Synthetic BA architecture

Generate:

* random 3D points.
* known camera trajectory.
* projected observations.
* Gaussian pixel noise.
* GPS noise.
* IMU noise.

Tests:

1. vision-only convergence.
2. telemetry-fused convergence.
3. pose/point recovery against ground truth.
4. iterations to tolerance.
5. wall-clock time.

Controlled synthetic data provides reproducible evidence for telemetry-fused convergence.

## 14. Brush training architecture

Export:

```text
transforms.json
sparse.ply
```

Feed into stock `brush-train`.

Brush training uses MCMC-style splat growth/pruning.

No manual Gaussian-count tuning.

Use:

```text
--max-splats
```

as ceiling.

Target ceiling: 100,000 splats.

Training budget:

```text
2,000 steps
100,000 splats maximum
```

## 15. Android architecture

Android-first for usage. Desktop used only for compilation/testing.

Android shell:

```text
brush-app
  |
  +-- Android UI
  +-- android.rs JNI bridge
  +-- TelemetrySparseReconstruction.java
  +-- Rust native libraries
```

Add prototype UI controls:

* Choose MP4.
* Choose CSV.
* Config (JSON).

Enforce:

```text
100 frames
640×360
3 GB memory
30 min runtime
```

Target phone: Pixel 9a.

## 15.1 Hardware detection and adaptation

**Qualcomm vs Pixel adaptation required** due to 128MB maxStorageBufferBindingSize limitation on Qualcomm stack.

Implement hardware detection and adaptive pipeline configuration:

- Detect Qualcomm vs Pixel platform
- Adjust memory allocations and buffer sizes per platform
- Scale resource usage based on hardware constraints

This is necessary hack for MVP to work across target platforms.

## 16. Build architecture

Rust:

```bash
cargo ndk -t arm64-v8a -o crates/brush-app/app/src/main/jniLibs/ build --release
```

Android:

```bash
./gradlew build
./gradlew installDebug
adb shell am start -n com.splats.app/.MainActivity
```

**Native libraries**: Provided directly as binary in brush repo. Build independently. No stale .so concern.

Required build order:

```text
desktop Rust build (for compilation/testing)
Android NDK cross-compile
Android install
```

## 16.1 JSON configuration

JSON configuration is required for experimental setup. Allows same config across tests. Do not remove.

UI should accept JSON config file for SfM/BA hyperparameters.

# Implementation Plan

## Sprint 0: Environment and Brush baseline

Install:

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
rustup target add aarch64-linux-android
cargo install cargo-ndk
sdkmanager --install "ndk;27.0.12077973" "platform-tools" "platforms;android-34"
export ANDROID_NDK_HOME=$HOME/Android/Sdk/ndk/27.0.12077973
```

Install OpenCV Android SDK. Pin exact upstream version in build documentation.

Clone Brush:

```bash
git clone https://github.com/ArthurBrussee/brush.git
cd brush
cargo build --release
```

Fork/branch Brush.

Run stock desktop demo with sample COLMAP dataset.

Artifact: Brush renders public sample dataset on laptop.

## Sprint 1: Telemetry and keyframes

Implement:

* Litchi CSV parser.
* DJI SRT parser.
* quality gate.
* WGS84 to ENU.
* timestamp alignment.
* PoseStamp.
* physical pose-change trigger.
* yaw wraparound.
* 100-keyframe cap handling.

Add unit tests:

* ENU distance.
* yaw wraparound.
* hover rejection.
* constant-speed spatial cadence.

Artifact: CLI accepts sample Litchi CSV and prints keyframes with trigger reasons.

## Sprint 2: Vision pipeline

Implement with `opencv-rust`:

* ORB.
* BFMatcher.
* Hamming.
* Lowe ratio test.
* RANSAC / essential matrix.
* pose recovery.
* triangulation.

Validate on public COLMAP/SfM dataset before drone footage.

Artifact: sparse point cloud plus camera poses, visually sane when plotted.

## Sprint 3: BA solver

Create:

```bash
cargo new --lib brush-ba
```

Implement:

* camera.
* point.
* observations.
* pinhole projection.
* reprojection residuals.
* GPS residuals.
* IMU residuals.
* LM.
* quaternion handling.
* Schur complement.
* sliding window.

Start with finite-difference Jacobians.

Move to analytic Jacobians after numerical version passes synthetic tests.

Add:

```text
cargo test
cargo bench
Criterion.rs
```

Use synthetic scenes with known ground truth.

Artifact: tests recover known poses from noisy observations. Benchmark shows vision-only versus telemetry-fused iteration counts.

## Sprint 4: Telemetry-guided pose recovery

Connect Sprint 1 telemetry priors to Sprint 2 vision.

Implement:

```text
Mode A
vision-only

Mode C(a)
solvePnPRansac with telemetry pose prior

Mode C(b)
telemetry-window feature matching
```

Add low-inlier fallback from Mode C to Mode A.

Log:

```text
ransac_iterations_used
wall_clock_ms
inlier_ratio
```

Artifact: Mode A versus Mode C comparison on real DJI Mini 2 + Litchi footage.

## Sprint 5: SfM export and Brush training

Implement:

* coordinate-frame conversion.
* Nerfstudio `transforms.json`.
* `sparse.ply`.

Feed output into stock `brush-train`.

Artifact: Gaussian splat trained from own drone footage, viewable in desktop Brush viewer.

## Sprint 6: Android integration

Port Sprint 1–5 pipeline into `brush-app`.

Connect:

```text
android.rs
TelemetrySparseReconstruction.java
JNI
```

Build native libraries:

```bash
cargo ndk -t arm64-v8a -o crates/brush-app/app/src/main/jniLibs/ build --release
```

Install Android build.

Artifact: full pipeline runs on device, even if slow.

## Sprint 7: Mobile optimization

Add egui controls from prototype.

Enforce:

```text
100-frame limit
640×360 resolution
3 GB memory budget
30-minute runtime budget
```

Profile:

* memory.
* wall-clock time.
* thermal behavior.

Fix thermal throttling.

Artifact: full target-phone run under 30 minutes on Pixel 9a, screen-recorded.

## Sprint 8: Validation and release

Complete quantitative experiments.

Publish `brush-ba` to crates.io.

Prepare technical paper.

Prepare Brush maintainer PR discussion.

Add:

* README.
* LICENSE matching Brush licensing.
* GitHub Actions.
* `cargo test`.
* `cargo clippy`.
* `cargo publish --dry-run`.

# Validation Plan

## RANSAC

Telemetry-guided Mode C only (MVP). Mode A vision-only is future scope.

Report iteration count per frame pair:

```text
baseline: 1,000–5,000
target: 50–200
```

Report mean ± standard deviation.

## Pipeline time

Measure full on-device pipeline.

Run at least 3 times.

Use device cooldown between runs.

Report:

```text
Mode A
Mode C
45–60 min baseline
12–18 min target
```

## Spatial accuracy

Measure 3–5 physically measured reference distances at capture site.

Compare physical distance against reconstructed point-cloud/splat distance.

Target:

```text
<0.5 m
```

## Solver comparison

Measure:

* reprojection RMSE.
* iterations to convergence.
* wall-clock milliseconds.
* peak memory.

Compare:

1. own solver, vision-only.
2. own solver, telemetry-fused.
3. Ceres or g2o if available, even desktop-only.

Use same synthetic data.

## Determinism

Run solver twice on identical input.

Diff output bit-for-bit.

Expected own-solver result: identical.

If Ceres is available, repeat comparison and report any run/platform variation.

## Memory and thermal

Use:

```text
Android Studio Profiler
adb shell dumpsys thermalservice
```

Measure full run.

Target:

```text
<3 GB
no thermal throttling
```

## BA quality

Report:

* reprojection RMSE before BA.
* reprojection RMSE after BA.
* PSNR/SSIM on held-out validation renders.

## Keyframe comparison

Compare:

1. physical pose-change.
2. fixed interval.
3. pixel difference.

Report:

* keyframe count.
* spatial coverage.
* bounding box.
* path length / frame count.
* downstream SfM reprojection error.

## Raw artifacts

Keep raw CSV logs for:

* RANSAC iterations.
* timings.
* inlier ratios.
* solver iterations.
* memory.
* thermal measurements.

Use raw logs as appendix/supplementary artifact.

# Failure Controls

## Coordinate failure

Symptoms:

* mirrored splats.
* upside-down splats.
* incorrect scale.

Control:

* one coordinate conversion function.
* known-point unit test.
* no manual matrix edits elsewhere.

## Stale Android native library

Android Studio can run stale `.so`.

Control:

```bash
cargo ndk -t arm64-v8a -o crates/brush-app/app/src/main/jniLibs/ build --release
```

after every Rust change.

## Bad GPS

GPS-denied periods degrade priors.

Control:

* `hdop > 3.0` quality gate.
* reject non-3D GPS fixes.
* explicitly test bad-GPS footage.
* fallback to Mode A when telemetry-guided estimation has low inlier ratio.

## Wrong telemetry initialization

Wrong prior can push LM toward worse local minimum.

Control:

* monitor inlier ratio.
* fallback to blind vision estimation.
* report accuracy beside iteration count.

## Misleading speed result

Fewer iterations alone do not prove improvement.

Control:

* report reprojection error with timing and iterations.
* compare Mode A and Mode C on identical data.

## Thermal benchmark drift

Repeated runs can throttle device.

Control:

* minimum 3 runs.
* device cooldown between runs.
* log thermal state.

## Keyframe overflow

More than 100 candidates violates mobile memory constraint.

Control:

* document selected handling.
* reservoir-sample excess or scale thresholds adaptively and rerun once.

# Required Reading

Read before code:

* Kerbl et al., 2023, *3D Gaussian Splatting for Real-Time Radiance Field Rendering*.
* ArthurBrussee/brush README and DeepWiki architecture documentation.
* Brush Gaussian Splat Representation and rendering-pipeline documentation.
* `rust-cv/levenberg-marquardt`.
* `apex-solver`.
* `factrs`.
* Triggs et al. (2000), Schur-complement sparse Bundle Adjustment reference.
* Nerfstudio `transforms.json` specification.
* Current Litchi CSV documentation.
* `opencv-rust`.
* Ceres/g2o documentation for BA comparison.

Read Brush source before modifying `brush-render`.

Read `apex-solver` and `factrs` benchmark suites for benchmark methodology.

Primary implementation order remains:

```text
Brush desktop baseline
telemetry/keyframe module
vision pipeline
synthetic BA
telemetry-guided pose recovery
SfM export
desktop Gaussian training
Android JNI integration
mobile profiling
full validation
publication
```
