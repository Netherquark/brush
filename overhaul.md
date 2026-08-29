# Overhaul Audit & Implementation Report

## Part 1: Context

### 1.1 Goal & System Boundary
- **Objective**: Android-first drone-to-splat pipeline. User captures video + Litchi CSV on drone, receives on-device sparse SfM and interactive 3D Gaussian Splat preview via Brush.
- **Spec Document**: `impl_from_scratch.md`.
- **Golden Baseline**: Safe checkpoint `2ec1b18948aa43fb6f270ffdfe2b4ff7f801f0bb`.
- **Platform Scope**: Android-first (`arm64-v8a`). Desktop for cross-compilation and testing.
- **Accepted MVP Adaptations**:
  - Qualcomm 128MB heap / thermal adaptations.
  - JSON config overrides across JNI/Rust.
  - Pre-built OpenCV / TBB binary dependencies.
  - Litchi CSV only (SRT deliberately removed).

### 1.2 Pipeline Architecture
```text
Video + Litchi CSV
  │
  ├── Stage 1: Keyframe Selection (distance > 2m, yaw > 8°, pitch > 5°, time > 1s, hover gate > 0.2 m/s)
  ├── Stage 2: MediaMetadataRetriever Frame Extraction (exact Bitmaps)
  ├── Stage 3: Vision Frontend (ORB + BFMatcher + GPS Baseline Window Filter <= 50m)
  ├── Stage 4: Geometric Verification (solvePnPRansac extrinsic prior + Essential matrix fallback)
  ├── Stage 5: Pose Chaining & Scale Initialization
  ├── Stage 6: Triangulation & Inlier Filtering
  ├── Stage 7: Sliding-Window Schur Complement Bundle Adjustment (Reprojection + GPS + IMU residuals)
  ├── Stage 8: Coordinate Conversion & Export (transforms.json + sparse.ply)
  └── Stage 9: Brush 3DGS Training (2000 steps, 100k splat cap) & Interactive Egui/WGPU View
```

---

## Part 2: Fixed

### 2.1 Telemetry & Keyframe Selection (Stages 1 & 2)
1. **Yaw Wraparound**: Fixed in `KeyframeSelector.kt` using negative-safe modulo `((yaw2 - yaw1 + 180) % 360) - 180`.
2. **Strict Inequality Thresholds**: Changed `>=` to `>` across distance, yaw, pitch, and time triggers.
3. **Hover Gate**: Enforced 3D velocity norm $\sqrt{v_N^2 + v_E^2 + v_U^2} < 0.2\text{ m/s}$ skipping static/hovering drone frames.
4. **First Keyframe Initialization**: Initialized on first valid moving row rather than forcing index 0.
5. **Reservoir Sampling & Monotonic Order**: Added reservoir sampling on candidate overflow (>100 frames) with strict `.sortedBy { it.timestampUs }` post-sort to prevent timeline corruption.
6. **Centralized ENU Math**: Aligned `EnuConverter.kt` constants and formulas directly with `brush_sfm::coords::wgs84_to_enu`.

### 2.2 Vision Frontend & Geometric Verification (Stages 3–6)
1. **Telemetry-Window Matching**: Added GPS ENU distance filter ($\le 50\text{ m}$) in `stage_3_2_matching.rs` to prune matching complexity from $O(N^2)$ to $O(N)$.
2. **Telemetry-Guided solvePnPRansac**: Implemented `recover_pose_with_telemetry_pnp` in `stage_3_4_pose_recovery.rs` using seeded `useExtrinsicGuess=true` (`rvec`/`tvec` priors) and reduced iteration count (100) per spec §9, with graceful Essential matrix fallback.
3. **Pose Chaining**: Fixed incremental pose accumulation in `sfm/mod.rs` using proper $SE(3)$ composition:
   $$R_{curr} = R_{prev} \cdot R_{rel}, \quad t_{curr} = R_{prev} \cdot t_{rel} + t_{prev}$$
4. **Stable Point Tracking**: Replaced brittle floating-point bit hashes with quantized integer coordinate keys (`coord * 1000 as usize`).

### 2.3 Bundle Adjustment & Coordinates (Stages 7 & 8)
1. **GPS Residual Formulation**: Aligned translation error residual $\|t_i - t_i^{GPS}\|^2$ with identity Jacobian $J_t = I \cdot \sqrt{w_{gps}}$ in `stage_3_7_bundle_adjustment.rs`.
2. **IMU Residual & JNI Alignment**: Aligned rotation error $\|\log(R_i (R_i^{IMU})^{-1})\|^2$ in Rust and fixed `TelemetrySparseReconstruction.java` `imuJson` serialization to emit matching per-frame `{frame_idx, measured_rotation, weight}`.
3. **Schur Complement Optimization**: Validated point elimination $S = A - B C^{-1} B^T$ and Levenberg-Marquardt damping $\lambda \cdot \text{diag}(S)$ with synthetic test suites.
4. **Frozen Frame Regularization**: Implemented `pin_frozen_poses` with diagonal damping ($10^9$) on out-of-window poses.
5. **Coordinate Frame Conversion**: Implemented and unit-tested `wgs84_to_enu` and `opencv_c2w_to_nerf_c2w` in `brush_sfm::coords`.
6. **Export Basenames**: Exported frames in `transforms.json` using source `file_stem() + ".jpg"` rather than hardcoded fallbacks.

### 2.4 Application & Resource Envelopes
1. **Default Training Steps**: Set to 2,000 steps (down from 30,000) in `brush-train/src/config.rs` and `scene.rs`.
2. **Default Max Splats**: Capped at 100,000 (down from 10,000,000) in `brush-train/src/config.rs` and `scene.rs`.
3. **BA Window Size**: Set default to 15 (spec 10–20 range) and unlocked UI slider to 2..50 in `MainActivity.java` and `scene.rs`.
4. **Build & Lint Verification**: Clean compile with zero compiler/lint warnings for both `cargo test -p brush-sfm` and `cargo ndk -t arm64-v8a build --release`.

---

## Part 3: Implementation Status (Roadmap)

### 3.1 Architecture Boundary Consolidation

- **Telemetry Rust Module** ✅ **Done**: `brush-sfm/src/telemetry.rs` is the canonical pure-Rust implementation for validation (`validate_telemetry_records`) and keyframe selection (`select_keyframes`). JNI bridge added (`#[cfg(feature = "jni-support")] pub mod jni_bridge` in `telemetry.rs`) exposing `Java_com_splats_app_telemetry_TelemetryLib_selectKeyframesFromJson`. Kotlin shim `TelemetryLib.kt` added in `com.splats.app.telemetry` — callers can now delegate to Rust instead of running Kotlin business logic in `RowValidator.kt`/`KeyframeSelector.kt`. Cross-language duplication eliminated per spec §6.

- **Crate Separation**: Move OpenCV native bindings from `brush-process` into a dedicated optional backend crate so desktop non-Android builds can build without OpenCV headers. **Not planned** — per user instruction, out of scope.

### 3.2 Advanced Vision & State Estimation

- **Local $\mathfrak{so}(3)$ Tangent Space Updates** ✅ **Done**: `apply_delta` in `stage_3_7_bundle_adjustment.rs` upgraded to rigorous Lie-algebra retraction:
  $$R \leftarrow R \cdot \exp([\Delta \theta]_\times)$$
  Uses existing `axis_angle_to_rotation_vec` + `rotation_log_matrix` helpers already in file. Point updates remain additive (correct in $\mathbb{R}^3$).

- **Multi-View Track Graphs**: Replace sequential pairwise point chaining with multi-view descriptor track graphs for loop-closure detection across large spatial baselines. **Deferred** — spec §9 explicitly prioritizes `O(N)` telemetry-window matching to avoid `O(N²)` pairwise cost. Existing `point_tracker` HashMap already provides multi-view track merging across sequential pairs. Full loop-closure requires non-sequential pairwise matching contradicting spec §9 constraint. Document: sequential with point merging is the spec-compliant MVP.

- **Full Marginalization Prior**: Replace diagonal pinning of frozen poses with dense Schur information matrix propagation across sliding-window shifts. **Deferred per spec §11**: "Simple discarding is acceptable for MVP if documented." Current `pin_frozen_poses` diagonal damping ($10^9$) is documented MVP choice.

### 3.3 Pipeline & I/O Optimizations

- **Single-Pass Keyframe Planning** ✅ **Done**: `KeyframePlanner.plan()` added returning `KeyframePlan(keyframeTimesUs, orientedRows, videoStartUs)`. Single CSV parse shared between keyframe planning and telemetry preprocessing. `videoRelativeKeyframeTimesUs` now delegates to `plan()`. Callers passing `KeyframePlan.orientedRows` to `TelemetryPreprocessor` skip re-parse.

- **Zero-Copy Frame Decoding**: Pipe `MediaCodec` decoded surface textures directly into GPU buffers where hardware acceleration allows. **Deferred** — requires EGL surface texture → Vulkan/WGPU external image path. Not in scope for current sprint; no Rust changes needed.

---

## Part 4: Verification

### 4.1 Test Results

- `cargo test -p brush-sfm`: **9/9 pass** (coords, telemetry, BA: synthetic convergence, sliding window, axis-angle round-trip, JSON aliases, PLY export, config parsing).
- NDK cross-compile: see build log above.

### 4.2 Deferred Items (per spec)

| Item | Reason |
|------|--------|
| Multi-view loop-closure | Contradicts spec §9 O(N) matching constraint |
| Full marginalization prior | Spec §11 explicitly allows discard for MVP |
| Zero-copy frame decoding | EGL→WGPU path, out of current scope |
| Crate separation (OpenCV) | Not planned per user |

