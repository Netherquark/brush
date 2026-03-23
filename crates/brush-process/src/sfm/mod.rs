// ============================================================
// FILE PATH: crates/brush-process/src/sfm/mod.rs
//
// PROJECT: brush-app — Drone-Driven On-Device Gaussian Splatting
//
// PURPOSE:
//   SfM (Structure from Motion) module root.
//   Stages 3.1–3.6 (OpenCV-based) are declared as stubs here so the
//   project compiles even if those files are not yet present.
//   Stage 3.7 (pure Rust BA) is fully implemented and exported.
//
// PLACE THIS FILE AT:
//   <workspace_root>/crates/brush-process/src/sfm/mod.rs
//
// PIPELINE OVERVIEW:
//   Stage 3.1  feature_extraction    — OpenCV ORB (needs OpenCV)
//   Stage 3.2  matching              — OpenCV BFMatcher + GPS window pruning
//   Stage 3.3  ransac                — OpenCV findEssentialMat / USAC_MAGSAC
//   Stage 3.4  pose_recovery         — OpenCV recoverPose
//   Stage 3.5  triangulation         — OpenCV triangulatePoints
//   Stage 3.6  inlier_filtering      — Pure Rust
//   Stage 3.7  bundle_adjustment     ← THIS FILE (pure Rust, zero OpenCV)
//   Stage 3.8  pose_export           — Pure Rust (writes transforms.json / sparse.ply)
// ============================================================

// ── Stage 3.7: Sliding-Window Bundle Adjustment (implemented) ────────────────
pub mod stage_3_7_bundle_adjustment;

// ── Placeholder stubs for OpenCV stages (fill in as you implement them) ───────
// Uncomment each line when the corresponding file exists:
//
// pub mod stage_3_1_feature_extraction;
// pub mod stage_3_2_matching;
// pub mod stage_3_3_ransac;
// pub mod stage_3_4_pose_recovery;
// pub mod stage_3_5_triangulation;
// pub mod stage_3_6_inlier_filtering;
// pub mod stage_3_8_pose_export;

// ── Contract type shared across stage boundaries ─────────────────────────────

use nalgebra::{Matrix3, Vector3};
use stage_3_7_bundle_adjustment::{CameraIntrinsics, GpsPrior, ImuRotationPrior, Observation};

/// Output contract from Stage 3.6 → Stage 3.7 (inlier filtering → BA)
///
/// Populated by Stage 3.6 and handed directly to `run_sliding_window_ba()`.
pub struct Stage36Output {
    /// (frame_id, rotation_matrix) for each keyframe
    pub frame_rotations: Vec<(usize, Matrix3<f64>)>,
    /// (frame_id, translation_vector) for each keyframe
    pub frame_translations: Vec<(usize, Vector3<f64>)>,
    /// 3-D positions of inlier points
    pub point_positions: Vec<Vector3<f64>>,
    /// 2-D observations linking frames to 3-D points
    pub observations: Vec<Observation>,
    /// GPS priors from telemetry Stage 2
    pub gps_priors: Vec<GpsPrior>,
    /// IMU rotation priors from telemetry Stage 2
    pub imu_priors: Vec<ImuRotationPrior>,
    /// Camera intrinsics (from DJI metadata or calibration)
    pub intrinsics: CameraIntrinsics,
}
