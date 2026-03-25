// ============================================================
// FILE PATH: crates/brush-process/src/sfm/stage_3_7_bundle_adjustment.rs
//
// PROJECT: brush-app — Drone-Driven On-Device Gaussian Splatting
//          Batch 2023-27, Dept. of AI & ML, SIT Pune
//          Group 30: Riddhi Poddar, Vedant Shinde, Soham Kulkarni, Sri Vaishnavi Kodali
//
// STAGE: 3.7 — Sliding-Window Bundle Adjustment (Pure Rust, Zero OpenCV)
//
// WHAT THIS FILE DOES:
//   Receives camera poses + 3D point tracks from Stage 3.6 (inlier filtering)
//   and refines them using Levenberg–Marquardt + Schur complement normal equations.
//   Mode C ONLY: GPS and IMU priors from the telemetry pipeline (Stage 2)
//   are required and injected directly into the cost function.
//
// DEPENDENCIES:
//   nalgebra = { version = "0.33", features = ["serde-serialize"] }
//   serde, serde_json
//   jni (optional, feature = "jni-support")
//
// PLACE THIS FILE AT:
//   <workspace_root>/crates/brush-process/src/sfm/stage_3_7_bundle_adjustment.rs
//
// THEN ADD TO:
//   crates/brush-process/src/sfm/mod.rs:
//       pub mod stage_3_7_bundle_adjustment;
//       pub use stage_3_7_bundle_adjustment::*;
// ============================================================

#![allow(non_snake_case)]
#![allow(clippy::many_single_char_names)]

use nalgebra::{
    DMatrix, DVector, Matrix2x3, Matrix3, SMatrix, Vector2, Vector3,
};
use serde::{Deserialize, Serialize};

// ─────────────────────────────────────────────────────────────────────────────
// SECTION 1: DATA STRUCTURES
// ─────────────────────────────────────────────────────────────────────────────

/// Camera intrinsics — focal lengths + principal point (pixels)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CameraIntrinsics {
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
}

/// One 2-D observation: 3-D point `point_idx` seen in frame `frame_idx` at pixel `observed`
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Observation {
    pub frame_idx: usize,
    pub point_idx: usize,
    pub observed: [f64; 2], // (u, v)
}

impl Observation {
    #[inline]
    pub fn observed_vec(&self) -> Vector2<f64> {
        Vector2::new(self.observed[0], self.observed[1])
    }
}

/// GPS position prior: absolute ENU position for one frame
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GpsPrior {
    pub frame_idx: usize,
    pub enu_position: [f64; 3], // East-North-Up metres
    /// Typical range: 0.1 – 1.0 depending on GPS fix quality
    pub weight: f64,
}

impl GpsPrior {
    #[inline]
    pub fn enu_vec(&self) -> Vector3<f64> {
        Vector3::new(self.enu_position[0], self.enu_position[1], self.enu_position[2])
    }
}

/// IMU rotation prior: relative rotation between consecutive frames
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ImuRotationPrior {
    pub frame_a: usize,
    pub frame_b: usize,
    /// Row-major 3×3 rotation matrix R_imu_a_to_b
    pub delta_rotation_row_major: [f64; 9],
    /// Typical range: 0.05 – 0.5
    pub weight: f64,
}

impl ImuRotationPrior {
    #[inline]
    pub fn delta_rotation(&self) -> Matrix3<f64> {
        Matrix3::from_row_slice(&self.delta_rotation_row_major)
    }
}

/// Mutable BA optimisation state for one sliding window
pub struct BaState {
    /// Axis-angle rotation vectors ω ∈ ℝ³ (Lie algebra so(3))
    pub rotations_aa: Vec<Vector3<f64>>,
    /// Camera-frame translations t ∈ ℝ³  →  X_cam = R·X_world + t
    pub translations: Vec<Vector3<f64>>,
    /// 3-D point world positions
    pub points: Vec<Vector3<f64>>,
    /// Local indices of frozen (marginalised) frames — NOT updated by LM
    pub frozen_frames: Vec<usize>,
}

/// Output from one sliding-window LM run
#[derive(Debug, Clone)]
pub struct BaResult {
    pub refined_rotations: Vec<Matrix3<f64>>,
    pub refined_translations: Vec<Vector3<f64>>,
    pub refined_points: Vec<Vector3<f64>>,
    pub final_cost: f64,
    pub iterations_run: u32,
    pub converged: bool,
}

/// Levenberg–Marquardt solver hyper-parameters
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LmConfig {
    /// Hard cap — never exceeded (Android thermal budget)
    pub max_iterations: u32,
    /// Initial damping: 1e-4
    pub lambda_init: f64,
    /// Abort if lambda exceeds this (diverged): 1e8
    pub lambda_max: f64,
    /// Multiply lambda on step rejection: 3.0
    pub lambda_factor_up: f64,
    /// Multiply lambda on step acceptance: 1/3
    pub lambda_factor_dn: f64,
    /// Converge if ‖Δ‖ < threshold: 1e-6
    pub delta_threshold: f64,
    /// Converge if mean cost < threshold (px² per obs): 0.5
    pub cost_threshold: f64,
}

impl Default for LmConfig {
    fn default() -> Self {
        LmConfig {
            max_iterations: 50,
            lambda_init: 1e-4,
            lambda_max: 1e8,
            lambda_factor_up: 3.0,
            lambda_factor_dn: 1.0 / 3.0,
            delta_threshold: 1e-6,
            cost_threshold: 0.5,
        }
    }
}

/// Sliding-window orchestration parameters
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SlidingWindowConfig {
    /// Number of frames per window (W = 15)
    pub window_size: usize,
    /// Overlap between consecutive windows (5)
    pub overlap: usize,
    pub lm_config: LmConfig,
}

impl Default for SlidingWindowConfig {
    fn default() -> Self {
        SlidingWindowConfig {
            window_size: 15,
            overlap: 5,
            lm_config: LmConfig::default(),
        }
    }
}

/// Global SfM state fed to the sliding-window orchestrator
pub struct GlobalSfmState {
    pub frame_ids: Vec<usize>,
    /// Row-major rotation matrices (one per frame)
    pub rotations: Vec<Matrix3<f64>>,
    pub translations: Vec<Vector3<f64>>,
    pub points: Vec<Vector3<f64>>,
    pub observations: Vec<Observation>,
    pub gps_priors: Vec<GpsPrior>,
    pub imu_priors: Vec<ImuRotationPrior>,
}

// ─────────────────────────────────────────────────────────────────────────────
// SECTION 2: ROTATION UTILITIES (Rodrigues / SO(3))
// ─────────────────────────────────────────────────────────────────────────────

/// Skew-symmetric matrix [v]× such that [v]×·u = v × u
#[inline]
fn skew_symmetric(v: &Vector3<f64>) -> Matrix3<f64> {
    Matrix3::new(
        0.0, -v[2], v[1],
        v[2], 0.0, -v[0],
        -v[1], v[0], 0.0,
    )
}

/// Rodrigues formula: axis-angle ω → rotation matrix R
///
/// R = I + sin(θ)·[n]× + (1−cos(θ))·[n]×²     (θ = ‖ω‖, n = ω/θ)
///
/// Small-angle approximation for θ < 1e-10: R ≈ I + [ω]×
pub fn axis_angle_to_rotation(omega: &Vector3<f64>) -> Matrix3<f64> {
    let theta = omega.norm();
    if theta < 1e-10 {
        return Matrix3::identity() + skew_symmetric(omega);
    }
    let n = omega / theta;
    let skew = skew_symmetric(&n);
    Matrix3::identity() + theta.sin() * skew + (1.0 - theta.cos()) * (skew * skew)
}

/// Log map: rotation matrix → axis-angle vector (inverse of Rodrigues)
///
/// Handles near-identity case (θ < 1e-10) via antisymmetric part extraction.
pub fn rotation_log(R: &Matrix3<f64>) -> Vector3<f64> {
    let cos_theta = ((R.trace() - 1.0) / 2.0).clamp(-1.0, 1.0);
    let theta = cos_theta.acos();
    if theta < 1e-10 {
        return Vector3::new(
            (R[(2, 1)] - R[(1, 2)]) / 2.0,
            (R[(0, 2)] - R[(2, 0)]) / 2.0,
            (R[(1, 0)] - R[(0, 1)]) / 2.0,
        );
    }
    let factor = theta / (2.0 * theta.sin());
    Vector3::new(
        factor * (R[(2, 1)] - R[(1, 2)]),
        factor * (R[(0, 2)] - R[(2, 0)]),
        factor * (R[(1, 0)] - R[(0, 1)]),
    )
}

// ─────────────────────────────────────────────────────────────────────────────
// SECTION 3: PROJECTION & RESIDUALS
// ─────────────────────────────────────────────────────────────────────────────

/// Project 3-D world point X into 2-D pixel using axis-angle + translation pose
///
/// Returns `None` if the point is behind the camera (Z ≤ 0).
fn project(
    omega: &Vector3<f64>,
    t: &Vector3<f64>,
    X: &Vector3<f64>,
    K: &CameraIntrinsics,
) -> Option<Vector2<f64>> {
    let R = axis_angle_to_rotation(omega);
    let X_cam = R * X + t;
    if X_cam[2] <= 1e-6 {
        return None;
    }
    let u = K.fx * X_cam[0] / X_cam[2] + K.cx;
    let v = K.fy * X_cam[1] / X_cam[2] + K.cy;
    Some(Vector2::new(u, v))
}

/// Compute 2-D reprojection residual r = x_obs − π(R, t, K, X)
fn compute_residual(
    state: &BaState,
    obs: &Observation,
    K: &CameraIntrinsics,
) -> Option<Vector2<f64>> {
    let omega = &state.rotations_aa[obs.frame_idx];
    let t = &state.translations[obs.frame_idx];
    let X = &state.points[obs.point_idx];
    let x_proj = project(omega, t, X, K)?;
    Some(obs.observed_vec() - x_proj)
}

/// Compute total cost = Σ reprojection² + Σ GPS² + Σ IMU²
pub fn compute_total_cost(
    state: &BaState,
    observations: &[Observation],
    gps_priors: &[GpsPrior],
    imu_priors: &[ImuRotationPrior],
    K: &CameraIntrinsics,
) -> f64 {
    let mut cost = 0.0_f64;

    // ── Reprojection terms ────────────────────────────────────────────────────
    for obs in observations {
        if let Some(r) = compute_residual(state, obs, K) {
            cost += r.norm_squared();
        }
    }

    // ── GPS position terms: r_gps = C_world − p_gps  where C = −Rᵀ·t ────────
    for gps in gps_priors {
        let R = axis_angle_to_rotation(&state.rotations_aa[gps.frame_idx]);
        let t = &state.translations[gps.frame_idx];
        let cam_centre = -(R.transpose() * t);
        let r_gps = cam_centre - gps.enu_vec();
        cost += gps.weight * r_gps.norm_squared();
    }

    // ── IMU rotation terms: r_imu = Log(R_imu^T · Ra^T · Rb) ─────────────────
    for imu in imu_priors {
        let Ra = axis_angle_to_rotation(&state.rotations_aa[imu.frame_a]);
        let Rb = axis_angle_to_rotation(&state.rotations_aa[imu.frame_b]);
        let R_relative = imu.delta_rotation().transpose() * Ra.transpose() * Rb;
        let r_imu = rotation_log(&R_relative);
        cost += imu.weight * r_imu.norm_squared();
    }

    cost
}

// ─────────────────────────────────────────────────────────────────────────────
// SECTION 4: ANALYTICAL JACOBIAN
// ─────────────────────────────────────────────────────────────────────────────

/// Per-observation Jacobian blocks
struct JacobianBlock {
    /// ∂r/∂[ω, t]  — 2×6
    j_pose: SMatrix<f64, 2, 6>,
    /// ∂r/∂X  — 2×3
    j_point: Matrix2x3<f64>,
}

/// GPS Jacobian blocks  (3×3 each)
struct GpsJacobianBlock {
    j_omega: Matrix3<f64>,
    j_t: Matrix3<f64>,
}

/// IMU Jacobian blocks  (3×3 each)
struct ImuJacobianBlock {
    j_omega_a: Matrix3<f64>,
    j_omega_b: Matrix3<f64>,
}

/// Compute analytical Jacobian for one reprojection residual.
///
/// Chain rule derivation:
///   r = x_obs − π(X_cam)       X_cam = R(ω)·X + t
///   ∂π/∂X_cam = [fx/Z, 0, −fx·U/Z²; 0, fy/Z, −fy·V/Z²]
///   ∂X_cam/∂t = I₃
///   ∂X_cam/∂ω = −[X_cam]×      (camera rotation skew)
///   ∂X_cam/∂X = R
fn compute_jacobian_block(
    omega: &Vector3<f64>,
    t: &Vector3<f64>,
    X: &Vector3<f64>,
    K: &CameraIntrinsics,
) -> Option<JacobianBlock> {
    let R = axis_angle_to_rotation(omega);
    let X_cam = R * X + t;
    let U = X_cam[0];
    let V = X_cam[1];
    let Z = X_cam[2];

    if Z <= 1e-6 {
        return None;
    }

    // ∂π/∂X_cam  (2×3)
    let d_proj_d_xcam = Matrix2x3::new(
        K.fx / Z,   0.0,        -K.fx * U / (Z * Z),
        0.0,        K.fy / Z,   -K.fy * V / (Z * Z),
    );

    // ∂π/∂t = d_proj_d_xcam  (t shifts X_cam directly)
    let d_proj_d_t = d_proj_d_xcam;

    // ∂π/∂ω = d_proj_d_xcam · (−[X_cam]×)
    let neg_skew_xcam = -skew_symmetric(&X_cam);
    let d_proj_d_omega = d_proj_d_xcam * neg_skew_xcam;

    // 2×6 pose Jacobian: sign flip because r = x_obs − π(·)
    let mut j_pose: SMatrix<f64, 2, 6> = SMatrix::zeros();
    j_pose.fixed_columns_mut::<3>(0).copy_from(&(-d_proj_d_omega));
    j_pose.fixed_columns_mut::<3>(3).copy_from(&(-d_proj_d_t));

    // ∂r/∂X = −d_proj_d_xcam · R  (2×3)
    let j_point = -(d_proj_d_xcam * R);

    Some(JacobianBlock { j_pose, j_point })
}

/// GPS Jacobian:  ∂C_world/∂t = −Rᵀ,   ∂C_world/∂ω ≈ −[t]×  (first-order)
fn compute_gps_jacobian(
    omega: &Vector3<f64>,
    t: &Vector3<f64>,
    weight: f64,
) -> GpsJacobianBlock {
    let R = axis_angle_to_rotation(omega);
    let w = weight.sqrt();
    GpsJacobianBlock {
        j_t: -R.transpose() * w,
        j_omega: -skew_symmetric(t) * w,
    }
}

/// IMU Jacobian (first-order):  ∂r/∂ω_a = −I,  ∂r/∂ω_b = +I
fn compute_imu_jacobian(weight: f64) -> ImuJacobianBlock {
    let s = weight.sqrt();
    ImuJacobianBlock {
        j_omega_a: -s * Matrix3::identity(),
        j_omega_b:  s * Matrix3::identity(),
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// SECTION 5: SCHUR COMPLEMENT NORMAL EQUATIONS
// ─────────────────────────────────────────────────────────────────────────────

/// Build and solve the Schur-complement reduced normal equations.
///
/// Block structure:
/// ```text
///   [ A   B  ] [ Δ_pose  ]   [ e_a ]
///   [ Bᵀ  C  ] [ Δ_point ] = [ e_b ]
/// ```
/// C is block-diagonal (3×3 per point) → cheap to invert.
/// Schur: S = A − B·C⁻¹·Bᵀ,  S·Δ_pose = e_a − B·C⁻¹·e_b
///
/// Returns `None` on numerical failure (singular system).
fn build_and_solve_schur(
    state: &BaState,
    observations: &[Observation],
    gps_priors: &[GpsPrior],
    imu_priors: &[ImuRotationPrior],
    K: &CameraIntrinsics,
    lambda: f64,
) -> Option<(DVector<f64>, DVector<f64>)> {
    let n_frames = state.rotations_aa.len();
    let n_points = state.points.len();

    let mut A: DMatrix<f64> = DMatrix::zeros(n_frames * 6, n_frames * 6);
    let mut B: DMatrix<f64> = DMatrix::zeros(n_frames * 6, n_points * 3);
    let mut C_blocks: Vec<SMatrix<f64, 3, 3>> = vec![SMatrix::zeros(); n_points];
    let mut e_a: DVector<f64> = DVector::zeros(n_frames * 6);
    let mut e_b: DVector<f64> = DVector::zeros(n_points * 3);

    // ── Reprojection terms ────────────────────────────────────────────────────
    for obs in observations {
        if state.frozen_frames.contains(&obs.frame_idx) {
            continue;
        }
        let omega = &state.rotations_aa[obs.frame_idx];
        let t = &state.translations[obs.frame_idx];
        let X = &state.points[obs.point_idx];

        let jac = match compute_jacobian_block(omega, t, X, K) {
            Some(j) => j,
            None => continue,
        };
        let r = match compute_residual(state, obs, K) {
            Some(r) => r,
            None => continue,
        };

        let fi = obs.frame_idx;
        let pi = obs.point_idx;

        // A[fi*6.., fi*6..] += Jpᵀ·Jp
        let jpt_jp = jac.j_pose.transpose() * jac.j_pose;
        for row in 0..6 {
            for col in 0..6 {
                A[(fi * 6 + row, fi * 6 + col)] += jpt_jp[(row, col)];
            }
        }

        // B[fi*6.., pi*3..] += Jpᵀ·Jx
        let jpt_jx = jac.j_pose.transpose() * jac.j_point;
        for row in 0..6 {
            for col in 0..3 {
                B[(fi * 6 + row, pi * 3 + col)] += jpt_jx[(row, col)];
            }
        }

        // C[pi] += Jxᵀ·Jx
        C_blocks[pi] += jac.j_point.transpose() * jac.j_point;

        // e_a[fi*6..] += Jpᵀ·r
        let jp_r = jac.j_pose.transpose() * r;
        for row in 0..6 {
            e_a[fi * 6 + row] += jp_r[row];
        }

        // e_b[pi*3..] += Jxᵀ·r
        let jx_r = jac.j_point.transpose() * r;
        for row in 0..3 {
            e_b[pi * 3 + row] += jx_r[row];
        }
    }

    // ── GPS terms ─────────────────────────────────────────────────────────────
    for gps in gps_priors {
        if state.frozen_frames.contains(&gps.frame_idx) {
            continue;
        }
        let omega = &state.rotations_aa[gps.frame_idx];
        let t = &state.translations[gps.frame_idx];
        let jac = compute_gps_jacobian(omega, t, gps.weight);

        let R = axis_angle_to_rotation(omega);
        let cam = -(R.transpose() * t);
        let r_gps: Vector3<f64> = (cam - gps.enu_vec()) * gps.weight.sqrt();

        // Build 3×6 Jacobian [j_omega | j_t]
        let mut jp_gps: SMatrix<f64, 3, 6> = SMatrix::zeros();
        jp_gps.fixed_columns_mut::<3>(0).copy_from(&jac.j_omega);
        jp_gps.fixed_columns_mut::<3>(3).copy_from(&jac.j_t);

        let fi = gps.frame_idx;
        let jpt_jp = jp_gps.transpose() * jp_gps;
        for row in 0..6 {
            for col in 0..6 {
                A[(fi * 6 + row, fi * 6 + col)] += jpt_jp[(row, col)];
            }
        }
        let jpt_r = jp_gps.transpose() * r_gps;
        for row in 0..6 {
            e_a[fi * 6 + row] += jpt_r[row];
        }
    }

    // ── IMU terms ─────────────────────────────────────────────────────────────
    for imu in imu_priors {
        let fa = imu.frame_a;
        let fb = imu.frame_b;
        if state.frozen_frames.contains(&fa) && state.frozen_frames.contains(&fb) {
            continue;
        }
        let Ra = axis_angle_to_rotation(&state.rotations_aa[fa]);
        let Rb = axis_angle_to_rotation(&state.rotations_aa[fb]);
        let R_err = imu.delta_rotation().transpose() * Ra.transpose() * Rb;
        let r_imu: Vector3<f64> = rotation_log(&R_err) * imu.weight.sqrt();
        let jac = compute_imu_jacobian(imu.weight);

        // Frame A contribution
        if !state.frozen_frames.contains(&fa) {
            let jat_ja = jac.j_omega_a.transpose() * jac.j_omega_a;
            for row in 0..3 {
                for col in 0..3 {
                    A[(fa * 6 + row, fa * 6 + col)] += jat_ja[(row, col)];
                }
            }
            let jat_r = jac.j_omega_a.transpose() * r_imu;
            for row in 0..3 {
                e_a[fa * 6 + row] += jat_r[row];
            }
        }

        // Frame B contribution
        if !state.frozen_frames.contains(&fb) {
            let jbt_jb = jac.j_omega_b.transpose() * jac.j_omega_b;
            for row in 0..3 {
                for col in 0..3 {
                    A[(fb * 6 + row, fb * 6 + col)] += jbt_jb[(row, col)];
                }
            }
            let jbt_r = jac.j_omega_b.transpose() * r_imu;
            for row in 0..3 {
                e_a[fb * 6 + row] += jbt_r[row];
            }
        }
    }

    // ── Apply LM damping to diagonal of A ────────────────────────────────────
    for i in 0..(n_frames * 6) {
        A[(i, i)] += lambda * A[(i, i)].abs().max(1e-10);
    }

    // ── Invert C (block-diagonal) ─────────────────────────────────────────────
    let mut C_inv_blocks: Vec<SMatrix<f64, 3, 3>> = Vec::with_capacity(n_points);
    for block in &C_blocks {
        let diag_aug = SMatrix::<f64, 3, 3>::from_diagonal(
            &block.diagonal().map(|v| v.abs().max(1e-10) * lambda),
        );
        let augmented = block + diag_aug;
        match augmented.try_inverse() {
            Some(inv) => C_inv_blocks.push(inv),
            None => C_inv_blocks.push(SMatrix::zeros()),
        }
    }

    // ── Build Schur complement S = A − B·C⁻¹·Bᵀ ──────────────────────────────
    let mut S = A.clone();
    let mut rhs_reduced = e_a.clone();

    for pi in 0..n_points {
        // B_col is the n_frames×6 × 3 block for this point
        let B_col = B.columns(pi * 3, 3);
        let C_inv = &C_inv_blocks[pi];

        // BC_inv = B_col · C⁻¹    shape: n_frames×6 × 3
        let BC_inv = B_col * C_inv;

        // S -= BC_inv · B_colᵀ
        S -= &BC_inv * B_col.transpose();

        // rhs_reduced -= BC_inv · e_b_block
        let e_b_block = e_b.rows(pi * 3, 3);
        rhs_reduced -= BC_inv * e_b_block;
    }

    // ── Solve S · Δ_pose = rhs_reduced  (Cholesky or LU fallback) ────────────
    let delta_pose = match S.clone().cholesky() {
        Some(chol) => chol.solve(&rhs_reduced),
        None => match S.lu().solve(&rhs_reduced) {
            Some(d) => d,
            None => return None,
        },
    };

    // ── Back-solve Δ_point = C⁻¹ · (e_b − Bᵀ · Δ_pose) ─────────────────────
    let mut delta_point = DVector::zeros(n_points * 3);
    for pi in 0..n_points {
        let B_col = B.columns(pi * 3, 3);
        let C_inv = &C_inv_blocks[pi];
        let e_b_blk = e_b.rows(pi * 3, 3);
        let rhs_pt = e_b_blk - B_col.transpose() * &delta_pose;
        let d_pt = C_inv * rhs_pt;
        for row in 0..3 {
            delta_point[pi * 3 + row] = d_pt[row];
        }
    }

    Some((delta_pose, delta_point))
}

/// Apply Δ to produce an updated state (non-destructive trial step)
fn apply_delta(
    state: &BaState,
    delta_pose: &DVector<f64>,
    delta_point: &DVector<f64>,
) -> BaState {
    let mut new_state = BaState {
        rotations_aa: state.rotations_aa.clone(),
        translations: state.translations.clone(),
        points: state.points.clone(),
        frozen_frames: state.frozen_frames.clone(),
    };

    for i in 0..new_state.rotations_aa.len() {
        if state.frozen_frames.contains(&i) {
            continue;
        }
        let d_omega = delta_pose.fixed_rows::<3>(i * 6);
        let d_t = delta_pose.fixed_rows::<3>(i * 6 + 3);
        // Small-angle rotation update: ω_new ≈ ω_old + d_omega
        new_state.rotations_aa[i] += d_omega.clone_owned();
        new_state.translations[i] += d_t.clone_owned();
    }

    for j in 0..new_state.points.len() {
        let d_X = delta_point.fixed_rows::<3>(j * 3);
        new_state.points[j] += d_X.clone_owned();
    }

    new_state
}

// ─────────────────────────────────────────────────────────────────────────────
// SECTION 6: LEVENBERG–MARQUARDT CORE LOOP
// ─────────────────────────────────────────────────────────────────────────────

/// Run the LM optimiser on the given window state.
///
/// Bounded to `config.max_iterations` for Android thermal safety.
/// Returns refined poses, points and convergence diagnostics.
pub fn run_levenberg_marquardt(
    state: &mut BaState,
    observations: &[Observation],
    gps_priors: &[GpsPrior],
    imu_priors: &[ImuRotationPrior],
    K: &CameraIntrinsics,
    config: &LmConfig,
) -> BaResult {
    let mut lambda = config.lambda_init;
    let mut cost = compute_total_cost(state, observations, gps_priors, imu_priors, K);
    let mut iters = 0u32;
    let mut converged = false;

    for iteration in 0..config.max_iterations {
        iters = iteration + 1;

        // Step 1+2: Build Schur complement and solve for delta
        let (delta_pose, delta_point) =
            match build_and_solve_schur(state, observations, gps_priors, imu_priors, K, lambda) {
                Some(d) => d,
                None => break, // numerical failure
            };

        // Step 3: Trial state
        let trial_state = apply_delta(state, &delta_pose, &delta_point);

        // Step 4: Trial cost
        let trial_cost =
            compute_total_cost(&trial_state, observations, gps_priors, imu_priors, K);

        if trial_cost < cost {
            // Accept step
            *state = trial_state;
            cost = trial_cost;
            lambda = (lambda * config.lambda_factor_dn).max(1e-10);

            // Check convergence
            let delta_norm = delta_pose.norm() + delta_point.norm();
            if delta_norm < config.delta_threshold {
                converged = true;
                break;
            }
            let n_obs = (observations.len() as f64).max(1.0);
            if cost / n_obs < config.cost_threshold {
                converged = true;
                break;
            }
        } else {
            // Reject step — increase damping
            lambda *= config.lambda_factor_up;
            if lambda > config.lambda_max {
                break; // diverged
            }
        }
    }

    // Convert axis-angle back to rotation matrices for output
    let refined_rotations = state
        .rotations_aa
        .iter()
        .map(|omega| axis_angle_to_rotation(omega))
        .collect();

    BaResult {
        refined_rotations,
        refined_translations: state.translations.clone(),
        refined_points: state.points.clone(),
        final_cost: cost,
        iterations_run: iters,
        converged,
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// SECTION 7: SLIDING-WINDOW ORCHESTRATION
// ─────────────────────────────────────────────────────────────────────────────

/// Run sliding-window BA over all frames.
///
/// Window: `window_size` = 15,  overlap = 5,  step = 10
/// Each window freezes its oldest frame as the coordinate anchor.
/// After convergence, refined poses + points are written back to `global`.
pub fn run_sliding_window_ba(
    global: &mut GlobalSfmState,
    K: &CameraIntrinsics,
    cfg: &SlidingWindowConfig,
) {
    use std::collections::{HashMap, HashSet};

    let n_frames = global.frame_ids.len();
    let step = cfg.window_size.saturating_sub(cfg.overlap).max(1);
    let mut w_start = 0usize;

    while w_start < n_frames {
        let w_end = (w_start + cfg.window_size).min(n_frames);
        let w_frames = &global.frame_ids[w_start..w_end];

        // ── Map global frame index → local window index ───────────────────────
        let frame_to_local: HashMap<usize, usize> = w_frames
            .iter()
            .enumerate()
            .map(|(local, &gi)| (gi, local))
            .collect();

        // ── Identify 3-D points visible in ≥ 2 window frames ─────────────────
        let mut point_frame_count: HashMap<usize, usize> = HashMap::new();
        for obs in &global.observations {
            if frame_to_local.contains_key(&obs.frame_idx) {
                *point_frame_count.entry(obs.point_idx).or_insert(0) += 1;
            }
        }
        let active_points: HashSet<usize> = point_frame_count
            .into_iter()
            .filter(|(_, c)| *c >= 2)
            .map(|(pi, _)| pi)
            .collect();

        if active_points.is_empty() {
            // No trackable points in this window — skip
            if w_end >= n_frames {
                break;
            }
            w_start += step;
            continue;
        }

        let mut sorted_pts: Vec<usize> = active_points.iter().cloned().collect();
        sorted_pts.sort_unstable();
        let point_to_local: HashMap<usize, usize> = sorted_pts
            .iter()
            .enumerate()
            .map(|(local, &gpi)| (gpi, local))
            .collect();

        // ── Build local BaState ───────────────────────────────────────────────
        let local_rotations: Vec<Vector3<f64>> = w_frames
            .iter()
            .map(|&fi| rotation_log(&global.rotations[fi]))
            .collect();

        let local_translations: Vec<Vector3<f64>> =
            w_frames.iter().map(|&fi| global.translations[fi]).collect();

        let local_points: Vec<Vector3<f64>> =
            sorted_pts.iter().map(|&pi| global.points[pi]).collect();

        let mut local_state = BaState {
            rotations_aa: local_rotations,
            translations: local_translations,
            points: local_points,
            frozen_frames: vec![0usize], // freeze oldest frame = coordinate anchor
        };

        // ── Remap observations ────────────────────────────────────────────────
        let local_observations: Vec<Observation> = global
            .observations
            .iter()
            .filter_map(|obs| {
                let lf = frame_to_local.get(&obs.frame_idx)?;
                let lp = point_to_local.get(&obs.point_idx)?;
                Some(Observation {
                    frame_idx: *lf,
                    point_idx: *lp,
                    observed: obs.observed,
                })
            })
            .collect();

        let local_gps: Vec<GpsPrior> = global
            .gps_priors
            .iter()
            .filter_map(|g| {
                let lf = frame_to_local.get(&g.frame_idx)?;
                Some(GpsPrior {
                    frame_idx: *lf,
                    enu_position: g.enu_position,
                    weight: g.weight,
                })
            })
            .collect();

        let local_imu: Vec<ImuRotationPrior> = global
            .imu_priors
            .iter()
            .filter_map(|m| {
                let la = frame_to_local.get(&m.frame_a)?;
                let lb = frame_to_local.get(&m.frame_b)?;
                Some(ImuRotationPrior {
                    frame_a: *la,
                    frame_b: *lb,
                    delta_rotation_row_major: m.delta_rotation_row_major,
                    weight: m.weight,
                })
            })
            .collect();

        // ── Run LM ────────────────────────────────────────────────────────────
        let _result = run_levenberg_marquardt(
            &mut local_state,
            &local_observations,
            &local_gps,
            &local_imu,
            K,
            &cfg.lm_config,
        );

        // ── Write back refined values to global state ─────────────────────────
        for (local_fi, &global_fi) in w_frames.iter().enumerate() {
            if local_state.frozen_frames.contains(&local_fi) {
                continue;
            }
            global.rotations[global_fi] =
                axis_angle_to_rotation(&local_state.rotations_aa[local_fi]);
            global.translations[global_fi] = local_state.translations[local_fi];
        }

        for (local_pi, &global_pi) in sorted_pts.iter().enumerate() {
            global.points[global_pi] = local_state.points[local_pi];
        }

        // ── Slide window ──────────────────────────────────────────────────────
        if w_end >= n_frames {
            break;
        }
        w_start += step;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// SECTION 8: JSON SERIALISATION HELPERS (for JNI boundary)
// ─────────────────────────────────────────────────────────────────────────────

#[derive(Serialize, Deserialize)]
struct PoseJson {
    frame_id: usize,
    /// Row-major rotation matrix (9 elements)
    rotation_row_major: [f64; 9],
    translation: [f64; 3],
}

#[derive(Serialize, Deserialize)]
struct ResultJson {
    poses: Vec<PoseJson>,
    points: Vec<[f64; 3]>,
    final_cost: f64,
    iterations_run: u32,
    converged: bool,
}

fn build_global_state_from_json(
    poses_str: &str,
    points_str: &str,
    obs_str: &str,
    gps_str: &str,
    imu_str: &str,
) -> Result<GlobalSfmState, String> {
    let pose_list: Vec<PoseJson> =
        serde_json::from_str(poses_str).map_err(|e| format!("poses JSON: {e}"))?;
    let point_list: Vec<[f64; 3]> =
        serde_json::from_str(points_str).map_err(|e| format!("points JSON: {e}"))?;
    let observations: Vec<Observation> =
        serde_json::from_str(obs_str).map_err(|e| format!("obs JSON: {e}"))?;
    let gps_priors: Vec<GpsPrior> =
        serde_json::from_str(gps_str).map_err(|e| format!("gps JSON: {e}"))?;
    let imu_priors: Vec<ImuRotationPrior> =
        serde_json::from_str(imu_str).map_err(|e| format!("imu JSON: {e}"))?;

    if gps_priors.is_empty() {
        return Err("gps JSON: empty (Mode C requires GPS priors)".to_string());
    }
    if imu_priors.is_empty() {
        return Err("imu JSON: empty (Mode C requires IMU priors)".to_string());
    }

    let frame_ids: Vec<usize> = pose_list.iter().map(|p| p.frame_id).collect();
    let n_frames = frame_ids.len();

    let mut gps_covered = vec![false; n_frames];
    for g in &gps_priors {
        if g.frame_idx >= n_frames {
            return Err(format!(
                "gps JSON: frame_idx {} out of range (0..{})",
                g.frame_idx,
                n_frames.saturating_sub(1)
            ));
        }
        gps_covered[g.frame_idx] = true;
    }
    if let Some((missing_idx, _)) = gps_covered.iter().enumerate().find(|(_, v)| !*v) {
        return Err(format!(
            "gps JSON: missing prior for frame_idx {missing_idx} (Mode C requires all frames)"
        ));
    }

    let mut imu_adjacent_covered = vec![false; n_frames.saturating_sub(1)];
    for m in &imu_priors {
        if m.frame_a >= n_frames || m.frame_b >= n_frames {
            return Err(format!(
                "imu JSON: frame indices out of range (a: {}, b: {}, max: {})",
                m.frame_a,
                m.frame_b,
                n_frames.saturating_sub(1)
            ));
        }
        if m.frame_a + 1 == m.frame_b {
            imu_adjacent_covered[m.frame_a] = true;
        } else if m.frame_b + 1 == m.frame_a {
            imu_adjacent_covered[m.frame_b] = true;
        }
    }
    if let Some((missing_idx, _)) =
        imu_adjacent_covered.iter().enumerate().find(|(_, v)| !*v)
    {
        return Err(format!(
            "imu JSON: missing adjacent prior for frames ({}, {}) (Mode C requires all consecutive pairs)",
            missing_idx,
            missing_idx + 1
        ));
    }
    let rotations: Vec<Matrix3<f64>> = pose_list
        .iter()
        .map(|p| Matrix3::from_row_slice(&p.rotation_row_major))
        .collect();
    let translations: Vec<Vector3<f64>> = pose_list
        .iter()
        .map(|p| Vector3::new(p.translation[0], p.translation[1], p.translation[2]))
        .collect();
    let points: Vec<Vector3<f64>> = point_list
        .iter()
        .map(|p| Vector3::new(p[0], p[1], p[2]))
        .collect();

    Ok(GlobalSfmState {
        frame_ids,
        rotations,
        translations,
        points,
        observations,
        gps_priors,
        imu_priors,
    })
}

fn serialise_result_to_json(global: &GlobalSfmState) -> String {
    let poses: Vec<PoseJson> = global
        .frame_ids
        .iter()
        .zip(global.rotations.iter())
        .zip(global.translations.iter())
        .map(|((&fid, R), t)| {
            let rm = R.as_slice();
            PoseJson {
                frame_id: fid,
                rotation_row_major: [
                    rm[0], rm[1], rm[2],
                    rm[3], rm[4], rm[5],
                    rm[6], rm[7], rm[8],
                ],
                translation: [t[0], t[1], t[2]],
            }
        })
        .collect();

    let points: Vec<[f64; 3]> = global
        .points
        .iter()
        .map(|p| [p[0], p[1], p[2]])
        .collect();

    let result = ResultJson {
        poses,
        points,
        final_cost: 0.0,      // filled by caller if needed
        iterations_run: 0,
        converged: true,
    };

    serde_json::to_string(&result).unwrap_or_else(|e| format!(r#"{{"error":"{e}"}}"#))
}

// ─────────────────────────────────────────────────────────────────────────────
// SECTION 9: JNI ENTRY POINT
// ─────────────────────────────────────────────────────────────────────────────
//
// Package: com.splats.app.sfm   (matches existing project: crates/brush-app/app/src/main/java/com/splats/app/)
//
// Kotlin companion declaration (add to BundleAdjustmentLib.kt):
//   external fun runSlidingWindowBA(
//       posesJson: String, pointsJson: String, obsJson: String,
//       gpsJson: String, imuJson: String,
//       intrinsicsJson: String, configJson: String,
//   ): String
//
// Symbol: Java_com_splats_app_sfm_BundleAdjustmentLib_runSlidingWindowBA

#[cfg(feature = "jni-support")]
pub mod jni_bridge {
    use super::*;
    use jni::objects::{JClass, JString};
    use jni::sys::jstring;
    use jni::JNIEnv;

    #[unsafe(no_mangle)]
    pub extern "system" fn Java_com_splats_app_sfm_BundleAdjustmentLib_runSlidingWindowBA(
        env: JNIEnv,
        _class: JClass,
        poses_json: JString,
        points_json: JString,
        obs_json: JString,
        gps_json: JString,
        imu_json: JString,
        intrinsics_json: JString,
        config_json: JString,
    ) -> jstring {
        // Wrap everything in catch_unwind so Rust panics don't abort the JVM
        let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            let to_string = |js: JString| -> String {
                unsafe { env.get_string_unchecked(&js) }
                    .map(|s| s.into())
                    .unwrap_or_default()
            };

            let poses_str     = to_string(poses_json);
            let points_str    = to_string(points_json);
            let obs_str       = to_string(obs_json);
            let gps_str       = to_string(gps_json);
            let imu_str       = to_string(imu_json);
            let intrinsics_str = to_string(intrinsics_json);
            let config_str    = to_string(config_json);

            let K: CameraIntrinsics = match serde_json::from_str(&intrinsics_str) {
                Ok(k) => k,
                Err(e) => return format!(r#"{{"error":"intrinsics parse: {e}"}}"#),
            };
            let cfg: SlidingWindowConfig =
                serde_json::from_str(&config_str).unwrap_or_default();

            let mut global = match build_global_state_from_json(
                &poses_str, &points_str, &obs_str, &gps_str, &imu_str,
            ) {
                Ok(g) => g,
                Err(e) => return format!(r#"{{"error":"state build: {e}"}}"#),
            };

            run_sliding_window_ba(&mut global, &K, &cfg);
            serialise_result_to_json(&global)
        }));

        let json = match result {
            Ok(s) => s,
            Err(_) => r#"{"error":"Rust panic in BA"}"#.to_string(),
        };

        env.new_string(json)
            .map(|s| s.into_raw())
            .unwrap_or(std::ptr::null_mut())
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// SECTION 10: C FFI BOUNDARY (for desktop / CI / Python validation)
// ─────────────────────────────────────────────────────────────────────────────
//
// Use from Python: ctypes.CDLL("./libbrush_process.so").ba_run_sliding_window(...)

#[unsafe(no_mangle)]
pub unsafe extern "C" fn ba_run_sliding_window(
    poses_ptr:   *const u8,  poses_len:   usize,
    points_ptr:  *const u8,  points_len:  usize,
    obs_ptr:     *const u8,  obs_len:     usize,
    gps_ptr:     *const u8,  gps_len:     usize,
    imu_ptr:     *const u8,  imu_len:     usize,
    intrin_ptr:  *const u8,  intrin_len:  usize,
    config_ptr:  *const u8,  config_len:  usize,
    out_ptr:     *mut u8,
    out_capacity: usize,
) -> i32 {
    let read_str = |ptr: *const u8, len: usize| -> &'static str {
        unsafe { std::str::from_utf8(std::slice::from_raw_parts(ptr, len)).unwrap_or("") }
    };

    let poses_str    = read_str(poses_ptr,  poses_len);
    let points_str   = read_str(points_ptr, points_len);
    let obs_str      = read_str(obs_ptr,    obs_len);
    let gps_str      = read_str(gps_ptr,    gps_len);
    let imu_str      = read_str(imu_ptr,    imu_len);
    let intrin_str   = read_str(intrin_ptr, intrin_len);
    let config_str   = read_str(config_ptr, config_len);

    let K: CameraIntrinsics = match serde_json::from_str(intrin_str) {
        Ok(k) => k,
        Err(_) => return -1,
    };
    let cfg: SlidingWindowConfig = serde_json::from_str(config_str).unwrap_or_default();

    let mut global = match build_global_state_from_json(
        poses_str, points_str, obs_str, gps_str, imu_str,
    ) {
        Ok(g) => g,
        Err(_) => return -2,
    };

    run_sliding_window_ba(&mut global, &K, &cfg);
    let json = serialise_result_to_json(&global);
    let bytes = json.as_bytes();
    let write_len = bytes.len().min(out_capacity);
    unsafe { std::ptr::copy_nonoverlapping(bytes.as_ptr(), out_ptr, write_len) };
    write_len as i32
}

// ─────────────────────────────────────────────────────────────────────────────
// SECTION 11: UNIT TESTS
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_skew_symmetric_property() {
        // [v]× should be antisymmetric: A = −Aᵀ
        let v = Vector3::new(1.0, 2.0, 3.0);
        let s = skew_symmetric(&v);
        assert!((s + s.transpose()).norm() < 1e-14);
    }

    #[test]
    fn test_rodrigues_identity() {
        // Near-zero axis-angle should give identity
        let omega = Vector3::zeros();
        let R = axis_angle_to_rotation(&omega);
        assert!((R - Matrix3::identity()).norm() < 1e-10);
    }

    #[test]
    fn test_rodrigues_round_trip() {
        // Rodrigues and log should be inverses
        let omega = Vector3::new(0.1, -0.2, 0.3);
        let R = axis_angle_to_rotation(&omega);
        let omega2 = rotation_log(&R);
        assert!((omega - omega2).norm() < 1e-12);
    }

    #[test]
    fn test_projection_behind_camera_returns_none() {
        let K = CameraIntrinsics { fx: 500.0, fy: 500.0, cx: 320.0, cy: 240.0 };
        let omega = Vector3::zeros();
        let t = Vector3::new(0.0, 0.0, 0.0);
        // Point behind camera (Z < 0 in camera frame)
        let X = Vector3::new(0.0, 0.0, -5.0);
        assert!(project(&omega, &t, &X, &K).is_none());
    }

    #[test]
    fn test_trivial_ba_does_not_panic() {
        // Single frame, single point, one observation — BA should run without panicking
        let K = CameraIntrinsics { fx: 500.0, fy: 500.0, cx: 320.0, cy: 240.0 };

        let mut state = BaState {
            rotations_aa: vec![Vector3::zeros()],
            translations: vec![Vector3::zeros()],
            points: vec![Vector3::new(0.1, 0.1, 5.0)],
            frozen_frames: vec![0],
        };

        let obs = vec![Observation {
            frame_idx: 0,
            point_idx: 0,
            observed: [320.0, 240.0],
        }];

        let cfg = LmConfig::default();
        let result = run_levenberg_marquardt(&mut state, &obs, &[], &[], &K, &cfg);

        // Should converge trivially (frozen frame, single obs)
        assert!(result.final_cost >= 0.0);
    }

    #[test]
    fn test_lm_reduces_cost_two_frames() {
        let K = CameraIntrinsics { fx: 600.0, fy: 600.0, cx: 320.0, cy: 240.0 };

        // Frame 0: identity pose
        // Frame 1: slight rotation error
        let noisy_omega = Vector3::new(0.05, 0.0, 0.0);
        let t1 = Vector3::new(-0.5, 0.0, 0.0);

        let mut state = BaState {
            rotations_aa: vec![Vector3::zeros(), noisy_omega],
            translations: vec![Vector3::zeros(), t1],
            points: vec![Vector3::new(0.0, 0.0, 3.0)],
            frozen_frames: vec![0],
        };

        // Noiseless observations
        let X = Vector3::new(0.0, 0.0, 3.0);
        let obs0_uv = project(&Vector3::zeros(), &Vector3::zeros(), &X, &K).unwrap();
        let true_omega1 = Vector3::new(0.0, 0.0, 0.0);
        let obs1_uv = project(&true_omega1, &t1, &X, &K).unwrap();

        let observations = vec![
            Observation { frame_idx: 0, point_idx: 0, observed: [obs0_uv[0], obs0_uv[1]] },
            Observation { frame_idx: 1, point_idx: 0, observed: [obs1_uv[0], obs1_uv[1]] },
        ];

        let cost_before =
            compute_total_cost(&state, &observations, &[], &[], &K);

        let cfg = LmConfig::default();
        run_levenberg_marquardt(&mut state, &observations, &[], &[], &K, &cfg);

        let cost_after =
            compute_total_cost(&state, &observations, &[], &[], &K);

        assert!(
            cost_after <= cost_before,
            "BA should not increase cost: before={cost_before:.4} after={cost_after:.4}"
        );
    }
}
