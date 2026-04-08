use anyhow::{Context, Result, bail};
use nalgebra::{DMatrix, DVector, Matrix2x3, Matrix3, SMatrix, Vector2, Vector3};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeSet, HashMap};
use std::io::Write;
use std::path::Path;

fn default_weight() -> f64 {
    1.0
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct CameraIntrinsics {
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct Observation {
    #[serde(alias = "frame_index")]
    pub frame_idx: usize,
    #[serde(alias = "point_index")]
    pub point_idx: usize,
    #[serde(alias = "xy")]
    pub observed: [f64; 2],
    #[serde(default = "default_weight")]
    pub weight: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct GpsPrior {
    #[serde(alias = "frame_index")]
    pub frame_idx: usize,
    #[serde(alias = "translation")]
    pub enu_position: [f64; 3],
    #[serde(default = "default_weight")]
    pub weight: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct ImuRotationPrior {
    #[serde(default)]
    pub frame_a: usize,
    #[serde(default)]
    pub frame_b: usize,
    #[serde(default = "identity_rotation")]
    pub delta_rotation: [[f64; 3]; 3],
    #[serde(default = "default_weight")]
    pub weight: f64,
}

fn identity_rotation() -> [[f64; 3]; 3] {
    [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Default)]
pub struct GlobalSfmState {
    #[serde(default)]
    pub frame_ids: Vec<u64>,
    #[serde(default)]
    pub rotations: Vec<[f64; 3]>,
    #[serde(default)]
    pub translations: Vec<[f64; 3]>,
    #[serde(default)]
    pub points: Vec<[f64; 3]>,
    #[serde(default)]
    pub observations: Vec<Observation>,
    #[serde(default)]
    pub gps_priors: Vec<GpsPrior>,
    #[serde(default)]
    pub imu_priors: Vec<ImuRotationPrior>,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Default)]
pub struct BaState {
    #[serde(default)]
    pub rotations_aa: Vec<[f64; 3]>,
    #[serde(default)]
    pub translations: Vec<[f64; 3]>,
    #[serde(default)]
    pub points: Vec<[f64; 3]>,
    #[serde(default)]
    pub frozen_frames: Vec<usize>,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct BaResult {
    pub refined_rotations: Vec<[[f64; 3]; 3]>,
    pub refined_translations: Vec<[f64; 3]>,
    pub refined_points: Vec<[f64; 3]>,
    pub final_cost: f64,
    pub iterations_run: u32,
    pub converged: bool,
    pub rms_reprojection_error: f64,
    pub updated_frames: usize,
    pub updated_points: usize,
}

impl Default for BaResult {
    fn default() -> Self {
        Self {
            refined_rotations: Vec::new(),
            refined_translations: Vec::new(),
            refined_points: Vec::new(),
            final_cost: 0.0,
            iterations_run: 0,
            converged: false,
            rms_reprojection_error: 0.0,
            updated_frames: 0,
            updated_points: 0,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct LmConfig {
    pub max_iterations: u32,
    pub lambda_init: f64,
    pub lambda_max: f64,
    pub lambda_factor_up: f64,
    pub lambda_factor_dn: f64,
    pub delta_threshold: f64,
    pub cost_threshold: f64,
}

impl Default for LmConfig {
    fn default() -> Self {
        Self {
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

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SlidingWindowConfig {
    pub window_size: usize,
    pub min_observations: usize,
    pub freeze_first_frame: bool,
    #[serde(default)]
    pub export_ply_path: Option<String>,
    #[serde(flatten)]
    pub lm: LmConfig,
}

impl Default for SlidingWindowConfig {
    fn default() -> Self {
        Self {
            window_size: 8,
            min_observations: 12,
            freeze_first_frame: true,
            export_ply_path: None,
            lm: LmConfig::default(),
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct PoseJson {
    #[serde(default)]
    frame_id: Option<u64>,
    rotation: [f64; 3],
    translation: [f64; 3],
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct BaJsonEnvelope {
    pub result: BaResult,
    pub global: GlobalSfmState,
}

#[derive(Debug, Clone)]
struct WindowContext {
    frame_mapping: Vec<usize>,
    point_mapping: Vec<usize>,
    state: BaState,
    observations: Vec<Observation>,
    gps_priors: Vec<GpsPrior>,
    imu_priors: Vec<ImuRotationPrior>,
}

#[derive(Debug, Clone)]
struct JacobianBlock {
    j_pose: SMatrix<f64, 2, 6>,
    j_point: Matrix2x3<f64>,
}

#[derive(Debug, Clone)]
struct GpsJacobianBlock {
    j_omega: Matrix3<f64>,
    j_t: Matrix3<f64>,
}

#[derive(Debug, Clone)]
struct ImuJacobianBlock {
    j_omega_a: Matrix3<f64>,
    j_omega_b: Matrix3<f64>,
}

pub fn axis_angle_to_rotation(axis_angle: [f64; 3]) -> [[f64; 3]; 3] {
    let rotation = axis_angle_to_rotation_vec(&vec3_from_array(axis_angle));
    matrix_to_array(&rotation)
}

pub fn rotation_log(rotation: [[f64; 3]; 3]) -> [f64; 3] {
    let matrix = matrix3_from_array(rotation);
    vec3_to_array(&rotation_log_matrix(&matrix))
}

pub fn run_levenberg_marquardt(
    global: &mut GlobalSfmState,
    k: &CameraIntrinsics,
    cfg: &LmConfig,
) -> BaResult {
    if let Err(error) = validate_global_state(global) {
        return error_result(error);
    }

    let state = global_to_ba_state(global, Vec::new());
    let result = run_lm_core(
        &state,
        &global.observations,
        &global.gps_priors,
        &global.imu_priors,
        k,
        cfg,
    );

    let frame_mapping: Vec<usize> = (0..global.rotations.len()).collect();
    let point_mapping: Vec<usize> = (0..global.points.len()).collect();
    apply_state_to_global(global, &state, &frame_mapping, &point_mapping);
    result
}

pub fn run_sliding_window_ba(
    global: &mut GlobalSfmState,
    k: &CameraIntrinsics,
    cfg: &SlidingWindowConfig,
) -> BaResult {
    if let Err(error) = validate_global_state(global) {
        return error_result(error);
    }
    if global.rotations.is_empty() || global.points.is_empty() {
        return BaResult::default();
    }

    let window_size = cfg.window_size.max(2).min(global.rotations.len());
    let mut aggregate = BaResult::default();
    let mut any_window = false;
    let mut start = 0usize;

    while start < global.rotations.len() {
        let end = (start + window_size).min(global.rotations.len());
        let Some(window) = build_window_context(global, start, end, cfg) else {
            if end == global.rotations.len() {
                break;
            }
            start += 1;
            continue;
        };

        any_window = true;
        let result = run_lm_core(
            &window.state,
            &window.observations,
            &window.gps_priors,
            &window.imu_priors,
            k,
            &cfg.lm,
        );

        let final_state = apply_result_state(
            &window.state,
            &window.observations,
            &window.gps_priors,
            &window.imu_priors,
            k,
            &cfg.lm,
        );
        merge_window_back(global, &window.frame_mapping, &window.point_mapping, &final_state);

        aggregate = result;
        aggregate.updated_frames += window.frame_mapping.len();
        aggregate.updated_points += window.point_mapping.len();

        if end == global.rotations.len() {
            break;
        }
        start += window_size.saturating_sub(1).max(1);
    }

    if !any_window {
        let result = BaResult::default();
        maybe_write_ply(global, cfg);
        result
    } else {
        maybe_write_ply(global, cfg);
        aggregate
    }
}

pub fn build_global_state_from_json(
    poses_json: &str,
    points_json: &str,
    obs_json: &str,
    gps_json: &str,
    imu_json: &str,
) -> Result<GlobalSfmState> {
    let poses: Vec<PoseJson> =
        serde_json::from_str(poses_json).context("failed to parse poses json")?;
    let points: Vec<[f64; 3]> =
        serde_json::from_str(points_json).context("failed to parse points json")?;
    let observations: Vec<Observation> =
        serde_json::from_str(obs_json).context("failed to parse observations json")?;
    let gps_priors: Vec<GpsPrior> =
        serde_json::from_str(gps_json).context("failed to parse gps priors json")?;
    let mut imu_priors: Vec<ImuRotationPrior> =
        serde_json::from_str(imu_json).context("failed to parse imu priors json")?;

    let frame_ids = poses
        .iter()
        .enumerate()
        .map(|(index, pose)| pose.frame_id.unwrap_or(index as u64))
        .collect();
    let rotations = poses.iter().map(|pose| pose.rotation).collect();
    let translations = poses.iter().map(|pose| pose.translation).collect();

    for imu in &mut imu_priors {
        if imu.frame_a == 0 && imu.frame_b == 0 && imu.delta_rotation != identity_rotation() {
            imu.frame_b = 1;
        }
    }

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

pub fn serialise_result_to_json(global: &GlobalSfmState) -> String {
    let result = BaResult {
        refined_rotations: global
            .rotations
            .iter()
            .copied()
            .map(axis_angle_to_rotation)
            .collect(),
        refined_translations: global.translations.clone(),
        refined_points: global.points.clone(),
        final_cost: 0.0,
        iterations_run: 0,
        converged: true,
        rms_reprojection_error: 0.0,
        updated_frames: global.rotations.len(),
        updated_points: global.points.len(),
    };

    serde_json::to_string(&BaJsonEnvelope {
        result,
        global: global.clone(),
    })
    .unwrap_or_else(|_| "{\"error\":\"serialise\"}".to_string())
}

pub fn sparse_points_to_ply_bytes(points: &[[f64; 3]]) -> Vec<u8> {
    let mut output = Vec::new();
    writeln!(&mut output, "ply").expect("vec write");
    writeln!(&mut output, "format ascii 1.0").expect("vec write");
    writeln!(&mut output, "comment Exported from brush-sfm").expect("vec write");
    writeln!(&mut output, "element vertex {}", points.len()).expect("vec write");
    writeln!(&mut output, "property float x").expect("vec write");
    writeln!(&mut output, "property float y").expect("vec write");
    writeln!(&mut output, "property float z").expect("vec write");
    writeln!(&mut output, "property uchar red").expect("vec write");
    writeln!(&mut output, "property uchar green").expect("vec write");
    writeln!(&mut output, "property uchar blue").expect("vec write");
    writeln!(&mut output, "end_header").expect("vec write");

    for (index, point) in points.iter().enumerate() {
        let color = pseudo_color(index);
        writeln!(
            &mut output,
            "{:.6} {:.6} {:.6} {} {} {}",
            point[0], point[1], point[2], color[0], color[1], color[2]
        )
        .expect("vec write");
    }

    output
}

pub fn global_state_to_ply_bytes(global: &GlobalSfmState) -> Vec<u8> {
    sparse_points_to_ply_bytes(&global.points)
}

pub fn write_sparse_points_ply(path: impl AsRef<Path>, points: &[[f64; 3]]) -> Result<()> {
    let bytes = sparse_points_to_ply_bytes(points);
    std::fs::write(path.as_ref(), bytes)
        .with_context(|| format!("failed to write PLY to {}", path.as_ref().display()))
}

pub fn write_global_state_ply(path: impl AsRef<Path>, global: &GlobalSfmState) -> Result<()> {
    write_sparse_points_ply(path, &global.points)
}

fn maybe_write_ply(global: &GlobalSfmState, cfg: &SlidingWindowConfig) {
    if let Some(path) = cfg.export_ply_path.as_deref() {
        if let Err(error) = write_global_state_ply(path, global) {
            eprintln!("brush-sfm: failed to write PLY to {path}: {error}");
        }
    }
}

fn run_lm_core(
    initial_state: &BaState,
    observations: &[Observation],
    gps_priors: &[GpsPrior],
    imu_priors: &[ImuRotationPrior],
    k: &CameraIntrinsics,
    config: &LmConfig,
) -> BaResult {
    let mut state = initial_state.clone();
    let mut lambda = config.lambda_init;
    let mut cost = compute_total_cost(&state, observations, gps_priors, imu_priors, k);
    let mut converged = false;
    let mut iterations_run = 0u32;

    for iteration in 0..config.max_iterations {
        iterations_run = iteration + 1;

        let Some((delta_pose, delta_point)) = build_and_solve_schur(
            &state,
            observations,
            gps_priors,
            imu_priors,
            k,
            lambda,
        ) else {
            break;
        };

        let delta_norm = delta_pose.norm() + delta_point.norm();
        if delta_norm < config.delta_threshold {
            converged = true;
            break;
        }

        let candidate = apply_delta(&state, &delta_pose, &delta_point);
        let candidate_cost = compute_total_cost(&candidate, observations, gps_priors, imu_priors, k);

        if candidate_cost < cost {
            state = candidate;
            cost = candidate_cost;
            lambda = (lambda * config.lambda_factor_dn).max(1e-10);
            if observations.is_empty()
                || cost / (observations.len() as f64) < config.cost_threshold
            {
                converged = true;
                break;
            }
        } else {
            lambda *= config.lambda_factor_up;
            if lambda > config.lambda_max {
                break;
            }
        }
    }

    build_result(&state, cost, iterations_run, converged, observations, k)
}

fn apply_result_state(
    initial_state: &BaState,
    observations: &[Observation],
    gps_priors: &[GpsPrior],
    imu_priors: &[ImuRotationPrior],
    k: &CameraIntrinsics,
    config: &LmConfig,
) -> BaState {
    let mut state = initial_state.clone();
    let mut lambda = config.lambda_init;
    let mut cost = compute_total_cost(&state, observations, gps_priors, imu_priors, k);

    for _ in 0..config.max_iterations {
        let Some((delta_pose, delta_point)) = build_and_solve_schur(
            &state,
            observations,
            gps_priors,
            imu_priors,
            k,
            lambda,
        ) else {
            break;
        };
        let candidate = apply_delta(&state, &delta_pose, &delta_point);
        let candidate_cost = compute_total_cost(&candidate, observations, gps_priors, imu_priors, k);
        if candidate_cost < cost {
            state = candidate;
            cost = candidate_cost;
            lambda = (lambda * config.lambda_factor_dn).max(1e-10);
        } else {
            lambda *= config.lambda_factor_up;
            if lambda > config.lambda_max {
                break;
            }
        }
    }

    state
}

fn build_result(
    state: &BaState,
    cost: f64,
    iterations_run: u32,
    converged: bool,
    observations: &[Observation],
    k: &CameraIntrinsics,
) -> BaResult {
    BaResult {
        refined_rotations: state
            .rotations_aa
            .iter()
            .copied()
            .map(axis_angle_to_rotation)
            .collect(),
        refined_translations: state.translations.clone(),
        refined_points: state.points.clone(),
        final_cost: cost,
        iterations_run,
        converged,
        rms_reprojection_error: compute_rms_error(state, observations, k),
        updated_frames: state.rotations_aa.len(),
        updated_points: state.points.len(),
    }
}

fn build_window_context(
    global: &GlobalSfmState,
    start: usize,
    end: usize,
    cfg: &SlidingWindowConfig,
) -> Option<WindowContext> {
    let frame_mapping: Vec<usize> = (start..end).collect();
    let frame_lookup: HashMap<usize, usize> = frame_mapping
        .iter()
        .enumerate()
        .map(|(local, global_idx)| (*global_idx, local))
        .collect();

    let point_set: BTreeSet<usize> = global
        .observations
        .iter()
        .filter(|obs| obs.frame_idx >= start && obs.frame_idx < end)
        .map(|obs| obs.point_idx)
        .collect();
    if point_set.is_empty() {
        return None;
    }

    let point_mapping: Vec<usize> = point_set.into_iter().collect();
    let point_lookup: HashMap<usize, usize> = point_mapping
        .iter()
        .enumerate()
        .map(|(local, global_idx)| (*global_idx, local))
        .collect();

    let observations: Vec<Observation> = global
        .observations
        .iter()
        .filter_map(|obs| {
            let local_frame = frame_lookup.get(&obs.frame_idx)?;
            let local_point = point_lookup.get(&obs.point_idx)?;
            Some(Observation {
                frame_idx: *local_frame,
                point_idx: *local_point,
                observed: obs.observed,
                weight: obs.weight,
            })
        })
        .collect();

    if observations.len() < cfg.min_observations {
        return None;
    }

    let gps_priors: Vec<GpsPrior> = global
        .gps_priors
        .iter()
        .filter_map(|prior| {
            frame_lookup.get(&prior.frame_idx).map(|local_frame| GpsPrior {
                frame_idx: *local_frame,
                enu_position: prior.enu_position,
                weight: prior.weight,
            })
        })
        .collect();

    let imu_priors: Vec<ImuRotationPrior> = global
        .imu_priors
        .iter()
        .filter_map(|prior| {
            let frame_a = frame_lookup.get(&prior.frame_a)?;
            let frame_b = frame_lookup.get(&prior.frame_b)?;
            Some(ImuRotationPrior {
                frame_a: *frame_a,
                frame_b: *frame_b,
                delta_rotation: prior.delta_rotation,
                weight: prior.weight,
            })
        })
        .collect();

    let frozen_frames = if cfg.freeze_first_frame && !frame_mapping.is_empty() {
        vec![0]
    } else {
        Vec::new()
    };

    let state = BaState {
        rotations_aa: frame_mapping
            .iter()
            .map(|idx| global.rotations[*idx])
            .collect(),
        translations: frame_mapping
            .iter()
            .map(|idx| global.translations[*idx])
            .collect(),
        points: point_mapping.iter().map(|idx| global.points[*idx]).collect(),
        frozen_frames,
    };

    Some(WindowContext {
        frame_mapping,
        point_mapping,
        state,
        observations,
        gps_priors,
        imu_priors,
    })
}

fn merge_window_back(
    global: &mut GlobalSfmState,
    frame_mapping: &[usize],
    point_mapping: &[usize],
    local_state: &BaState,
) {
    for (local_idx, global_idx) in frame_mapping.iter().enumerate() {
        global.rotations[*global_idx] = local_state.rotations_aa[local_idx];
        global.translations[*global_idx] = local_state.translations[local_idx];
    }

    for (local_idx, global_idx) in point_mapping.iter().enumerate() {
        global.points[*global_idx] = local_state.points[local_idx];
    }
}

fn global_to_ba_state(global: &GlobalSfmState, frozen_frames: Vec<usize>) -> BaState {
    BaState {
        rotations_aa: global.rotations.clone(),
        translations: global.translations.clone(),
        points: global.points.clone(),
        frozen_frames,
    }
}

fn apply_state_to_global(
    global: &mut GlobalSfmState,
    state: &BaState,
    frame_mapping: &[usize],
    point_mapping: &[usize],
) {
    for (local_idx, global_idx) in frame_mapping.iter().enumerate() {
        global.rotations[*global_idx] = state.rotations_aa[local_idx];
        global.translations[*global_idx] = state.translations[local_idx];
    }
    for (local_idx, global_idx) in point_mapping.iter().enumerate() {
        global.points[*global_idx] = state.points[local_idx];
    }
}

fn validate_global_state(global: &GlobalSfmState) -> Result<()> {
    if global.rotations.len() != global.translations.len() {
        bail!("rotations and translations must have the same length");
    }

    for observation in &global.observations {
        if observation.frame_idx >= global.rotations.len() {
            bail!("observation frame index {} is out of bounds", observation.frame_idx);
        }
        if observation.point_idx >= global.points.len() {
            bail!("observation point index {} is out of bounds", observation.point_idx);
        }
    }

    for gps in &global.gps_priors {
        if gps.frame_idx >= global.rotations.len() {
            bail!("gps frame index {} is out of bounds", gps.frame_idx);
        }
    }

    for imu in &global.imu_priors {
        if imu.frame_a >= global.rotations.len() || imu.frame_b >= global.rotations.len() {
            bail!("imu prior frame index is out of bounds");
        }
    }

    Ok(())
}

fn compute_total_cost(
    state: &BaState,
    observations: &[Observation],
    gps_priors: &[GpsPrior],
    imu_priors: &[ImuRotationPrior],
    k: &CameraIntrinsics,
) -> f64 {
    let mut cost = 0.0;

    for obs in observations {
        if let Some(residual) = compute_residual(state, obs, k) {
            cost += obs.weight * residual.norm_squared();
        }
    }

    for gps in gps_priors {
        let rotation = axis_angle_to_rotation_vec(&vec3_from_array(state.rotations_aa[gps.frame_idx]));
        let translation = vec3_from_array(state.translations[gps.frame_idx]);
        let cam_centre = -(rotation.transpose() * translation);
        let residual = cam_centre - vec3_from_array(gps.enu_position);
        cost += gps.weight * residual.norm_squared();
    }

    for imu in imu_priors {
        let ra = axis_angle_to_rotation_vec(&vec3_from_array(state.rotations_aa[imu.frame_a]));
        let rb = axis_angle_to_rotation_vec(&vec3_from_array(state.rotations_aa[imu.frame_b]));
        let delta_rotation = matrix3_from_array(imu.delta_rotation);
        let relative = delta_rotation.transpose() * ra.transpose() * rb;
        let residual = rotation_log_matrix(&relative);
        cost += imu.weight * residual.norm_squared();
    }

    cost
}

fn compute_residual(
    state: &BaState,
    obs: &Observation,
    k: &CameraIntrinsics,
) -> Option<Vector2<f64>> {
    let omega = vec3_from_array(*state.rotations_aa.get(obs.frame_idx)?);
    let translation = vec3_from_array(*state.translations.get(obs.frame_idx)?);
    let point = vec3_from_array(*state.points.get(obs.point_idx)?);
    let projected = project(&omega, &translation, &point, k)?;
    Some(Vector2::new(obs.observed[0], obs.observed[1]) - projected)
}

fn project(
    omega: &Vector3<f64>,
    translation: &Vector3<f64>,
    point: &Vector3<f64>,
    k: &CameraIntrinsics,
) -> Option<Vector2<f64>> {
    let rotation = axis_angle_to_rotation_vec(omega);
    let x_cam = rotation * point + translation;
    if x_cam[2] <= 1e-6 {
        return None;
    }

    Some(Vector2::new(
        k.fx * x_cam[0] / x_cam[2] + k.cx,
        k.fy * x_cam[1] / x_cam[2] + k.cy,
    ))
}

fn build_and_solve_schur(
    state: &BaState,
    observations: &[Observation],
    gps_priors: &[GpsPrior],
    imu_priors: &[ImuRotationPrior],
    k: &CameraIntrinsics,
    lambda: f64,
) -> Option<(DVector<f64>, DVector<f64>)> {
    let n_frames = state.rotations_aa.len();
    let n_points = state.points.len();
    let pose_dim = n_frames * 6;
    let point_dim = n_points * 3;

    let mut a = DMatrix::<f64>::zeros(pose_dim, pose_dim);
    let mut b = DMatrix::<f64>::zeros(pose_dim, point_dim);
    let mut c_blocks = vec![Matrix3::<f64>::zeros(); n_points];
    let mut e_a = DVector::<f64>::zeros(pose_dim);
    let mut e_b = DVector::<f64>::zeros(point_dim);

    for obs in observations {
        if state.frozen_frames.contains(&obs.frame_idx) {
            continue;
        }

        let omega = vec3_from_array(state.rotations_aa[obs.frame_idx]);
        let translation = vec3_from_array(state.translations[obs.frame_idx]);
        let point = vec3_from_array(state.points[obs.point_idx]);
        let jac = compute_jacobian_block(&omega, &translation, &point, k)?;
        let residual = compute_residual(state, obs, k)? * obs.weight.sqrt();
        let j_pose = jac.j_pose * obs.weight.sqrt();
        let j_point = jac.j_point * obs.weight.sqrt();

        let frame_offset = obs.frame_idx * 6;
        let point_offset = obs.point_idx * 3;

        let jpt_jp = j_pose.transpose() * j_pose;
        let jpt_jx = j_pose.transpose() * j_point;
        let jxt_jx = j_point.transpose() * j_point;

        add_block(&mut a, frame_offset, frame_offset, &jpt_jp);
        add_block_rect(&mut b, frame_offset, point_offset, &jpt_jx);
        c_blocks[obs.point_idx] += jxt_jx;
        add_vec_block(&mut e_a, frame_offset, &(j_pose.transpose() * residual));
        add_vec_block(&mut e_b, point_offset, &(j_point.transpose() * residual));
    }

    for gps in gps_priors {
        if state.frozen_frames.contains(&gps.frame_idx) {
            continue;
        }
        let omega = vec3_from_array(state.rotations_aa[gps.frame_idx]);
        let translation = vec3_from_array(state.translations[gps.frame_idx]);
        let jac = compute_gps_jacobian(&omega, &translation, gps.weight);
        let rotation = axis_angle_to_rotation_vec(&omega);
        let cam_centre = -(rotation.transpose() * translation);
        let residual = (cam_centre - vec3_from_array(gps.enu_position)) * gps.weight.sqrt();

        let mut j_pose = SMatrix::<f64, 3, 6>::zeros();
        j_pose.fixed_columns_mut::<3>(0).copy_from(&jac.j_omega);
        j_pose.fixed_columns_mut::<3>(3).copy_from(&jac.j_t);

        let frame_offset = gps.frame_idx * 6;
        add_block(&mut a, frame_offset, frame_offset, &(j_pose.transpose() * j_pose));
        add_vec_block(&mut e_a, frame_offset, &(j_pose.transpose() * residual));
    }

    for imu in imu_priors {
        if state.frozen_frames.contains(&imu.frame_a) && state.frozen_frames.contains(&imu.frame_b) {
            continue;
        }
        let ra = axis_angle_to_rotation_vec(&vec3_from_array(state.rotations_aa[imu.frame_a]));
        let rb = axis_angle_to_rotation_vec(&vec3_from_array(state.rotations_aa[imu.frame_b]));
        let delta_rotation = matrix3_from_array(imu.delta_rotation);
        let r_err = delta_rotation.transpose() * ra.transpose() * rb;
        let residual = rotation_log_matrix(&r_err) * imu.weight.sqrt();
        let jac = compute_imu_jacobian(imu.weight);

        let frame_a = imu.frame_a * 6;
        let frame_b = imu.frame_b * 6;

        add_block_3(&mut a, frame_a, frame_a, &(jac.j_omega_a.transpose() * jac.j_omega_a));
        add_block_3(&mut a, frame_b, frame_b, &(jac.j_omega_b.transpose() * jac.j_omega_b));
        add_block_3(&mut a, frame_a, frame_b, &(jac.j_omega_a.transpose() * jac.j_omega_b));
        add_block_3(&mut a, frame_b, frame_a, &(jac.j_omega_b.transpose() * jac.j_omega_a));
        add_vec_block(&mut e_a, frame_a, &(jac.j_omega_a.transpose() * residual));
        add_vec_block(&mut e_a, frame_b, &(jac.j_omega_b.transpose() * residual));
    }

    for i in 0..pose_dim {
        a[(i, i)] += lambda * a[(i, i)].abs().max(1e-10);
    }

    let mut c_inv_blocks = Vec::with_capacity(n_points);
    for block in &c_blocks {
        let augmented = *block
            + lambda
                * Matrix3::<f64>::from_diagonal(&block.diagonal().map(|value| value.abs().max(1e-10)));
        c_inv_blocks.push(augmented.try_inverse().unwrap_or_else(Matrix3::<f64>::zeros));
    }

    let mut s = a.clone();
    let mut rhs_reduced = e_a.clone();

    for (point_idx, c_inv) in c_inv_blocks.iter().enumerate() {
        let point_offset = point_idx * 3;
        let b_col = b.columns(point_offset, 3).into_owned();
        let bc_inv = &b_col * c_inv;
        s -= &bc_inv * b_col.transpose();
        let e_b_block = e_b.rows(point_offset, 3).into_owned();
        rhs_reduced -= bc_inv * e_b_block;
    }

    let delta_pose = if let Some(chol) = s.clone().cholesky() {
        chol.solve(&rhs_reduced)
    } else if let Some(solution) = s.clone().lu().solve(&rhs_reduced) {
        solution
    } else {
        return None;
    };

    let mut delta_point = DVector::<f64>::zeros(point_dim);
    for (point_idx, c_inv) in c_inv_blocks.iter().enumerate() {
        let point_offset = point_idx * 3;
        let b_col = b.columns(point_offset, 3).into_owned();
        let e_b_block = e_b.rows(point_offset, 3).into_owned();
        let rhs = e_b_block - b_col.transpose() * &delta_pose;
        let solution = c_inv * rhs;
        delta_point.rows_mut(point_offset, 3).copy_from(&solution);
    }

    Some((delta_pose, delta_point))
}

fn apply_delta(
    state: &BaState,
    delta_pose: &DVector<f64>,
    delta_point: &DVector<f64>,
) -> BaState {
    let mut next = state.clone();

    for frame_idx in 0..next.rotations_aa.len() {
        if state.frozen_frames.contains(&frame_idx) {
            continue;
        }
        for axis in 0..3 {
            next.rotations_aa[frame_idx][axis] += delta_pose[frame_idx * 6 + axis];
            next.translations[frame_idx][axis] += delta_pose[frame_idx * 6 + 3 + axis];
        }
    }

    for point_idx in 0..next.points.len() {
        for axis in 0..3 {
            next.points[point_idx][axis] += delta_point[point_idx * 3 + axis];
        }
    }

    next
}

fn compute_jacobian_block(
    omega: &Vector3<f64>,
    translation: &Vector3<f64>,
    point: &Vector3<f64>,
    k: &CameraIntrinsics,
) -> Option<JacobianBlock> {
    let rotation = axis_angle_to_rotation_vec(omega);
    let x_cam = rotation * point + translation;
    let u = x_cam[0];
    let v = x_cam[1];
    let z = x_cam[2];
    if z <= 1e-6 {
        return None;
    }

    let d_proj_d_xcam = Matrix2x3::new(
        k.fx / z,
        0.0,
        -k.fx * u / (z * z),
        0.0,
        k.fy / z,
        -k.fy * v / (z * z),
    );
    let neg_skew_xcam = -skew_symmetric(&x_cam);
    let d_proj_d_omega = d_proj_d_xcam * neg_skew_xcam;
    let d_proj_d_t = d_proj_d_xcam;

    let mut j_pose = SMatrix::<f64, 2, 6>::zeros();
    j_pose
        .fixed_columns_mut::<3>(0)
        .copy_from(&(-d_proj_d_omega));
    j_pose
        .fixed_columns_mut::<3>(3)
        .copy_from(&(-d_proj_d_t));

    let j_point = -(d_proj_d_xcam * rotation);
    Some(JacobianBlock { j_pose, j_point })
}

fn compute_gps_jacobian(
    omega: &Vector3<f64>,
    translation: &Vector3<f64>,
    weight: f64,
) -> GpsJacobianBlock {
    let rotation = axis_angle_to_rotation_vec(omega);
    let scale = weight.sqrt();
    GpsJacobianBlock {
        j_omega: -skew_symmetric(translation) * scale,
        j_t: -rotation.transpose() * scale,
    }
}

fn compute_imu_jacobian(weight: f64) -> ImuJacobianBlock {
    let scale = weight.sqrt();
    ImuJacobianBlock {
        j_omega_a: -scale * Matrix3::<f64>::identity(),
        j_omega_b: scale * Matrix3::<f64>::identity(),
    }
}

fn compute_rms_error(state: &BaState, observations: &[Observation], k: &CameraIntrinsics) -> f64 {
    let mut squared_sum = 0.0;
    let mut count = 0usize;
    for observation in observations {
        if let Some(residual) = compute_residual(state, observation, k) {
            squared_sum += residual.norm_squared();
            count += 1;
        }
    }
    if count == 0 {
        0.0
    } else {
        (squared_sum / count as f64).sqrt()
    }
}

fn axis_angle_to_rotation_vec(omega: &Vector3<f64>) -> Matrix3<f64> {
    let theta = omega.norm();
    if theta < 1e-10 {
        return Matrix3::<f64>::identity() + skew_symmetric(omega);
    }

    let n = omega / theta;
    let skew = skew_symmetric(&n);
    Matrix3::<f64>::identity() + theta.sin() * skew + (1.0 - theta.cos()) * (skew * skew)
}

fn rotation_log_matrix(rotation: &Matrix3<f64>) -> Vector3<f64> {
    let cos_theta = ((rotation.trace() - 1.0) / 2.0).clamp(-1.0, 1.0);
    let theta = cos_theta.acos();
    if theta < 1e-10 {
        return Vector3::new(
            (rotation[(2, 1)] - rotation[(1, 2)]) / 2.0,
            (rotation[(0, 2)] - rotation[(2, 0)]) / 2.0,
            (rotation[(1, 0)] - rotation[(0, 1)]) / 2.0,
        );
    }
    let factor = theta / (2.0 * theta.sin());
    Vector3::new(
        factor * (rotation[(2, 1)] - rotation[(1, 2)]),
        factor * (rotation[(0, 2)] - rotation[(2, 0)]),
        factor * (rotation[(1, 0)] - rotation[(0, 1)]),
    )
}

fn skew_symmetric(v: &Vector3<f64>) -> Matrix3<f64> {
    Matrix3::new(
        0.0, -v[2], v[1], v[2], 0.0, -v[0], -v[1], v[0], 0.0,
    )
}

fn vec3_from_array(value: [f64; 3]) -> Vector3<f64> {
    Vector3::new(value[0], value[1], value[2])
}

fn vec3_to_array(value: &Vector3<f64>) -> [f64; 3] {
    [value[0], value[1], value[2]]
}

fn matrix3_from_array(value: [[f64; 3]; 3]) -> Matrix3<f64> {
    Matrix3::new(
        value[0][0],
        value[0][1],
        value[0][2],
        value[1][0],
        value[1][1],
        value[1][2],
        value[2][0],
        value[2][1],
        value[2][2],
    )
}

fn matrix_to_array(matrix: &Matrix3<f64>) -> [[f64; 3]; 3] {
    [
        [matrix[(0, 0)], matrix[(0, 1)], matrix[(0, 2)]],
        [matrix[(1, 0)], matrix[(1, 1)], matrix[(1, 2)]],
        [matrix[(2, 0)], matrix[(2, 1)], matrix[(2, 2)]],
    ]
}

fn pseudo_color(index: usize) -> [u8; 3] {
    let base = index as u32;
    [
        ((base.wrapping_mul(53) + 80) % 255) as u8,
        ((base.wrapping_mul(97) + 120) % 255) as u8,
        ((base.wrapping_mul(193) + 160) % 255) as u8,
    ]
}

fn add_block(matrix: &mut DMatrix<f64>, row: usize, col: usize, block: &SMatrix<f64, 6, 6>) {
    for r in 0..6 {
        for c in 0..6 {
            matrix[(row + r, col + c)] += block[(r, c)];
        }
    }
}

fn add_block_rect(matrix: &mut DMatrix<f64>, row: usize, col: usize, block: &SMatrix<f64, 6, 3>) {
    for r in 0..6 {
        for c in 0..3 {
            matrix[(row + r, col + c)] += block[(r, c)];
        }
    }
}

fn add_block_3(matrix: &mut DMatrix<f64>, row: usize, col: usize, block: &Matrix3<f64>) {
    for r in 0..3 {
        for c in 0..3 {
            matrix[(row + r, col + c)] += block[(r, c)];
        }
    }
}

fn add_vec_block(vector: &mut DVector<f64>, offset: usize, block: &impl AsRef<[f64]>) {
    for (index, value) in block.as_ref().iter().enumerate() {
        vector[offset + index] += *value;
    }
}

fn error_result(error: anyhow::Error) -> BaResult {
    BaResult {
        converged: false,
        final_cost: f64::INFINITY,
        rms_reprojection_error: f64::INFINITY,
        iterations_run: 0,
        refined_rotations: Vec::new(),
        refined_translations: Vec::new(),
        refined_points: Vec::new(),
        updated_frames: 0,
        updated_points: 0,
    }
    .tap(|_| {
        let _ = error;
    })
}

trait Tap: Sized {
    fn tap<F: FnOnce(&Self)>(self, f: F) -> Self {
        f(&self);
        self
    }
}

impl<T> Tap for T {}

#[cfg(feature = "jni-support")]
pub mod jni_bridge {
    use super::*;
    use jni::JNIEnv;
    use jni::objects::{JClass, JString};
    use jni::sys::jstring;

    #[unsafe(no_mangle)]
    pub extern "system" fn Java_com_splats_app_sfm_BundleAdjustmentLib_runSlidingWindowBA(
        mut env: JNIEnv<'_>,
        _class: JClass<'_>,
        poses_json: JString<'_>,
        points_json: JString<'_>,
        obs_json: JString<'_>,
        gps_json: JString<'_>,
        imu_json: JString<'_>,
        intrinsics_json: JString<'_>,
        config_json: JString<'_>,
    ) -> jstring {
        let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            let mut to_string = |value: JString<'_>| -> String {
                env.get_string(&value)
                    .map(|s| s.into())
                    .unwrap_or_default()
            };

            let poses_str = to_string(poses_json);
            let points_str = to_string(points_json);
            let obs_str = to_string(obs_json);
            let gps_str = to_string(gps_json);
            let imu_str = to_string(imu_json);
            let intrinsics_str = to_string(intrinsics_json);
            let config_str = to_string(config_json);

            let intrinsics: CameraIntrinsics = match serde_json::from_str(&intrinsics_str) {
                Ok(value) => value,
                Err(error) => return format!(r#"{{"error":"intrinsics parse: {error}"}}"#),
            };

            let config: SlidingWindowConfig =
                serde_json::from_str(&config_str).unwrap_or_default();

            let mut global = match build_global_state_from_json(
                &poses_str,
                &points_str,
                &obs_str,
                &gps_str,
                &imu_str,
            ) {
                Ok(value) => value,
                Err(error) => return format!(r#"{{"error":"state build: {error}"}}"#),
            };

            let result = run_sliding_window_ba(&mut global, &intrinsics, &config);
            serde_json::to_string(&BaJsonEnvelope { result, global })
                .unwrap_or_else(|_| r#"{"error":"serialise"}"#.to_string())
        }));

        let json = match result {
            Ok(value) => value,
            Err(_) => r#"{"error":"Rust panic in BA"}"#.to_string(),
        };

        env.new_string(json)
            .map(|value| value.into_raw())
            .unwrap_or(std::ptr::null_mut())
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn brush_sfm_abi_version() -> u32 {
    1
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample_intrinsics() -> CameraIntrinsics {
        CameraIntrinsics {
            fx: 600.0,
            fy: 600.0,
            cx: 320.0,
            cy: 240.0,
        }
    }

    fn sample_global_state() -> GlobalSfmState {
        GlobalSfmState {
            frame_ids: vec![0, 1],
            rotations: vec![[0.0, 0.0, 0.0], [0.0, 0.01, 0.0]],
            translations: vec![[0.0, 0.0, 0.0], [0.1, 0.0, 0.0]],
            points: vec![[0.2, 0.1, 3.0], [0.4, -0.2, 4.0]],
            observations: vec![
                Observation {
                    frame_idx: 0,
                    point_idx: 0,
                    observed: [360.0, 260.0],
                    weight: 1.0,
                },
                Observation {
                    frame_idx: 1,
                    point_idx: 1,
                    observed: [380.0, 220.0],
                    weight: 1.0,
                },
            ],
            gps_priors: vec![GpsPrior {
                frame_idx: 1,
                enu_position: [0.2, 0.0, 0.0],
                weight: 0.1,
            }],
            imu_priors: vec![ImuRotationPrior {
                frame_a: 0,
                frame_b: 1,
                delta_rotation: identity_rotation(),
                weight: 0.1,
            }],
        }
    }

    #[test]
    fn axis_angle_round_trip_is_stable() {
        let axis_angle = [0.0, 0.2, 0.0];
        let rotation = axis_angle_to_rotation(axis_angle);
        let recovered = rotation_log(rotation);
        assert!((recovered[1] - axis_angle[1]).abs() < 1e-6);
    }

    #[test]
    fn global_state_from_json_uses_aliases() {
        let state = build_global_state_from_json(
            r#"[{"frame_id":3,"rotation":[0.0,0.0,0.0],"translation":[0.0,0.0,0.0]}]"#,
            r#"[[0.0,0.0,3.0]]"#,
            r#"[{"frame_index":0,"point_index":0,"xy":[1.0,2.0],"weight":1.0}]"#,
            r#"[{"frame_index":0,"translation":[0.0,0.0,0.0],"weight":1.0}]"#,
            r#"[{"frame_a":0,"frame_b":0,"delta_rotation":[[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]],"weight":1.0}]"#,
        )
        .expect("json should parse");
        assert_eq!(state.frame_ids, vec![3]);
        assert_eq!(state.observations[0].frame_idx, 0);
    }

    #[test]
    fn sliding_window_runs_with_doc_shaped_state() {
        let mut global = sample_global_state();
        let result = run_sliding_window_ba(&mut global, &sample_intrinsics(), &SlidingWindowConfig::default());
        assert!(result.updated_frames > 0);
        assert!(result.refined_rotations.len() <= global.rotations.len());
    }

    #[test]
    fn sliding_window_can_auto_write_ply() {
        let mut global = sample_global_state();
        let out = std::env::temp_dir().join("brush_sfm_auto_export_test.ply");
        let mut cfg = SlidingWindowConfig::default();
        cfg.export_ply_path = Some(out.display().to_string());

        let _ = run_sliding_window_ba(&mut global, &sample_intrinsics(), &cfg);

        assert!(out.exists());
        let content = std::fs::read_to_string(&out).expect("ply should be readable");
        assert!(content.contains("ply"));
        let _ = std::fs::remove_file(out);
    }

    #[test]
    fn lm_runs_on_full_state() {
        let mut global = sample_global_state();
        let result = run_levenberg_marquardt(&mut global, &sample_intrinsics(), &LmConfig::default());
        assert!(result.updated_points > 0);
    }

    #[test]
    fn sparse_points_export_produces_ply_header() {
        let ply = sparse_points_to_ply_bytes(&[[0.0, 1.0, 2.0], [3.0, 4.0, 5.0]]);
        let text = String::from_utf8(ply).expect("valid utf8");
        assert!(text.contains("ply"));
        assert!(text.contains("element vertex 2"));
        assert!(text.contains("0.000000 1.000000 2.000000"));
    }
}
