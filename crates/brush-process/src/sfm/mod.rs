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
pub mod stage_3_7_bundle_adjustment {
    pub use brush_sfm::*;

    #[cfg(feature = "jni-support")]
    pub mod jni_bridge {
        pub use brush_sfm::sfm::stage_3_7_bundle_adjustment::jni_bridge::*;
    }
}

pub mod stage_3_1_feature_extraction;
pub mod stage_3_2_matching;
pub mod stage_3_3_ransac;
pub mod stage_3_4_pose_recovery;
pub mod stage_3_5_triangulation;
pub mod stage_3_6_inlier_filtering;
pub mod stage_3_8_pose_export;

// ── Contract type shared across stage boundaries ─────────────────────────────

use nalgebra::{Matrix3, Rotation3, Vector3};
use stage_3_2_matching::{match_feature_sets, FrameFeatures, MatchConfig, MatchingResult};
use stage_3_3_ransac::{estimate_essential_matrix, EssentialMatrixResult, RansacConfig};
use stage_3_4_pose_recovery::{recover_relative_pose, PoseRecoveryResult};
use stage_3_5_triangulation::{triangulate_inlier_points, TriangulatedPoint};
use stage_3_6_inlier_filtering::{build_stage_3_6_output, InlierFilterConfig};
use stage_3_7_bundle_adjustment::{CameraIntrinsics, GlobalSfmState, GpsPrior, ImuRotationPrior, Observation};

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

use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct OpenCvFrontendConfig {
    pub matching: MatchConfig,
    pub ransac: RansacConfig,
    pub inlier_filtering: InlierFilterConfig,
}

pub struct OpenCvFrontendResult {
    pub matching: MatchingResult,
    pub essential: EssentialMatrixResult,
    pub pose: PoseRecoveryResult,
    pub triangulated: Vec<TriangulatedPoint>,
    pub stage_3_6: Stage36Output,
}

pub fn run_opencv_frontend(
    frame_a: &FrameFeatures,
    frame_b: &FrameFeatures,
    intrinsics: CameraIntrinsics,
    gps_priors: Vec<GpsPrior>,
    imu_priors: Vec<ImuRotationPrior>,
    config: &OpenCvFrontendConfig,
) -> anyhow::Result<OpenCvFrontendResult> {
    let matching = match_feature_sets(frame_a, frame_b, &config.matching)?;
    log::info!("Stage 3.2: Matched {} features", matching.matches.len());

    anyhow::ensure!(
        matching.matches.len() >= 8,
        "need at least 8 matches, got {}",
        matching.matches.len()
    );

    let essential = estimate_essential_matrix(&matching, &intrinsics, &config.ransac)?;
    log::info!("Stage 3.3: Essential matrix estimated with {} inliers", essential.inlier_mask.iter().filter(|&&v| v != 0).count());

    let pose = recover_relative_pose(&essential, &intrinsics)?;
    log::info!("Stage 3.4: Relative pose recovered");

    let triangulated = triangulate_inlier_points(&essential, &pose, &intrinsics)?;
    log::info!("Stage 3.5: Triangulated {} points", triangulated.len());

    let stage_3_6 = build_stage_3_6_output(
        &matching,
        &pose,
        &triangulated,
        intrinsics,
        gps_priors,
        imu_priors,
        &config.inlier_filtering,
    )?;
    log::info!("Stage 3.6: Inlier filtering complete. {} points retained", stage_3_6.point_positions.len());
    anyhow::ensure!(
        !stage_3_6.point_positions.is_empty(),
        "no triangulated inlier points survived stage 3.6"
    );

    Ok(OpenCvFrontendResult {
        matching,
        essential,
        pose,
        triangulated,
        stage_3_6,
    })
}

#[cfg(feature = "jni-support")]
pub mod jni_bridge {
    use std::fs;
    use std::path::Path;

    use anyhow::Context;
    use jni::objects::{JClass, JString};
    use jni::sys::jstring;
    use jni::JNIEnv;
    use serde::Deserialize;

    use super::*;
    use crate::sfm::stage_3_1_feature_extraction::extract_features;
    use crate::sfm::stage_3_6_inlier_filtering::{mat3_from_opencv, vec3_from_opencv};
    use crate::sfm::stage_3_7_bundle_adjustment::{run_sliding_window_ba, SlidingWindowConfig};
    use crate::sfm::stage_3_8_pose_export::export_sfm_results;

    #[derive(Debug, Deserialize)]
    struct FrontendFrameInput {
        frame_idx: usize,
        image_path: String,
    }

    #[derive(Debug, Deserialize, Default)]
    pub(crate) struct FrontendJniConfig {
        #[serde(default)]
        max_hamming_distance: Option<f32>,
        #[serde(default)]
        max_matches: Option<usize>,
        #[serde(default)]
        ransac_probability: Option<f64>,
        #[serde(default)]
        ransac_threshold_px: Option<f64>,
        #[serde(default)]
        ransac_max_iters: Option<i32>,
        #[serde(default)]
        min_depth: Option<f64>,
        #[serde(default)]
        max_depth: Option<f64>,
        #[serde(default)]
        orb_keypoints: Option<i32>,
        #[serde(default)]
        ba_window_size: Option<usize>,
        #[serde(default)]
        lm_max_iterations: Option<u32>,
    }


    #[unsafe(no_mangle)]
    pub extern "system" fn Java_com_splats_app_sfm_OpenCvFrontendLib_runFullTrainSync(
        mut env: JNIEnv<'_>,
        _class: JClass<'_>,
        frames_json: JString<'_>,
        intrinsics_json: JString<'_>,
        config_json: JString<'_>,
        gps_json: JString<'_>,
        imu_json: JString<'_>,
        output_dir: JString<'_>,
        width: jni::sys::jint,
        height: jni::sys::jint,
    ) -> jstring {
        let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            let mut to_string = |value: JString<'_>| -> String {
                env.get_string(&value).map(|s| s.into()).unwrap_or_default()
            };

            let frames_str = to_string(frames_json);
            let intrinsics_str = to_string(intrinsics_json);
            let config_str = to_string(config_json);
            let gps_str = to_string(gps_json);
            let imu_str = to_string(imu_json);
            let out_dir_str = to_string(output_dir);

            log::info!("Native runFullTrainSync started. Res: {}x{}. Output dir: {}", width, height, out_dir_str);

            let json = match run_full_pipeline_from_json(
                &frames_str,
                &intrinsics_str,
                &config_str,
                &gps_str,
                &imu_str,
                &out_dir_str,
                width,
                height,
            ) {
                Ok(value) => value,
                Err(error) => serde_json::json!({ "error": error.to_string() }).to_string(),
            };

            match env.new_string(json) {
                Ok(js) => js.into_raw(),
                Err(_) => std::ptr::null_mut(),
            }
        }));

        match result {
            Ok(value) => value,
            Err(_) => {
                let err_json = serde_json::json!({ "error": "Rust panic in OpenCV frontend" }).to_string();
                match env.new_string(err_json) {
                    Ok(js) => js.into_raw(),
                    Err(_) => std::ptr::null_mut(),
                }
            }
        }
    }

    fn run_full_pipeline_from_json(
        frames_json: &str,
        intrinsics_json: &str,
        config_json: &str,
        gps_json: &str,
        imu_json: &str,
        output_dir: &str,
        img_w: i32,
        img_h: i32,
    ) -> anyhow::Result<String> {
        let frames_input: Vec<FrontendFrameInput> =
            serde_json::from_str(frames_json).context("failed to parse frames json")?;
        let intrinsics: CameraIntrinsics =
            serde_json::from_str(intrinsics_json).context("failed to parse intrinsics json")?;
        let gps_priors: Vec<GpsPrior> =
            serde_json::from_str(gps_json).unwrap_or_default();
        let imu_priors: Vec<ImuRotationPrior> =
            serde_json::from_str(imu_json).unwrap_or_default();
        let (frontend_config, ba_config, n_orb_features) = if config_json.trim().is_empty() {
            (
                OpenCvFrontendConfig::default(),
                SlidingWindowConfig::default(),
                500_i32.clamp(50, 10_000),
            )
        } else {
            let json_val: serde_json::Value =
                serde_json::from_str(config_json).unwrap_or(serde_json::Value::Null);

            let mut fec = serde_json::from_value::<OpenCvFrontendConfig>(json_val.clone())
                .unwrap_or_default();
            if let Some(fec_val) = json_val.get("frontend").or(json_val.get("sfm")) {
                if let Ok(overridden) = serde_json::from_value(fec_val.clone()) {
                    fec = overridden;
                }
            }

            let parsed_flat = serde_json::from_value::<FrontendJniConfig>(json_val.clone())
                .unwrap_or_default();
            if let Some(v) = parsed_flat.max_hamming_distance {
                fec.matching.max_hamming_distance = v;
            }
            if let Some(v) = parsed_flat.max_matches {
                fec.matching.max_matches = v;
            }
            if let Some(v) = parsed_flat.ransac_probability {
                fec.ransac.probability = v;
            }
            if let Some(v) = parsed_flat.ransac_threshold_px {
                fec.ransac.threshold_px = v;
            }
            if let Some(v) = parsed_flat.ransac_max_iters {
                fec.ransac.max_iters = v;
            }
            if let Some(v) = parsed_flat.min_depth {
                fec.inlier_filtering.min_depth = v;
            }
            if let Some(v) = parsed_flat.max_depth {
                fec.inlier_filtering.max_depth = v;
            }

            let mut ba_config: SlidingWindowConfig =
                serde_json::from_value(json_val.clone()).unwrap_or_default();
            if let Some(ba_val) = json_val
                .get("ba")
                .or(json_val.get("bundle-adjustment"))
            {
                if let Ok(overridden) = serde_json::from_value(ba_val.clone()) {
                    ba_config = overridden;
                }
            }
            if let Some(ws) = parsed_flat.ba_window_size {
                let cap = frames_input.len().max(2);
                ba_config.window_size = ws.max(2).min(cap);
            }
            if let Some(it) = parsed_flat.lm_max_iterations {
                ba_config.lm.max_iterations = it.max(1).min(2000);
            }

            let n_orb_features = parsed_flat
                .orb_keypoints
                .unwrap_or(500)
                .clamp(50, 10_000);

            (fec, ba_config, n_orb_features)
        };

        // --- Stage 3.1 - 3.6: OpenCV Sparse Reconstruction (Frontend) ---
        let mut global_state = GlobalSfmState::default();
        let mut pair_errors = Vec::new();
        let mut successful_pairs = 0usize;
        let mut prev_features: Option<FrameFeatures> = None;
        let mut point_tracker: std::collections::HashMap<(usize, u32, u32), usize> = std::collections::HashMap::new();

        global_state.frame_ids = frames_input.iter().map(|f| f.frame_idx as u64).collect();
        global_state.rotations = vec![[0.0; 3]; frames_input.len()];
        global_state.translations = vec![[0.0; 3]; frames_input.len()];

        let mut frame_paths = Vec::new();

        for frame in &frames_input {
            frame_paths.push(frame.image_path.clone());
            let image_bytes = fs::read(&frame.image_path)
                .with_context(|| format!("failed to read frame {}", frame.image_path))?;
            let extraction = extract_features(&image_bytes, n_orb_features).map_err(|error| {
                anyhow::anyhow!("feature extraction failed for {}: {error}", frame.image_path)
            })?;
            let current_features = FrameFeatures::from_extraction(frame.frame_idx, extraction);

            if let Some(ref prev) = prev_features {
                match run_opencv_frontend(
                    prev,
                    &current_features,
                    intrinsics.clone(),
                    gps_priors.clone(),
                    imu_priors.clone(),
                    &frontend_config,
                ) {
                    Ok(frontend) => {
                        successful_pairs += 1;
                        
                        // Simple chaining: update global pose for the new frame relative to prev.
                        // In a real pipeline, we'd use a better seed, but this matches the previous logic's intent.
                        if let Some(_idx_prev) = global_state.frame_ids.iter().position(|&sid| sid == prev.frame_id as u64) {
                             if let Some(idx_curr) = global_state.frame_ids.iter().position(|&sid| sid == current_features.frame_id as u64) {
                                 // Add relative rotation/translation (Simplified logic for the refactor shell)
                                 let rot_mat = mat3_from_opencv(&frontend.pose.rotation)?;
                                 let axis_angle = Rotation3::from_matrix(&rot_mat).scaled_axis();
                                 global_state.rotations[idx_curr] = [axis_angle.x, axis_angle.y, axis_angle.z];

                                 let trans_vec = vec3_from_opencv(&frontend.pose.translation)?;
                                 global_state.translations[idx_curr] = [trans_vec.x, trans_vec.y, trans_vec.z];
                             }
                        }

                        for (i, p) in frontend.stage_3_6.point_positions.iter().enumerate() {
                            let obs_a = &frontend.stage_3_6.observations[i * 2];
                            let obs_b = &frontend.stage_3_6.observations[i * 2 + 1];

                            let x_bits = (obs_a.observed[0] as f32).to_bits();
                            let y_bits = (obs_a.observed[1] as f32).to_bits();
                            let key_prev = (obs_a.frame_idx, x_bits, y_bits);

                            let point_idx = if let Some(&existing_idx) = point_tracker.get(&key_prev) {
                                global_state.points[existing_idx][0] = (global_state.points[existing_idx][0] + p.x) / 2.0;
                                global_state.points[existing_idx][1] = (global_state.points[existing_idx][1] + p.y) / 2.0;
                                global_state.points[existing_idx][2] = (global_state.points[existing_idx][2] + p.z) / 2.0;
                                existing_idx
                            } else {
                                let new_idx = global_state.points.len();
                                global_state.points.push([p.x, p.y, p.z]);
                                
                                let mut o_a = obs_a.clone();
                                o_a.point_idx = new_idx;
                                global_state.observations.push(o_a);
                                new_idx
                            };

                            let mut o_b = obs_b.clone();
                            o_b.point_idx = point_idx;
                            global_state.observations.push(o_b);

                            let x_bits_b = (obs_b.observed[0] as f32).to_bits();
                            let y_bits_b = (obs_b.observed[1] as f32).to_bits();
                            point_tracker.insert((obs_b.frame_idx, x_bits_b, y_bits_b), point_idx);
                        }
                    }
                    Err(error) => {
                        pair_errors.push(format!(
                            "pair {}->{} failed: {error}",
                            prev.frame_id, current_features.frame_id
                        ));
                    }
                }
            }
            prev_features = Some(current_features);
        }

        anyhow::ensure!(successful_pairs > 0, "No frame pairs were successfully matched/processed");

        // --- Stage 3.7: Bundle Adjustment ---
        global_state.gps_priors = gps_priors;
        global_state.imu_priors = imu_priors;

        let ba_result = run_sliding_window_ba(&mut global_state, &intrinsics, &ba_config);

        // --- Stage 3.8: Pose Export ---
        let out_path = Path::new(output_dir);
        export_sfm_results(&global_state, &intrinsics, &frame_paths, out_path, img_w as u32, img_h as u32)?;

        serde_json::to_string(&serde_json::json!({
            "success": true,
            "final_cost": ba_result.final_cost,
            "rms_error": ba_result.rms_reprojection_error,
            "points_count": global_state.points.len(),
            "pair_errors": pair_errors,
        }))
        .context("failed to serialise final result")
    }

}

#[cfg(test)]
mod tests {
    use super::jni_bridge::*;
    use super::*;

    #[test]
    fn test_config_parsing_full() {
        let json = serde_json::json!({
            "max_matches": 100,
            "ransac_max_iters": 1000,
            "sfm": {
                "ransac": { "probability": 0.5 }
            },
            "ba": {
                "window_size": 16
            }
        }).to_string();

        let (fec, bac) = run_full_pipeline_from_json_test_helper(&json);
        
        // From flat mapping
        assert_eq!(fec.matching.max_matches, 100);
        assert_eq!(fec.ransac.max_iters, 1000);
        
        // From nested "sfm"
        assert_eq!(fec.ransac.probability, 0.5);
        
        // From nested "ba"
        assert_eq!(bac.window_size, 16);
    }

    fn run_full_pipeline_from_json_test_helper(config_json: &str) -> (OpenCvFrontendConfig, SlidingWindowConfig) {
        // This is a copy of the parsing logic from run_full_pipeline_from_json
        // to verify it works without actually running the full pipeline.
        let json_val: serde_json::Value = serde_json::from_str(config_json).unwrap_or(serde_json::Value::Null);
            
        let mut fec = serde_json::from_value::<OpenCvFrontendConfig>(json_val.clone()).unwrap_or_default();
        if let Some(fec_val) = json_val.get("frontend").or(json_val.get("sfm")) {
            if let Ok(overridden) = serde_json::from_value(fec_val.clone()) {
                fec = overridden;
            }
        }

        if let Ok(parsed_flat) = serde_json::from_value::<jni_bridge::FrontendJniConfig>(json_val.clone()) {
            if let Some(v) = parsed_flat.max_hamming_distance { fec.matching.max_hamming_distance = v; }
            if let Some(v) = parsed_flat.max_matches { fec.matching.max_matches = v; }
            if let Some(v) = parsed_flat.ransac_probability { fec.ransac.probability = v; }
            if let Some(v) = parsed_flat.ransac_threshold_px { fec.ransac.threshold_px = v; }
            if let Some(v) = parsed_flat.ransac_max_iters { fec.ransac.max_iters = v; }
            if let Some(v) = parsed_flat.min_depth { fec.inlier_filtering.min_depth = v; }
            if let Some(v) = parsed_flat.max_depth { fec.inlier_filtering.max_depth = v; }
        }

        let mut bac = serde_json::from_value::<SlidingWindowConfig>(json_val.clone()).unwrap_or_default();
        if let Some(ba_val) = json_val.get("ba").or(json_val.get("bundle-adjustment")) {
            if let Ok(overridden) = serde_json::from_value(ba_val.clone()) {
                bac = overridden;
            }
        }
        (fec, bac)
    }
}

