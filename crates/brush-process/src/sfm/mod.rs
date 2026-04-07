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

// ── Placeholder stubs for OpenCV stages (fill in as you implement them) ───────
// Uncomment each line when the corresponding file exists:
//
pub mod stage_3_1_feature_extraction;
pub mod stage_3_2_matching;
pub mod stage_3_3_ransac;
pub mod stage_3_4_pose_recovery;
pub mod stage_3_5_triangulation;
pub mod stage_3_6_inlier_filtering;
// pub mod stage_3_8_pose_export;

// ── Contract type shared across stage boundaries ─────────────────────────────

use nalgebra::{Matrix3, Vector3};
use stage_3_2_matching::{match_feature_sets, FrameFeatures, MatchConfig, MatchingResult};
use stage_3_3_ransac::{estimate_essential_matrix, EssentialMatrixResult, RansacConfig};
use stage_3_4_pose_recovery::{recover_relative_pose, PoseRecoveryResult};
use stage_3_5_triangulation::{triangulate_inlier_points, TriangulatedPoint};
use stage_3_6_inlier_filtering::{build_stage_3_6_output, InlierFilterConfig};
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

#[derive(Debug, Clone, Default)]
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
    anyhow::ensure!(
        matching.matches.len() >= 8,
        "need at least 8 matches, got {}",
        matching.matches.len()
    );
    let essential = estimate_essential_matrix(&matching, &intrinsics, &config.ransac)?;
    let pose = recover_relative_pose(&essential, &intrinsics)?;
    let triangulated = triangulate_inlier_points(&essential, &pose, &intrinsics)?;
    let stage_3_6 = build_stage_3_6_output(
        &matching,
        &pose,
        &triangulated,
        intrinsics,
        gps_priors,
        imu_priors,
        &config.inlier_filtering,
    )?;
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

    use anyhow::Context;
    use jni::objects::{JClass, JString};
    use jni::sys::jstring;
    use jni::JNIEnv;
    use serde::{Deserialize, Serialize};

    use super::*;
    use crate::sfm::stage_3_1_feature_extraction::extract_features;

    #[derive(Debug, Deserialize)]
    struct FrontendFrameInput {
        frame_idx: usize,
        image_path: String,
    }

    #[derive(Debug, Deserialize, Default)]
    struct FrontendJniConfig {
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
    }

    #[derive(Debug, Serialize)]
    struct FrontendJniResult {
        points: Vec<[f64; 3]>,
        observations: Vec<Observation>,
        total_pairs: usize,
        successful_pairs: usize,
        total_matches: usize,
        total_inliers: usize,
        pair_errors: Vec<String>,
    }

    #[unsafe(no_mangle)]
    pub extern "system" fn Java_com_splats_app_sfm_OpenCvFrontendLib_runOpenCvFrontend(
        mut env: JNIEnv<'_>,
        _class: JClass<'_>,
        frames_json: JString<'_>,
        intrinsics_json: JString<'_>,
        config_json: JString<'_>,
    ) -> jstring {
        let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            let mut to_string = |value: JString<'_>| -> String {
                env.get_string(&value).map(|s| s.into()).unwrap_or_default()
            };

            let frames_str = to_string(frames_json);
            let intrinsics_str = to_string(intrinsics_json);
            let config_str = to_string(config_json);

            let json = match run_frontend_from_json(&frames_str, &intrinsics_str, &config_str) {
                Ok(value) => value,
                Err(error) => serde_json::json!({ "error": error.to_string() }).to_string(),
            };

            env.new_string(json)
                .expect("failed to allocate Java string")
                .into_raw()
        }));

        match result {
            Ok(value) => value,
            Err(_) => env
                .new_string(
                    serde_json::json!({ "error": "Rust panic in OpenCV frontend" }).to_string(),
                )
                .expect("failed to allocate Java string")
                .into_raw(),
        }
    }

    fn run_frontend_from_json(
        frames_json: &str,
        intrinsics_json: &str,
        config_json: &str,
    ) -> anyhow::Result<String> {
        let frames: Vec<FrontendFrameInput> =
            serde_json::from_str(frames_json).context("failed to parse frames json")?;
        let intrinsics: CameraIntrinsics =
            serde_json::from_str(intrinsics_json).context("failed to parse intrinsics json")?;
        let parsed_config: FrontendJniConfig = if config_json.trim().is_empty() {
            FrontendJniConfig::default()
        } else {
            serde_json::from_str(config_json).unwrap_or_default()
        };

        let frontend_config = OpenCvFrontendConfig {
            matching: MatchConfig {
                max_hamming_distance: parsed_config.max_hamming_distance.unwrap_or(64.0),
                max_matches: parsed_config.max_matches.unwrap_or(512),
            },
            ransac: RansacConfig {
                probability: parsed_config.ransac_probability.unwrap_or(0.999),
                threshold_px: parsed_config.ransac_threshold_px.unwrap_or(1.0),
                max_iters: parsed_config.ransac_max_iters.unwrap_or(10_000),
            },
            inlier_filtering: InlierFilterConfig {
                min_depth: parsed_config.min_depth.unwrap_or(0.01),
                max_depth: parsed_config.max_depth.unwrap_or(10_000.0),
            },
        };

        let mut features = Vec::with_capacity(frames.len());
        for frame in &frames {
            let image_bytes = fs::read(&frame.image_path)
                .with_context(|| format!("failed to read frame {}", frame.image_path))?;
            let extraction = extract_features(&image_bytes).map_err(|error| {
                anyhow::anyhow!(
                    "feature extraction failed for {}: {error}",
                    frame.image_path
                )
            })?;
            features.push(FrameFeatures::from_extraction(frame.frame_idx, extraction));
        }

        let mut points = Vec::new();
        let mut observations = Vec::new();
        let mut pair_errors = Vec::new();
        let mut total_matches = 0usize;
        let mut total_inliers = 0usize;
        let total_pairs = features.len().saturating_sub(1);
        let mut successful_pairs = 0usize;

        for pair_idx in 0..total_pairs {
            let frame_a = &features[pair_idx];
            let frame_b = &features[pair_idx + 1];

            match run_opencv_frontend(
                frame_a,
                frame_b,
                intrinsics.clone(),
                Vec::new(),
                Vec::new(),
                &frontend_config,
            ) {
                Ok(frontend) => {
                    successful_pairs += 1;
                    total_matches += frontend.matching.matches.len();
                    total_inliers += frontend.pose.inlier_count.max(0) as usize;

                    let point_offset = points.len();
                    points.extend(
                        frontend
                            .stage_3_6
                            .point_positions
                            .iter()
                            .map(|point| [point.x, point.y, point.z]),
                    );
                    observations.extend(frontend.stage_3_6.observations.into_iter().map(
                        |mut obs| {
                            obs.point_idx += point_offset;
                            obs
                        },
                    ));
                }
                Err(error) => {
                    pair_errors.push(format!(
                        "pair {}->{} failed: {error}",
                        frame_a.frame_id, frame_b.frame_id
                    ));
                }
            }
        }

        serde_json::to_string(&FrontendJniResult {
            points,
            observations,
            total_pairs,
            successful_pairs,
            total_matches,
            total_inliers,
            pair_errors,
        })
        .context("failed to serialise frontend result")
    }
}
