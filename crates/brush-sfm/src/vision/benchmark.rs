//! Benchmarking module comparing Mode A (Vision-Only) vs Mode C (Telemetry-Guided).
//!
//! Mode A:
//! - Exhaustive frame pair matching O(N²)
//! - Standard RANSAC (search mode: 1,000–5,000 iterations, use_extrinsic_guess = false)
//!
//! Mode C:
//! - Telemetry Windowed Matching (prunes frame pairs by physical distance baseline, ~O(N))
//! - Guided RANSAC (verification mode: 50–200 iterations, use_extrinsic_guess = true with prior rvec/tvec)

use serde::{Deserialize, Serialize};
use std::time::Instant;

use crate::{
    coords::pose_stamp_to_opencv_prior,
    telemetry::PoseStamp,
    vision::{
        matching::{FramePair, TelemetryWindowConfig, telemetry_window_pairs},
        pose::{GuidedRansacConfig, PoseEstimationMode, SolvePnPRansacRequest},
    },
};

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PosePairBenchmark {
    pub frame_a: usize,
    pub frame_b: usize,
    pub mode: PoseEstimationMode,
    pub ransac_iterations_budget: u32,
    pub use_extrinsic_guess: bool,
    pub prior_available: bool,
    pub matched_features: usize,
    pub inliers: usize,
    pub inlier_ratio: f64,
    pub reprojection_rmse_px: f64,
    pub duration_ms: f64,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SuiteBenchmarkSummary {
    pub total_keyframes: usize,
    pub total_possible_pairs: usize,
    pub mode_a_pairs_matched: usize,
    pub mode_c_pairs_matched: usize,
    pub mode_a_avg_iterations: f64,
    pub mode_c_avg_iterations: f64,
    pub iteration_reduction_ratio: f64,
    pub pair_matching_reduction_ratio: f64,
}

pub fn generate_pair_matching_plan(
    poses: &[PoseStamp],
    mode: PoseEstimationMode,
    window_config: TelemetryWindowConfig,
) -> Vec<FramePair> {
    match mode {
        PoseEstimationMode::VisionOnly => {
            let mut pairs = Vec::new();
            for a in 0..poses.len() {
                for b in (a + 1)..poses.len() {
                    pairs.push(FramePair { a, b });
                }
            }
            pairs
        }
        PoseEstimationMode::TelemetryGuided => telemetry_window_pairs(poses, window_config),
    }
}

pub fn create_pnp_request(
    mode: PoseEstimationMode,
    config: GuidedRansacConfig,
    pose: &PoseStamp,
) -> Result<SolvePnPRansacRequest, crate::coords::PosePriorError> {
    match mode {
        PoseEstimationMode::VisionOnly => Ok(SolvePnPRansacRequest::vision_only(config)),
        PoseEstimationMode::TelemetryGuided => {
            let prior = pose_stamp_to_opencv_prior(pose)?;
            Ok(SolvePnPRansacRequest::telemetry_guided(config, prior))
        }
    }
}

pub fn benchmark_pair_execution<F>(
    frame_a: usize,
    frame_b: usize,
    request: SolvePnPRansacRequest,
    solve_fn: F,
) -> PosePairBenchmark
where
    F: FnOnce(&SolvePnPRansacRequest) -> (usize, usize, f64), // returns (matched, inliers, rmse)
{
    let start = Instant::now();
    let (matched_features, inliers, reprojection_rmse_px) = solve_fn(&request);
    let duration_ms = start.elapsed().as_secs_f64() * 1000.0;

    let inlier_ratio = if matched_features > 0 {
        inliers as f64 / matched_features as f64
    } else {
        0.0
    };

    PosePairBenchmark {
        frame_a,
        frame_b,
        mode: request.mode,
        ransac_iterations_budget: request.iterations_count,
        use_extrinsic_guess: request.use_extrinsic_guess,
        prior_available: request.prior_rvec.is_some(),
        matched_features,
        inliers,
        inlier_ratio,
        reprojection_rmse_px,
        duration_ms,
    }
}

pub fn summarize_benchmarks(
    total_keyframes: usize,
    mode_a_results: &[PosePairBenchmark],
    mode_c_results: &[PosePairBenchmark],
) -> SuiteBenchmarkSummary {
    let total_possible_pairs = if total_keyframes > 1 {
        total_keyframes * (total_keyframes - 1) / 2
    } else {
        0
    };

    let mode_a_avg_iterations = if !mode_a_results.is_empty() {
        mode_a_results.iter().map(|r| r.ransac_iterations_budget as f64).sum::<f64>() / mode_a_results.len() as f64
    } else {
        0.0
    };

    let mode_c_avg_iterations = if !mode_c_results.is_empty() {
        mode_c_results.iter().map(|r| r.ransac_iterations_budget as f64).sum::<f64>() / mode_c_results.len() as f64
    } else {
        0.0
    };

    let iteration_reduction_ratio = if mode_c_avg_iterations > 0.0 {
        mode_a_avg_iterations / mode_c_avg_iterations
    } else {
        0.0
    };

    let pair_matching_reduction_ratio = if !mode_c_results.is_empty() {
        total_possible_pairs as f64 / mode_c_results.len() as f64
    } else {
        0.0
    };

    SuiteBenchmarkSummary {
        total_keyframes,
        total_possible_pairs,
        mode_a_pairs_matched: mode_a_results.len(),
        mode_c_pairs_matched: mode_c_results.len(),
        mode_a_avg_iterations,
        mode_c_avg_iterations,
        iteration_reduction_ratio,
        pair_matching_reduction_ratio,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::telemetry::Enu;

    fn mock_pose(idx: u64, e: f64) -> PoseStamp {
        PoseStamp {
            frame_index: idx,
            video_timestamp: idx as f64,
            telemetry_timestamp: idx as f64,
            position_enu: Enu { e, n: 0.0, u: 10.0 },
            yaw: Some(0.0),
            pitch: Some(0.0),
            roll: Some(0.0),
            gimbal_pitch: Some(-45.0),
        }
    }

    #[test]
    fn mode_a_generates_all_pairs_mode_c_prunes_by_window() {
        let poses = [
            mock_pose(0, 0.0),
            mock_pose(1, 2.0),
            mock_pose(2, 5.0),
            mock_pose(3, 50.0), // outside window
        ];

        let mode_a_plan = generate_pair_matching_plan(
            &poses,
            PoseEstimationMode::VisionOnly,
            TelemetryWindowConfig::default(),
        );
        let mode_c_plan = generate_pair_matching_plan(
            &poses,
            PoseEstimationMode::TelemetryGuided,
            TelemetryWindowConfig {
                min_baseline_m: 1.0,
                max_baseline_m: 10.0,
                max_time_gap_s: 10.0,
            },
        );

        assert_eq!(mode_a_plan.len(), 6); // 4 * 3 / 2
        assert_eq!(mode_c_plan.len(), 3); // (0,1), (0,2), (1,2) within 1.0..=10.0m window
    }

    #[test]
    fn guided_ransac_request_configures_extrinsic_guess() {
        let pose = mock_pose(0, 0.0);
        let config = GuidedRansacConfig::default();

        let req_a = create_pnp_request(PoseEstimationMode::VisionOnly, config, &pose).unwrap();
        let req_c = create_pnp_request(PoseEstimationMode::TelemetryGuided, config, &pose).unwrap();

        assert!(!req_a.use_extrinsic_guess);
        assert_eq!(req_a.iterations_count, 1000);

        assert!(req_c.use_extrinsic_guess);
        assert_eq!(req_c.iterations_count, 100);
        assert!(req_c.prior_rvec.is_some());
    }

    #[test]
    fn benchmarks_and_summarizes_mode_a_vs_mode_c() {
        let req_a = SolvePnPRansacRequest::vision_only(GuidedRansacConfig::default());
        let prior = pose_stamp_to_opencv_prior(&mock_pose(0, 0.0)).unwrap();
        let req_c = SolvePnPRansacRequest::telemetry_guided(GuidedRansacConfig::default(), prior);

        let res_a = benchmark_pair_execution(0, 1, req_a, |_| (100, 80, 1.2));
        let res_c = benchmark_pair_execution(0, 1, req_c, |_| (100, 85, 0.9));

        let summary = summarize_benchmarks(4, &[res_a], &[res_c]);
        assert_eq!(summary.total_possible_pairs, 6);
        assert_eq!(summary.iteration_reduction_ratio, 10.0); // 1000 / 100
    }
}
