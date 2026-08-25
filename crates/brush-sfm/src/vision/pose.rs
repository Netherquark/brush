use serde::{Deserialize, Serialize};

use crate::coords::OpenCvPosePrior;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum PoseEstimationMode {
    #[default]
    VisionOnly,
    TelemetryGuided,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct GuidedRansacConfig {
    pub vision_only_iterations: u32,
    pub telemetry_guided_iterations: u32,
    pub reprojection_error_px: f64,
    pub confidence: f64,
}

impl Default for GuidedRansacConfig {
    fn default() -> Self {
        Self {
            vision_only_iterations: 1_000,
            telemetry_guided_iterations: 100,
            reprojection_error_px: 8.0,
            confidence: 0.99,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct SolvePnPRansacRequest {
    pub mode: PoseEstimationMode,
    pub iterations_count: u32,
    pub reprojection_error_px: f64,
    pub confidence: f64,
    pub use_extrinsic_guess: bool,
    pub prior_rvec: Option<[f64; 3]>,
    pub prior_tvec: Option<[f64; 3]>,
}

impl SolvePnPRansacRequest {
    pub fn vision_only(config: GuidedRansacConfig) -> Self {
        Self {
            mode: PoseEstimationMode::VisionOnly,
            iterations_count: config.vision_only_iterations,
            reprojection_error_px: config.reprojection_error_px,
            confidence: config.confidence,
            use_extrinsic_guess: false,
            prior_rvec: None,
            prior_tvec: None,
        }
    }

    pub fn telemetry_guided(config: GuidedRansacConfig, prior: OpenCvPosePrior) -> Self {
        Self {
            mode: PoseEstimationMode::TelemetryGuided,
            iterations_count: config.telemetry_guided_iterations,
            reprojection_error_px: config.reprojection_error_px,
            confidence: config.confidence,
            use_extrinsic_guess: true,
            prior_rvec: Some(prior.rvec),
            prior_tvec: Some(prior.tvec),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        coords::pose_stamp_to_opencv_prior,
        telemetry::{Enu, PoseStamp},
    };

    #[test]
    fn telemetry_guided_request_uses_extrinsic_guess_and_lower_iterations() {
        let config = GuidedRansacConfig::default();
        let prior = pose_stamp_to_opencv_prior(&PoseStamp {
            frame_index: 0,
            video_timestamp: 0.0,
            telemetry_timestamp: 0.0,
            position_enu: Enu {
                e: 0.0,
                n: 0.0,
                u: 10.0,
            },
            yaw: Some(0.0),
            pitch: Some(0.0),
            roll: Some(0.0),
            gimbal_pitch: Some(-45.0),
        })
        .unwrap();

        let request = SolvePnPRansacRequest::telemetry_guided(config, prior);

        assert_eq!(request.mode, PoseEstimationMode::TelemetryGuided);
        assert!(request.use_extrinsic_guess);
        assert_eq!(request.iterations_count, 100);
        assert!(request.prior_rvec.is_some());
        assert!(request.prior_tvec.is_some());
    }

    #[test]
    fn vision_only_request_disables_extrinsic_guess() {
        let request = SolvePnPRansacRequest::vision_only(GuidedRansacConfig::default());

        assert_eq!(request.mode, PoseEstimationMode::VisionOnly);
        assert!(!request.use_extrinsic_guess);
        assert_eq!(request.iterations_count, 1_000);
        assert!(request.prior_rvec.is_none());
        assert!(request.prior_tvec.is_none());
    }
}
