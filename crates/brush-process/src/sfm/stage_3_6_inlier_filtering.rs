use nalgebra::{Matrix3, Vector3};

use crate::{
    sfm::{
        stage_3_2_matching::MatchingResult, stage_3_4_pose_recovery::PoseRecoveryResult,
        stage_3_5_triangulation::TriangulatedPoint, Stage36Output,
    },
    CameraIntrinsics, GpsPrior, ImuRotationPrior, Observation,
};

#[derive(Debug, Clone)]
pub struct InlierFilterConfig {
    pub min_depth: f64,
    pub max_depth: f64,
}

impl Default for InlierFilterConfig {
    fn default() -> Self {
        Self {
            min_depth: 0.01,
            max_depth: 10_000.0,
        }
    }
}

pub fn build_stage_3_6_output(
    matching: &MatchingResult,
    pose: &PoseRecoveryResult,
    triangulated: &[TriangulatedPoint],
    intrinsics: CameraIntrinsics,
    gps_priors: Vec<GpsPrior>,
    imu_priors: Vec<ImuRotationPrior>,
    config: &InlierFilterConfig,
) -> anyhow::Result<Stage36Output> {
    let rotation = mat3_from_opencv(&pose.rotation)?;
    let translation = vec3_from_opencv(&pose.translation)?;

    let mut point_positions = Vec::new();
    let mut observations = Vec::new();

    for point in triangulated {
        if !point.point.iter().all(|value| value.is_finite()) {
            continue;
        }
        if point.point.z < config.min_depth || point.point.z > config.max_depth {
            continue;
        }

        let point_idx = point_positions.len();
        point_positions.push(point.point);
        observations.push(Observation {
            frame_idx: matching.frame_a,
            point_idx,
            observed: [point.observation_a.x as f64, point.observation_a.y as f64],
            weight: 1.0,
        });
        observations.push(Observation {
            frame_idx: matching.frame_b,
            point_idx,
            observed: [point.observation_b.x as f64, point.observation_b.y as f64],
            weight: 1.0,
        });
    }

    Ok(Stage36Output {
        frame_rotations: vec![
            (matching.frame_a, Matrix3::identity()),
            (matching.frame_b, rotation),
        ],
        frame_translations: vec![
            (matching.frame_a, Vector3::zeros()),
            (matching.frame_b, translation),
        ],
        point_positions,
        observations,
        gps_priors,
        imu_priors,
        intrinsics,
    })
}

fn mat3_from_opencv(matrix: &opencv::core::Mat) -> anyhow::Result<Matrix3<f64>> {
    Ok(Matrix3::new(
        *matrix.at_2d::<f64>(0, 0)?,
        *matrix.at_2d::<f64>(0, 1)?,
        *matrix.at_2d::<f64>(0, 2)?,
        *matrix.at_2d::<f64>(1, 0)?,
        *matrix.at_2d::<f64>(1, 1)?,
        *matrix.at_2d::<f64>(1, 2)?,
        *matrix.at_2d::<f64>(2, 0)?,
        *matrix.at_2d::<f64>(2, 1)?,
        *matrix.at_2d::<f64>(2, 2)?,
    ))
}

fn vec3_from_opencv(vector: &opencv::core::Mat) -> anyhow::Result<Vector3<f64>> {
    Ok(Vector3::new(
        *vector.at_2d::<f64>(0, 0)?,
        *vector.at_2d::<f64>(1, 0)?,
        *vector.at_2d::<f64>(2, 0)?,
    ))
}
