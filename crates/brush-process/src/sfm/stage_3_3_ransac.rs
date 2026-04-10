use opencv::{
    calib3d,
    core::{Mat, Point2d, Point2f, Vector},
    prelude::*,
};

use crate::{sfm::stage_3_2_matching::MatchingResult, CameraIntrinsics};

#[derive(Debug)]
pub struct EssentialMatrixResult {
    pub essential_matrix: Mat,
    pub inlier_mask: Vec<u8>,
    pub points_a: Vec<Point2f>,
    pub points_b: Vec<Point2f>,
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct RansacConfig {
    pub probability: f64,
    pub threshold_px: f64,
    pub max_iters: i32,
}

impl Default for RansacConfig {
    fn default() -> Self {
        Self {
            probability: 0.999,
            threshold_px: 1.0,
            max_iters: 10_000,
        }
    }
}

pub fn estimate_essential_matrix(
    matching: &MatchingResult,
    intrinsics: &CameraIntrinsics,
    config: &RansacConfig,
) -> opencv::Result<EssentialMatrixResult> {
    let mut points_a = Vector::<Point2f>::new();
    let mut points_b = Vector::<Point2f>::new();

    for matched in &matching.matches {
        points_a.push(matched.query_point);
        points_b.push(matched.train_point);
    }

    let camera_matrix = camera_matrix(intrinsics)?;
    let mut mask = Mat::default();
    let essential_matrix = calib3d::find_essential_mat(
        &points_a,
        &points_b,
        &camera_matrix,
        calib3d::USAC_MAGSAC,
        config.probability,
        config.threshold_px,
        config.max_iters,
        &mut mask,
    )?;

    let inlier_mask = mask.data_typed::<u8>()?.to_vec();

    Ok(EssentialMatrixResult {
        essential_matrix,
        inlier_mask,
        points_a: points_a.to_vec(),
        points_b: points_b.to_vec(),
    })
}

fn camera_matrix(intrinsics: &CameraIntrinsics) -> opencv::Result<Mat> {
    let binding = [
        intrinsics.fx, 0.0, intrinsics.cx,
        0.0, intrinsics.fy, intrinsics.cy,
        0.0, 0.0, 1.0,
    ];
    let tmp = Mat::from_slice(&binding)?;
    tmp.reshape(1, 3)?.try_clone()
}

#[allow(dead_code)]
fn to_point2d(points: &[Point2f]) -> Vector<Point2d> {
    let mut output = Vector::<Point2d>::new();
    for point in points {
        output.push(Point2d::new(point.x as f64, point.y as f64));
    }
    output
}
