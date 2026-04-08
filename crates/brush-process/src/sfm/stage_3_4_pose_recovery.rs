use opencv::{
    calib3d,
    core::{Mat, Point2d, Vector},
    prelude::*,
};

use crate::{sfm::stage_3_3_ransac::EssentialMatrixResult, CameraIntrinsics};

#[derive(Debug)]
pub struct PoseRecoveryResult {
    pub rotation: Mat,
    pub translation: Mat,
    pub inlier_mask: Vec<u8>,
    pub inlier_count: i32,
}

pub fn recover_relative_pose(
    essential: &EssentialMatrixResult,
    intrinsics: &CameraIntrinsics,
) -> opencv::Result<PoseRecoveryResult> {
    let mut rotation = Mat::default();
    let mut translation = Mat::default();
    let mut mask = Mat::default();

    let mut points_a = Vector::<Point2d>::new();
    let mut points_b = Vector::<Point2d>::new();
    for point in &essential.points_a {
        points_a.push(Point2d::new(point.x as f64, point.y as f64));
    }
    for point in &essential.points_b {
        points_b.push(Point2d::new(point.x as f64, point.y as f64));
    }

    let principal_point = opencv::core::Point2d::new(intrinsics.cx, intrinsics.cy);
    let inlier_count = calib3d::recover_pose(
        &essential.essential_matrix,
        &points_a,
        &points_b,
        &mut rotation,
        &mut translation,
        intrinsics.fx,
        principal_point,
        &mut mask,
    )?;

    Ok(PoseRecoveryResult {
        rotation,
        translation,
        inlier_mask: mask.data_typed::<u8>()?.to_vec(),
        inlier_count,
    })
}
