use opencv::{
    calib3d,
    core::{Mat, Point2d, Point3d, Vector},
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

/// Spec §9: Telemetry-guided PnP pose recovery with solvePnPRansac(useExtrinsicGuess=true, rvec=prior_r, tvec=prior_t)
pub fn recover_pose_with_telemetry_pnp(
    object_points: &[Point3d],
    image_points: &[Point2d],
    intrinsics: &CameraIntrinsics,
    prior_rvec: Option<[f64; 3]>,
    prior_tvec: Option<[f64; 3]>,
) -> opencv::Result<Option<PoseRecoveryResult>> {
    if object_points.len() < 4 || object_points.len() != image_points.len() {
        return Ok(None);
    }

    let mut obj_vec = Vector::<Point3d>::new();
    let mut img_vec = Vector::<Point2d>::new();
    for (&pt3, &pt2) in object_points.iter().zip(image_points.iter()) {
        obj_vec.push(pt3);
        img_vec.push(pt2);
    }

    let camera_mat = camera_matrix(intrinsics)?;
    let dist_coeffs = Mat::zeros(4, 1, opencv::core::CV_64F)?.to_mat()?;

    let mut rvec = Mat::zeros(3, 1, opencv::core::CV_64F)?.to_mat()?;
    let mut tvec = Mat::zeros(3, 1, opencv::core::CV_64F)?.to_mat()?;
    let use_guess = if let (Some(pr), Some(pt)) = (prior_rvec, prior_tvec) {
        *rvec.at_2d_mut::<f64>(0, 0)? = pr[0];
        *rvec.at_2d_mut::<f64>(1, 0)? = pr[1];
        *rvec.at_2d_mut::<f64>(2, 0)? = pr[2];
        *tvec.at_2d_mut::<f64>(0, 0)? = pt[0];
        *tvec.at_2d_mut::<f64>(1, 0)? = pt[1];
        *tvec.at_2d_mut::<f64>(2, 0)? = pt[2];
        true
    } else {
        false
    };

    let mut inliers = Mat::default();
    let success = calib3d::solve_pnp_ransac(
        &obj_vec,
        &img_vec,
        &camera_mat,
        &dist_coeffs,
        &mut rvec,
        &mut tvec,
        use_guess,
        100, // iterationsCount lower when seeded
        2.0, // reprojectionError threshold
        0.99, // confidence
        &mut inliers,
        calib3d::SOLVEPNP_ITERATIVE,
    )?;

    if !success {
        return Ok(None);
    }

    let inlier_count = inliers.rows();
    let mut rot_mat = Mat::default();
    calib3d::rodrigues(&rvec, &mut rot_mat, &mut Mat::default())?;

    let inlier_mask = vec![1u8; inlier_count as usize];
    Ok(Some(PoseRecoveryResult {
        rotation: rot_mat,
        translation: tvec,
        inlier_mask,
        inlier_count,
    }))
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
