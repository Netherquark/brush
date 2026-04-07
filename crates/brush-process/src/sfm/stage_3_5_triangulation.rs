use nalgebra::Vector3;
use opencv::{
    calib3d,
    core::{self, Mat, Point2f, Vector},
    prelude::*,
};

use crate::{
    sfm::{stage_3_3_ransac::EssentialMatrixResult, stage_3_4_pose_recovery::PoseRecoveryResult},
    CameraIntrinsics,
};

#[derive(Debug, Clone)]
pub struct TriangulatedPoint {
    pub point: Vector3<f64>,
    pub feature_index: usize,
    pub observation_a: Point2f,
    pub observation_b: Point2f,
}

pub fn triangulate_inlier_points(
    essential: &EssentialMatrixResult,
    pose: &PoseRecoveryResult,
    intrinsics: &CameraIntrinsics,
) -> opencv::Result<Vec<TriangulatedPoint>> {
    let projection_a = projection_identity(intrinsics)?;
    let projection_b = projection_from_pose(intrinsics, &pose.rotation, &pose.translation)?;

    let mut points_a = Vector::<Point2f>::new();
    let mut points_b = Vector::<Point2f>::new();
    let mut indices = Vec::new();

    for (idx, ((point_a, point_b), is_inlier)) in essential
        .points_a
        .iter()
        .zip(essential.points_b.iter())
        .zip(pose.inlier_mask.iter().copied())
        .enumerate()
    {
        if is_inlier == 0 {
            continue;
        }
        points_a.push(*point_a);
        points_b.push(*point_b);
        indices.push(idx);
    }

    if indices.is_empty() {
        return Ok(Vec::new());
    }

    let mut homogeneous = Mat::default();
    calib3d::triangulate_points(
        &projection_a,
        &projection_b,
        &points_a,
        &points_b,
        &mut homogeneous,
    )?;

    let mut output = Vec::with_capacity(indices.len());
    for (column, feature_index) in indices.into_iter().enumerate() {
        let x = *homogeneous.at_2d::<f64>(0, column as i32)?;
        let y = *homogeneous.at_2d::<f64>(1, column as i32)?;
        let z = *homogeneous.at_2d::<f64>(2, column as i32)?;
        let w = *homogeneous.at_2d::<f64>(3, column as i32)?;
        if w.abs() < f64::EPSILON {
            continue;
        }

        output.push(TriangulatedPoint {
            point: Vector3::new(x / w, y / w, z / w),
            feature_index,
            observation_a: essential.points_a[feature_index],
            observation_b: essential.points_b[feature_index],
        });
    }

    Ok(output)
}

fn projection_identity(intrinsics: &CameraIntrinsics) -> opencv::Result<Mat> {
    let k = camera_matrix(intrinsics)?;
    let rt = Mat::from_slice_2d(&[
        [1.0_f64, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
    ])?;
    let mut projection = Mat::default();
    core::gemm(&k, &rt, 1.0, &Mat::default(), 0.0, &mut projection, 0)?;
    Ok(projection)
}

fn projection_from_pose(
    intrinsics: &CameraIntrinsics,
    rotation: &Mat,
    translation: &Mat,
) -> opencv::Result<Mat> {
    let k = camera_matrix(intrinsics)?;
    let mut rt = Mat::zeros(3, 4, core::CV_64F)?.to_mat()?;

    for row in 0..3 {
        for col in 0..3 {
            *rt.at_2d_mut::<f64>(row, col)? = *rotation.at_2d::<f64>(row, col)?;
        }
        *rt.at_2d_mut::<f64>(row, 3)? = *translation.at_2d::<f64>(row, 0)?;
    }

    let mut projection = Mat::default();
    core::gemm(&k, &rt, 1.0, &Mat::default(), 0.0, &mut projection, 0)?;
    Ok(projection)
}

fn camera_matrix(intrinsics: &CameraIntrinsics) -> opencv::Result<Mat> {
    Mat::from_slice_2d(&[
        [intrinsics.fx, 0.0, intrinsics.cx],
        [0.0, intrinsics.fy, intrinsics.cy],
        [0.0, 0.0, 1.0],
    ])
}
