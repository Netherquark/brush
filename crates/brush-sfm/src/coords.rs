use nalgebra::{Matrix3, Rotation3, UnitQuaternion, Vector3};
use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::telemetry::{Enu, PoseStamp};

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct OpenCvPosePrior {
    /// Rodrigues rotation vector, world-to-camera, in OpenCV coordinates.
    pub rvec: [f64; 3],
    /// Translation vector, world-to-camera, in OpenCV coordinates.
    pub tvec: [f64; 3],
    /// Camera center in the local ENU world frame.
    pub camera_center_enu: Enu,
    /// Camera-to-world orientation built from telemetry before inversion.
    pub camera_to_world: [[f64; 3]; 3],
}

#[derive(Clone, Debug, Error, PartialEq)]
pub enum PosePriorError {
    #[error("pose stamp is missing yaw")]
    MissingYaw,
    #[error("pose stamp is missing pitch and gimbal_pitch")]
    MissingPitch,
    #[error("camera forward direction is degenerate")]
    DegenerateForward,
}

pub fn pose_stamp_to_opencv_prior(
    pose: &PoseStamp,
) -> Result<OpenCvPosePrior, PosePriorError> {
    let yaw = pose.yaw.ok_or(PosePriorError::MissingYaw)?;
    let pitch = match (pose.pitch, pose.gimbal_pitch) {
        (Some(body), Some(gimbal)) => body + gimbal,
        (Some(body), None) => body,
        (None, Some(gimbal)) => gimbal,
        (None, None) => return Err(PosePriorError::MissingPitch),
    };
    let roll = pose.roll.unwrap_or(0.0);

    let camera_to_world = telemetry_camera_to_world(yaw, pitch, roll)?;
    let world_to_camera = camera_to_world.transpose();
    let camera_center = Vector3::new(
        pose.position_enu.e,
        pose.position_enu.n,
        pose.position_enu.u,
    );
    let tvec = -(world_to_camera * camera_center);
    let rotation = Rotation3::from_matrix_unchecked(world_to_camera);
    let rvec = UnitQuaternion::from_rotation_matrix(&rotation).scaled_axis();

    Ok(OpenCvPosePrior {
        rvec: vector_to_array(rvec),
        tvec: vector_to_array(tvec),
        camera_center_enu: pose.position_enu,
        camera_to_world: matrix_to_array(camera_to_world),
    })
}

pub fn opencv_to_nerf_point(point: [f64; 3]) -> [f64; 3] {
    [point[0], -point[1], -point[2]]
}

fn telemetry_camera_to_world(
    yaw_deg: f64,
    pitch_deg: f64,
    roll_deg: f64,
) -> Result<Matrix3<f64>, PosePriorError> {
    let yaw = yaw_deg.to_radians();
    let pitch = pitch_deg.to_radians();

    // ENU world: x=east, y=north, z=up. Drone yaw is clockwise from north.
    // OpenCV camera: x=right, y=down, z=forward.
    let forward = Vector3::new(
        yaw.sin() * pitch.cos(),
        yaw.cos() * pitch.cos(),
        pitch.sin(),
    );
    let forward = forward
        .try_normalize(1e-12)
        .ok_or(PosePriorError::DegenerateForward)?;

    let world_up = Vector3::z_axis().into_inner();
    let mut right = forward.cross(&world_up);
    if right.norm_squared() < 1e-12 {
        right = Vector3::x_axis().into_inner();
    } else {
        right = right.normalize();
    }
    let down = forward.cross(&right).normalize();

    let base = Matrix3::from_columns(&[right, down, forward]);
    let roll = Rotation3::from_axis_angle(&Vector3::z_axis(), roll_deg.to_radians());
    Ok(base * roll.matrix())
}

fn vector_to_array(v: Vector3<f64>) -> [f64; 3] {
    [v.x, v.y, v.z]
}

fn matrix_to_array(m: Matrix3<f64>) -> [[f64; 3]; 3] {
    [
        [m[(0, 0)], m[(0, 1)], m[(0, 2)]],
        [m[(1, 0)], m[(1, 1)], m[(1, 2)]],
        [m[(2, 0)], m[(2, 1)], m[(2, 2)]],
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pose(yaw: f64, pitch: Option<f64>, gimbal_pitch: Option<f64>) -> PoseStamp {
        PoseStamp {
            frame_index: 0,
            video_timestamp: 0.0,
            telemetry_timestamp: 0.0,
            position_enu: Enu {
                e: 10.0,
                n: 20.0,
                u: 30.0,
            },
            yaw: Some(yaw),
            pitch,
            roll: Some(0.0),
            gimbal_pitch,
        }
    }

    #[test]
    fn converts_forward_north_pose_to_opencv_translation() {
        let prior = pose_stamp_to_opencv_prior(&pose(0.0, Some(0.0), Some(0.0))).unwrap();

        assert!(prior.rvec.iter().all(|v| v.is_finite()));
        assert!((prior.tvec[0] + 10.0).abs() < 1e-9);
        assert!((prior.tvec[1] - 30.0).abs() < 1e-9);
        assert!((prior.tvec[2] + 20.0).abs() < 1e-9);
    }

    #[test]
    fn down_gimbal_points_camera_forward_toward_negative_up() {
        let prior = pose_stamp_to_opencv_prior(&pose(0.0, Some(0.0), Some(-90.0))).unwrap();

        let forward_col = [
            prior.camera_to_world[0][2],
            prior.camera_to_world[1][2],
            prior.camera_to_world[2][2],
        ];
        assert!(forward_col[0].abs() < 1e-9);
        assert!(forward_col[1].abs() < 1e-9);
        assert!((forward_col[2] + 1.0).abs() < 1e-9);
    }

    #[test]
    fn converts_opencv_point_to_nerf_point() {
        assert_eq!(opencv_to_nerf_point([1.0, 2.0, 3.0]), [1.0, -2.0, -3.0]);
    }
}
