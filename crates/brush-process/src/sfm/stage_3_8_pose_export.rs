use anyhow::Result;
use nalgebra::{Matrix3, Matrix4, Vector3};
use serde::Serialize;
use std::fs;
use std::path::Path;

use crate::sfm::stage_3_7_bundle_adjustment::{CameraIntrinsics, GlobalSfmState, write_global_state_ply};

#[derive(Serialize)]
struct TransformsJson {
    pub fl_x: f64,
    pub fl_y: f64,
    pub cx: f64,
    pub cy: f64,
    pub w: u32,
    pub h: u32,
    pub camera_model: String,
    pub frames: Vec<FrameJson>,
}

#[derive(Serialize)]
struct FrameJson {
    pub file_path: String,
    pub transform_matrix: [[f32; 4]; 4],
}

pub fn export_sfm_results(
    global: &GlobalSfmState,
    intrinsics: &CameraIntrinsics,
    frame_paths: &[String],
    output_dir: &Path,
    img_width: u32,
    img_height: u32,
) -> Result<()> {
    fs::create_dir_all(output_dir)?;

    // 1. Export sparse.ply
    let ply_path = output_dir.join("sparse.ply");
    write_global_state_ply(&ply_path, global)?;

    // 2. Export transforms.json
    let mut frames = Vec::new();
    for (i, (rot_aa, trans)) in global.rotations.iter().zip(global.translations.iter()).enumerate() {
        let rotation = axis_angle_to_rotation_matrix(rot_aa);
        let translation = Vector3::new(trans[0], trans[1], trans[2]);

        // OpenCV W2C -> OpenCV C2W
        let mut w2c = Matrix4::identity();
        w2c.fixed_view_mut::<3, 3>(0, 0).copy_from(&rotation);
        w2c.fixed_view_mut::<3, 1>(0, 3).copy_from(&translation);

        let c2w_cv = w2c.try_inverse().ok_or_else(|| anyhow::anyhow!("Matrix inversion failed for frame {}", i))?;

        // OpenCV C2W (x-right, y-down, z-forward) -> NeRF C2W (x-right, y-up, z-back)
        // This involves flipping the y and z axes.
        let mut c2w_nerf = c2w_cv;
        for r in 0..3 {
            c2w_nerf[(r, 1)] *= -1.0; // Flip Y column
            c2w_nerf[(r, 2)] *= -1.0; // Flip Z column
        }

        let mut matrix_arr = [[0.0f32; 4]; 4];
        for r in 0..4 {
            for c in 0..4 {
                matrix_arr[r][c] = c2w_nerf[(r, c)] as f32;
            }
        }

        let file_path = if let Some(path) = frame_paths.get(i) {
            Path::new(path).file_name()
                .and_then(|n| n.to_str())
                .unwrap_or("frame.jpg")
                .to_string()
        } else {
            format!("frame_{:05}.jpg", i)
        };

        frames.push(FrameJson {
            file_path,
            transform_matrix: matrix_arr,
        });
    }

    let transforms = TransformsJson {
        fl_x: intrinsics.fx,
        fl_y: intrinsics.fy,
        cx: intrinsics.cx,
        cy: intrinsics.cy,
        w: img_width,
        h: img_height,
        camera_model: "OPENCV".to_string(),
        frames,
    };

    let json_path = output_dir.join("transforms.json");
    let json_str = serde_json::to_string_pretty(&transforms)?;
    fs::write(json_path, json_str)?;

    Ok(())
}

fn axis_angle_to_rotation_matrix(aa: &[f64; 3]) -> Matrix3<f64> {
    let axis = Vector3::new(aa[0], aa[1], aa[2]);
    let angle = axis.norm();
    if angle < 1e-10 {
        Matrix3::identity()
    } else {
        nalgebra::Rotation3::from_axis_angle(&nalgebra::Unit::new_normalize(axis), angle).into_inner()
    }
}
