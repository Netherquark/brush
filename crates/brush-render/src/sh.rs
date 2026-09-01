use crate::shaders;

use glam::Vec3;
const SH_C0: f32 = shaders::SH_C0;

pub const fn sh_coeffs_for_degree(degree: u32) -> u32 {
    (degree + 1).pow(2)
}

pub fn sh_degree_from_coeffs(coeffs_per_channel: u32) -> u32 {
    match coeffs_per_channel {
        0..=1 => 0,
        2..=4 => 1,
        5..=9 => 2,
        10..=16 => 3,
        _ => 4,
    }
}

pub fn channel_to_sh(rgb: f32) -> f32 {
    (rgb - 0.5) / SH_C0
}

pub fn rgb_to_sh(rgb: Vec3) -> Vec3 {
    glam::vec3(
        channel_to_sh(rgb.x),
        channel_to_sh(rgb.y),
        channel_to_sh(rgb.z),
    )
}
