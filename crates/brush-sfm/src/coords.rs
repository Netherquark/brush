use nalgebra::Matrix4;

/// Spec: `E = (lon - lon0) * cos(lat0 * π / 180) * 111320`, `N = (lat - lat0) * 111320`.
pub fn wgs84_to_enu(
    lat: f64,
    lon: f64,
    alt: f64,
    lat0: f64,
    lon0: f64,
    alt0: f64,
) -> [f64; 3] {
    let east = (lon - lon0) * lat0.to_radians().cos() * 111_320.0;
    let north = (lat - lat0) * 111_320.0;
    let up = alt - alt0;
    [east, north, up]
}

/// OpenCV C2W (x-right, y-down, z-forward) -> NeRF C2W (x-right, y-up, z-back): `(x, y, z)_cv -> (x, -y, -z)_nerf`.
pub fn opencv_c2w_to_nerf_c2w(c2w_cv: Matrix4<f64>) -> Matrix4<f64> {
    let mut c2w_nerf = c2w_cv;
    for r in 0..3 {
        c2w_nerf[(r, 1)] *= -1.0;
        c2w_nerf[(r, 2)] *= -1.0;
    }
    c2w_nerf
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Matrix4;

    #[test]
    fn wgs84_to_enu_matches_spec_formula() {
        let lat0 = 18.0;
        let lon0 = 73.0;
        let [e, n, u] = wgs84_to_enu(18.001, 73.001, 10.0, lat0, lon0, 0.0);
        let expected_e = (73.001 - 73.0) * (18.0_f64).to_radians().cos() * 111_320.0;
        let expected_n = (18.001 - 18.0) * 111_320.0;
        assert!((e - expected_e).abs() < 1e-9);
        assert!((n - expected_n).abs() < 1e-9);
        assert!((u - 10.0).abs() < 1e-12);
    }

    #[test]
    fn opencv_c2w_to_nerf_flips_y_and_z_columns() {
        let mut c2w = Matrix4::<f64>::identity();
        c2w[(0, 1)] = 2.0;
        c2w[(1, 1)] = 3.0;
        c2w[(2, 1)] = 4.0;
        c2w[(0, 2)] = 5.0;
        c2w[(1, 2)] = 6.0;
        c2w[(2, 2)] = 7.0;
        let nerf = opencv_c2w_to_nerf_c2w(c2w);
        assert!((nerf[(0, 1)] + 2.0).abs() < 1e-12);
        assert!((nerf[(1, 2)] + 6.0).abs() < 1e-12);
        assert!((nerf[(0, 0)] - 1.0).abs() < 1e-12);
    }
}
