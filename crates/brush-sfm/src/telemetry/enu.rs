use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Wgs84Origin {
    pub lat: f64,
    pub lon: f64,
    pub alt: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Enu {
    pub e: f64,
    pub n: f64,
    pub u: f64,
}

/// Convert WGS84 latitude/longitude/altitude to a local flat-Earth ENU frame.
///
/// This intentionally uses the report's small-baseline approximation. It is
/// appropriate for survey flights whose baseline is much smaller than Earth's
/// radius and keeps the mobile SfM path deterministic and cheap.
pub fn wgs84_to_enu(lat: f64, lon: f64, alt: f64, origin: Wgs84Origin) -> Enu {
    const METERS_PER_DEGREE: f64 = 111_320.0;

    let lat0_rad = origin.lat.to_radians();
    Enu {
        e: (lon - origin.lon) * lat0_rad.cos() * METERS_PER_DEGREE,
        n: (lat - origin.lat) * METERS_PER_DEGREE,
        u: alt - origin.alt,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn converts_about_100_meters_north() {
        let origin = Wgs84Origin {
            lat: 12.9716,
            lon: 77.5946,
            alt: 920.0,
        };
        let delta_lat_for_100m = 100.0 / 111_320.0;

        let enu = wgs84_to_enu(origin.lat + delta_lat_for_100m, origin.lon, origin.alt, origin);

        assert!((enu.n - 100.0).abs() <= 1.0, "northing was {}", enu.n);
        assert!(enu.e.abs() <= 0.001, "easting was {}", enu.e);
        assert!(enu.u.abs() <= 0.001, "up was {}", enu.u);
    }

    #[test]
    fn converts_about_100_meters_east() {
        let origin = Wgs84Origin {
            lat: 12.9716,
            lon: 77.5946,
            alt: 920.0,
        };
        let delta_lon_for_100m = 100.0 / (111_320.0 * origin.lat.to_radians().cos());

        let enu = wgs84_to_enu(origin.lat, origin.lon + delta_lon_for_100m, origin.alt, origin);

        assert!((enu.e - 100.0).abs() <= 1.0, "easting was {}", enu.e);
        assert!(enu.n.abs() <= 0.001, "northing was {}", enu.n);
        assert!(enu.u.abs() <= 0.001, "up was {}", enu.u);
    }
}
