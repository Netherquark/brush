use serde::{Deserialize, Serialize};

use crate::telemetry::{Enu, PoseStamp};

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct TelemetryWindowConfig {
    pub min_baseline_m: f64,
    pub max_baseline_m: f64,
    pub max_time_gap_s: f64,
}

impl Default for TelemetryWindowConfig {
    fn default() -> Self {
        Self {
            min_baseline_m: 1.0,
            max_baseline_m: 25.0,
            max_time_gap_s: 5.0,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct FramePair {
    pub a: usize,
    pub b: usize,
}

pub fn telemetry_window_pairs(
    poses: &[PoseStamp],
    config: TelemetryWindowConfig,
) -> Vec<FramePair> {
    let mut pairs = Vec::new();

    for a in 0..poses.len() {
        for b in (a + 1)..poses.len() {
            let time_gap = (poses[b].telemetry_timestamp - poses[a].telemetry_timestamp).abs();
            if time_gap > config.max_time_gap_s {
                break;
            }

            let baseline = enu_distance(poses[a].position_enu, poses[b].position_enu);
            if baseline >= config.min_baseline_m && baseline <= config.max_baseline_m {
                pairs.push(FramePair { a, b });
            }
        }
    }

    pairs
}

fn enu_distance(a: Enu, b: Enu) -> f64 {
    let de = b.e - a.e;
    let dn = b.n - a.n;
    let du = b.u - a.u;
    (de * de + dn * dn + du * du).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pose(idx: u64, e: f64) -> PoseStamp {
        PoseStamp {
            frame_index: idx,
            video_timestamp: idx as f64,
            telemetry_timestamp: idx as f64,
            position_enu: Enu { e, n: 0.0, u: 0.0 },
            yaw: Some(0.0),
            pitch: Some(0.0),
            roll: Some(0.0),
            gimbal_pitch: Some(-45.0),
        }
    }

    #[test]
    fn keeps_only_pairs_inside_telemetry_window() {
        let poses = [pose(0, 0.0), pose(1, 0.5), pose(2, 2.0), pose(3, 8.0), pose(9, 9.0)];

        let pairs = telemetry_window_pairs(
            &poses,
            TelemetryWindowConfig {
                min_baseline_m: 1.0,
                max_baseline_m: 6.0,
                max_time_gap_s: 3.0,
            },
        );

        assert_eq!(
            pairs,
            vec![
                FramePair { a: 0, b: 2 },
                FramePair { a: 1, b: 2 },
                FramePair { a: 2, b: 3 },
            ]
        );
    }
}
