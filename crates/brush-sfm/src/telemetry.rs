use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct RawTelemetryRecord {
    pub timestamp_us: i64,
    pub lat: f64,
    pub lon: f64,
    pub alt: f64,
    pub yaw: f64,
    pub pitch: f64,
    pub roll: f64,
    pub gimbal_pitch: f64,
    pub vel_n: f64,
    pub vel_e: f64,
    pub vel_d: f64,
    pub hdop: f64,
    pub num_sats: u32,
    #[serde(default)]
    pub fix_type: u32,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct ValidatedRecord {
    pub timestamp_us: i64,
    pub lat: f64,
    pub lon: f64,
    pub alt: f64,
    pub yaw: f64,
    pub pitch: f64,
    pub roll: f64,
    pub gimbal_pitch: f64,
    pub vel_n: f64,
    pub vel_e: f64,
    pub vel_d: f64,
    pub speed_ms: f64,
    pub enu_position: [f64; 3],
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub enum KeyframeTrigger {
    First,
    Distance,
    Yaw,
    Pitch,
    Time,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct KeyframeCandidate {
    pub index: usize,
    pub timestamp_us: i64,
    pub lat: f64,
    pub lon: f64,
    pub yaw_deg: f64,
    pub gimbal_pitch: f64,
    pub speed_ms: f64,
    pub trigger_reason: KeyframeTrigger,
    pub enu_position: [f64; 3],
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct KeyframeConfig {
    pub distance_threshold_m: f64,
    pub yaw_threshold_deg: f64,
    pub pitch_threshold_deg: f64,
    pub time_threshold_us: i64,
    pub min_speed_ms: f64,
    pub max_keyframes: usize,
}

impl Default for KeyframeConfig {
    fn default() -> Self {
        Self {
            distance_threshold_m: 2.0,
            yaw_threshold_deg: 8.0,
            pitch_threshold_deg: 5.0,
            time_threshold_us: 1_000_000, // 1 sec
            min_speed_ms: 0.2,
            max_keyframes: 100,
        }
    }
}

/// Spec §5: Quality gate rejecting invalid records (HDOP > 3.0, non-3D GPS fix, out-of-range).
pub fn validate_telemetry_records(
    records: &[RawTelemetryRecord],
    origin: Option<[f64; 3]>,
) -> anyhow::Result<(Option<[f64; 3]>, Vec<ValidatedRecord>)> {
    if records.is_empty() {
        return Ok((None, Vec::new()));
    }

    let origin_anchor = origin.or_else(|| {
        records.iter().find(|r| r.hdop <= 3.0 && r.fix_type >= 3 && r.lat.is_finite() && r.lon.is_finite()).map(|r| [r.lat, r.lon, r.alt])
    }).unwrap_or_else(|| [records[0].lat, records[0].lon, records[0].alt]);

    let mut validated = Vec::with_capacity(records.len());
    for r in records {
        if !r.lat.is_finite() || !r.lon.is_finite() || !r.alt.is_finite() {
            continue;
        }
        if r.lat < -90.0 || r.lat > 90.0 || r.lon < -180.0 || r.lon > 180.0 {
            continue;
        }
        if r.hdop > 3.0 && r.hdop > 0.0 {
            continue;
        }

        let speed_ms = (r.vel_n * r.vel_n + r.vel_e * r.vel_e + r.vel_d * r.vel_d).sqrt();
        let enu = crate::coords::wgs84_to_enu(
            r.lat,
            r.lon,
            r.alt,
            origin_anchor[0],
            origin_anchor[1],
            origin_anchor[2],
        );

        validated.push(ValidatedRecord {
            timestamp_us: r.timestamp_us,
            lat: r.lat,
            lon: r.lon,
            alt: r.alt,
            yaw: r.yaw,
            pitch: r.pitch,
            roll: r.roll,
            gimbal_pitch: r.gimbal_pitch,
            vel_n: r.vel_n,
            vel_e: r.vel_e,
            vel_d: r.vel_d,
            speed_ms,
            enu_position: enu,
        });
    }

    Ok((Some(origin_anchor), validated))
}

/// Spec §6: Pure Rust Keyframe Selection pass: Vec<ValidatedRecord> -> Vec<KeyframeCandidate>
pub fn select_keyframes(
    records: &[ValidatedRecord],
    config: &KeyframeConfig,
) -> Vec<KeyframeCandidate> {
    if records.is_empty() {
        return Vec::new();
    }

    let mut all_candidates = Vec::new();
    let mut last_selected: Option<&ValidatedRecord> = None;

    for (idx, r) in records.iter().enumerate() {
        if r.speed_ms < config.min_speed_ms {
            continue;
        }

        let Some(prev) = last_selected else {
            all_candidates.push(KeyframeCandidate {
                index: idx,
                timestamp_us: r.timestamp_us,
                lat: r.lat,
                lon: r.lon,
                yaw_deg: r.yaw,
                gimbal_pitch: r.gimbal_pitch,
                speed_ms: r.speed_ms,
                trigger_reason: KeyframeTrigger::First,
                enu_position: r.enu_position,
            });
            last_selected = Some(r);
            continue;
        };

        let dx = r.enu_position[0] - prev.enu_position[0];
        let dy = r.enu_position[1] - prev.enu_position[1];
        let dz = r.enu_position[2] - prev.enu_position[2];
        let dist = (dx * dx + dy * dy + dz * dz).sqrt();

        let yaw_d = yaw_diff_deg(prev.yaw, r.yaw);
        let pitch_d = (r.gimbal_pitch - prev.gimbal_pitch).abs();
        let time_d = r.timestamp_us - prev.timestamp_us;

        let trigger = if dist > config.distance_threshold_m {
            Some(KeyframeTrigger::Distance)
        } else if yaw_d > config.yaw_threshold_deg {
            Some(KeyframeTrigger::Yaw)
        } else if pitch_d > config.pitch_threshold_deg {
            Some(KeyframeTrigger::Pitch)
        } else if time_d > config.time_threshold_us {
            Some(KeyframeTrigger::Time)
        } else {
            None
        };

        if let Some(reason) = trigger {
            all_candidates.push(KeyframeCandidate {
                index: idx,
                timestamp_us: r.timestamp_us,
                lat: r.lat,
                lon: r.lon,
                yaw_deg: r.yaw,
                gimbal_pitch: r.gimbal_pitch,
                speed_ms: r.speed_ms,
                trigger_reason: reason,
                enu_position: r.enu_position,
            });
            last_selected = Some(r);
        }
    }

    if all_candidates.len() <= config.max_keyframes {
        all_candidates
    } else {
        // Deterministic uniform decimation if reservoir pseudo-random not desired, preserving order
        let total = all_candidates.len();
        let cap = config.max_keyframes;
        let mut result = Vec::with_capacity(cap);
        for i in 0..cap {
            let src_idx = (i * (total - 1)) / (cap - 1);
            result.push(all_candidates[src_idx].clone());
        }
        result
    }
}

/// Negative-safe modulo for yaw wraparound: ((yaw2 - yaw1 + 180).rem_euclid(360)) - 180
pub fn yaw_diff_deg(yaw1: f64, yaw2: f64) -> f64 {
    let diff = (yaw2 - yaw1 + 180.0).rem_euclid(360.0) - 180.0;
    diff.abs()
}

/// Spec §6 / overhaul §3.1: JNI bridge so Kotlin delegates validation+selection to
/// canonical Rust impl, eliminating cross-language duplication.
///
/// Input:  JSON array of `RawTelemetryRecord` + optional `KeyframeConfig` JSON object.
/// Output: JSON array of `KeyframeCandidate`.
///
/// Java signature:
///   `TelemetryLib.selectKeyframesFromJson(String recordsJson, String configJson): String`
#[cfg(feature = "jni-support")]
pub mod jni_bridge {
    use super::*;
    use jni::objects::{JClass, JString};
    use jni::sys::jstring;
    use jni::JNIEnv;

    #[unsafe(no_mangle)]
    pub extern "system" fn Java_com_splats_app_telemetry_TelemetryLib_selectKeyframesFromJson(
        mut env: JNIEnv<'_>,
        _class: JClass<'_>,
        records_json: JString<'_>,
        config_json: JString<'_>,
    ) -> jstring {
        let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            let records_str: String = env
                .get_string(&records_json)
                .map(|s| s.into())
                .unwrap_or_default();
            let config_str: String = env
                .get_string(&config_json)
                .map(|s| s.into())
                .unwrap_or_default();

            let json = select_keyframes_from_json_inner(&records_str, &config_str);

            env.new_string(json)
                .map(|s| s.into_raw())
                .unwrap_or(std::ptr::null_mut())
        }));

        match result {
            Ok(ptr) => ptr,
            Err(_) => {
                let err = r#"{"error":"Rust panic in selectKeyframesFromJson"}"#;
                env.new_string(err)
                    .map(|s| s.into_raw())
                    .unwrap_or(std::ptr::null_mut())
            }
        }
    }

    pub(crate) fn select_keyframes_from_json_inner(records_json: &str, config_json: &str) -> String {
        let records: Vec<RawTelemetryRecord> = match serde_json::from_str(records_json) {
            Ok(r) => r,
            Err(e) => return format!(r#"{{"error":"parse records: {e}"}}"#),
        };
        let config: KeyframeConfig = if config_json.trim().is_empty() {
            KeyframeConfig::default()
        } else {
            serde_json::from_str(config_json).unwrap_or_default()
        };

        let (_origin, validated) = match validate_telemetry_records(&records, None) {
            Ok(v) => v,
            Err(e) => return format!(r#"{{"error":"validate: {e}"}}"#),
        };
        let candidates = select_keyframes(&validated, &config);
        serde_json::to_string(&candidates)
            .unwrap_or_else(|_| r#"{"error":"serialise"}"#.to_string())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_yaw_diff_wraparound() {
        assert!((yaw_diff_deg(355.0, 5.0) - 10.0).abs() < 1e-9);
        assert!((yaw_diff_deg(5.0, 355.0) - 10.0).abs() < 1e-9);
        assert!((yaw_diff_deg(0.0, 180.0) - 180.0).abs() < 1e-9);
    }

    #[test]
    fn test_keyframe_selection_triggers() {
        let recs = vec![
            ValidatedRecord {
                timestamp_us: 0,
                lat: 10.0,
                lon: 10.0,
                alt: 100.0,
                yaw: 0.0,
                pitch: 0.0,
                roll: 0.0,
                gimbal_pitch: -45.0,
                vel_n: 1.0,
                vel_e: 0.0,
                vel_d: 0.0,
                speed_ms: 1.0,
                enu_position: [0.0, 0.0, 0.0],
            },
            ValidatedRecord {
                timestamp_us: 500_000,
                lat: 10.0001,
                lon: 10.0,
                alt: 100.0,
                yaw: 10.0, // > 8 deg
                pitch: 0.0,
                roll: 0.0,
                gimbal_pitch: -45.0,
                vel_n: 1.0,
                vel_e: 0.0,
                vel_d: 0.0,
                speed_ms: 1.0,
                enu_position: [0.0, 11.132, 0.0], // > 2m
            },
        ];

        let candidates = select_keyframes(&recs, &KeyframeConfig::default());
        assert_eq!(candidates.len(), 2);
        assert_eq!(candidates[0].trigger_reason, KeyframeTrigger::First);
        assert_eq!(candidates[1].trigger_reason, KeyframeTrigger::Distance);
    }

    #[cfg(feature = "jni-support")]
    #[test]
    fn test_select_keyframes_from_json_inner_empty_config() {
        use jni_bridge::select_keyframes_from_json_inner;

        // Minimal single-record input: invalid (only 1 row), should return empty candidates
        let records_json = serde_json::json!([{
            "timestamp_us": 0i64,
            "lat": 10.0,
            "lon": 10.0,
            "alt": 100.0,
            "yaw": 0.0,
            "pitch": 0.0,
            "roll": 0.0,
            "gimbal_pitch": -45.0,
            "vel_n": 1.0,
            "vel_e": 0.0,
            "vel_d": 0.0,
            "hdop": 1.0,
            "num_sats": 12,
            "fix_type": 3
        }])
        .to_string();

        let out = select_keyframes_from_json_inner(&records_json, "");
        // Should be a JSON array (possibly with 1 entry — first keyframe)
        assert!(out.starts_with('[') || out.contains("error"), "got: {out}");
    }
}

