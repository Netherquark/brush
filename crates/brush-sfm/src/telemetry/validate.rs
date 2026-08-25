use serde::{Deserialize, Serialize};

use super::{
    enu::{Enu, Wgs84Origin, wgs84_to_enu},
    parser::RawTelemetryRecord,
};

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct GpsQualityGate {
    pub max_hdop: f64,
    pub min_3d_fix_satellites: u32,
}

impl Default for GpsQualityGate {
    fn default() -> Self {
        Self {
            max_hdop: 3.0,
            min_3d_fix_satellites: 4,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct ValidatedTelemetryRecord {
    pub raw: RawTelemetryRecord,
    pub enu: Enu,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum TelemetryRejectionReason {
    MissingHdop,
    HdopTooHigh,
    MissingGpsFix,
    Not3dGpsFix,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct RejectedTelemetryRecord {
    pub raw: RawTelemetryRecord,
    pub reason: TelemetryRejectionReason,
}

pub fn validate_records(
    records: &[RawTelemetryRecord],
    origin: Wgs84Origin,
    gate: GpsQualityGate,
) -> (Vec<ValidatedTelemetryRecord>, Vec<RejectedTelemetryRecord>) {
    let mut accepted = Vec::new();
    let mut rejected = Vec::new();

    for &raw in records {
        if let Some(reason) = rejection_reason(raw, gate) {
            rejected.push(RejectedTelemetryRecord { raw, reason });
            continue;
        }

        accepted.push(ValidatedTelemetryRecord {
            raw,
            enu: wgs84_to_enu(raw.lat, raw.lon, raw.alt, origin),
        });
    }

    (accepted, rejected)
}

fn rejection_reason(
    record: RawTelemetryRecord,
    gate: GpsQualityGate,
) -> Option<TelemetryRejectionReason> {
    match record.hdop {
        Some(hdop) if hdop > gate.max_hdop => Some(TelemetryRejectionReason::HdopTooHigh),
        None => Some(TelemetryRejectionReason::MissingHdop),
        _ => match record.num_sats {
            Some(sats) if sats >= gate.min_3d_fix_satellites => None,
            Some(_) => Some(TelemetryRejectionReason::Not3dGpsFix),
            None => Some(TelemetryRejectionReason::MissingGpsFix),
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn record(timestamp: f64, hdop: Option<f64>, num_sats: Option<u32>) -> RawTelemetryRecord {
        RawTelemetryRecord {
            timestamp,
            lat: 12.9716 + timestamp * 0.00001,
            lon: 77.5946,
            alt: 920.0,
            yaw: Some(45.0),
            pitch: Some(-1.0),
            roll: Some(0.2),
            gimbal_pitch: Some(-35.0),
            vel_n: None,
            vel_e: None,
            vel_d: None,
            hdop,
            num_sats,
        }
    }

    #[test]
    fn rejects_high_hdop_and_non_3d_fix() {
        let records = [
            record(0.0, Some(0.8), Some(12)),
            record(1.0, Some(3.1), Some(12)),
            record(2.0, Some(0.9), Some(3)),
            record(3.0, None, Some(12)),
            record(4.0, Some(0.9), None),
        ];

        let (accepted, rejected) = validate_records(
            &records,
            Wgs84Origin {
                lat: 12.9716,
                lon: 77.5946,
                alt: 920.0,
            },
            GpsQualityGate::default(),
        );

        assert_eq!(accepted.len(), 1);
        assert_eq!(accepted[0].raw.timestamp, 0.0);
        assert_eq!(rejected.len(), 4);
        assert_eq!(rejected[0].reason, TelemetryRejectionReason::HdopTooHigh);
        assert_eq!(rejected[1].reason, TelemetryRejectionReason::Not3dGpsFix);
        assert_eq!(rejected[2].reason, TelemetryRejectionReason::MissingHdop);
        assert_eq!(rejected[3].reason, TelemetryRejectionReason::MissingGpsFix);
    }
}
