//! Time Synchronization Engine.
//!
//! ### Scope & Synchronization Approach
//! - **Pragmatic MVP Alignment**: Uses video file creation/start timestamp + frame index
//!   vs telemetry row timestamps, linearly aligned via [`LinearTimeSync`].
//! - **Deliberate Scope Decision**: Barometric-altitude vs optical-flow vertical motion
//!   cross-correlation is reserved as a research-grade extension if compute permits.
//!   Linear alignment provides a deterministic, lightweight MVP suitable for on-device processing.

use serde::{Deserialize, Serialize};
use thiserror::Error;

use super::{
    enu::Enu,
    validate::ValidatedTelemetryRecord,
};

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct LinearTimeSync {
    /// Video creation/capture-start timestamp, in seconds.
    pub video_start_timestamp: f64,
    /// Telemetry timestamp corresponding to the video start, in seconds.
    ///
    /// For absolute telemetry clocks this is usually the same as
    /// `video_start_timestamp`. For relative telemetry logs, use the first
    /// telemetry row timestamp. Cross-correlation alignment is deliberately
    /// left for the research extension; this MVP uses one linear offset.
    pub telemetry_start_timestamp: f64,
    pub frames_per_second: f64,
}

impl LinearTimeSync {
    pub fn telemetry_timestamp_for_frame(&self, frame_index: u64) -> Result<f64, SyncTelemetryError> {
        if self.frames_per_second <= 0.0 {
            return Err(SyncTelemetryError::InvalidFrameRate(self.frames_per_second));
        }

        let video_timestamp =
            frame_video_timestamp(self.video_start_timestamp, frame_index, self.frames_per_second)?;

        Ok(self.telemetry_start_timestamp + (video_timestamp - self.video_start_timestamp))
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct PoseStamp {
    pub frame_index: u64,
    pub video_timestamp: f64,
    pub telemetry_timestamp: f64,
    pub position_enu: Enu,
    pub yaw: Option<f64>,
    pub pitch: Option<f64>,
    pub roll: Option<f64>,
    pub gimbal_pitch: Option<f64>,
}

#[derive(Clone, Debug, Error, PartialEq)]
pub enum SyncTelemetryError {
    #[error("frames_per_second must be positive, got {0}")]
    InvalidFrameRate(f64),
    #[error("at least one validated telemetry record is required")]
    EmptyTelemetry,
    #[error("telemetry records must be sorted by timestamp")]
    TelemetryNotSorted,
    #[error("frame {frame_index} maps to telemetry timestamp {telemetry_timestamp}, outside telemetry range {start}..={end}")]
    FrameOutsideTelemetryRange {
        frame_index: u64,
        telemetry_timestamp: f64,
        start: f64,
        end: f64,
    },
}

pub fn frame_video_timestamp(
    video_start_timestamp: f64,
    frame_index: u64,
    frames_per_second: f64,
) -> Result<f64, SyncTelemetryError> {
    if frames_per_second <= 0.0 {
        return Err(SyncTelemetryError::InvalidFrameRate(frames_per_second));
    }
    Ok(video_start_timestamp + frame_index as f64 / frames_per_second)
}

pub fn pose_stamp_for_frame(
    telemetry: &[ValidatedTelemetryRecord],
    sync: LinearTimeSync,
    frame_index: u64,
) -> Result<PoseStamp, SyncTelemetryError> {
    ensure_sorted(telemetry)?;

    let telemetry_timestamp = sync.telemetry_timestamp_for_frame(frame_index)?;
    let start = telemetry[0].raw.timestamp;
    let end = telemetry[telemetry.len() - 1].raw.timestamp;
    if telemetry_timestamp < start || telemetry_timestamp > end {
        return Err(SyncTelemetryError::FrameOutsideTelemetryRange {
            frame_index,
            telemetry_timestamp,
            start,
            end,
        });
    }

    let video_timestamp = frame_video_timestamp(
        sync.video_start_timestamp,
        frame_index,
        sync.frames_per_second,
    )?;

    let (a, b) = bracketing_records(telemetry, telemetry_timestamp);
    let t = interpolation_factor(a.raw.timestamp, b.raw.timestamp, telemetry_timestamp);

    Ok(PoseStamp {
        frame_index,
        video_timestamp,
        telemetry_timestamp,
        position_enu: interpolate_enu(a.enu, b.enu, t),
        yaw: interpolate_angle(a.raw.yaw, b.raw.yaw, t),
        pitch: interpolate_optional(a.raw.pitch, b.raw.pitch, t),
        roll: interpolate_optional(a.raw.roll, b.raw.roll, t),
        gimbal_pitch: interpolate_optional(a.raw.gimbal_pitch, b.raw.gimbal_pitch, t),
    })
}

pub fn pose_stamps_for_frames(
    telemetry: &[ValidatedTelemetryRecord],
    sync: LinearTimeSync,
    frame_indices: impl IntoIterator<Item = u64>,
) -> Result<Vec<PoseStamp>, SyncTelemetryError> {
    frame_indices
        .into_iter()
        .map(|frame_index| pose_stamp_for_frame(telemetry, sync, frame_index))
        .collect()
}

fn ensure_sorted(telemetry: &[ValidatedTelemetryRecord]) -> Result<(), SyncTelemetryError> {
    if telemetry.is_empty() {
        return Err(SyncTelemetryError::EmptyTelemetry);
    }

    if telemetry
        .windows(2)
        .any(|pair| pair[0].raw.timestamp > pair[1].raw.timestamp)
    {
        return Err(SyncTelemetryError::TelemetryNotSorted);
    }

    Ok(())
}

fn bracketing_records(
    telemetry: &[ValidatedTelemetryRecord],
    timestamp: f64,
) -> (&ValidatedTelemetryRecord, &ValidatedTelemetryRecord) {
    match telemetry.binary_search_by(|record| {
        record
            .raw
            .timestamp
            .partial_cmp(&timestamp)
            .unwrap_or(std::cmp::Ordering::Less)
    }) {
        Ok(idx) => (&telemetry[idx], &telemetry[idx]),
        Err(idx) => (&telemetry[idx - 1], &telemetry[idx]),
    }
}

fn interpolation_factor(a: f64, b: f64, value: f64) -> f64 {
    let span = b - a;
    if span.abs() <= f64::EPSILON {
        0.0
    } else {
        (value - a) / span
    }
}

fn interpolate_enu(a: Enu, b: Enu, t: f64) -> Enu {
    Enu {
        e: lerp(a.e, b.e, t),
        n: lerp(a.n, b.n, t),
        u: lerp(a.u, b.u, t),
    }
}

fn interpolate_optional(a: Option<f64>, b: Option<f64>, t: f64) -> Option<f64> {
    Some(lerp(a?, b?, t))
}

fn interpolate_angle(a: Option<f64>, b: Option<f64>, t: f64) -> Option<f64> {
    let a = a?;
    let b = b?;
    let delta = shortest_angle_delta(a, b);
    Some(normalize_degrees(a + delta * t))
}

fn shortest_angle_delta(from: f64, to: f64) -> f64 {
    (to - from + 180.0).rem_euclid(360.0) - 180.0
}

fn normalize_degrees(angle: f64) -> f64 {
    angle.rem_euclid(360.0)
}

fn lerp(a: f64, b: f64, t: f64) -> f64 {
    a + (b - a) * t
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::telemetry::{
        GpsQualityGate, RawTelemetryRecord, Wgs84Origin, validate_records,
    };

    fn raw(timestamp: f64, lat_offset_m: f64, yaw: f64) -> RawTelemetryRecord {
        RawTelemetryRecord {
            timestamp,
            lat: 12.9716 + lat_offset_m / 111_320.0,
            lon: 77.5946,
            alt: 920.0,
            yaw: Some(yaw),
            pitch: Some(-1.0),
            roll: Some(0.2),
            gimbal_pitch: Some(-35.0),
            vel_n: Some(1.0),
            vel_e: Some(0.0),
            vel_d: Some(0.0),
            hdop: Some(0.8),
            num_sats: Some(12),
        }
    }

    fn telemetry() -> Vec<ValidatedTelemetryRecord> {
        let records = [raw(10.0, 0.0, 350.0), raw(11.0, 10.0, 10.0)];
        let (accepted, rejected) = validate_records(
            &records,
            Wgs84Origin {
                lat: 12.9716,
                lon: 77.5946,
                alt: 920.0,
            },
            GpsQualityGate::default(),
        );
        assert!(rejected.is_empty());
        accepted
    }

    #[test]
    fn maps_frame_index_to_linear_telemetry_time() {
        let sync = LinearTimeSync {
            video_start_timestamp: 1_700_000_000.0,
            telemetry_start_timestamp: 10.0,
            frames_per_second: 30.0,
        };

        let timestamp = sync.telemetry_timestamp_for_frame(15).unwrap();

        assert!((timestamp - 10.5).abs() < 1e-9);
    }

    #[test]
    fn interpolates_pose_stamp_for_frame() {
        let pose = pose_stamp_for_frame(
            &telemetry(),
            LinearTimeSync {
                video_start_timestamp: 100.0,
                telemetry_start_timestamp: 10.0,
                frames_per_second: 10.0,
            },
            5,
        )
        .unwrap();

        assert_eq!(pose.frame_index, 5);
        assert!((pose.video_timestamp - 100.5).abs() < 1e-9);
        assert!((pose.telemetry_timestamp - 10.5).abs() < 1e-9);
        assert!((pose.position_enu.n - 5.0).abs() <= 0.05);
        assert_eq!(pose.yaw, Some(0.0));
    }

    #[test]
    fn rejects_frames_outside_telemetry_range() {
        let error = pose_stamp_for_frame(
            &telemetry(),
            LinearTimeSync {
                video_start_timestamp: 100.0,
                telemetry_start_timestamp: 10.0,
                frames_per_second: 10.0,
            },
            11,
        )
        .unwrap_err();

        assert!(matches!(
            error,
            SyncTelemetryError::FrameOutsideTelemetryRange { .. }
        ));
    }

    #[test]
    fn rejects_unsorted_telemetry() {
        let mut telemetry = telemetry();
        telemetry.swap(0, 1);

        let error = pose_stamp_for_frame(
            &telemetry,
            LinearTimeSync {
                video_start_timestamp: 100.0,
                telemetry_start_timestamp: 10.0,
                frames_per_second: 10.0,
            },
            0,
        )
        .unwrap_err();

        assert_eq!(error, SyncTelemetryError::TelemetryNotSorted);
    }
}
