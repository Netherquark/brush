//! Per-keyframe pair instrumentation for Mode A (Vision-Only) vs Mode C (Telemetry-Guided).
//!
//! Captures empirical metrics per frame-pair:
//! - `ransac_iterations_used`
//! - `wall_clock_ms`
//! - `inlier_ratio`
//! - `reprojection_rmse_px`

use serde::{Deserialize, Serialize};
use std::fmt::Write as FmtWrite;
use std::io::Write as IoWrite;

use crate::vision::pose::PoseEstimationMode;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PairInstrumentationRecord {
    pub frame_a: usize,
    pub frame_b: usize,
    pub mode: PoseEstimationMode,
    pub ransac_iterations_used: u32,
    pub wall_clock_ms: f64,
    pub inlier_ratio: f64,
    pub reprojection_rmse_px: f64,
    pub matched_features: usize,
    pub inliers: usize,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct InstrumentationLogger {
    pub records: Vec<PairInstrumentationRecord>,
}

impl InstrumentationLogger {
    pub fn new() -> Self {
        Self { records: Vec::new() }
    }

    pub fn log(&mut self, record: PairInstrumentationRecord) {
        self.records.push(record);
    }

    pub fn to_csv(&self) -> String {
        let mut csv = String::new();
        csv.push_str("frame_a,frame_b,mode,ransac_iterations_used,wall_clock_ms,inlier_ratio,reprojection_rmse_px,matched_features,inliers\n");

        for r in &self.records {
            let mode_str = match r.mode {
                PoseEstimationMode::VisionOnly => "ModeA_VisionOnly",
                PoseEstimationMode::TelemetryGuided => "ModeC_TelemetryGuided",
            };
            let _ = writeln!(
                csv,
                "{},{},{},{},{:.4},{:.4},{:.4},{},{}",
                r.frame_a,
                r.frame_b,
                mode_str,
                r.ransac_iterations_used,
                r.wall_clock_ms,
                r.inlier_ratio,
                r.reprojection_rmse_px,
                r.matched_features,
                r.inliers
            );
        }
        csv
    }

    pub fn write_csv<W: IoWrite>(&self, writer: &mut W) -> std::io::Result<()> {
        writer.write_all(self.to_csv().as_bytes())
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PairedComparisonReport {
    pub paired_count: usize,
    pub mode_a_mean_iterations: f64,
    pub mode_c_mean_iterations: f64,
    pub iteration_reduction_ratio: f64,
    pub mode_a_mean_wall_clock_ms: f64,
    pub mode_c_mean_wall_clock_ms: f64,
    pub speedup_factor: f64,
    pub mode_a_mean_inlier_ratio: f64,
    pub mode_c_mean_inlier_ratio: f64,
    pub mode_a_mean_rmse_px: f64,
    pub mode_c_mean_rmse_px: f64,
}

impl PairedComparisonReport {
    pub fn summarize(records: &[PairInstrumentationRecord]) -> Self {
        let mode_a_recs: Vec<_> = records
            .iter()
            .filter(|r| r.mode == PoseEstimationMode::VisionOnly)
            .collect();
        let mode_c_recs: Vec<_> = records
            .iter()
            .filter(|r| r.mode == PoseEstimationMode::TelemetryGuided)
            .collect();

        // Match common pairs (frame_a, frame_b)
        let mut paired_a = Vec::new();
        let mut paired_c = Vec::new();

        for rec_c in &mode_c_recs {
            if let Some(rec_a) = mode_a_recs
                .iter()
                .find(|r| r.frame_a == rec_c.frame_a && r.frame_b == rec_c.frame_b)
            {
                paired_a.push(*rec_a);
                paired_c.push(*rec_c);
            }
        }

        let paired_count = paired_a.len();
        if paired_count == 0 {
            return Self {
                paired_count: 0,
                mode_a_mean_iterations: 0.0,
                mode_c_mean_iterations: 0.0,
                iteration_reduction_ratio: 0.0,
                mode_a_mean_wall_clock_ms: 0.0,
                mode_c_mean_wall_clock_ms: 0.0,
                speedup_factor: 0.0,
                mode_a_mean_inlier_ratio: 0.0,
                mode_c_mean_inlier_ratio: 0.0,
                mode_a_mean_rmse_px: 0.0,
                mode_c_mean_rmse_px: 0.0,
            };
        }

        let mode_a_mean_iterations =
            paired_a.iter().map(|r| r.ransac_iterations_used as f64).sum::<f64>() / paired_count as f64;
        let mode_c_mean_iterations =
            paired_c.iter().map(|r| r.ransac_iterations_used as f64).sum::<f64>() / paired_count as f64;
        let iteration_reduction_ratio = if mode_c_mean_iterations > 0.0 {
            mode_a_mean_iterations / mode_c_mean_iterations
        } else {
            0.0
        };

        let mode_a_mean_wall_clock_ms =
            paired_a.iter().map(|r| r.wall_clock_ms).sum::<f64>() / paired_count as f64;
        let mode_c_mean_wall_clock_ms =
            paired_c.iter().map(|r| r.wall_clock_ms).sum::<f64>() / paired_count as f64;
        let speedup_factor = if mode_c_mean_wall_clock_ms > 0.0 {
            mode_a_mean_wall_clock_ms / mode_c_mean_wall_clock_ms
        } else {
            0.0
        };

        let mode_a_mean_inlier_ratio =
            paired_a.iter().map(|r| r.inlier_ratio).sum::<f64>() / paired_count as f64;
        let mode_c_mean_inlier_ratio =
            paired_c.iter().map(|r| r.inlier_ratio).sum::<f64>() / paired_count as f64;

        let mode_a_mean_rmse_px =
            paired_a.iter().map(|r| r.reprojection_rmse_px).sum::<f64>() / paired_count as f64;
        let mode_c_mean_rmse_px =
            paired_c.iter().map(|r| r.reprojection_rmse_px).sum::<f64>() / paired_count as f64;

        Self {
            paired_count,
            mode_a_mean_iterations,
            mode_c_mean_iterations,
            iteration_reduction_ratio,
            mode_a_mean_wall_clock_ms,
            mode_c_mean_wall_clock_ms,
            speedup_factor,
            mode_a_mean_inlier_ratio,
            mode_c_mean_inlier_ratio,
            mode_a_mean_rmse_px,
            mode_c_mean_rmse_px,
        }
    }

    pub fn to_markdown_table(&self) -> String {
        format!(
            "| Metric | Mode A (Vision-Only) | Mode C (Telemetry-Guided) | Speedup / Reduction |\n\
             | :--- | :---: | :---: | :---: |\n\
             | Paired Frame Pairs Evaluated | {0} | {0} | - |\n\
             | Mean RANSAC Iterations | {1:.1} | {2:.1} | {3:.2}x reduction |\n\
             | Mean Wall Clock Time (ms) | {4:.2} ms | {5:.2} ms | {6:.2}x speedup |\n\
             | Mean Inlier Ratio | {7:.2}% | {8:.2}% | - |\n\
             | Mean Reprojection RMSE (px) | {9:.2} px | {10:.2} px | - |\n",
            self.paired_count,
            self.mode_a_mean_iterations,
            self.mode_c_mean_iterations,
            self.iteration_reduction_ratio,
            self.mode_a_mean_wall_clock_ms,
            self.mode_c_mean_wall_clock_ms,
            self.speedup_factor,
            self.mode_a_mean_inlier_ratio * 100.0,
            self.mode_c_mean_inlier_ratio * 100.0,
            self.mode_a_mean_rmse_px,
            self.mode_c_mean_rmse_px
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn logs_and_exports_csv() {
        let mut logger = InstrumentationLogger::new();

        logger.log(PairInstrumentationRecord {
            frame_a: 0,
            frame_b: 1,
            mode: PoseEstimationMode::VisionOnly,
            ransac_iterations_used: 1000,
            wall_clock_ms: 45.2,
            inlier_ratio: 0.65,
            reprojection_rmse_px: 1.2,
            matched_features: 200,
            inliers: 130,
        });

        logger.log(PairInstrumentationRecord {
            frame_a: 0,
            frame_b: 1,
            mode: PoseEstimationMode::TelemetryGuided,
            ransac_iterations_used: 100,
            wall_clock_ms: 8.5,
            inlier_ratio: 0.82,
            reprojection_rmse_px: 0.9,
            matched_features: 200,
            inliers: 164,
        });

        let csv = logger.to_csv();
        assert!(csv.contains("frame_a,frame_b,mode,ransac_iterations_used,wall_clock_ms,inlier_ratio"));
        assert!(csv.contains("0,1,ModeA_VisionOnly,1000,45.2000,0.6500,1.2000,200,130"));
        assert!(csv.contains("0,1,ModeC_TelemetryGuided,100,8.5000,0.8200,0.9000,200,164"));
    }

    #[test]
    fn summarizes_paired_comparison_metrics() {
        let rec_a = PairInstrumentationRecord {
            frame_a: 0,
            frame_b: 1,
            mode: PoseEstimationMode::VisionOnly,
            ransac_iterations_used: 1000,
            wall_clock_ms: 50.0,
            inlier_ratio: 0.70,
            reprojection_rmse_px: 1.5,
            matched_features: 100,
            inliers: 70,
        };

        let rec_c = PairInstrumentationRecord {
            frame_a: 0,
            frame_b: 1,
            mode: PoseEstimationMode::TelemetryGuided,
            ransac_iterations_used: 100,
            wall_clock_ms: 5.0,
            inlier_ratio: 0.85,
            reprojection_rmse_px: 1.0,
            matched_features: 100,
            inliers: 85,
        };

        let report = PairedComparisonReport::summarize(&[rec_a, rec_c]);

        assert_eq!(report.paired_count, 1);
        assert_eq!(report.mode_a_mean_iterations, 1000.0);
        assert_eq!(report.mode_c_mean_iterations, 100.0);
        assert_eq!(report.iteration_reduction_ratio, 10.0);
        assert_eq!(report.speedup_factor, 10.0);

        let table = report.to_markdown_table();
        assert!(table.contains("10.00x reduction"));
        assert!(table.contains("10.00x speedup"));
    }
}
