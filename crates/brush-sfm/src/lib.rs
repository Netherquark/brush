//! Telemetry-guided structure-from-motion pipeline for Brush.

pub mod coords;
pub mod instrumentation;
pub mod telemetry;
pub mod vision;

pub use instrumentation::{
    InstrumentationLogger, PairInstrumentationRecord, PairedComparisonReport,
};
