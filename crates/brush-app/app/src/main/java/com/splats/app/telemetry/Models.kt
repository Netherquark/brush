package com.splats.app.telemetry

import java.io.File

data class TelRow(
    val sourceRowIndex: Int,
    val timestampUs:    Long,
    val lat:            Double,
    val lon:            Double,
    val altM:           Double,
    val hdop:           Double,
    val pitchDeg:       Double,
    val rollDeg:        Double,
    val yawDeg:         Double,
    val gimbalPitch:    Double,
    val gimbalYaw:      Double,
    val velN:           Double,
    val velE:           Double,
    val velD:           Double,
    val fixType:        Int,
    val satellites:     Int,
    val batteryV:       Double? = null
)

data class TelRecord(
    val sourceRowIndex: Int,
    val timestampUs:    Long,
    val tsAligned:      Long,
    val lat:            Double,
    val lon:            Double,
    val altM:           Double,
    val hdop:           Double,
    val pitchDeg:       Double,
    val rollDeg:        Double,
    val yawDeg:         Double,
    val gimbalPitch:    Double,
    val gimbalYaw:      Double,
    val velN:           Double,
    val velE:           Double,
    val velD:           Double,
    val velNFiltered:   Double,
    val velEFiltered:   Double,
    val velUFiltered:   Double,
    val enuE:           Double,
    val enuN:           Double,
    val enuU:           Double,
    val qW:             Double,
    val qX:             Double,
    val qY:             Double,
    val qZ:             Double,
    val fixType:        Int,
    val satellites:     Int,
    val batteryV:       Double? = null,
    val gpsOk:          Boolean = true,
    val imuGapFlag:     Boolean = false,
    val isInterpolated: Boolean = false,
    val keyframeTrigger: KeyframeTrigger? = null,
    val flags:          Int = QualityFlag.CLEAN
)

data class PoseStamp(
    val frameIndex:  Int,
    val ptsUs:       Long,
    val enuE:        Double,
    val enuN:        Double,
    val enuU:        Double,
    val headingDeg:  Double,
    val gimbalPitch: Double,
    val velE:        Double,
    val velN:        Double,
    val velU:        Double,
    val hdop:        Double,
    val covPosition: Double,
    val covHeading:  Double,
    val flags:       Int,
    val qW:          Double = 1.0,
    val qX:          Double = 0.0,
    val qY:          Double = 0.0,
    val qZ:          Double = 0.0,
    val trigger:     KeyframeTrigger = KeyframeTrigger.TIME
)

enum class KeyframeTrigger { FIRST, DISTANCE, YAW, PITCH, TIME }

data class TelemetryDiagnostics(
    val totalRecords: Int,
    val gpsValidPct: Double,
    val imuGapCount: Int,
    val keyframeCount: Int,
    val syncOffsetUs: Long,
    val syncConfidence: Double,
    val warnings: List<String>,
    val okToProceed: Boolean
)

// ─── Quality flags as bitmask constants ──────────────────────────────────────

object QualityFlag {
    const val CLEAN                = 0x0000
    const val POOR_GPS             = 0x0001
    const val NO_FIX               = 0x0002  // hard exclusion
    const val IMU_GAP              = 0x0004
    const val IMPLAUSIBLE_VELOCITY = 0x0008  // hard exclusion
    const val IMPLAUSIBLE_GIMBAL   = 0x0010
    const val NEGATIVE_ALTITUDE    = 0x0020
    const val EXTRAPOLATED         = 0x0040
    const val LARGE_SCENE_COORD    = 0x0080
    const val HEADING_INTERPOLATED = 0x0100
}

// ─── ENU origin anchor ───────────────────────────────────────────────────────

data class EnuOrigin(
    val lat0:        Double,    // WGS84 latitude of origin
    val lon0:        Double,    // WGS84 longitude of origin
    val alt0:        Double,    // barometric altitude of origin, metres
    val cosLat0:     Double,    // precomputed cos(lat0 × π/180)
    val timestampUs: Long       // timestamp of the anchor record
)

// ─── Final sequence output ────────────────────────────────────────────────────

data class PoseStampSequence(
    val origin:       EnuOrigin,
    val timeOffsetUs: Long,
    val records:      List<PoseStamp>,
    val sourceMode:   TelemetryMode,
    val logPath:      File,
    val videoPath:    File,
    val createdAt:    Long,
    val diagnostics:  TelemetryDiagnostics? = null
)

enum class TelemetryMode { MODE_A, MODE_B, MODE_C }

// ─── Processing summary ───────────────────────────────────────────────────────

data class TelemetryProcessingReport(
    val totalCsvRows:     Int,
    val rejectedRows:     Int,
    val totalKeyframes:   Int,
    val outputFrames:     Int,          // after hard gate exclusions
    val excludedFrames:   Int,
    val flaggedFrames:    Int,          // soft warnings, still in output
    val syncOffsetMs:     Double,
    val syncCorrelation:  Double,       // 0.0–1.0; > 0.6 required
    val origin:           EnuOrigin,
    val processingTimeMs: Long,
    val warnings:         List<String>,
    val gpsValidPct:      Double = 0.0,
    val imuGapCount:      Int = 0,
    val okToProceed:      Boolean = true
)

// ─── Processing stages ────────────────────────────────────────────────────────

enum class ProcessingStage(val label: String) {
    PARSING("Parsing CSV"),
    VALIDATING("Validating telemetry"),
    CONVERTING("Converting coordinates"),
    ORIENTING("Fusing orientation"),
    SYNCING("Synchronising timelines"),
    INTERPOLATING("Interpolating poses"),
    FLAGGING("Applying quality gates"),
    EMITTING("Writing output")
}
