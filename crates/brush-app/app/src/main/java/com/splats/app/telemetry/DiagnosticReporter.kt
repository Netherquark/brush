package com.splats.app.telemetry

internal object DiagnosticReporter {
    fun build(
        validated: RowValidator.Result,
        keyframes: List<KeyframeCandidate>,
        syncOffsetUs: Long,
        syncConfidence: Double,
        stageWarnings: List<String>
    ): TelemetryDiagnostics {
        val warnings = mutableListOf<String>()
        warnings += validated.warnings
        warnings += stageWarnings

        if (validated.gpsValidPct < 50.0) {
            warnings += "GPS coverage below 50% (${validated.gpsValidPct.format(1)}%)."
        }
        if (keyframes.size < 8) {
            warnings += "Too few keyframes for SfM (${keyframes.size})."
        }
        if (syncConfidence < 0.4) {
            warnings += "Poor time-sync confidence (${syncConfidence.format(2)})."
        }
        if (validated.rows.isNotEmpty() && validated.imuGapCount.toDouble() / validated.rows.size > 0.1) {
            warnings += "Frequent IMU gaps (${validated.imuGapCount})."
        }

        val okToProceed = keyframes.size >= 8
        return TelemetryDiagnostics(
            totalRecords = validated.rows.size,
            gpsValidPct = validated.gpsValidPct,
            imuGapCount = validated.imuGapCount,
            keyframeCount = keyframes.size,
            syncOffsetUs = syncOffsetUs,
            syncConfidence = syncConfidence,
            warnings = warnings.distinct(),
            okToProceed = okToProceed
        )
    }
}

private fun Double.format(decimals: Int) = "%.${decimals}f".format(this)
