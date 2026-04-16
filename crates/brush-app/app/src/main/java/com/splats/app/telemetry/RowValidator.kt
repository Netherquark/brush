package com.splats.app.telemetry

import android.util.Log

// ─── Row Validator ────────────────────────────────────────────────────────────

/**
 * Stage 3 — Per-row validation.
 *
 * Rejects individual rows that violate range or ordering invariants.
 * If > 20 % of rows are rejected, raises [TelemetryError.ExcessiveRejections].
 * If < 10 valid rows remain, raises [TelemetryError.InsufficientRecords].
 * If no row has a 3D GPS fix (fixType ≥ 3), raises [TelemetryError.NoGpsFix].
 */
internal object RowValidator {
    private const val TAG = "TelemetryRows"

    data class Result(
        val rows: List<TelRow>,
        val rejectedCount: Int,
        val gpsValidPct: Double,
        val imuGapCount: Int,
        val warnings: List<String>
    )

    fun validate(rows: List<TelRow>): Result {
        if (rows.isEmpty()) throw TelemetryError.InsufficientRecords(0)

        val orderedRows = rows.sortedBy { it.timestampUs }
        val valid = mutableListOf<TelRow>()
        var prevTimestamp = Long.MIN_VALUE
        var imuGapCount = 0
        val rejectionCounts = linkedMapOf(
            "non_finite_lat_lon" to 0,
            "lat_out_of_range" to 0,
            "lon_out_of_range" to 0,
            "non_finite_alt" to 0,
            "alt_out_of_range" to 0,
            "timestamp_non_positive" to 0,
            "timestamp_non_increasing" to 0,
        )

        for (row in orderedRows) {
            val rejection = rejectionReason(row, prevTimestamp)
            if (rejection != null) {
                rejectionCounts[rejection] = rejectionCounts.getValue(rejection) + 1
                continue
            }
            if (prevTimestamp != Long.MIN_VALUE && row.timestampUs - prevTimestamp > 200_000L) {
                imuGapCount++
            }
            valid += row
            prevTimestamp = row.timestampUs
        }

        val rejectedCount = orderedRows.size - valid.size
        val rejectedPct   = rejectedCount.toDouble() / orderedRows.size * 100.0
        val gpsValidCount = valid.count { isGpsUsable(it) }
        val gpsValidPct = if (valid.isEmpty()) 0.0 else gpsValidCount.toDouble() / valid.size * 100.0
        val warnings = mutableListOf<String>()
        if (rejectedCount > 0) {
            val summary = rejectionCounts.entries
                .filter { it.value > 0 }
                .joinToString(", ") { "${it.key}=${it.value}" }
            Log.i(TAG, "Validated ${valid.size}/${orderedRows.size} telemetry rows; rejected: $summary")
            warnings += "Rejected $rejectedCount row(s): $summary"
        }

        if (rejectedPct > 20.0) {
            throw TelemetryError.ExcessiveRejections(rejectedPct)
        }
        if (valid.size < 10) {
            throw TelemetryError.InsufficientRecords(valid.size)
        }
        if (valid.none { it.fixType >= 3 }) {
            throw TelemetryError.NoGpsFix
        }

        return Result(
            rows = valid,
            rejectedCount = rejectedCount,
            gpsValidPct = gpsValidPct,
            imuGapCount = imuGapCount,
            warnings = warnings
        )
    }

    private fun rejectionReason(row: TelRow, prevTimestamp: Long): String? {
        if (!row.lat.isFinite() || !row.lon.isFinite()) return "non_finite_lat_lon"
        if (row.lat < -90.0 || row.lat > 90.0) return "lat_out_of_range"
        if (row.lon < -180.0 || row.lon > 180.0) return "lon_out_of_range"
        if (!row.altM.isFinite()) return "non_finite_alt"
        if (row.altM < -50.0 || row.altM > 6000.0) return "alt_out_of_range"
        if (row.timestampUs <= 0L) return "timestamp_non_positive"
        if (row.timestampUs <= prevTimestamp) return "timestamp_non_increasing"
        return null
    }

    fun isGpsUsable(row: TelRow): Boolean =
        !(row.lat == 0.0 && row.lon == 0.0) &&
            row.lat.isFinite() &&
            row.lon.isFinite() &&
            row.altM.isFinite() &&
            row.fixType >= 3 &&
            (row.hdop <= 0.0 || row.hdop <= 3.0)
}
