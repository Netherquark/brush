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

    fun validate(rows: List<TelRow>): List<TelRow> {
        if (rows.isEmpty()) throw TelemetryError.InsufficientRecords(0)

        // Some DJI exports arrive newest-first. Validate against timestamp order
        // instead of rejecting an otherwise valid descending log wholesale.
        val orderedRows = rows.sortedBy { it.timestampUs }
        val valid = mutableListOf<TelRow>()
        var prevTimestamp = Long.MIN_VALUE
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
            valid += row
            prevTimestamp = row.timestampUs
        }

        val rejectedCount = orderedRows.size - valid.size
        val rejectedPct   = rejectedCount.toDouble() / orderedRows.size * 100.0
        if (rejectedCount > 0) {
            val summary = rejectionCounts.entries
                .filter { it.value > 0 }
                .joinToString(", ") { "${it.key}=${it.value}" }
            Log.i(TAG, "Validated ${valid.size}/${orderedRows.size} telemetry rows; rejected: $summary")
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

        return valid
    }

    private fun rejectionReason(row: TelRow, prevTimestamp: Long): String? {
        // Latitude and longitude range checks
        if (!row.lat.isFinite() || !row.lon.isFinite()) return "non_finite_lat_lon"
        if (row.lat < -90.0 || row.lat > 90.0) return "lat_out_of_range"
        if (row.lon < -180.0 || row.lon > 180.0) return "lon_out_of_range"
        // Altitude plausibility (barometric AGL)
        if (!row.altM.isFinite()) return "non_finite_alt"
        if (row.altM < -50.0 || row.altM > 6000.0) return "alt_out_of_range"
        // Timestamp must be positive and monotonically increasing
        if (row.timestampUs <= 0L) return "timestamp_non_positive"
        if (row.timestampUs <= prevTimestamp) return "timestamp_non_increasing"
        return null
    }
}
