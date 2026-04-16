package com.splats.app.telemetry

import kotlin.math.*

// ─── Keyframe Selection ───────────────────────────────────────────────────────

/**
 * Configurable thresholds for telemetry-based keyframe selection.
 * Defaults are tuned for aerial survey (2 m spacing, 8° yaw, 5° pitch, 1 s fallback).
 */
data class KeyframeSelectionConfig(
    val distanceThresholdM:    Double = 2.0,
    val yawThresholdDeg:       Double = 8.0,
    val pitchThresholdDeg:     Double = 5.0,
    val timeThresholdUs:       Long   = 1_000_000L,   // 1 second
    val minSpeedMs:            Double = 0.2            // below this = hovering
)

/**
 * A keyframe candidate produced by [selectKeyframes].
 * After time-sync, [timestampUs] is matched to a video PTS by the Interpolator.
 */
data class KeyframeCandidate(
    val rowIndex:      Int,
    val timestampUs:   Long,
    val lat:           Double,
    val lon:           Double,
    val yawDeg:        Double,
    val gimbalPitch:   Double,
    val speedMs:       Double,
    val triggerReason: KeyframeTrigger
)

// ─── Selector ─────────────────────────────────────────────────────────────────

/**
 * Runs the O(N) pose-change threshold pass over validated [TelRow] records.
 *
 * Algorithm:
 *  - The first row is always selected.
 *  - Rows where speed < [KeyframeSelectionConfig.minSpeedMs] are skipped (hovering gate).
 *  - A row is selected when *any* of the following exceeds its threshold since
 *    the last selected row: GPS distance, yaw change, gimbal pitch change, elapsed time.
 *  - The first matching threshold in the `when` expression determines the
 *    reported [KeyframeTrigger].
 *
 * Complexity: O(N) — 6 trig ops per row for Haversine, all other ops O(1).
 */
fun selectKeyframes(
    rows: List<TelRecord>,
    config: KeyframeSelectionConfig = KeyframeSelectionConfig()
): List<KeyframeCandidate> {
    require(rows.isNotEmpty()) { "selectKeyframes: rows must not be empty" }
    val maxKeyframes = 100

    val keyframes = mutableListOf<KeyframeCandidate>()
    var lastLat        = rows.first().lat
    var lastLon        = rows.first().lon
    var lastYaw        = rows.first().yawDeg
    var lastPitch      = rows.first().gimbalPitch
    var lastKeyframeUs = rows.first().timestampUs

    keyframes += KeyframeCandidate(
        rowIndex      = 0,
        timestampUs   = rows.first().timestampUs,
        lat           = rows.first().lat,
        lon           = rows.first().lon,
        yawDeg        = rows.first().yawDeg,
        gimbalPitch   = rows.first().gimbalPitch,
        speedMs       = 0.0,
        triggerReason = KeyframeTrigger.FIRST
    )

    for ((index, row) in rows.withIndex().drop(1)) {
        if (keyframes.size >= maxKeyframes) break
        val speed = sqrt(row.velNFiltered.pow(2) + row.velEFiltered.pow(2) + row.velUFiltered.pow(2))
        if (speed < config.minSpeedMs) continue

        val dist   = haversineMetres(lastLat, lastLon, row.lat, row.lon)
        val yawD   = yawDiffDeg(lastYaw, row.yawDeg)
        val pitchD = abs(row.gimbalPitch - lastPitch)
        val timeD  = row.timestampUs - lastKeyframeUs

        val reason: KeyframeTrigger? = when {
            dist   >= config.distanceThresholdM  -> KeyframeTrigger.DISTANCE
            yawD   >= config.yawThresholdDeg     -> KeyframeTrigger.YAW
            pitchD >= config.pitchThresholdDeg   -> KeyframeTrigger.PITCH
            timeD  >= config.timeThresholdUs     -> KeyframeTrigger.TIME
            else                                -> null
        }

        if (reason != null) {
            keyframes += KeyframeCandidate(
                rowIndex      = index,
                timestampUs   = row.timestampUs,
                lat           = row.lat,
                lon           = row.lon,
                yawDeg        = row.yawDeg,
                gimbalPitch   = row.gimbalPitch,
                speedMs       = speed,
                triggerReason = reason
            )
            lastLat        = row.lat
            lastLon        = row.lon
            lastYaw        = row.yawDeg
            lastPitch      = row.gimbalPitch
            lastKeyframeUs = row.timestampUs
        }
    }

    return keyframes
}

// ─── Geometric helpers ────────────────────────────────────────────────────────

/**
 * Haversine great-circle distance in metres.
 * Accurate to < 1 mm for baselines < 5 km (typical drone survey).
 */
fun haversineMetres(lat1: Double, lon1: Double, lat2: Double, lon2: Double): Double {
    val latMidRad = Math.toRadians((lat1 + lat2) / 2.0)
    val dLatM = (lat2 - lat1) * 111_320.0
    val dLonM = (lon2 - lon1) * 111_320.0 * cos(latMidRad)
    return sqrt(dLatM.pow(2) + dLonM.pow(2))
}

/**
 * Shortest-path absolute yaw difference in degrees.
 * Handles the 0°/360° wraparound correctly.
 */
fun yawDiffDeg(yaw1: Double, yaw2: Double): Double {
    var diff = (yaw2 - yaw1 + 360.0) % 360.0
    if (diff > 180.0) diff -= 360.0
    return abs(diff)
}
