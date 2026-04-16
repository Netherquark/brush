package com.splats.app.telemetry

// ─── Interpolator ─────────────────────────────────────────────────────────────

/**
 * Stage 7 — Pose interpolation at keyframe timestamps.
 *
 * For each keyframe timestamp [keyframeTimestampsUs], finds the two nearest
 * [TelRecord] entries in the aligned telemetry and interpolates:
 *
 *  - Position (enuE, enuN, enuU):  linear interpolation
 *  - Heading:                      shortest-path SLERP on the unit circle
 *  - Gimbal pitch:                 linear interpolation
 *  - Velocity (velE, velN, velU):  linear interpolation
 *  - HDOP:                         take the *worse* (larger) of the two bracketing values
 *
 * Timestamps that fall outside the telemetry range extrapolate from the
 * nearest endpoint and set [QualityFlag.EXTRAPOLATED].
 */
internal object Interpolator {

    fun interpolate(
        records: List<TelRecord>,
        keyframeTimestampsUs: LongArray,
        existingFlags: IntArray   // per-record flags from orientation/validation stages
    ): List<PoseStamp> {
        require(records.isNotEmpty()) { "interpolate: records must not be empty" }

        return keyframeTimestampsUs.mapIndexed { frameIndex, kfTs ->
            interpolateAt(frameIndex, kfTs, records, existingFlags)
        }
    }

    private fun interpolateAt(
        frameIndex: Int,
        tsUs: Long,
        records: List<TelRecord>,
        existingFlags: IntArray
    ): PoseStamp {
        // Binary-search for the bracketing records.
        val hi = records.indexOfFirst { it.tsAligned >= tsUs }

        return when {
            hi < 0 -> {
                // After last record — extrapolate from the end.
                val rec = records.last()
                buildPoseStamp(frameIndex, tsUs, rec, rec, 1.0,
                    existingFlags.last() or QualityFlag.EXTRAPOLATED)
            }
            hi == 0 -> {
                // Before first record — extrapolate from the start.
                val rec = records.first()
                buildPoseStamp(frameIndex, tsUs, rec, rec, 0.0,
                    existingFlags.first() or QualityFlag.EXTRAPOLATED)
            }
            else -> {
                val lo = hi - 1
                val recLo = records[lo]
                val recHi = records[hi]
                val span  = (recHi.tsAligned - recLo.tsAligned).toDouble()
                val t     = if (span > 0) (tsUs - recLo.tsAligned) / span else 0.0
                val flags = existingFlags[lo] or existingFlags[hi]  // union of both
                buildPoseStamp(frameIndex, tsUs, recLo, recHi, t, flags)
            }
        }
    }

    private fun buildPoseStamp(
        frameIndex: Int,
        ptsUs: Long,
        lo: TelRecord,
        hi: TelRecord,
        t: Double,
        flags: Int
    ): PoseStamp {
        val headingInterp = slerpHeading(lo.yawDeg, hi.yawDeg, t)
        val hdop          = maxOf(lo.hdop, hi.hdop)   // take the worse value
        val covPosition   = hdop * if (flags and QualityFlag.POOR_GPS != 0) 2.5 else if (lo.isInterpolated || hi.isInterpolated) 4.0 else 1.5

        val resultFlags   = if (lo.yawDeg != hi.yawDeg)
            flags or QualityFlag.HEADING_INTERPOLATED
        else
            flags

        return PoseStamp(
            frameIndex  = frameIndex,
            ptsUs       = ptsUs,
            enuE        = lerp(lo.enuE, hi.enuE, t),
            enuN        = lerp(lo.enuN, hi.enuN, t),
            enuU        = lerp(lo.enuU, hi.enuU, t),
            headingDeg  = headingInterp,
            gimbalPitch = lerp(lo.gimbalPitch, hi.gimbalPitch, t),
            velE        = lerp(lo.velEFiltered, hi.velEFiltered, t),
            velN        = lerp(lo.velNFiltered, hi.velNFiltered, t),
            velU        = lerp(lo.velUFiltered, hi.velUFiltered, t),
            hdop        = hdop,
            covPosition = covPosition,
            covHeading  = 2.0,   // fixed 2.0° for DJI
            flags       = resultFlags,
            qW          = lerp(lo.qW, hi.qW, t),
            qX          = lerp(lo.qX, hi.qX, t),
            qY          = lerp(lo.qY, hi.qY, t),
            qZ          = lerp(lo.qZ, hi.qZ, t),
            trigger     = hi.keyframeTrigger ?: lo.keyframeTrigger ?: KeyframeTrigger.TIME
        )
    }

    // ── Helpers ───────────────────────────────────────────────────────────────

    private fun lerp(a: Double, b: Double, t: Double): Double = a + (b - a) * t

    /**
     * Shortest-path SLERP between two compass headings (degrees).
     * Returns a value in [0°, 360°).
     */
    private fun slerpHeading(h1: Double, h2: Double, t: Double): Double {
        var diff = (h2 - h1 + 360.0) % 360.0
        if (diff > 180.0) diff -= 360.0
        return ((h1 + diff * t) + 360.0) % 360.0
    }
}
