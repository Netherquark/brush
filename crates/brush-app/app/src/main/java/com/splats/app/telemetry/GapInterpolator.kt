package com.splats.app.telemetry

internal object GapInterpolator {
    fun interpolate(records: List<TelRecord>): List<TelRecord> {
        if (records.isEmpty()) return records
        val out = records.toMutableList()

        for (i in out.indices) {
            if (out[i].gpsOk) continue
            val prev = (i - 1 downTo 0).firstOrNull { out[it].gpsOk }
            val next = ((i + 1) until out.size).firstOrNull { out[it].gpsOk }

            out[i] = when {
                prev != null && next != null -> interpolateBetween(out[prev], out[next], out[i])
                prev != null -> extrapolateFrom(out[prev], out[i])
                else -> out[i]
            }
        }

        return out
    }

    private fun interpolateBetween(lo: TelRecord, hi: TelRecord, current: TelRecord): TelRecord {
        val span = (hi.timestampUs - lo.timestampUs).toDouble().coerceAtLeast(1.0)
        val t = ((current.timestampUs - lo.timestampUs) / span).coerceIn(0.0, 1.0)
        return current.copy(
            enuE = lerp(lo.enuE, hi.enuE, t),
            enuN = lerp(lo.enuN, hi.enuN, t),
            enuU = lerp(lo.enuU, hi.enuU, t),
            yawDeg = slerpAngle(lo.yawDeg, hi.yawDeg, t),
            gimbalPitch = lerp(lo.gimbalPitch, hi.gimbalPitch, t),
            isInterpolated = true,
            flags = current.flags or QualityFlag.EXTRAPOLATED
        )
    }

    private fun extrapolateFrom(anchor: TelRecord, current: TelRecord): TelRecord {
        val dtSeconds = (current.timestampUs - anchor.timestampUs).toDouble() / 1_000_000.0
        return current.copy(
            enuE = anchor.enuE + anchor.velEFiltered * dtSeconds,
            enuN = anchor.enuN + anchor.velNFiltered * dtSeconds,
            enuU = anchor.enuU + anchor.velUFiltered * dtSeconds,
            yawDeg = anchor.yawDeg,
            gimbalPitch = anchor.gimbalPitch,
            isInterpolated = true,
            flags = current.flags or QualityFlag.EXTRAPOLATED
        )
    }

    private fun lerp(a: Double, b: Double, t: Double): Double = a + (b - a) * t

    private fun slerpAngle(a: Double, b: Double, t: Double): Double {
        var diff = (b - a + 360.0) % 360.0
        if (diff > 180.0) diff -= 360.0
        return ((a + diff * t) + 360.0) % 360.0
    }
}
