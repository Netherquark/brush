package com.splats.app.telemetry

import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.pow
import kotlin.math.sin
import kotlin.math.sqrt

internal object OrientationFusionEngine {
    private const val IMU_GAP_THRESHOLD_US = 200_000L
    private const val MEDIAN_WINDOW = 5
    private const val COMPASS_SPIKE_DEG = 45.0
    private const val COMPASS_SPEED_THRESHOLD_MS = 2.0

    data class Result(
        val records: List<TelRecord>,
        val imuGapCount: Int,
        val compassWarning: Boolean
    )

    fun process(records: List<TelRecord>): Result {
        if (records.isEmpty()) return Result(emptyList(), 0, false)

        val unwrappedYaw = unwrapYaw(records.map { it.yawDeg })
        val filtN = medianFilter(records.map { it.velN })
        val filtE = medianFilter(records.map { it.velE })
        val filtU = medianFilter(records.map { -it.velD })

        var imuGapCount = 0
        val updated = records.mapIndexed { idx, record ->
            val imuGap = idx > 0 && record.timestampUs - records[idx - 1].timestampUs > IMU_GAP_THRESHOLD_US
            if (imuGap) imuGapCount++
            val yawDeg = wrap360(unwrappedYaw[idx])
            val quaternion = buildCameraQuaternion(
                yawDeg = yawDeg,
                pitchDeg = record.pitchDeg,
                rollDeg = record.rollDeg,
                gimbalPitchDeg = record.gimbalPitch
            )
            record.copy(
                tsAligned = record.timestampUs,
                yawDeg = yawDeg,
                velNFiltered = filtN[idx],
                velEFiltered = filtE[idx],
                velUFiltered = filtU[idx],
                qW = quaternion[0],
                qX = quaternion[1],
                qY = quaternion[2],
                qZ = quaternion[3],
                imuGapFlag = imuGap,
                flags = record.flags or if (imuGap) QualityFlag.IMU_GAP else QualityFlag.CLEAN
            )
        }.toMutableList()

        val corruptIndices = mutableSetOf<Int>()
        for (i in 1 until updated.size) {
            val speed = sqrt(
                updated[i].velEFiltered.pow(2) +
                    updated[i].velNFiltered.pow(2) +
                    updated[i].velUFiltered.pow(2)
            )
            if (yawDiffDeg(updated[i - 1].yawDeg, updated[i].yawDeg) > COMPASS_SPIKE_DEG &&
                speed < COMPASS_SPEED_THRESHOLD_MS
            ) {
                corruptIndices += i
            }
        }

        if (corruptIndices.isNotEmpty()) {
            for (index in corruptIndices) {
                val prev = (index - 1 downTo 0).firstOrNull { it !in corruptIndices } ?: continue
                val next = ((index + 1) until updated.size).firstOrNull { it !in corruptIndices } ?: prev
                val t = if (next == prev) 0.0 else {
                    val span = (updated[next].timestampUs - updated[prev].timestampUs).toDouble().coerceAtLeast(1.0)
                    (updated[index].timestampUs - updated[prev].timestampUs) / span
                }
                val repairedYaw = slerpAngle(updated[prev].yawDeg, updated[next].yawDeg, t)
                val repairedQuaternion = buildCameraQuaternion(
                    yawDeg = repairedYaw,
                    pitchDeg = updated[index].pitchDeg,
                    rollDeg = updated[index].rollDeg,
                    gimbalPitchDeg = updated[index].gimbalPitch
                )
                updated[index] = updated[index].copy(
                    yawDeg = repairedYaw,
                    qW = repairedQuaternion[0],
                    qX = repairedQuaternion[1],
                    qY = repairedQuaternion[2],
                    qZ = repairedQuaternion[3],
                    flags = updated[index].flags or QualityFlag.HEADING_INTERPOLATED
                )
            }
        }

        return Result(
            records = updated,
            imuGapCount = imuGapCount,
            compassWarning = corruptIndices.size.toDouble() / updated.size > 0.10
        )
    }

    fun yawDiffDeg(fromDeg: Double, toDeg: Double): Double {
        var d = toDeg - fromDeg
        while (d > 180.0) d -= 360.0
        while (d <= -180.0) d += 360.0
        return abs(d)
    }

    fun buildCameraQuaternion(
        yawDeg: Double,
        pitchDeg: Double,
        rollDeg: Double,
        gimbalPitchDeg: Double
    ): DoubleArray {
        val y = Math.toRadians(yawDeg)
        val p = Math.toRadians(pitchDeg)
        val r = Math.toRadians(rollDeg)
        val gp = Math.toRadians(gimbalPitchDeg)

        val qYaw = axisAngle(doubleArrayOf(0.0, 0.0, 1.0), y)
        val qPitch = axisAngle(doubleArrayOf(0.0, 1.0, 0.0), p)
        val qRoll = axisAngle(doubleArrayOf(1.0, 0.0, 0.0), r)
        val qBody = qMul(qMul(qYaw, qPitch), qRoll)

        val qGimbal = axisAngle(doubleArrayOf(1.0, 0.0, 0.0), gp)
        val qCamNed = qMul(qBody, qGimbal)
        val qNedToEnu = qMul(
            axisAngle(doubleArrayOf(0.0, 0.0, 1.0), Math.PI / 2.0),
            axisAngle(doubleArrayOf(1.0, 0.0, 0.0), Math.PI)
        )
        return normalizeQ(qMul(qNedToEnu, qCamNed))
    }

    private fun unwrapYaw(values: List<Double>): DoubleArray {
        val out = DoubleArray(values.size)
        if (values.isEmpty()) return out
        out[0] = values[0]
        for (i in 1 until values.size) {
            var diff = values[i] - values[i - 1]
            while (diff > 180.0) diff -= 360.0
            while (diff <= -180.0) diff += 360.0
            out[i] = out[i - 1] + diff
        }
        return out
    }

    private fun medianFilter(values: List<Double>): DoubleArray {
        val out = DoubleArray(values.size)
        for (i in values.indices) {
            val start = maxOf(0, i - MEDIAN_WINDOW + 1)
            val window = values.subList(start, i + 1).sorted()
            out[i] = window[window.size / 2]
        }
        return out
    }

    private fun wrap360(deg: Double): Double = ((deg % 360.0) + 360.0) % 360.0

    private fun slerpAngle(a: Double, b: Double, t: Double): Double {
        var diff = (b - a + 360.0) % 360.0
        if (diff > 180.0) diff -= 360.0
        return ((a + diff * t) + 360.0) % 360.0
    }

    private fun axisAngle(axis: DoubleArray, angle: Double): DoubleArray {
        val half = angle * 0.5
        val s = sin(half)
        return doubleArrayOf(cos(half), axis[0] * s, axis[1] * s, axis[2] * s)
    }

    private fun qMul(a: DoubleArray, b: DoubleArray): DoubleArray =
        doubleArrayOf(
            a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
            a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
            a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
            a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]
        )

    private fun normalizeQ(q: DoubleArray): DoubleArray {
        val norm = sqrt(q.sumOf { it * it }).coerceAtLeast(1e-12)
        return doubleArrayOf(q[0] / norm, q[1] / norm, q[2] / norm, q[3] / norm)
    }
}
