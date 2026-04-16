package com.splats.app.telemetry

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test
import java.io.File
import kotlin.math.abs
import kotlin.math.sqrt

class TelemetryPipelineTest {
    @Test
    fun csvParserHandlesSpecStyleHeaders() {
        val csv = File.createTempFile("telemetry", ".csv")
        csv.writeText(
            """
            time(millisecond),latitude,longitude,altitude(m),compass_heading(degrees),pitch(degrees),roll(degrees),gimbal_pitch(degrees),speed_northward(m/s),speed_eastward(m/s),speed_downward(m/s),hdop
            0,18.5204,73.8567,554.0,355.0,1.0,2.0,-90.0,1.0,0.0,0.0,0.9
            1000,18.5205,73.8568,555.0,5.0,1.5,2.5,-85.0,1.0,0.0,0.0,1.1
            """.trimIndent()
        )
        try {
            val rows = CsvParser.parse(csv)
            assertEquals(2, rows.size)
            assertEquals(18.5204, rows.first().lat, 1e-6)
            assertEquals(355.0, rows.first().yawDeg, 1e-6)
        } finally {
            csv.delete()
        }
    }

    @Test
    fun yawDeltaWrapsAcrossZero() {
        assertEquals(10.0, OrientationFusionEngine.yawDiffDeg(355.0, 5.0), 1e-6)
        assertEquals(10.0, OrientationFusionEngine.yawDiffDeg(5.0, 355.0), 1e-6)
    }

    @Test
    fun cameraQuaternionIsNormalised() {
        val q = OrientationFusionEngine.buildCameraQuaternion(0.0, 0.0, 0.0, -90.0)
        val norm = sqrt(q.sumOf { it * it })
        assertTrue(abs(norm - 1.0) < 1e-6)
    }

    @Test
    fun keyframeSelectorTriggersOnYawWrap() {
        val rows = listOf(
            testRecord(timestampUs = 0L, yawDeg = 355.0),
            testRecord(timestampUs = 1_000_000L, yawDeg = 5.0)
        )
        val keyframes = selectKeyframes(rows, KeyframeSelectionConfig(yawThresholdDeg = 8.0))
        assertTrue(keyframes.any { it.triggerReason == KeyframeTrigger.YAW })
    }

    @Test
    fun flowProxySampleRateIsPointTwoHz() {
        val samples = TimeSyncEngine.flowProxySampleTimesUs(10 * 60 * 1000L)
        assertEquals(120, samples.size)
        assertEquals(5_000_000L, samples[1] - samples[0])
    }

    private fun testRecord(timestampUs: Long, yawDeg: Double): TelRecord =
        TelRecord(
            sourceRowIndex = 0,
            timestampUs = timestampUs,
            tsAligned = timestampUs,
            lat = 18.5204,
            lon = 73.8567,
            altM = 554.0,
            hdop = 0.9,
            pitchDeg = 0.0,
            rollDeg = 0.0,
            yawDeg = yawDeg,
            gimbalPitch = -90.0,
            gimbalYaw = 0.0,
            velN = 1.0,
            velE = 0.0,
            velD = 0.0,
            velNFiltered = 1.0,
            velEFiltered = 0.0,
            velUFiltered = 0.0,
            enuE = 0.0,
            enuN = 0.0,
            enuU = 0.0,
            qW = 1.0,
            qX = 0.0,
            qY = 0.0,
            qZ = 0.0,
            fixType = 3,
            satellites = 12
        )
}
