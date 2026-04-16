package com.splats.app.telemetry

import java.io.File
import kotlin.math.roundToInt

/**
 * Plans keyframe sample times (microseconds **relative to video metadata start**)
 * using the same CSV ingest + [selectKeyframes] path as [TelemetryPreprocessor].
 */
object KeyframePlanner {

    /**
     * @param maxOutputFrames caps output length (uniform subsample if more keyframes are selected).
     */
    @JvmStatic
    fun videoRelativeKeyframeTimesUs(
        csvFile: File,
        videoFile: File,
        config: KeyframeSelectionConfig,
        maxOutputFrames: Int
    ): LongArray {
        require(maxOutputFrames >= 1) { "maxOutputFrames must be >= 1" }

        val parsedRows = CsvParser.parse(csvFile)
        val targetStartUs = readVideoFileStartTimeUs(videoFile)
            ?: CsvParser.parseStartTimeFromFilename(videoFile.name, parsedRows)
        val clippedRows = if (targetStartUs > 0L) {
            parsedRows.filter { it.timestampUs >= targetStartUs }
        } else {
            parsedRows
        }

        val validated = RowValidator.validate(clippedRows)
        if (validated.rows.isEmpty()) return longArrayOf()
        val (_, enuRecords) = EnuConverter.convert(validated.rows)
        val oriented = OrientationFusionEngine.process(GapInterpolator.interpolate(enuRecords)).records

        val cands = selectKeyframes(oriented, config)
        val rel = LongArray(cands.size) { i ->
            (cands[i].timestampUs - targetStartUs).coerceAtLeast(0L)
        }

        return subsampleUniform(rel, maxOutputFrames)
    }

    private fun subsampleUniform(sortedTimes: LongArray, max: Int): LongArray {
        if (sortedTimes.isEmpty()) return sortedTimes
        val cap = max.coerceAtLeast(1)
        if (sortedTimes.size <= cap) return sortedTimes

        val out = LongArray(cap)
        for (i in 0 until cap) {
            val srcIdx = (i * (sortedTimes.size - 1).toDouble() / (cap - 1).coerceAtLeast(1))
                .roundToInt()
                .coerceIn(0, sortedTimes.lastIndex)
            out[i] = sortedTimes[srcIdx]
        }
        return out
    }
}
