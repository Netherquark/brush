package com.splats.app.telemetry

import java.text.SimpleDateFormat
import java.util.Locale
import java.util.TimeZone

/**
 * Extracts and interpolates video start timestamps from filenames and metadata.
 */
object VideoTimeExtractor {
    private val filenameDateTimeRegex = Regex("""(\d{4}-\d{2}-\d{2})[ _](\d{2})[:_-](\d{2})[:_-](\d{2})""")
    private val filenameTimeRegex = Regex("""(?<!\d)(\d{2})[:_-](\d{2})[:_-](\d{2})(?!\d)""")

    /**
     * Determines the video's absolute start time in microseconds, given its filename
     * and a base telemetry anchor timestamp (used to provide the date if the filename only has time).
     */
    fun extractTargetStartUs(filename: String, baseDateUs: Long): Long {
        val dateTimeMatch = filenameDateTimeRegex.find(filename)
        if (dateTimeMatch != null) {
            val (date, hr, min, sec) = dateTimeMatch.destructured
            val sdf = SimpleDateFormat("yyyy-MM-dd HH:mm:ss", Locale.US)
            sdf.timeZone = TimeZone.getTimeZone("UTC")
            return try {
                sdf.parse("$date $hr:$min:$sec")?.time?.times(1_000L) ?: 0L
            } catch (e: Exception) {
                0L
            }
        }

        if (baseDateUs == 0L) return 0L
        val timeMatch = filenameTimeRegex.find(filename) ?: return 0L
        val (hr, min, sec) = timeMatch.destructured
        val sdf = SimpleDateFormat("yyyy-MM-dd HH:mm:ss", Locale.US)
        sdf.timeZone = TimeZone.getTimeZone("UTC")
        return try {
            val dayStartUs = (baseDateUs / 86_400_000_000L) * 86_400_000_000L
            val dayStart = sdf.format(java.util.Date(dayStartUs / 1_000L))
                .substring(0, 10)
            sdf.parse("$dayStart $hr:$min:$sec")?.time?.times(1_000L) ?: 0L
        } catch (e: Exception) {
            0L
        }
    }
}
