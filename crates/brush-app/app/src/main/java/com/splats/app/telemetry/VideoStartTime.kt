package com.splats.app.telemetry

import android.media.MediaMetadataRetriever
import android.util.Log
import java.io.File
import java.text.SimpleDateFormat
import java.util.Locale
import java.util.TimeZone

private const val TAG_START = "VideoStartTime"

/**
 * Reads creation-time metadata from [videoFile] and returns telemetry-clock microseconds
 * at video start, or null if unavailable (caller may fall back to filename parsing).
 */
internal fun readVideoFileStartTimeUs(videoFile: File): Long? {
    var retriever: MediaMetadataRetriever? = null
    return try {
        retriever = MediaMetadataRetriever()
        retriever.setDataSource(videoFile.absolutePath)
        val rawDate = retriever.extractMetadata(MediaMetadataRetriever.METADATA_KEY_DATE)
        val parsed = parseVideoCreationTimeUs(rawDate)
        if (parsed != null) {
            Log.i(TAG_START, "Video metadata start time for ${videoFile.name}: $rawDate")
        }
        parsed
    } catch (e: Exception) {
        Log.w(TAG_START, "Failed to read video metadata date for ${videoFile.name}", e)
        null
    } finally {
        runCatching { retriever?.release() }
    }
}

private fun parseVideoCreationTimeUs(rawDate: String?): Long? {
    if (rawDate.isNullOrBlank()) return null
    val normalized = rawDate.trim()

    val patterns = listOf(
        "yyyy-MM-dd'T'HH:mm:ss.SSSSSS'Z'",
        "yyyy-MM-dd'T'HH:mm:ss.SSS'Z'",
        "yyyy-MM-dd'T'HH:mm:ss'Z'",
        "yyyyMMdd'T'HHmmss.SSS'Z'",
        "yyyyMMdd'T'HHmmss'Z'",
        "yyyyMMdd'T'HHmmssZ",
        "yyyy-MM-dd HH:mm:ss"
    )

    for (pattern in patterns) {
        val sdf = SimpleDateFormat(pattern, Locale.US)
        sdf.timeZone = TimeZone.getTimeZone("UTC")
        val parsed = runCatching { sdf.parse(normalized) }.getOrNull()
        if (parsed != null) {
            return parsed.time * 1_000L
        }
    }

    return null
}
