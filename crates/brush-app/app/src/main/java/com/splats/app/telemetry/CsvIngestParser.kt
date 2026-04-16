package com.splats.app.telemetry

import android.util.Log
import java.io.File
import java.time.LocalDateTime
import java.time.OffsetDateTime
import java.time.ZoneOffset
import java.time.format.DateTimeFormatter
import java.time.format.DateTimeParseException
import java.util.Locale

private data class HeaderAliases(val aliases: List<String>)

private data class CsvSchema(
    val timestampIdx: Int,
    val latIdx: Int,
    val lonIdx: Int,
    val altIdx: Int,
    val hdopIdx: Int?,
    val pitchIdx: Int?,
    val rollIdx: Int?,
    val yawIdx: Int?,
    val gimbalPitchIdx: Int?,
    val gimbalYawIdx: Int?,
    val velNIdx: Int?,
    val velEIdx: Int?,
    val velDIdx: Int?,
    val fixTypeIdx: Int?,
    val satellitesIdx: Int?,
    val batteryVIdx: Int?,
    val normalizedHeaders: List<String>
)

private object ColumnMap {
    // Litchi export schema only.
    // Prefer absolute clock when present. `time(millisecond)` is elapsed-from-start and
    // must NOT be used for epoch clipping against video metadata.
    val TIMESTAMP = HeaderAliases(listOf("datetime(utc)", "timestamp", "time(millisecond)"))
    val LAT = HeaderAliases(listOf("latitude"))
    val LON = HeaderAliases(listOf("longitude"))
    val ALT = HeaderAliases(listOf("altitude(m)", "altitudeRaw"))
    // Litchi exports do not include HDOP directly; satellites is the closest proxy available.
    val HDOP = HeaderAliases(listOf("satellites"))
    val PITCH = HeaderAliases(listOf("pitch(deg)", "pitchRaw"))
    val ROLL = HeaderAliases(listOf("roll(deg)", "rollRaw"))
    val YAW = HeaderAliases(listOf("yaw(deg)", "yawRaw"))
    val GIMBAL_PITCH = HeaderAliases(listOf("gimbalPitchRaw"))
    val GIMBAL_YAW = HeaderAliases(listOf("gimbalYawRaw"))
    val VEL_N = HeaderAliases(listOf("velocityY(mps)", "velocityYRaw"))
    val VEL_E = HeaderAliases(listOf("velocityX(mps)", "velocityXRaw"))
    val VEL_D = HeaderAliases(listOf("velocityZ(mps)", "velocityZRaw"))
    val FIX_TYPE = HeaderAliases(emptyList())
    val SATELLITES = HeaderAliases(listOf("satellites"))
    val BATTERY_V = HeaderAliases(listOf("voltage(v)", "currentVoltage"))
}

internal object CsvIngest {
    fun read(file: File): Pair<List<String>, List<List<String>>> {
        if (!file.exists()) throw TelemetryError.CsvNotFound(file.absolutePath)
        val rows = mutableListOf<List<String>>()
        var headers: List<String>? = null
        file.inputStream().bufferedReader(Charsets.UTF_8).useLines { lines ->
            for (raw in lines) {
                val line = raw.trim().removePrefix("\uFEFF")
                if (line.isBlank() || !line.contains(',')) continue
                if (headers == null && looksLikeHeader(line)) {
                    headers = splitCsvLine(line)
                } else if (headers != null) {
                    rows += splitCsvLine(line)
                }
            }
        }
        return (headers ?: throw TelemetryError.InsufficientRecords(0)) to rows
    }
}

internal object CsvParser {
    private const val TAG = "TelemetryCsv"
    private val localDateTimeFormatter: DateTimeFormatter =
        DateTimeFormatter.ofPattern("yyyy-MM-dd HH:mm:ss", Locale.US)
    private val filenameDateTimeRegex = Regex("""(\d{4}-\d{2}-\d{2})[ _](\d{2})[:_-](\d{2})[:_-](\d{2})""")
    private val filenameTimeRegex = Regex("""(?<!\d)(\d{2})[:_-](\d{2})[:_-](\d{2})(?!\d)""")

    fun parse(csvFile: File, targetStartUs: Long = 0L): List<TelRow> {
        if (!csvFile.exists()) throw TelemetryError.CsvNotFound(csvFile.absolutePath)

        val rows = ArrayList<TelRow>(4_000)
        var schema: CsvSchema? = null
        var dataRowIndex = 0

        csvFile.inputStream().bufferedReader(Charsets.UTF_8).useLines { lines ->
            for (raw in lines) {
                val line = raw.trim().removePrefix("\uFEFF")
                if (line.isBlank() || !line.contains(',')) continue

                if (schema == null) {
                    if (!looksLikeHeader(line)) continue
                    val headers = splitCsvLine(line)
                    schema = detectSchema(headers)
                    Log.i(TAG, "Detected CSV schema: ${schema!!.normalizedHeaders.joinToString("|")}")
                    continue
                }

                val parsed = parseFields(splitCsvLine(line), schema!!, dataRowIndex++) ?: continue
                if (targetStartUs <= 0L || parsed.timestampUs >= targetStartUs) {
                    rows += parsed
                }
            }
        }

        if (schema == null) throw TelemetryError.InsufficientRecords(0)
        return rows
    }

    fun parse(headers: List<String>, dataRows: List<List<String>>, targetStartUs: Long = 0L): List<TelRow> {
        val schema = detectSchema(headers)
        return dataRows.mapIndexedNotNull { idx, row ->
            val parsed = parseFields(row, schema, idx) ?: return@mapIndexedNotNull null
            if (targetStartUs > 0L && parsed.timestampUs < targetStartUs) null else parsed
        }
    }

    fun parseStartTimeFromFilename(filename: String, rows: List<TelRow>): Long {
        val dateTimeMatch = filenameDateTimeRegex.find(filename)
        if (dateTimeMatch != null) {
            val (date, hr, min, sec) = dateTimeMatch.destructured
            return try {
                LocalDateTime.parse("$date $hr:$min:$sec", localDateTimeFormatter)
                    .toInstant(ZoneOffset.UTC)
                    .toEpochMilli() * 1_000L
            } catch (_: Exception) {
                0L
            }
        }

        val baseDateUs = rows.firstOrNull { it.timestampUs > 0L }?.timestampUs ?: return 0L
        val timeMatch = filenameTimeRegex.find(filename) ?: return 0L
        val (hr, min, sec) = timeMatch.destructured
        return try {
            val baseDay = java.time.Instant.ofEpochMilli(baseDateUs / 1_000L)
                .atOffset(ZoneOffset.UTC)
                .toLocalDate()
            baseDay.atTime(hr.toInt(), min.toInt(), sec.toInt())
                .toInstant(ZoneOffset.UTC)
                .toEpochMilli() * 1_000L
        } catch (_: Exception) {
            0L
        }
    }

    private fun detectSchema(headers: List<String>): CsvSchema {
        val normalized = headers.map(::normalizeHeader)

        fun resolve(aliases: HeaderAliases, required: Boolean = false): Int? {
            for (alias in aliases.aliases) {
                val needle = normalizeHeader(alias)
                val idx = normalized.indexOfFirst { it.contains(needle) }
                if (idx >= 0) return idx
            }
            if (required) throw TelemetryError.InsufficientRecords(0)
            return null
        }

        return CsvSchema(
            timestampIdx = resolve(ColumnMap.TIMESTAMP, required = true)!!,
            latIdx = resolve(ColumnMap.LAT, required = true)!!,
            lonIdx = resolve(ColumnMap.LON, required = true)!!,
            altIdx = resolve(ColumnMap.ALT, required = true)!!,
            hdopIdx = resolve(ColumnMap.HDOP),
            pitchIdx = resolve(ColumnMap.PITCH),
            rollIdx = resolve(ColumnMap.ROLL),
            yawIdx = resolve(ColumnMap.YAW),
            gimbalPitchIdx = resolve(ColumnMap.GIMBAL_PITCH),
            gimbalYawIdx = resolve(ColumnMap.GIMBAL_YAW),
            velNIdx = resolve(ColumnMap.VEL_N),
            velEIdx = resolve(ColumnMap.VEL_E),
            velDIdx = resolve(ColumnMap.VEL_D),
            fixTypeIdx = resolve(ColumnMap.FIX_TYPE),
            satellitesIdx = resolve(ColumnMap.SATELLITES),
            batteryVIdx = resolve(ColumnMap.BATTERY_V),
            normalizedHeaders = normalized
        )
    }

    private fun parseFields(fields: List<String>, schema: CsvSchema, rowIndex: Int): TelRow? {
        val timestampUs = parseTimestampUs(fields.getOrNull(schema.timestampIdx).orEmpty()) ?: return null
        val lat = parseDouble(fields.getOrNull(schema.latIdx)) ?: return null
        val lon = parseDouble(fields.getOrNull(schema.lonIdx)) ?: return null

        return TelRow(
            sourceRowIndex = rowIndex,
            timestampUs = timestampUs,
            lat = lat,
            lon = lon,
            altM = parseAltitudeMeters(fields.getOrNull(schema.altIdx), schema.normalizedHeaders[schema.altIdx]),
            hdop = schema.hdopIdx?.let { parseHdop(fields.getOrNull(it), schema.normalizedHeaders[it]) } ?: 99.0,
            pitchDeg = parseDouble(schema.pitchIdx?.let(fields::getOrNull)) ?: 0.0,
            rollDeg = parseDouble(schema.rollIdx?.let(fields::getOrNull)) ?: 0.0,
            yawDeg = parseDouble(schema.yawIdx?.let(fields::getOrNull)) ?: 0.0,
            gimbalPitch = parseDouble(schema.gimbalPitchIdx?.let(fields::getOrNull)) ?: 0.0,
            gimbalYaw = parseDouble(schema.gimbalYawIdx?.let(fields::getOrNull)) ?: 0.0,
            velN = parseSpeedMps(schema.velNIdx?.let(fields::getOrNull), schema.velNIdx?.let { schema.normalizedHeaders[it] }.orEmpty()),
            velE = parseSpeedMps(schema.velEIdx?.let(fields::getOrNull), schema.velEIdx?.let { schema.normalizedHeaders[it] }.orEmpty()),
            velD = parseSpeedMps(schema.velDIdx?.let(fields::getOrNull), schema.velDIdx?.let { schema.normalizedHeaders[it] }.orEmpty()),
            fixType = schema.fixTypeIdx?.let { parseFixType(fields.getOrNull(it), schema.normalizedHeaders[it]) } ?: 3,
            satellites = parseInt(schema.satellitesIdx?.let(fields::getOrNull)) ?: 0,
            batteryV = parseDouble(schema.batteryVIdx?.let(fields::getOrNull))
        )
    }

    private fun parseTimestampUs(raw: String): Long? {
        val trimmed = raw.trim()
        if (trimmed.isEmpty() || trimmed == "N/A" || trimmed == "-") return null
        trimmed.toDoubleOrNull()?.let { numeric ->
            return when {
                numeric > 1_000_000_000_000.0 -> numeric.toLong() * 1_000L
                numeric > 1_000_000_000.0 -> (numeric * 1_000_000.0).toLong()
                numeric > 100_000.0 -> (numeric * 1_000.0).toLong()
                else -> (numeric * 1_000_000.0).toLong()
            }
        }
        return parseDateTimeUs(trimmed)
    }

    private fun parseDateTimeUs(raw: String): Long? {
        val cleaned = raw.removeSuffix("Z")
        return try {
            OffsetDateTime.parse(raw).toInstant().toEpochMilli() * 1_000L
        } catch (_: DateTimeParseException) {
            try {
                val dotIdx = cleaned.indexOf('.')
                val base = if (dotIdx >= 0) cleaned.substring(0, dotIdx) else cleaned
                val millis = LocalDateTime.parse(base, localDateTimeFormatter)
                    .toInstant(ZoneOffset.UTC)
                    .toEpochMilli()
                millis * 1_000L + if (dotIdx >= 0) parseFractionMicros(cleaned.substring(dotIdx + 1)) else 0L
            } catch (_: Exception) {
                null
            }
        }
    }

    private fun parseFractionMicros(raw: String): Long =
        raw.takeWhile { it.isDigit() }.padEnd(6, '0').take(6).toLongOrNull() ?: 0L

    private fun parseDouble(raw: String?): Double? {
        val normalized = raw?.trim()?.takeUnless { it.isEmpty() || it == "N/A" || it == "-" } ?: return null
        return normalized.toDoubleOrNull()
    }

    private fun parseInt(raw: String?): Int? = parseDouble(raw)?.toInt()

    private fun parseAltitudeMeters(raw: String?, header: String): Double {
        val value = parseDouble(raw) ?: return 0.0
        return if (header.contains("[ft]") || header.contains("_ft")) value * 0.3048 else value
    }

    private fun parseSpeedMps(raw: String?, header: String): Double {
        val value = parseDouble(raw) ?: return 0.0
        return if (header.contains("[mph]")) value * 0.44704 else value
    }

    private fun parseHdop(raw: String?, header: String): Double {
        val value = parseDouble(raw) ?: return 99.0
        return when {
            header.contains("gpslevel") || header.contains("gps_level") -> {
                if (value <= 0.0) 99.0 else (6.0 - value).coerceIn(0.8, 5.0)
            }
            header.contains("gpsnum") || header.contains("satellites") -> when {
                value >= 20.0 -> 0.9
                value >= 15.0 -> 1.5
                value >= 10.0 -> 2.5
                value > 0.0 -> 4.5
                else -> 99.0
            }
            else -> value
        }
    }

    private fun parseFixType(raw: String?, header: String): Int {
        val value = parseInt(raw) ?: return 3
        return if (header.contains("gpslevel")) {
            when {
                value >= 4 -> 3
                value >= 2 -> 2
                value >= 1 -> 1
                else -> 0
            }
        } else {
            value
        }
    }
}

private fun looksLikeHeader(line: String): Boolean {
    val norm = normalizeHeader(line)
    return norm.contains("latitude") &&
        norm.contains("longitude") &&
        (norm.contains("time(millisecond)") || norm.contains("datetime(utc)"))
}

private fun splitCsvLine(line: String): List<String> =
    line.split(',').map { it.trim().trim('"') }

private fun normalizeHeader(raw: String): String =
    raw.lowercase()
        .replace("\uFEFF", "")
        .replace("\"", "")
        .replace(" ", "")
        .replace("__", "_")
        .trim()
