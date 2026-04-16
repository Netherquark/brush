package com.splats.app.telemetry

import java.io.File
import java.text.SimpleDateFormat
import java.util.Locale
import java.util.TimeZone
import android.util.Log

// ─── Column name mapping ──────────────────────────────────────────────────────

private data class HeaderAliases(
    val aliases: List<String>
)

private object ColumnMap {
    val TIMESTAMP    = HeaderAliases(listOf("datetime(utc)"))
    val LAT          = HeaderAliases(listOf("latitude"))
    val LON          = HeaderAliases(listOf("longitude"))
    val ALT          = HeaderAliases(listOf("altitude(m)"))
    val HEADING      = HeaderAliases(listOf("yaw(deg)"))
    val GIMBAL_PITCH = HeaderAliases(listOf("gimbalpitchraw"))
    val VEL_N        = HeaderAliases(listOf("velocityy(mps)"))
    val VEL_E        = HeaderAliases(listOf("velocityx(mps)"))
    val VEL_D        = HeaderAliases(listOf("velocityz(mps)"))
    val HDOP         = HeaderAliases(emptyList()) // Not usually in Litchi
    val FIX_TYPE     = HeaderAliases(listOf("fixType"))
    val SATELLITES   = HeaderAliases(listOf("satellites"))
}

// ─── CSV Stream Pipeline ────────────────────────────────────────────────────────
/**
 * Single-pass streaming parser.
 * Reads line by line, identifies headers, resolves columns, parses into [TelRow],
 * and applies the timestamp filter immediately, preventing OOM on large files.
 */
internal object CsvIngestParser {
    private const val TAG = "TelemetryCsv"
    private val filenameDateTimeRegex = Regex("""(\d{4}-\d{2}-\d{2})[ _](\d{2})[:_-](\d{2})[:_-](\d{2})""")
    private val filenameTimeRegex = Regex("""(?<!\d)(\d{2})[:_-](\d{2})[:_-](\d{2})(?!\d)""")

    fun streamAndParse(
        file: File,
        videoFileName: String,
        videoStartUsFromMetadata: Long?
    ): Triple<Long, List<TelRow>, Int> {
        if (!file.exists()) throw TelemetryError.CsvNotFound(file.absolutePath)

        val ext = file.extension.lowercase()
        if (ext != "csv") {
            val label = if (ext.isBlank()) "unknown" else ext
            throw TelemetryError.UnsupportedFormat(label)
        }

        var headers: List<String>? = null
        var firstNonEmpty: String? = null
        var parser: ((List<String>) -> TelRow?)? = null

        var baseDateUs: Long? = null
        var targetStartUs: Long? = videoStartUsFromMetadata

        val rows = mutableListOf<TelRow>()
        var dataRowsSize = 0

        file.inputStream().bufferedReader(Charsets.UTF_8).use { reader ->
            while (true) {
                val rawLine = reader.readLine() ?: break
                var line = rawLine.trim()
                if (line.isEmpty()) continue
                if (firstNonEmpty == null) {
                    line = line.removePrefix("\uFEFF") // strip UTF-8 BOM on first line
                    firstNonEmpty = line
                }

                // Locate the header row
                if (headers == null) {
                    if (line.contains(',') && looksLikeHeader(line)) {
                        val headerLine = splitCsvLine(line)
                        headers = headerLine
                        parser = buildRowParser(headerLine)
                        Log.i(TAG, "CSV headers: ${headerLine.joinToString("|")}")
                    }
                    continue
                }

                if (line.contains(',')) {
                    val cols = splitCsvLine(line)
                    dataRowsSize++
                    val row = parser?.invoke(cols)
                    if (row != null) {
                        if (baseDateUs == null && row.timestampUs > 0L) {
                            baseDateUs = row.timestampUs
                        }
                        
                        if (targetStartUs == null && baseDateUs != null) {
                            targetStartUs = parseStartTimeFromFilename(videoFileName, baseDateUs!!)
                        }

                        val actualStartUs = targetStartUs ?: 0L
                        if (actualStartUs == 0L || row.timestampUs >= actualStartUs) {
                            rows.add(row)
                        }
                    }
                }
            }
        }

        if (headers == null) {
            Log.e(TAG, "CSV: could not find header row. First line: $firstNonEmpty")
            throw TelemetryError.InsufficientRecords(0)
        }

        return Triple(targetStartUs ?: 0L, rows, dataRowsSize)
    }

    private fun parseStartTimeFromFilename(filename: String, baseDateUs: Long): Long {
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

    private fun buildRowParser(headers: List<String>): (List<String>) -> TelRow? {
        val lower = headers.map { normalizeHeader(it) }

        fun resolve(headerAliases: HeaderAliases): Int? {
            for (alias in headerAliases.aliases) {
                val idx = lower.indexOf(normalizeHeader(alias))
                if (idx >= 0) return idx
            }
            return null
        }

        val iTs         = resolve(ColumnMap.TIMESTAMP)
            ?: throw TelemetryError.InsufficientRecords(0)
        val iLat        = resolve(ColumnMap.LAT)        ?: throw TelemetryError.InsufficientRecords(0)
        val iLon        = resolve(ColumnMap.LON)        ?: throw TelemetryError.InsufficientRecords(0)
        val iAlt        = resolve(ColumnMap.ALT)        ?: throw TelemetryError.InsufficientRecords(0)
        val iHeading    = resolve(ColumnMap.HEADING)    ?: throw TelemetryError.InsufficientRecords(0)
        val iPitch      = resolve(ColumnMap.GIMBAL_PITCH)?: throw TelemetryError.InsufficientRecords(0)
        val iVelN       = resolve(ColumnMap.VEL_N)      ?: throw TelemetryError.InsufficientRecords(0)
        val iVelE       = resolve(ColumnMap.VEL_E)      ?: throw TelemetryError.InsufficientRecords(0)
        val iVelD       = resolve(ColumnMap.VEL_D)      ?: throw TelemetryError.InsufficientRecords(0)
        val iHdop       = resolve(ColumnMap.HDOP)
        val iFixType    = resolve(ColumnMap.FIX_TYPE)
        val iSatellites = resolve(ColumnMap.SATELLITES)

        val altHeader = lower[iAlt]
        val velNHeader = lower[iVelN]
        val velEHeader = lower[iVelE]
        val velDHeader = lower[iVelD]
        val hdopHeader = iHdop?.let { lower[it] }
        val fixTypeHeader = iFixType?.let { lower[it] }

        return { cols: List<String> ->
            runCatching {
                val tsUs = parseLitchiDateTime(cols.getOrElse(iTs) { "" })
                TelRow(
                    timestampUs = tsUs,
                    lat         = cols.getOrElse(iLat)     { "0" }.toDouble(),
                    lon         = cols.getOrElse(iLon)     { "0" }.toDouble(),
                    altM        = parseAltitudeMeters(cols.getOrElse(iAlt) { "0" }, altHeader),
                    headingDeg  = cols.getOrElse(iHeading) { "0" }.toDouble(),
                    gimbalPitch = cols.getOrElse(iPitch)   { "0" }.toDouble(),
                    velN        = parseSpeedMps(cols.getOrElse(iVelN) { "0" }, velNHeader),
                    velE        = parseSpeedMps(cols.getOrElse(iVelE) { "0" }, velEHeader),
                    velD        = parseSpeedMps(cols.getOrElse(iVelD) { "0" }, velDHeader),
                    hdop        = iHdop?.let {
                        parseHdop(cols.getOrElse(it) { "0" }, hdopHeader ?: "")
                    } ?: 1.0,
                    fixType     = iFixType?.let {
                        parseFixType(cols.getOrElse(it) { "0" }, fixTypeHeader ?: "")
                    } ?: 3,
                    satellites  = iSatellites?.let {
                        cols.getOrElse(it) { "0" }.toIntOrNull() ?: 0
                    } ?: 0
                )
            }.getOrNull()   // malformed rows are dropped; row validator counts them
        }
    }

    private fun parseAltitudeMeters(raw: String, header: String): Double {
        val value = raw.toDoubleOrNull() ?: return 0.0
        return if (header.contains("[ft]") || header.contains("_ft")) value * 0.3048 else value
    }

    private fun parseSpeedMps(raw: String, header: String): Double {
        val value = raw.toDoubleOrNull() ?: return 0.0
        return if (header.contains("[mph]")) value * 0.44704 else value
    }

    private fun parseHdop(raw: String, header: String): Double {
        val value = raw.toDoubleOrNull() ?: return 99.0
        return when {
            header.contains("gpslevel") -> {
                if (value <= 0.0) 99.0 else (6.0 - value).coerceIn(0.8, 5.0)
            }
            header.contains("gpsnum") -> when {
                value >= 20.0 -> 0.9
                value >= 15.0 -> 1.5
                value >= 10.0 -> 2.5
                value > 0.0 -> 4.5
                else -> 99.0
            }
            else -> value
        }
    }

    private fun parseFixType(raw: String, header: String): Int {
        val value = raw.toIntOrNull() ?: raw.toDoubleOrNull()?.toInt() ?: return 3
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

    /**
     * Parse Litchi datetime strings of the form "2024-06-01 10:23:45.456"
     * into microseconds since Unix epoch.
     */
    private fun parseLitchiDateTime(raw: String): Long {
        // Format: yyyy-MM-dd HH:mm:ss.SSS or yyyy-MM-dd HH:mm:ss
        val sdf = SimpleDateFormat("yyyy-MM-dd HH:mm:ss", Locale.US)
        sdf.timeZone = TimeZone.getTimeZone("UTC")
        return try {
            val parts = raw.split('.')
            val baseMs = sdf.parse(parts[0])!!.time
            val fracMs = if (parts.size > 1) parts[1].padEnd(3, '0').take(3).toLong() else 0L
            (baseMs + fracMs) * 1_000L
        } catch (e: Exception) {
            0L
        }
    }
    // A line "looks like" a CSV header if it has at least one recognisable keyword.
    private fun looksLikeHeader(line: String): Boolean {
        val cells = splitCsvLine(line)
        return looksLikeHeaderCells(cells)
    }

    private fun splitCsvLine(line: String): List<String> =
        buildList {
            val current = StringBuilder()
            var inQuotes = false

            for (ch in line) {
                when (ch) {
                    '"' -> inQuotes = !inQuotes
                    ',' -> {
                        if (inQuotes) {
                            current.append(ch)
                        } else {
                            add(current.toString().trim())
                            current.setLength(0)
                        }
                    }
                    else -> current.append(ch)
                }
            }

            add(current.toString().trim())
        }
}

private fun looksLikeHeaderCells(cells: List<String>): Boolean {
    val norm = cells.map { normalizeHeader(it) }
    return norm.any { it.contains("latitude") }
        || norm.any { it.contains("datetime(utc)") }
}

private fun normalizeHeader(raw: String): String =
    raw.lowercase()
        .replace("\uFEFF", "")
        .replace("\"", "")
        .replace(" ", "")
        .trim()
