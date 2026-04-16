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

// ─── CSV Ingest ───────────────────────────────────────────────────────────────

/**
 * Stage 1 — Raw Ingest.
 *
 * Reads the file as a plain string matrix (rows × columns).
 * Handles UTF-8 BOM, CRLF/LF line endings, and preamble rows that appear
 * before the actual header row in older DJI firmware exports.
 *
 * No type parsing is performed here.
 */
internal object CsvIngest {
    private const val TAG = "TelemetryCsv"

    fun read(file: File): Pair<List<String>, List<List<String>>> {
        if (!file.exists()) throw TelemetryError.CsvNotFound(file.absolutePath)

        val ext = file.extension.lowercase()
        if (ext != "csv") {
            val label = if (ext.isBlank()) "unknown" else ext
            throw TelemetryError.UnsupportedFormat(label)
        }
        return readCsv(file)
    }

    private fun readCsv(file: File): Pair<List<String>, List<List<String>>> {
        val dataRows = mutableListOf<List<String>>()
        var headers: List<String>? = null
        var firstNonEmpty: String? = null

        file.inputStream().bufferedReader(Charsets.UTF_8).use { reader ->
            while (true) {
                val rawLine = reader.readLine() ?: break
                var line = rawLine.trim()
                if (line.isEmpty()) continue
                if (firstNonEmpty == null) {
                    line = line.removePrefix("\uFEFF") // strip UTF-8 BOM on first line
                    firstNonEmpty = line
                }

                // Locate the header row: first line that looks like a CSV header
                // (contains a comma and at least one recognisable keyword).
                if (headers == null) {
                    if (line.contains(',') && looksLikeHeader(line)) {
                        val headerLine = splitCsvLine(line)
                        headers = headerLine
                        Log.i(TAG, "CSV headers: ${headerLine.joinToString("|")}")
                    }
                    continue
                }

                if (line.contains(',')) {
                    dataRows += splitCsvLine(line)
                }
            }
        }

        val headersFinal = headers ?: run {
            Log.e(TAG, "CSV: could not find header row. First line: $firstNonEmpty")
            throw TelemetryError.InsufficientRecords(0)
        }

        return Pair(headersFinal, dataRows)
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

// ─── CSV Parser ───────────────────────────────────────────────────────────────

/**
 * Stage 2 — Type-safe parsing.
 *
 * Resolves Litchi header names to column indices.
 * Coerces raw strings into typed [TelRow] instances.
 * Litchi "datetime(utc)" strings are converted to microseconds since epoch.
 */
internal object CsvParser {
    private const val TAG = "TelemetryCsv"
    private val filenameDateTimeRegex = Regex("""(\d{4}-\d{2}-\d{2})[ _](\d{2})[:_-](\d{2})[:_-](\d{2})""")
    private val filenameTimeRegex = Regex("""(?<!\d)(\d{2})[:_-](\d{2})[:_-](\d{2})(?!\d)""")

    /**
     * Streaming CSV parser that avoids allocating per-row `List<String>` / `StringBuilder`.
     *
     * It still allocates strings for only the columns we actually need, then builds `TelRow`.
     */
    fun parse(csvFile: File, targetStartUs: Long = 0L): List<TelRow> {
        if (!csvFile.exists()) throw TelemetryError.CsvNotFound(csvFile.absolutePath)

        val ext = csvFile.extension.lowercase()
        if (ext != "csv") {
            val label = if (ext.isBlank()) "unknown" else ext
            throw TelemetryError.UnsupportedFormat(label)
        }

        fun splitCsvLineForHeader(line: String): List<String> =
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

        fun looksLikeHeaderLine(line: String): Boolean {
            // Header detection is only needed until we find the first real header row.
            // Avoid per-line CSV splitting here (performance hotspot in PERF-002).
            val norm = line
                .lowercase()
                .replace("\"", "")
                .replace(" ", "")
                .trim()
            return norm.contains("latitude") || norm.contains("datetime(utc)")
        }

        val out = ArrayList<TelRow>()
        var headers: List<String>? = null
        var firstNonEmpty: String? = null

        // Reused buffer for data-row parsing (the storm fix).
        val current = StringBuilder()
        var parseRow: ((String) -> TelRow?)? = null

        csvFile.inputStream().bufferedReader(Charsets.UTF_8).use { reader ->
            while (true) {
                val rawLine = reader.readLine() ?: break
                var line = rawLine.trim()
                if (line.isEmpty()) continue

                if (firstNonEmpty == null) {
                    line = line.removePrefix("\uFEFF") // strip UTF-8 BOM on first line
                    firstNonEmpty = line
                }

                // Locate the header row.
                if (headers == null) {
                    if (line.contains(',') && looksLikeHeaderLine(line)) {
                        val headerLine = splitCsvLineForHeader(line)
                        headers = headerLine

                        val lower = headerLine.map { normalizeHeader(it) }
                        fun resolve(headerAliases: HeaderAliases): Int? {
                            for (alias in headerAliases.aliases) {
                                val idx = lower.indexOf(normalizeHeader(alias))
                                if (idx >= 0) return idx
                            }
                            return null
                        }

                        val iTs =
                            resolve(ColumnMap.TIMESTAMP) ?: throw TelemetryError.InsufficientRecords(0)
                        val iLat =
                            resolve(ColumnMap.LAT) ?: throw TelemetryError.InsufficientRecords(0)
                        val iLon =
                            resolve(ColumnMap.LON) ?: throw TelemetryError.InsufficientRecords(0)
                        val iAlt =
                            resolve(ColumnMap.ALT) ?: throw TelemetryError.InsufficientRecords(0)
                        val iHeading =
                            resolve(ColumnMap.HEADING) ?: throw TelemetryError.InsufficientRecords(0)
                        val iPitch =
                            resolve(ColumnMap.GIMBAL_PITCH) ?: throw TelemetryError.InsufficientRecords(0)
                        val iVelN =
                            resolve(ColumnMap.VEL_N) ?: throw TelemetryError.InsufficientRecords(0)
                        val iVelE =
                            resolve(ColumnMap.VEL_E) ?: throw TelemetryError.InsufficientRecords(0)
                        val iVelD =
                            resolve(ColumnMap.VEL_D) ?: throw TelemetryError.InsufficientRecords(0)
                        val iHdop = resolve(ColumnMap.HDOP)
                        val iFixType = resolve(ColumnMap.FIX_TYPE)
                        val iSatellites = resolve(ColumnMap.SATELLITES)

                        val altHeader = lower[iAlt]
                        val velNHeader = lower[iVelN]
                        val velEHeader = lower[iVelE]
                        val velDHeader = lower[iVelD]
                        val hdopHeader = iHdop?.let { lower[it] }
                        val fixTypeHeader = iFixType?.let { lower[it] }

                        val sdf = SimpleDateFormat("yyyy-MM-dd HH:mm:ss", Locale.US).apply {
                            timeZone = TimeZone.getTimeZone("UTC")
                        }

                        parseRow = { dataLine ->
                            // Per-row parse state: keep only needed columns.
                            var tsRaw: String? = null
                            var latRaw: String? = null
                            var lonRaw: String? = null
                            var altRaw: String? = null
                            var headingRaw: String? = null
                            var pitchRaw: String? = null
                            var velNRaw: String? = null
                            var velERaw: String? = null
                            var velDRaw: String? = null
                            var hdopRaw: String? = null
                            var fixTypeRaw: String? = null
                            var satellitesRaw: String? = null

                            current.setLength(0)
                            var inQuotes = false
                            var fieldIndex = 0

                            fun captureIfNeeded() {
                                when (fieldIndex) {
                                    iTs -> tsRaw = current.toString().trim()
                                    iLat -> latRaw = current.toString().trim()
                                    iLon -> lonRaw = current.toString().trim()
                                    iAlt -> altRaw = current.toString().trim()
                                    iHeading -> headingRaw = current.toString().trim()
                                    iPitch -> pitchRaw = current.toString().trim()
                                    iVelN -> velNRaw = current.toString().trim()
                                    iVelE -> velERaw = current.toString().trim()
                                    iVelD -> velDRaw = current.toString().trim()
                                    else -> {
                                        if (iHdop != null && fieldIndex == iHdop) hdopRaw = current.toString().trim()
                                        else if (iFixType != null && fieldIndex == iFixType) fixTypeRaw = current.toString().trim()
                                        else if (iSatellites != null && fieldIndex == iSatellites) satellitesRaw = current.toString().trim()
                                    }
                                }
                            }

                            for (ch in dataLine) {
                                when (ch) {
                                    '"' -> inQuotes = !inQuotes
                                    ',' -> {
                                        if (inQuotes) {
                                            current.append(ch)
                                        } else {
                                            captureIfNeeded()
                                            fieldIndex++
                                            current.setLength(0)
                                        }
                                    }
                                    else -> current.append(ch)
                                }
                            }
                            captureIfNeeded() // last field

                            runCatching {
                                val tsUs = parseLitchiDateTimeFast(tsRaw ?: "", sdf)
                                TelRow(
                                    timestampUs = tsUs,
                                    lat = (latRaw ?: "0").toDouble(),
                                    lon = (lonRaw ?: "0").toDouble(),
                                    altM = parseAltitudeMeters(altRaw ?: "0", altHeader),
                                    headingDeg = (headingRaw ?: "0").toDouble(),
                                    gimbalPitch = (pitchRaw ?: "0").toDouble(),
                                    velN = parseSpeedMps(velNRaw ?: "0", velNHeader),
                                    velE = parseSpeedMps(velERaw ?: "0", velEHeader),
                                    velD = parseSpeedMps(velDRaw ?: "0", velDHeader),
                                    hdop = iHdop?.let {
                                        parseHdop(hdopRaw ?: "0", hdopHeader ?: "")
                                    } ?: 1.0,
                                    fixType = iFixType?.let {
                                        parseFixType(fixTypeRaw ?: "0", fixTypeHeader ?: "")
                                    } ?: 3,
                                    satellites = iSatellites?.let {
                                        satellitesRaw?.toIntOrNull() ?: 0
                                    } ?: 0
                                )
                            }.getOrNull() // malformed rows are dropped
                        }
                    }
                    continue
                }

                // Parse data rows once we have indices.
                if (line.contains(',') && parseRow != null) {
                    val telRow = parseRow!!(line)
                    if (telRow != null && (targetStartUs <= 0L || telRow.timestampUs >= targetStartUs)) {
                        out += telRow
                    }
                }
            }
        }

        if (headers == null) {
            Log.e(TAG, "CSV: could not find header row. First line: $firstNonEmpty")
            throw TelemetryError.InsufficientRecords(0)
        }

        return out
    }

    fun parse(
        headers: List<String>,
        dataRows: List<List<String>>,
        targetStartUs: Long = 0L
    ): List<TelRow> {
        Log.i(TAG, "Parse headers raw: ${headers.joinToString("|")}")
        Log.i(TAG, "Parse headers norm: ${headers.map { normalizeHeader(it) }.joinToString("|")}")
        return try {
            val rows = parseInternal(headers, dataRows)
            if (targetStartUs > 0) {
                Log.i(TAG, "Filtering CSV to start at time $targetStartUs")
                rows.filter { it.timestampUs >= targetStartUs }
            } else {
                rows
            }
        } catch (e: TelemetryError.InsufficientRecords) {
            throw TelemetryError.InsufficientRecords(0)
        }
    }

    fun parseStartTimeFromFilename(filename: String, rows: List<TelRow>): Long {
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

        val baseDateUs = rows.firstOrNull { it.timestampUs > 0L }?.timestampUs ?: return 0L
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

    private fun parseInternal(
        headers: List<String>,
        dataRows: List<List<String>>
    ): List<TelRow> {
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

        return dataRows.mapNotNull { cols ->
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

    /**
     * Allocation-reduced version used by the streaming parser.
     * - Avoids `raw.split('.')` list allocation.
     * - Uses a provided `SimpleDateFormat` for the whole parse run.
     */
    private fun parseLitchiDateTimeFast(raw: String, sdf: SimpleDateFormat): Long {
        return try {
            val dotIdx = raw.indexOf('.')
            val baseStr = if (dotIdx >= 0) raw.substring(0, dotIdx) else raw
            val baseMs = sdf.parse(baseStr)?.time ?: return 0L
            if (dotIdx < 0) return baseMs * 1_000L

            val fracStart = dotIdx + 1
            var fracMs = 0L
            var multiplier = 100
            for (i in 0 until 3) {
                val idx = fracStart + i
                val digit = if (idx < raw.length) {
                    val c = raw[idx]
                    if (c in '0'..'9') c.code - '0'.code else throw NumberFormatException("Non-digit fraction: $c")
                } else 0
                fracMs += digit.toLong() * multiplier.toLong()
                multiplier /= 10
            }
            (baseMs + fracMs) * 1_000L
        } catch (_: Exception) {
            0L
        }
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
