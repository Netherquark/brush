package com.splats.app.telemetry

import java.io.File
import java.text.SimpleDateFormat
import java.util.Locale
import java.util.TimeZone
import android.util.Log

// ─── Column name mapping ──────────────────────────────────────────────────────

/**
 * Maps canonical field names to the two known vendor header variants.
 * DJI aliases are checked only on the DJI pass.
 * Litchi aliases are checked only on the Litchi pass.
 */
private data class HeaderAliases(
    val dji: List<String>,
    val litchi: List<String> = emptyList()
)

private object ColumnMap {
    val TIMESTAMP    = HeaderAliases(
        dji = listOf("time(millisecond)", "timestamps_ns"),
        litchi = listOf("datetime(utc)")
    )
    val LAT          = HeaderAliases(
        dji = listOf("latitude", "osd.latitude"),
        litchi = listOf("gps(0)[latitude]")
    )
    val LON          = HeaderAliases(
        dji = listOf("longitude", "osd.longitude"),
        litchi = listOf("gps(0)[longitude]")
    )
    val ALT          = HeaderAliases(
        dji = listOf("altitude(m)", "osd.altitude[ft]", "osd.height[ft]", "altitude_ft"),
        litchi = listOf("altitude(m)")
    )
    val HEADING      = HeaderAliases(
        dji = listOf("compass_heading(degrees)", "yaw"),
        litchi = listOf("yaw(deg)")
    )
    val GIMBAL_PITCH = HeaderAliases(
        dji = listOf("gimbal_pitch(degrees)", "gimbal.pitch", "gimbal_pitch"),
        litchi = listOf("gimbalpitchraw")
    )
    val VEL_N        = HeaderAliases(
        dji = listOf("speed_n(m/s)", "osd.yspeed[mph]"),
        litchi = listOf("velocityy(mps)")
    )
    val VEL_E        = HeaderAliases(
        dji = listOf("speed_e(m/s)", "osd.xspeed[mph]"),
        litchi = listOf("velocityx(mps)")
    )
    val VEL_D        = HeaderAliases(
        dji = listOf("speed_d(m/s)", "osd.zspeed[mph]", "vertical_speed_mph"),
        litchi = listOf("velocityz(mps)")
    )
    val HDOP         = HeaderAliases(
        dji = listOf("gps(accuracy)", "osd.gpslevel", "osd.gpsnum", "satellites")
    )
    val FIX_TYPE     = HeaderAliases(
        dji = listOf("gps(fixType)", "osd.gpslevel"),
        litchi = listOf("fixType")
    )
    val SATELLITES   = HeaderAliases(
        dji = listOf("satellites", "osd.gpsnum"),
        litchi = listOf("satellites")
    )
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
 * Resolves header names to column indices (DJI first, Litchi second).
 * Coerces raw strings into typed [TelRow] instances.
 * Litchi "datetime(utc)" strings are converted to microseconds since epoch.
 */
internal object CsvParser {
    private const val TAG = "TelemetryCsv"

    /**
     * Attempt DJI column map first, then Litchi.
     * If both fail, raises [TelemetryError.InsufficientRecords].
     * Returns (rows, vendorWarning) — vendorWarning is true if the second
     * (Litchi) pass was needed, so the caller can record it in the report.
     */
    fun parse(
        headers: List<String>,
        dataRows: List<List<String>>
    ): Pair<List<TelRow>, Boolean> {
        Log.i(TAG, "Parse headers raw: ${headers.joinToString("|")}")
        Log.i(TAG, "Parse headers norm: ${headers.map { normalizeHeader(it) }.joinToString("|")}")
        // First pass: DJI
        return try {
            val rows = parseWithMap(headers, dataRows, vendorIndex = 0)
            Pair(rows, false)
        } catch (e: TelemetryError.InsufficientRecords) {
            // Second pass: Litchi (spec §6.2 — Litchi vs DJI header mismatch)
            try {
                val rows = parseWithMap(headers, dataRows, vendorIndex = 1)
                Pair(rows, true)   // vendorWarning = true
            } catch (e2: TelemetryError.InsufficientRecords) {
                throw TelemetryError.InsufficientRecords(0)
            }
        }
    }

    /**
     * Internal parse using vendor index 0 = DJI, 1 = Litchi.
     */
    private fun parseWithMap(
        headers: List<String>,
        dataRows: List<List<String>>,
        vendorIndex: Int
    ): List<TelRow> {
        val lower = headers.map { normalizeHeader(it) }

        fun resolve(aliases: HeaderAliases): Int? {
            val candidates = if (vendorIndex == 0) aliases.dji else aliases.litchi
            for (alias in candidates) {
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
        val iHdop       = resolve(ColumnMap.HDOP)       ?: throw TelemetryError.InsufficientRecords(0)
        val iFixType    = resolve(ColumnMap.FIX_TYPE)
        val iSatellites = resolve(ColumnMap.SATELLITES)

        val tsHeader = lower[iTs]
        val altHeader = lower[iAlt]
        val velNHeader = lower[iVelN]
        val velEHeader = lower[iVelE]
        val velDHeader = lower[iVelD]
        val hdopHeader = lower[iHdop]
        val fixTypeHeader = iFixType?.let { lower[it] }

        // Detect whether we're reading a Litchi file (datetime vs DJI timestamps).
        val isLitchi = tsHeader.contains("datetime")

        return dataRows.mapNotNull { cols ->
            runCatching {
                val tsUs = parseTimestampUs(cols.getOrElse(iTs) { "" }, tsHeader, isLitchi)
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
                    hdop        = parseHdop(cols.getOrElse(iHdop) { "0" }, hdopHeader),
                    fixType     = iFixType?.let {
                        parseFixType(cols.getOrElse(it) { "0" }, fixTypeHeader ?: "")
                    } ?: 3,
                    satellites  = iSatellites?.let { cols.getOrElse(it) { "0" }.toInt() } ?: 0
                )
            }.getOrNull()   // malformed rows are dropped; row validator counts them
        }
    }

    private fun parseTimestampUs(raw: String, header: String, isLitchi: Boolean): Long {
        if (isLitchi) return parseLitchiDateTime(raw)
        val value = raw.toDoubleOrNull() ?: return 0L
        return when {
            header.contains("timestamps_ns") -> (value / 1_000.0).toLong()
            header.contains("time(millisecond)") -> (value * 1_000.0).toLong()
            else -> (value * 1_000.0).toLong()
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
}

private fun looksLikeHeaderCells(cells: List<String>): Boolean {
    val norm = cells.map { normalizeHeader(it) }
    return norm.any { it.contains("latitude") }
        || norm.any { it.contains("gps(0)[latitude]") }
        || norm.any { it.contains("time(millisecond)") }
        || norm.any { it.contains("datetime(utc)") }
}

private fun normalizeHeader(raw: String): String =
    raw.lowercase()
        .replace("\uFEFF", "")
        .replace("\"", "")
        .replace(" ", "")
        .trim()
