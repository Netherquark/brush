package com.splats.app.telemetry

import android.util.Log
import org.apache.commons.csv.CSVFormat
import org.apache.commons.csv.CSVRecord
import java.io.File
import java.text.SimpleDateFormat
import java.util.Locale
import java.util.TimeZone

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

class CsvParseContext(var targetStartUs: Long = 0L, var totalSourceRows: Int = 0, var malformedRows: Int = 0)

// ─── CSV Stream Pipeline ────────────────────────────────────────────────────────
/**
 * Single-pass streaming parser providing a safe scope for the CSV reader.
 * Pre-scans for headers (bypassing preambles), initializes commons-csv with strict config,
 * and maintains determinism using the first valid timestamp dynamically.
 */
internal object CsvIngestParser {
    private const val TAG = "TelemetryCsv"

    fun <R> streamAndParse(
        file: File,
        videoFileName: String,
        videoStartUsFromMetadata: Long?,
        block: (CsvParseContext, Sequence<TelRow>) -> R
    ): R {
        if (!file.exists()) throw TelemetryError.CsvNotFound(file.absolutePath)

        val ext = file.extension.lowercase()
        if (ext != "csv") {
            val label = if (ext.isBlank()) "unknown" else ext
            throw TelemetryError.UnsupportedFormat(label)
        }

        // Bound to the entire function scope! No stream leakage allowed.
        return file.inputStream().bufferedReader(Charsets.UTF_8).use { reader ->
            var headerLine: Array<String>? = null
            
            // 1. The Seeker Logic: Consume and discard preamble until header
            reader.mark(1024 * 64) // 64kb max lookahead for preamble checking
            while (true) {
                val rawLine = reader.readLine() ?: break
                val line = rawLine.trim().removePrefix("\uFEFF")
                if (line.isEmpty()) continue

                val isHeader = line.contains("latitude", ignoreCase = true) 
                    || line.contains("datetime(utc)", ignoreCase = true)
                
                if (isHeader) {
                    // Extract exactly what the file stated, keeping case. 
                    // No spaces/bom trimming here; commons-csv format controls it.
                    val cells = splitPreCheckLine(line)
                    headerLine = cells.toTypedArray()
                    break
                }
            }
            
            if (headerLine == null) {
                Log.e(TAG, "CSV: could not find header row. Malformed or purely preamble.")
                throw TelemetryError.InsufficientRecords(0)
            }

            // 2. Format with strict consistency protocols
            val format = CSVFormat.DEFAULT.builder()
                .setHeader(*headerLine)
                .setAllowMissingColumnNames(false)
                .setIgnoreHeaderCase(true)
                .setTrim(true)
                .build()

            val csvParser = format.parse(reader)
            
            // Build the row parser exactly using mapped names dynamically
            val headerMap = csvParser.headerMap ?: emptyMap()
            val rowParser = buildRowParser(headerMap)
            
            Log.i(TAG, "CSV headers detected successfully: ${headerLine.size} columns")

            val context = CsvParseContext(targetStartUs = videoStartUsFromMetadata ?: 0L)
            var baseDateUs: Long? = null

            // 3. Dynamic sequence implementation
            val rowSequence = sequence {
                for (record in csvParser) {
                    context.totalSourceRows++
                    // Strict Leniency Rule - drop mismatched columns immediately
                    if (!record.isConsistent) {
                        Log.w(TAG, "Dropping malformed CSV row (inconsistent column size): line ${record.recordNumber}")
                        context.malformedRows++
                        continue
                    }

                    val row = rowParser(record)
                    if (row == null) {
                        context.malformedRows++
                        continue
                    }

                    // Determine Target Start Time once statically
                    if (baseDateUs == null && row.timestampUs > 0L) {
                        baseDateUs = row.timestampUs
                    }
                    if (videoStartUsFromMetadata == null && context.targetStartUs == 0L && baseDateUs != null) {
                        context.targetStartUs = VideoTimeExtractor.extractTargetStartUs(videoFileName, baseDateUs!!)
                    }

                    val activeFilterUs = context.targetStartUs
                    if (activeFilterUs == 0L || row.timestampUs >= activeFilterUs) {
                        yield(row)
                    }
                }
            }

            block(context, rowSequence)
        }
    }

    private fun splitPreCheckLine(line: String): List<String> {
        // Fast split just to identify the initial header array schema
        return line.split(',').map { it.trim().removePrefix("\"").removeSuffix("\"") }
    }

    private fun buildRowParser(headerMap: Map<String, Int>): (CSVRecord) -> TelRow? {
        // Prepare normalized mapping table 
        val normalizedHeaders = headerMap.entries.associate { (k, v) -> 
            normalizeHeader(k) to v 
        }

        fun resolve(headerAliases: HeaderAliases): Int? {
            for (alias in headerAliases.aliases) {
                val normAlias = normalizeHeader(alias)
                if (normalizedHeaders.containsKey(normAlias)) return normalizedHeaders[normAlias]
            }
            return null
        }

        val iTs         = resolve(ColumnMap.TIMESTAMP) ?: throw TelemetryError.InsufficientRecords(0)
        val iLat        = resolve(ColumnMap.LAT)       ?: throw TelemetryError.InsufficientRecords(0)
        val iLon        = resolve(ColumnMap.LON)       ?: throw TelemetryError.InsufficientRecords(0)
        val iAlt        = resolve(ColumnMap.ALT)       ?: throw TelemetryError.InsufficientRecords(0)
        val iHeading    = resolve(ColumnMap.HEADING)   ?: throw TelemetryError.InsufficientRecords(0)
        val iPitch      = resolve(ColumnMap.GIMBAL_PITCH)?: throw TelemetryError.InsufficientRecords(0)
        val iVelN       = resolve(ColumnMap.VEL_N)     ?: throw TelemetryError.InsufficientRecords(0)
        val iVelE       = resolve(ColumnMap.VEL_E)     ?: throw TelemetryError.InsufficientRecords(0)
        val iVelD       = resolve(ColumnMap.VEL_D)     ?: throw TelemetryError.InsufficientRecords(0)
        val iHdop       = resolve(ColumnMap.HDOP)
        val iFixType    = resolve(ColumnMap.FIX_TYPE)
        val iSatellites = resolve(ColumnMap.SATELLITES)

        val revMap = headerMap.entries.associate { it.value to it.key }
        val altHeader = revMap[iAlt] ?: ""
        val velNHeader = revMap[iVelN] ?: ""
        val velEHeader = revMap[iVelE] ?: ""
        val velDHeader = revMap[iVelD] ?: ""
        val hdopHeader = iHdop?.let { revMap[it] } ?: ""
        val fixTypeHeader = iFixType?.let { revMap[it] } ?: ""

        return { cols: CSVRecord ->
            fun getCol(idx: Int, default: String = "0"): String {
                return if (idx >= 0 && idx < cols.size()) {
                    val raw = cols.get(idx).trim()
                    if (raw.isBlank()) default else raw
                } else default
            }

            runCatching {
                val tsUs = parseLitchiDateTime(getCol(iTs, ""))
                TelRow(
                    timestampUs = tsUs,
                    lat         = getCol(iLat).toDouble(),
                    lon         = getCol(iLon).toDouble(),
                    altM        = parseAltitudeMeters(getCol(iAlt), altHeader),
                    headingDeg  = getCol(iHeading).toDouble(),
                    gimbalPitch = getCol(iPitch).toDouble(),
                    velN        = parseSpeedMps(getCol(iVelN), velNHeader),
                    velE        = parseSpeedMps(getCol(iVelE), velEHeader),
                    velD        = parseSpeedMps(getCol(iVelD), velDHeader),
                    hdop        = iHdop?.let {
                        parseHdop(getCol(it), hdopHeader)
                    } ?: 1.0,
                    fixType     = iFixType?.let {
                        parseFixType(getCol(it), fixTypeHeader)
                    } ?: 3,
                    satellites  = iSatellites?.let {
                        getCol(it).toIntOrNull() ?: 0
                    } ?: 0
                )
            }.getOrNull()
        }
    }

    private fun parseAltitudeMeters(raw: String, header: String): Double {
        val value = raw.toDoubleOrNull() ?: return 0.0
        return if (header.contains("[ft]", ignoreCase = true) || header.contains("_ft", ignoreCase = true)) 
            value * 0.3048 else value
    }

    private fun parseSpeedMps(raw: String, header: String): Double {
        val value = raw.toDoubleOrNull() ?: return 0.0
        return if (header.contains("[mph]", ignoreCase = true)) value * 0.44704 else value
    }

    private fun parseHdop(raw: String, header: String): Double {
        val value = raw.toDoubleOrNull() ?: return 99.0
        return when {
            header.contains("gpslevel", ignoreCase = true) -> {
                if (value <= 0.0) 99.0 else (6.0 - value).coerceIn(0.8, 5.0)
            }
            header.contains("gpsnum", ignoreCase = true) -> when {
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
        return if (header.contains("gpslevel", ignoreCase = true)) {
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

    private fun parseLitchiDateTime(raw: String): Long {
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

    private fun normalizeHeader(raw: String): String =
        raw.lowercase(Locale.US)
            .replace("\uFEFF", "")
            .replace("\"", "")
            .replace(" ", "")
            .trim()
}
