package com.splats.app.telemetry

import android.util.Log
import org.apache.commons.csv.CSVFormat
import org.apache.commons.csv.CSVParser
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
internal object CsvIngestParser {
    private const val TAG = "TelemetryCsv"
    private const val MAX_INIT_BUFFER_SIZE = 5000

    /**
     * Single-pass streaming parser providing a safe scope for the CSV reader.
     * The Reader remains open ONLY while the provided lambda evaluates the `Sequence`.
     * Caller MUST ensure terminal evaluations (like `.toList()`) are synchronously contained
     * before the lambda returns.
     */
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

        return file.inputStream().bufferedReader(Charsets.UTF_8).use { reader ->
            var headerLine: Array<String>? = null
            
            val formatBase = CSVFormat.DEFAULT.builder()
                .setAllowMissingColumnNames(false)
                .setIgnoreHeaderCase(true)
                .setTrim(true)
                .build()

            // 1. Zero-Allocation Seeker: Consume lines until headers are naturally found
            while (true) {
                val rawLine = reader.readLine() ?: break
                val line = rawLine.trim().removePrefix("\uFEFF")
                if (line.isEmpty()) continue

                val isHeader = line.contains("latitude", ignoreCase = true) 
                    || line.contains("datetime(utc)", ignoreCase = true)
                
                if (isHeader) {
                    val parsed = CSVParser.parse(line, formatBase)
                    val records = parsed.records
                    if (records.isNotEmpty()) {
                        headerLine = records.first().toList().toTypedArray()
                        break
                    }
                }
            }
            
            if (headerLine == null) {
                Log.e(TAG, "CSV: could not find header row. Malformed or purely preamble.")
                throw TelemetryError.InsufficientRecords(0)
            }

            // 2. Main data tokenizer initialized dynamically 
            val dataFormat = formatBase.builder()
                .setHeader(*headerLine)
                .build()

            val csvParser = dataFormat.parse(reader)
            val headerMap = csvParser.headerMap ?: emptyMap()
            val rowParser = buildRowParser(headerMap)

            Log.i(TAG, "CSV headers detected successfully: ${headerLine.size} columns")

            val context = CsvParseContext(targetStartUs = videoStartUsFromMetadata ?: 0L)
            var baseDateUs: Long? = null

            // 3. Dynamic sequence implementation with Peeker Buffer Constraints
            val rowSequence = sequence {
                var initializationBuffer: MutableList<TelRow>? = mutableListOf()

                for (record in csvParser) {
                    context.totalSourceRows++
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

                    if (context.targetStartUs == 0L) {
                        if (row.timestampUs > 0L) {
                            baseDateUs = row.timestampUs
                            context.targetStartUs = VideoTimeExtractor.extractTargetStartUs(videoFileName, baseDateUs!!)
                            
                            // Flush micro-buffer safely
                            initializationBuffer?.forEach { b ->
                                if (b.timestampUs >= context.targetStartUs) yield(b)
                            }
                            initializationBuffer = null
                            
                            if (row.timestampUs >= context.targetStartUs) yield(row)
                        } else {
                            initializationBuffer?.add(row)
                            if (initializationBuffer != null && initializationBuffer!!.size > MAX_INIT_BUFFER_SIZE) {
                                throw TelemetryError.InsufficientRecords(MAX_INIT_BUFFER_SIZE)
                            }
                        }
                    } else {
                        if (row.timestampUs >= context.targetStartUs) {
                            yield(row)
                        }
                    }
                }
            }.constrainOnce()

            block(context, rowSequence)
        }
    }

    private fun buildRowParser(headerMap: Map<String, Int>): (CSVRecord) -> TelRow? {
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
            fun getCol(idx: Int): String {
                if (idx < 0 || idx >= cols.size()) throw IllegalArgumentException("Missing coordinate column index")
                val raw = cols.get(idx).trim()
                if (raw.isBlank()) throw IllegalArgumentException("Empty required coordinate cell")
                return raw
            }

            fun getColOrEmpty(idx: Int): String {
                return if (idx >= 0 && idx < cols.size()) cols.get(idx).trim() else ""
            }

            // Explicit localized boundary tracking kinematics strictly
            runCatching {
                val tsUs = parseLitchiDateTime(getColOrEmpty(iTs))
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
                    hdop        = iHdop?.let { parseHdop(getColOrEmpty(it), hdopHeader) } ?: 1.0,
                    fixType     = iFixType?.let { parseFixType(getColOrEmpty(it), fixTypeHeader) } ?: 3,
                    satellites  = iSatellites?.let { getColOrEmpty(it).toIntOrNull() ?: 0 } ?: 0
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
