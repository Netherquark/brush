package com.splats.app.telemetry

import kotlin.math.cos
import kotlin.math.PI

// ─── ENU Converter ────────────────────────────────────────────────────────────

/**
 * Stage 4 — WGS84 → Local ENU conversion.
 *
 * Establishes the ENU origin as the first row where HDOP ≤ 3.0 and
 * fixType = 3D (fixType ≥ 3). All coordinates are then expressed as
 * metre offsets (East, North, Up) from that origin.
 *
 * Flat-Earth approximation:
 *   E = (lon − lon₀) × cos(lat₀ × π/180) × 111_320
 *   N = (lat − lat₀) × 111_320
 *   U = alt − alt₀
 *
 * Accurate to < 1 mm within 10 km; < 10 mm within 50 km of origin.
 * Scenes exceeding 50 km in any ENU dimension receive
 * [QualityFlag.LARGE_SCENE_COORD] on the output header (handled by the
 * PoseStampSequence emitter, not here).
 */
internal object EnuConverter {

    private const val METRES_PER_DEGREE = 111_320.0
    private const val MAX_HDOP_FOR_ORIGIN = 3.0

    /** Convert a list of validated [TelRow] to [TelRecord] in the ENU frame. */
    fun convert(rows: List<TelRow>): Pair<EnuOrigin, List<TelRecord>> {
        val originRow = rows.firstOrNull { RowValidator.isGpsUsable(it) && it.hdop <= MAX_HDOP_FOR_ORIGIN }
            ?: rows.firstOrNull { it.fixType >= 3 }
            ?: throw TelemetryError.NoValidOrigin

        val lat0   = originRow.lat
        val lon0   = originRow.lon
        val alt0   = originRow.altM
        val cosLat = cos(lat0 * PI / 180.0)

        val origin = EnuOrigin(
            lat0        = lat0,
            lon0        = lon0,
            alt0        = alt0,
            cosLat0     = cosLat,
            timestampUs = originRow.timestampUs
        )

        val records = rows.map { row ->
            val enuE = (row.lon - lon0) * cosLat * METRES_PER_DEGREE
            val enuN = (row.lat - lat0) * METRES_PER_DEGREE
            val enuU = row.altM - alt0

            TelRecord(
                sourceRowIndex = row.sourceRowIndex,
                timestampUs = row.timestampUs,
                tsAligned   = row.timestampUs, // time sync offset applied later
                lat         = row.lat,
                lon         = row.lon,
                altM        = row.altM,
                hdop        = row.hdop,
                pitchDeg    = row.pitchDeg,
                rollDeg     = row.rollDeg,
                yawDeg      = row.yawDeg,
                gimbalPitch = row.gimbalPitch,
                gimbalYaw   = row.gimbalYaw,
                velN        = row.velN,
                velE        = row.velE,
                velD        = row.velD,
                velNFiltered = row.velN,
                velEFiltered = row.velE,
                velUFiltered = -row.velD,
                enuE        = enuE,
                enuN        = enuN,
                enuU        = enuU,
                qW          = 1.0,
                qX          = 0.0,
                qY          = 0.0,
                qZ          = 0.0,
                fixType     = row.fixType,
                satellites  = row.satellites,
                batteryV    = row.batteryV,
                gpsOk       = RowValidator.isGpsUsable(row)
            )
        }

        return Pair(origin, records)
    }
}
