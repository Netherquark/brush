package com.splats.app.telemetry

import android.graphics.Bitmap
import android.media.MediaMetadataRetriever
import android.util.Log
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.TimeoutCancellationException
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext
import kotlinx.coroutines.withTimeout
import java.io.File
import kotlin.jvm.JvmOverloads
import kotlin.math.abs
import kotlin.math.pow
import kotlin.math.round
import kotlin.math.sqrt

interface TelemetryPreprocessorCallback {
    fun onProgress(stage: ProcessingStage, fraction: Float)
    fun onComplete(
        sequence: PoseStampSequence?,
        error: Throwable?,
        report: TelemetryProcessingReport?
    )
}

private object HardwareTier {
    fun isFlagship(): Boolean {
        return try {
            val soc = if (android.os.Build.VERSION.SDK_INT >= 31) {
                android.os.Build.SOC_MODEL
            } else {
                android.os.Build.HARDWARE
            }
            
            val cpuInfo = File("/proc/cpuinfo").readText().lowercase()
            val flagshipStrings = listOf(
                "sm8550", "sm8650", "sm8750", "kona", "taro", // Qualcomm
                "tensor", "gs101", "gs201", "gs301",          // Google
                "mt68", "mt69", "dimensity",                  // MediaTek
                "exynos"                                      // Exynos
            )
            
            flagshipStrings.any { 
                soc.lowercase().contains(it) || cpuInfo.contains(it)
            }
        } catch (_: Exception) {
            Runtime.getRuntime().availableProcessors() >= 8
        }
    }

    fun timeoutMs(): Long = if (isFlagship()) 90_000L else 180_000L
}

class TelemetryPreprocessor @JvmOverloads constructor(
    private val csvFile: File,
    private val videoFile: File,
    private val keyframeTimestampsUs: LongArray = LongArray(0),
    private val callback: TelemetryPreprocessorCallback,
    private val keyframeSelectionConfig: KeyframeSelectionConfig = KeyframeSelectionConfig(),
    private val outputDir: File = File(System.getProperty("java.io.tmpdir") ?: "/tmp"),
    private val sessionId: String = "session_${System.currentTimeMillis()}",
    val configJsonStr: String = "{}"
) {
    private var job: Job? = null
    private val logTag = "TelemetryPreprocessor"

    private val activeKeyframeConfig by lazy {
        try {
            val json = org.json.JSONObject(configJsonStr)
            var config = keyframeSelectionConfig
            if (json.has("distanceThresholdM")) config = config.copy(distanceThresholdM = json.getDouble("distanceThresholdM"))
            if (json.has("yawThresholdDeg")) config = config.copy(yawThresholdDeg = json.getDouble("yawThresholdDeg"))
            if (json.has("pitchThresholdDeg")) config = config.copy(pitchThresholdDeg = json.getDouble("pitchThresholdDeg"))
            if (json.has("timeThresholdUs")) config = config.copy(timeThresholdUs = json.getLong("timeThresholdUs"))
            if (json.has("minSpeedMs")) config = config.copy(minSpeedMs = json.getDouble("minSpeedMs"))
            if (json.has("kf_distance_m")) config = config.copy(distanceThresholdM = json.getDouble("kf_distance_m"))
            if (json.has("kf_yaw_deg")) config = config.copy(yawThresholdDeg = json.getDouble("kf_yaw_deg"))
            if (json.has("kf_pitch_deg")) config = config.copy(pitchThresholdDeg = json.getDouble("kf_pitch_deg"))
            if (json.has("kf_time_s")) {
                val sec = json.getDouble("kf_time_s")
                config = config.copy(timeThresholdUs = round(sec * 1_000_000.0).toLong())
            }
            if (json.has("kf_min_speed_ms")) config = config.copy(minSpeedMs = json.getDouble("kf_min_speed_ms"))
            config
        } catch (e: Exception) {
            Log.e(logTag, "Failed to parse keyframe config from JSON, using defaults", e)
            keyframeSelectionConfig
        }
    }

    fun start(scope: CoroutineScope) {
        job = scope.launch(Dispatchers.Default) {
            val timeoutMs = HardwareTier.timeoutMs()
            var report: TelemetryProcessingReport? = null

            val result = runCatching {
                withTimeout(timeoutMs) { process().also { report = it.second }.first }
            }.recoverCatching { e ->
                if (e is TimeoutCancellationException) {
                    throw TelemetryError.Timeout(timeoutMs / 1000.0)
                } else {
                    throw e
                }
            }

            withContext(Dispatchers.Main) {
                result.fold(
                    onSuccess = { seq -> callback.onComplete(seq, null, report) },
                    onFailure = { err ->
                        Log.e(logTag, "Telemetry preprocess failed", err)
                        callback.onComplete(null, err, report)
                    }
                )
            }
        }
    }

    fun cancel() { job?.cancel() }

    private suspend fun process(): Pair<PoseStampSequence, TelemetryProcessingReport> =
        withContext(Dispatchers.Default) {
            val startMs = System.currentTimeMillis()
            val warnings = mutableListOf<String>()

            if (!csvFile.exists()) throw TelemetryError.CsvNotFound(csvFile.absolutePath)
            if (!videoFile.exists()) throw TelemetryError.VideoNotFound(videoFile.absolutePath)

            reportProgress(ProcessingStage.PARSING, 0.0f)
            val parsedRows = CsvParser.parse(csvFile)
            val targetStartUs = readVideoFileStartTimeUs(videoFile)
                ?: CsvParser.parseStartTimeFromFilename(videoFile.name, parsedRows)
            val clippedRows = if (targetStartUs > 0L) {
                parsedRows.filter { it.timestampUs >= targetStartUs }
            } else {
                parsedRows
            }
            reportProgress(ProcessingStage.PARSING, 1.0f)

            reportProgress(ProcessingStage.VALIDATING, 0.0f)
            val validated = RowValidator.validate(clippedRows)
            warnings += validated.warnings
            reportProgress(ProcessingStage.VALIDATING, 1.0f)

            reportProgress(ProcessingStage.CONVERTING, 0.0f)
            val (origin, enuRecords) = EnuConverter.convert(validated.rows)
            val gapInterpolated = GapInterpolator.interpolate(enuRecords)
            val largeScene = gapInterpolated.any { abs(it.enuE) > 50_000.0 || abs(it.enuN) > 50_000.0 }
            reportProgress(ProcessingStage.CONVERTING, 1.0f)

            reportProgress(ProcessingStage.ORIENTING, 0.0f)
            val orientationResult = OrientationFusionEngine.process(gapInterpolated)
            if (orientationResult.compassWarning) {
                warnings += "Compass interference in > 10% of records; headings repaired."
            }
            reportProgress(ProcessingStage.ORIENTING, 1.0f)

            val keyframeCandidates = if (keyframeTimestampsUs.isNotEmpty()) {
                keyframeTimestampsUs.mapIndexed { index, ts ->
                    KeyframeCandidate(index, ts, 0.0, 0.0, 0.0, 0.0, 0.0, KeyframeTrigger.TIME)
                }
            } else {
                selectKeyframes(orientationResult.records, activeKeyframeConfig)
            }

            val keyframeTimesUs = if (keyframeTimestampsUs.isNotEmpty()) {
                keyframeTimestampsUs
            } else {
                LongArray(keyframeCandidates.size) { keyframeCandidates[it].timestampUs }
            }

            val recordsWithTriggers = if (keyframeTimestampsUs.isNotEmpty()) {
                orientationResult.records
            } else {
                val byTimestamp = keyframeCandidates.associateBy { it.timestampUs }
                orientationResult.records.map { record ->
                    val trigger = byTimestamp[record.timestampUs]?.triggerReason
                    if (trigger != null) record.copy(keyframeTrigger = trigger) else record
                }
            }

            reportProgress(ProcessingStage.SYNCING, 0.0f)
            val syncResult = TimeSyncEngine.sync(recordsWithTriggers, videoFile)
            val alignedRecords = recordsWithTriggers.map { it.copy(tsAligned = it.timestampUs + syncResult.offsetUs) }
            reportProgress(ProcessingStage.SYNCING, 1.0f)

            reportProgress(ProcessingStage.INTERPOLATING, 0.0f)
            val interpolated = Interpolator.interpolate(
                alignedRecords,
                keyframeTimesUs,
                IntArray(alignedRecords.size) { alignedRecords[it].flags }
            )
            reportProgress(ProcessingStage.INTERPOLATING, 1.0f)

            reportProgress(ProcessingStage.FLAGGING, 0.0f)
            val quality = QualityFlagger.apply(interpolated, largeScene)
            warnings += quality.warnings
            reportProgress(ProcessingStage.FLAGGING, 1.0f)

            val diagnostics = DiagnosticReporter.build(
                validated = validated,
                keyframes = keyframeCandidates,
                syncOffsetUs = syncResult.offsetUs,
                syncConfidence = syncResult.correlation,
                stageWarnings = warnings
            )
            if (!diagnostics.okToProceed) {
                throw TelemetryError.InternalError(diagnostics.warnings.joinToString("; "))
            }

            validateOutputContract(quality.retained, origin)

            reportProgress(ProcessingStage.EMITTING, 0.0f)
            val sequence = PoseStampSequence(
                origin = origin,
                timeOffsetUs = syncResult.offsetUs,
                records = quality.retained.sortedBy { it.ptsUs },
                sourceMode = TelemetryMode.MODE_C,
                logPath = csvFile,
                videoPath = videoFile,
                createdAt = System.currentTimeMillis(),
                diagnostics = diagnostics
            )
            PoseStampEmitter.emit(sequence, outputDir, sessionId)
            reportProgress(ProcessingStage.EMITTING, 1.0f)

            val report = TelemetryProcessingReport(
                totalCsvRows = clippedRows.size,
                rejectedRows = validated.rejectedCount,
                totalKeyframes = keyframeTimesUs.size,
                outputFrames = quality.retained.size,
                excludedFrames = quality.excluded.size,
                flaggedFrames = quality.retained.count { it.flags != QualityFlag.CLEAN },
                syncOffsetMs = syncResult.offsetUs / 1_000.0,
                syncCorrelation = syncResult.correlation,
                origin = origin,
                processingTimeMs = System.currentTimeMillis() - startMs,
                warnings = diagnostics.warnings,
                gpsValidPct = diagnostics.gpsValidPct,
                imuGapCount = diagnostics.imuGapCount,
                okToProceed = diagnostics.okToProceed
            )

            sequence to report
        }

    private fun validateOutputContract(records: List<PoseStamp>, origin: EnuOrigin) {
        check(records.size >= 8) { "SfM requires at least 8 pose stamps" }
        records.windowed(2).forEach {
            check(it[1].ptsUs > it[0].ptsUs) { "Non-monotonic PTS at frame ${it[1].frameIndex}" }
        }
        records.forEach { p ->
            check(p.enuE.isFinite() && p.enuN.isFinite() && p.enuU.isFinite()) {
                "Non-finite ENU at frame ${p.frameIndex}"
            }
            val norm = sqrt(p.qW * p.qW + p.qX * p.qX + p.qY * p.qY + p.qZ * p.qZ)
            check(abs(norm - 1.0) < 1e-3) { "Non-unit quaternion at frame ${p.frameIndex}" }
        }
        check(origin.cosLat0 > 0.0) { "ENU origin not initialised" }
    }

    private suspend fun reportProgress(stage: ProcessingStage, fraction: Float) {
        withContext(Dispatchers.Main) { callback.onProgress(stage, fraction) }
    }
}

internal object TimeSyncEngine {
    private const val SAMPLE_RATE_US = 5_000_000L
    private const val SEARCH_RANGE_US = 10_000_000L

    data class SyncResult(val offsetUs: Long, val correlation: Double)

    suspend fun sync(records: List<TelRecord>, videoFile: File): SyncResult {
        val altCurve = extractAltitudeCurve(records)
        val videoCurve = extractVideoMotionProxy(videoFile, altCurve.size)
        if (altCurve.size < 3 || videoCurve.size < 3) return SyncResult(0L, 0.0)
        val (offsetUs, corr) = ncc(altCurve, videoCurve, SEARCH_RANGE_US, SAMPLE_RATE_US)
        return SyncResult(offsetUs, corr)
    }

    internal fun flowProxySampleTimesUs(durationMs: Long): LongArray {
        if (durationMs <= 0L) return LongArray(0)
        val count = ((durationMs * 1_000L + SAMPLE_RATE_US - 1) / SAMPLE_RATE_US).toInt()
        return LongArray(count) { index -> index * SAMPLE_RATE_US }
    }

    private fun extractAltitudeCurve(records: List<TelRecord>): DoubleArray {
        if (records.isEmpty()) return DoubleArray(0)
        val start = records.first().timestampUs
        val end = records.last().timestampUs
        val count = (((end - start) / SAMPLE_RATE_US) + 1).toInt().coerceAtLeast(1)
        return DoubleArray(count) { i ->
            val target = start + i * SAMPLE_RATE_US
            interpolateAltitude(records, target)
        }
    }

    private fun interpolateAltitude(records: List<TelRecord>, targetUs: Long): Double {
        val hi = records.indexOfFirst { it.timestampUs >= targetUs }
        return when {
            hi <= 0 -> records.first().enuU
            hi < 0 -> records.last().enuU
            else -> {
                val lo = hi - 1
                val span = (records[hi].timestampUs - records[lo].timestampUs).toDouble().coerceAtLeast(1.0)
                val t = (targetUs - records[lo].timestampUs) / span
                records[lo].enuU + t * (records[hi].enuU - records[lo].enuU)
            }
        }
    }

    private suspend fun extractVideoMotionProxy(videoFile: File, targetLen: Int): DoubleArray {
        if (targetLen <= 0) return DoubleArray(0)
        val result = DoubleArray(targetLen)
        val retriever = MediaMetadataRetriever()
        try {
            retriever.setDataSource(videoFile.absolutePath)
            val durationMs = retriever.extractMetadata(MediaMetadataRetriever.METADATA_KEY_DURATION)?.toLongOrNull() ?: 0L
            val sampleTimes = flowProxySampleTimesUs(durationMs).take(targetLen)
            var previous: Bitmap? = null
            var pxA: IntArray? = null
            var pxB: IntArray? = null
            for ((i, timeUs) in sampleTimes.withIndex()) {
                kotlinx.coroutines.yield()
                val current = retriever.getFrameAtTime(timeUs, MediaMetadataRetriever.OPTION_CLOSEST_SYNC) ?: continue
                if (previous == null) {
                    result[i] = 0.0
                } else {
                    val w = minOf(previous.width, current.width)
                    val h = minOf(previous.height, current.height)
                    val cropW = (w * 0.2).toInt().coerceAtLeast(1)
                    val cropH = (h * 0.2).toInt().coerceAtLeast(1)
                    val size = cropW * cropH
                    if (pxA == null || pxA.size != size) pxA = IntArray(size)
                    if (pxB == null || pxB.size != size) pxB = IntArray(size)
                    
                    result[i] = frameDelta(previous, current, pxA, pxB, w, h, cropW, cropH)
                    previous.recycle()
                }
                previous = current
            }
            previous?.recycle()
        } catch (_: Exception) {
            return DoubleArray(targetLen)
        } finally {
            retriever.release()
        }
        return result
    }

    private fun frameDelta(a: Bitmap, b: Bitmap, pxA: IntArray, pxB: IntArray, w: Int, h: Int, cropW: Int, cropH: Int): Double {
        val left = (w * 0.4).toInt()
        val top = (h * 0.4).toInt()
        a.getPixels(pxA, 0, cropW, left, top, cropW, cropH)
        b.getPixels(pxB, 0, cropW, left, top, cropW, cropH)
        var diff = 0.0
        for (i in pxA.indices) {
            diff += abs(luma(pxA[i]) - luma(pxB[i]))
        }
        return diff / pxA.size
    }

    private fun luma(argb: Int): Double {
        val r = (argb shr 16 and 0xFF) / 255.0
        val g = (argb shr 8 and 0xFF) / 255.0
        val b = (argb and 0xFF) / 255.0
        return 0.2126 * r + 0.7152 * g + 0.0722 * b
    }

    private suspend fun ncc(
        signal: DoubleArray,
        reference: DoubleArray,
        searchRangeUs: Long,
        sampleRateUs: Long
    ): Pair<Long, Double> {
        if (signal.isEmpty() || reference.isEmpty()) return 0L to 0.0
        val sig = normalize(signal)
        val ref = normalize(reference)
        val maxLag = (searchRangeUs / sampleRateUs).toInt()

        var bestLag = 0
        var bestCorr = Double.NEGATIVE_INFINITY
        for (lag in -maxLag..maxLag) {
            kotlinx.coroutines.yield()
            var sum = 0.0
            var n = 0
            for (i in sig.indices) {
                val j = i + lag
                if (j < 0 || j >= ref.size) continue
                sum += sig[i] * ref[j]
                n++
            }
            if (n == 0) continue
            val corr = sum / n
            if (corr > bestCorr) {
                bestCorr = corr
                bestLag = lag
            }
        }
        return bestLag * sampleRateUs to bestCorr
    }

    private fun normalize(values: DoubleArray): DoubleArray {
        if (values.isEmpty()) return values
        val mean = values.average()
        val std = sqrt(values.sumOf { (it - mean).pow(2) } / values.size).coerceAtLeast(1e-9)
        return DoubleArray(values.size) { index -> (values[index] - mean) / std }
    }
}

