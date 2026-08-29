package com.splats.app.telemetry

import androidx.annotation.WorkerThread
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.withContext

/**
 * Spec §6 / Overhaul §3.1: Thin Kotlin JNI shim.
 *
 * Delegates telemetry validation + keyframe selection to the canonical
 * pure-Rust implementation in brush-sfm (`telemetry.rs`), eliminating
 * cross-language duplication of business logic from RowValidator.kt /
 * KeyframeSelector.kt.
 *
 * Input / output are JSON strings so no additional bindings are needed.
 *
 * recordsJson : JSON array of RawTelemetryRecord (see brush-sfm telemetry.rs)
 * configJson  : Optional JSON object of KeyframeConfig; empty string → defaults
 * Returns     : JSON array of KeyframeCandidate, or {"error":"..."} on failure
 */
object TelemetryLib {

    init {
        System.loadLibrary("brush_process")
    }

    @JvmStatic
    private external fun selectKeyframesFromJson(
        recordsJson: String,
        configJson: String,
    ): String

    @JvmStatic
    @WorkerThread
    fun selectKeyframesSync(
        recordsJson: String,
        configJson: String = "{}",
    ): String = selectKeyframesFromJson(recordsJson, configJson)

    suspend fun selectKeyframes(
        recordsJson: String,
        configJson: String = "{}",
    ): String = withContext(Dispatchers.Default) {
        selectKeyframesFromJson(recordsJson, configJson)
    }
}
