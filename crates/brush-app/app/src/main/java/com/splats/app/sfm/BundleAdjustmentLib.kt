// ============================================================
// FILE PATH: crates/brush-app/app/src/main/java/com/splats/app/sfm/BundleAdjustmentLib.kt
//
// PROJECT: brush-app — Drone-Driven On-Device Gaussian Splatting
//
// PURPOSE:
//   Kotlin-side JNI declaration that calls into libbrush_process.so.
//   The `external fun` signature MUST match the symbol exported by
//   stage_3_7_bundle_adjustment.rs Section 9 (jni_bridge module).
//
//   JNI symbol: Java_com_splats_app_sfm_BundleAdjustmentLib_runSlidingWindowBA
//
// HOW TO USE (from SfmPipeline.kt or MainActivity.java):
//   val result = BundleAdjustmentLib.runSlidingWindowBA(
//       posesJson       = gson.toJson(poses),
//       pointsJson      = gson.toJson(points),
//       obsJson         = gson.toJson(observations),
//       gpsJson         = gson.toJson(gpsPriors),
//       imuJson         = gson.toJson(imuPriors),
//       intrinsicsJson  = gson.toJson(intrinsics),
//       configJson      = gson.toJson(SlidingWindowConfig()),   // or "{}" for defaults
//   )
//   val refined = gson.fromJson(result, RefinedState::class.java)
//   if (refined.error != null) { /* handle error */ }
//
// PLACE THIS FILE AT:
//   crates/brush-app/app/src/main/java/com/splats/app/sfm/BundleAdjustmentLib.kt
//   (you will need to CREATE the sfm/ subdirectory)
// ============================================================

package com.splats.app.sfm

import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.withContext

/**
 * Thin Kotlin wrapper around the Rust-native Bundle Adjustment .so.
 *
 * All JSON encoding/decoding is done by the caller — this class only
 * handles the JNI boundary and coroutine dispatch.
 */
object BundleAdjustmentLib {

    init {
        // Loads  crates/brush-app/app/src/main/jniLibs/arm64-v8a/libbrush_process.so
        // This is SEPARATE from libbrush_app.so (the Gaussian splat renderer).
        System.loadLibrary("brush_process")
    }

    // ── JNI declaration ───────────────────────────────────────────────────────
    // Symbol in Rust:
    //   Java_com_splats_app_sfm_BundleAdjustmentLib_runSlidingWindowBA
    //
    // All parameters and return value are JSON strings.
    // Rust handles all heavy computation; Kotlin only passes/receives strings.

    @JvmStatic
    private external fun runSlidingWindowBA(
        posesJson:      String,   // [{"frame_id":0,"rotation_row_major":[...],"translation":[...]}]
        pointsJson:     String,   // [[x,y,z], ...]
        obsJson:        String,   // [{"frame_idx":0,"point_idx":0,"observed":[u,v]}, ...]
        gpsJson:        String,   // [{"frame_idx":0,"enu_position":[e,n,u],"weight":0.5}, ...]
        imuJson:        String,   // [{"frame_a":0,"frame_b":1,"delta_rotation_row_major":[...],"weight":0.2}]
        intrinsicsJson: String,   // {"fx":600,"fy":600,"cx":320,"cy":240}
        configJson:     String,   // {"window_size":15,"overlap":5,"lm_config":{...}} or "{}"
    ): String                     // returns refined state JSON or {"error":"..."}

    // ── Coroutine-safe public API ─────────────────────────────────────────────
    /**
     * Run sliding-window BA on a background dispatcher.
     * Suspends the calling coroutine; never blocks the main thread.
     *
     * @return Refined state JSON string. Check for "error" key before use.
     */
    suspend fun runBA(
        posesJson:      String,
        pointsJson:     String,
        obsJson:        String,
        gpsJson:        String      = "[]",
        imuJson:        String      = "[]",
        intrinsicsJson: String,
        configJson:     String      = "{}",
    ): String = withContext(Dispatchers.Default) {
        runSlidingWindowBA(
            posesJson      = posesJson,
            pointsJson     = pointsJson,
            obsJson        = obsJson,
            gpsJson        = gpsJson,
            imuJson        = imuJson,
            intrinsicsJson = intrinsicsJson,
            configJson     = configJson,
        )
    }
}
