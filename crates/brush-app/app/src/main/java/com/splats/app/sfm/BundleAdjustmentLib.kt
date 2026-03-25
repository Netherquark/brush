package com.splats.app.sfm

import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.withContext

object BundleAdjustmentLib {

    init {
        System.loadLibrary("brush_sfm")
    }

    @JvmStatic
    private external fun runSlidingWindowBA(
        posesJson: String,
        pointsJson: String,
        obsJson: String,
        gpsJson: String,
        imuJson: String,
        intrinsicsJson: String,
        configJson: String,
    ): String

    suspend fun runBA(
        posesJson: String,
        pointsJson: String,
        obsJson: String,
        intrinsicsJson: String,
        gpsJson: String = "[]",
        imuJson: String = "[]",
        configJson: String = "{}",
    ): String = withContext(Dispatchers.Default) {
        runSlidingWindowBA(
            posesJson = posesJson,
            pointsJson = pointsJson,
            obsJson = obsJson,
            gpsJson = gpsJson,
            imuJson = imuJson,
            intrinsicsJson = intrinsicsJson,
            configJson = configJson,
        )
    }
}
