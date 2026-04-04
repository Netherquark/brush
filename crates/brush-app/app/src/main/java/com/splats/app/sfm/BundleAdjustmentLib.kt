package com.splats.app.sfm

import android.os.Looper
import androidx.annotation.WorkerThread
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.withContext

object BundleAdjustmentLib {

    init {
        System.loadLibrary("brush_process")
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

    @JvmStatic
    @WorkerThread
    fun runBASync(
        posesJson: String,
        pointsJson: String,
        obsJson: String,
        intrinsicsJson: String,
        gpsJson: String = "[]",
        imuJson: String = "[]",
        configJson: String = "{}",
    ): String {
        checkWorkerThread()
        return runSlidingWindowBA(
            posesJson = posesJson,
            pointsJson = pointsJson,
            obsJson = obsJson,
            gpsJson = gpsJson,
            imuJson = imuJson,
            intrinsicsJson = intrinsicsJson,
            configJson = configJson,
        )
    }

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

    private fun checkWorkerThread() {
        check(Looper.myLooper() != Looper.getMainLooper()) {
            "Bundle adjustment must not run on the main thread"
        }
    }
}
