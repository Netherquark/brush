package com.splats.app.sfm

import android.os.Looper
import androidx.annotation.WorkerThread
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.withContext

object OpenCvFrontendLib {

    init {
        System.loadLibrary("brush_process")
    }

    @JvmStatic
    private external fun runOpenCvFrontend(
        framesJson: String,
        intrinsicsJson: String,
        configJson: String,
        gpsJson: String,
        imuJson: String,
    ): String

    @JvmStatic
    @WorkerThread
    fun runOpenCvFrontendSync(
        framesJson: String,
        intrinsicsJson: String,
        configJson: String = "{}",
        gpsJson: String = "[]",
        imuJson: String = "[]",
    ): String {
        checkWorkerThread()
        return runOpenCvFrontend(
            framesJson = framesJson,
            intrinsicsJson = intrinsicsJson,
            configJson = configJson,
            gpsJson = gpsJson,
            imuJson = imuJson,
        )
    }

    suspend fun runOpenCvFrontendAsync(
        framesJson: String,
        intrinsicsJson: String,
        configJson: String = "{}",
        gpsJson: String = "[]",
        imuJson: String = "[]",
    ): String = withContext(Dispatchers.Default) {
        runOpenCvFrontend(
            framesJson = framesJson,
            intrinsicsJson = intrinsicsJson,
            configJson = configJson,
            gpsJson = gpsJson,
            imuJson = imuJson,
        )
    }

    private fun checkWorkerThread() {
        check(Looper.myLooper() != Looper.getMainLooper()) {
            "OpenCV frontend must not run on the main thread"
        }
    }
}
