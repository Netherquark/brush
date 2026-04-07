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
    ): String

    @JvmStatic
    @WorkerThread
    fun runOpenCvFrontendSync(
        framesJson: String,
        intrinsicsJson: String,
        configJson: String = "{}",
    ): String {
        checkWorkerThread()
        return runOpenCvFrontend(
            framesJson = framesJson,
            intrinsicsJson = intrinsicsJson,
            configJson = configJson,
        )
    }

    suspend fun runOpenCvFrontendAsync(
        framesJson: String,
        intrinsicsJson: String,
        configJson: String = "{}",
    ): String = withContext(Dispatchers.Default) {
        runOpenCvFrontend(
            framesJson = framesJson,
            intrinsicsJson = intrinsicsJson,
            configJson = configJson,
        )
    }

    private fun checkWorkerThread() {
        check(Looper.myLooper() != Looper.getMainLooper()) {
            "OpenCV frontend must not run on the main thread"
        }
    }
}
