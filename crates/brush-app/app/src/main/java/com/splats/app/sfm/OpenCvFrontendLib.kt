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
    private external fun runFullTrainSync(
        framesJson: String,
        intrinsicsJson: String,
        configJson: String,
        gpsJson: String,
        imuJson: String,
        outputDir: String,
        width: Int,
        height: Int,
    ): String

    @JvmStatic
    @WorkerThread
    fun runFullPipelineSync(
        framesJson: String,
        intrinsicsJson: String,
        outputDir: String,
        configJson: String = "{}",
        gpsJson: String = "[]",
        imuJson: String = "[]",
        width: Int,
        height: Int,
    ): String {
        checkWorkerThread()
        return runFullTrainSync(
            framesJson = framesJson,
            intrinsicsJson = intrinsicsJson,
            configJson = configJson,
            gpsJson = gpsJson,
            imuJson = imuJson,
            outputDir = outputDir,
            width = width,
            height = height,
        )
    }

    private fun checkWorkerThread() {
        check(Looper.myLooper() != Looper.getMainLooper()) {
            "SfM pipeline must not run on the main thread"
        }
    }
}
