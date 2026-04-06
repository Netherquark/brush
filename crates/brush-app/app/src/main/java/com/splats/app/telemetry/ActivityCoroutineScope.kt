package com.splats.app.telemetry

import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.SupervisorJob
import kotlinx.coroutines.cancel

object ActivityCoroutineScope {
    @JvmStatic
    fun create(): CoroutineScope = CoroutineScope(SupervisorJob() + Dispatchers.Default)

    @JvmStatic
    fun cancel(scope: CoroutineScope?) {
        scope?.cancel()
    }
}
