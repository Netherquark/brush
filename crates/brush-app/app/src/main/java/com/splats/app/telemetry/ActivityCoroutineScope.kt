package com.splats.app.telemetry

import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.MainScope
import kotlinx.coroutines.cancel

object ActivityCoroutineScope {
    @JvmStatic
    fun create(): CoroutineScope = MainScope()

    @JvmStatic
    fun cancel(scope: CoroutineScope?) {
        scope?.cancel()
    }
}
