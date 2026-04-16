package com.splats.app.telemetry

import android.os.Debug
import android.util.Log
import java.io.File

/**
 * Micro-benchmark / instrumentation for PERF-002.
 *
 * Compares allocations + time between:
 *  - Old path: CsvIngest.read() -> CsvParser.parse(headers, dataRows)
 *  - New path: CsvParser.parse(csvFile) streaming
 *
 * This helper is not automatically invoked by the app; call it from a debug context.
 */
internal object CsvParseBench {
    private const val TAG = "CsvParseBench"

    data class Result(
        val impl: String,
        val iters: Int,
        val elapsedMs: Double,
        val allocCountDelta: Long,
        val rows: Int,
    )

    /**
     * Runs [iters] loops and logs a single aggregated result per implementation.
     *
     * Note: `Debug.getThreadAllocCount()` measures allocations on the calling thread.
     */
    fun compareOldVsNew(csvFile: File, targetStartUs: Long = 0L, iters: Int = 3): Pair<Result, Result> {
        require(iters >= 1) { "iters must be >= 1" }

        val old = bench("OLD (ingest+parse)", iters) {
            val (headers, dataRows) = CsvIngest.read(csvFile)
            val rows = CsvParser.parse(headers, dataRows, targetStartUs)
            rows.size
        }

        val new = bench("NEW (streaming)", iters) {
            val rows = CsvParser.parse(csvFile, targetStartUs)
            rows.size
        }

        Log.i(TAG, "CSV parse bench for ${csvFile.name}: old=$old new=$new")
        return old to new
    }

    private inline fun bench(impl: String, iters: Int, block: () -> Int): Result {
        var lastRows = 0
        var elapsedNanosTotal = 0L
        var allocDeltaTotal = 0L

        for (i in 0 until iters) {
            val allocBefore = Debug.getThreadAllocCount()
            val t0 = System.nanoTime()
            lastRows = block()
            val t1 = System.nanoTime()
            val allocAfter = Debug.getThreadAllocCount()

            elapsedNanosTotal += (t1 - t0)
            allocDeltaTotal += (allocAfter - allocBefore)
        }

        return Result(
            impl = impl,
            iters = iters,
            elapsedMs = elapsedNanosTotal.toDouble() / 1_000_000.0,
            allocCountDelta = allocDeltaTotal,
            rows = lastRows,
        )
    }
}

