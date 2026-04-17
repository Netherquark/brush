package com.splats.app;

import android.app.AlertDialog;
import android.content.Context;
import android.graphics.Bitmap;
import android.media.MediaMetadataRetriever;
import android.net.Uri;
import android.os.Build;
import android.os.Environment;
import android.util.Log;
import android.widget.LinearLayout;
import android.widget.ProgressBar;
import android.widget.TextView;
import android.widget.Toast;

import java.io.File;
import java.io.FileOutputStream;

public class VideoFrameExtractor {
    private static final String TAG = "VideoFrameExtractor";
    /** Default upper bound on longest edge when decoding (speed vs quality). */
    private static final int DEFAULT_MAX_DIMENSION = 360;
    private static final int JPEG_QUALITY = 85;

    public interface ExtractionCallback {
        void onProgress(float progress);
        void onFinished();
        void onFailure(Exception e);
    }

    public static final class Params {
        public int frameCount = 50;
        /** Caps longest image edge during decode (e.g. 144 … 720). Lower = faster. */
        public int maxDecodeDimension = DEFAULT_MAX_DIMENSION;
        /**
         * If non-null and non-empty, extracts at these PTS values (microseconds from video start).
         * Otherwise uses uniform temporal sampling with {@link #frameCount}.
         */
        public long[] timesUsRelative;
    }

    public static void extractFrames(Context context, Uri videoUri, Params params, ExtractionCallback callback) {
        final Params extractionParams = params != null ? params : new Params();
        cleanupExtractedFrames(context);

        final int totalSteps;
        if (extractionParams.timesUsRelative != null && extractionParams.timesUsRelative.length > 0) {
            totalSteps = extractionParams.timesUsRelative.length;
        } else {
            totalSteps = Math.max(1, Math.min(extractionParams.frameCount, 100));
        }

        final AlertDialog[] dialogHolder = new AlertDialog[1];
        final ProgressBar[] progressBarHolder = new ProgressBar[1];
        final TextView[] statusTextHolder = new TextView[1];

        if (context instanceof android.app.Activity) {
            ((android.app.Activity) context).runOnUiThread(() -> {
                try {
                    AlertDialog.Builder builder = new AlertDialog.Builder(context);
                    builder.setTitle("Extracting frames");

                    LinearLayout layout = new LinearLayout(context);
                    layout.setOrientation(LinearLayout.VERTICAL);
                    layout.setPadding(32, 24, 32, 8);

                    ProgressBar progressBar = new ProgressBar(context, null, android.R.attr.progressBarStyleHorizontal);
                    progressBar.setIndeterminate(false);
                    progressBar.setMax(totalSteps);
                    progressBar.setProgress(0);
                    layout.addView(progressBar, new LinearLayout.LayoutParams(LinearLayout.LayoutParams.MATCH_PARENT, LinearLayout.LayoutParams.WRAP_CONTENT));

                    TextView statusText = new TextView(context);
                    statusText.setText("0 / " + totalSteps);
                    statusText.setPadding(0, 12, 0, 0);
                    layout.addView(statusText, new LinearLayout.LayoutParams(LinearLayout.LayoutParams.MATCH_PARENT, LinearLayout.LayoutParams.WRAP_CONTENT));

                    builder.setView(layout);
                    builder.setCancelable(false);

                    AlertDialog dialog = builder.create();
                    dialog.show();

                    dialogHolder[0] = dialog;
                    progressBarHolder[0] = progressBar;
                    statusTextHolder[0] = statusText;
                } catch (Exception e) {
                    Log.w(TAG, "Could not show progress dialog", e);
                }
            });
        }

        final int dim = Math.max(96, Math.min(extractionParams.maxDecodeDimension, 4096));

        final Context appContext = context.getApplicationContext();

        new Thread(() -> {
            int numThreads = Math.max(1, Math.min(4, Runtime.getRuntime().availableProcessors()));
            java.util.concurrent.ExecutorService executor = new java.util.concurrent.ThreadPoolExecutor(
                    numThreads, numThreads,
                    0L, java.util.concurrent.TimeUnit.MILLISECONDS,
                    new java.util.concurrent.ArrayBlockingQueue<>(numThreads * 2),
                    new java.util.concurrent.ThreadPoolExecutor.CallerRunsPolicy());
                    
            java.util.concurrent.ArrayBlockingQueue<MediaMetadataRetriever> retrievers = new java.util.concurrent.ArrayBlockingQueue<>(numThreads);

            try {
                // Determine duration using a single temporary retriever
                long durationUs = 0;
                File outputDir = new File(appContext.getExternalFilesDir(Environment.DIRECTORY_PICTURES), "extracted_frames");
                if (!outputDir.exists()) outputDir.mkdirs();

                java.util.concurrent.atomic.AtomicInteger writtenFrames = new java.util.concurrent.atomic.AtomicInteger(0);

                try (MediaMetadataRetriever infoRetriever = new MediaMetadataRetriever()) {
                    infoRetriever.setDataSource(appContext, videoUri);
                    String durationStr = infoRetriever.extractMetadata(MediaMetadataRetriever.METADATA_KEY_DURATION);
                    if (durationStr != null) {
                        try {
                            durationUs = Long.parseLong(durationStr) * 1000L;
                        } catch (NumberFormatException e) {
                            Log.w(TAG, "Invalid duration metadata", e);
                        }
                    }

                    if (durationUs <= 0 && (extractionParams.timesUsRelative == null || extractionParams.timesUsRelative.length == 0)) {
                        Bitmap single = infoRetriever.getFrameAtTime(0);
                        if (single != null) {
                            single = scaleDownIfNeeded(single, dim);
                            File f = new File(outputDir, "frame_000.jpg");
                            try (FileOutputStream fos = new FileOutputStream(f)) {
                                single.compress(Bitmap.CompressFormat.JPEG, JPEG_QUALITY, fos);
                            }
                            single.recycle();
                            writtenFrames.set(1);
                        }
                        executor.shutdown();
                        finishExtraction(appContext, dialogHolder[0], callback, "Extraction complete (1 frame)");
                        return;
                    }
                }

                // Initialize retriever pool safely
                for (int i = 0; i < numThreads; i++) {
                    MediaMetadataRetriever r = new MediaMetadataRetriever();
                    try {
                        r.setDataSource(appContext, videoUri);
                        retrievers.add(r);
                    } catch (Exception e) {
                        r.release();
                        Log.e(TAG, "Failed to initialize MediaMetadataRetriever " + i, e);
                    }
                }

                if (retrievers.isEmpty()) {
                    throw new IllegalStateException("Failed to initialize any MediaMetadataRetrievers");
                }

                Log.i(TAG, "Starting extraction loop. Steps: " + totalSteps);
                java.util.List<java.util.concurrent.Future<Void>> futures = new java.util.ArrayList<>();
                
                long[] timeUsList = new long[totalSteps];
                for (int i = 0; i < totalSteps; i++) {
                    long timeUs;
                    if (extractionParams.timesUsRelative != null && extractionParams.timesUsRelative.length > 0) {
                        timeUs = extractionParams.timesUsRelative[Math.min(i, extractionParams.timesUsRelative.length - 1)];
                        if (durationUs > 0) {
                            timeUs = Math.min(timeUs, durationUs - 1);
                        }
                    } else {
                        if (totalSteps <= 1) {
                            timeUs = 0;
                        } else {
                            timeUs = (durationUs * (long) i) / (long) (totalSteps - 1);
                        }
                    }
                    timeUsList[i] = timeUs;
                }

                java.util.concurrent.atomic.AtomicInteger progressCounter = new java.util.concurrent.atomic.AtomicInteger(0);

                for (int i = 0; i < totalSteps; i++) {
                    final int stepIndex = i;
                    final long timeUs = timeUsList[i];
                    
                    // Skip redundant seeks if the gap is < 1ms
                    if (stepIndex > 0 && Math.abs(timeUs - timeUsList[stepIndex - 1]) < 1000L) {
                        int c = progressCounter.incrementAndGet();
                        float progress = (float)c / totalSteps;
                        if (callback != null) callback.onProgress(progress);
                        updateProgress(appContext, progressBarHolder[0], statusTextHolder[0], c, totalSteps);
                        continue;
                    }

                    futures.add(executor.submit(() -> {
                        MediaMetadataRetriever retriever = null;
                        try {
                            retriever = retrievers.take();
                            
                            Bitmap bitmap = null;
                            try {
                                int opt = MediaMetadataRetriever.OPTION_CLOSEST_SYNC;
                                if (Build.VERSION.SDK_INT >= 27) {
                                    try {
                                        bitmap = retriever.getScaledFrameAtTime(timeUs, opt, dim, dim);
                                    } catch (Exception t) {
                                        bitmap = retriever.getFrameAtTime(timeUs, opt);
                                    }
                                } else {
                                    bitmap = retriever.getFrameAtTime(timeUs, opt);
                                }

                                if (bitmap == null) {
                                    opt = MediaMetadataRetriever.OPTION_CLOSEST;
                                    if (Build.VERSION.SDK_INT >= 27) {
                                        try {
                                            bitmap = retriever.getScaledFrameAtTime(timeUs, opt, dim, dim);
                                        } catch (Exception t) {
                                            bitmap = retriever.getFrameAtTime(timeUs, opt);
                                        }
                                    } else {
                                        bitmap = retriever.getFrameAtTime(timeUs, opt);
                                    }
                                }

                                if (bitmap != null) {
                                    Bitmap scaled = scaleDownIfNeeded(bitmap, dim);
                                    if (scaled != bitmap) {
                                        bitmap.recycle();
                                        bitmap = scaled;
                                    }
                                    
                                    File outFile = new File(outputDir, String.format("frame_%03d.jpg", stepIndex));
                                    try (FileOutputStream fos = new FileOutputStream(outFile)) {
                                        bitmap.compress(Bitmap.CompressFormat.JPEG, JPEG_QUALITY, fos);
                                        fos.flush();
                                        writtenFrames.incrementAndGet();
                                    } catch (java.io.IOException e) {
                                        Log.e(TAG, "Failed to write frame file " + outFile, e);
                                    }
                                }
                            } catch (OutOfMemoryError oom) {
                                Log.e(TAG, "OOM extracting frame " + stepIndex, oom);
                            } catch (Exception t) {
                                Log.e(TAG, "Error extracting frame " + stepIndex, t);
                            } finally {
                                if (bitmap != null) {
                                    bitmap.recycle();
                                }
                            }
                        } catch (InterruptedException ie) {
                            Thread.currentThread().interrupt();
                        } finally {
                            if (retriever != null) {
                                try {
                                    retrievers.put(retriever);
                                } catch (InterruptedException ignore) {
                                    Thread.currentThread().interrupt();
                                }
                            }
                            int c = progressCounter.incrementAndGet();
                            if (c % 10 == 0) {
                                Log.i(TAG, "Extraction progress: " + c + " of " + totalSteps);
                            }
                            float progress = (float)c / totalSteps;
                            if (callback != null) callback.onProgress(progress);
                            updateProgress(appContext, progressBarHolder[0], statusTextHolder[0], c, totalSteps);
                        }
                        return null;
                    }));
                }

                for (java.util.concurrent.Future<Void> f : futures) {
                    try {
                        f.get();
                    } catch (Exception e) {
                        Log.e(TAG, "Error waiting for frame extraction thread", e);
                    }
                }
                
                executor.shutdown();

                Log.i(TAG, "Extraction finished: wrote " + writtenFrames.get() + "/" + totalSteps
                        + ", outputDir=" + (outputDir != null ? outputDir.getAbsolutePath() : "<null>"));
                finishExtraction(appContext, dialogHolder[0], callback, "Extraction finished");

            } catch (Exception e) {
                Log.e(TAG, "Extraction failed", e);
                failExtraction(appContext, dialogHolder[0], callback, e);
            } finally {
                for (MediaMetadataRetriever r : retrievers) {
                    try {
                        r.release();
                    } catch (Exception ignore) {}
                }
            }
        }).start();
    }

    private static Bitmap scaleDownIfNeeded(Bitmap bitmap, int maxEdge) {
        int w = bitmap.getWidth();
        int h = bitmap.getHeight();
        int maxDim = Math.max(w, h);
        if (maxDim <= maxEdge) {
            return bitmap;
        }
        float scale = (float) maxEdge / (float) maxDim;
        int newW = Math.max(1, Math.round(w * scale));
        int newH = Math.max(1, Math.round(h * scale));
        return Bitmap.createScaledBitmap(bitmap, newW, newH, true);
    }

    private static void updateProgress(Context context, ProgressBar bar, TextView text, int progress, int total) {
        new android.os.Handler(android.os.Looper.getMainLooper()).post(() -> {
            if (bar != null) bar.setProgress(progress);
            if (text != null) text.setText(progress + " / " + total);
        });
    }

    private static void finishExtraction(Context context, AlertDialog dialog, ExtractionCallback callback, String msg) {
        new android.os.Handler(android.os.Looper.getMainLooper()).post(() -> {
            if (dialog != null) dialog.dismiss();
            Toast.makeText(context, msg, Toast.LENGTH_SHORT).show();
            if (callback != null) callback.onFinished();
        });
    }

    private static void failExtraction(Context context, AlertDialog dialog, ExtractionCallback callback, Exception e) {
        new android.os.Handler(android.os.Looper.getMainLooper()).post(() -> {
            if (dialog != null) dialog.dismiss();
            Toast.makeText(context, "Extraction failed: " + e.getMessage(), Toast.LENGTH_LONG).show();
            if (callback != null) callback.onFailure(e);
        });
    }

    public static void cleanupExtractedFrames(Context context) {
        File outputDir = new File(context.getApplicationContext().getExternalFilesDir(Environment.DIRECTORY_PICTURES), "extracted_frames");
        deleteFilesMatching(outputDir, name -> name.startsWith("frame_") && name.endsWith(".jpg"));
    }

    private interface NameMatcher {
        boolean matches(String name);
    }

    private static void deleteFilesMatching(File dir, NameMatcher matcher) {
        if (dir == null || !dir.exists() || !dir.isDirectory()) return;
        File[] files = dir.listFiles();
        if (files == null) return;
        for (File file : files) {
            if (file.isFile() && matcher.matches(file.getName())) {
                deleteIfExists(file);
            }
        }
    }

    private static void deleteIfExists(File file) {
        if (file == null || !file.exists()) return;
        if (!file.delete()) {
            Log.w(TAG, "Failed to delete stale file: " + file.getAbsolutePath());
        }
    }
}
