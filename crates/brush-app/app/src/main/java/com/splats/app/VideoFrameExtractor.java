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

        new Thread(() -> {
            try {
                MediaMetadataRetriever retriever = new MediaMetadataRetriever();
                retriever.setDataSource(context, videoUri);
                Log.i(TAG, "Starting frame extraction for " + videoUri + " dim=" + dim + " steps=" + totalSteps);

                String durationStr = retriever.extractMetadata(MediaMetadataRetriever.METADATA_KEY_DURATION);
                long durationMs = 0;
                try {
                    durationMs = Long.parseLong(durationStr);
                } catch (Exception e) {
                    Log.w(TAG, "Invalid duration metadata, defaulting to 0", e);
                }
                long durationUs = durationMs * 1000L;

                File outputDir = context.getExternalFilesDir(Environment.DIRECTORY_PICTURES);
                if (outputDir != null && !outputDir.exists()) outputDir.mkdirs();
                int writtenFrames = 0;

                if (durationMs <= 0 && (extractionParams.timesUsRelative == null || extractionParams.timesUsRelative.length == 0)) {
                    Bitmap single = retriever.getFrameAtTime(0);
                    if (single != null) {
                        single = scaleDownIfNeeded(single, dim);
                        File f = new File(outputDir, "frame_000.jpg");
                        try (FileOutputStream fos = new FileOutputStream(f)) {
                            single.compress(Bitmap.CompressFormat.JPEG, JPEG_QUALITY, fos);
                        }
                        single.recycle();
                        writtenFrames = 1;
                    }
                    retriever.release();
                    finishExtraction(context, dialogHolder[0], callback, "Extraction complete (1 frame)");
                    return;
                }

                for (int i = 0; i < totalSteps; i++) {
                    long timeUs;
                    if (extractionParams.timesUsRelative != null && extractionParams.timesUsRelative.length > 0) {
                        timeUs = extractionParams.timesUsRelative[Math.min(i, extractionParams.timesUsRelative.length - 1)];
                        if (durationUs > 0) {
                            timeUs = Math.min(timeUs, durationUs - 1);
                        }
                    } else {
                        long stepUs = durationUs / totalSteps;
                        timeUs = i * stepUs;
                    }

                    Bitmap bitmap = null;
                    try {
                        int opt = MediaMetadataRetriever.OPTION_CLOSEST_SYNC;
                        if (Build.VERSION.SDK_INT >= 27) {
                            try {
                                bitmap = retriever.getScaledFrameAtTime(timeUs, opt, dim, dim);
                            } catch (Throwable t) {
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
                                } catch (Throwable t) {
                                    bitmap = retriever.getFrameAtTime(timeUs, opt);
                                }
                            } else {
                                bitmap = retriever.getFrameAtTime(timeUs, opt);
                            }
                        }

                        if (bitmap == null) {
                            updateProgress(context, progressBarHolder[0], statusTextHolder[0], i + 1, totalSteps);
                            continue;
                        }

                        bitmap = scaleDownIfNeeded(bitmap, dim);

                        File outFile = new File(outputDir, String.format("frame_%03d.jpg", i));
                        try (FileOutputStream fos = new FileOutputStream(outFile)) {
                            bitmap.compress(Bitmap.CompressFormat.JPEG, JPEG_QUALITY, fos);
                            fos.flush();
                            writtenFrames++;
                        } catch (Exception e) {
                            Log.e(TAG, "Failed to write frame file " + outFile, e);
                        } finally {
                            bitmap.recycle();
                        }
                    } catch (OutOfMemoryError oom) {
                        Log.e(TAG, "OOM extracting frame " + i, oom);
                    } catch (Throwable t) {
                        Log.e(TAG, "Error extracting frame " + i, t);
                    }

                    updateProgress(context, progressBarHolder[0], statusTextHolder[0], i + 1, totalSteps);
                }

                retriever.release();
                Log.i(TAG, "Extraction finished: wrote " + writtenFrames + "/" + totalSteps
                        + ", outputDir=" + (outputDir != null ? outputDir.getAbsolutePath() : "<null>"));
                finishExtraction(context, dialogHolder[0], callback, "Extraction finished");

            } catch (Exception e) {
                Log.e(TAG, "Extraction failed", e);
                failExtraction(context, dialogHolder[0], callback, e);
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
        Bitmap scaled = Bitmap.createScaledBitmap(bitmap, newW, newH, true);
        bitmap.recycle();
        return scaled;
    }

    private static void updateProgress(Context context, ProgressBar bar, TextView text, int progress, int total) {
        if (context instanceof android.app.Activity) {
            ((android.app.Activity) context).runOnUiThread(() -> {
                if (bar != null) bar.setProgress(progress);
                if (text != null) text.setText(progress + " / " + total);
            });
        }
    }

    private static void finishExtraction(Context context, AlertDialog dialog, ExtractionCallback callback, String msg) {
        if (context instanceof android.app.Activity) {
            ((android.app.Activity) context).runOnUiThread(() -> {
                if (dialog != null) dialog.dismiss();
                Toast.makeText(context, msg, Toast.LENGTH_SHORT).show();
                if (callback != null) callback.onFinished();
            });
        }
    }

    private static void failExtraction(Context context, AlertDialog dialog, ExtractionCallback callback, Exception e) {
        if (context instanceof android.app.Activity) {
            ((android.app.Activity) context).runOnUiThread(() -> {
                if (dialog != null) dialog.dismiss();
                Toast.makeText(context, "Extraction failed: " + e.getMessage(), Toast.LENGTH_LONG).show();
                if (callback != null) callback.onFailure(e);
            });
        }
    }

    public static void cleanupExtractedFrames(Context context) {
        File outputDir = context.getExternalFilesDir(Environment.DIRECTORY_PICTURES);
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
