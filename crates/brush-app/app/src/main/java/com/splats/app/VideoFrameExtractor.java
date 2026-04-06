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
    public static final int FRAME_COUNT = 100;
    private static final int MAX_DIMENSION = 4096;

    public interface ExtractionCallback {
        void onFinished();
        void onFailure(Exception e);
    }

    public static void extractFrames(Context context, Uri videoUri, ExtractionCallback callback) {
        cleanupExtractedFrames(context);

        // Show a simple modal progress dialog with a horizontal progress bar
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
                    progressBar.setMax(FRAME_COUNT);
                    progressBar.setProgress(0);
                    layout.addView(progressBar, new LinearLayout.LayoutParams(LinearLayout.LayoutParams.MATCH_PARENT, LinearLayout.LayoutParams.WRAP_CONTENT));

                    TextView statusText = new TextView(context);
                    statusText.setText("0 / " + FRAME_COUNT);
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

        new Thread(() -> {
            try {
                MediaMetadataRetriever retriever = new MediaMetadataRetriever();
                retriever.setDataSource(context, videoUri);

                String durationStr = retriever.extractMetadata(MediaMetadataRetriever.METADATA_KEY_DURATION);
                long durationMs = 0;
                try {
                    durationMs = Long.parseLong(durationStr);
                } catch (Exception e) {
                    Log.w(TAG, "Invalid duration metadata, defaulting to 0", e);
                }

                if (durationMs <= 0) {
                    // fallback: single frame at 0
                    Bitmap single = retriever.getFrameAtTime(0);
                    if (single != null) {
                        File outDir = context.getExternalFilesDir(Environment.DIRECTORY_PICTURES);
                        if (outDir != null && !outDir.exists()) outDir.mkdirs();
                        File f = new File(outDir, "frame_000.jpg");
                        try (FileOutputStream fos = new FileOutputStream(f)) {
                            single.compress(Bitmap.CompressFormat.JPEG, 95, fos);
                        }
                        single.recycle();
                    }
                    retriever.release();
                    finishExtraction(context, dialogHolder[0], callback, "Extraction complete (1 frame)");
                    return;
                }

                long durationUs = durationMs * 1000L;
                long stepUs = durationUs / FRAME_COUNT;

                File outputDir = context.getExternalFilesDir(Environment.DIRECTORY_PICTURES);
                if (outputDir != null && !outputDir.exists()) outputDir.mkdirs();

                for (int i = 0; i < FRAME_COUNT; i++) {
                    long timeUs = i * stepUs;

                    Bitmap bitmap = null;
                    try {
                        if (Build.VERSION.SDK_INT >= 27) {
                            int targetW = MAX_DIMENSION;
                            int targetH = MAX_DIMENSION;
                            try {
                                bitmap = retriever.getScaledFrameAtTime(timeUs, MediaMetadataRetriever.OPTION_CLOSEST, targetW, targetH);
                            } catch (Throwable t) {
                                bitmap = retriever.getFrameAtTime(timeUs, MediaMetadataRetriever.OPTION_CLOSEST);
                            }
                        } else {
                            bitmap = retriever.getFrameAtTime(timeUs, MediaMetadataRetriever.OPTION_CLOSEST);
                        }

                        if (bitmap == null) {
                            updateProgress(context, progressBarHolder[0], statusTextHolder[0], i + 1);
                            continue;
                        }

                        int w = bitmap.getWidth();
                        int h = bitmap.getHeight();
                        if (Math.max(w, h) > MAX_DIMENSION) {
                            float scale = (float) MAX_DIMENSION / (float) Math.max(w, h);
                            int newW = Math.max(1, Math.round(w * scale));
                            int newH = Math.max(1, Math.round(h * scale));
                            Bitmap scaled = Bitmap.createScaledBitmap(bitmap, newW, newH, true);
                            bitmap.recycle();
                            bitmap = scaled;
                        }

                        File outFile = new File(outputDir, String.format("frame_%03d.jpg", i));
                        try (FileOutputStream fos = new FileOutputStream(outFile)) {
                            bitmap.compress(Bitmap.CompressFormat.JPEG, 95, fos);
                            fos.flush();
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

                    updateProgress(context, progressBarHolder[0], statusTextHolder[0], i + 1);
                }

                retriever.release();
                finishExtraction(context, dialogHolder[0], callback, "Extraction finished");

            } catch (Exception e) {
                Log.e(TAG, "Extraction failed", e);
                failExtraction(context, dialogHolder[0], callback, e);
            }
        }).start();
    }

    private static void updateProgress(Context context, ProgressBar bar, TextView text, int progress) {
        if (context instanceof android.app.Activity) {
            ((android.app.Activity) context).runOnUiThread(() -> {
                if (bar != null) bar.setProgress(progress);
                if (text != null) text.setText(progress + " / " + FRAME_COUNT);
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
