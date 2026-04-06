package com.splats.app;

import androidx.core.view.WindowCompat;
import androidx.core.view.WindowInsetsCompat;
import androidx.core.view.WindowInsetsControllerCompat;

import com.google.androidgamesdk.GameActivity;

import android.annotation.SuppressLint;
import android.app.Activity;
import android.app.AlertDialog;
import android.content.Intent;
import android.database.Cursor;
import android.graphics.Bitmap;
import android.graphics.Color;
import android.graphics.drawable.GradientDrawable;
import android.media.MediaMetadataRetriever;
import android.net.Uri;
import android.os.Build;
import android.os.Bundle;
import android.os.Environment;
import android.os.Handler;
import android.os.Looper;
import android.os.ParcelFileDescriptor;
import android.util.Log;
import android.provider.OpenableColumns;
import android.view.View;
import android.view.WindowManager;
import android.widget.Toast;

import java.io.File;
import java.io.FileOutputStream;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.Locale;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import kotlinx.coroutines.CoroutineScope;

import com.splats.app.sfm.TelemetrySparseReconstruction;
import com.splats.app.telemetry.ActivityCoroutineScope;
import com.splats.app.telemetry.TelemetryPreprocessor;
import com.splats.app.telemetry.TelemetryPreprocessorCallback;
import com.splats.app.telemetry.ProcessingStage;
import com.splats.app.telemetry.PoseStampSequence;
import com.splats.app.telemetry.TelemetryProcessingReport;
import com.splats.app.telemetry.KeyframeSelectionConfig;

public class MainActivity extends GameActivity {

    static {
        System.loadLibrary("brush_app");
    }

    @SuppressLint("StaticFieldLeak")
    public static MainActivity instance;

    private static final int REQUEST_CODE_EXTRACT_FRAMES = 1001;
    private static final int REQUEST_CODE_PICK_CSV = 1002;
    private static final String TAG = "MainActivity";

    private File selectedCsvFile = null;
    private File selectedVideoFile = null;
    private boolean telemetryRunning = false;
    private final ExecutorService backgroundExecutor = Executors.newSingleThreadExecutor();
    private final Handler mainHandler = new Handler(Looper.getMainLooper());
    private CoroutineScope telemetryScope;
    private TelemetryPreprocessor telemetryPreprocessor;

    private void hideSystemUI() {
        getWindow().getAttributes().layoutInDisplayCutoutMode =
                WindowManager.LayoutParams.LAYOUT_IN_DISPLAY_CUTOUT_MODE_ALWAYS;

        View decorView = getWindow().getDecorView();

        WindowInsetsControllerCompat controller =
                new WindowInsetsControllerCompat(getWindow(), decorView);

        controller.hide(WindowInsetsCompat.Type.systemBars());
        controller.hide(WindowInsetsCompat.Type.displayCutout());

        controller.setSystemBarsBehavior(
                WindowInsetsControllerCompat.BEHAVIOR_SHOW_TRANSIENT_BARS_BY_SWIPE
        );
    }


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        instance = this;
        telemetryScope = ActivityCoroutineScope.create();

        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        WindowCompat.setDecorFitsSystemWindows(getWindow(), false);

        hideSystemUI();

        FilePicker.Register(this);
    }

    public static void startTelemetryPickerFlow() {
        if (instance == null) return;
        instance.runOnUiThread(() -> {
            Log.i(TAG, "Starting telemetry picker flow from Rust");
            FilePicker.startCsvPicker(REQUEST_CODE_PICK_CSV);
        });
    }


    @Override
    protected void onDestroy() {
        if (telemetryPreprocessor != null) {
            telemetryPreprocessor.cancel();
            telemetryPreprocessor = null;
        }
        ActivityCoroutineScope.cancel(telemetryScope);
        backgroundExecutor.shutdownNow();
        super.onDestroy();
    }

    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {

        if (requestCode == REQUEST_CODE_PICK_CSV) {
            try {
                if (resultCode == Activity.RESULT_OK && data != null) {
                    Uri uri = data.getData();
                    if (uri != null) {
                        try {
                            int takeFlags = Intent.FLAG_GRANT_READ_URI_PERMISSION;
                            if ((data.getFlags() & Intent.FLAG_GRANT_WRITE_URI_PERMISSION) != 0) {
                                takeFlags |= Intent.FLAG_GRANT_WRITE_URI_PERMISSION;
                            }
                            getContentResolver().takePersistableUriPermission(uri, takeFlags);
                        } catch (Exception e) {
                            Log.i(TAG, "Could not take persistable permission: " + e);
                        }

                        String displayName = queryDisplayName(uri);
                        if (!isCsvName(displayName)) {
                            Log.w(TAG, "Rejected non-CSV telemetry file: " + displayName);
                            Toast.makeText(this,
                                    "Only CSV telemetry logs are supported",
                                    Toast.LENGTH_SHORT).show();
                            return;
                        }

                        selectedCsvFile = ensureLocalFileForUri(uri, "telemetry_csv_", ".csv");

                        if (selectedCsvFile != null) {
                            Toast.makeText(this,
                                    "Telemetry CSV selected: " + selectedCsvFile.getName(),
                                    Toast.LENGTH_SHORT).show();
                            
                            // Now pick the video
                            FilePicker.startFilePicker(REQUEST_CODE_EXTRACT_FRAMES);
                        } else {
                            Toast.makeText(this,
                                    "Failed to read telemetry CSV",
                                    Toast.LENGTH_SHORT).show();
                        }
                    }
                }
            } catch (Exception e) {
                Log.e(TAG, "CSV pick error", e);
            }

            super.onActivityResult(requestCode, resultCode, data);
            return;
        }

        // Keep original handling for the app's FilePicker-based flows
        if (requestCode == FilePicker.REQUEST_CODE_PICK_FILE || requestCode == REQUEST_CODE_EXTRACT_FRAMES) {

            int fd = -1;

            try {

                if (resultCode == Activity.RESULT_OK && data != null) {

                    Uri uri = data.getData();

                    if (uri != null) {
                        // Persist read permission so MediaMetadataRetriever can read later
                        try {
                            int takeFlags = Intent.FLAG_GRANT_READ_URI_PERMISSION;
                            if ((data.getFlags() & Intent.FLAG_GRANT_WRITE_URI_PERMISSION) != 0) {
                                takeFlags |= Intent.FLAG_GRANT_WRITE_URI_PERMISSION;
                            }
                            getContentResolver().takePersistableUriPermission(uri, takeFlags);
                        } catch (Exception e) {
                            // Not all URIs allow persistable permission; ignore safely
                            Log.i("MainActivity", "Could not take persistable permission: " + e);
                        }

                        // Try to open FD for the native/Rust code as before.
                        ParcelFileDescriptor parcelFileDescriptor =
                                getContentResolver().openFileDescriptor(uri, "r");

                        if (parcelFileDescriptor != null) {
                            fd = parcelFileDescriptor.detachFd();

                            // IMPORTANT: first kick off Java extraction (only for our extract request),
                            // then call FilePicker.onPicked so existing native logic still runs.
                            if (requestCode == REQUEST_CODE_EXTRACT_FRAMES) {
                                VideoFrameExtractor.extractFrames(this, uri, new VideoFrameExtractor.ExtractionCallback() {
                                    @Override
                                    public void onFinished() {
                                        startTelemetryPreprocessIfReady();
                                    }

                                    @Override
                                    public void onFailure(Exception e) {
                                        Log.e(TAG, "Frame extraction failed", e);
                                    }
                                });
                            }

                            // keep original behavior for native side
                            FilePicker.onPicked(uri, fd);

                            selectedVideoFile = ensureLocalFileForUri(uri, "telemetry_video_", ".mp4");
                            return;
                        } else {
                            // If no parcelFileDescriptor, still allow extraction if this was the extract request
                            if (requestCode == REQUEST_CODE_EXTRACT_FRAMES) {
                                VideoFrameExtractor.extractFrames(this, uri, new VideoFrameExtractor.ExtractionCallback() {
                                    @Override
                                    public void onFinished() {
                                        startTelemetryPreprocessIfReady();
                                    }

                                    @Override
                                    public void onFailure(Exception e) {
                                        Log.e(TAG, "Frame extraction failed", e);
                                    }
                                });
                                // call native with invalid fd to preserve previous behavior
                                FilePicker.onPicked(uri, -1);
                                selectedVideoFile = ensureLocalFileForUri(uri, "telemetry_video_", ".mp4");
                                return;
                            }
                        }
                    }
                }

            } catch (Exception e) {
                Log.e("MainActivity", "onActivityResult error", e);
            }

            // Ensure the native picker is notified in all failure paths, to match previous behavior
            FilePicker.onPicked(null, -1);
        }

        super.onActivityResult(requestCode, resultCode, data);
    }

    private void startTelemetryPreprocessIfReady() {
        if (telemetryRunning) return;
        if (selectedCsvFile == null || selectedVideoFile == null) return;
        if (!selectedCsvFile.exists() || !selectedVideoFile.exists()) return;

        cleanupTelemetryOutputs();
        telemetryRunning = true;
        Toast.makeText(this, "Starting telemetry preprocess...", Toast.LENGTH_SHORT).show();

        TelemetryPreprocessorCallback callback = new TelemetryPreprocessorCallback() {
            @Override
            public void onProgress(ProcessingStage stage, float fraction) {
                Log.i(TAG, "Telemetry progress: " + stage + " " + fraction);
            }

            @Override
            public void onComplete(PoseStampSequence sequence,
                                   Throwable error,
                                   TelemetryProcessingReport report) {
                telemetryRunning = false;
                if (error == null) {
                    Toast.makeText(MainActivity.this, "Telemetry preprocess complete", Toast.LENGTH_SHORT).show();
                    if (sequence != null) {
                        File telemetryDir = sequence.getLogPath().getParentFile();
                        if (telemetryDir == null) {
                            Log.e(TAG, "Telemetry log path has no parent directory");
                            Toast.makeText(
                                    MainActivity.this,
                                    "Telemetry output directory is unavailable",
                                    Toast.LENGTH_LONG
                            ).show();
                            return;
                        }
                        String sessionBase = stripExtension(sequence.getLogPath().getName());
                        File plyFile = new File(telemetryDir, sessionBase + "_sparse.ply");
                        File resultFile = new File(telemetryDir, sessionBase + "_ba_result.json");

                        backgroundExecutor.execute(() -> runSparseExport(sequence, plyFile, resultFile));

                        Toast.makeText(
                                MainActivity.this,
                                "Output: " + sequence.getLogPath().getParent(),
                                Toast.LENGTH_LONG
                        ).show();
                    }
                } else {
                    Log.e(TAG, "Telemetry preprocess failed", error);
                    String msg = error.getMessage() != null ? error.getMessage() : "Telemetry preprocess failed";
                    Toast.makeText(MainActivity.this, "Telemetry preprocess failed", Toast.LENGTH_LONG).show();
                    Toast.makeText(MainActivity.this, msg, Toast.LENGTH_LONG).show();
                }
            }
        };

        File outDir = getExternalFilesDir("telemetry");
        if (outDir == null) {
            outDir = getCacheDir();
        }
        if (!outDir.exists()) {
            outDir.mkdirs();
        }
        String sessionId = "session_" + System.currentTimeMillis();

        telemetryPreprocessor =
                new TelemetryPreprocessor(
                        selectedCsvFile,
                        selectedVideoFile,
                        new long[0],
                        callback,
                        new KeyframeSelectionConfig(),
                        outDir,
                        sessionId
                );
        telemetryPreprocessor.start(telemetryScope);
    }

    private File ensureLocalFileForUri(Uri uri, String prefix, String fallbackExt) {
        try {
            if (uri == null) return null;

            String name = queryDisplayName(uri);
            String ext = fallbackExt;
            if (name != null && name.contains(".")) {
                ext = name.substring(name.lastIndexOf('.'));
            }
            File out = new File(getCacheDir(), prefix + System.currentTimeMillis() + ext);

            try (InputStream in = getContentResolver().openInputStream(uri);
                 OutputStream outStream = new FileOutputStream(out)) {
                if (in == null) return null;
                byte[] buffer = new byte[8192];
                int read;
                while ((read = in.read(buffer)) != -1) {
                    outStream.write(buffer, 0, read);
                }
            }

            return out;
        } catch (Exception e) {
            Log.e(TAG, "Failed to cache uri " + uri, e);
            return null;
        }
    }

    private boolean isCsvName(String name) {
        if (name == null) return false;
        return name.toLowerCase(Locale.US).endsWith(".csv");
    }

    private String queryDisplayName(Uri uri) {
        if (uri == null) return null;
        Cursor cursor = null;
        try {
            cursor = getContentResolver().query(uri, null, null, null, null);
            if (cursor != null) {
                int nameIndex = cursor.getColumnIndex(OpenableColumns.DISPLAY_NAME);
                if (nameIndex >= 0 && cursor.moveToFirst()) {
                    return cursor.getString(nameIndex);
                }
            }
        } catch (Exception e) {
            Log.w(TAG, "Failed to get display name", e);
        } finally {
            if (cursor != null) cursor.close();
        }
        return null;
    }

    private void runSparseExport(PoseStampSequence sequence, File plyFile, File resultFile) {
        try {
            TelemetrySparseReconstruction.Result result =
                    TelemetrySparseReconstruction.run(this, sequence, plyFile, resultFile);
            final boolean plyExists = plyFile.exists();
            final String toastMessage = plyExists
                    ? "Sparse PLY written: " + plyFile.getAbsolutePath()
                    : "BA finished, but no PLY was written";

            Log.i(TAG, "Sparse reconstruction points: " + result.pointCount + ", matches: " + result.matchCount);
            Log.i(TAG, "Bundle adjustment result saved to " + result.resultFile.getAbsolutePath());
            Log.i(TAG, "Sparse PLY path: " + result.plyFile.getAbsolutePath());

            if (Thread.currentThread().isInterrupted()) {
                Log.i(TAG, "Sparse export interrupted before UI notification");
                return;
            }
            mainHandler.post(() -> {
                if (isFinishing() || isDestroyed()) {
                    Log.i(TAG, "Skipping sparse export toast because activity is finishing");
                    return;
                }
                Toast.makeText(getApplicationContext(), toastMessage, Toast.LENGTH_LONG).show();
            });
        } catch (Exception e) {
            Log.e(TAG, "Sparse export failed", e);
            if (Thread.currentThread().isInterrupted()) {
                Log.i(TAG, "Sparse export interrupted after failure");
                return;
            }
            mainHandler.post(() -> {
                if (isFinishing() || isDestroyed()) {
                    Log.i(TAG, "Skipping sparse export failure toast because activity is finishing");
                    return;
                }
                Toast.makeText(
                        getApplicationContext(),
                        "Sparse export failed: " + e.getMessage(),
                        Toast.LENGTH_LONG
                ).show();
            });
        }
    }

    private String stripExtension(String name) {
        int index = name.lastIndexOf('.');
        if (index <= 0) {
            return name;
        }
        return name.substring(0, index);
    }


    private void cleanupTelemetryOutputs() {
        File telemetryAppDir = getExternalFilesDir("telemetry");
        if (telemetryAppDir != null) {
            deleteFilesMatching(telemetryAppDir, name -> name.endsWith(".posestamps") || name.endsWith(".posestamps.json"));
        }

        if (selectedCsvFile != null) {
            File telemetryDir = selectedCsvFile.getParentFile();
            if (telemetryDir != null) {
                String sessionBase = stripExtension(selectedCsvFile.getName());
                deleteIfExists(new File(telemetryDir, sessionBase + "_sparse.ply"));
                deleteIfExists(new File(telemetryDir, sessionBase + "_ba_result.json"));
            }
        }
    }

    private interface NameMatcher {
        boolean matches(String name);
    }

    private void deleteFilesMatching(File dir, NameMatcher matcher) {
        if (dir == null || !dir.exists() || !dir.isDirectory()) return;
        File[] files = dir.listFiles();
        if (files == null) return;
        for (File file : files) {
            if (file.isFile() && matcher.matches(file.getName())) {
                deleteIfExists(file);
            }
        }
    }

    private void deleteIfExists(File file) {
        if (file == null || !file.exists()) return;
        if (!file.delete()) {
            Log.w(TAG, "Failed to delete stale file: " + file.getAbsolutePath());
        }
    }
}
