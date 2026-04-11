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

import org.json.JSONException;
import org.json.JSONObject;

import kotlinx.coroutines.CoroutineScope;

import com.splats.app.sfm.TelemetrySparseReconstruction;
import com.splats.app.telemetry.ActivityCoroutineScope;
import com.splats.app.telemetry.TelemetryPreprocessor;
import com.splats.app.telemetry.TelemetryPreprocessorCallback;
import com.splats.app.telemetry.ProcessingStage;
import com.splats.app.telemetry.PoseStampSequence;
import com.splats.app.telemetry.TelemetryProcessingReport;
import com.splats.app.telemetry.KeyframePlanner;
import com.splats.app.telemetry.KeyframeSelectionConfig;

public class MainActivity extends GameActivity {

    static {
        System.loadLibrary("tbb");
        System.loadLibrary("opencv_core");
        System.loadLibrary("opencv_imgproc");
        System.loadLibrary("opencv_imgcodecs");
        System.loadLibrary("opencv_features2d");
        System.loadLibrary("opencv_flann");
        System.loadLibrary("opencv_calib3d");
        System.loadLibrary("brush_app");
    }

    @SuppressLint("StaticFieldLeak")
    public static MainActivity instance;

    private static native void notifyPlatformEvent(String event, String data);

    // ── Request codes ─────────────────────────────────────────────────────────
    private static final int REQUEST_CODE_CHOOSE_MP4         = 1000;
    private static final int REQUEST_CODE_PICK_CSV           = 1002;
    private static final int REQUEST_CODE_PICK_CONFIG        = 1003;

    // ── JNI-callable static methods (called from Rust via platform callbacks) ─

    /** Button 2 – just pick an MP4, store it, no extraction yet */
    public static void chooseMp4() {
        if (instance == null) return;
        instance.runOnUiThread(() -> {
            Log.i(TAG, "chooseMp4 from Rust");
            FilePicker.startFilePicker(REQUEST_CODE_CHOOSE_MP4);
        });
    }

    /** Button 3 – extract frames from the already-selected MP4 (see {@link #chooseMp4()}). */
    public static void extractFrames(String configJson) {
        if (instance == null) return;
        instance.runOnUiThread(() -> {
            Log.i(TAG, "extractFrames from Rust");
            instance.extractFramesFromSelectedVideo(configJson != null ? configJson : "{}");
        });
    }

    /** Button 4 – pick a CSV telemetry log */
    public static void chooseCsv() {
        if (instance == null) return;
        instance.runOnUiThread(() -> {
            Log.i(TAG, "chooseCsv from Rust");
            FilePicker.startCsvPicker(REQUEST_CODE_PICK_CSV);
        });
    }

    /** Button 5 – pick a Config JSON */
    public static void chooseConfig() {
        if (instance == null) return;
        instance.runOnUiThread(() -> {
            Log.i(TAG, "chooseConfig from Rust");
            FilePicker.startFilePicker(REQUEST_CODE_PICK_CONFIG);
        });
    }

    /** Unified Train button – runs full SfM pipeline (3.1 - 3.8) */
    public static void runTrain(String configJson) {
        if (instance == null) return;
        instance.runOnUiThread(() -> {
            Log.i(TAG, "runTrain from Rust");
            instance.applyPipelineConfigJson(configJson != null ? configJson : "{}");
            instance.startTelemetryPreprocessIfReady();
        });
    }

    public static void runTelemetry() {
        runTrain("{}");
    }



    // ── Instance fields ───────────────────────────────────────────────────────
    private static final String TAG = "MainActivity";

    private File selectedCsvFile = null;
    private File selectedVideoFile = null;
    private File selectedConfigFile = null;
    private boolean telemetryRunning = false;
    /** Native SfM / OpenCV frontend JSON (orb, BA window, LM iterations). */
    private String nativeSfmConfigJson = "{}";
    private KeyframeSelectionConfig keyframeSelectionConfig =
            new KeyframeSelectionConfig(2.0, 8.0, 5.0, 1_000_000L, 0.2);
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
        if (savedInstanceState == null) {
            cleanupCachedMedia();
        }
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

        // ── Config JSON picker (Choose Config) ─────────────────────────────
        if (requestCode == REQUEST_CODE_PICK_CONFIG) {
            if (resultCode == Activity.RESULT_OK && data != null) {
                Uri uri = data.getData();
                if (uri != null) {
                    try {
                        int takeFlags = Intent.FLAG_GRANT_READ_URI_PERMISSION;
                        if ((data.getFlags() & Intent.FLAG_GRANT_WRITE_URI_PERMISSION) != 0)
                            takeFlags |= Intent.FLAG_GRANT_WRITE_URI_PERMISSION;
                        getContentResolver().takePersistableUriPermission(uri, takeFlags);
                    } catch (Exception e) {
                        Log.i(TAG, "Could not take persistable permission: " + e);
                    }
                    selectedConfigFile = ensureLocalFileForUri(uri, "telemetry_config_", ".json");
                    if (selectedConfigFile != null) {
                        Toast.makeText(this, "Config: " + selectedConfigFile.getName(),
                                Toast.LENGTH_SHORT).show();
                        notifyPlatformEvent("config_picked", selectedConfigFile.getAbsolutePath());
                    } else {
                        Toast.makeText(this, "Failed to read config file", Toast.LENGTH_SHORT).show();
                    }
                }
            }
            super.onActivityResult(requestCode, resultCode, data);
            return;
        }

        // ── CSV picker (Button 4: Choose CSV) ───────────────────────────────
        if (requestCode == REQUEST_CODE_PICK_CSV) {
            if (resultCode == Activity.RESULT_OK && data != null) {
                Uri uri = data.getData();
                if (uri != null) {
                    try {
                        int takeFlags = Intent.FLAG_GRANT_READ_URI_PERMISSION;
                        if ((data.getFlags() & Intent.FLAG_GRANT_WRITE_URI_PERMISSION) != 0)
                            takeFlags |= Intent.FLAG_GRANT_WRITE_URI_PERMISSION;
                        getContentResolver().takePersistableUriPermission(uri, takeFlags);
                    } catch (Exception e) {
                        Log.i(TAG, "Could not take persistable permission: " + e);
                    }

                    String displayName = queryDisplayName(uri);
                    if (!isCsvName(displayName)) {
                        Toast.makeText(this, "Only CSV telemetry logs are supported",
                                Toast.LENGTH_SHORT).show();
                    } else {
                        selectedCsvFile = ensureLocalFileForUri(uri, "telemetry_csv_", ".csv");
                        if (selectedCsvFile != null) {
                            Toast.makeText(this, "CSV selected: " + selectedCsvFile.getName(),
                                    Toast.LENGTH_SHORT).show();
                            notifyPlatformEvent("csv_picked", selectedCsvFile.getAbsolutePath());
                        } else {
                            Toast.makeText(this, "Failed to read telemetry CSV",
                                    Toast.LENGTH_SHORT).show();
                        }
                    }
                }
            }
            // Do NOT call FilePicker.onPicked — this never goes through Rust
            super.onActivityResult(requestCode, resultCode, data);
            return;
        }

        // ── Choose MP4 only (Button 2: Choose MP4) ───────────────────────
        if (requestCode == REQUEST_CODE_CHOOSE_MP4) {
            if (resultCode == Activity.RESULT_OK && data != null) {
                Uri uri = data.getData();
                if (uri != null) {
                    selectedVideoFile = ensureLocalFileForUri(uri, "telemetry_video_", ".mp4");
                    if (selectedVideoFile != null) {
                        Toast.makeText(this, "MP4 selected: " + selectedVideoFile.getName(),
                                Toast.LENGTH_SHORT).show();
                        notifyPlatformEvent("mp4_picked", selectedVideoFile.getAbsolutePath());
                    } else {
                        Toast.makeText(this, "Failed to read MP4", Toast.LENGTH_SHORT).show();
                    }
                }
            }
            // Do NOT call FilePicker.onPicked — this never goes through Rust
            super.onActivityResult(requestCode, resultCode, data);
            return;
        }

        // ── Regular Rust file picker (Button 1: File .ply viewer) ────────────
        if (requestCode == FilePicker.REQUEST_CODE_PICK_FILE) {
            int fd = -1;
            try {
                if (resultCode == Activity.RESULT_OK && data != null) {
                    Uri uri = data.getData();
                    if (uri != null) {
                        try {
                            int takeFlags = Intent.FLAG_GRANT_READ_URI_PERMISSION;
                            if ((data.getFlags() & Intent.FLAG_GRANT_WRITE_URI_PERMISSION) != 0)
                                takeFlags |= Intent.FLAG_GRANT_WRITE_URI_PERMISSION;
                            getContentResolver().takePersistableUriPermission(uri, takeFlags);
                        } catch (Exception e) {
                            Log.i(TAG, "Could not take persistable permission: " + e);
                        }
                        ParcelFileDescriptor pfd = getContentResolver().openFileDescriptor(uri, "r");
                        if (pfd != null) {
                            fd = pfd.detachFd();
                        }
                        FilePicker.onPicked(uri, fd);
                        super.onActivityResult(requestCode, resultCode, data);
                        return;
                    }
                }
            } catch (Exception e) {
                Log.e(TAG, "onActivityResult error", e);
            }
            FilePicker.onPicked(null, -1);
        }

        super.onActivityResult(requestCode, resultCode, data);
    }

    private void startTelemetryPreprocessIfReady() {
        if (telemetryRunning) return;
        if (selectedCsvFile == null || selectedVideoFile == null) {
            Toast.makeText(this, "Choose MP4 and CSV before Train", Toast.LENGTH_SHORT).show();
            notifyPlatformEvent("train_not_ready", "");
            return;
        }
        if (!selectedCsvFile.exists() || !selectedVideoFile.exists()) {
            Toast.makeText(this, "MP4 or CSV file is missing — re-select files", Toast.LENGTH_SHORT).show();
            notifyPlatformEvent("train_not_ready", "");
            return;
        }

        cleanupTelemetryOutputs();

        String configJsonStr = "{}";
        if (selectedConfigFile != null && selectedConfigFile.exists()) {
            try {
                java.nio.file.Path p = selectedConfigFile.toPath();
                configJsonStr = new String(java.nio.file.Files.readAllBytes(p));
            } catch (Exception e) {
                Log.e(TAG, "Failed to read config file", e);
            }
        }
        final String finalConfigJsonStr = configJsonStr;

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
                            notifyPlatformEvent("train_not_ready", "");
                            return;
                        }
                        String sessionBase = stripExtension(sequence.getLogPath().getName());
                        File plyFile = new File(telemetryDir, sessionBase + "_sparse.ply");
                        File resultFile = new File(telemetryDir, sessionBase + "_ba_result.json");

                        notifyPlatformEvent("telemetry_complete", plyFile.getAbsolutePath());

                        backgroundExecutor.execute(() -> runSparseExport(sequence, plyFile, resultFile, finalConfigJsonStr));

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
                    notifyPlatformEvent("train_not_ready", "");
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
                        keyframeSelectionConfig,
                        outDir,
                        sessionId,
                        finalConfigJsonStr
                );
        telemetryPreprocessor.start(telemetryScope);
    }

    private void applyPipelineConfigJson(String json) {
        PipelineConfig cfg = PipelineConfig.parse(json);
        keyframeSelectionConfig = cfg.toKeyframeSelectionConfig();
        nativeSfmConfigJson = cfg.toNativeSfmConfigJson();
    }

    private void extractFramesFromSelectedVideo(String json) {
        PipelineConfig cfg = PipelineConfig.parse(json);
        if (selectedVideoFile == null || !selectedVideoFile.exists()) {
            Toast.makeText(this, "Choose an MP4 first", Toast.LENGTH_SHORT).show();
            notifyPlatformEvent("extraction_complete", "");
            return;
        }
        Uri uri = Uri.fromFile(selectedVideoFile);
        VideoFrameExtractor.Params params = new VideoFrameExtractor.Params();
        params.frameCount = cfg.frameCount;
        params.maxDecodeDimension = cfg.maxFrameDimension;

        if ("telemetry".equalsIgnoreCase(cfg.extractionMode)) {
            if (selectedCsvFile == null || !selectedCsvFile.exists()) {
                Toast.makeText(this, "Telemetry mode needs a CSV — choose CSV first", Toast.LENGTH_LONG).show();
                notifyPlatformEvent("extraction_complete", "");
                return;
            }
            KeyframeSelectionConfig kfCfg = cfg.toKeyframeSelectionConfig();
            long[] timesUs = KeyframePlanner.videoRelativeKeyframeTimesUs(
                    selectedCsvFile,
                    selectedVideoFile,
                    kfCfg,
                    cfg.frameCount
            );
            if (timesUs.length == 0) {
                Toast.makeText(this, "No telemetry keyframes — check CSV and video", Toast.LENGTH_LONG).show();
                notifyPlatformEvent("extraction_complete", "");
                return;
            }
            params.timesUsRelative = timesUs;
        }

        VideoFrameExtractor.extractFrames(this, uri, params, new VideoFrameExtractor.ExtractionCallback() {
            @Override
            public void onFinished() {
                Toast.makeText(MainActivity.this, "Frames extracted!", Toast.LENGTH_SHORT).show();
                notifyPlatformEvent("extraction_complete", "");
            }

            @Override
            public void onFailure(Exception e) {
                Log.e(TAG, "Frame extraction failed", e);
                Toast.makeText(MainActivity.this, "Extraction failed: " + e.getMessage(),
                        Toast.LENGTH_SHORT).show();
                notifyPlatformEvent("extraction_complete", "");
            }
        });
    }

    private File ensureLocalFileForUri(Uri uri, String prefix, String fallbackExt) {
        try {
            if (uri == null) return null;

            String originalName = queryDisplayName(uri);
            if (originalName == null || originalName.trim().isEmpty()) {
                String lastSegment = uri.getLastPathSegment();
                if (lastSegment != null && !lastSegment.trim().isEmpty()) {
                    originalName = lastSegment;
                }
            }

            String ext = fallbackExt;
            if (originalName != null && originalName.contains(".")) {
                ext = originalName.substring(originalName.lastIndexOf('.'));
            }

            String safeName = (originalName != null) ? originalName.replaceAll("[^a-zA-Z0-9.\\-_]", "_") : System.currentTimeMillis() + ext;
            File out = new File(getCacheDir(), safeName);
            if (out.exists()) {
                String baseName = stripExtension(safeName);
                String fileExt = safeName.contains(".")
                        ? safeName.substring(safeName.lastIndexOf('.'))
                        : ext;
                out = new File(
                        getCacheDir(),
                        prefix + baseName + "_" + System.currentTimeMillis() + fileExt
                );
            }

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

    private void runSparseExport(PoseStampSequence sequence, File plyFile, File resultFile, String configJsonStr) {
        try {
            TelemetrySparseReconstruction.Result result =
                    TelemetrySparseReconstruction.run(this, sequence, plyFile, resultFile, nativeSfmConfigJson);
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

    private void cleanupCachedMedia() {
        File cacheDir = getCacheDir();
        if (cacheDir != null) {
            deleteFilesMatching(cacheDir, name -> name.startsWith("telemetry_csv_") || name.startsWith("telemetry_video_"));
        }
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

    /** Mirrors the JSON emitted from the Rust welcome screen (`AndroidPipelineConfig`). */
    private static final class PipelineConfig {
        String extractionMode = "uniform";
        int maxFrameDimension = 360;
        int frameCount = 50;
        int orbKeypoints = 512;
        int baWindowSize = 5;
        int lmMaxIterations = 50;
        double kfDistanceM = 2.0;
        double kfYawDeg = 8.0;
        double kfPitchDeg = 5.0;
        double kfTimeS = 1.0;
        double kfMinSpeedMs = 0.2;

        static PipelineConfig parse(String raw) {
            PipelineConfig c = new PipelineConfig();
            if (raw == null || raw.trim().isEmpty()) {
                return c;
            }
            try {
                JSONObject o = new JSONObject(raw);
                if (o.has("extraction_mode")) {
                    c.extractionMode = o.optString("extraction_mode", c.extractionMode);
                }
                c.maxFrameDimension = clamp(o.optInt("max_frame_dimension", c.maxFrameDimension), 144, 720);
                c.frameCount = clamp(o.optInt("frame_count", c.frameCount), 1, 100);
                c.orbKeypoints = clamp(o.optInt("orb_keypoints", c.orbKeypoints), 50, 1000);
                c.baWindowSize = clamp(o.optInt("ba_window_size", c.baWindowSize), 2, 5);
                c.lmMaxIterations = clamp(o.optInt("lm_max_iterations", c.lmMaxIterations), 1, 2000);
                c.kfDistanceM = o.optDouble("kf_distance_m", c.kfDistanceM);
                c.kfYawDeg = o.optDouble("kf_yaw_deg", c.kfYawDeg);
                c.kfPitchDeg = o.optDouble("kf_pitch_deg", c.kfPitchDeg);
                c.kfTimeS = o.optDouble("kf_time_s", c.kfTimeS);
                c.kfMinSpeedMs = o.optDouble("kf_min_speed_ms", c.kfMinSpeedMs);
            } catch (JSONException e) {
                Log.w(TAG, "Pipeline config JSON parse failed, using defaults", e);
            }
            return c;
        }

        private static int clamp(int v, int lo, int hi) {
            return Math.max(lo, Math.min(hi, v));
        }

        KeyframeSelectionConfig toKeyframeSelectionConfig() {
            long timeUs = (long) Math.round(kfTimeS * 1_000_000L);
            return new KeyframeSelectionConfig(
                    kfDistanceM,
                    kfYawDeg,
                    kfPitchDeg,
                    timeUs,
                    kfMinSpeedMs
            );
        }

        String toNativeSfmConfigJson() {
            try {
                JSONObject o = new JSONObject();
                o.put("orb_n_features", orbKeypoints);
                o.put("max_matches", Math.min(orbKeypoints, 2000));
                o.put("ba_window_size", baWindowSize);
                o.put("lm_max_iterations", lmMaxIterations);
                return o.toString();
            } catch (JSONException e) {
                return "{}";
            }
        }
    }
}
