package com.splats.app.sfm;

import android.content.Context;
import android.media.MediaMetadataRetriever;
import android.os.Environment;
import android.util.Log;

import com.splats.app.telemetry.PoseStamp;
import com.splats.app.telemetry.PoseStampSequence;

import org.json.JSONArray;
import org.json.JSONObject;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

public final class TelemetrySparseReconstruction {

    private static final String TAG = "TelemetrySparseRecon";

    private TelemetrySparseReconstruction() {}

    public static Result run(
            Context context,
            PoseStampSequence sequence,
            File plyFile,
            File resultFile,
            String nativeConfigJson
    ) throws Exception {
        if (sequence.getRecords().size() < 2) {
            throw new IllegalStateException("Need at least 2 pose records for sparse reconstruction");
        }

        File framesDir = context.getExternalFilesDir(Environment.DIRECTORY_PICTURES);
        if (framesDir == null || !framesDir.exists()) {
            throw new IllegalStateException("Extracted frames directory is missing");
        }

        int filesOnDisk = countExtractedFrames(framesDir);
        Log.i(TAG, "Preparing sparse reconstruction with "
                + sequence.getRecords().size() + " pose record(s), "
                + filesOnDisk + " extracted frame file(s) in " + framesDir.getAbsolutePath());

        VideoIntrinsics intrinsics = estimateIntrinsics(sequence.getVideoPath());
        FrameLoadSummary frameSummary = loadFramesForSequence(sequence, framesDir);
        List<FrameData> frames = frameSummary.frames;
        Log.i(TAG, "Matched " + frames.size() + "/" + frameSummary.expectedCount
                + " pose frame(s) to extracted images");
        if (!frameSummary.missingFrameNames.isEmpty()) {
            Log.w(TAG, "Missing extracted frame files (sample): " + frameSummary.missingFrameNames);
        }
        if (frames.size() < 2) {
            throw new IllegalStateException(
                    "Could not load enough extracted frames. matched=" + frames.size()
                            + ", expected=" + frameSummary.expectedCount
                            + ", filesOnDisk=" + filesOnDisk
                            + ", dir=" + framesDir.getAbsolutePath()
                            + ", missingSample=" + frameSummary.missingFrameNames
            );
        }

        JSONArray gps = new JSONArray();
        JSONArray imu = new JSONArray();

        for (int i = 0; i < frames.size(); i++) {
            FrameData frame = frames.get(i);
            gps.put(gpsJson(i, frame.record));
        }

        for (int i = 0; i < frames.size() - 1; i++) {
            imu.put(imuJson(i, frames.get(i), i + 1, frames.get(i + 1)));
        }

        String cfg = (nativeConfigJson != null && !nativeConfigJson.isEmpty()) ? nativeConfigJson : "{}";
        String resultJson = OpenCvFrontendLib.runFullPipelineSync(
                framesJson(frames).toString(),
                intrinsicsJson(intrinsics).toString(),
                plyFile.getParent(), // Output directory for transforms.json and sparse.ply
                cfg,
                gps.toString(),
                imu.toString()
        );

        JSONObject finalResult = new JSONObject(resultJson);
        if (finalResult.has("error")) {
            throw new IllegalStateException("Native SfM pipeline failed: " + finalResult.optString("error"));
        }

        int pointIndex = finalResult.optInt("points_count", 0);
        int matchCount = finalResult.optInt("total_matches", 0);

        return new Result(plyFile, resultFile, pointIndex, matchCount);
    }

    private static JSONArray framesJson(List<FrameData> frames) throws Exception {
        JSONArray framesJson = new JSONArray();
        for (FrameData frame : frames) {
            JSONObject frameJson = new JSONObject();
            frameJson.put("frame_idx", frame.frameListIndex);
            frameJson.put("image_path", frame.frameFile.getAbsolutePath());
            framesJson.put(frameJson);
        }
        return framesJson;
    }

    private static JSONObject intrinsicsJson(VideoIntrinsics intrinsics) throws Exception {
        JSONObject intrinsicsJson = new JSONObject();
        intrinsicsJson.put("fx", intrinsics.fx);
        intrinsicsJson.put("fy", intrinsics.fy);
        intrinsicsJson.put("cx", intrinsics.cx);
        intrinsicsJson.put("cy", intrinsics.cy);
        return intrinsicsJson;
    }


    private static FrameLoadSummary loadFramesForSequence(PoseStampSequence sequence, File framesDir) {
        List<FrameData> frames = new ArrayList<>();
        List<String> missing = new ArrayList<>();
        for (PoseStamp record : sequence.getRecords()) {
            File frameFile = new File(framesDir, String.format("frame_%03d.jpg", record.getFrameIndex()));
            if (!frameFile.exists()) {
                if (missing.size() < 12) {
                    missing.add(frameFile.getName());
                }
                continue;
            }
            frames.add(new FrameData(frames.size(), record, frameFile, buildCameraPose(record)));
        }
        return new FrameLoadSummary(frames, sequence.getRecords().size(), missing);
    }



    private static JSONObject gpsJson(int frameListIndex, PoseStamp record) throws Exception {
        JSONObject gps = new JSONObject();
        gps.put("frame_idx", frameListIndex);
        gps.put("enu_position", arrayOf(record.getEnuE(), record.getEnuN(), record.getEnuU()));
        gps.put("weight", 0.2);
        return gps;
    }

    private static JSONObject imuJson(int frameAIndex, FrameData a, int frameBIndex, FrameData b) throws Exception {
        double[][] delta = matMul(transpose(a.cameraPose.rotationWorldToCamera), b.cameraPose.rotationWorldToCamera);
        JSONObject imu = new JSONObject();
        imu.put("frame_a", frameAIndex);
        imu.put("frame_b", frameBIndex);
        imu.put("delta_rotation", matrixToJson(delta));
        imu.put("weight", 0.05);
        return imu;
    }


    private static VideoIntrinsics estimateIntrinsics(File videoFile) {
        MediaMetadataRetriever retriever = new MediaMetadataRetriever();
        try {
            retriever.setDataSource(videoFile.getAbsolutePath());
            int width = parseIntOrDefault(
                    retriever.extractMetadata(MediaMetadataRetriever.METADATA_KEY_VIDEO_WIDTH),
                    1920
            );
            int height = parseIntOrDefault(
                    retriever.extractMetadata(MediaMetadataRetriever.METADATA_KEY_VIDEO_HEIGHT),
                    1080
            );
            double focal = Math.max(width, height);
            return new VideoIntrinsics(focal, focal, width * 0.5, height * 0.5);
        } catch (Exception e) {
            Log.w(TAG, "Falling back to default intrinsics", e);
            return new VideoIntrinsics(1920.0, 1920.0, 960.0, 540.0);
        } finally {
            try {
                retriever.release();
            } catch (Exception ignored) {
            }
        }
    }

    private static int parseIntOrDefault(String value, int fallback) {
        if (value == null) {
            return fallback;
        }
        try {
            return Integer.parseInt(value);
        } catch (NumberFormatException e) {
            return fallback;
        }
    }

    private static JSONArray arrayOf(double... values) {
        JSONArray array = new JSONArray();
        for (double value : values) {
            try {
                array.put(value);
            } catch (Exception e) {
                throw new RuntimeException("Failed to encode numeric JSON array", e);
            }
        }
        return array;
    }


    private static JSONArray matrixToJson(double[][] matrix) {
        JSONArray rows = new JSONArray();
        for (double[] row : matrix) {
            rows.put(arrayOf(row[0], row[1], row[2]));
        }
        return rows;
    }

    private static CameraPose buildCameraPose(PoseStamp record) {
        double headingRad = Math.toRadians(record.getHeadingDeg());
        double pitchRad = Math.toRadians(record.getGimbalPitch());

        double[] forward = normalize(new double[]{
                Math.sin(headingRad) * Math.cos(pitchRad),
                Math.cos(headingRad) * Math.cos(pitchRad),
                Math.sin(pitchRad)
        });

        double[] referenceUp = Math.abs(forward[2]) > 0.95
                ? new double[]{0.0, 1.0, 0.0}
                : new double[]{0.0, 0.0, 1.0};
        double[] right = normalize(cross(forward, referenceUp));
        double[] down = normalize(cross(forward, right));

        double[][] rotation = new double[][]{
                {right[0], right[1], right[2]},
                {down[0], down[1], down[2]},
                {forward[0], forward[1], forward[2]}
        };
        double[] cameraCenter = new double[]{
                record.getEnuE(),
                record.getEnuN(),
                record.getEnuU()
        };
        double[] translation = scale(matVecMul(rotation, cameraCenter), -1.0);
        double[] axisAngle = rotationMatrixToAxisAngle(rotation);

        return new CameraPose(rotation, translation, axisAngle, cameraCenter);
    }

    private static double[] rotationMatrixToAxisAngle(double[][] rotation) {
        double trace = rotation[0][0] + rotation[1][1] + rotation[2][2];
        double cosTheta = clamp((trace - 1.0) * 0.5, -1.0, 1.0);
        double angle = Math.acos(cosTheta);
        if (angle < 1e-8) {
            return new double[]{0.0, 0.0, 0.0};
        }

        if (Math.PI - angle < 1e-5) {
            double xx = Math.sqrt(Math.max(0.0, (rotation[0][0] + 1.0) * 0.5));
            double yy = Math.sqrt(Math.max(0.0, (rotation[1][1] + 1.0) * 0.5));
            double zz = Math.sqrt(Math.max(0.0, (rotation[2][2] + 1.0) * 0.5));
            double x = copySign(xx, rotation[2][1] - rotation[1][2]);
            double y = copySign(yy, rotation[0][2] - rotation[2][0]);
            double z = copySign(zz, rotation[1][0] - rotation[0][1]);
            double[] axis = normalize(new double[]{x, y, z});
            return scale(axis, angle);
        }

        double denom = 2.0 * Math.sin(angle);
        double x = (rotation[2][1] - rotation[1][2]) / denom;
        double y = (rotation[0][2] - rotation[2][0]) / denom;
        double z = (rotation[1][0] - rotation[0][1]) / denom;
        return scale(normalize(new double[]{x, y, z}), angle);
    }

    private static double[] normalize(double[] v) {
        double n = Math.sqrt(dot(v, v));
        if (n <= 1e-9) {
            return new double[]{0.0, 0.0, 1.0};
        }
        return new double[]{v[0] / n, v[1] / n, v[2] / n};
    }

    private static double dot(double[] a, double[] b) {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    }

    private static double norm(double[] v) {
        return Math.sqrt(dot(v, v));
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private static double copySign(double magnitude, double signCarrier) {
        return signCarrier < 0.0 ? -Math.abs(magnitude) : Math.abs(magnitude);
    }

    private static double[] add(double[] a, double[] b) {
        return new double[]{a[0] + b[0], a[1] + b[1], a[2] + b[2]};
    }

    private static double[] sub(double[] a, double[] b) {
        return new double[]{a[0] - b[0], a[1] - b[1], a[2] - b[2]};
    }

    private static double[] scale(double[] v, double s) {
        return new double[]{v[0] * s, v[1] * s, v[2] * s};
    }

    private static double[] cross(double[] a, double[] b) {
        return new double[]{
                a[1] * b[2] - a[2] * b[1],
                a[2] * b[0] - a[0] * b[2],
                a[0] * b[1] - a[1] * b[0]
        };
    }

    private static double[] matVecMul(double[][] matrix, double[] vector) {
        return new double[]{
                dot(matrix[0], vector),
                dot(matrix[1], vector),
                dot(matrix[2], vector)
        };
    }

    private static double[][] transpose(double[][] matrix) {
        return new double[][]{
                {matrix[0][0], matrix[1][0], matrix[2][0]},
                {matrix[0][1], matrix[1][1], matrix[2][1]},
                {matrix[0][2], matrix[1][2], matrix[2][2]}
        };
    }

    private static double[][] matMul(double[][] a, double[][] b) {
        double[][] out = new double[3][3];
        for (int row = 0; row < 3; row++) {
            for (int col = 0; col < 3; col++) {
                out[row][col] =
                        a[row][0] * b[0][col]
                                + a[row][1] * b[1][col]
                                + a[row][2] * b[2][col];
            }
        }
        return out;
    }


    public static final class Result {
        public final File plyFile;
        public final File resultFile;
        public final int pointCount;
        public final int matchCount;

        Result(File plyFile, File resultFile, int pointCount, int matchCount) {
            this.plyFile = plyFile;
            this.resultFile = resultFile;
            this.pointCount = pointCount;
            this.matchCount = matchCount;
        }
    }


    private static final class FrameData {
        final int frameListIndex;
        final PoseStamp record;
        final File frameFile;
        final CameraPose cameraPose;

        FrameData(
                int frameListIndex,
                PoseStamp record,
                File frameFile,
                CameraPose cameraPose
        ) {
            this.frameListIndex = frameListIndex;
            this.record = record;
            this.frameFile = frameFile;
            this.cameraPose = cameraPose;
        }
    }

    private static final class FrameLoadSummary {
        final List<FrameData> frames;
        final int expectedCount;
        final List<String> missingFrameNames;

        FrameLoadSummary(List<FrameData> frames, int expectedCount, List<String> missingFrameNames) {
            this.frames = frames;
            this.expectedCount = expectedCount;
            this.missingFrameNames = missingFrameNames;
        }
    }

    private static final class CameraPose {
        final double[][] rotationWorldToCamera;
        final double[] translation;
        final double[] axisAngle;
        final double[] cameraCenter;

        CameraPose(double[][] rotationWorldToCamera, double[] translation, double[] axisAngle, double[] cameraCenter) {
            this.rotationWorldToCamera = rotationWorldToCamera;
            this.translation = translation;
            this.axisAngle = axisAngle;
            this.cameraCenter = cameraCenter;
        }
    }


    private static final class VideoIntrinsics {
        final double fx;
        final double fy;
        final double cx;
        final double cy;

        VideoIntrinsics(double fx, double fy, double cx, double cy) {
            this.fx = fx;
            this.fy = fy;
            this.cx = cx;
            this.cy = cy;
        }
    }

    private static int countExtractedFrames(File dir) {
        if (dir == null || !dir.exists() || !dir.isDirectory()) {
            return 0;
        }
        File[] files = dir.listFiles((ignored, name) -> name.startsWith("frame_") && name.endsWith(".jpg"));
        return files != null ? files.length : 0;
    }
}
