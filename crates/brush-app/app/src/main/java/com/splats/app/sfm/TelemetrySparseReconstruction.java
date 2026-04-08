package com.splats.app.sfm;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.media.MediaMetadataRetriever;
import android.os.Environment;
import android.util.Log;

import com.splats.app.telemetry.PoseStamp;
import com.splats.app.telemetry.PoseStampSequence;

import org.json.JSONArray;
import org.json.JSONObject;

import java.io.File;
import java.io.FileOutputStream;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public final class TelemetrySparseReconstruction {

    private static final String TAG = "TelemetrySparseRecon";
    private static final int MAX_FEATURES_PER_FRAME = 80;
    private static final int GRID_SIZE = 24;
    private static final int PATCH_RADIUS = 3;
    private static final int SEARCH_RADIUS = 18;
    private static final double MAX_PATCH_SCORE = 0.08;
    private static final double MAX_TRACK_REPROJECTION_ERROR_PX = 14.0;
    private static final double MAX_MEAN_REPROJECTION_ERROR_PX = 8.0;
    private static final double MIN_POINT_DEPTH_M = 0.5;
    private static final double MAX_POINT_DEPTH_M = 5000.0;

    private TelemetrySparseReconstruction() {}

    public static Result run(
            Context context,
            PoseStampSequence sequence,
            File plyFile,
            File resultFile
    ) throws Exception {
        if (sequence.getRecords().size() < 2) {
            throw new IllegalStateException("Need at least 2 pose records for sparse reconstruction");
        }

        File framesDir = context.getExternalFilesDir(Environment.DIRECTORY_PICTURES);
        if (framesDir == null || !framesDir.exists()) {
            throw new IllegalStateException("Extracted frames directory is missing");
        }

        VideoIntrinsics intrinsics = estimateIntrinsics(sequence.getVideoPath());
        List<FrameData> frames = loadFramesForSequence(sequence, framesDir);
        if (frames.size() < 2) {
            throw new IllegalStateException("Could not load enough extracted frames");
        }

        JSONArray poses = new JSONArray();
        JSONArray points = new JSONArray();
        JSONArray observations = new JSONArray();
        JSONArray gps = new JSONArray();
        JSONArray imu = new JSONArray();

        for (int i = 0; i < frames.size(); i++) {
            FrameData frame = frames.get(i);
            poses.put(poseJson(frame));
            gps.put(gpsJson(i, frame.record));
        }

        for (int i = 0; i < frames.size() - 1; i++) {
            imu.put(imuJson(i, frames.get(i), i + 1, frames.get(i + 1)));
        }

        SparseFrontendOutput frontendOutput = null;
        try {
            frontendOutput = runNativeFrontend(frames, intrinsics, gps, imu);
            Log.i(
                    TAG,
                    frontendOutput.backend + " frontend produced " + frontendOutput.pointCount
                            + " points and " + frontendOutput.matchCount + " observations"
            );
        } catch (Exception e) {
            Log.w(TAG, "Native OpenCV frontend failed, falling back to legacy Java sparse frontend", e);
        }

        if (frontendOutput == null || frontendOutput.pointCount < 8) {
            if (frontendOutput != null) {
                Log.w(
                        TAG,
                        "Native OpenCV frontend produced only " + frontendOutput.pointCount
                                + " points, falling back to legacy Java sparse frontend"
                );
            }
            frontendOutput = runLegacyFrontend(frames, intrinsics);
            Log.i(
                    TAG,
                    frontendOutput.backend + " frontend produced " + frontendOutput.pointCount
                            + " points and " + frontendOutput.matchCount + " observations"
            );
        }

        points = frontendOutput.points;
        observations = frontendOutput.observations;
        int pointIndex = frontendOutput.pointCount;
        int matchCount = frontendOutput.matchCount;

        if (pointIndex < 8) {
            writeFallbackPly(plyFile, sequence.getRecords());
            writeFallbackResult(
                    resultFile,
                    "fallback",
                    "Not enough triangulated matches for sparse reconstruction",
                    pointIndex,
                    matchCount,
                    sequence.getRecords().size()
            );
            return new Result(plyFile, resultFile, pointIndex, matchCount);
        }

        JSONObject intrinsicsJson = new JSONObject();
        intrinsicsJson.put("fx", intrinsics.fx);
        intrinsicsJson.put("fy", intrinsics.fy);
        intrinsicsJson.put("cx", intrinsics.cx);
        intrinsicsJson.put("cy", intrinsics.cy);

        JSONObject configJson = new JSONObject();
        configJson.put("window_size", Math.max(2, Math.min(8, frames.size())));
        configJson.put("min_observations", 8);
        configJson.put("freeze_first_frame", true);
        configJson.put("export_ply_path", plyFile.getAbsolutePath());

        String resultJson;
        try {
            resultJson = BundleAdjustmentLib.runBASync(
                    poses.toString(),
                    points.toString(),
                    observations.toString(),
                    intrinsicsJson.toString(),
                    gps.toString(),
                    imu.toString(),
                    configJson.toString()
            );
        } catch (Exception e) {
            Log.w(TAG, "Bundle adjustment failed, writing fallback sparse PLY", e);
            writeFallbackPly(plyFile, sequence.getRecords());
            writeFallbackResult(
                    resultFile,
                    "fallback",
                    "Bundle adjustment failed: " + e.getMessage(),
                    pointIndex,
                    matchCount,
                    sequence.getRecords().size()
            );
            return new Result(plyFile, resultFile, pointIndex, matchCount);
        }

        if (resultFile.getParentFile() != null && !resultFile.getParentFile().exists()) {
            resultFile.getParentFile().mkdirs();
        }
        try (FileOutputStream out = new FileOutputStream(resultFile)) {
            out.write(resultJson.getBytes());
        }

        if (!plyFile.exists() || plyFile.length() == 0L) {
            Log.w(TAG, "BA completed without producing a non-empty PLY, writing telemetry fallback PLY");
            writeFallbackPly(plyFile, sequence.getRecords());
        }

        return new Result(plyFile, resultFile, pointIndex, matchCount);
    }

    private static SparseFrontendOutput runNativeFrontend(
            List<FrameData> frames,
            VideoIntrinsics intrinsics,
            JSONArray gps,
            JSONArray imu
    ) throws Exception {
        JSONArray framesJson = new JSONArray();
        for (FrameData frame : frames) {
            JSONObject frameJson = new JSONObject();
            frameJson.put("frame_idx", frame.frameListIndex);
            frameJson.put("image_path", frame.frameFile.getAbsolutePath());
            framesJson.put(frameJson);
        }

        JSONObject intrinsicsJson = new JSONObject();
        intrinsicsJson.put("fx", intrinsics.fx);
        intrinsicsJson.put("fy", intrinsics.fy);
        intrinsicsJson.put("cx", intrinsics.cx);
        intrinsicsJson.put("cy", intrinsics.cy);

        String resultJson = OpenCvFrontendLib.runOpenCvFrontendSync(
                framesJson.toString(),
                intrinsicsJson.toString(),
                "{}",
                gps.toString(),
                imu.toString()
        );

        JSONObject result = new JSONObject(resultJson);
        if (result.has("error")) {
            throw new IllegalStateException(result.optString("error", "unknown native frontend error"));
        }

        JSONArray points = result.optJSONArray("points");
        JSONArray observations = result.optJSONArray("observations");
        if (points == null || observations == null) {
            throw new IllegalStateException("Native frontend returned no points/observations payload");
        }

        return new SparseFrontendOutput(
                "native-opencv",
                points,
                observations,
                points.length(),
                observations.length()
        );
    }

    private static SparseFrontendOutput runLegacyFrontend(
            List<FrameData> frames,
            VideoIntrinsics intrinsics
    ) throws Exception {
        JSONArray points = new JSONArray();
        JSONArray observations = new JSONArray();

        List<List<FeatureMatch>> pairMatches = new ArrayList<>();
        for (int i = 0; i < frames.size() - 1; i++) {
            FrameData a = frames.get(i);
            FrameData b = frames.get(i + 1);
            pairMatches.add(matchFrames(a, b));
        }

        List<FeatureTrack> tracks = buildTracks(frames, pairMatches);
        int pointIndex = 0;
        int matchCount = 0;
        for (FeatureTrack track : tracks) {
            if (track.observations.size() < 2) {
                continue;
            }

            TrackObservation first = track.observations.get(0);
            TrackObservation last = track.observations.get(track.observations.size() - 1);
            double[] point = triangulate(
                    frames.get(first.frameListIndex),
                    frames.get(last.frameListIndex),
                    intrinsics,
                    first.xNative,
                    first.yNative,
                    last.xNative,
                    last.yNative
            );
            if (point == null) {
                continue;
            }
            TrackCandidate candidate = evaluateTrackCandidate(track, frames, intrinsics, point);
            if (candidate == null) {
                continue;
            }

            points.put(arrayOf(candidate.point[0], candidate.point[1], candidate.point[2]));
            for (TrackObservation observation : candidate.inlierObservations) {
                observations.put(observationJson(
                        observation.frameListIndex,
                        pointIndex,
                        observation.xNative,
                        observation.yNative
                ));
                matchCount += 1;
            }
            pointIndex += 1;
        }

        return new SparseFrontendOutput("legacy-java", points, observations, pointIndex, matchCount);
    }

    private static void writeFallbackPly(File plyFile, List<PoseStamp> records) throws Exception {
        if (plyFile.getParentFile() != null && !plyFile.getParentFile().exists()) {
            plyFile.getParentFile().mkdirs();
        }

        StringBuilder builder = new StringBuilder();
        builder.append("ply\n");
        builder.append("format ascii 1.0\n");
        builder.append("comment generated from telemetry fallback\n");
        builder.append("element vertex ").append(records.size()).append('\n');
        builder.append("property float x\n");
        builder.append("property float y\n");
        builder.append("property float z\n");
        builder.append("property uchar red\n");
        builder.append("property uchar green\n");
        builder.append("property uchar blue\n");
        builder.append("end_header\n");

        for (PoseStamp record : records) {
            builder.append(record.getEnuE()).append(' ')
                    .append(record.getEnuN()).append(' ')
                    .append(record.getEnuU()).append(' ')
                    .append("255 196 64\n");
        }

        try (FileOutputStream out = new FileOutputStream(plyFile)) {
            out.write(builder.toString().getBytes());
        }
    }

    private static void writeFallbackResult(
            File resultFile,
            String status,
            String message,
            int pointCount,
            int matchCount,
            int poseCount
    ) throws Exception {
        if (resultFile.getParentFile() != null && !resultFile.getParentFile().exists()) {
            resultFile.getParentFile().mkdirs();
        }
        JSONObject result = new JSONObject();
        result.put("status", status);
        result.put("message", message);
        result.put("point_count", pointCount);
        result.put("match_count", matchCount);
        result.put("pose_count", poseCount);

        try (FileOutputStream out = new FileOutputStream(resultFile)) {
            out.write(result.toString(2).getBytes());
        }
    }

    private static List<FrameData> loadFramesForSequence(PoseStampSequence sequence, File framesDir) {
        List<FrameData> frames = new ArrayList<>();
        for (PoseStamp record : sequence.getRecords()) {
            File frameFile = new File(framesDir, String.format("frame_%03d.jpg", record.getFrameIndex()));
            if (!frameFile.exists()) {
                continue;
            }
            GrayImage image = GrayImage.load(frameFile);
            if (image == null || image.width < PATCH_RADIUS * 4 || image.height < PATCH_RADIUS * 4) {
                continue;
            }
            List<FeaturePoint> features = detectFeatures(image);
            if (features.size() < 12) {
                continue;
            }
            frames.add(new FrameData(frames.size(), record, frameFile, image, features, buildCameraPose(record)));
        }
        return frames;
    }

    private static List<FeaturePoint> detectFeatures(GrayImage image) {
        List<ScoredFeature> scored = new ArrayList<>();
        int border = PATCH_RADIUS + 2;
        for (int y = border; y < image.height - border; y += 2) {
            for (int x = border; x < image.width - border; x += 2) {
                double gx = image.get(x + 1, y) - image.get(x - 1, y);
                double gy = image.get(x, y + 1) - image.get(x, y - 1);
                double score = gx * gx + gy * gy;
                if (score > 0.02) {
                    scored.add(new ScoredFeature(x, y, score));
                }
            }
        }
        scored.sort(Comparator.comparingDouble((ScoredFeature f) -> f.score).reversed());

        List<FeaturePoint> selected = new ArrayList<>();
        Map<Long, Boolean> occupied = new HashMap<>();
        for (ScoredFeature candidate : scored) {
            int cellX = candidate.x / GRID_SIZE;
            int cellY = candidate.y / GRID_SIZE;
            long cellKey = (((long) cellX) << 32) | (cellY & 0xffffffffL);
            if (occupied.containsKey(cellKey)) {
                continue;
            }
            occupied.put(cellKey, Boolean.TRUE);
            selected.add(new FeaturePoint(candidate.x, candidate.y));
            if (selected.size() >= MAX_FEATURES_PER_FRAME) {
                break;
            }
        }
        return selected;
    }

    private static List<FeatureMatch> matchFrames(FrameData a, FrameData b) {
        List<FeatureMatch> forwardMatches = new ArrayList<>();
        for (FeaturePoint feature : a.features) {
            FeatureMatch best = findBestMatch(a.image, b.image, feature);
            if (best != null) {
                forwardMatches.add(best);
            }
        }

        List<FeatureMatch> verified = new ArrayList<>();
        for (FeatureMatch match : forwardMatches) {
            FeatureMatch reverse = findBestMatch(
                    b.image,
                    a.image,
                    new FeaturePoint(match.bx, match.by)
            );
            if (reverse == null) {
                continue;
            }
            if (Math.abs(reverse.bx - match.ax) <= 2 && Math.abs(reverse.by - match.ay) <= 2) {
                verified.add(match);
            }
        }
        return verified;
    }

    private static FeatureMatch findBestMatch(GrayImage source, GrayImage target, FeaturePoint feature) {
        int minX = Math.max(PATCH_RADIUS, feature.x - SEARCH_RADIUS);
        int maxX = Math.min(target.width - PATCH_RADIUS - 1, feature.x + SEARCH_RADIUS);
        int minY = Math.max(PATCH_RADIUS, feature.y - SEARCH_RADIUS);
        int maxY = Math.min(target.height - PATCH_RADIUS - 1, feature.y + SEARCH_RADIUS);

        double bestScore = Double.MAX_VALUE;
        int bestX = -1;
        int bestY = -1;
        for (int y = minY; y <= maxY; y++) {
            for (int x = minX; x <= maxX; x++) {
                double score = patchScore(source, target, feature.x, feature.y, x, y);
                if (score < bestScore) {
                    bestScore = score;
                    bestX = x;
                    bestY = y;
                }
            }
        }
        if (bestX < 0 || bestScore > MAX_PATCH_SCORE) {
            return null;
        }

        return new FeatureMatch(
                feature.x,
                feature.y,
                bestX,
                bestY,
                source.toNativeX(feature.x),
                source.toNativeY(feature.y),
                target.toNativeX(bestX),
                target.toNativeY(bestY)
        );
    }

    private static List<FeatureTrack> buildTracks(List<FrameData> frames, List<List<FeatureMatch>> pairMatches) {
        List<FeatureTrack> activeTracks = new ArrayList<>();
        List<FeatureTrack> completedTracks = new ArrayList<>();

        for (int pairIndex = 0; pairIndex < pairMatches.size(); pairIndex++) {
            int frameA = pairIndex;
            int frameB = pairIndex + 1;
            List<FeatureMatch> matches = pairMatches.get(pairIndex);
            Map<FeatureTrack, Boolean> extendedThisRound = new HashMap<>();

            for (FeatureMatch match : matches) {
                FeatureTrack track = findContinuingTrack(activeTracks, frameA, match.axNative, match.ayNative);
                if (track == null) {
                    track = new FeatureTrack();
                    track.observations.add(new TrackObservation(frameA, match.axNative, match.ayNative));
                    activeTracks.add(track);
                }

                TrackObservation lastObs = track.observations.get(track.observations.size() - 1);
                if (lastObs.frameListIndex != frameB) {
                    track.observations.add(new TrackObservation(frameB, match.bxNative, match.byNative));
                } else {
                    lastObs.xNative = 0.5 * (lastObs.xNative + match.bxNative);
                    lastObs.yNative = 0.5 * (lastObs.yNative + match.byNative);
                }
                extendedThisRound.put(track, Boolean.TRUE);
            }

            List<FeatureTrack> stillActive = new ArrayList<>();
            for (FeatureTrack track : activeTracks) {
                TrackObservation last = track.observations.get(track.observations.size() - 1);
                if (last.frameListIndex == frameB || last.frameListIndex == frameA) {
                    stillActive.add(track);
                } else {
                    completedTracks.add(track);
                }
            }
            activeTracks = stillActive;
        }

        completedTracks.addAll(activeTracks);

        List<FeatureTrack> filtered = new ArrayList<>();
        for (FeatureTrack track : completedTracks) {
            dedupeTrack(track);
            if (track.observations.size() >= 3) {
                filtered.add(track);
            }
        }

        if (!filtered.isEmpty()) {
            return filtered;
        }

        for (FeatureTrack track : completedTracks) {
            if (track.observations.size() >= 2) {
                filtered.add(track);
            }
        }
        return filtered;
    }

    private static FeatureTrack findContinuingTrack(
            List<FeatureTrack> tracks,
            int frameListIndex,
            double xNative,
            double yNative
    ) {
        FeatureTrack best = null;
        double bestDistance = Double.MAX_VALUE;
        for (FeatureTrack track : tracks) {
            TrackObservation last = track.observations.get(track.observations.size() - 1);
            if (last.frameListIndex != frameListIndex) {
                continue;
            }
            double distance = Math.hypot(last.xNative - xNative, last.yNative - yNative);
            if (distance < 6.0 && distance < bestDistance) {
                best = track;
                bestDistance = distance;
            }
        }
        return best;
    }

    private static void dedupeTrack(FeatureTrack track) {
        List<TrackObservation> deduped = new ArrayList<>();
        for (TrackObservation observation : track.observations) {
            if (!deduped.isEmpty()) {
                TrackObservation previous = deduped.get(deduped.size() - 1);
                if (previous.frameListIndex == observation.frameListIndex) {
                    previous.xNative = 0.5 * (previous.xNative + observation.xNative);
                    previous.yNative = 0.5 * (previous.yNative + observation.yNative);
                    continue;
                }
            }
            deduped.add(observation);
        }
        track.observations = deduped;
    }

    private static double patchScore(GrayImage a, GrayImage b, int ax, int ay, int bx, int by) {
        double score = 0.0;
        int count = 0;
        for (int dy = -PATCH_RADIUS; dy <= PATCH_RADIUS; dy++) {
            for (int dx = -PATCH_RADIUS; dx <= PATCH_RADIUS; dx++) {
                double diff = a.get(ax + dx, ay + dy) - b.get(bx + dx, by + dy);
                score += diff * diff;
                count += 1;
            }
        }
        return score / Math.max(1, count);
    }

    private static double[] triangulate(
            FrameData a,
            FrameData b,
            VideoIntrinsics k,
            double ax,
            double ay,
            double bx,
            double by
    ) {
        double[] c1 = a.cameraPose.cameraCenter;
        double[] c2 = b.cameraPose.cameraCenter;

        double[] d1Cam = normalize(new double[]{
                (ax - k.cx) / k.fx,
                (ay - k.cy) / k.fy,
                1.0
        });
        double[] d2Cam = normalize(new double[]{
                (bx - k.cx) / k.fx,
                (by - k.cy) / k.fy,
                1.0
        });
        double[] d1 = normalize(matTransposeMul(a.cameraPose.rotationWorldToCamera, d1Cam));
        double[] d2 = normalize(matTransposeMul(b.cameraPose.rotationWorldToCamera, d2Cam));

        double baseline = norm(sub(c2, c1));
        double rayDot = dot(d1, d2);
        if (baseline < 0.25 || Math.abs(rayDot) > 0.995) {
            return null;
        }

        double[] r = sub(c2, c1);
        double a11 = dot(d1, d1);
        double a12 = -dot(d1, d2);
        double a22 = dot(d2, d2);
        double b1 = dot(d1, r);
        double b2 = -dot(d2, r);
        double det = a11 * a22 - a12 * a12;
        if (Math.abs(det) < 1e-9) {
            return null;
        }

        double s = (b1 * a22 - a12 * b2) / det;
        double t = (a11 * b2 - a12 * b1) / det;
        if (s <= 0.1 || t <= 0.1) {
            return null;
        }

        double[] p1 = add(c1, scale(d1, s));
        double[] p2 = add(c2, scale(d2, t));
        double[] point = scale(add(p1, p2), 0.5);
        double reprojGap = norm(sub(p1, p2));
        if (reprojGap > Math.max(2.0, baseline * 0.5)) {
            return null;
        }
        return point;
    }

    private static TrackCandidate evaluateTrackCandidate(
            FeatureTrack track,
            List<FrameData> frames,
            VideoIntrinsics intrinsics,
            double[] point
    ) {
        List<TrackObservation> inliers = new ArrayList<>();
        double errorSum = 0.0;

        for (TrackObservation observation : track.observations) {
            FrameData frame = frames.get(observation.frameListIndex);
            double depth = pointDepth(frame.cameraPose, point);
            if (depth < MIN_POINT_DEPTH_M || depth > MAX_POINT_DEPTH_M) {
                continue;
            }

            double[] projected = projectPoint(frame.cameraPose, intrinsics, point);
            if (projected == null) {
                continue;
            }

            double error = Math.hypot(projected[0] - observation.xNative, projected[1] - observation.yNative);
            if (error > MAX_TRACK_REPROJECTION_ERROR_PX) {
                continue;
            }

            inliers.add(observation);
            errorSum += error;
        }

        if (inliers.size() < 2) {
            return null;
        }

        double meanError = errorSum / inliers.size();
        if (meanError > MAX_MEAN_REPROJECTION_ERROR_PX) {
            return null;
        }

        return new TrackCandidate(point, inliers, meanError);
    }

    private static double pointDepth(CameraPose pose, double[] point) {
        double[] cameraPoint = add(matVecMul(pose.rotationWorldToCamera, point), pose.translation);
        return cameraPoint[2];
    }

    private static double[] projectPoint(CameraPose pose, VideoIntrinsics intrinsics, double[] point) {
        double[] cameraPoint = add(matVecMul(pose.rotationWorldToCamera, point), pose.translation);
        if (cameraPoint[2] <= 1e-6) {
            return null;
        }
        return new double[]{
                intrinsics.fx * cameraPoint[0] / cameraPoint[2] + intrinsics.cx,
                intrinsics.fy * cameraPoint[1] / cameraPoint[2] + intrinsics.cy
        };
    }

    private static JSONObject poseJson(FrameData frame) throws Exception {
        JSONObject pose = new JSONObject();
        pose.put("frame_id", frame.record.getFrameIndex());
        pose.put("rotation", arrayOf(
                frame.cameraPose.axisAngle[0],
                frame.cameraPose.axisAngle[1],
                frame.cameraPose.axisAngle[2]
        ));
        pose.put("translation", arrayOf(
                frame.cameraPose.translation[0],
                frame.cameraPose.translation[1],
                frame.cameraPose.translation[2]
        ));
        return pose;
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

    private static JSONObject observationJson(int frameIndex, int pointIndex, double x, double y) throws Exception {
        JSONObject obs = new JSONObject();
        obs.put("frame_idx", frameIndex);
        obs.put("point_idx", pointIndex);
        obs.put("observed", arrayOf(x, y));
        obs.put("weight", 1.0);
        return obs;
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

    private static JSONArray identityMatrix() {
        JSONArray rows = new JSONArray();
        rows.put(arrayOf(1.0, 0.0, 0.0));
        rows.put(arrayOf(0.0, 1.0, 0.0));
        rows.put(arrayOf(0.0, 0.0, 1.0));
        return rows;
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

    private static double[] matTransposeMul(double[][] matrix, double[] vector) {
        return new double[]{
                matrix[0][0] * vector[0] + matrix[1][0] * vector[1] + matrix[2][0] * vector[2],
                matrix[0][1] * vector[0] + matrix[1][1] * vector[1] + matrix[2][1] * vector[2],
                matrix[0][2] * vector[0] + matrix[1][2] * vector[1] + matrix[2][2] * vector[2]
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

    private static final class SparseFrontendOutput {
        final String backend;
        final JSONArray points;
        final JSONArray observations;
        final int pointCount;
        final int matchCount;

        SparseFrontendOutput(String backend, JSONArray points, JSONArray observations, int pointCount, int matchCount) {
            this.backend = backend;
            this.points = points;
            this.observations = observations;
            this.pointCount = pointCount;
            this.matchCount = matchCount;
        }
    }

    private static final class FrameData {
        final int frameListIndex;
        final PoseStamp record;
        final File frameFile;
        final GrayImage image;
        final List<FeaturePoint> features;
        final CameraPose cameraPose;

        FrameData(
                int frameListIndex,
                PoseStamp record,
                File frameFile,
                GrayImage image,
                List<FeaturePoint> features,
                CameraPose cameraPose
        ) {
            this.frameListIndex = frameListIndex;
            this.record = record;
            this.frameFile = frameFile;
            this.image = image;
            this.features = features;
            this.cameraPose = cameraPose;
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

    private static final class FeatureTrack {
        List<TrackObservation> observations = new ArrayList<>();
    }

    private static final class TrackObservation {
        final int frameListIndex;
        double xNative;
        double yNative;

        TrackObservation(int frameListIndex, double xNative, double yNative) {
            this.frameListIndex = frameListIndex;
            this.xNative = xNative;
            this.yNative = yNative;
        }
    }

    private static final class TrackCandidate {
        final double[] point;
        final List<TrackObservation> inlierObservations;
        final double meanReprojectionErrorPx;

        TrackCandidate(double[] point, List<TrackObservation> inlierObservations, double meanReprojectionErrorPx) {
            this.point = point;
            this.inlierObservations = inlierObservations;
            this.meanReprojectionErrorPx = meanReprojectionErrorPx;
        }
    }

    private static final class GrayImage {
        final int width;
        final int height;
        final int nativeWidth;
        final int nativeHeight;
        final float scaleX;
        final float scaleY;
        final float[] data;

        GrayImage(int width, int height, int nativeWidth, int nativeHeight, float[] data) {
            this.width = width;
            this.height = height;
            this.nativeWidth = nativeWidth;
            this.nativeHeight = nativeHeight;
            this.scaleX = nativeWidth / (float) width;
            this.scaleY = nativeHeight / (float) height;
            this.data = data;
        }

        static GrayImage load(File file) {
            BitmapFactory.Options bounds = new BitmapFactory.Options();
            bounds.inJustDecodeBounds = true;
            BitmapFactory.decodeFile(file.getAbsolutePath(), bounds);
            if (bounds.outWidth <= 0 || bounds.outHeight <= 0) {
                return null;
            }

            int sample = 1;
            int maxDim = Math.max(bounds.outWidth, bounds.outHeight);
            while (maxDim / sample > 640) {
                sample *= 2;
            }

            BitmapFactory.Options options = new BitmapFactory.Options();
            options.inSampleSize = sample;
            options.inPreferredConfig = Bitmap.Config.ARGB_8888;
            Bitmap bitmap = BitmapFactory.decodeFile(file.getAbsolutePath(), options);
            if (bitmap == null) {
                return null;
            }

            int width = bitmap.getWidth();
            int height = bitmap.getHeight();
            int[] pixels = new int[width * height];
            bitmap.getPixels(pixels, 0, width, 0, 0, width, height);
            bitmap.recycle();

            float[] data = new float[width * height];
            for (int i = 0; i < pixels.length; i++) {
                int pixel = pixels[i];
                int r = (pixel >> 16) & 0xff;
                int g = (pixel >> 8) & 0xff;
                int b = pixel & 0xff;
                data[i] = (0.299f * r + 0.587f * g + 0.114f * b) / 255.0f;
            }

            return new GrayImage(width, height, bounds.outWidth, bounds.outHeight, data);
        }

        double get(int x, int y) {
            return data[y * width + x];
        }

        double toNativeX(int x) {
            return x * scaleX;
        }

        double toNativeY(int y) {
            return y * scaleY;
        }
    }

    private static final class ScoredFeature {
        final int x;
        final int y;
        final double score;

        ScoredFeature(int x, int y, double score) {
            this.x = x;
            this.y = y;
            this.score = score;
        }
    }

    private static final class FeaturePoint {
        final int x;
        final int y;

        FeaturePoint(int x, int y) {
            this.x = x;
            this.y = y;
        }
    }

    private static final class FeatureMatch {
        final int ax;
        final int ay;
        final int bx;
        final int by;
        final double axNative;
        final double ayNative;
        final double bxNative;
        final double byNative;

        FeatureMatch(int ax, int ay, int bx, int by, double axNative, double ayNative, double bxNative, double byNative) {
            this.ax = ax;
            this.ay = ay;
            this.bx = bx;
            this.by = by;
            this.axNative = axNative;
            this.ayNative = ayNative;
            this.bxNative = bxNative;
            this.byNative = byNative;
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
}
