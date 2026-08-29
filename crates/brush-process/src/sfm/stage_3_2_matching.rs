use opencv::{
    core::{DMatch, Point2f, Vector, NORM_HAMMING},
    features2d::BFMatcher,
    prelude::*,
};

use crate::sfm::stage_3_1_feature_extraction::FeatureExtractionResult;

#[derive(Debug, Clone)]
pub struct FrameFeatures {
    pub frame_id: usize,
    pub keypoints: Vec<Point2f>,
    pub descriptors: Mat,
    pub enu_position: Option<[f64; 3]>, // ENU position for telemetry-window matching
}

impl FrameFeatures {
    pub fn from_extraction(frame_id: usize, extraction: FeatureExtractionResult) -> Self {
        Self {
            frame_id,
            keypoints: extraction.keypoints,
            descriptors: extraction.descriptors,
            enu_position: None,
        }
    }

    pub fn with_telemetry(mut self, enu_position: [f64; 3]) -> Self {
        self.enu_position = Some(enu_position);
        self
    }
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct MatchConfig {
    pub max_hamming_distance: f32,
    pub max_matches: usize,
    pub telemetry_baseline_threshold: f64, // meters
}

impl Default for MatchConfig {
    fn default() -> Self {
        Self {
            max_hamming_distance: 64.0,
            max_matches: 512,
            telemetry_baseline_threshold: 50.0, // 50 meter baseline threshold
        }
    }
}

#[derive(Debug, Clone)]
pub struct FeatureMatch {
    pub query_index: usize,
    pub train_index: usize,
    pub distance: f32,
    pub query_point: Point2f,
    pub train_point: Point2f,
}

#[derive(Debug, Clone)]
pub struct MatchingResult {
    pub frame_a: usize,
    pub frame_b: usize,
    pub matches: Vec<FeatureMatch>,
}

pub fn match_feature_sets(
    frame_a: &FrameFeatures,
    frame_b: &FrameFeatures,
    config: &MatchConfig,
) -> opencv::Result<MatchingResult> {
    if frame_a.descriptors.empty() || frame_b.descriptors.empty() {
        return Ok(MatchingResult {
            frame_a: frame_a.frame_id,
            frame_b: frame_b.frame_id,
            matches: Vec::new(),
        });
    }

    // Telemetry-window matching: only match if baseline threshold satisfied (distance within threshold)
    if let (Some(pos_a), Some(pos_b)) = (frame_a.enu_position, frame_b.enu_position) {
        let dx = pos_b[0] - pos_a[0];
        let dy = pos_b[1] - pos_a[1];
        let dz = pos_b[2] - pos_a[2];
        let distance = (dx * dx + dy * dy + dz * dz).sqrt();

        if distance > config.telemetry_baseline_threshold {
            return Ok(MatchingResult {
                frame_a: frame_a.frame_id,
                frame_b: frame_b.frame_id,
                matches: Vec::new(),
            });
        }
    }

    let matcher = BFMatcher::new(NORM_HAMMING, false)?;
    let mut knn_matches = Vector::<Vector<DMatch>>::new();
    matcher.knn_train_match_def(&frame_a.descriptors, &frame_b.descriptors, &mut knn_matches, 2)?;

    let mut matches: Vec<_> = knn_matches
        .iter()
        .filter_map(|m_vec| {
            if m_vec.len() < 2 {
                return None;
            }
            let m1 = m_vec.get(0).ok()?;
            let m2 = m_vec.get(1).ok()?;

            // Lowe's ratio test (e.g. 0.75) and absolute max distance
            if m1.distance <= 0.75 * m2.distance && m1.distance <= config.max_hamming_distance {
                let query_index = usize::try_from(m1.query_idx).ok()?;
                let train_index = usize::try_from(m1.train_idx).ok()?;
                let query_point = *frame_a.keypoints.get(query_index)?;
                let train_point = *frame_b.keypoints.get(train_index)?;
                Some(FeatureMatch {
                    query_index,
                    train_index,
                    distance: m1.distance,
                    query_point,
                    train_point,
                })
            } else {
                None
            }
        })
        .collect();

    matches.sort_by(|left, right| left.distance.total_cmp(&right.distance));
    matches.truncate(config.max_matches);

    Ok(MatchingResult {
        frame_a: frame_a.frame_id,
        frame_b: frame_b.frame_id,
        matches,
    })
}
