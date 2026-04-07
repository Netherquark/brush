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
}

impl FrameFeatures {
    pub fn from_extraction(frame_id: usize, extraction: FeatureExtractionResult) -> Self {
        Self {
            frame_id,
            keypoints: extraction.keypoints,
            descriptors: extraction.descriptors,
        }
    }
}

#[derive(Debug, Clone)]
pub struct MatchConfig {
    pub max_hamming_distance: f32,
    pub max_matches: usize,
}

impl Default for MatchConfig {
    fn default() -> Self {
        Self {
            max_hamming_distance: 64.0,
            max_matches: 512,
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

    let matcher = BFMatcher::new(NORM_HAMMING, true)?;
    let mut raw_matches = Vector::<DMatch>::new();
    matcher.train_match_def(&frame_a.descriptors, &frame_b.descriptors, &mut raw_matches)?;

    let mut matches: Vec<_> = raw_matches
        .iter()
        .filter(|m| m.distance <= config.max_hamming_distance)
        .filter_map(|m| {
            let query_index = usize::try_from(m.query_idx).ok()?;
            let train_index = usize::try_from(m.train_idx).ok()?;
            let query_point = *frame_a.keypoints.get(query_index)?;
            let train_point = *frame_b.keypoints.get(train_index)?;
            Some(FeatureMatch {
                query_index,
                train_index,
                distance: m.distance,
                query_point,
                train_point,
            })
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
