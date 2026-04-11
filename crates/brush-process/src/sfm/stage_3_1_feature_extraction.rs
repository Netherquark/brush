use opencv::{
    core::{Mat, Point2f, Vector},
    features2d::{ORB, ORB_ScoreType},
    imgcodecs,
    imgproc,
    prelude::*,
};

pub struct FeatureExtractionResult {
    /// Normalized (x,y) coordinates of the detected keypoints
    pub keypoints: Vec<Point2f>,
    /// Dense ORB descriptors for the keypoints
    pub descriptors: Mat,
}

pub fn extract_features(
    image_bytes: &[u8],
    n_features: i32,
) -> Result<FeatureExtractionResult, Box<dyn std::error::Error>> {
    // Decode the image from the byte buffer
    let byte_vector: Vector<u8> = Vector::from_slice(image_bytes);
    let original_image = imgcodecs::imdecode(&byte_vector, imgcodecs::IMREAD_COLOR)?;
    
    // Convert to grayscale
    let mut gray_image = Mat::default();
    imgproc::cvt_color(&original_image, &mut gray_image, imgproc::COLOR_BGR2GRAY, 0, opencv::core::AlgorithmHint::ALGO_HINT_DEFAULT)?;
    
    // ORB: n_features, scale_factor, n_levels, edge_threshold, first_level, WTA_K, score_type, patch_size, fast_threshold
    let nf = n_features.clamp(50, 10_000);
    let mut orb = ORB::create(
        nf,
        1.2_f32,
        8,
        31,
        0,
        2,
        ORB_ScoreType::HARRIS_SCORE,
        31,
        20,
    )?;
    
    let mut keypoints = Vector::<opencv::core::KeyPoint>::new();
    let mut descriptors = Mat::default();

    // The signature for detect_and_compute in opencv crate:
    // (image, mask, keypoints, descriptors, use_provided_keypoints)
    orb.detect_and_compute(&gray_image, &opencv::core::no_array(), &mut keypoints, &mut descriptors, false)?;
    
    // Convert keypoints to point2f
    let mut pts = Vec::with_capacity(keypoints.len());
    for kp in keypoints.iter() {
        pts.push(kp.pt());
    }
    
    Ok(FeatureExtractionResult {
        keypoints: pts,
        descriptors,
    })
}
