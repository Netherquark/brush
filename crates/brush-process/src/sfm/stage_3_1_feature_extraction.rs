use opencv::{
    core::{Mat, Vector, Point2f},
    features2d::ORB,
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

pub fn extract_features(image_bytes: &[u8]) -> Result<FeatureExtractionResult, Box<dyn std::error::Error>> {
    // Decode the image from the byte buffer
    let byte_vector: Vector<u8> = Vector::from_slice(image_bytes);
    let original_image = imgcodecs::imdecode(&byte_vector, imgcodecs::IMREAD_COLOR)?;
    
    // Convert to grayscale
    let mut gray_image = Mat::default();
    imgproc::cvt_color(&original_image, &mut gray_image, imgproc::COLOR_BGR2GRAY, 0, opencv::core::AlgorithmHint::ALGO_HINT_DEFAULT)?;
    
    // Use ORB to detect keypoints and compute descriptors
    // N_FEATURES, SCALE_FACTOR, N_LEVELS, EDGE_THRESHOLD, FIRST_LEVEL, WTA_K, SCORE_TYPE, PATCH_SIZE, FAST_THRESHOLD
    let mut orb = ORB::create_def()?;
    
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
