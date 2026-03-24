use opencv::{
    core,
    features2d,
    prelude::*,
    Result,
};

/// FFI Interface: Exposed to Android/JNI
/// Returns 0 on success, -1 on error.
#[no_mangle]
pub extern "C" fn rust_opencv_test() -> i32 {
    match run_orb_detection() {
        Ok(count) => {
            println!("OpenCV Success! Keypoints found: {}", count);
            count
        }
        Err(e) => {
            eprintln!("OpenCV Error: {:?}", e);
            -1
        }
    }
}

/// Internal logic using the `opencv` Rust crate
fn run_orb_detection() -> Result<i32> {
    // 1. Create ORB Detector
    let mut orb = features2d::ORB::create(
        500,    // nfeatures
        1.2,    // scaleFactor
        8,      // nlevels
        31,     // edgeThreshold
        0,      // firstLevel
        2,      // WTA_K
        features2d::ORB_ScoreType::HARRIS_SCORE,
        31,     // patchSize
        20      // fastThreshold
    )?;

    // 2. Create a dummy image (black, 640x480)
    let img = core::Mat::zeros(480, 640, core::CV_8UC1)?.to_mat()?;

    // 3. Detect and Compute
    let mut keypoints = opencv::core::Vector::new();
    let mut descriptors = core::Mat::default();

    orb.detect_and_compute(
        &img,
        &core::no_array(),
        &mut keypoints,
        &mut descriptors,
        false,
    )?;

    // Return the number of keypoints
    Ok(keypoints.len() as i32)
}