# Brush-SFM Repository Knowledge Base

## 1. High-Level Architecture Overview

Android-first drone-to-splat pipeline. User inputs MP4 video + Litchi CSV, system generates Gaussian splat using telemetry-informed keyframe extraction, provides interactive preview in Brush renderer.

**Platform priority**: Android-first for usage. Desktop used only for compilation/testing.

**Core contribution**: Telemetry-guided SfM pipeline converting drone footage to Gaussian splats for on-device 3D reconstruction.

## 2. Component Architecture

### A. Android Application Layer (`crates/brush-app`)
- **MainActivity.java**: Orchestrates UI, file selection, native library loading
- **VideoFrameExtractor.java**: MP4 frame extraction using Android MediaMetadataRetriever
- **TelemetrySparseReconstruction.java**: Prepares telemetry payload, invokes native SfM pipeline
- **OpenCvFrontendLib.kt**: JNI bridge for unified SfM pipeline (loads `brush_process`)
- **BundleAdjustmentLib.kt**: JNI bridge for BA operations (loads `brush_process`)

### B. Telemetry Processing Layer (Kotlin)
- **CsvIngestParser.kt**: Litchi CSV parsing with streaming approach
- **EnuConverter.kt**: WGS84 to ENU coordinate conversion
- **KeyframeSelector.kt**: Physical pose-change keyframe selection
- **TelemetryPreprocessor.kt**: Telemetry preprocessing orchestration
- **QualityFlagger.kt**: GPS quality filtering
- **Interpolator.kt**: Temporal interpolation
- **ImuIntegrator.kt**: IMU data integration
- **GapInterpolator.kt**: Telemetry gap handling

### C. Native SfM Pipeline (`crates/brush-process/src/sfm`)
- **Stage 3.1**: ORB feature extraction (OpenCV)
- **Stage 3.2**: Feature matching with GPS window pruning
- **Stage 3.3**: RANSAC essential matrix estimation
- **Stage 3.4**: Pose recovery
- **Stage 3.5**: Triangulation
- **Stage 3.6**: Inlier filtering
- **Stage 3.7**: Bundle adjustment (pure Rust, via `brush-sfm`)
- **Stage 3.8**: Pose export (transforms.json + sparse.ply)

### D. Bundle Adjustment (`crates/brush-sfm`)
- **Sliding-window BA**: 10-20 frame window (currently defaults to 8)
- **Multimodal residuals**: Reprojection, GPS, IMU
- **Schur complement**: Efficient BA implementation
- **JNI bridge**: Native library exports

### E. UI Layer (`crates/brush-ui`)
- **File selection**: MP4, CSV, JSON config
- **Extraction modes**: Uniform vs Telemetry-based
- **Training button**: Unified pipeline trigger
- **Status updates**: Real-time progress feedback

## 3. Implementation Status

### Completed
- Full SfM pipeline (Stages 3.1-3.8)
- Telemetry preprocessing with quality filtering
- Keyframe selection with physical pose-change triggers
- OpenCV integration for feature detection/matching
- Bundle adjustment with GPS/IMU priors
- Coordinate frame conversion (OpenCV to NeRF)
- Export to transforms.json + sparse.ply
- Android UI with file selection and progress feedback
- Hardware detection (Qualcomm vs Pixel adaptation)
- JSON configuration system for experimental parameters

### Known Limitations
- BA window default 8 (spec: 10-20)
- Training steps default 30,000 (spec: 2,000)
- Max splats default 10,000,000 (spec: 100,000)
- Telemetry processing in Kotlin (ideally Rust)
- Some coordinate conversion duplication
- Missing telemetry-window matching in current implementation

## 4. Hardware Adaptations

### Qualcomm vs Pixel Detection
**Required**: Hardware-specific adaptation due to 128MB maxStorageBufferBindingSize limitation on Qualcomm stack.

**Implementation**: Platform detection and adaptive memory allocation. This is necessary hack for MVP to work across target platforms.

## 5. Configuration System

### JSON Configuration
**Required**: JSON config for experimental setup. Allows same config across tests.

**Usage**: UI accepts JSON config file for SfM/BA hyperparameters. Enables reproducible research experimentation.

## 6. Build Process

### Native Libraries
**Approach**: Native libraries provided directly as binary in brush repo. Build independently.

**Benefit**: No stale .so concern. Libraries managed as binary artifacts.

### Build Order
1. Desktop Rust build (compilation/testing)
2. Android NDK cross-compile
3. Android install

## 7. Data Flow

```text
User Input: MP4 + CSV + JSON Config
       ↓
Telemetry Preprocessing (Kotlin)
       ↓
Keyframe Selection (Kotlin)
       ↓
Frame Extraction (Android)
       ↓
Feature Detection (OpenCV)
       ↓
Feature Matching (OpenCV)
       ↓
RANSAC/Pose Recovery (OpenCV)
       ↓
Triangulation (OpenCV)
       ↓
Bundle Adjustment (Rust)
       ↓
Coordinate Conversion (Rust)
       ↓
Export: transforms.json + sparse.ply
       ↓
Brush Training (stock brush-train)
       ↓
Interactive Preview (Brush renderer)
```

## 8. Research Scope

### MVP Focus
- Telemetry-guided SfM only
- Litchi CSV only (DJI SRT removed)
- Android-first usage
- Qualcomm + Pixel support

### Future Scope
- Vision-only Mode A benchmarking
- Additional telemetry formats
- Desktop parity

## 9. Validation Requirements

### Performance Validation
- End-to-end pipeline timing on target device
- Memory usage profiling
- Thermal behavior
- Resource budget enforcement (100 frames, 640x360, 3GB, 30min)

### Correctness Validation
- Synthetic BA tests
- Coordinate conversion unit tests
- Reprojection error analysis
- Spatial accuracy measurements

## 10. Key Technical Decisions

### Android-First Architecture
**Rationale**: Target usage is Android device. Desktop for compilation/testing only.

### Kotlin Telemetry Processing
**Current**: Telemetry processing in Kotlin layer.
**Rationale**: Android-first architecture. May be acceptable for MVP.
**Future**: Consider moving to Rust for architectural consistency.

### Hardware Detection
**Rationale**: Qualcomm 128MB limitation requires platform-specific adaptation.
**Status**: Necessary hack for MVP. Document as platform-specific.

### JSON Configuration
**Rationale**: Experimental setup requires reproducible config across tests.
**Status**: Required feature. Do not remove.

## 11. Known Issues

### Mathematical Correctness
- Pose chaining may need verification
- GPS/IMU residual definitions may need correction
- SO(3) updates need validation
- Yaw wraparound uses % instead of proper modulo

### Architecture
- SfM location ambiguous (brush-process vs brush-sfm)
- Coordinate conversions duplicated
- Some telemetry complexity may be excessive

### Defaults
- Resource budget defaults misaligned with spec
- Need validation on target device

## 12. Next Steps

### Immediate
1. Fix mathematical correctness issues (pose chaining, residuals, SO(3))
2. Implement telemetry-window matching
3. Align resource defaults with spec
4. Validate end-to-end pipeline

### Short-term
1. Evaluate telemetry components for performance value
2. Centralize coordinate conversions
3. Add synthetic BA tests
4. Profile on target device

### Long-term
1. Consider moving telemetry to Rust
2. Restore crate boundaries
3. Implement validation artifacts
4. Clean up technical debt
