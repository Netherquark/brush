## Key Contributions

- Developed and integrated a **full end-to-end Structure-from-Motion (SfM) pipeline** (Stages 3.1–3.8) using native OpenCV, covering:
  - Feature detection (ORB)
  - Feature matching (KNN)
  - Geometric filtering (RANSAC)
  - Pose recovery
  - Triangulation
  - Bundle adjustment
  - Final export

- Replaced legacy C++ wrappers with a **Rust-based OpenCV integration**, improving maintainability and enabling a unified native pipeline.

- Designed and implemented a **JNI communication layer** between Android (Java/Kotlin) and Rust backend for seamless cross-platform execution.

- Built a **unified "Train" pipeline trigger** in the UI, consolidating multiple processing stages into a single action with real-time status updates.

- Implemented a **video frame extraction system** for processing MP4 inputs into SfM-compatible image sequences.

- Developed a **telemetry preprocessing pipeline**:
  - Integrated GPS and IMU data handling
  - Built CSV parsing and realignment logic
  - Enabled sensor-driven spatial priors for reconstruction

- Added **bundle adjustment module** to improve global reconstruction accuracy.

- Implemented **PLY point cloud export** and `transforms.json` generation for downstream Gaussian Splatting workflows.

- Optimized Android build system:
  - Reduced build time (~3× faster)
  - Modularized OpenCV 4.13 native libraries with NEON support

- Designed and integrated **file picker backend** and improved Android UI/UX:
  - Button state handling during processing
  - Layout improvements
  - Processing feedback indicators

- Added **hyperparameter configuration via JSON input through UI** for flexible experimentation.

- Implemented **robust error handling and feedback loop** from native backend to UI via JNI.


## System-Level Impact

- Delivered a **fully functional on-device 3D reconstruction pipeline** for drone data.

- Transitioned architecture to a **unified Rust-native processing engine**, eliminating fragmented workflows.

- Enabled compatibility with **Gaussian Splatting training pipelines** via standardized outputs (`sparse.ply`, `transforms.json`).