# Brush

<video src=https://github.com/user-attachments/assets/5756967a-846c-44cf-bde9-3ca4c86f1a4d>A video showing various Brush features and scenes</video>

<p align="center">
  <i>
    Massive thanks to <a href="https://www.youtube.com/@gradeeterna">@GradeEterna</a> for the beautiful scenes
  </i>
</p>

Brush is a 3D reconstruction engine using [Gaussian splatting](https://repo-sam.inria.fr/fungraph/3d-gaussian-splatting/). It works on a wide range of systems: **macOS/windows/linux**, **AMD/Nvidia/Intel** cards, **Android**, and in a **browser**. To achieve this, it uses WebGPU compatible tech and the [Burn](https://github.com/tracel-ai/burn) machine learning framework.

Machine learning for real time rendering has tons of potential, but most ML tools don't work well with it: Rendering requires realtime interactivity, usually involve dynamic shapes & computations, don't run on most platforms, and it can be cumbersome to ship apps with large CUDA deps. Brush on the other hand produces simple dependency free binaries, runs on nearly all devices, without any setup.

[**Try the web demo** <img src="https://cdn-icons-png.flaticon.com/256/888/888846.png" alt="chrome logo" width="24"/>
](https://arthurbrussee.github.io/brush-demo)
_NOTE: Only works on Chrome and Edge. Firefox and Safari are hopefully supported soon)_

[![](https://dcbadge.limes.pink/api/server/https://discord.gg/TbxJST2BbC)](https://discord.gg/TbxJST2BbC)

# Features

## Training

Brush takes in COLMAP data or datasets in the Nerfstudio format. Training is fully supported natively, on mobile, and in a browser. While training you can interact with the scene and see the training dynamics live, and compare the current rendering to input views as the training progresses.

It also supports masking images:
- Images with transparency. This will force the final splat to match the transparency of the input.
- A folder of images called 'masks'. This ignores parts of the image that are masked out.

## Viewer
Brush also works well as a splat viewer, including on the web. It can load .ply & .compressed.ply files. You can stream in data from a URL (for a web app, simply append `?url=`).

Brush also can load .zip of splat files to display them as an animation, or a special ply that includes delta frames (see [cat-4D](https://cat-4d.github.io/) and [Cap4D](https://felixtaubner.github.io/cap4d/)!).

## CLI
Brush can be used as a CLI. Run `brush --help` to get an overview. Every CLI command can work with `--with-viewer` which also opens the UI, for easy debugging.

## Rerun

https://github.com/user-attachments/assets/f679fec0-935d-4dd2-87e1-c301db9cdc2c

While training, additional data can be visualized with the excellent [rerun](https://rerun.io/). To install rerun on your machine, please follow their [instructions](https://rerun.io/docs/getting-started/installing-viewer). Open the ./brush_blueprint.rbl in the viewer for best results.

## Building Brush
First install rust 1.88+. You can run tests with `cargo test --all`. Brush uses the wonderful [rerun](https://rerun.io/) for additional visualizations while training, run `cargo install rerun-cli` if you want to use it.

### Windows/macOS/Linux
Use `cargo run --release` from the workspace root to make an optimized build. Use `cargo run` to run a debug build. 

### Web
Brush can be compiled to WASM. Run `npm run dev` to start the demo website using Next.js, see the brush_nextjs directory.

Brush uses [`wasm-pack`](https://drager.github.io/wasm-pack/) to build the WASM bundle. You can also use it without a bundler, see [wasm-pack's documentation](https://drager.github.io/wasm-pack/book/).

WebGPU is still an upcoming standard, and as such, only Chrome 134+ on Windows and macOS is currently supported.

### Android

As a one-time setup, make sure you have the Android SDK & NDK installed.
- Copy `.env.example` to `.env` and fill in the Android/OpenCV paths you want Android Studio to use
- Or set `sdk.dir` / `ndk.dir` in `local.properties`
- Add the Android target to rust: `rustup target add aarch64-linux-android`
- Install cargo-ndk to manage building a lib: `cargo install cargo-ndk`

**Building OpenCV Native Libraries**
Brush uses custom OpenCV native libraries for Android cross-compilation. You must clone and build OpenCV 4.13.0 from source before building the main application.
```bash
git clone https://github.com/opencv/opencv.git
cd opencv && git checkout 4.13.0
mkdir build && cd build

export ANDROID_NDK_HOME=/path/to/your/ndk/directory
export ANDROID_SDK_HOME=/path/to/your/sdk/directory

cmake -GNinja \
  -DCMAKE_MAKE_PROGRAM=ninja-build \
  -DCMAKE_TOOLCHAIN_FILE=$ANDROID_NDK_HOME/build/cmake/android.toolchain.cmake \
  -DANDROID_ABI="arm64-v8a" \
  -DANDROID_PLATFORM=android-30 \
  -DANDROID_SDK=$ANDROID_SDK_HOME \
  -DBUILD_SHARED_LIBS=ON \
  -DBUILD_opencv_java=OFF \
  -DBUILD_opencv_js=OFF \
  -DBUILD_ANDROID_PROJECTS=OFF \
  -DBUILD_ANDROID_EXAMPLES=OFF \
  -DBUILD_opencv_videoio=OFF \
  -DBUILD_opencv_video=OFF \
  -DBUILD_opencv_dnn=OFF \
  -DBUILD_opencv_ml=OFF \
  -DBUILD_opencv_photo=OFF \
  -DBUILD_opencv_gapi=OFF \
  -DBUILD_opencv_objdetect=OFF \
  -DWITH_TBB=ON \
  -DBUILD_TESTS=OFF \
  -DBUILD_PERF_TESTS=OFF \
  -DBUILD_EXAMPLES=OFF \
  ..

ninja-build
```

Android Studio now checks configuration in this order where applicable:
- Gradle project properties / `local.properties`
- workspace `.env`
- regular process environment variables

Each time you change the rust code, run
- `cargo ndk -t arm64-v8a -o crates/brush-app/app/src/main/jniLibs/ build`
- Nb:  Nb, for best performance, build in release mode. This is separate
  from the Android Studio app build configuration.
- `cargo ndk -t arm64-v8a -o crates/brush-app/app/src/main/jniLibs/  build --release`

You can now either run the project from Android Studio (Android Studio does NOT build the rust code), or run it from the command line:
```
./gradlew build
./gradlew installDebug
adb shell am start -n com.splats.app/.MainActivity
```

### Android Workflow
1. **Choose MP4**: Select the drone video file.
2. **Choose CSV**: Select the corresponding telemetry log (DJI CSV format).
3. **Choose Config**: (Optional) Select a JSON configuration file for SfM/Training parameters.
4. **Extract**: Choose between "Uniform" extraction or "Telemetry" based extraction.
5. **Train**: Start the full on-device SfM and Splatting pipeline.

You can also open this folder as a project in Android Studio and run things from there. Nb: Running in Android Studio does _not_ rebuild the rust code automatically.

## Benchmarks

Rendering and training are generally faster than gsplat. You can run benchmarks of some of the kernels using `cargo bench`.

# Acknowledgements

[**gSplat**](https://github.com/nerfstudio-project/gsplat), for their reference version of the kernels

**Peter Hedman, George Kopanas & Bernhard Kerbl**, for the many discussions & pointers.

**The Burn team**, for help & improvements to Burn along the way

**Raph Levien**, for the [original version](https://github.com/googlefonts/compute-shader-101/pull/31) of the GPU radix sort.

**GradeEterna**, for feedback and their scenes.

# Disclaimer

This is *not* an official Google product. This repository is a forked public version of [the google-research repository](https://github.com/google-research/google-research/tree/master/brush_splat)
