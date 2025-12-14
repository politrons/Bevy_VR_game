# bevy_quest_hello

A minimal Bevy app you can:
- run on **macOS** first (`cargo run`) to validate the Rust/Bevy side,
- then build as an **Android APK** and sideload onto **Meta Quest** (Quest 3/3S/Pro/2).

This build is **not yet immersive VR** (OpenXR). On Quest it will run as a **2D panel app** showing a 3D scene (spinning cube).
Once this pipeline works, the next step is switching the renderer/input to OpenXR for real VR.

## 1) Run on macOS (smoke test)

From the project root:

    rustup update
    cargo run

You should see a window with a spinning cube.

## 2) Build the native library for Android (arm64)

Prereqs:
- Android Studio (installs SDK + NDK)
- `rustup target add aarch64-linux-android`
- `cargo install --locked cargo-ndk`

Build:

    cargo ndk -t arm64-v8a -o app/src/main/jniLibs build --release

Note: Bevy may require `libc++_shared.so` to be packaged in the APK depending on your setup.
If the app crashes on launch, copy the NDK `libc++_shared.so` into:

    app/src/main/jniLibs/arm64-v8a/libc++_shared.so

## 3) Build APK with Gradle

Generate the wrapper (once):

    gradle wrapper

Build:

    ./gradlew :app:assembleDebug

APK output:

    app/build/outputs/apk/debug/app-debug.apk

## 4) Install to Quest via USB (sideload)

Enable Developer Mode on the headset and accept USB debugging prompts.

Then:

    adb devices
    adb install -r app/build/outputs/apk/debug/app-debug.apk
