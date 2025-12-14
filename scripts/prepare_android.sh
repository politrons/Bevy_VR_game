#!/usr/bin/env bash
set -euo pipefail

# English comments as requested.
# This script builds the ARM64 native library via cargo-ndk and tries to copy libc++_shared.so if found.

PROJECT_PKG="bevy_quest_hello"
OUT_DIR="app/src/main/jniLibs"
ABI_DIR="${OUT_DIR}/arm64-v8a"

command -v cargo >/dev/null 2>&1 || { echo "cargo not found"; exit 1; }
command -v cargo-ndk >/dev/null 2>&1 || { echo "cargo-ndk not found. Install with: cargo install --locked cargo-ndk"; exit 1; }

mkdir -p "${ABI_DIR}"

echo "[1/2] Building Rust library for Android (arm64-v8a)..."
cargo ndk -t arm64-v8a -o "${OUT_DIR}" build --package "${PROJECT_PKG}" --release

echo "[2/2] Attempting to locate and copy libc++_shared.so..."
NDK_HOME="${ANDROID_NDK_HOME:-${ANDROID_NDK_ROOT:-}}"
if [[ -z "${NDK_HOME}" ]]; then
  echo "ANDROID_NDK_HOME / ANDROID_NDK_ROOT not set; skipping libc++_shared.so copy."
  echo "If your app crashes on launch, copy libc++_shared.so into ${ABI_DIR}/ manually."
  exit 0
fi

# Try common prebuilt paths (macOS).
CANDIDATES=(
  "${NDK_HOME}/toolchains/llvm/prebuilt/darwin-x86_64/sysroot/usr/lib/aarch64-linux-android/libc++_shared.so"
  "${NDK_HOME}/toolchains/llvm/prebuilt/darwin-arm64/sysroot/usr/lib/aarch64-linux-android/libc++_shared.so"
)

FOUND=""
for c in "${CANDIDATES[@]}"; do
  if [[ -f "${c}" ]]; then
    FOUND="${c}"
    break
  fi
done

if [[ -z "${FOUND}" ]]; then
  echo "Could not find libc++_shared.so under NDK: ${NDK_HOME}"
  echo "If your app crashes on launch, locate libc++_shared.so in your NDK and copy it into ${ABI_DIR}/"
  exit 0
fi

cp -f "${FOUND}" "${ABI_DIR}/libc++_shared.so"
echo "Copied libc++_shared.so -> ${ABI_DIR}/libc++_shared.so"
