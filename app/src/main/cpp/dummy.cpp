#include <jni.h>

// This file exists only to satisfy Gradle/AGP native build expectations.
// The real native code is built with Rust and placed under app/src/main/jniLibs/.
extern "C" JNIEXPORT jint JNICALL
JNI_OnLoad(JavaVM* vm, void* reserved) {
    (void)vm;
    (void)reserved;
    return JNI_VERSION_1_6;
}
