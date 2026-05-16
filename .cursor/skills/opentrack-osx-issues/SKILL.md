---
name: opentrack-osx-issues
description: >-
  macOS-specific quirks, bugs, and workarounds in the opentrack codebase.
  Covers macdeployqt aftermath (stale rpaths, duplicate Qt loads), code-signing
  the bundle (including the presets/README.txt trap), AVFoundation vs Qt
  camera enumeration on macOS, TCC permission prompts (Input Monitoring and
  Camera), CVPixelBuffer lifetime hazards with OpenCV, and the X-Plane recenter
  bridge plugin (which exists because Qt global shortcuts on macOS can't bind
  to joystick buttons). Use when the user mentions macdeployqt, install_name_tool,
  rpath, code signing, QCameraInfo, AVFoundation, AVCaptureDeviceDiscoverySession,
  Input Monitoring permission, CVPixelBuffer, IOSurface, opentrack/center,
  XPLMRegisterCommandHandler, or a macOS-only opentrack crash.
---

# opentrack — macOS issues

## macdeployqt aftermath

`macdeployqt` rewrites Qt links into the bundle but leaves dylibs in
`Contents/Frameworks/` and `Contents/MacOS/Plugins/` carrying **stale
rpaths** that re-resolve back to the Homebrew Cellar. dyld then loads
a **second** copy of Qt at runtime → duplicate-class warnings,
`QPixmap: Must construct a QGuiApplication before a QPixmap`, hard
crash.

Strip rpaths from **both** dirs before deep code-signing:

```bash
for f in install/opentrack.app/Contents/Frameworks/*.dylib \
         install/opentrack.app/Contents/MacOS/Plugins/*.dylib; do
    install_name_tool -delete_rpath /opt/homebrew/opt/qtbase/lib "$f" 2>/dev/null || true
    # (delete any other LC_RPATH entries pointing at the Cellar)
done
```

Verify with:

```bash
otool -l install/opentrack.app/Contents/MacOS/Plugins/libopentrack-tracker-psvr.dylib | grep -A2 LC_RPATH
```

## `presets/README.txt` codesign trap

Deep-signing the whole bundle fails on `presets/README.txt` — it's a
plain text file, codesign chokes, and the entire pass aborts. Either
exclude that file or remove it before the deep-sign.

## Re-sign after `install_name_tool`

Any `install_name_tool` edit to a Mach-O invalidates its signature.
On Apple Silicon, dyld then refuses to load the binary. Re-sign every
modified `.dylib` and the main exe after editing.

## Camera enumeration: AVFoundation, not Qt

`tracker-psvr/psvr_camera.mm` enumerates webcams via AVFoundation, not
Qt. Reason:

- `QCameraInfo::availableCameras()` returns devices in an order that
  **doesn't match** `cv::VideoCapture`'s integer device indices on
  macOS. User picks "PSVR camera" in the combobox; OpenCV opens the
  FaceTime camera.
- AVFoundation's `AVCaptureDeviceDiscoverySession` sorted by
  `uniqueID` produces a stable ordering that `cv::VideoCapture(i)`
  agrees with.

Do not regress to Qt enumeration on macOS.

## TCC permissions

The PSVR plugin needs the user to grant **two** permissions in
`System Settings > Privacy & Security`:

| Permission | Why |
|---|---|
| Input Monitoring | USB HID reads from the PSVR processor unit |
| Camera | AVFoundation capture for the constellation tracker |

The opentrack binary needs to surface a **clear banner** when either
permission is missing — silent capture failures are very hard for users
to debug.

## CVPixelBuffer use-after-unlock (OpenCV)

When wrapping a `CVPixelBuffer` for `cv::Mat`:

```objc
// BAD - cv::Mat referenced IOSurface backing after unlock
CVPixelBufferLockBaseAddress(pixelBuffer, kCVPixelBufferLock_ReadOnly);
cv::Mat src(h, w, CV_8UC4, CVPixelBufferGetBaseAddress(pixelBuffer));
CVPixelBufferUnlockBaseAddress(pixelBuffer, kCVPixelBufferLock_ReadOnly);
cv::cvtColor(src, dst, cv::COLOR_BGRA2BGR);   // use-after-unlock!
```

```objc
// GOOD - convert first, then unlock
CVPixelBufferLockBaseAddress(pixelBuffer, kCVPixelBufferLock_ReadOnly);
cv::Mat src(h, w, CV_8UC4, CVPixelBufferGetBaseAddress(pixelBuffer));
cv::cvtColor(src, dst, cv::COLOR_BGRA2BGR);
CVPixelBufferUnlockBaseAddress(pixelBuffer, kCVPixelBufferLock_ReadOnly);
```

Symptoms of the bug: occasional corrupted frames, hard-to-reproduce
crashes in OpenCV under memory pressure.

## X-Plane recenter bridge (a macOS-specific workaround)

`x-plane-plugin/plugin.c` exists because **Qt's global-shortcut system
on macOS supports only keyboard keys, not joystick buttons**. The
plugin registers an `opentrack/center` command in X-Plane and bridges
it through POSIX shared memory to the opentrack desktop app, giving
the user joystick-button re-center by binding the command in X-Plane's
UI.

Pitfalls to avoid:

- **Don't call `reinit_offset()`** from the center command — it
  introduces Y-axis drift.
- **Always call `XPLMUnregisterCommandHandler` in `XPluginStop`.**
  Without it, reload-on-edit during dev crashes X-Plane on the next
  enable.
- Shared memory uses `flock` + an atomic flag, no busy-wait.
- Shared-memory POSIX name is set in `plugin.c`; keep it in sync with
  the opentrack-side reader.
