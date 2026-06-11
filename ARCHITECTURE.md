# opentrack — architecture overview

High-level map of this opentrack fork for contributors and agents.
Build/run details live in the Cursor skills under `.cursor/skills/`
(`opentrack-build`, `opentrack-osx-issues`, `opentrack-psvr-issues`).

## What opentrack is

A 6DOF head-tracking app. The core process reads a **pose** (yaw, pitch,
roll, X, Y, Z) from a *tracker*, optionally transforms it through
*filters*, and emits it to a game/sim through a *protocol*. Everything
beyond the GUI core is a dynamically-loaded plugin (a `.dylib` on
macOS).

```
  Tracker  ──pose──▶  Filter(s)  ──▶  Mapping/curves  ──▶  Protocol  ──▶  game/sim
 (input)            (smoothing)        (per-axis)         (output)
```

## Plugin model

Plugins are built as separate shared libraries and discovered at runtime
from the app bundle's `Contents/MacOS/Plugins/`. Three kinds:

| Kind | Prefix | Examples |
|---|---|---|
| Tracker (pose source) | `tracker-` | `tracker-psvr`, `tracker-pt`, `tracker-easy`, `tracker-aruco`, `tracker-neuralnet` |
| Protocol (pose sink) | `proto-` | `proto-wine`, `proto-iokit-foohid`, `proto-osc` |
| Filter | `filter-` | accela, kalman, etc. |

Each plugin exposes a metadata struct + a settings dialog (Qt) and is
loaded via the `ITracker` / `IProtocol` / `IFilter` + `*Dialog`
interfaces.

## Key directories

| Path | Role |
|---|---|
| `gui/` | main window, plugin pickers, app icon (`gui/images/opentrack.png`) |
| `options/` | settings/ini bundle system (`options::value<T>`, `tie_setting`) |
| `compat/` | cross-cutting helpers (e.g. `compat/camera-names` shared camera enumerator) |
| `api/` | plugin interfaces (`ITracker`, `IProtocol`, `ITrackerDialog`, …) |
| `tracker-*/` `proto-*/` `filter-*/` | the plugins |
| `macosx/` | macOS bundle packaging: `Info.plist`, `make-app-bundle.sh` |
| `dev/` | this fork's dev tooling: `hot-install.sh`, `setup-signing-cert.sh`, `psvr-poweron` |
| `*/lang/*.ts` | Qt translations (lupdate output) |

## tracker-psvr (this fork's focus)

PlayStation VR head tracker, macOS-native. Two pose sources fused:

- **IMU (rotation)** — reads the PSVR's gyro+accel over USB-HID
  (`IOHIDManager`), a complementary filter produces yaw/pitch/roll.
  Files: `psvr.cpp/.h`.
- **Camera (position)** — optional. AVFoundation capture
  (`psvr_camera.mm`) + OpenCV blob extraction + LED-constellation PnP
  (`psvr_constellation.cpp`) recover X/Y/Z. Works with a webcam or, best,
  the PS4 Camera (OV580). The plugin embeds + uploads the PS4 Camera
  firmware (libusb) so it appears as a UVC webcam.

Output: `data[0..2]` = X/Y/Z (cm/mm), `data[3..5]` = yaw/pitch/roll.

## Sibling repos (not part of this build)

- **cvfr-bridge** (`github.com/msupino/cvfr-bridge`) — X-Plane → JSON
  bridge feeding the cvfr-map web app.
- **x-plane-plugin** — an `opentrack/center` recenter bridge over POSIX
  shared memory (parked PR).

## Build & run

See `.cursor/skills/opentrack-build/SKILL.md`. TL;DR:
- First build: `cmake -S . -B build -DCMAKE_BUILD_TYPE=RELEASE
  -DCMAKE_INSTALL_PREFIX="$PWD/install"` → `cmake --build build` →
  `cmake --install build`.
- Dev inner loop: `dev/hot-install.sh tracker-psvr`.
- Full distributable: `cmake --install build` + `macosx/make-app-bundle.sh`.

## macOS pitfalls

See `.cursor/skills/opentrack-osx-issues/SKILL.md` — the short list:
double-Qt from un-fixed rpaths, `presets/README.txt` codesign trap,
re-sign after `install_name_tool`, AVFoundation (not Qt) for camera
enumeration, TCC permissions, and the PS4 Camera USB-3.0 requirement.
