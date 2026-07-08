---
name: opentrack-psvr-issues
description: >-
  Internals, quirks, and historical bugs of opentrack's tracker-psvr plugin
  (PlayStation VR head tracker). Covers the plugin's file layout, the
  ITrackerDialog embedding hooks that let PSVRDialog work as both a standalone
  window and an Options-dialog tab, the calibration state machine and its
  USB-unplug failure mode, the gated constellation/PnP diagnostic logging,
  the optional PS4 Camera/OV580 positional tracker, the experimental
  ini-configurable HID keepalive, and PSVR hardware quirks (HDMI passthrough,
  processor-unit cold-boot timing). Use when the user mentions PSVR, PlayStation VR, the
  PSVR helmet, head tracker, tracker-psvr, psvr.cpp, the constellation tracker,
  recalibration, the HID keepalive, the PSVR control box, headset auto-sleep,
  or the Options-dialog "Tracker" tab.
---

# opentrack — PSVR plugin issues

## Plugin layout (`tracker-psvr/`)

```
tracker-psvr/
├── psvr.cpp/.h              dialog + tracker driver, top-level glue
├── psvr_camera.h/.mm        AVFoundation enum + capture (.mm = Obj-C++)
├── psvr_constellation.cpp/.h  LED constellation PnP for positional tracking
├── psvr_mirror.h/.mm        helmet-mirror window (user can see the screens)
├── lang/*.ts                Qt translations (lupdate output)
└── CMakeLists.txt
```

The `.mm` extensions matter — those files use Objective-C++ to talk to
AVFoundation. Don't rename them to `.cpp` even though it's tempting.

## `PSVRDialog` embedding in the global Options dialog

`PSVRDialog` is rendered two ways:

1. **Standalone** — when the user clicks the wrench icon next to
   "PSVR" in the tracker dropdown.
2. **Embedded as a tab** — inside opentrack's global Options dialog,
   under the "Tracker" tab group.

Both modes share the same class. The Options dialog provides its own
OK/Cancel row at the bottom; without intervention, `PSVRDialog`'s own
`QDialogButtonBox` would render **above** that, giving two stacked
OK/Cancel rows.

The fix is the `ITrackerDialog` API hooks the Options dialog already
expects from embeddable plugins. `PSVRDialog` must override:

- `set_buttons_visible(bool)` — hide/show its own `QDialogButtonBox`.
  Options dialog calls this with `false` in `add_module_tab` before
  showing the tab.
- (plus the matching hooks for tab title and parent assignment — see
  base `ITrackerDialog` declaration.)

Standalone use is unaffected because nothing calls
`set_buttons_visible` in that path.

## Calibration state machine — USB-unplug failure mode

A naive `recalibrate` action succeeds **silently** when USB is unplugged:

- The "calibration complete" atomic flag from the previous successful
  run stays set.
- The watchdog checks the wrong conditions and doesn't surface the
  disconnect.

Fix pattern:

- **Reset calibration state on entry** to `recalibrate` so stale
  success-flags can't satisfy the completion check.
- **Extend watchdog conditions** so a USB-disconnect during
  re-calibration surfaces the disconnect banner, not a silent "OK".

Don't re-introduce a code path where recalibration depends on the
**previous** run's atomic flags.

## Diagnostic logging is opt-in

The constellation tracker's per-frame PnP solve produces a lot of
detail. Logging it on every frame drowns the console and makes real
warnings unreadable.

Per-frame logs are **gated** on the `enable-diag-log` setting in the
plugin's ini or on the `PSVR_CONSTELLATION_LOG` environment variable.
`enable-diag-log` bridges to `/tmp/psvr-constellation.log` at tracker
start; setting the env var directly lets you choose another path. The
per-second `[psvr-cam]` summaries go to stderr and are okay; don't add
unconditional `qDebug()` calls in any per-frame path.

## HID keepalive — experimental, opt-in

PSVR firmware auto-sleeps the headset after a period of HID inactivity.
A lightweight periodic HID write is available, but it is still
experimental because some command bytes interrupt the IMU stream.

The keepalive is:

- **Configurable from the ini**, not the GUI (deliberately
  experimental).
- **Off by default**.
- Default probe is `keepalive-cmd=0x17`, `keepalive-interval-s=60`.
  `0x17` sends `SetHeadsetPower(ON)` only; it is much lighter than the
  full activation burst.
- The old 10 s full-burst keepalive (`0x17` + `0x11`) caused visible
  1-2 s IMU stalls every time it fired. Do not bring that behavior back.
- If the HID stream goes silent after calibration, the watchdog still
  fires the full activation burst as a recovery path.

## Hardware quirks

| Quirk | What to tell the user |
|---|---|
| PSVR control box HDMI passthrough is off when the box is off | If they see "no image" on the headset, suggest bypassing the box and plugging HDMI directly into the helmet to isolate whether the box is the issue. |
| Processor-unit cold-boot handshake takes 5–15 s after Start | The watchdog should not surface an error until ~25 s. Premature error banners confuse users into unplugging while the box is still negotiating. |
| Auto-sleep mid-session | See HID keepalive above. Use `/tmp/psvr-diag.log` to confirm whether reports stopped and whether keepalive commands were accepted. |

## PS4 Camera (OV580) — camera-based positional tracking

The optional camera path tracks the helmet's 9 blue LEDs for X/Y/Z. The
**PS4 Camera** (Sony's stereo cam, OV580 ASIC) is the right hardware for
it (blue-tuned IR-cut filter). Key facts:

- **USB 3.0 only.** It will NOT enumerate behind a USB 2.0 hub (e.g. the
  Honeycomb XPC / GenesysLogic "USB2.1 Hub"). Plug into a native
  SuperSpeed port or a *powered* USB 3.0 hub. Verify:
  `ioreg -p IOUSB -l | grep UsbLinkSpeed` → want `5000000000`, not
  `480000000`. At 2.0 the OV580 won't even show in Boot mode.
- **One-time firmware upload per power cycle.** Until uploaded it
  enumerates as `USB Boot` (no camera). tracker-psvr embeds the blob and
  uploads it via libusb on tracker start. Manual tool:
  `psmove camera-firmware` (thp/psmoveapi); retry once if it returns
  `-4`/`-7` mid-re-enumeration. After upload it appears as a UVC webcam
  `USB Camera-OV580`. Firmware blob from ps4eye/ps4eye (SHA-1
  `fe86162309518a0ffe267075a2fcf728c5856b3e`).
- **Frame layout (de-interleave).** AVFoundation delivers the native
  `yuvs` (YUYV) frame; the image data is packed CONTIGUOUSLY at `w*2`
  bytes/row (NOT the reported `CVPixelBufferGetBytesPerRow`), and each
  row is `[32B+64B hdr][left eye ew*2][right eye ew*2][junk]`. Decode at
  `w*2` stride, skip 96B/row, take the left eye (640×400 for the
  1748×408 mode). Decoding at the reported stride/width shears the image
  diagonally. We use the left lens only (monocular PnP); stereo is a
  possible future upgrade.
- **Frame rate.** Default is the stable AVFoundation-negotiated cadence
  (observed as 1748×408 at ~30 fps). `PSVR_CAM_FPS=60` can force a
  faster OV580 mode for experiments, using AVFoundation's exact
  advertised `CMTime` duration. In testing it started cleanly but the
  stream went stale after a few frames, so keep it opt-in.
- **HFOV** auto-selects by camera (OV580 = 85°); manual override in the
  dialog.
- **Pose solve.** User-visible yaw/pitch/roll always comes from the
  PSVR IMU. The camera solver is free-pose PnP internally, but its rvec
  is used only as the optical prior for the next frame. The current
  stability stack is:
  - project the 9-LED model from the last accepted camera pose;
  - greedily match projected visible LEDs to blobs one-to-one;
  - if prior matching gives enough inliers, run direct iterative
    `solvePnP` seeded from the prior;
  - otherwise fall back to sampled AP3P correspondence search and
    `solvePnPRansac`;
  - reject with RMS, Z-range, cm/jump, and internal camera-rvec jump
    gates (`ROT_JUMP`) so mirror/twin PnP branches don't become visible
    XYZ jumps.
- **Cold first lock.** A 4-LED first lock must be stable for several
  close frames before publishing; otherwise `TENTATIVE_FIRST_LOCK` /
  `WEAK_FIRST_LOCK` protects against latching a one-frame false pose.
- **Tracking diagnostics.** The worker logs `[psvr-cam] frames=… blobs=…
  vis=… matched=… pnp=… reject=… ypr=[…] pos=[…]` to stderr (~1/s) plus
  optional per-frame constellation lines to `/tmp/psvr-constellation.log`
  when `enable-diag-log` is on or `PSVR_CONSTELLATION_LOG` is set.
  Frame/image dumps are off by default; set `PSVR_CAM_DUMP_FRAMES=1` to
  write `/tmp/psvr-frame.pgm` and `/tmp/psvr-preview.ppm`. Run opentrack
  with stderr captured (see opentrack-build skill) to see the summaries.
  A dark room dramatically cuts noise blobs.

## Current known-good PSVR tracking profile

Last validated on this branch with a PS4 Camera OV580:

- Active camera format: `1748x408 yuvs`, de-interleaved to left-eye
  `640x400`.
- Default cadence: ~30 fps.
- Stable lock: `pnp_ok` close to total frame count (example:
  `837/840`), four or five LED matches, RMS around 1-2 px when the
  headset is still and LEDs are clean.
- If XYZ feels slow after those numbers are good, suspect camera frame
  rate or opentrack filter/mapping smoothing rather than the PnP solve.
