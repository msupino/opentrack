---
name: opentrack-psvr-issues
description: >-
  Internals, quirks, and historical bugs of opentrack's tracker-psvr plugin
  (PlayStation VR head tracker). Covers the plugin's file layout, the
  ITrackerDialog embedding hooks that let PSVRDialog work as both a standalone
  window and an Options-dialog tab, the calibration state machine and its
  USB-unplug failure mode, the gated constellation/PnP diagnostic logging,
  the experimental ini-configurable HID keepalive (delays auto-sleep by
  ~2 minutes), and PSVR hardware quirks (HDMI passthrough, processor-unit
  cold-boot timing). Use when the user mentions PSVR, PlayStation VR, the
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
detail. Logging it on every frame at 60 fps drowns the console and
makes real warnings unreadable.

Per-frame logs are **gated** on the `enable-diag-log` setting in the
plugin's ini. Add a conditional, e.g.:

```cpp
if (s.enable_diag_log)
    qDebug() << "constellation:" << ...;
```

Don't add unconditional `qDebug()` calls in any per-frame path.

## HID keepalive — experimental, opt-in

PSVR firmware auto-sleeps the headset after a period of HID inactivity.
A periodic HID write **delays** auto-sleep — but only by about
2 minutes total, not indefinitely. That's a firmware quirk, not a code
bug.

The keepalive is:

- **Configurable from the ini**, not the GUI (deliberately
  experimental).
- **Off by default**.
- Worth mentioning if a user reports the headset sleeping mid-session
  — but don't promise it'll keep the headset awake forever.

## Hardware quirks

| Quirk | What to tell the user |
|---|---|
| PSVR control box HDMI passthrough is off when the box is off | If they see "no image" on the headset, suggest bypassing the box and plugging HDMI directly into the helmet to isolate whether the box is the issue. |
| Processor-unit cold-boot handshake takes 5–15 s after Start | The watchdog should not surface an error until ~25 s. Premature error banners confuse users into unplugging while the box is still negotiating. |
| Auto-sleep mid-session | See HID keepalive above. Even with it on, max delay is ~2 min. |
