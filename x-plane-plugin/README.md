# opentrack X-Plane plugin

A 258-line C plugin (`opentrack.xpl`) that runs inside X-Plane and drives
the six pilot-head-pose datarefs from head tracking data written by
opentrack. It is the X-Plane-side counterpart to the **"X-Plane"** output
protocol in opentrack; the two processes communicate through a POSIX
shared-memory block, not a network socket.

Supported platforms: Linux, macOS (arm64 and x86_64), Windows.

## How the two halves fit together

```
opentrack process                              X-Plane process
┌─────────────────────────────┐                ┌─────────────────────────────┐
│  "X-Plane" output protocol   │                │  opentrack.xpl:              │
│  (proto-wine built with      │ ── POSIX ──>  │   flock()s the shm, reads 6  │
│  -DOTR_WINE_NO_WRAPPER):     │  shm_open +    │   doubles per frame, drives  │
│    writes {x,y,z,yaw,pitch,  │  mmap          │   sim/graphics/view/         │
│    roll} into shm block      │                │    pilots_head_{x,y,z,psi,   │
│    /facetracknoir-wine-shm   │                │    the,phi}                  │
└─────────────────────────────┘                └─────────────────────────────┘
```

The shared-memory block is named `facetracknoir-wine-shm` (same name used
by the Wine freetrack proxy, so one plugin binary serves both use cases).

## Building

The plugin is only compiled when the X-Plane SDK path is provided via the
`SDK_XPLANE` CMake cache variable; with an empty path it is silently
skipped.

1. Download the X-Plane SDK from
   [developer.x-plane.com/sdk/plugin-sdk-downloads/](https://developer.x-plane.com/sdk/plugin-sdk-downloads/)
   (free, ~2 MB zipped). Current version at time of writing: `XPSDK430.zip`.

2. Unzip it somewhere; note the `SDK/` directory inside the zip.

3. Configure the opentrack build tree with the SDK path:

   ```bash
   cmake -S . -B build -DSDK_XPLANE=/path/to/XPSDK430/SDK
   cmake --build build --target opentrack-xplane-plugin opentrack-proto-wine
   ```

   - `opentrack-xplane-plugin` → produces `opentrack.xpl` (the X-Plane side).
   - `opentrack-proto-wine` → produces `opentrack-proto-wine.dylib`/`.so`/
     `.dll`, the **"X-Plane"** entry in opentrack's output dropdown.

   Both targets need `SDK_XPLANE` set. Without it, the output dropdown
   shows only non-X-Plane protocols.

The output bundle is emitted at `x-plane-plugin/opentrack.xpl`.

## Installing the plugin into X-Plane

Copy `opentrack.xpl` into the standard X-Plane plugin layout. The
subdirectory name is platform-specific:

```
X-Plane 12/
  Resources/
    plugins/
      opentrack/
        mac_x64/   ← macOS (arm64 and x86_64)
        64/        ← Linux / Windows
          opentrack.xpl
```

One-liner for a typical macOS install:

```bash
XP12=~/X-Plane\ 12
mkdir -p "$XP12/Resources/plugins/opentrack/mac_x64"
cp x-plane-plugin/opentrack.xpl "$XP12/Resources/plugins/opentrack/mac_x64/"
```

Verify which path X-Plane actually picked up by grepping its
`X-Plane 12/Log.txt` for `opentrack.xpl` after the next launch;
if both `mac_x64/` and `64/` trees exist, X-Plane loads one and
silently ignores the other.

On first launch X-Plane's `Log.txt` (in the sim's root folder) should
contain a line like `opentrack init complete` emitted by
`XPluginStart`. If you instead see `opentrack failed to init SHM!`, the
plugin couldn't open `/facetracknoir-wine-shm` for RW — check your
filesystem permissions on `/dev/shm` (Linux) or System Settings →
Privacy sandboxing (macOS).

## Using it from opentrack

1. In opentrack, pick **X-Plane** from the **Output** dropdown. This is
   `proto-wine` built in no-wrapper mode; its icon is the X-Plane logo.
   It writes the shared-memory block the plugin reads.
2. Start opentrack tracking as usual. The plugin is passive — it just
   mirrors whatever is currently in the shm block to X-Plane's head-pose
   datarefs every flight-loop tick.

## Plugin commands

The plugin registers three X-Plane commands that you can bind to joystick
buttons or keys from **Settings → Joystick** or **Settings → Keyboard**:

| Command                         | Description                                                                                       |
| ------------------------------- | ------------------------------------------------------------------------------------------------- |
| `opentrack/toggle`              | Enable/disable the flight-loop callback. When off, no data is sent to X-Plane's datarefs.         |
| `opentrack/toggle_translation`  | Enable/disable the translation part only (yaw/pitch/roll remain active). Re-centers on re-enable. |
| `opentrack/center`              | Ask opentrack to re-center the head pose. Goes the other direction over the same shm — see below. |

All three commands operate on `xplm_CommandBegin`, so they fire on button
press (not release).

## Data conversions

The plugin reads six `double`s from the shm and writes to X-Plane:

| shm index | Semantic   | Dataref                                   | Conversion          |
| --------- | ---------- | ----------------------------------------- | ------------------- |
| 0 (TX)    | x (mm)     | `sim/graphics/view/pilots_head_x`         | `mm × 1e-3 + offset_x` |
| 1 (TY)    | y (mm)     | `sim/graphics/view/pilots_head_y`         | `mm × 1e-3 + offset_y` |
| 2 (TZ)    | z (mm)     | `sim/graphics/view/pilots_head_z`         | `mm × 1e-3 + offset_z` |
| 3 (Yaw)   | yaw (rad)  | `sim/graphics/view/pilots_head_psi`       | `rad × 180/π`       |
| 4 (Pitch) | pitch (rad)| `sim/graphics/view/pilots_head_the`       | `rad × 180/π`       |
| 5 (Roll)  | roll (rad) | `sim/graphics/view/pilots_head_phi`       | `rad × 180/π`       |

Translation offsets (`offset_{x,y,z}`) are captured at plugin start and
whenever translation is re-enabled; this lets the user move around their
sim cockpit from a home position that matches the current seat pose.

## Shared-memory layout

Defined in both `x-plane-plugin/plugin.c` and `proto-wine/wine-shm.h`:

```c
typedef struct WineSHM {
    double        data[6];    // x,y,z,yaw,pitch,roll (see conversions)
    int           gameid, gameid2;
    unsigned char table[8];
    bool          stop;
    int           center_seq; // X-Plane → opentrack re-center request, see below
} volatile WineSHM;
```

The X-Plane plugin reads `data[]` and writes `center_seq`; the other
fields are used by the Wine freetrack flavor of proto-wine and are
ignored here.

Access is guarded with a POSIX `flock(fd, LOCK_SH)` around the read and
`LOCK_EX` around the `center_seq` bump — cheap, contention-free for
single-writer/single-reader.

`center_seq` is appended at the **end** of the struct so old binaries
on either side stay backward-compatible: an old `opentrack.xpl` against
new opentrack simply doesn't bump it (opentrack reads zero and treats
that as "no center request pending"). New `opentrack.xpl` against old
opentrack writes one `int` past where the older struct ended — the SHM
segment must be sized for the new struct, which the new `proto-wine`
takes care of. Recommendation: rebuild both halves together.

## Plugin lifecycle

The plugin implements the standard X-Plane SDK lifecycle. Each callback
owns specific resources, and teardown happens in reverse order so
`Plugin Admin → Reload Plugins` is safe (no dangling function pointers
inside the unloaded dylib):

| Callback              | Owns                                                | Cleaned up in        |
| --------------------- | --------------------------------------------------- | -------------------- |
| `XPluginStart`        | datarefs, commands, command handlers, SHM mapping   | `XPluginStop`        |
| `XPluginEnable`       | flight-loop callback registration                   | `XPluginDisable`     |
| `XPluginReceiveMessage` | re-centers offset on AIRPORT_LOADED + PLANE_LOADED | n/a (event-driven)   |

`XPluginStop` unregisters the flight-loop callback and both command
handlers before freeing the SHM, so reload doesn't leave X-Plane
holding pointers into the unloaded dylib.

Failure paths in `shm_wrapper_init` (malloc / shm_open / ftruncate /
mmap) are checked individually and log the failing syscall with
`strerror(errno)` to X-Plane's `Log.txt` for diagnosis. On any failure
`XPluginStart` returns 0 and X-Plane unloads the plugin cleanly.

## Re-centering from in-sim controls

The `opentrack/center` command is a back-channel that lets an X-Plane
button (yoke, throttle quadrant, panel switch, anything bindable in
**Settings → Joystick**) ask opentrack to re-center, with the same
effect as clicking opentrack's **Center** button or pressing its
keyboard hotkey.

```
┌─────────────────────────┐                  ┌──────────────────────────┐
│ opentrack (proto-wine)  │                  │ X-Plane (opentrack.xpl)  │
│                         │                  │                          │
│ writes pose ──────────► │ POSIX SHM        │ ◄─ reads pose, drives    │
│                         │ facetracknoir-   │    pilot view datarefs   │
│ reads center_seq    ◄── │ wine-shm         │ ── writes center_seq     │
│ rising-edge detect      │                  │    (CenterHandler)       │
│ → set_center(true)      │                  │                          │
└─────────────────────────┘                  └──────────────────────────┘
```

### Why this exists (macOS in particular)

opentrack's global hotkey on macOS uses Carbon `RegisterEventHotKey`,
which only sees **keyboard** events. When X-Plane has exclusive focus
(especially fullscreen) it consumes function keys before the hotkey
registration sees them, so keyboard-mapper workarounds (e.g. yoke
button → F13 → opentrack Center shortcut) silently fail mid-flight.
The `opentrack/center` command sidesteps the keyboard path entirely:
the X-Plane plugin atomically bumps `center_seq` in shared memory,
opentrack's `proto-wine` reads it on the other side and fires
`set_center(true)` exactly once per increment — same code path the
keyboard hotkey would have triggered. Mash-button presses are coalesced
(multiple bumps between polls = one re-center, not one per press).

### Binding it (Honeycomb Alpha example)

1. In X-Plane: **Settings → Joystick**.
2. Pick your Honeycomb Alpha (or any HID controller).
3. Click the button you want (e.g. the white "Reset View" button on
   the yoke).
4. In the assignment dialog, search for `opentrack/center`.
5. Pick it. Save.

That button now re-centers opentrack regardless of focus state,
fullscreen mode, or what other apps are listening for keyboard events.

### Implementation pieces

- `api/plugin-api.hpp` — `virtual bool IProtocol::center_requested()`
  (default returns false).
- `logic/pipeline.cpp` — polls `center_requested()` after each pose;
  calls `set_center(true)` on rising edge.
- `proto-wine/wine-shm.h` — `int center_seq` field, appended at end of
  `WineSHM` (backward-compatible, see SHM layout above).
- `proto-wine/ftnoir_protocol_wine.{h,cpp}` — `center_requested()`
  override with edge-detection (returns true exactly once per
  `center_seq` increment).
- `x-plane-plugin/plugin.c` — `CenterHandler` callback bumps
  `shm_posix->center_seq` atomically inside the plugin's `flock`'d
  critical section.

### Why `CenterHandler` doesn't call `reinit_offset()`

An earlier prototype called `reinit_offset()` from `CenterHandler` to
wipe the X-Plane translation offset alongside the re-center. That
regression accumulated the pre-center pose into the X-Plane offset,
causing visible Y-axis drift after each re-center. Removed: opentrack's
`set_center` on the next pose tick is the single source of truth for
"where is forward" — let it own that responsibility.

### See also

- [opentrack#2174](https://github.com/opentrack/opentrack/pull/2174) —
  original upstream PR (closed pending end-to-end verification on
  macOS).
- [msupino#3](https://github.com/msupino/opentrack/pull/3) — fork PR
  tracking this work while parked.

## Limitations and notes

- **No UDP fallback in the plugin.** If you want to drive X-Plane from a
  different machine, use opentrack's **UDP over network** output and a
  different plugin on the X-Plane side. This plugin is local-only.
- **Head roll can affect the field-of-view dataref on some X-Plane
  builds.** The commented-out `field_of_view_roll_deg` line at the top of
  `XPluginStart` shows the older binding. If you dislike roll acting on
  FoV on your X-Plane version, swap the two lines and rebuild.
- **Plugin is not codesigned.** On macOS, X-Plane may warn about loading
  an unsigned plugin. Either accept the warning or ad-hoc-sign with
  `codesign --force -s - opentrack.xpl` before installing.
- **`volatile WineSHM`** is mostly cosmetic — it prevents within-thread
  compiler reordering but doesn't provide cross-process synchronization.
  The actual safety comes from the surrounding `flock(LOCK_SH)` /
  `LOCK_EX` pair, which the kernel orders with implicit memory barriers.

## Building with a different SDK version

The minimum API version is set in `x-plane-plugin/CMakeLists.txt`:

```cmake
-DXPLM200 -DXPLM210
```

That targets X-Plane 10.00 and newer. If you need features from a newer
SDK (e.g. `XPLM400` for X-Plane 12.04+ datarefs), add the corresponding
definition; see the SDK's
[version-define documentation](https://developer.x-plane.com/sdk/plugin-sdk-documents/)
for the full ladder. No new defines are needed for pose tracking as
used here.
