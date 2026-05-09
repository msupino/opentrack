# Developer tooling

Scripts and small utilities used during opentrack and PSVR plugin development on macOS. None of these ship in the user-facing build; they exist only to make the inner-loop fast.

## `hot-install.sh`

**Fast incremental rebuild → hot-copy into the bundle → re-sign → relaunch.** The default workflow when you've made a code change to one or two plugins and want to see the result.

```bash
dev/hot-install.sh                          # default: tracker-psvr proto-wine
dev/hot-install.sh tracker-psvr             # one target
dev/hot-install.sh tracker-psvr proto-wine  # specific targets
dev/hot-install.sh --exe [targets...]       # also rebuild + reinstall the main opentrack binary
dev/hot-install.sh --no-launch              # skip the final `open`
dev/hot-install.sh -h                       # full help
```

### Why this exists

A naive incremental `make <target>` produces a dylib whose Qt links and rpath still point at the Homebrew Cellar (e.g. `/opt/homebrew/opt/qtbase/lib`). Without macdeployqt's post-install pass, copying that raw dylib into the bundle causes dyld to load a **second** copy of Qt at runtime, duplicating Qt's classes and aborting with messages like:

```
QPixmap: Must construct a QGuiApplication before a QPixmap
objc[..]: Class QT_ROOT_LEVEL_POOL... is implemented in both ...
```

Going through `make install` would work but invokes macdeployqt — slow (~10 minutes on a fresh install) and trips on a codesign issue with `presets/README.txt`. This script does the minimum surgery needed for an incremental cycle: it never reconfigures cmake, never reruns macdeployqt, and finishes in a few seconds (~5 s for one plugin, ~15 s with `--exe`).

### Assumptions

- Build is in-tree at the repo root (`CMAKE_BINARY_DIR == repo root`) — auto-detects out-of-source `./build/` if present.
- Install prefix is `<repo>/install`.
- Generator emits a Makefile (`Unix Makefiles`).
- Qt comes from Homebrew (works on both Apple Silicon `/opt/homebrew` and Intel `/usr/local` layouts; `brew --prefix` is consulted at runtime).
- Local code-signing cert is set up — see `setup-signing-cert.sh` below.

### When NOT to use it

- After pulling new commits that touch CMakeLists or add new files → do a full `make` (or clean build).
- After upgrading Homebrew Qt → reconfigure first, otherwise the rebuilt dylibs will link against the new cellar path that the bundle's other libs don't yet know about.
- For a fresh clone or a release-quality bundle → use `cmake --install` once, then iterate with this script.

## `psvr-poweron.c` / `psvr-poweron`

**Standalone CLI that wakes a sleeping PSVR.** Sends the activation HID command sequence (`0x17 EnableSensors`, `0x11 EnableTracking` with the magic `0xFFFFFF00` payload, `0x23 SetVRMode`) directly over USB without launching opentrack.

Build (one-time):

```bash
clang dev/psvr-poweron.c -framework IOKit -framework CoreFoundation -o dev/psvr-poweron
```

Run:

```bash
sudo dev/psvr-poweron     # may need sudo on first invocation for raw HID access
```

Useful when:
- You need the PSVR alive but don't want to start the full opentrack tracker (e.g. during HDMI handshake debugging).
- The PSVR has gone to sleep mid-session and the in-app stall recovery hasn't kicked in.
- You're testing the activation sequence itself — modify the script to try other HID command bytes (community-documented but unconfirmed: `0x1A`, `0x1F`, `0x40`, `0xA0`).

## `setup-signing-cert.sh`

**One-time setup of a local code-signing certificate** that `hot-install.sh` uses to re-sign rebuilt dylibs. macOS Gatekeeper rejects unsigned dylibs loaded into a previously-signed bundle, so each surgical replace needs a quick ad-hoc resign.

Run once per dev machine:

```bash
dev/setup-signing-cert.sh
```

Creates a self-signed `opentrack-dev` cert in your login keychain. `hot-install.sh` then uses `codesign -s opentrack-dev` for each replaced dylib. Idempotent — safe to re-run; it skips if the cert already exists.

## Typical inner-loop workflow

1. **First time** (or after pulling fresh): full clean build →
   ```bash
   cmake -S . -B build -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release
   cmake --build build -j
   cmake --install build
   ```

2. **Each iteration** on a plugin:
   ```bash
   # edit tracker-psvr/psvr.cpp
   dev/hot-install.sh tracker-psvr
   # opentrack restarts with your changes; ~5 seconds
   ```

3. **PSVR not waking up** during testing →
   ```bash
   sudo dev/psvr-poweron
   ```

4. **After upgrading deps or pulling commits that touch CMakeLists** → back to step 1.
