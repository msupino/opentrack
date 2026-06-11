---
name: opentrack-build
description: >-
  Build, install, run, and ship the opentrack codebase on macOS. Covers
  prerequisites (Homebrew qt/opencv/cmake/pkg-config/libusb), first-time
  cmake configure, the dev/hot-install.sh inner-loop, the full
  make install + macdeployqt + make-app-bundle.sh path, the app/dock
  icon pipeline, running opentrack with stderr capture, macOS TCC
  permissions, the Apple-Silicon CMakeCache /usr/local gotcha, the Qt
  lupdate / lang/*.ts translation workflow, and the upstream-PR
  splitting convention. Use when the user mentions building/running
  opentrack, hot-install.sh, macdeployqt, make install, cmake configure,
  the bundle layout, opentrack.app rebuilds, Qt linking/double-load
  issues, the dock/app icon or opentrack.icns, Input Monitoring/Camera
  permissions, lupdate/lang/*.ts, or upstream PRs to opentrack/opentrack.
---

# opentrack — build, install, run, package (macOS)

Uses Qt6 (Homebrew), CMake, OpenCV. Verified on Apple Silicon
(`/opt/homebrew`); Intel (`/usr/local`) is analogous. See the sibling
skills `opentrack-osx-issues` and `opentrack-psvr-issues`, and the
repo-root `ARCHITECTURE.md`.

## Prerequisites

```bash
brew install qt opencv cmake pkg-config libusb
dev/setup-signing-cert.sh     # local "opentrack-dev" cert for hot-install
```

- **qt** → Qt6 (found at `/opt/homebrew/opt/qtbase/...`)
- **opencv** → camera trackers (psvr, pt, easy, aruco)
- **libusb** → only tracker-psvr's PS4-Camera firmware uploader needs it

### Apple-Silicon gotcha: stale /usr/local paths in CMakeCache
A stale `build/CMakeCache.txt` can hard-code Intel tool paths and break
the build:
```
make: /usr/local/bin/cmake: No such file or directory
Could NOT find PkgConfig (missing: PKG_CONFIG_EXECUTABLE)
```
Fix (one-time): symlink the tools where the cache expects them, or
reconfigure with explicit paths:
```bash
ln -sf /opt/homebrew/bin/cmake      /usr/local/bin/cmake
ln -sf /opt/homebrew/bin/pkg-config /usr/local/bin/pkg-config
# or: cmake -S . -B build -DPKG_CONFIG_EXECUTABLE=/opt/homebrew/bin/pkg-config
```

## First build (configure + compile + install)

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=RELEASE \
      -DCMAKE_INSTALL_PREFIX="$PWD/install"
cmake --build build -j8
cmake --install build          # populates install/opentrack.app
```
Generator is Unix Makefiles (so `dev/hot-install.sh` can drive `make`).

## Inner-loop build: `dev/hot-install.sh`

**Default workflow for code changes.** Rebuild → hot-copy into the
bundle → re-sign → relaunch in a few seconds.

```bash
dev/hot-install.sh                          # default: tracker-psvr proto-wine
dev/hot-install.sh tracker-psvr             # one target
dev/hot-install.sh --exe [targets...]       # also reinstall opentrack.app binary
dev/hot-install.sh --no-launch              # skip the final `open`
```

Why this exists: `make <target>` alone produces a dylib still linked
against `/opt/homebrew/opt/qtbase/lib`, so dropping it into the bundle
loads Qt twice and aborts with `QPixmap: Must construct a
QGuiApplication…` or duplicate-class warnings. `hot-install.sh` does
the minimal `install_name_tool` surgery + re-sign without invoking
`macdeployqt`.

**When NOT to use hot-install:**
- After pulling commits that touch `CMakeLists.txt` or add new files → full `make` first.
- After `brew upgrade qt` → reconfigure, the old bundle libs won't find new Cellar paths.
- For a release-quality bundle → use the proper `make install` + `macdeployqt` path.

## Full-bundle build (`make install` + `macosx/make-app-bundle.sh`)

Slow (~10 min on a clean install). `make-app-bundle.sh` runs
`macdeployqt` (copies Qt frameworks in), generates the `.icns` from
`gui/images/opentrack.png`, and builds a DMG. `macdeployqt` leaves
stale rpaths and trips on `presets/README.txt` during deep
code-signing — see the `opentrack-osx-issues` skill for the aftermath
fixes.

## App / dock icon

- Source of truth: `gui/images/opentrack.png` (the pink octopus logo).
- The macOS `.icns` is generated **at package time** by
  `make-app-bundle.sh`; no `.icns` is committed. `macosx/Info.plist`
  sets `CFBundleIconFile = opentrack.icns`.
- **`dev/hot-install.sh` does NOT generate the icon**, so a dev bundle
  shows the generic white placeholder in the dock. Cosmetic. To install
  one manually from a square PNG:
```bash
mkdir /tmp/o.iconset
for s in 16 32 128 256 512; do
  sips -z $s $s SRC.png --out /tmp/o.iconset/icon_${s}x${s}.png
  sips -z $((s*2)) $((s*2)) SRC.png --out /tmp/o.iconset/icon_${s}x${s}@2x.png
done
iconutil -c icns /tmp/o.iconset -o install/opentrack.app/Contents/Resources/opentrack.icns
codesign --force --deep --sign - install/opentrack.app && killall Dock
```

## Running opentrack

```bash
open install/opentrack.app                 # normal

# stderr-capturing launch (REQUIRED to see plugin diagnostics like the
# PSVR [psvr-cam] logs — Finder/`open` sends stderr to /dev/null):
pkill -x opentrack
nohup install/opentrack.app/Contents/MacOS/opentrack >/tmp/opentrack.log 2>&1 &
```

macOS permissions (System Settings → Privacy & Security), toggle
off/on if stale after a rebuild:
- **Input Monitoring** → PSVR USB-HID reads
- **Camera** → any webcam tracker (psvr camera, pt, easy)
- **Screen Recording** → only the PSVR display mirror

## Build assumptions

- In-tree build at repo root, **or** out-of-source `./build/`
  (auto-detected by `hot-install.sh`).
- Install prefix is `<repo>/install`.
- Generator emits a Makefile (`Unix Makefiles`).
- Qt comes from Homebrew (`brew --prefix qtbase` is consulted at
  runtime; works on both Apple-Silicon `/opt/homebrew` and Intel
  `/usr/local` layouts).
- Local code-signing cert is set up via `dev/setup-signing-cert.sh`.

## Common one-liners

```bash
# fast dev cycle on one plugin change
dev/hot-install.sh tracker-psvr

# rebuild + reinstall the main exe too
dev/hot-install.sh --exe tracker-psvr

# inspect a plugin dylib's actual link paths (debug double-Qt issues)
otool -L install/opentrack.app/Contents/MacOS/Plugins/libopentrack-tracker-psvr.dylib

# verify rpaths are clean post-deploy
otool -l install/opentrack.app/Contents/MacOS/Plugins/libopentrack-tracker-psvr.dylib | grep -A2 LC_RPATH
```

## Translations: `lang/*.ts` files

The `.ts` files in each module's `lang/` directory are **`lupdate`
output** — hand-editing is pointless, they regenerate every Qt build.

Workflow:

1. Add or change a translatable string (`tr(...)`, `QT_TR_NOOP`, etc.) in `.cpp`/`.h`.
2. The Qt build's `lupdate` pass updates the matching `lang/*.ts`.
3. Commit the regenerated `.ts` files in their own commit. Suggested
   message: `lang: regenerate .ts files for recent X updates`.

**Gotcha — don't `git add lang/*.ts` with a shell glob.** Some modules
keep certain locales (e.g. `de_DE.ts` in some `proto-*` modules)
git-ignored. The shell glob expands to include them; `git add` sees an
ignored path and refuses the **whole** add. Two safe alternatives:

```bash
git add -u 'lang/*.ts'           # only already-tracked files
git add path/to/specific.ts ...  # list every modified file explicitly
```

## Upstream PR splitting convention

Large branches get split into focused, single-purpose feature branches
off `origin/master`. Each becomes its own upstream PR; reviewers can
land them independently and an umbrella branch stays on the fork until
its prerequisites land.

Pattern: `git checkout -b <feature> origin/master && git cherry-pick
<sha-range>` then push to the fork remote and open the PR.

Don't try to land everything in one branch. Don't force-push to an
upstream remote. Always make a `<branch>-pre-<event>` safety branch
before rewriting >1 commit on a branch you may want to recover.
