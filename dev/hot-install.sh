#!/usr/bin/env bash
# hot-install.sh - rebuild one or more opentrack modules and hot-copy
# their dylibs into the installed .app bundle at <repo>/install/opentrack.app,
# fix install-names, strip bad rpaths, ad-hoc re-sign, and relaunch.
#
# Why this exists
# ---------------
# An incremental `make <target>` emits a dylib whose Qt links and rpath
# still point at the Homebrew Cellar (e.g. /opt/homebrew/opt/qtbase/lib).
# Without macdeployqt's post-install pass, copying that raw dylib into
# the bundle causes dyld to load a SECOND copy of Qt at runtime,
# duplicating Qt's classes and aborting with messages like:
#
#     QPixmap: Must construct a QGuiApplication before a QPixmap
#     objc[..]: Class QT_ROOT_LEVEL_POOL... is implemented in both ...
#
# Going through `make install` would work but invokes macdeployqt which
# is slow (~10 minutes on a fresh install) and trips on a codesign issue
# with presets/README.txt. This script does the minimum surgery needed
# for an incremental cycle: it never reconfigures cmake, never reruns
# macdeployqt, and finishes in a few seconds.
#
# Assumptions
# -----------
#  - Build is in-tree (CMAKE_BINARY_DIR == repo root).
#  - Install prefix is <repo>/install.
#  - Generator emits a Makefile (Unix Makefiles).
#  - Qt comes from Homebrew (works on both Apple Silicon /opt/homebrew
#    and Intel /usr/local layouts; `brew --prefix` is consulted at run
#    time to decide).
#
# Usage
# -----
#   dev/hot-install.sh                          # default: tracker-psvr proto-wine
#   dev/hot-install.sh tracker-psvr             # one target
#   dev/hot-install.sh tracker-psvr proto-wine  # specific targets
#   dev/hot-install.sh --exe [targets...]       # also rebuild + reinstall
#                                               # the main opentrack binary
#   dev/hot-install.sh --no-launch              # skip the final `open`
#   dev/hot-install.sh -h | --help              # show this header

set -euo pipefail

# --- locate things --------------------------------------------------------
REPO="$(cd "$(dirname "$0")/.." && pwd)"

# Auto-detect build layout: prefer an out-of-source tree at ./build/
# (upstream-compatible), fall back to an in-source build in ./ if the
# project was configured in-source historically. New clones should
# use `cmake -S . -B build` and never touch this fallback.
if [[ -f "$REPO/build/Makefile" ]]; then
    BUILD="$REPO/build"
elif [[ -f "$REPO/Makefile" ]]; then
    BUILD="$REPO"
else
    echo "no Makefile at \$REPO/build or \$REPO; run \`cmake -S . -B build\` first" >&2
    exit 2
fi

BUNDLE="$REPO/install/opentrack.app"
BUNDLE_PLUGINS="$BUNDLE/Contents/MacOS/Plugins"
STANDALONE_PLUGINS="$REPO/install/Plugins"
EXE_IN_BUNDLE="$BUNDLE/Contents/MacOS/opentrack"
BUILT_EXE="$BUILD/opentrack/opentrack.app/Contents/MacOS/opentrack"

[[ "$(uname -s)" == "Darwin" ]] || { echo "macOS only" >&2; exit 2; }
[[ -d "$BUNDLE" ]] || { echo "no .app at $BUNDLE — run \`make install\` once first" >&2; exit 2; }

# We deliberately do NOT trust `brew --prefix` here: machines with both
# /opt/homebrew (Apple Silicon) and /usr/local (Intel/Rosetta) Homebrew
# installs have a `brew` in $PATH that points at one prefix while the
# build was actually linked against the other. Instead, fixup is driven
# by what's actually inside each dylib: enumerate every absolute path
# that mentions a Qt framework and rewrite it to @executable_path.
# Likewise, every LC_RPATH entry that mentions /opt/homebrew or
# /usr/local is stripped; @executable_path/../Frameworks is added.

# --- arg parse ------------------------------------------------------------
MODULES=()
REBUILD_EXE=0
RELAUNCH=1
for arg in "$@"; do
    case "$arg" in
        --exe)        REBUILD_EXE=1 ;;
        --no-launch)  RELAUNCH=0 ;;
        --help|-h)    sed -n '1,/^set -euo/p' "$0" | sed -n '2,/^$/p'; exit 0 ;;
        -*)           echo "unknown flag: $arg" >&2; exit 2 ;;
        *)            MODULES+=("$arg") ;;
    esac
done

# Default targets are the ones with stale-rpath problems in this tree.
# Extend if you add a new plugin that builds against Qt and gets affected.
if [[ ${#MODULES[@]} -eq 0 ]]; then
    MODULES=(tracker-psvr proto-wine)
fi

cd "$BUILD"

# --- helpers --------------------------------------------------------------
cmake_target() { echo "opentrack-$1"; }
dylib_for()    { echo "$BUILD/$1/opentrack-$1.dylib"; }

# Rewrite every absolute Qt-framework load command in the dylib to use
# @executable_path/../Frameworks/.  Driven by `otool -L`, so it works
# regardless of which Homebrew prefix the build linked against.
# Also rewrite bare-dylib references for libraries that macdeployqt has
# already copied into Contents/Frameworks (opencv, libomp, openblas,
# ...). If two copies of such a library end up in the same process
# under different names (e.g. /opt/homebrew/opt/libomp/lib/libomp.dylib
# from a freshly-built plugin vs @executable_path/../Frameworks/libomp
# .dylib from the bundled OpenBLAS), OMP's self-init check aborts with
# "OMP: Error #15: Initializing libomp.dylib, but found libomp.dylib
# already initialized." - same class of duplicate-runtime bug Qt would
# trip on.
fix_install_names() {
    local dylib="$1"
    # Qt frameworks
    while read -r path; do
        local fw
        fw="$(basename "$(dirname "$(dirname "$(dirname "$path")")")" .framework)"
        install_name_tool -change "$path" \
            "@executable_path/../Frameworks/${fw}.framework/Versions/A/${fw}" \
            "$dylib" 2>/dev/null || true
    done < <(
        otool -L "$dylib" 2>/dev/null \
        | awk '/Qt[A-Z][a-zA-Z]*\.framework/ && ($1 ~ /^\/(opt|usr)\/(homebrew|local)/) { print $1 }'
    )
    # Bare dylibs already present inside Contents/Frameworks. We match
    # /opt/homebrew or /usr/local absolute paths and redirect to the
    # bundled basename if it exists.
    while read -r path; do
        local base
        base="$(basename "$path")"
        if [[ -f "$BUNDLE/Contents/Frameworks/$base" ]]; then
            install_name_tool -change "$path" \
                "@executable_path/../Frameworks/$base" \
                "$dylib" 2>/dev/null || true
        fi
    done < <(
        otool -L "$dylib" 2>/dev/null \
        | awk '/\.dylib/ && ($1 ~ /^\/(opt|usr)\/(homebrew|local)/) { print $1 }'
    )
}

# Delete every LC_RPATH entry that points outside the bundle into a
# Homebrew tree or into opentrack's build-tree install/Library.
# Anything starting with @executable_path or @loader_path stays.
strip_bad_rpaths() {
    local dylib="$1"
    while read -r r; do
        case "$r" in
            /opt/homebrew/*|/usr/local/*|"$REPO/install/"*)
                install_name_tool -delete_rpath "$r" "$dylib" 2>/dev/null || true
                ;;
        esac
    done < <(otool -l "$dylib" | awk '/^ *cmd LC_RPATH/{p=1;next} p && /path /{print $2; p=0}')
}

# --- plugin pass ----------------------------------------------------------
for m in "${MODULES[@]}"; do
    tgt="$(cmake_target "$m")"
    echo "[hot-install] building $tgt"
    make -j8 "$tgt" >/dev/null
    src="$(dylib_for "$m")"
    [[ -f "$src" ]] || { echo "missing $src" >&2; exit 1; }
    base="$(basename "$src")"
    echo "[hot-install] installing $base"
    cp -f "$src" "$BUNDLE_PLUGINS/$base"
    cp -f "$src" "$STANDALONE_PLUGINS/$base" 2>/dev/null || true
    fix_install_names "$BUNDLE_PLUGINS/$base"
    strip_bad_rpaths  "$BUNDLE_PLUGINS/$base"
done

# --- optional main-exe pass -----------------------------------------------
if [[ $REBUILD_EXE -eq 1 ]]; then
    echo "[hot-install] building opentrack-executable"
    make -j8 opentrack-executable >/dev/null
    echo "[hot-install] installing main exe"
    cp -f "$BUILT_EXE" "$EXE_IN_BUNDLE"
    fix_install_names "$EXE_IN_BUNDLE"
    strip_bad_rpaths  "$EXE_IN_BUNDLE"
    # The main exe needs @executable_path/../Frameworks as an rpath so
    # @rpath/Qt... references inside the bundled frameworks resolve to
    # the bundle, not the system Qt.
    install_name_tool -add_rpath '@executable_path/../Frameworks' \
        "$EXE_IN_BUNDLE" 2>/dev/null || true
fi

# --- delete stray CMake-produced skeleton bundle --------------------------
# cmake's `add_executable(... MACOSX_BUNDLE ...)` drops a hollow
# opentrack.app next to the built binary. It has no Frameworks/ dir,
# so its exe still links to /opt/homebrew/opt/qtbase/...  LaunchServices
# can easily pick it over install/opentrack.app in Spotlight / `open -a`
# / the Dock, at which point it loads BOTH homebrew Qt and the bundled
# Qt (dragged in via plugins), trips QWidgetPrivate's ABI version
# guard, and aborts at the first QWidget construction. See crash
# signature `QWidgetPrivate::QWidgetPrivate(QtPrivate_6_11_0) (.cold.1)
# → qFatal → SIGABRT`.
STRAY_BUNDLE="$BUILD/opentrack/opentrack.app"
if [[ -d "$STRAY_BUNDLE" && ! -d "$STRAY_BUNDLE/Contents/Frameworks" ]]; then
    echo "[hot-install] remove stray skeleton bundle at $STRAY_BUNDLE"
    rm -rf "$STRAY_BUNDLE"
fi

# --- relaunch -------------------------------------------------------------
echo "[hot-install] kill running opentrack"
pkill -f 'install/opentrack.app/Contents/MacOS/opentrack' 2>/dev/null || true
sleep 1

# Prefer a stable self-signed dev identity over ad-hoc when available.
# Ad-hoc signatures use a fresh random hash on every resign, which
# invalidates TCC grants (Screen Recording, Input Monitoring) every
# time. A stable identity keeps those grants persistent across
# rebuilds. See dev/setup-signing-cert.sh to create one.
SIGN_ID="-"
if security find-identity -v -p codesigning "$HOME/Library/Keychains/login.keychain-db" \
    2>/dev/null | grep -Fq '"opentrack-dev"'; then
    SIGN_ID="opentrack-dev"
else
    echo "[hot-install] ad-hoc re-sign (no opentrack-dev cert; "
    echo "              run dev/setup-signing-cert.sh to make TCC grants stick)"
fi

# Surgical re-sign: only the files we just copied in (plugins + maybe
# the main exe), not the whole bundle. `codesign --deep --force` on the
# full bundle is ~2.5 s on this tree (it recursively signs ~23 plugins
# + ~30 Qt/OpenCV dylibs + the main exe), whereas signing a single
# dylib is ~50 ms. Since we almost always iterate on ONE plugin at a
# time, the difference per edit-compile-test cycle is huge.
#
# Launch works with surgical signing because macOS code-signing is
# per-dylib: Gatekeeper validates each Mach-O object against its own
# signature, not against a parent-bundle hash manifest (that's only
# used for notarization, which a dev build doesn't need). Unsigned or
# stale-signed untouched files keep their existing (valid) signatures
# and load fine.
echo "[hot-install] re-sign ${#MODULES[@]} plugin(s) with identity '$SIGN_ID'"
for m in "${MODULES[@]}"; do
    plug="$BUNDLE_PLUGINS/opentrack-$m.dylib"
    [[ -f "$plug" ]] || continue
    codesign --force -s "$SIGN_ID" "$plug" 2>&1 | tail -1
done
if [[ $REBUILD_EXE -eq 1 ]]; then
    echo "[hot-install] re-sign main exe with identity '$SIGN_ID'"
    codesign --force -s "$SIGN_ID" "$EXE_IN_BUNDLE" 2>&1 | tail -1
fi

if [[ $RELAUNCH -eq 1 ]]; then
    echo "[hot-install] relaunch"
    open "$BUNDLE"
    sleep 2
    if pgrep -lf opentrack.app/Contents/MacOS/opentrack; then
        echo "[hot-install] OK"
    else
        echo "[hot-install] opentrack did not start - check Console.app or run the binary directly:" >&2
        echo "  $EXE_IN_BUNDLE" >&2
        exit 1
    fi
else
    echo "[hot-install] done (not relaunched)"
fi
