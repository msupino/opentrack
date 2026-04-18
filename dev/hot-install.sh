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
fix_install_names() {
    local dylib="$1"
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
    echo "[hot-install] re-sign bundle with stable identity '$SIGN_ID'"
else
    echo "[hot-install] ad-hoc re-sign bundle (no opentrack-dev cert; "
    echo "              run dev/setup-signing-cert.sh to make TCC grants stick)"
fi
codesign --deep --force -s "$SIGN_ID" "$BUNDLE" 2>&1 | tail -1

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
