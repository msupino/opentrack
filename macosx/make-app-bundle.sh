#!/bin/sh

# exit when any command fails
set -e

# keep track of the last executed command
trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
# echo an error message before exiting
trap 'echo "--\n--\n--\n--\n\"${last_command}\" command failed with exit code $?."' EXIT 

APPNAME=opentrack
# Alternative we could look at https://github.com/arl/macdeployqtfix ??

#macosx directory
dir="$1" 
test -n "$dir" 
# install directory
install="$2"
test -n "$install" 
version="$3"
test -n "$version" 

tmp="$(mktemp -d "/tmp/$APPNAME-tmp.XXXXXXX")"
test $? -eq 0 


# Add rpath for application so it can find the libraries
#install_name_tool -add_rpath @executable_path/../Frameworks "$install/$APPNAME.app/Contents/MacOS/$APPNAME"

# Copy our own plist and set correct version
cp "$dir/Info.plist" "$install/$APPNAME.app/Contents/"
sed -i '' -e "s#@OPENTRACK-VERSION@#$version#g" "$install/$APPNAME.app/Contents/Info.plist" 

# Copy PkgInfo
cp "$dir/PkgInfo" "$install/$APPNAME.app/Contents/"

# Copy plugins
mkdir -p "$install/$APPNAME.app/Contents/MacOS/Plugins"
cp -r "$install/Plugins" "$install/$APPNAME.app/Contents/MacOS/"

# Strip presets/README.txt from the plugins tree before macdeployqt's
# codesign pass. macdeployqt invokes `codesign --deep --force` on the
# whole bundle, which refuses to sign non-Mach-O files placed inside
# code-signed subdirectories ("In subcomponent: .../presets/README.txt").
# The abort happens late in macdeployqt, AFTER it has copied the Qt
# frameworks but BEFORE it copies the QPA platform plugins, leaving an
# unbootable bundle that segfaults in QGuiApplication on launch with
# `Could not find the Qt platform plugin "cocoa"`.
# README.txt has no functional purpose at runtime; remove it so
# macdeployqt's signing pass succeeds and we get a complete bundle.
rm -f "$install/$APPNAME.app/Contents/MacOS/Plugins/presets/README.txt"
rmdir "$install/$APPNAME.app/Contents/MacOS/Plugins/presets" 2>/dev/null || true

# Use either of these, two of them at the same time will break things!
macdeployqt "$install/$APPNAME.app" -libpath="$install/Library"
#sh "$dir/install-fail-tool" "$install/$APPNAME.app/Contents/Frameworks"

# macdeployqt only walks dependencies of the main executable; sibling
# helper libraries placed under Contents/Frameworks (e.g. opentrack-api,
# opentrack-compat, opentrack-options, opentrack-logic, opentrack-input,
# opentrack-user-interface, opentrack-pose-widget, opentrack-spline,
# opentrack-qxt-mini, opentrack-migration, opentrack-video) are not
# recursively rewritten. They retain LC_RPATH entries pointing back at
# `/opt/homebrew/opt/qtbase/lib` (or `/opt/homebrew/opt/qtmultimedia/lib`,
# etc.) inherited from their build-time link line.
#
# That leak is harmless on the original build machine -- until @rpath
# resolution finds Homebrew's QtCore BEFORE the bundled one, loading
# both QtCores into the same process. Symptom at startup:
#   objc[]: Class QT_ROOT_LEVEL_POOL ... is implemented in both
#           /opt/homebrew/Cellar/qtbase/.../QtCore
#       and /<bundle>/Contents/Frameworks/QtCore.framework/.../QtCore
#   This may cause spurious casting failures and mysterious crashes.
# followed by a hard abort. Same root cause as the libomp double-init
# error from tracker-neuralnet's stale `/opt/homebrew/lib` rpath.
#
# Walk every Mach-O in the bundle and delete any LC_RPATH that points
# outside the bundle (Homebrew Cellar/Opt, /usr/local for Intel-rosetta
# brew, or our own install prefix from CMAKE_INSTALL_PREFIX). Add
# @loader_path so sibling-lib lookups (e.g. @rpath/opentrack-api.dylib
# inside opentrack-options.dylib) still resolve to the same Frameworks
# directory.
echo "make-app-bundle: stripping stale rpaths from bundled Mach-O files"
find "$install/$APPNAME.app" -type f \( -name "*.dylib" -o -name "$APPNAME" \
        -o -name "*.framework" -prune -false \) -print 2>/dev/null | \
while read f; do
    # Only act on Mach-O files (skip text/scripts that find may surface)
    file "$f" 2>/dev/null | grep -q "Mach-O" || continue
    otool -l "$f" 2>/dev/null | awk '
        /^Load command/ {cmd_lc=$0}
        /cmd LC_RPATH/  {in_rpath=1}
        in_rpath && /path / { print $2; in_rpath=0 }
    ' | grep -E "^(/opt/homebrew|/usr/local|$install)" | while read bad; do
        install_name_tool -delete_rpath "$bad" "$f" 2>/dev/null || true
    done
    # Ensure libraries can find their siblings via @loader_path. Idempotent
    # because install_name_tool refuses duplicate -add_rpath silently here.
    case "$f" in
        */Contents/Frameworks/*.dylib)
            install_name_tool -add_rpath "@loader_path" "$f" 2>/dev/null || true ;;
    esac
done

# Bundle codesign was potentially invalidated by the install_name_tool
# rewrites above (every modified Mach-O loses its prior signature). Re-sign
# deeply with the local opentrack-dev certificate if one is available;
# otherwise ad-hoc-sign as a fallback so the bundle at least passes
# Gatekeeper's launch check on the same machine. Without this step the
# kernel sends SIGKILL on launch with "Code Signature Invalid /
# Invalid Page" before any Qt code runs.
if security find-certificate -c opentrack-dev >/dev/null 2>&1; then
    codesign --force --deep --sign opentrack-dev "$install/$APPNAME.app"
else
    echo "make-app-bundle: WARNING no 'opentrack-dev' cert found; ad-hoc signing"
    codesign --force --deep --sign - "$install/$APPNAME.app"
fi

# Build the .icns iconset and (optionally) the DMG. Both steps are
# nice-to-have packaging niceties, NOT functional requirements for
# the .app bundle to launch. They depend on tools that aren't part of
# the macOS toolchain by default:
#   * `convert`     -- ImageMagick (`brew install imagemagick`)
#   * `create-dmg`  -- (`brew install create-dmg`)
# If either is missing, skip its step with a warning rather than aborting
# the whole install. The .app remains usable for launch from Finder /
# `open`; only the polished icon and DMG distribution artifact are absent.
if command -v convert >/dev/null 2>&1; then
    # Create an 512 resolution size for the icon (for retina displays mostly)
    #gm convert -size 512x512 "$dir/../gui/images/opentrack.png" "$tmp/opentrack.png"
    convert "$dir/../gui/images/opentrack.png" -filter triangle -resize 512x512 "$tmp/opentrack.png"

    # Build iconset
    mkdir "$tmp/$APPNAME.iconset"
    sips -z 16 16     "$tmp/opentrack.png" --out "$tmp/$APPNAME.iconset/icon_16x16.png"
    sips -z 32 32     "$tmp/opentrack.png" --out "$tmp/$APPNAME.iconset/icon_16x16@2x.png"
    sips -z 32 32     "$tmp/opentrack.png" --out "$tmp/$APPNAME.iconset/icon_32x32.png"
    sips -z 64 64     "$tmp/opentrack.png" --out "$tmp/$APPNAME.iconset/icon_32x32@2x.png"
    sips -z 128 128   "$tmp/opentrack.png" --out "$tmp/$APPNAME.iconset/icon_128x128.png"
    sips -z 256 256   "$tmp/opentrack.png" --out "$tmp/$APPNAME.iconset/icon_128x128@2x.png"
    sips -z 512 512   "$tmp/opentrack.png" --out "$tmp/$APPNAME.iconset/icon_256x256@2x.png"
    sips -z 512 512   "$tmp/opentrack.png" --out "$tmp/$APPNAME.iconset/icon_512x512.png"
    iconutil -c icns -o "$install/$APPNAME.app/Contents/Resources/$APPNAME.icns" "$tmp/$APPNAME.iconset"
else
    echo "make-app-bundle: WARNING ImageMagick 'convert' not found -- skipping icon"
    echo "  install via 'brew install imagemagick' to get the polished bundle icon"
fi
rm -rf "$tmp"

if command -v create-dmg >/dev/null 2>&1; then
    #Build DMG
    #https://github.com/andreyvit/create-dmg
    rm -rf $install/../*.dmg
    create-dmg \
      --volname "$APPNAME" \
      --volicon "$install/$APPNAME.app/Contents/Resources/$APPNAME.icns" \
      --window-pos 200 120 \
      --window-size 800 450 \
      --icon-size 80 \
      --background "$dir/dmgbackground.png" \
      --icon "$APPNAME.app" 200 180 \
      --app-drop-link 420 180 \
      --hide-extension "$APPNAME.app" \
      --no-internet-enable \
      --add-folder "Document" "$install/doc" 20 40 \
      --add-folder "Xplane-Plugin" "$install/xplane" 420 40 \
      --add-folder "thirdparty" "$install/thirdparty" 620 40 \
      "$version.dmg" \
      "$install/$APPNAME.app"

    # Check if we have a DMG otherwise fail
    FILE=$install/../$version.dmg
    if [ -f $FILE ]; then
       ls -ial $install/../*.dmg
    else
       echo "Failed to create ${FILE}"
       exit 2
    fi
else
    echo "make-app-bundle: WARNING create-dmg not found -- skipping DMG"
    echo "  install via 'brew install create-dmg' to package the bundle for distribution"
    echo "  the .app at $install/$APPNAME.app is complete and launchable"
fi
