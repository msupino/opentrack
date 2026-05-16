---
name: opentrack-build
description: >-
  Build, install, and ship the opentrack codebase on macOS. Covers the
  dev/hot-install.sh inner-loop workflow, when to use it vs. a full
  make install + macdeployqt pass, the Qt lupdate / lang/*.ts translation
  regeneration workflow (including the git-ignored-locales gotcha), and
  the upstream-PR splitting convention. Use when the user mentions
  hot-install.sh, macdeployqt, make install, the bundle layout, opentrack.app
  rebuilds, Qt linking issues during dev, lupdate, lang/*.ts files, the
  proto-iokit-foohid translations, or how to organise PRs for upstream
  opentrack/opentrack.
---

# opentrack — build, install, package

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

## Full-bundle build (`make install` + `macdeployqt`)

Slow (~10 min on a clean install). `macdeployqt` rewrites Qt links and
re-resolves the Frameworks, but leaves stale rpaths and trips on
`presets/README.txt` during deep code-signing — see the
`opentrack-osx-issues` skill for the macdeployqt aftermath fixes.

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
