#!/bin/bash
# Builds the replay: Replay.java, the app at HEAD (out/desktop/app, via build_shim.sh) and the app at OLD_REV
# (default 052c86a86a, the parent of the overpass fix d43431dd40) into out/desktop/app_old, from git, no checkout.
set -e
cd "$(dirname "$0")/../.."
export PATH=/opt/homebrew/opt/openjdk@17/bin:$PATH
AJ=${ANDROID_JAR:-/tmp/hu/sdk/platforms/android-17/android.jar}
XP=${XPOSED_JAR:-/tmp/hu/api-82.jar}
OLD=${OLD_REV:-052c86a86a}
desktop/build_shim.sh
rm -rf out/desktop/old_src out/desktop/app_old out/desktop/replay
mkdir -p out/desktop/old_src out/desktop/app_old out/desktop/replay
git archive "$OLD" src | tar -x -C out/desktop/old_src
javac -nowarn -source 8 -target 8 -cp "out/desktop/shim:$AJ:$XP" -d out/desktop/app_old $(find out/desktop/old_src -name '*.java') 2>&1 \
  | grep -v "^warning\|bootstrap\|^[0-9]* warning\|^Note:" || true
javac -nowarn -cp "out/desktop/app:out/desktop/shim:$AJ:$XP" -d out/desktop/replay $(find desktop/replay -name '*.java')
echo "built: HEAD=$(git rev-parse --short HEAD) OLD=$OLD"
