#!/bin/bash
# Renders the five demo scenes with the app's real drawing code on the desktop JVM (software canvas path, Java2D shims).
#   desktop/render.sh            build the shims + app, build the scene, render docs/demo/*.png
# Needs: JDK 17 (openjdk@17), the android-17 android.jar and api-82.jar (ANDROID_JAR / XPOSED_JAR, as build.sh), docker with
# the oprad-test:py312 image for the scene's tile encoders (set SKIP_SCENE=1 to reuse desktop/scene/scene.json).
set -e
cd "$(dirname "$0")/.."
export PATH=/opt/homebrew/opt/openjdk@17/bin:$PATH
REPO=$(cd ../../.. && pwd)
desktop/build_shim.sh
if [ -z "$SKIP_SCENE" ]; then
  docker run --rm -v "$REPO":/w -w /w -e PYTHONPATH=/w oprad-test:py312 python tools/head_unit/aaroad/desktop/make_scene.py
fi
# the module's assets as the view reads them from its zip: civic_map.png, fonts, the turn card's arrows
M=out/desktop/module; rm -rf $M; mkdir -p $M/assets/nav
cp assets/civic_map.png $M/assets/; cp -r assets/fonts $M/assets/fonts
NAV=../../../openpilot/starpilot/ui
python3 -c 'import json,sys; print("\n".join(f["file"] for f in json.load(open(sys.argv[1]))["files"]))' $NAV/navigation-assets.json \
  | while read -r f; do cp "$NAV/assets/navigation/$f" $M/assets/nav/; done
(cd $M && jar cfM ../module.zip assets)
CP="out/desktop/render:out/desktop/app:out/desktop/shim"
S=desktop/scene/scene.json; Z=out/desktop/module.zip; D=docs/demo
r() { java -Djava.awt.headless=true $JOPT -cp "$CP" com.starpilot.aaroad.Render "$@"; }
mkdir -p $D
#   scenario   music nav  png                   scene Z  metres ahead of the maneuver
# the music card waits for the maneuver when the turn card is too tall to leave it room (MapView SONG_LAST), so this one
# uses a one-line instruction
JOPT=-Dshort=true r roundabout  1  1  $D/nav_music.png  $S $Z 110
JOPT=
r roundabout  0  1  $D/nav_only.png   $S $Z 110
r free        1  0  $D/music_only.png $S $Z
r free        0  0  $D/idle.png       $S $Z
r uturn       0  1  $D/uturn.png      $S $Z 120
