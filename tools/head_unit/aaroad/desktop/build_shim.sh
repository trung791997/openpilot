#!/bin/bash
# Compiles the shims, then the app's real sources against them. Output: out/desktop/{shim,app,render}.
set -e
cd "$(dirname "$0")/.."
export PATH=/opt/homebrew/opt/openjdk@17/bin:$PATH
AJ=${ANDROID_JAR:-/tmp/hu/sdk/platforms/android-17/android.jar}   # compile-time only: for classes the shims leave out
XP=${XPOSED_JAR:-/tmp/hu/api-82.jar}
rm -rf out/desktop/shim out/desktop/app out/desktop/render
mkdir -p out/desktop/shim out/desktop/app out/desktop/render
javac -nowarn -cp "$AJ" -d out/desktop/shim $(find desktop/shim -name '*.java')
# the app: shims first on the class path, so android.jar only fills in what the shims do not have (GL, sockets, Xposed hooks:
# never run on the desktop)
javac -nowarn -source 8 -target 8 -cp "out/desktop/shim:$AJ:$XP" -d out/desktop/app $(find src -name '*.java') 2>&1 \
  | grep -v "^warning\|bootstrap\|^[0-9]* warning" || true
javac -nowarn -cp "out/desktop/app:out/desktop/shim:$AJ:$XP" -d out/desktop/render $(find desktop/render -name '*.java')
