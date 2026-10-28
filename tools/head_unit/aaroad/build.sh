#!/bin/bash
set -e
cd "$(dirname "$0")"
SDK=/tmp/hu/sdk; BT=$SDK/build-tools/34.0.0; AJ=$SDK/platforms/android-17/android.jar
rm -rf out; mkdir -p out/classes out/dex
javac -source 8 -target 8 -bootclasspath $AJ -cp /tmp/hu/api-82.jar -d out/classes $(find src -name '*.java') 2>&1 | grep -v "bootstrap\|^warning\|^1 warning" || true
$BT/d8 --min-api 17 --lib $AJ --classpath /tmp/hu/api-82.jar --output out/dex $(find out/classes -name '*.class')
aapt package -f -M AndroidManifest.xml -S res -A assets -I $AJ -F out/unsigned.apk
(cd out/dex && aapt add ../unsigned.apk classes.dex >/dev/null)
zipalign -f 4 out/unsigned.apk out/aligned.apk
[ -f debug.keystore ] || keytool -genkeypair -keystore debug.keystore -storepass android -keypass android -alias aaroad -keyalg RSA -keysize 2048 -validity 10000 -dname "CN=aaroad" >/dev/null 2>&1
apksigner sign --ks debug.keystore --ks-pass pass:android --v1-signing-enabled true --v2-signing-enabled true --min-sdk-version 17 --out out/aaroad.apk out/aligned.apk
apksigner verify --min-sdk-version 17 out/aaroad.apk && ls -l out/aaroad.apk
