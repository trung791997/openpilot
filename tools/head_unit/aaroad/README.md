# aaroad — Android Auto road name on the Civic cluster

Xposed module for the 10th-gen Civic head unit (Mitsubishi Electric ADA, Android 4.2.2,
Honda Hack root). Stock `NavigationFocusManager` keeps the Android Auto next-turn road in
`mRoad` but hardcodes `setGuidancePointName()` to `""` and `GuidanceRoadNameCode/Num` to 0,
so the meter gets arrow + distance only. The module fills them from `mRoad` (charset from
`mCharsetCode`, ≤31 chars, ≤36 bytes — the `CanFormatAnalyze.replyBcanNavGuide4` cap).
Any exception leaves the stock values.

Evidence: **static** only (smali of the pulled AndroidAuto and Navigation apps). Not yet
installed or road-tested; whether Honda Hack's bundled XposedBridge loads third-party
modules is unverified.

Build (arm64 or x86 host): Android SDK `build-tools;34.0.0` (for `d8`) and
`platforms;android-17` under `/tmp/hu/sdk`, `api-82.jar` at `/tmp/hu/api-82.jar`,
plus `aapt`, `zipalign`, `apksigner` on PATH. `./build.sh` → `out/aaroad.apk`.
The debug keystore is generated locally and not committed.
