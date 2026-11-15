# Desktop render harness

Draws the dash map app's **real, unmodified** `src/` on the desktop JVM and writes PNGs to `docs/demo/`. Everything here is
synthetic: the roads, names, route and "Demo Track" are invented, no real location.

## Run

    tools/head_unit/aaroad/desktop/render.sh              # shims + app + scene + 5 PNGs
    SKIP_SCENE=1 tools/head_unit/aaroad/desktop/render.sh # reuse desktop/scene/scene.json (no docker)

Needs JDK 17 (`/opt/homebrew/opt/openjdk@17`), the android-17 `android.jar` and Xposed `api-82.jar` (same files `build.sh` uses,
`ANDROID_JAR` / `XPOSED_JAR` to override), and, for the scene, docker with the `oprad-test:py312` image (`make_scene.py` calls
`road_tiles.encode_road_tile` and `traffic_tiles.encode_traffic_tile`). Outputs land in `out/desktop/` (git-ignored).

## What it does

- `make_scene.py`: a synthetic network (motorway with ramps, arterials, streets, a bridge, a roundabout with 4 arms, signals)
  as one road tile + one traffic tile (moderate / heavy / severe) and three scenarios (roundabout route, U-turn route, free drive).
- `render/.../Render.java` (package `com.starpilot.aaroad`): feeds `Feed.handle`, the method the UDP pump calls, with the same
  JSON datagrams the comma sends (`m`/`q` tiles, `r` route, `p` pose, `s` state, `a` cover), steps a virtual clock 100 ms at a
  time for 4.5 s, and calls `MapView.onDraw` on a 584x215 bitmap (the size the view's code lays out in; Cast crops it for the meter).
- `shim/`: Java2D versions of the `android.*` classes the drawing code touches (Canvas, Paint, Path, Matrix with perspective,
  Region, shaders, Typeface, Bitmap, Handler/Looper/SystemClock, View, Context, AssetManager from a zip, TextUtils) plus an
  `org.json` subset (the JDK has none) and an `XposedBridge` that logs to stderr.

## Limits

- **Software canvas path only** (`MapView.GPU = false`, the app's own fallback). The Tegra GLES layer (`GlRoads`) does not run, so
  road edges, fades and anti-aliasing differ from the car. Clips are not anti-aliased, as on the head unit's Android 4.2.
- Fonts are the app's own Inter files from `assets/fonts`, but Java2D hints and spaces text differently from Skia.
- The music card hides while the turn card is tall (the app's own rule, `SONG_LAST`); `nav_music.png` uses a one-line instruction.
- Music title and artist are shown at rest; they scroll when they do not fit, as on the car.
- Nothing here proves behaviour on the car.
