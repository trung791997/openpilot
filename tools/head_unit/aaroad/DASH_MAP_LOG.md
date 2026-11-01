# Dash map (Civic cluster) — work log

Where the cluster-map work stands, so it can be paused and picked up again. Newest first under **Log**. Build, install and feed details are in `README.md` next to this file.

**Evidence words.** *static* means a desktop render or compile only. *limited road evidence* means Peter drove it. Nothing here has been checked beyond what an entry says.

## Where it lives

- **Branch:** `night-star7-vasm-jetson`.
- **Working copy:** the git worktree `/tmp/hu_feed` (local branch `hu-wip`), pushed with `git push origin HEAD:night-star7-vasm-jetson`.
  - This is an exception to AGENTS.md §7 (one working directory), which Peter approved 2026-10-31. The main checkout stays on the radar branch other agents use. See PROGRESS-7.0.md entry 13.
  - If `/tmp/hu_feed` is gone (a new VM), recreate it from the main repo: `git worktree add /tmp/hu_feed -b hu-wip origin/night-star7-vasm-jetson`.
- **Code:**
  - `tools/head_unit/aaroad/src/com/starpilot/aaroad/`:
    - `MapView.java` is the software layer: cards, car, street tag, speed sign, and label choice.
    - `GlRoads.java` is the GPU layer: roads, carpet, fade, labels, its own 30 fps clock.
    - `Feed.java` is the pose and scene feed from the comma.
  - The comma-side feed is `hu_feed.py` (see the README).
- **Desktop render harness:** `/tmp/hu/jvmtest`, with `render_slim.sh <MapView.java> <classdir> <outdir>`. It is not in the repo, so it is lost with the VM.

## Head unit state (2026-10-31)

| | APK | Notes |
|---|---|---|
| Installed now | `aaroad-signals.apk` | exit/highway badges (`f091686b2`), free-drive roads ahead (`4856311cb`), traffic-light icons (`1d2a7a2f9`); installed 2026-10-31 night, comma on `1d2a7a2f9` |
| Previous | `aaroad-turn-hwy.apk` | cheap culling, route traffic, brake lights, turn card from the comma, gold highways, bridges on top (`887675898`) |
| Rollback | `aaroad-cards.apk` | the last software-only build |

The APKs are in `/tmp/hu/`, which does not survive the VM. Rebuild from the commit with `build.sh` in a scratch copy (it runs `rm -rf out`).

**Rules on the car:**
- Install or reboot only in Park, with Peter's OK.
- Reboot with `hu-reboot`, which checks for Park. Never `su reboot`; after a reboot Peter cycles the ignition.
- **Reading the log:** read `/data/data/cn.autohack.hondahack/log/error.log` through the comma.
  1. Copy the adb key to the comma.
  2. Run `adb shell "su -c 'cat …'"` there.
  3. Filter on the comma, because the head unit has no grep.
  4. Delete the key on the comma afterwards.
- Never print the head unit's IP.
- **Network (2026-10-31):** the head unit is on Peter's phone hotspot. The comma's Wi-Fi keeps switching back to `starpilot-android-auto` (the AAWireless network), which leaves it no path to the head unit. Run `nmcli con up` on the hotspot connection and do the adb work in the same command, right away. Peter OK'd switching it. The hotspot can hand out new addresses, so if `/tmp/hu/ip` stops answering, scan the subnet for port 5555.

## Open

1. **Map glitch while driving.** The head-unit log shows GL frames of 32–43 ms (20–24 fps), with worst frames of 0.23–0.38 s that match the freeze in Peter's video. It is the map's own frames, not GPS. Card redraws take 35–55 ms each, 2–6 times a second, and compete for the CPU.
   - **Drive 2026-10-31 (gl parts log):** 9–10 fps. Per frame avg/max ms: cull 46–49/146, swap ~30/60–80, camera 8–24/160–225, upload 0–4/330–360 (one frame per scene), roads 8–9/30–44.
   - **Fixed, static only (2026-10-31, "Map frames: cull, upload, camera"):** cheap cull, scene packed off the GL thread and sent in 256 KB slices, no per-frame allocations in `frame()`/`drawFrame`. Not installed yet.
   - **Next:** install `aaroad-fast-traffic.apk`, drive 10 min, and read the `gl parts` lines again. Swap (~30 ms) is likely GPU fill and is untouched; card redraws, Feed JSON parsing and the scene's `Path` objects still make garbage.
2. **Slim cards** (`a9da6e81b`): static only, so Peter has to see them on the dash. Song names cut off sooner now, so maybe scroll them or widen that card a little.
3. **Double X on the StarPilot Auto car screen** (`2299affae`): on the comma since `ec0b1f6ce` (2026-10-31); Peter has not seen it yet.
4. **No map after the comma reboot (2026-10-31 evening).** The head unit gets no poses because the comma is off the hotspot.
   - At the cold boot, the Android Auto daemon made its second radio `aa0` (10:05:11), gave up 2 s later (`car_interface.py` `READY_TIMEOUT_S = 5.0`, `DedicatedUnavailable`), and fell back to the old lease on `wlan0` (`starpilot-android-auto`). `network.py` notes aa0 once took 16 s on a cold start.
   - An ignition cycle did not bring aa0 back; the daemon stays in fallback until it restarts.
   - **Fix options:** restart the Android Auto service while warm (quick), or raise the aa0 wait for cold boots (code; check with the StarPilot Auto owner first).

5. **Traffic colours** (2026-10-31): static only, not installed. Check on a drive that refreshes stay quiet in the log and the colours change. In `exp` mode the carpet is orange, so moderate amber is hard to tell apart; Peter may want a different colour there.
6. **Brake lights on the car icon** (2026-10-31): static only, not installed. Check on a drive that the lights come on with the pedal and with openpilot's firm braking (accel below -0.5), and go off cleanly. On the GPU layer the glow can start up to 100 ms after the pose (the slow tick); Peter may want the -0.5 threshold or the glow size changed.
7. **Turn card from the comma** (2026-10-31): static only, not installed. Check on a drive that the dash and the car screen show the same arrow, text and distance, that the distance counts down smoothly between the comma's once-a-second updates, and that the card goes within ~3 s when guidance stops. The Honda Hack panel (`Panel.java`, its own page) still uses Android Auto's next-turn data.
8. **Highway colour** (2026-10-31): static only (software path rendered; the GPU path uses the same constants but cannot run on the desktop). Peter to judge the gold at night on the dash.
9. **Overpasses** (2026-10-31): static only, not installed. Needs the new APK *and* the comma updated (`navtilesd`, `hu_feed`). Check on a drive under a highway that the bridge is drawn over the road below with a dark edge, and that the route line stays on top. Each bridge end shows a thin dark arc across the road (the round line caps); Peter may want it gone. Saved tiles away from the car stay without bridges until a drive comes near them.
10. **Exit and highway badges** (2026-10-31): static only, not installed. The turn card and map labels need only the APK (`aaroad-badges.apk`). The street tag's badge for a named highway ("US 75" before "North Central Expressway") also needs the comma updated (`route_engine`, `hu_feed`, `custom.capnp` `roadRef`); an old comma just sends no ref. Road tiles carry `name or ref`, so a map label is a badge only on a road with no name. Peter to judge the badge sizes, and whether he wants "Exit 34" or just "34" in the green box.

11. **Free-drive map loads ahead** (2026-10-31): static only, not installed. APK only; the comma already sends the tiles ahead. Check on a 70 mph drive with no route that roads reach the top of the screen the whole way, and that the corners and ramps do not leave a bare edge. The scene now holds more roads at highway speed (harness: 1932 lines, 5572 pieces, against 206 and 440), so read the `gl parts` cull and upload lines to see what that costs on the Tegra.

12. **Turn card through GPS drops** (2026-11-01): static and replay only. Needs the comma updated (`hu_feed`, `nav_status`); no APK change. Check on a drive that the card and route line no longer blink when the car's GPS drops, and that End navigation still clears them at once.

13. **Dash map through long GPS gaps, with an events file** (2026-11-01): static and replay only. APK only: `aaroad-navhold.apk`. After a drive, pull `/sdcard/aaroad/events.log` (and `.1`). Each `hidden reason=` line says why the map went away. Check that a `map held` line is followed by `position back` and not by a hide. If a drop still happens, the reason says whether it was the page, the comma feed or the view.

## Log

- **Installed** 2026-10-31 night (parked, Peter's OK): comma fast-forwarded to `1d2a7a2f9` and rebooted (came back, nav/feed processes up); `aaroad-signals.apk` installed on the head unit (`install -r` Success), `hu-reboot` sent in Park, adb key removed. Waiting on the ignition cycle. Not driven yet. Check on the next drive: Open 10 (badges), 11 (free-drive roads ahead, `gl parts` cull/upload cost), and traffic-light coverage in the z14 tiles. Rollback: `aaroad-turn-hwy.apk`.

### 2026-11-01
- **The turn card stays through a short GPS drop.** *static + replay of the logged gap*.
  - Problem: Peter saw the dash turn card vanish for a moment (only the map and the Spotify card left), then come back.
  - Drive `00000075--4ad120a8a2` (2026-10-31, comma on `1d2a7a2f9`), parsed on the comma: navigationd went `waitingForLocation` twice while guiding. At 504.2-508.0 s (26 mph) the car's GPS (`headUnitGps`) sent no fix for ~7 s, past the 3 s limit, so there was no location for 3.8 s. At 586.8 s (46 mph) one fix was unusable for 0.2 s. Route 71 had two more 0.2 s drops at ~70 mph; route 72 none.
  - Cause: with no location, navigationd still has the route but publishes no step, and `hu_feed` sent `nav: null` and dropped the route at once. The car's cluster (`nav_status`) went inactive the same way.
  - Fix (comma only): new `navigation/guidance_hold.py`. Through `waitingForLocation` on the same route, `hu_feed` and `nav_status` keep the last step for up to 20 s, counted down at the car's speed once a second, and the route stays. The dash step carries `"held": 1` (older apps ignore it). Arrival, End navigation, a reroute or a new destination still clear it at once. navigationd's own message is unchanged, so nothing that steers, brakes or picks a speed limit sees a held step.
  - Not changed: the comma's own screen (`ui/navigation_state.py`, `ui/onroad_map.py`) still blinks on a GPS drop.
  - Static: `test_guidance_hold.py` (10 tests, with the drive's message timing replayed through the dash feed and the cluster feed), `test_hu_feed.py` and `test_nav_status.py` pass.
  - Commit: this code went in with `b3da066fa` (the tile format 5 commit), which picked it up from the shared index. It was not split back out because that commit is already pushed.
- **The whole dash map no longer drops back to Honda Hack's page during a long GPS gap; each show and hide is now logged to a file.** *static + replay of the logged gap*; APK only (`aaroad-navhold.apk`).
  - Problem: now and then the whole cluster map went back to the Honda Hack view, then came back.
  - Likely cause (limited evidence, no head-unit log from that time): the comma sends the map a position only while the car's GPS fix is at most 1 s old (`hu_feed.py` `carry`, `MAX_CARRY`). Once the map was up, it hid after 10 s without a position. Drive `00000071--37216af48f` (2026-10-31) had a 25.9 s car-GPS gap at ~320 s (67 mph, no route), so the map would have been off for ~15 s. Its other gaps (5.0, 4.2, 3.1 s) and route 75's 6.9 s gap were all under the 10 s limit. Second cause, from reading the code: the head unit sends hellos only while it hears the comma's beacon, and it heard only the beacon, which is a broadcast. A few lost broadcasts paused the hellos, so the comma dropped the head unit after 5 s and the whole feed stopped.
  - Fix: `Liveness.java`. Once the map is up it stays through a GPS gap of up to 60 s, as long as the comma's state message (sent once a second, with or without a fix) is still arriving. The car marker stays where it last was. The map still hides at once when Honda Hack's page goes away, and after the old 10 s when the comma itself goes quiet. It still comes up only on a fresh position. `Feed.java` now counts any message from the comma as hearing it, not only the beacon.
  - Logging: every change is one line, `aaroad: shown reason=...` or `aaroad: hidden reason=...`. The reasons are page not shown (with which Honda Hack view or window is hidden), no position for N s, feed stale (comma silent N s), view detached, and tick failed. There are also lines for `map held` and `position back after N s`, `comma feed lost` and `comma feed back`, `gpu roads on/off`, `meter map added`, and the `gl parts` and `map draw` timing lines. They go to logcat and to `/sdcard/aaroad/events.log` (or the meter app's files dir if the SD card is not writable). At 256 KB the file moves to `events.log.1`, replacing the older copy. Logcat says once where the file is.
  - Static: `test/LivenessTest.java` replays drive 71's gap at 10 Hz: the old rule hid the map ~15 s, the new one 0 s. It also checks that a hidden page hides the map at once, a silent comma hides it, the hold stops at 60 s, and coming up needs a fresh position. `test/EventsTest.java` checks the rotation and the fallback directory. Not driven.

### 2026-11-01
- **Tile format 5: bridge layers, lane counts, one-way and sidewalks.** *static*; comma side only, no drawing yet.
  - Problem: Peter wants Google Maps' close-up look (stacked overpasses, lane lines, sidewalks and crosswalks), and the tiles dropped all of it.
  - `road_tiles.py` reads Mapbox's `layer`, `lane_count` (Mapbox has it only in Japan) and `oneway`, plus class `path` (sidewalk, crossing, footway) and `pedestrian` lines as walks. Cycleways, steps and trails are dropped.
  - `navtilesd` stores format 5 and fetches near tiles in format 4 or older again, once each. `hu_feed` sends a head unit that asks for 4 or less the old format, unchanged.
  - Size, from a synthetic tile (none are cached here): a jittered 22 x 22 downtown grid with 3312 walk lines goes from 7.4 to 27.5 KB, about 6 B per walk line. Roads alone cost 2 B a line more.
- **Head unit reads format 5 and builds the close-up detail.** *static*; nothing is drawn by this commit.
  - `Feed.java` decodes format 5 (layer, lane count, one-way per line; the walk table) and asks the comma for it.
  - New `CloseUp.java` (pure Java) turns the scene's lines into segments by range: bridge decks per layer and class, with a ramp where a bridge meets a ground road (or inside a lone bridge), lane lines for motorways (lane count, else 3 one-way / 4 two-way), and sidewalks and dashed crosswalks. Height 12 m a layer. Capped at 6144 road and 4096 walk segments, nearest first, within 1400 / 1000 m.
  - Static: `test/CloseUpTest.java` all ok. A busy synthetic scene is 403 road + 1209 walk segments, 226 KB on the GPU, built in 2 ms (desktop JIT).

### 2026-10-31
- **Traffic lights on the dash map, like Google Maps.** *static*.
  - Problem: Peter asked for the small traffic-light icon Google Maps puts at an intersection with a light or a stop sign.
  - `Feed.java` reads tile format 4 and asks `rt` 4.
  - `Signals.java` (desktop test `test/SignalsTest.java`):
    - OSM's signals within 30 m of each other are merged into one icon, since a divided road has one at each corner.
    - It keeps those within 300 m and not more than 15 m behind, at most the 6 nearest, drawn farthest first.
    - With a route, the next one on it ahead is drawn 1.25x.
  - `SignalIcon.java`: the icon is upright, centred on the intersection, and drawn under the street names.
    - A light is a dark pill, 7 x 15 px, with a light rim and red, yellow and green dots.
    - A stop sign is a red octagon, about 15 px with its white rim. None arrive yet, because the data has no stop signs.
  - Both paths draw the icons the same way:
    - Both place them with `MapView.placeSignals`, which hides them where a label would be hidden.
    - The view draws them with `SignalIcon.draw`. The GPU layer draws the same routine once into a 96 x 24 sprite texture and lays the sprites at the same whole pixels; the harness compares them and no pixels differ.
  - Harness renders are in `/tmp/hu/signals/`. The software path costs +0.2 to 0.4 ms a frame on the desktop JVM.
- **Road tiles carry traffic signals (format 4), comma side.** *static*.
  - Problem: Peter asked for Google's small traffic-light icon at signalised intersections. The road tiles held only lines, so the head unit had no signal positions.
  - `road_tiles.py`: format 4 ends with a points table (`<H` count, then `<h x, <h y, <B kind` per point; kind 1 a signal, 2 a stop, reserved). The points are Mapbox Streets' `traffic_signals` points in the z14 `road` layer, which come from OSM `highway=traffic_signals`. Mapbox has no stop or give-way points, and mapd has no node tags, so no stop signs reach the map.
  - `navtilesd.py`: format 4 is current. A tile near the car saved without the points is fetched again once. `hu_feed` sends a head unit that asks `rt` 3 or older its format with the points dropped.
  - Tests: `test_road_tiles.py`, `test_navtilesd.py`, `test_hu_feed.py`.
  - Not checked: how many US signals the z14 tiles actually carry (no token on this VM).
- **The map ahead loads in free drive** (Open 11). *static*.
  - Problem: with no route, at highway speed the top of the dash map was often bare and filled in only as the car got close. The comma had the tiles; the head unit kept only the roads within 700 m of the last rebuild and rebuilt every 600 m, while the view at 70 mph reaches about 790 m ahead and 1200 m to the side. In the harness the road ahead fell to about 100 m before each rebuild. The cull box also stopped at x 240, which left the top right of the free-drive view bare.
  - `FreeView.java`: with no route the scene keeps a rectangle turned to the car's heading. It covers the view at that speed (turned up to 30 degrees either way), 250 m around that, and 30 s of road ahead. The scene is rebuilt as soon as the view now, 5 s on, or turned 30 degrees would leave it. Service roads go above 264 m of range (StarPilot Auto's 2.2 m/px, about 49 mph). Road points within 1 m of a straight line are dropped. The box's right edge is now 290.
  - Harness (synthetic city, software path): the least road ahead was 100/102 m before (highway/arterial) and is 1800/1502 m after.
  - Tests: `test/FreeViewTest.java` (straight, curves, a cloverleaf, 0 to 80 mph, city turns, 30 random drives, with the build landing 1 s late) and `test_hu_feed.py` (70 mph: every tile is on the head unit 65 s or more before the car enters it).
- **Exit numbers in a green box, highway numbers in a white box** (Open 10). *static*.
  - Problem: the turn card read "Exit 34 McDermott Drive" and "Keep left onto US 75 South" as plain text, where Google Maps marks the exit and the route number.
  - `Shield.java` (desktop test `test/ShieldTest.java`) finds a leading exit number and route numbers (I-635, US 75, SH/SR, state codes, Loop, Spur, FM/RM, CR, Beltway) and writes them one way ("I 635" -> "I-635"). Tollway names and words without a number stay text.
  - Turn card: the exit is a green badge, route numbers white badges with black text; lines break between words and badges, never inside a badge, and a badge that does not fit is left off rather than cut.
  - Street tag: a route number in the name, or the comma's new `ref` for the road, is a white badge before the name; when both do not fit at 14 px the badge shows alone.
  - Map labels: a label that is a route number draws as a white badge with a dark rim. The GPU atlas draws through the same `MapView.drawLabel`, so both paths match; its width is the one `placeLabels` reserves.
  - Comma: `route_engine` keeps Mapbox's step `ref` as `roadRef` (new `custom.capnp` field @10), and `hu_feed` sends it as `ref` in the nav message. Old head units ignore the key.
- **Installed** (parked, Peter's OK): comma updated to `bb4a12ca5` and rebooted (aa0 and the hotspot came up this time); `aaroad-turn-hwy.apk` on the head unit, rebooted, ignition cycled. Head-unit log: meter map showing, GPU on, parked frame parts cull 1.2 ms avg (was ~48 ms driving before `c48ffae76`), swap 18.5 ms avg. Not driven yet.
- **Bridges drawn over the roads under them** (Open 9). *static*.
  - Problem: a highway overpass and the street under it were drawn as one flat crossing, so the dash showed a junction that is not there.
  - Tiles: `road_tiles.py` format 3 keeps the vector tile's `structure` tag as a flags byte per line (`BRIDGE` 1, `TUNNEL` 2; `layer` is not kept). It adds about one byte a line before compression.
  - Old head units: the hello's `rt` says which format the app reads. This APK sends `rt` 3. The 10-30 app (`rt` 2) gets format 2 with names, and older apps get format 1. `downgrade_road_tile` now keeps names when going to 2.
  - Comma tile store: tiles are files under the maps `roads` folder with the format in byte 4; nothing else versions them. `navtilesd` re-fetches a saved tile near the car once when it is older than format 3 (`CURRENT_FORMAT`; this was `VERSION_WITH_NAMES` 2). Tiles elsewhere, including offline areas, stay in their old format and draw without bridges until the car comes near them or the area refreshes (120 days). Old files still decode everywhere.
  - Head unit: `Feed.decode` reads formats 1-3 and keeps a per-line `bridge` flag. A bridge line's scene class is its tile class + `MapView.BRIDGE` (5). Both paths draw every non-bridge kerb and surface first, then the bridges: a dark casing (`#101010`, 2 px each side past the kerb), kerb, and surface in the class colours. The orange route line is still drawn last. GPU: road groups 0-9 and the route groups 10-13. An empty group costs no draw call, so a frame with no bridge in view does the same GL work as before.
  - Static checks: python tests pass (bridge round trip, v3 to v2/v1, navtilesd re-fetch, hu_feed `rt` 2/3). `GlCullTest` passes. A harness render with I-635 on a bridge over US 75 and Main St shows the bridge on top and the carpet over it. The GPU path is not rendered.
- **Highways in their own colour** (Open 8). *static*.
  - Motorways and trunks (tile class 0) have a muted gold surface `#7a6420` with a darker gold kerb `#54451a`; ramps (class 3) a dimmer gold `#5a4b1e` / `#433816`. Other roads stay grey. Constants `MapView.KERBS/ROADS`, used by both `GlRoads.drawFrame` and the software path, so they cannot drift. Same number of GL draws per frame.
  - The route's own grey road is now drawn under the tile roads' surfaces, so on a highway the gold shows round the route line; the orange/traffic line stays on top. Without highways the renders are pixel-identical to the last commit (only the ETA minute differs).
- **The dash turn card matches StarPilot Auto** (Open 7). *static*.
  - Problem: the card read Honda Hack's `turn_icon`/`seg_dist`/`next_road` widgets, fed by Android Auto's throttled next-turn path, so it lagged or froze behind the car screen.
  - Now it draws from `Feed.nav`, the comma's own step (`nav` in the `s` datagram, which the car screen also draws). `Turn.java` ports `icon_name`, `distance_text` and the text clean-up; the arrows are the car screen's 88 PNGs, copied into the APK's `assets/nav/` by `build.sh` from `navigation-assets.json` (+~130 KB). The instruction text wraps to two lines like the car screen. Between datagrams the distance counts down at the car's speed. The card blanks when nav is over 3 s old.
  - No fallback to the old widgets: the comma is the only Android Auto nav source, so they only ever held a late copy of the same step, and a fallback could bring back a frozen turn.
  - `Hook.street` no longer nulls "Exit 19A Walnut Hill Lane" (used by `Panel.java`): "exit" followed by a number is a sign, not an instruction.
  - Static: `test_dash_turn_card.py` (4 tests: arrows for 18 types x 13 modifiers, 46k distances both units with rounding ties, text) passes against the Python. Harness renders before/after look right (arrow, distance, wrapped exit text, trip line unchanged).
- **Brake lights on our car icon** (Open 6). *static*.
  - Comma: `hu_feed.brake_of` is true when the driver presses the brake (`carState.brakePressed`) or openpilot long is active and asks for less than -0.5 m/s² (`carControl.actuators.accel`). `carState.brakeLights` is deprecated and the Honda carstate fills no brake-light field, so it is not used. The pose `p` carries `"b": 1` only while braking; older apps ignore it.
  - Head unit: `Feed.brake` from the pose. The two tail lights glow red (a halo under the sprite, a bright core over each lamp) and fade in or out over 150 ms; off if the pose is stale. Drawn in `drawCar`, which both the GPU and software paths call. The view redraws only while the glow changes, and the GPU-path tick drops from 100 ms to 33 ms during a fade.
  - Static: python 324 passed (322 before). GlCullTest PASS. Harness without braking is identical to the last commit (13/13, with and without traffic). Brake renders at 75 ms (half) and 200 ms (full) look right. APK 492,278 bytes.
- **Traffic colours on the route line** (Open 5). *static*, plus one live Mapbox check.
  - Comma: Directions asks for `congestion` and `enable_refresh`. Every 5 min while guiding, `directions-refresh` fetches fresh traffic from the car's place on. It runs on the nav executor and counts on the `directions` budget without blocking. On any error the old colours stay. Geometry, steps and guidance are never touched.
  - `navigation.routeCongestion` (custom.capnp @14) has one level per preview point, the worst within each stride. `hu_feed` sends it as `c` in the `r` datagrams, aligned with `pts`. A traffic change resends the route under a new id.
  - Head unit: the carpet is drawn per level. Unknown and low keep the mode colour; moderate is amber `#ffb300`, heavy orange `#ff5722`, severe red `#c62828`. Done on the GPU layer (route groups 5–8 in `GlCull`) and in the software fallback.
  - Static: python 322 passed (308 before). GlCullTest PASS (runs and bytes identical). Without traffic, harness renders are identical to the last commit (13/13). With traffic, the carpet changes colour at the right points.
  - Live (from the comma, token kept on the device): Directions `Ok` with a uuid, 571 congestion entries for 572 points. Refresh at index 285 returned `Ok` with 286 entries, the tail from that index, which is how `refresh()` merges it.
- **Map frames: cull, upload, camera** (Open 1). *static*
  - Cull: new `GlCull` keeps a candidate list from a circle round the view, tests each candidate's centre, and runs the old corner test (without `Math.min/max`) only on the rest. `test/GlCullTest.java`, synthetic 14.5k-piece scene: identical runs in all 7,499 frames, identical packed bytes. Interpreted JVM (`-Xint`, closest to Dalvik): 18.3 → 1.7 ms a frame while driving; JIT: 0.25 → 0.08 ms.
  - Upload: a packer thread (`aaroad-pack`) builds the quads. The GL thread sends 256 KB a frame into a second VBO while the old scene keeps drawing, then swaps. Packing alone took 178 → 36 ms interpreted.
  - Camera: `frame()` fills a Frame the caller owns. `Feed.world` writes into a caller array, `drawFrame`'s `int[2]` is a field, the tick's hide reason is built only when logged, and `carpetStart` clamps without calls.
  - Harness renders are byte-identical to the old ones (13/13).
- `a9da6e81b` **Slimmer left column.**
  - The trip card is folded into the turn card as a third line: "11 min · 5.2 mi · 2:15 AM".
  - The column right edge moves 270 → 210 px and the dark band 300 → 236, so the map gets about 60 px.
  - The music card sits under the turn card. *static* (render compare `/tmp/hu/slim_compare.png`).
- `ace790491` **Frame-part timing.** While moving, the log gets the average and worst time per stage every 10 s, plus the stage the slow frames spent their time in. Compile only.
- `2299affae` **One X.** The StarPilot Auto car screen hides the active Home/Work button while the X under the turn card already ends that route. Static tests pass.
- `70a580174`, `91ed57f74`, `82e8f0b91`, `83e1e17d3`: the GPU layer runs its own 30 fps clock and draws the street labels and fades. MapView redraws only when the car, tag, sign or cards change. Installed as `aaroad-gl30.apk`; Peter drove it and found it a little glitchy, see Open 1.

### 2026-10-29 – 10-30
- Built the cluster map: the turn/trip/music cards in the style of Peter's mockup, the Spotify card with progress and scrolling names, and street labels without a route.
- Roads moved to the GPU (`283bb4e47`), with one camera shared by both drawing paths and a car that glides between GPS fixes (`3bc4f6ce7`).
- The full list is in `git log -- tools/head_unit`.
