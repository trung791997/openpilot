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

## Head unit state (2026-11-03)

| | APK | Notes |
|---|---|---|
| Installed now | `aaroad-overpass2.apk` | everything in `aaroad-hold.apk`, plus turn and slip lanes drawn as streets (Open 16) and real overpasses as 3D decks; installed 2026-11-05 in Park (Peter's OK), `install -r` Success, comma on `2336dc471`, `hu-reboot` sent, adb key removed, waiting on the ignition cycle; not driven yet |
| Previous | `aaroad-hold.apk` | built at `ba1ecd65f`: events.log fallback dirs, time-stamped error.log lines; installed 2026-11-03 |
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
- **Network (2026-11-03):** the head unit's Wi-Fi is 2.4 GHz only. With the phone hotspot on 5 GHz (channel 149) the head unit dropped off it and the dash map got no feed; Peter turned on the hotspot's 2.4 + 5 GHz broadcast and the map came back. The comma's hotspot profile (`iqpilot connection Galaxy S20 FE 5G6CC3`) is pinned to 2.4 GHz (`802-11-wireless.band bg`, set on the device with Peter's OK, persisted in /etc/netplan on /data) so wlan0 stays off the 5 GHz band StarPort's car link (aa0, 5180 MHz) uses; takes effect on the next join. If the map is missing, check the phone still broadcasts 2.4 GHz.

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

14. **Lane guidance and the "toward" line** (2026-11-01): static only, not installed, not driven. **Peter picks where the lane row goes:** A, under the turn card (`/tmp/hu/lanes/*_card.png`), or B, under the car with the map lifted and zoomed in a little (`*_car.png`). Set `Lanes.at` in `Lanes.java` (`AT_CARD` ships now) and rebuild. Needs the comma updated (`route_engine`, `hu_feed`, `custom.capnp`) and the APK (`aaroad-lanes.apk`); an old comma just sends no lanes. Check on a drive that the lanes match the road's painted arrows, that the row shows from about 1 km out and goes after the turn, and in B that the lift eases in without the car marker jumping off the GPU map. In A the song card is hidden while the row shows, because there is no room for both.

15. **Map stays up at a stop, events.log, timed error.log lines** (2026-11-01): static and replay only, not installed, not driven. Needs the comma updated (`hu_feed`) for the stop fix and the APK (`aaroad-hold.apk`) for the log fixes; either works without the other. Check on a drive with a long red light that the map stays on the car and no `map held` / `hidden` line comes while stopped. Find `aaroad: map events go to` or `aaroad: events.log not written:` in error.log to learn where the events file went, or why none did. **Peter to decide:** the hold limits (a held stop pose dies after 5 m of wheel roll; a heading with no GPS bearing dies after 10 s of moving), and whether to look at the comma's own GPS (route 79 never had a fix from it).

16. **Turn lanes drawn as streets** (2026-11-05): static only, not installed. APK only: `aaroad-overpass2.apk`. Check on a drive that the Angel Pkwy turn lanes show as grey street, that highway ramps stay gold right up to the highway, and that no ramp turns grey and then gold as tiles load.

## Log

### 2026-11-08
- **Blind-spot side glow on the dash map car.** *static*: Python tests, APK build and existing desktop tests only; not installed, not road-tested
  - Comma (`hu_feed.py`): pose `p` gains optional `"bl"`/`"br"`: 1 while that side warns (`blind_of`: the car's blind-spot monitor OR the V-ASM warning, as `projection_theme.side_colors`), left out otherwise. V-ASM is sampled with the gates of `projection_onroad._vasm_warning` (fresh drive-gear carState with canValid, saved preferences enabled, a clock pair, `PiPWarningSource`); any failure is (False, False).
  - Head unit: `Feed.blindLeft`/`blindRight`; `MapView.drawCar` draws a steady red halo along that side of the sprite (rotates with the car), faded over `GLOW_MS` with the brake glow's stale rule. Brake glow untouched.
  - Not checked: how the halo looks on the head unit (`BLIND_*` set from a desktop mock render over the 52 px sprite, not the head unit), real V-ASM warnings, GPU-path fps.

### 2026-11-07
- **The car's outline under a bridge or overpass now shows at every zoom, not only in the close-up, and on a frontage road under a viaduct too.** *static*: desktop tests and harness renders on real Dallas tiles
  - Problem: Peter, "the outline to indicate when I drive beneath a bridge or an overpass is inconsistent". It was drawn only in the close-up (gate `ROAD_SHOW`/`ROAD_HIDE`), and a car on a ground road beside a viaduct counted as on the viaduct.
  - Every zoom: `Under.covering` runs at hs 0 too (the flat footprint cruising draws); `MapView.underDeck`/`underWanted` no longer gate on the close-up.
  - The car's road: `CloseUp.Detail.ground` (new, never drawn) holds plain ground roads' centrelines. `Under.carHeight` takes the nearest road running the car's way (ground or raised); a tie keeps the higher one. Bbox prefilter: ~29 us a call on the desktop at the segment caps.
  - Fade: `Under.fadeMs`, 150 ms at or under 15 m/s, down to 80 ms by about 28 m/s. Ramps already counted once at least `OVER_M` above the car (unchanged).
  - Static: all 10 tests in `test/` ok (UnderTest 29), `build.sh` ok. Close-up renders of 18 real overpasses byte-identical to before; zoomed out, 2 of 5 real cases now show the outline (the other 3 have no deck over the car at that point, same before and after). Renders in `/tmp/hu/outline5/`.
  - Not checked: the frontage-road case on real tiles (unit test only); the GPU path on the head unit; time on the Tegra 3. In cruise, a raised non-bridge ramp over the car can cut the car where the flat map draws no deck.

- **Blank map mid-drive (Peter 2026-11-06 ~21:00-22:18):** the head unit log showed scene rebuilds fall from ~40 per 10 min to none after 21:00 while GL kept drawing at 29 fps, and two comma feed losses (21:02, 21:25) logged no "hidden" line, so the view's tick (main-thread postDelayed) had stopped; no error, no hung thread (stack dump). Cause of the stop not proven. Fix in `aaroad-tickfix.apk` (also carries the outline work): the tick always re-posts (finally, delay capped at 500 ms), the GL loop re-posts it if quiet for 2 s and logs `map tick quiet ... posted again`, and `gl parts` adds `tick age`. Installed 2026-11-07 in Park (Peter's OK), `install -r` Success, `hu-reboot` sent; waiting on the ignition cycle. Static only. Known leftover: old aaroad-gl/aaroad-pack threads stay alive after each meter page re-creation (6 seen).
- **Map never showed after the tick fix (Peter 2026-11-07):** the tick fix's `static Handler MAIN = new Handler(Looper.getMainLooper())` ran in the zygote (`Hook.initZygote` touches `MapView`), where there is no main looper: `ExceptionInInitializerError` / NPE at every head unit boot after the 11-06 install, and the class stays unusable until the next boot. Fixed in `b794e48d9` (per-view handler). `aaroad-zygotefix.apk` (tick fix + outline + thread fix `a7572e3a5` + this) installed in Park with Peter's OK, `hu-reboot`, ignition cycle: no error at boot, `comma feed back`, `shown`, `gpu roads on` at 15:37, Peter confirms the map is up. Parked only; not driven yet. Never install `aaroad-threadfix.apk` or `aaroad-tickfix.apk` again. Side note: the comma's `/tmp/hu/ip` copy on the VM had been stale since 10-31, and the comma had been on home Wi-Fi, which first looked like a dead head unit.
- **Map froze with a big white arrow over it (Peter 2026-11-07, 17:54 drive):** Android Auto guidance started (`panel start` 17:54:29) and one second later the cluster process's (`com.mitsubishielectric.ada.app.externaldisplay`) main thread hung for good: thread dump shows it in Honda Hack's receiver (`C0170b` `UPDATE_NAVI_WAZE` → `XSharedPreferences.reload` → `Thread.start`, VMWAIT) with the new load thread waiting on the lock main holds. The GPU layer kept drawing, but `map tick quiet` grew past 54 s, so the card, street and position stopped. The big right arrow was Android Auto's own next turn (its route, not the comma's) on Honda Hack's panel. Each `Panel.java` turn broadcast (about one a second) makes Honda Hack re-read its settings three times on that main thread, each read starting a thread. Fix: `Panel.ENABLED = false`, so the hook is not installed; the dash turn card comes from the comma. APK `/tmp/hu/aaroad-nopanel.apk` (zygotefix + this), static build only, not installed. Check after install: start Google Maps guidance on Android Auto, no `panel` lines in the log, no arrow over the map, no `map tick quiet`.

### 2026-11-05
- **On the dash map, turn and slip lanes at street corners now look like the street instead of a short olive streak. Only highway ramps keep the ramp gold.** *static*: desktop tests and harness renders only. Not installed on the head unit, not driven, and the GPU path was not run on the Tegra.
  - Problem: Peter's Angel Pkwy photo (entry below). Tile "link" lines (class 3) are motorway and trunk ramps, but also `primary_link`, `secondary_link` and `tertiary_link` turn lanes. All of them were drawn in the ramp gold, so inside a grey street they showed as olive strokes that looked like traffic.
  - The rule (`CloseUp.ramps`): a link keeps the gold only if it touches a motorway or trunk line, or joins one through other links.
    - "Touches" means an end of one line lies within `JOIN_M` (4 m) of the other line. A link that only crosses a highway in plan (going under or over it) does not touch it.
    - Links that touch each other count as one ramp, so the far piece of a ramp past a tile edge, or a branch off its middle, stays gold too.
    - Every other link is drawn exactly as a street: street grey, street width, street kerb (class 2).
  - Where it runs: once per scene build, in the app. `road_tiles.py` and the tiles are unchanged.
    - `CloseView.links` judges all loaded tiles together in one metre frame, so a ramp cut at a tile edge is judged whole.
    - `MapView.build` uses the result for the base map, and `CloseView.detail` uses it for the close-up. Both the GPU path (`GlRoads`, `GlClose`) and the software path read what these build, so all four draw the same thing with the same passes in the same order.
    - Nothing is added per frame. In a busy test (400 links, 200 highway lines) it takes about 7 ms, or 12 ms with the JIT off, once per scene on the build thread.
  - Static:
    - `CloseUpTest` 91 ok (was 82). New cases:
      - a slip lane at a street corner is not a ramp;
      - an off-ramp leaving the motorway mid-line is a ramp, and so are its far piece past a tile cut and a branch off its middle;
      - an end 6 m from the motorway does not touch it;
      - a link crossing the motorway in plan is not a ramp;
      - a link joined only to a street link is not a ramp;
      - a motorway ending on a link's middle makes the link a ramp;
      - the empty and busy cases.
    - `UnderTest` 21 ok and `TiltTest` 17 ok. The free view, lanes, shield, signals, events, liveness and GL cull tests are ok. `build.sh` builds.
    - Harness (`-Dscene=links`: a street grid with four slip lanes at Main St / Elm Ave, plus a diamond interchange on US 75). Outputs are in `/tmp/hu/yellow/before_links` and `after_links`, with side-by-side pairs in `/tmp/hu/yellow/pairs`.
      - Street-grid frames: 763 and 402 gold pixels before, 0 after.
      - Highway-ramp frames: byte-identical, still gold (424 and 290 pixels).
    - Bridges and plain scenes (22 frames, `/tmp/hu/yellow/before` and `after`): byte-identical to HEAD. The harness flyover was moved to start on US 75, because a flyover that touches no highway would now turn grey.
    - APK: `/tmp/hu/aaroad-overpass2.apk`. It holds the overpass redesign plus this change and is signed with the same key.
  - Known ways the rule goes wrong:
    - If the highway, or the rest of the ramp, is in a tile that has not loaded yet, the ramp is grey until the next scene build.
    - A street link that ends within 4 m of a highway it does not join (for example, under a highway bridge) is gold.
    - A slip lane at an at-grade junction on a trunk road is gold. Trunk counts as highway.
    - A U-turn lane under a highway that joins the two frontage roads is grey.

### 2026-11-05
- **Zoomed in, a bridge on the road you're driving (or over a creek) now looks like the rest of the road instead of a grey slab laid on top. Only real overpasses, the ones a road passes under, stand up, now drawn as 3D decks with side faces, piers and a shadow.** *static*: desktop tests and harness renders only. Not installed on the head unit, not driven, GPU path not run on the Tegra.
  - Problem: Peter on FM 544 at 12 mph, close-up. A creek bridge on his own road was drawn as a raised slab with casing and ramps, so it looked like patched work or another route drawn on top. Every tile bridge was drawn raised, whatever was under it.
  - The rule (`CloseUp.overpasses`): a bridge is raised only if the car could drive under it.
    - Something must cross under it in plan: a ground road or a lower-layer bridge. Rail lines and paths don't count.
    - The crossing must be at least `CROSS_MIN_DEG` (20 degrees) and away from both lines' ends (`JOIN_M`), so a road meeting the bridge end on is its approach, not under it.
    - Bridges joined end to end on one layer count as one.
    - The road being driven is never raised. That means a bridge with half its length (`ON_PATH_SHARE`) within 12 m and 30 degrees of the route; with no route, of the car's own road ahead and behind (`carRoad`: the line within 20 m running within 35 degrees of the heading, plus the straightest joins out to `ROAD_R`).
    - Every other bridge is drawn as the plain road it is part of, in the base map too (`MapView.build` gets the flat/raised result per tile line), at every zoom.
  - The overpass model (`CloseUp.build`, `CloseView.passes`):
    - The full-height stretch of the deck goes in its own ranges (`deck(g, c)`).
    - Fascia: three slices down the side face, the lowest in underside colour.
    - Parapet: in place of the casing.
    - Piers: pairs of 3 px columns about every `PIER_M` (30 m), kept `PIER_CLEAR_M` (6 m) clear of any road under it; a span shorter than 30 m rests on its ends.
    - Shadow: the existing shadow pass.
    - Pass order: walks, ground lanes, all shadows, all piers, then per layer group its walls, fascia, casing/parapet, kerb, surface, lanes.
    - The GPU (`GlClose`) and software paths both run `CloseView.passes`, so they draw the same passes in the same order.
    - The base map's bridge copy fades out as the close-up fades in (software alpha; `GlRoads` waits until `GlClose` shows the scene's own detail, so one copy is always drawn).
    - `Under` (the car's outline under a deck) looks at the deck ranges too.
  - Free drive: if a scene held an overpass flat because the car was on it, the scene is rebuilt as soon as the car leaves that road (`Detail.heldOn`, `CloseUp.onRoad`). Without this, a harness drive west over I-635 and then north under it on US 75 drew I-635 flat over the car for the next 600 m. In a scene where nothing was held down, this check does nothing.
  - Peter's second photo (Angel Pkwy, free drive, 18 mph): the short dull olive strokes along road centres are not traffic, lane paint or turn hints.
    - What they are: tile "link" lines, class 3 (`road_tiles.py` `_CLASS_OF`). These are Mapbox `primary_link`, `secondary_link` and `tertiary_link` (turn and slip lanes at intersections) as well as motorway and trunk ramps.
    - How they're drawn: in the dim ramp gold (`ROADS[3]` 0xff5a4b1e), 12 m wide, last in `DRAW_ORDER`. Kerbs are drawn first, so inside a grey street they show as a narrow olive stroke with no outline.
    - When they show: whenever the tiles have one, with or without a route, at every zoom.
    - Traffic colours only ever go on the route line, and the close-up yellow line is only drawn on motorways.
    - Recommendation (done 2026-11-05, entry above): restyle. The gold was meant to say "highway ramp" (2026-10-31). Draw street-level links as street grey, either in `road_tiles.py` (needs the tiles rebuilt) or in the app (gold only when the link touches a motorway or trunk).
  - Static:
    - `CloseUpTest` 82 ok.
      - New: a creek bridge stays flat; rail or a path under it doesn't raise it; roads meeting its ends don't count; an 11-degree crossing doesn't count; a same-layer crossing doesn't count; over a lower bridge only the flyover rises; a route under it gives a deck, ramps and 8 piers in pairs clear of the roads; a route on it, or the car on it, keeps it flat; `carRoad` follows the straightest join; the car under it keeps it raised; the held flag and `onRoad`.
      - The busy-scene build is still about 2 ms.
    - `UnderTest` 21 ok. Free view, lanes, tilt, shield, signals, events, liveness and GL cull tests ok. `build.sh` builds.
    - Harness renders, `/tmp/hu/overpass/before` and `after` (side by side in `/tmp/hu/overpass/pairs`):
      - The scene with no bridges (`plain_*`, 4 frames) is byte-identical to HEAD when rendered in the same minute; otherwise only the arrival clock differs. `free_far_25ms` is identical too.
      - Route under I-635: deck, fascia, piers, shadow.
      - Route on I-635 and the 60 m creek bridge, route or free drive: drawn flat.
  - Not checked:
    - Not on the head unit, so the GPU draw cost on the Tegra 3 is unknown. The added work is about 5 deck ranges and 3 pier ranges, of a few segments each, and only where an overpass is in reach.
    - Known ways the rule goes wrong:
      - the road under it is missing from the tiles: drawn flat;
      - a long water bridge with a shore road under one end: all of it raised;
      - a frontage road crossing under 20 degrees: flat;
      - lines dropped by the build cap or beyond `ROAD_R`: the deck is lost while the base copy fades;
      - a pier can stand through a lower deck;
      - the other-direction twin of the bridge being driven, within 12 m and 30 degrees, is also drawn flat;
      - on the GPU, the base copy switches back briefly while a new scene's detail uploads.
    - Pre-existing and not changed: in a free-drive scene anchored at the I-635 overpass, the lane lines on US 75 past the deck are missing; HEAD does the same (`/tmp/hu/overpass/fresh_before`).

- `Lanes.at = AT_CAR`: Peter picked the lane row under the car (option B: map lifts 38 px and zooms in 15% while lanes show; song card stays). Comma updated to `1c15daba8` and rebooted while parked (Peter's OK for the comma). Ready to install: `/tmp/hu/aaroad-lanes-car.apk` = GPS-gap hold + map-drop log (`f045536c1`), close-up 3D (`e177a7065`), lanes (`1c15daba8`), this switch; same signing key as the installed app. Not installed, not driven. Rollback: `aaroad-signals.apk`.
- **Installed** 2026-10-31 night (parked, Peter's OK): comma fast-forwarded to `1d2a7a2f9` and rebooted (came back, nav/feed processes up); `aaroad-signals.apk` installed on the head unit (`install -r` Success), `hu-reboot` sent in Park, adb key removed. Waiting on the ignition cycle. Not driven yet. Check on the next drive: Open 10 (badges), 11 (free-drive roads ahead, `gl parts` cull/upload cost), and traffic-light coverage in the z14 tiles. Rollback: `aaroad-turn-hwy.apk`.

### 2026-11-01
- **Stopped at a light, the dash map keeps showing the car instead of hiding it. The head unit's events file now gets written somewhere, and every map line in error.log carries the time.** *static and replay*: desktop tests and a replay of route `00000079--c1010e535e` only; not installed on the comma or the head unit, not driven.
  - Problem: at about 13:20 on route 79 the map said `map held` and then `hidden` while the car sat stopped. No `events.log` showed up on the head unit, and the error.log lines had no time on them.
  - Cause, from the replay: the comma's own GPS had no fix for the whole drive (hasFix false, about 81 km accuracy), so the only position was the head unit's GPS, relayed at 1 Hz. That GPS reports no bearing below about 1.2 m/s. At 4956.1 s the car crept at exactly 1.0 m/s, which `hu_feed`'s `HeadingFilter` does not count as stopped (`STOPPED_SPEED` 1.0), on a fix with no bearing. The filter then set the heading to None, and a stopped car with no bearing could never get one back. With no heading there was no `fix`, so no pose went out from then until the car drove off. The app held the map 10 s and hid it at 60 s.
  - Fix, comma side (`hu_feed.py`):
    - A fix with no bearing keeps the heading the gyro carries, for up to `MAX_BEARING_GAP` (10 s) of moving. A stop resets that clock.
    - New `StopHold`: when the fix itself goes (no source, or older than `MAX_CARRY`), it keeps sending the last pose with `age` 0, `spd` 0 and `"held": 1`. This lasts only while carState is fresh (0.5 s), the car is under `STOPPED_SPEED`, the wheels have rolled under 5 m since the last real pose, and ticks are under 1 s apart. A moving car never gets a held pose.
    - The app needs no change: `held` is ignored, and age 0 keeps `Liveness` happy.
  - Fix, app side:
    - `Events` tries `/sdcard/aaroad`, then `aaroad` under the app context's files dir, the view context's files dir, then the cache dirs. Each is asked separately, so one that throws or gives null no longer drops the rest. Before, one failing `getFilesDir()` left only the SD card.
    - `Events.status()` names the file in use, or each directory's reason (`not a directory`, `mkdirs failed (… not writable)`, the exception). MapView logs it once, and again only when it changes.
    - New `Xlog` puts the wall-clock stamp (the events.log format) on every aaroad line sent through `XposedBridge.log`, so error.log lines carry the time.
  - Static:
    - `test_hu_feed.py`: 34 passed. Five tests are new: the 1.0 m/s creep with no bearing, a bearing gap carried by the gyro, a 10-minute standstill held, no hold while moving or after 5 m of roll, and the feed sending `held` poses with no fix. ruff clean.
    - `EventsTest` adds the cache-dir fallback, status reasons, stamps and `use()`, ok. The other JVM tests are ok. `build.sh` ok, giving `/tmp/hu/aaroad-hold.apk` (675,980 bytes), signed with the same key.
  - Replay of route 79, segments 0-10, at 20 Hz: before the fix, no pose from 4956 s to the end. After it, poses run without a break from 4945.5 s to the end (5069 s), with zero "no heading" ticks; 527 ticks were filled by `StopHold`. The first 126 s of the drive (4426-4552) still have no pose, because no source had a fix at all.
  - Not checked: on the car and the head unit. Why `events.log` was missing on the head unit is not confirmed; the new error.log status line will say.

### 2026-11-01
- **Zoomed in, the dash map's camera tilts further toward the horizon for a more 3D look, and eases back to today's tilt as the view zooms out. Cruise is unchanged.** *static*: desktop renders and tests only, not driven, GPU path not run.
  - Problem: Peter's immersive-nav request asked for a steeper camera tilt in the close-up, like Google Maps' immersive view. The camera always looked down at 55 degrees.
  - What it does: the camera looks down 48 degrees (`CloseUp.CLOSE_PITCH`) at a displayed range of 180 m or less (`TILT_SHOW`), and 55 degrees from 260 m (`TILT_HIDE`, the close-up look's `ROAD_HIDE`), with a smooth curve between. The tilt follows the displayed range, so it glides with the zoom's own easing (`TAU_IN` 0.8 s, `TAU_OUT` 2 s) and never jumps: in tests it moves at most 0.13 degrees per metre of range.
  - The camera stays in the same place, so the raised decks (`CloseUp.lift`) are still drawn correctly. The focal length and centre row change with the tilt so that the car stays at (369, 180) and the road is as wide at the car. Only the road ahead looks more laid back: the top edge now sees about 191 m ahead instead of 177.
  - The lane row's lift and zoom (option B, `Lanes.AT_CAR`) is untouched: it is a screen shift and a range change on top of this.
  - Code: `CloseUp.pitch(range)` and `CloseUp.camera(pitch, out)` are the one camera; `MapView.cameraValues()` is now `camera(PITCH)`. Each `Frame` carries its tilt and camera values, set in `frame()` only when the tilt changes, so no allocation per frame.
    - The GPU layer copies the frame's values into its uniforms each frame and places its labels with them.
    - The UI thread (the software map, label choice, the car's outline under a deck) switches to a frame's camera with `useCamera`.
  - Static:
    - `test/TiltTest.java` is new, with 17 checks, all ok. At 55 degrees the camera is bit for bit the old one. Ranges from 260 m up give today's tilt and 180 m or less gives the close-up tilt. The tilt only moves one way, with no jump, and is half way at 220 m. The car stays at (369, 180) with the same width per metre at every tilt. The top edge sees about 191 m. A lifted deck point lies on the camera's ray at the new tilt.
    - The other tests are ok and `build.sh` builds.
    - Renders in `/tmp/hu/under/r2_before` and `r2_after`:
      - The 300 m and 500 m frames are byte-identical to before, apart from the arrival clock text, which also differs between runs of the old code.
      - The close-up frames and the 192 m frame (part way) are tilted.
      - Path and op counts are the same in every frame, and draw time is within noise.
    - Side-by-side in `/tmp/hu/under/compare.png`.
  - Not checked:
    - The GL map on the head unit.
    - The car and its outline in the slow layer (drawn with the GL layer's frame) may sit a frame off the tilted GL map while the tilt eases.
    - In free drive at 48 degrees, the top corners of the view reach past the drawn box above row 21. They are bare there, under the fog, which is about 95% dark at that height.

### 2026-11-01
- **The dash map shows railroad crossings as a yellow round "RR" sign, like the traffic lights, and draws the tracks under the roads when zoomed in.** *static*: desktop renders and tests only, not driven, GPU path not run, no real Mapbox crossing seen.
  - Problem: Peter wants a railroad crossing sign on the dash map, and the rail lines if the tiles have them.
  - Head unit: `Feed` reads tile format 6 (the rail table; `VERSION` = 6, so the comma sends format 6 to this app). `Signals` takes point kind 3 as a crossing; crossings cluster only with crossings (a double track is one sign), and the next crossing ahead is drawn big, like the next light. The cap of 6 signs is shared with the lights. `SignalIcon` adds sprites 4 and 5: a yellow disc with a black X and "R R" (the US advance warning sign, W10-1).
  - Rails: only when zoomed in (the road gate, `ROAD_SHOW` 220 / `ROAD_HIDE` 260, so cruise is unchanged). `CloseUp` cuts each rail into pieces of at most 100 m, keeps those within 1 km, and adds ties every 8 m (2.6 m each side), at most 3072 segments. Drawn grey, under the roads, in both the software draw (`CloseView.rails`) and the GPU draw (`GlClose.rails`, called from `GlRoads`). Their draw ranges come after the road ranges, so the existing range numbers do not move. Rail bridges are drawn flat under the roads, not raised; tunnels are dropped on the comma.
  - Fix on the way: `CloseUp.cap` could keep one more segment than its cap when several had the same distance at the cut; it now counts the ones below the cut first.
  - Static: `test/SignalsTest.java` (crossings, crossing-only clusters, next crossing) and `test/CloseUpTest.java` (long rail kept in pieces, ties across the rail, tie count, rail ranges after the road ranges, rail cap) ok, the other tests ok, `build.sh` ok. Desktop software draw timing: cruise frames unchanged (same paths and ops); zoomed-in frames with rails ~0.6-3.7 ms more, mostly the ties. Renders in `/tmp/hu/rail/` (free drive with a crossing 75 m ahead, a route over a crossing, its close-up, before/after).
  - Not checked: the GPU rail draw on the head unit, the Tegra cost of the ties when zoomed in, whether the small sign's "R R" reads at 1x on the cluster, and how many US crossings the Mapbox z14 tiles carry.

### 2026-11-01
- **Driving under a bridge or overpass in the close-up, the dash map shows the bridge over the car and a white outline of the car through it, as Google Maps' immersive view does.** *static*: desktop renders and tests only, not driven, not run on the head unit's GPU.
  - Problem: Peter asked for an outline of the car when it goes beneath a bridge or overpass. Until now the car sprite was drawn on top of every deck, so a bridge looked like it ran under the car.
  - What it does: new `Under.java` (pure Java) finds the raised decks drawn over the car's body. The car's own height is the highest raised segment running its way (within 30 degrees) that holds it (half the road's width plus 6 m), else the ground; a deck counts as over the car when it is at least half a deck higher and its drawn (lifted) footprint reaches the car's body. So on the bridge itself there is no outline, and on a bridge under a flyover there is one.
  - Only in the close-up: the effect is multiplied by the close-up gate (`ROAD_SHOW` 220 to `ROAD_HIDE` 260 m), so cruise draws exactly as before. It eases in and out over 150 ms (`Under.Fade`, turning back part way at a deck edge as the brake glow does), and keeps the last decks while fading out so the edge does not flicker.
  - Drawing (`MapView.drawCar`, both paths): the covering decks, cut to 40 display metres each side of the car, become a screen clip. Outside it the car draws as before. Inside it the car fades out and its outline fades in: a 2 px white edge with a faint fill, made once from the sprite (body = alpha 170 and up, so the soft shadow is left out). The map's own deck shows where the car is cut away, lane lines included: on the software path it is the deck just drawn, on the GPU path the GL map through the transparent slow layer. With no sprite, the fallback arrow is stroked white.
  - GPU path: the slow layer now draws the car with the GL layer's latest frame, checks every tick whether the car went under a deck, and redraws every 33 ms (as during the brake glow) while the outline shows or eases, since the deck slides by the car. Software path: `changed()` also redraws while it eases.
  - Static: `test/UnderTest.java` (new, 21 checks: under the bridge both ways, on the bridge both ways and slightly off its line, cruise gate shut, before and after the bridge, under a flyover while on a bridge, on the flyover, one stretch with no flicker driving through, the ease in, out and turned back part way) ok; CloseUp, Lanes, Signals, Shield, FreeView, Liveness, Events and GlCull tests ok; `build.sh` ok. Renders (software path) in `/tmp/hu/under/r_before` and `r_after`: approach at the deck edge, under, exit, out, and on the overpass. Every frame without a deck over the car is byte-identical to before, except the arrival clock text on the two route frames, which also differs between two runs of the old code. Desktop draw time is the same within noise (under: 9.8 vs 9.7 ms).
  - Not checked: on the head unit's GPU, the slow layer redraws at about 30 a second while under a deck (a few hundred ms in a normal pass under) and the car on it may sit a frame off the GL map; real tile decks (only the synthetic US 75 / I-635 scene). The feed has no layer for the car's road, so a car on a ground road running alongside under a viaduct counts as on the deck and shows no outline.

### 2026-11-01
- **The comma now sends the dash map the railroad crossings and the rail lines (nothing drawn yet).** *static*: desktop tests only, no real Mapbox tile read for a crossing.
  - Problem: Peter wants a railroad crossing sign on the dash map, like the traffic lights, for the few crossings near home.
  - Data: Mapbox Streets v8's road layer has point class `level_crossing` ("where a road crosses a railway on the same level") and line classes `major_rail`, `minor_rail` and `service_rail`. Its docs give no zoom for them, so whether the z14 tiles near home carry them is not measured yet.
  - Tile format 6 (`road_tiles.py`): point kind `LEVEL_CROSSING` = 3, and after the walks a rail table in the walk table's layout, a kind byte per line (1 mainline, 2 light rail or tram, 3 yard or siding, plus 0x80 on a bridge). Rails in a tunnel (subways) are dropped. At most 4096 rail lines and 32767 points a tile.
  - Older apps: `downgrade_road_tile` to 5 or older drops the rails and the crossing points, so the installed app (format 4) and the lanes app (format 5) get exactly the tile they got before. Reading the code: those apps would also skip a kind-3 point (`Signals.cluster` keeps only kinds 1 and 2), but they never see one.
  - `navtilesd` `CURRENT_FORMAT` = 6, so near tiles saved in format 5 or older are fetched once more.
  - Static: `test_road_tiles.py` (3 new tests: decode with a rail tunnel and a buffer crossing, the exact format 5 downgrade, cut and padded rail tables and the line cap), `test_navtilesd.py` (format 5 tile fetched again, with the crossing and rail kept), `test_hu_feed.py` (rt 5 gets format 5, rt 6 as is) pass; ruff clean. `test_main_subscribes_to_registered_gps_carriers` fails in this worktree because `opendbc` does not import here, not from this change.

### 2026-11-01
- **The dash map shows which lanes to be in before a turn or exit, and a "toward Garland" line under the turn, as Google Maps does.** *static*: renders and desktop tests only, not driven, no real Mapbox lane answer seen.
  - Problem: Peter wants Google Maps' lane guidance on the cluster: a row of the lanes with their painted arrows, the ones to use in white, plus the destination line under the road name.
  - Comma (`route_engine.py`): each step takes the lanes for its maneuver from the banner's `sub` lane parts, else from the next step's first intersection `lanes`, left to right. Each lane is its arrows as bits (straight 1, slight right 2, right 4, sharp right 8, uturn 16, sharp left 32, left 64, slight left 128, merge left 256, merge right 512; "none" counts as straight), whether to use it, and the arrow to light (always one painted on that lane). More than 12 lanes, or any lane with an unknown or malformed arrow, drops the whole row. "toward" comes from the banner's secondary text, else the step's `destinations` (after the colon), up to 80 characters, and is left out when the road name already says it.
  - `custom.capnp`: `Navigation.Instruction` gets `lanes @11 :List(Lane)` and `toward @12 :Text`. `hu_feed` adds `"lanes": [[arrows, use, lit], ...]` and `"toward"` to the dash step only when there are any, so an older app sees nothing new. `guidance_hold` keeps both through a GPS gap.
  - Head unit: new `Lanes.java` (pure Java: which arrow lights, when the row shows, the arrow shapes for straight, slight, turn, sharp, U-turn and merge, mirrored U-turn where traffic keeps left). The row shows only with lanes and within 1 km of the maneuver. `MapView` draws a "toward X" row on the turn card and the lane row, lanes to use in white and the rest dim grey, on a dark rounded strip.
  - Two places, one switch (`Lanes.at`): **A** a lane card under the turn card (the song card is skipped while it shows if it would not fit); **B** a strip under the car, with the map and the car eased up 38 px over 0.35 s and the view zoomed in 15% to make room. The lift goes through the software draw, the GPU projection (`GlRoads.projection`), label blocking and the slow tick.
  - Static: `test_route_lanes.py` (new, 10 tests), `test_hu_feed.py`, `test_guidance_hold.py`, ruff clean; `test/LanesTest.java` ok; `build.sh` ok. Renders in `/tmp/hu/lanes/`: highway exit at 800 m with 2 of 4 lanes lit and "toward Garland", a city left at 300 m, both places each, and the same exit at 1.5 km with no row. APK: `/tmp/hu/aaroad-lanes.apk`.
  - Not checked: the GPU lift is not run on the head unit (desktop renders use the software path); during the 0.35 s ease the car on the slow layer may be a frame off the GPU map; `Feed.Nav.of`'s lane parsing is not run on the desktop (the JSON stubs are dummies).

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
- **Zoomed in, the dash map draws raised overpasses, motorway lane lines, sidewalks and crosswalks.** *static*: desktop renders and a GL compile check only, not run on the head unit's GPU, not driven.
  - Only when the view is zoomed in: roads under a 220 m range (fully off by 260 m), walks under 130 m (off by 160 m). Free drive that is under about 18 m/s for roads and 11 m/s for walks; with a route, when the view closes in on a turn. Cruising is unchanged: the free-far and nav-cruise renders are byte-identical before and after.
  - New `CloseView.java`: the draw passes and the software (Canvas) draw. Per deck layer 1-3: a soft shadow on the ground, a dark side wall (4 samples between the ground and the deck), then casing, kerb, surface and the lane lines (dashed between lanes, solid edges, yellow centre). Ground lane lines first, sidewalks and dashed crosswalks under everything.
  - New `GlClose.java`: the same passes on the GPU layer (`GlRoads`), with its own program (the road shader plus a lift, a width scale and dashes) and two buffers. A new scene is sent 1024 segments a frame while the old one draws. If the program does not build, the map draws without the close-up look instead of failing.
  - Height 24 m a layer (was 12): at 12 m a deck moved only 5-7 px in the renders and did not read as raised. It is exaggerated on purpose.
  - Cost: at most 10240 segments, 1.47 MB on the GPU; the busy test scene is 226 KB. Software draw on the desktop (50 reps): close frames 10-12 ms, was 5-8 ms; cruise 17 ms, about as before. The close frames stay under the cruise cost. GPU timing on the Tegra 3 is not measured.
  - Not done: the carpet and the car stay at ground level on a deck; lane counts are US defaults (Mapbox has them only in Japan); walk layers are ignored; ramps come only from line ends, not mid-line junctions.
  - Static: `test/CloseUpTest.java` all ok (it also checks the GPU corner packing). Renders: `/tmp/hu/walk3d/compare.png`. APK: `/tmp/hu/aaroad-walk3d.apk`.

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

## Car outline under overpasses removed (Peter 2026-11-08)
Peter: the outline of the car under a highway was hit or miss, and sometimes showed while he drove on the bridge
itself; he is fine with it gone. `MapView.UNDER_OUTLINE = false`: `underDeck` returns 0 and `underWanted` false, so
the car always draws whole on top of any deck and the slow layer no longer redraws for it. Raised decks themselves are
unchanged. Built as /tmp/hu/aaroad-nooutline.apk (static only: build passes; not yet installed).

## Bridge seams at ramp/flyover joints (Peter 2026-11-08)

Reproduced offline on Peter's drives a6, a9 and aa (San Antonio interchanges, Konik GPS + real Mapbox tiles,
harness /tmp/hu/jvmtest/hbridgeR). Two causes, both where bridge ways with different OSM `layer` tags meet end to end:
- Cruising view: CloseUp.overpasses joined bridges into one raised-or-flat group only on the same layer, so one piece
  of a deck could be drawn raised (cased) and the next flat. Now a chain joined at any layer is raised as a whole;
  the road being driven still stays down per same-layer group. Renders: the motorway deck at aa_06 / under_aa_02 reads
  as one piece instead of capsules with rounded ends.
- Close-up: a bridge end meeting a bridge at another layer counted as a free end, so both decks ramped toward the
  ground at the joint. Now the lower deck rises to the higher one there and draws in its group.
Static evidence only (desktop renders); not yet seen on the car.

## Dash map lag: needless redraws (Peter 2026-11-08)
Peter: the dash map felt laggy. Drive 01:43-02:08: the GPU map ran 14-18 fps (21-29 the night before), its pure-math
camera step stalled 300-860 ms in every 10 s window, and the map tick went quiet for up to 142 s. A stack dump showed
no deadlock: the map process's main thread was busy drawing the window in software and the GPU thread waited for CPU
longer than it ran. Parked with the car on, the main thread still used ~28% of a core: the slow layer redrew the whole
view 4-6 times a second at 31-37 ms each with nothing moving. Two causes in the code:
- A long song title scrolls at 25 px/s, and each scroll step re-rendered the cards' bitmap and the whole view, up to
  10 times a second. Now the scrolling lines are drawn live over the cached cards, and a scroll step redraws only
  their strip (`invalidate(rollBox)`). The progress ring re-keys the cards every 3 degrees instead of every degree.
- The comma's state message (1 a second) forced a redraw by its arrival time even when nothing in it changed. `stamp()`
  now hashes only what is drawn (metric added, since the sign uses it).
Also the "cluster guide reply" log line (14,637 in one drive, the reply flips between a few values about 4 a second)
is now written once per distinct reply. The map draw log line adds "N song strips".
Static only: build passes, harness renders identical to before except the ETA wall-clock minute, also with a long
scrolling title. Next drive: check "view N redraws/s" (expect ~1 or less without a scrolling title), fps and tick age.

## Dash map lag with navigation on: GPU culling and draw batching (Peter 2026-11-08)
Peter: the map still lagged drawing with navigation on. Head-unit log, 13:29 drive (*limited road evidence*): lag began
when navigation started (13:29:18, qlog); cull went from 1-8 to 15-21 ms a frame, sleep fell to ~0, fps 28 -> 19. The
route adds many scattered pieces, so the per-frame cull and the number of small draw calls both grew.
- `GlCull.update()` culls 10 times a second over a widened margin box (3-90 display m), re-culling early only when the
  view leaves it; pieces are ordered along a Z curve (64 m cells, `MORTON`) so neighbours draw as one run.
- `GlRoads.lines()` draws each run with one `glDrawElements` from a page offset; attribute pointers are set once per
  16384-quad page instead of per run.
- Minor streets (class 4) fade out when zoomed out (`LOD_MINOR`, ik 0.18 -> 0.15) and their labels hide with them
  (Peter: "hide side streets if needed").
- Hook logs the `street` line only when the street changes. "gl parts" adds roads cand/runs/draws/ptr/quads, culls
  (early), rebuilds and ik.
Replay (`test/MapReplayBench.java` over the real drive, desktop): draws/frame 280 -> 109, GL calls 840 -> 111, runs
138 -> 53, cull time about halved; superset check 0 misses in 53,978 frames (negative control 13,303). Tests and build
pass. Installed 15:15; first head-unit reading 15:28, parked, no route: cull 0.4 ms, 18-19 draws, ~29 fps. Next nav
drive: compare "gl parts" fps, draws and cull with the 13:29 numbers. After a comma reboot the map took ~7 min to show
(first fresh position; cause not proven).
