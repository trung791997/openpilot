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
| Installed now | `aaroad-slim-timing.apk` | slim cards + frame-part timing (`ace790491`), installed 2026-10-31 evening |
| Previous | `aaroad-gl30.apk` | GPU map at 30 fps, glitchy while driving |
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

## Log

### 2026-10-31
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
