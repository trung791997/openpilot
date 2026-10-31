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
| Installed now | `aaroad-gl30.apk` | GPU map at 30 fps, glitchy while driving |
| Ready to install | `aaroad-slim-timing.apk` | slim cards + frame-part timing (`ace790491`) |
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

## Open

1. **Map glitch while driving.** The head-unit log shows GL frames of 32–43 ms (20–24 fps), with worst frames of 0.23–0.38 s that match the freeze in Peter's video. It is the map's own frames, not GPS. Card redraws take 35–55 ms each, 2–6 times a second, and compete for the CPU.
   - **Next:** install `aaroad-slim-timing.apk`, drive about 10 min, and read the `aaroad: gl parts avg/max ms:` lines to see which stage is slow.
   - **Suspects:** card redraws, `new int[2]` per frame in `GlRoads.drawFrame`, and `new Frame()` per frame in `MapView.frame()`, which could cause GC pauses.
2. **Slim cards** (`a9da6e81b`): static only, so Peter has to see them on the dash. Song names cut off sooner now, so maybe scroll them or widen that card a little.
3. **Double X on the StarPilot Auto car screen** (`2299affae`): fixed, but the comma has not been updated yet. Update it when parked (standing rule).

## Log

### 2026-10-31
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
