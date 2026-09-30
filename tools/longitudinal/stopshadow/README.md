# stopshadow-radar

Working branch for the stopped-track shadow test (plan: Radar Work (Bob), `/tmp/rv/stopshadow/PLAN.md`), run by
Radar Work (Bob), Radar Knowledge (Jason) and UI Work + Tool Design (Steve). Branched from ns-bosch-radar-testing at
ae6ae25e8 (the Bosch-A track tool the corpus run is pinned to).

Peter, 2026-09-30: plots may go on this branch; nothing merges into ns-bosch-radar-testing until the work is verified.

- Evidence here is replay only. No car-side code changes on this branch.
- Route data is cited by route ID and never committed. Plots and small CSV summaries are fine.
- The shared checkout stays on ns-bosch-radar-testing. Commit here without checking out with
  `/tmp/stopshadow_commit.sh "<message>" <repo-path>=<local-file> ...` (temporary index + commit-tree).

## Status (2026-09-30)
- Yaw sign of the stationary reference -vEgo + w*yRel: settled on replay (dy/dt follows -w*x).
- D: no mount-angle offset detected; turning-case b ≈ +0.3 m/s likely selection, open. Lever arm L ≈ 3 m on 00000297.

## Closed items (2026-09-30, replay only)
- **Stopped-flag lead onset: negative.** The flag (0.5 s window, 0.5 s hold) adds no lead radard lacked. On 258 tid 26 and 297 tid 56, lead one
  already had the track via vision. 26b tid 16 was a correct hard-width rejection and a flag false positive. See `adoptmiss.txt` and `followup.txt`.
- **Option 2, camera car-vs-clutter for ONPATH_RADAR_ADOPT: negative.** No gate is added. The FP's effect stays capped by ONPATH_LEAD_MAX_BRAKE (D-048). See `clutter.txt` and `clutter.csv`.

  | cue | 297 roadside FP (tid 42) | true onsets | separates? |
  |---|---|---|---|
  | road-edge margin | inside, +2.51..+2.63 m | lowest 297 tid 6 +2.70 m | no (0.2 m, one case) |
  | ego lane-line margin | inside, +1.15..+1.43 m | 297 tid 6 outside at times (min -1.74 m) | no (inverts) |
  | model lead prob | median 0.04, max 0.10 | 297 tid 6 median 0.05, max 0.38; straight-road TPs 0.59-1.00 | no |
  | roadEdgeStd, object side | 2.47 | medians 0.35-1.69 | gap on one FP; road-level, untested on negatives, not a gate |
- **Path-end NaN (radard.py:1446) is latent, not fixed.** The path offset is computed only when dRel <= model position.x[-1]. Beyond that it is NaN and
  clears ONPATH adopt's history. Over 19 routes there are 38 episodes (6 of them >= 1 s) where a stopped track beyond the path end had no lead one covering it.
  Open case: 00000284 tid 36 at 600.9-602.5 s (42 -> 37 m at 3.4 m/s, flag only, ego already stopping). Curve-at-speed is not ruled out. See `pathend.txt`.
- **Time base:** the radard.py:368 comment's "30:18.1" roadside FP is 00000297 tid 42 at 1824.78 s on our clock (t = logMonoTime - seg-0 initData),
  and "31:08" is 1874.03 s. That is a 6-7 s offset. radard.py is not edited for this.
- **Option 1, NC-at-rail** (Jason, c40fe684f / 9b48429a4): on railed, stopped, < 50 m rows NC is present on 100% and sigma < 32 on 99.7%. See `sigma.txt`.
  The on/off replay is pending (`ncrail.txt`).
