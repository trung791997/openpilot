# Checkpoint Manifest

**Timestamp:** 2026-09-24T18:10
**Branch/Worktree:** claude/radar-testing-state-88vt2t (== ns-bosch-radar-testing), primary checkout /Users/peternguyen/openpilot-radar
**User Goal:** Plan `docs/superpowers/plans/2026-09-24-lat-tune-workspace.md` (STATUS 117, Lateral Tune workspace + off-device CLI) — COMPLETE and pushed.

## 1. Files Touched & Key Symbols
- `selfdrive/controls/lib/lat_tune_analyzer.py` — item-116 rules, `build_trial`, `build_schedule`, `effective_p_pct`, `FrameSource`, `RouteLog`, `analyze_sources` (101ab983b, 1cd30d0fe)
- `tools/lateral/lat_tune_cli.py` — latest ≤8 route dirs → paste-ready `LatGainSchedule` (1cd30d0fe)
- `starpilot/system/the_galaxy/lat_tune_workspace.py` — trials, detached worker, apply/revert stack (492df0aa1)
- `starpilot/system/the_galaxy/the_galaxy.py` — `/api/lat_tune/*` routes (036973475)
- `starpilot/system/the_galaxy/assets/components/tools/lat_tune.{js,css}` + router/sidebar/index.html (0d610a14f)
- `STATUS.md` item 117 (01a9f243e)

## 2. Test Verification Matrix
- **Command:** per file, docker `oprad-test:py312` with `pip install flask pillow` in the throwaway container, `-p no:unraisableexception`
- **Status:** PASS — analyzer 29, CLI 4, workspace 10, API 3, longitudinal_mode_api 20, frontend_module_graph 11, device_settings_layout 29, lat_gain_schedule 26; latcontrol 171 + 2 known base failures; dashboard_stats 70 + 2 pre-existing failures (confirmed on stash).
- Render check: static, real assets + mocked API in headless Chromium; text-color defect found and fixed.

## 3. Settled Decisions & Invariants
- No new Params keys; apply writes only `LatGainSchedule`; analyze/apply/revert offroad only; one factor step per trial; item-116 thresholds not retuned.
- Evidence level: unit-test/static only, not driven, no real-route replay yet.

## 4. Immediate Next Step (updated 2026-09-24 late)
Pushed and done: 3-band rework e0ba9af8a, 7c21a3da7, 41d0813bf (STATUS 117 band re-run: all bands hold; Standard held by override veto 2.10/min > 1.5).

UNCOMMITTED (RED: 6 analyzer tests still expect the fixed 0.05 step):
- `selfdrive/controls/lib/lat_tune_analyzer.py`: MAX_STEP=0.15 (owner: "up to 15 % per trial"), MAX_NEIGHBOUR_GAP 0.15, `curve_step(cr)` = clamp(round(|1/cr-1|/0.05)*0.05, 0.05, 0.15) for curve up/down steps; sign-rate down stays -STEP; reverts undo `-prev["stepped"]`; `analyze_sources(..., baseline_overrides=None)` what-if baseline (+ warning, fingerprint follows override).
- `tools/lateral/lat_tune_cli.py`: `--baseline KEY=N` (Lat[PIF]Scale* only) + CLI test.
- Failing tests to update: test_short_drive_holds_and_carries, test_overshoot_steps_down (1.1 -> 0.90), test_revert_after_increase_that_raised_oscillation_only_in_apply (up 0.10 with cr 0.9; non-apply 1.10 -> 1.15 cap), test_one_step_per_drive (cr 0.5 -> 1.15), test_build_trial_reports_bands... (Standard 1.10, P 110), test_build_band_params_writes_only_p (Standard 110).

NEW FINDING (owner pasted another chat's analysis): with NrdrDriverOverrideThreshold 2000, short torque-sensor blips (<=0.2 s, peaks 2001-2394) trip steeringPressed: 41 of 62 override onsets on the 48-min drive were blips. My press-rate veto counts every steeringPressed onset, so Standard's 2.10/min and LowSpeed's 16.8/min are inflated by false overrides. The other chat recommends threshold 2400 + fade-up 0.5 s first, then LatPScaleStandard 115 (sim), recorded as STATUS 112 by the other agent on the same branch -> `git pull` before committing.

Proposed to owner (awaiting reply; do not start before they answer unless they say go):
1. Press-rate veto counts only presses lasting > 0.2 s; exclude the ~1 s fade-in frames after an override from curve/straight metrics.
2. Keep the 15 % step change (fix the 6 tests, commit).
3. Re-run on routes 262-26b + the new 48-min drive (newer than 268; get its route id from STATUS 112) via scratchpad run_bands.sh (fetch8.sh; add route; optional `--baseline LatPScaleStandard=115`).
4. Recommend setting override threshold/fade first.
Alternative if owner insists: just run the 115 what-if.
Runners: scratchpad t.sh <test file> (analyzer, CLI), tf.sh (galaxy). Commits `YYYY-MM-DD ...` + Co-Authored-By trailer; push both branches; delete route logs + fstrim after.

## 5. Update 2026-09-24 (after /compact)
- git pull: nothing new; STATUS 112 already in tree (line ~6135) = route 0000026b--92b1979afa, the owner's "best drive yet" — the "48-min drive" IS 26b, already in the 8-route set. No extra route to add.
- Route logs were deleted from the oprad-routes volume (only 0000025b and an2/ remain) -> refetch needed (scratchpad fetch8.sh pattern; oprad-routes/konik tooling).
- OWNER (after compact): "The 32:40 one is on route 26b, feel free to take a look at that one." = the other chat's open question: the car went LEFT toward the other car. Segment 32, t~40 s (26b--32). Owner said resume => go on the plan.
- Next, in order:
  a) Fetch rlog 0000026b--92b1979afa--32 (+31/33 for context). Decode 30..50 s: carState.steeringPressed/steeringTorque, controlsState lateral pidState (p,i,f,output, desired vs actual curvature/angle), modelV2 path y & lane lines (laneLineProbs), vEgo, blinkers, lateral fade after override, NrdrDriverOverrideThreshold. Decide: PID/override-fade cause vs model path/lane-line cause. Report to owner in plain words (limited log evidence, not driven).
  b) Blip filter in analyzer press-rate veto (presses > 0.2 s only) + exclude ~1 s post-override fade frames; fix the 6 red tests; commit; push both branches.
  c) Re-run run_bands.sh on 262-26b (refetch), also with `--baseline LatPScaleStandard=115`; update STATUS 117; push; delete route logs + fstrim.
