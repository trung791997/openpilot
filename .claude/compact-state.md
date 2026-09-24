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

## 4. Immediate Next Step — REWORK to StarPilot's 3 speed bands (owner request 2026-09-24)
Owner: "make sure it matches the current available tuning settings on StarPilot for PID tuning: 3 speed bands 0-25, 25-50, 50+, each with a P/I/F scale; see reverted PR #6."
Facts found:
- Bands: `latcontrol_pid._lat_pid_scale_banded(v)`: v < 25 mph → LowSpeed, v < 50 mph → Standard, else Highway (hard steps, no blending). Params: `LatPScaleLowSpeed/Standard/Highway`, `LatIScale*`, `LatFScale*` (INT percent, PERSISTENT; Galaxy range 0-500, step 5). manager.py defaults P: 100/135/200.
- `LatGainSchedule` (STRING) overrides the bands per term when set (`lat_gain_schedule_scale`). The car currently has it empty (not in the logged tuning).
- STATUS 116 (PR #6) explains why only P is trimmed; I/F are shown and kept.
Decided design (owner's intent is clear; no new Params keys):
1. `lat_tune_analyzer.py`: replace KNOTS 20/30/40/50 with BANDS [("LowSpeed",0,25),("Standard",25,50),("Highway",50,inf)]. Bin frames hard by band (as `_lat_pid_scale_banded` does; replace triangular `_weights`). Keep the item-116 rules per band: ready ≥3 min, one 0.05 factor step, 0.85-1.15, neighbours ≤0.10, straight <3°, curve >5°, sign-rate, press-rate vetoes. Proposed band P = baseline band P% × factor, rounded to the Galaxy step 5, but never letting rounding cancel the step (at least ±5 in the step direction). Trial JSON: `bands` [{name, lowMph, highMph, minutes, ready, signRate, curveRatio, pressRate, factor, reason, current {p,i,f}, proposed {p,i,f}}]; I/F proposed = current. Bump SCHEMA_VERSION. Drop build_schedule/_interp_at_knots and add `build_band_params(trial)` → {"LatPScaleLowSpeed": int, ...}.
2. `lat_tune_workspace.py` apply: record prior values of the 3 LatPScale* + LatGainSchedule; write the 3 INT band params; if LatGainSchedule has a "p" term, remove "p" (clear the param if nothing is left) so the bands take effect. Revert restores all 4 exactly. Fingerprint gate, stack, offroad-only unchanged.
3. CLI prints a band table with current P/I/F → proposed P and paste lines `LatPScaleLowSpeed = N` etc.
4. The classic `lat_tune.js` and mobile `NrdrLatTunePanel.js` show bands (0-25 / 25-50 / 50+ mph) with current P/I/F and proposed P; update the confirm text ("writes LatPScaleLowSpeed/Standard/Highway").
5. Update tests (analyzer 29, workspace 10, API 3, CLI 5, test_ui_vue_frontend), render-check both UIs (scratchpad render_srv.py / render_mobile.py, mock data → bands), re-run the CLI on the 8 routes (retrieve: `python3 tools/konik_fetch.py --route '11c8fa231c0499ed|<route>' --out <scratch>` then tar into the volume at /routes/konik; see scratchpad fetch8.sh; the count compare must strip whitespace), update STATUS 117 with the band result (replace the 4-knot table), commit and push both branches, delete route logs and fstrim.
- Previous 4-knot result (to be superseded): +5% at 40 mph only; 20/30/50 held; LatIScaleStandard differed across routes (25 vs 75).
