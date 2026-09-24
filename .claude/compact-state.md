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

## 4. Immediate Next Step
- None. The 3-band rework is done and pushed (e0ba9af8a, 7c21a3da7, 41d0813bf). Band re-run on 262-26b proposes no change. Wait for the owner.
