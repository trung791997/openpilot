# Hybrid off-policy / on-policy lateral gain training for the owner's Civic (C020 modified EPS)

Branch: `sim-lat-training` (created 2026-09-26 at 09cf8421e; the Mac sim work lives here, apart from the OP fork
branches). Supervisor: Claude (architecture, gates, audit, MetaDrive runs, docs, commits). Grunt work: agy workers
with disjoint paths. Everything produced is **sim / replay evidence only; no device behaviour change** until the owner
drives a proposed schedule.

## 1. What is being tuned, and why "RL" here means episodic policy search

The deployable artifact is a small set of live-read lateral params, not a network:
`LatGainSchedule` (`{"v_mph":[20,30,40,50],"p":[..],"i":[..],"f":[..]}`, percent of the C020 table, parsed by
`parse_lat_gain_schedule` in `selfdrive/controls/lib/latcontrol_pid.py`, re-read by the controller every 300 frames).
A neural steering policy could not be driven on the car and would have no safety case. So "RL" = **episodic
black-box policy search (cross-entropy method, CEM)** over the schedule knots, with the reward measured on driving
episodes. Two episode sources:

| source | what it is | strength | weakness |
|---|---|---|---|
| **off-policy (real routes)** | `tools/lateral/lat_pid_sim.py sim`: the real `LatControlPID` follows the *logged* desired curvature of the owner's drives through a plant fitted from the same drives (delay 5, C020 firmware torque table) | real curves, real speeds, real EPS; validated open-loop replay; ~seconds per episode | desired curvature is exogenous (no lane-position closure); under-predicts oscillation; one plant = one guess |
| **on-policy (MetaDrive)** | `tools/sim/run_mac_tsfdo.sh` with `SIM_CAR_CONFIG` (the car's own params), `SIM_MODEL=stock`, `SIM_STEER_MODEL` (fitted EPS response) | closes perception + planning + paramsd around the real controlsd; can go off the road | MetaDrive roads are far tighter than the owner's (baseline 2026-09-26: mean desired 17 deg vs 9 real, err rms 4.4 vs 1.4 deg, lag 0.35 s vs 0.0); one episode = 3 min wall clock |

**Assessment of the owner's proposal** (train on real footage, hybrid on/off policy): right idea, two corrections.
(1) "Footage" cannot be used as pixels: the lateral tune sits below the driving model, so the real drives enter as
logged desired-angle trajectories plus the fitted plant. The repo already has that harness (`lat_pid_sim.py`,
`lat_autotune.py`, STATUS 115/147: coordinate search proposed p 100/130/125/105, i 50/95/75/75; only the 25-50 mph
band trusted; never driven). (2) A single fitted plant overfits. The upgrade is **domain randomisation**: score every
candidate on an ensemble of perturbed plants (command delay -1..+2 frames, gain x0.8..1.2, damping x0.8..1.2) and
optimise mean + 0.5 x spread, so the winner is robust to the plant being wrong, which it is. MetaDrive is kept as the
**stability gate and mistake generator**, not the main reward: each MetaDrive episode is recorded in
`lat_pid_sim` route format and appended to the off-policy pool (weight 0.25, flagged `sim_route`), so a departure or
a hunting episode becomes training data for the next CEM round. That is the hybrid.

## 2. Frozen contracts

- **Routes (real, off-policy):** rlogs under `/tmp/epsfit/<route>/<seg>/rlog.zst` (8 C020 segments, fetched via
  `tools/konik_fetch.py`; route IDs cited in TSFDO.md; never committed; delete after). More segments of routes
  271/276/277/278/27a may be fetched the same way for a held-out set.
- **Plant:** `/tmp/latplant/plant_d5_fw.json` = `lat_pid_sim.py fit /tmp/epsfit/* --delay 5 --eps-rwd
  "eps_tools/rwd/39990-TBA,C020-20260805-...Pclamp7373.rwd"`. Plant ensemble = the fitted plant plus perturbations
  above, 6 members per generation, fixed RNG seed.
- **Seed policy:** the car's driven settings from `~/.openpilot-sim/civic/params.json` (`LatGainSchedule` if set,
  else the `Lat{P,I,F}Scale{LowSpeed,Standard,Highway}` bands evaluated at the knots).
- **Search vector theta:** p and i at knots 20/30/40/50 mph, 8 dims, percent; f frozen. Bounds p 25..300, i 0..300,
  trust region +-40 points from seed. CEM: population 24, elites 6, 6 generations, init std 15, floor std 2,
  smoothing 0.7. Alternate 2 min blocks: even = fit, odd = holdout (same rule as `lat_autotune.py`).
- **Cost (lower is better), per route x band x plant, minutes-weighted, only trusted bands
  (reuse `lat_autotune`'s trust gate):** `err_rms + straight_rms + 5*|curve_ratio outside 0.97..1.03|
  + 2*max(0, sign_change_rate/seed_rate - 1.10)`; candidate cost = mean over plants + 0.5 * std over plants;
  `sim_route` episodes weighted 0.25, plus 5.0 if the episode left the road. Reported as ratio to the seed.
- **Verdict:** recommend only if holdout cost ratio < 0.97 on every plant member and no trusted band worse than 2 %.
- **Results file:** `<out>.json` = `{seed, generations:[{mean, std, elites, best_cost}], best_theta,
  holdout: {seed_cost, best_cost, per_band}, plants, routes, sim_routes}`; plus the `LatGainSchedule` JSON string to
  paste. Not committed (route IDs only in docs).
- **MetaDrive episode:** `/tmp/runsim.sh NAME 120 SIM_MODEL=stock SIM_STEER_MODEL=$PWD/tools/sim/eps_models/
  honda_civic_bosch_c020.json`; overrides applied ~10 s after launch by `tools/sim/sim_set_overrides.py
  --prefix tsfdo-mac k=v ...` (controller re-reads every 300 frames); recorded by `tools/sim/sim_lat_record.py OUTDIR
  SECS` into `OUTDIR/lat_pid_sim.npz` with exactly `lat_pid_sim.FIELDS` at controlsState rate plus `cp_bytes`
  (carParams) and `params` (JSON of `TUNING_KEYS`), so `lat_pid_sim.extract(OUTDIR)` loads it unchanged.
  Gate per candidate: 2 episodes, no departure, `tools/sim/lat_metrics.py` ge25mph rms and osc not worse than the
  baseline by more than 10 %.

## 3. Tasks

| # | task | owner | paths | verify |
|---|---|---|---|---|
| T1 | plant fit + real-route baseline metrics (`lat_metrics.py` on the 8 rlogs) | supervisor (one command each) | /tmp only | validate report |
| T2 | `tools/lateral/lat_cem_tune.py` (+ `tests/test_lat_cem_tune.py`): CEM per §2, reusing `lat_pid_sim.load/simulate/metrics` and `lat_autotune` trust/cost helpers; `-j` multiprocessing; `--sim-route DIR` (weight 0.25, `offroad` flag from `OUTDIR/episode.json`) | agy-pro (Tier 2) | those two files only | `ruff check`, `pytest tools/lateral/tests/test_lat_cem_tune.py` on synthetic data (no routes) |
| T3 | `tools/sim/sim_lat_record.py` and `tools/sim/sim_set_overrides.py` per §2 (+ `tools/sim/tests/test_sim_lat_record.py`: synthetic samples -> npz has all FIELDS, equal lengths, loads via `lat_pid_sim.extract`) | agy-flash (Tier 1) | those three files only | `ruff check`, pytest on the new test |
| T4 | run CEM on real routes (off-policy round 1) | supervisor | /tmp | results JSON |
| T5 | MetaDrive: baseline x2, top-3 candidates x2; record as sim routes | supervisor (GUI/tmux) | /tmp | gate in §2 |
| T6 | CEM round 2 with sim routes appended; final verdict | supervisor | /tmp | verdict rule |
| T7 | TSFDO.md + STATUS entry (sim evidence only), commit on `sim-lat-training`, delete /tmp route data | supervisor | docs | — |

```json
{"modelTier": {"T2": "standard", "T3": "mechanical"}}
```

## 4. Baselines (2026-09-26)

Real, route 27a seg 1 (`lat_metrics.py`): <25 mph rms 3.77 deg, >=25 mph rms 1.44 deg, lag 0.00 s, osc 0.14/s.
MetaDrive stock+torque episode 8: >=25 mph rms 4.40 deg, p90 7.6, lag 0.35 s, osc 0.74/s, rate rms 21 deg/s,
mean |desired| 17 deg, 119 s engaged, no departure. Pooled 8-route real numbers: see TSFDO.md once T1 finishes.
