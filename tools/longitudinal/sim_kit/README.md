# sim_kit: Bob's closed-loop longitudinal replay, packaged to run on another machine

Replay only. Nothing here is road evidence. Report results as "replay" (AGENTS.md).

## What is here
- `closed_loop.py`: the replay harness. For each event window, every candidate planner drives its own fitted car plant from T0,
  fed by the logged radar and model. It writes a pickle of per-frame rows.
- `car/longitudinal_planner.py` (+ `longitudinal_lead.py`, `blotv3.py`): the planner that was in the car for 2a6/2a4
  (build 87505f426). This is the default `CAR_LP`.
- `car/longitudinal_planner_d073.py`: the same planner plus D-073 (`EXP_CLOSE_RATE` env, default 1.5). Set `CAR_LP=` to this path.
- `plant/`:
  - `plant.py` + `plant_params.json` (table plant; the default, and the one used for stop work).
  - `plant_bp.py` + `plant_params_bpos.json` (Steve's brake-overshoot plant, used for jab work).
  - `plant_params_bpos2.json` (harsh-side bracket only).
  - Select the bp plant with `PLANT_MODULE=plant_bp PLANT_PARAMS=<kit>/plant/plant_params_bpos.json`.
- `events/`:
  - `ev_2a6.json`, `ev_2a4.json`: the 14 jab and real-brake windows.
  - `ev_need.json`: 12 windows on 283/2a4/280/278/293/2a6/297.
  - Route paths inside are resolved as `$ROUTES/<route-name>`.
- `open_loop_fleet.py`: open-loop fleet sweep. `VARIANTS=name=planner.py,...`, args `ROUTE_DIR OUT.npz`.
- `score.py`: metrics per event; `--ref reference/expected.json KEY` diffs against Bob's results.
- `reference/expected.json`: Bob's numbers. Keys are `<events>/<plant>/<candidate>`. D073 means `CAR_LP=car/longitudinal_planner_d073.py`, CANDS=BASE.

## Not in git (delivered separately once Peter approves a transfer method)
- Route logs: 000002a6--f5f92c6f78, 000002a4--4736f8e372, 00000283--fe4e75f88b, 00000280--d02d9c2f8e, 00000278--8f101d683e,
  00000293--9d152a3cdc, 00000297--f971b5896f. Point `ROUTES` at their parent directory.
- Params snapshots: `params_2a6`, `params_2a4` (the car's params at those drives) and `params_car` (2026-09-30). These hold device
  identity and API keys, so they never go in git. Set `PARAMS_ROOT=` to the snapshot dir.

## Run (from the repo root)
    K=tools/longitudinal/sim_kit
    export PYTHONPATH=.:opendbc_repo:tools ROUTES=/path/to/routes
    # today's planner + chill reference, table plant
    CANDS=BASE,ACC PARAMS_ROOT=/path/params_2a6 python $K/closed_loop.py $K/events/ev_2a6.json out/x_2a6.pkl
    # D-073 planner
    CAR_LP=$K/car/longitudinal_planner_d073.py CANDS=BASE PARAMS_ROOT=/path/params_2a6 python $K/closed_loop.py $K/events/ev_2a6.json out/v3_2a6.pkl
    python $K/score.py out/x_2a6.pkl BASE --ref $K/reference/expected.json ev_2a6/plant/BASE
- `ev_need.json` uses `PARAMS_ROOT=params_car`.
- The first line of the harness output starts with `CHECK planner ...`. It must name the kit's planner file.

## Validation before real work
Reproduce `ev_2a6/plant/BASE`, `ev_2a6/plant/D073`, `ev_2a4/plant/BASE`, `ev_2a4/plant/D073`, then `ev_need/plant/BASE`.
WORST diff should be < 0.05 (Bob's own kit run reproduces 0.000). If it is larger, report it before anything else:
- x86 vs aarch64 acados differences
- params mismatch
- a different repo head for `tools/longitudinal/alpha_*_replay.py`
