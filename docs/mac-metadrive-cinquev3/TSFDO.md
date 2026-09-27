# TSFDO in the Mac MetaDrive sim (sim evidence only)

State on 2026-09-26, the owner's M1 MacBook Air. Vision-only e2e, no radar. Nothing here is road evidence.

## Run it

```
uv sync --extra simulation-coreml        # adds onnxruntime (darwin only)
tools/sim/run_mac_tsfdo.sh               # from a Terminal window; the bridge reads keys from the tty
```

`TSFDO_DIR` (default `~/Downloads/tsfdo`) must hold:

- `driving_supercombo.onnx`: commaai/openpilot branch `tsfdo`, from the gitlab LFS mirror, sha256 `baa30b01…d78b`.
- `tsfdo_cpu.pkl`: a CPU compile. `DEV=CPU WARP_DEV=CPU JIT_BATCH_SIZE=0 OPENPILOT_HACKS=1 compile_modeld.py --model-type supercombo --model-size 512x256 --camera-resolutions 1928x1208 1344x760 --image-history-pipeline policy --out-of-band --behavior-version v16`. modeld uses it only for the metadata and the warp.

The network runs in ONNX Runtime with the CoreML provider on CPU+GPU (`selfdrive/modeld/sim_ort_runner.py`). The history queues run in numpy; that path is bit-identical to the tinygrad queues over 10 random frames. modeld takes about 24 ms per frame, and modelV2 runs at 20 Hz.

## Measured on this Mac

| Path | ms/frame | Max diff vs ORT CPU |
|---|---|---|
| ORT CoreML CPUAndGPU (used) | 14.7 | 0.06 |
| ORT CoreML ALL (Neural Engine) | 11 | 0.63 |
| ORT CPU | 24 | 0 |
| tinygrad CPU | ~320 | — |
| tinygrad METAL JIT | ~15 | wrong before the fix below: up to 58 on ~1000 of 2580 outputs; passes the compile self-check after it |

## Fixes needed to get here

- **MetaDrive terrain shader.** `terrain.frag.glsl` calls `texture2D`, which the macOS GL 4.1 core profile rejects. The shader then fails silently and the ground renders flat grey: no road surface, no lane lines. The model sees no road and plans a stop. Fixed in `tools/sim/bridge/metadrive/metadrive_process.py`, darwin only.
- **Dual camera.** TSFDO takes the wide camera. Without `--dual_camera`, the sim feeds it the narrow frame through the wide warp. The launcher now passes `--dual_camera`.
- **selfdrivedLagging.** On a thermally throttled laptop (`kernel_task` at about 84 %), the 100 Hz loop misses its deadline and soft-disables mid-drive. The check is skipped only when SIMULATION is set on darwin (`MAC_SIMULATION` in selfdrived.py).
- **rednose on macOS.** `rednose/helpers/ekf_sym_pyx.so` is the checked-in aarch64 Linux build, so locationd fails (posenetInvalid, locationdTemporaryError). Rebuild it locally with `scons rednose/helpers/ekf_sym_pyx.so`; the file is skip-worktree, so never commit it.

## Open

- **Set speed.** Engaging with one press of `1` leaves the cruise set speed at about 9 km/h. The car then crawls at 3 m/s and TSFDO loses the lanes in the first curve. `run_mac_tsfdo.sh` now sets `SIM_CRUISE_KPH=25` and the bridge taps RES_ACCEL after engaging until the set speed gets there (override with `SIM_CRUISE_KPH=...`; `0` turns it off). On 2026-09-26 the car then held 7 m/s and went round the track's 90° right-hand curves for 105 s without leaving the road. Lane-line probs reached 0.7-0.9. It still stops for about 7 s in some curves (desiredAcceleration goes negative when lane probs fall below about 0.1), then pulls away again.
- **Pipeline check on real frames** (`tools/sim/tsfdo_frame_check.py`). One mici segment (route `000001f3--39087a1a96` seg 5) went through the same NV12 packing, ModelState, warp and ORT policy that the sim uses. TSFDO lane-line probs averaged 0.30/0.58/0.07/0.00; the modelV2 recorded in the car averaged 0.24/0.57/0.06/0.00, and the two track frame by frame. The result was the same after reprojecting into tici 1928x1208 intrinsics (`--as-tici`), the sim's camera. The sim camerad's RGB-to-NV12 conversion matches OpenCV. MetaDrive's buffer is BGR, which is what the kernel expects. So the remaining weakness is MetaDrive's look, not the pipeline.
- **Steering telemetry.** carState.steeringAngleDeg in the sim is the road-wheel angle, with the steering-wheel command divided by a fixed ratio of 8 (upstream convention). A gap of about 8x between the commanded and reported angle is therefore expected.
- **Recording.** `SIM_RECORD_DIR=/tmp/simrec tools/sim/run_mac_tsfdo.sh` saves the road camera at 5 fps with speed and engagement drawn on it, for when the desktop cannot be screen-captured. `ffmpeg -framerate 5 -pattern_type glob -i '/tmp/simrec/*.jpg' -c:v libx264 -pix_fmt yuv420p out.mp4`. MetaDrive's image buffer is BGR, so frames are written without conversion (an earlier version swapped red and blue).

## tinygrad METAL JIT bug (fixed locally, static)

One kernel in TSFDO's output head, `r_215_3_4_128_4_...`, binds 31 buffers, which is the ICB maximum (`setMaxKernelBufferBindCount(31)`). Replayed from the Metal indirect command buffer on this M1 (Apple7), its output is wrong. Run by itself, it is correct.

- Bisection: graph batches that include this kernel fail. `[304,320)`, which excludes it, passes.
- Fix: `MetalGraph.supports_uop` in `tinygrad_repo/tinygrad/runtime/graph/metal.py` keeps kernels with 31 or more buffers out of the graph.
- With the fix, the full compile passes the self-check. With the fix reverted, it fails with `outputs differ from baseline`.
- No upstream issue or PR was found for this. Searches covered tinygrad #6313, #10170, #15129, #15156 and #17685.
- The sim still uses ORT CoreML. Switching modeld back to the Metal pickle is untested.

## Sky fix (2026-09-26)

On macOS MetaDrive loads `#version 120` skybox shaders, which the GL 4.1 core profile rejects. The sky then rendered flat grey, with the skybox texture on a stray tilted panel beside the road. `metadrive_process.py` now makes the skybox use MetaDrive's `#version 150` shaders. In two otherwise identical 120 s engaged runs at `SIM_CRUISE_KPH=25`:

| | stopped (v < 0.5 m/s) | mean speed | mean lane-line probs |
|---|---|---|---|
| grey sky | 18% (five stops of ~7 s) | 4.1 m/s | .28 .45 .46 .03 |
| real sky | 2% (one stop of ~4 s) | 5.5 m/s | .33 .49 .48 .01 |

The remaining stop came after the car drifted onto the right road edge in a curve. Sim evidence only.

## Driving as your own car (2026-09-26)

`tools/sim/sim_car_config.py extract --log <qlog or rlog> --out ~/.openpilot-sim/civic` reads one segment from the car. It keeps the logged CarParams (firmware and VIN included) and the car's settings params from initData: the PERSISTENT bool/int/float keys and every key with a default, but not calibration, learned state or device identity. Then run `SIM_CAR_CONFIG=~/.openpilot-sim/civic tools/sim/run_mac_tsfdo.sh`. That sets `FINGERPRINT` from the CarParams and applies the params after `set_params_enabled`. The bridge seeds `CarParamsCache`, so card takes the car's firmware from the cache and the interface picks the car's own branch (for this Civic: the NRDR C020 modified-EPS PID tune, steer at standstill, alpha long). The simulated car packs its CAN with that car's DBC. For a Bosch car with a radar, powertrain goes on bus 1, and a second copy of `CAMERA_MESSAGES` goes on the powertrain bus (with its own counter). `BoschARadar` is forced off because MetaDrive has no radar.

Checked 2026-09-26 with route `00000283--fe4e75f88b` segment 0 (log deleted after extracting): fingerprint `HONDA_CIVIC_BOSCH` from the cache (22 firmware versions), CAN valid, engaged with no alerts. The car's own cruise toggles took the set speed to 64 km/h. It drove 58 s at a mean 14 m/s (lane-line probs .25 .77 .64 .31) and then left the road in a curve at about 50 km/h. `SIM_CRUISE_KPH` does not cap a set speed that the toggles overshoot.

What carries over: the vehicle model (steer ratio, wheelbase), longitudinal planning and the toggles. What does not: the bridge steers MetaDrive with `actuators.steeringAngleDeg`, so the lateral PID/torque tune and EPS behaviour are not exercised unless `SIM_STEER_MODEL` is set (below), and the radar is absent. Sim evidence only.

## Steering through the car's own tune (2026-09-26)

`SIM_STEER_MODEL=tools/sim/eps_models/honda_civic_bosch_c020.json` (with `SIM_CAR_CONFIG`) makes the bridge steer with the car's torque command (`carOutput.actuatorsOutput.torque`) instead of openpilot's desired angle. The torque goes through a steering response fitted from the car's rlogs, and the resulting steering-wheel angle both steers MetaDrive and is read back by the car on `STEERING_SENSORS`. So the car's lateral PID (the C020 four-point tune) runs closed loop against the fitted response. The steering-wheel angle is turned into road curvature with the car's own `VehicleModel` (what `latcontrol_torque` measures against; for this Civic within 10% of kinematic), and that curvature into a MetaDrive command with a measured gain (below).

`tools/sim/eps_fit.py --out MODEL.json RLOG...` fits `rate[k+1] = a·rate + (b0 + b1·v)·u + (c0 + c1·v²)·angle + bias` on samples where openpilot steered (latActive, no driver torque, v > 5 m/s). It seeds with one-step least squares, then refines on 1 s free-run windows. One-step least squares alone gave a sluggish model (2.5 s mode), and the car under-steered off the road in the first right curve. The model is fitted on 8 C020 segments picked by a qlog sweep for sustained curves: routes `00000283--fe4e75f88b` 14/19/21/33, `00000280--d02d9c2f8e` 16/28/29 and `0000027a--4eae257c95` 1 (logs deleted after fitting). The error after 2 s free-running on the held-out segment has a median of 1.2° and a p90 of 6.0° (p90 9.5° before refining). It covers 5–21 m/s and commands from −0.85 to 0.51, and extrapolates outside that. Command delay is not identifiable from one held-out segment (the error is flat across 0–0.3 s), so the fitted delay is 0.

Checked in one 30 s sim run: angle tracks the controller's desired angle with ~0.1–0.5 s lag, and RMS 3.4° after lag on a run with 28° command std. The real car on the hardest fitted segment has RMS 3.0° on 35° std; on ordinary segments it has 0.4–0.7° with 0.14–0.32 s lag. The run left the road at 30 s after TSFDO's lane-line probabilities dropped to ~0.01 twice (the MetaDrive perception gap, which angle mode also has). Sim evidence only: the response model is not the car, and nothing here changes device behaviour.

Same map, one run each unless noted: angle mode left the road after 11 s (58 s in the earlier run), torque mode after 17, 30 and 37 s, and perfect tracking at the physical scale (a temporary diagnostic, removed) after 44 s. Every run ended where TSFDO's lane-line probabilities collapsed, so the limit on how far the sim drives is TSFDO's perception of MetaDrive's roads, not the steering model. `SIM_RECORD_DIR` now starts recording at the first engagement.

## Stock driving model in the sim (2026-09-26)

`SIM_MODEL=stock tools/sim/run_mac_tsfdo.sh` runs the tree's own split model (`selfdrive/modeld/models/driving_vision.onnx` + `driving_policy.onnx`) on ONNX Runtime CoreML instead of TSFDO. `sim_ort_runner.make_numpy_run_split` runs the vision session, slices its `hidden_state` into the policy's feature buffer and runs the policy; `modeld.py` takes the two networks from `SIMULATION_VISION_ONNX`/`SIMULATION_POLICY_ONNX`. The artifact for metadata and warp is compiled once with `selfdrive/modeld/compile_modeld.py --model-type vision_policy --vision-onnx ... --policy-onnx ...` into `~/Downloads/stock/stock_cpu.pkl` (~20 s, 66 MB, not in the repo). The policy has no action head, so `SIMULATION_MODEL_VERSION=v8` (plan-based curvature) overrides the car's `ModelVersion=v15` param, which otherwise crashes modeld with `KeyError: 'action'`.

Result, same map, angle mode: **120 s engaged with no departure**, mean 14.5 m/s, 1741 m, lane-line probabilities averaging [.06 .81 .64 .30] at ~19 Hz. TSFDO on the same map leaves the road within 11–44 s when its lane probabilities collapse. So the perception limit is TSFDO on MetaDrive, not the rendering or the sim pipeline.

**MetaDrive steering gain** (measured with a standalone MetaDrive loop, constant steer at constant speed): 0.00066 curvature per sent degree at 8 m/s, 0.00054 at 12, 0.00053 at 16, over 10–30°, with a dead band under 5°; about 60% of kinematic for its 2.47 m wheelbase. The bridge's torque mode uses `METADRIVE_CURV_PER_DEG = 0.00055`. The map's curves are radius 120 m (curvature 0.0083), which the car's `VehicleModel` says needs a 21° steering-wheel angle at 12 m/s.

**Stock model + torque mode: 120 s, no departure** (mean 13.9 m/s, 1664 m), once the sim's IMU was fixed. It first left the road at 18 s in four runs, and the trace showed why: the MetaDrive bridge never filled `state.imu`, so the gyro and accelerometer published zeros. With no yaw rate, paramsd learned a runaway steering angle offset (+2° to +36° in 17 s), `latcontrol_torque` measured its curvature from `steeringAngleDeg - angleOffsetDeg` and believed a 45° wheel was nearly straight, and kept pushing torque into the curve. Angle mode survived this because MetaDrive's read-back angle is the road-wheel angle (the command over 8), too small for the offset to matter much. The bridge now sends the yaw rate from MetaDrive's heading over each 50 ms physics step and an accelerometer with gravity, the centripetal term and longitudinal acceleration. Axis order matters: locationd reads the raw sensors as device `[-v[2], -v[1], -v[0]]`, and MetaDrive's heading is clockwise-positive against the device's counter-clockwise z; the sign was checked against `cameraOdometry.rot[2]` (vision yaw), which agrees with the gyro to ~10%. Without the accelerometer the pose yaw came out a third low and the offset still drifted. With both, `livePose` yaw matches the gyro (−0.125 vs 0.122 rad/s), the offset stays within ±3°, and the wheel sits at 17–36° through the 120 m curves (the car's `VehicleModel` says 21° at 12 m/s). The raw gyro also matches what the `VehicleModel` angle through `METADRIVE_CURV_PER_DEG` predicts (0.143 rad/s at 30°, 12.4 m/s), so the calibrated mapping holds. The EPS fit's steady-state gain, the earlier suspect, was not it: re-fitting with a steady-state term did not change the outcome and was discarded. Sim evidence only; no device behaviour change.

## Lateral gain training, hybrid off-policy / on-policy (2026-09-26/27)

Plan: `docs/superpowers/plans/2026-09-26-lat-tune-hybrid.md`. Everything below is **sim / replay evidence only; no device behaviour change**. No schedule is recommended for the car.

**What "RL" means here.** The deployable artifact is `LatGainSchedule` (p and i at 20/30/40/50 mph, percent of the C020 table, re-read by `LatControlPID` every 300 frames), not a network, so the training is episodic black-box policy search: the cross-entropy method (CEM) over the 8 knot values, with the reward measured on driving episodes. Off-policy episodes are the owner's real drives replayed through `tools/lateral/lat_pid_sim.py` (the real controller follows the logged desired curvature through a plant fitted from the same drives). On-policy episodes are MetaDrive runs of the full stack as the owner's car (`tools/sim/lat_episode.sh`), recorded in the same route format and appended to the pool at weight 0.25, with an RBF mistake penalty around the theta of any episode that left the road. Two corrections to the owner's proposal: footage cannot enter as pixels (the tune sits below the driving model, so drives enter as logged desired-angle trajectories plus the plant), and a single fitted plant overfits, so every candidate is scored on an ensemble of perturbed plants (delay −1..+2 frames, gain ×0.8..1.2, damping ×0.8..1.2) and the cost is mean + 0.5·std across plants. Tooling: `tools/lateral/lat_cem_tune.py` (`--kind pid|clarity_eps`, `--sim-route`), `tools/sim/sim_lat_record.py`, `tools/sim/sim_set_overrides.py`, `tools/sim/lat_episode.sh`, `tools/sim/lat_metrics.py`.

**Data.** Routes 0000027a--4eae257c95, 00000280--d02d9c2f8e, 00000283--fe4e75f88b (86 segments, ~61 engaged minutes, ~9 min below 25 mph, ~44 min at 25–50, ~8 min above 50). Even 2-minute blocks fit, odd blocks hold out. Route data deleted after the run.

**Plant.** Delay 5 frames with the fitted C020 firmware torque table (`plant_d5_fw`), window rms 0.754°, free-run 3 s end error 1.6–1.8° against a hold-angle baseline of 6.7–13.7°. Replaying the logged settings through it reproduces the drives per band: standard 25–50 mph err rms 0.78/0.69/0.75° sim vs 0.77/0.97/0.78° logged, curve actual/desired within 0.01, sign changes within 0.1/s, lag within 0.08 s. The low-speed band (<25 mph) is dominated by intersection turns (err rms 6–16° on both sides) and is not a tuning target.

**Real drives vs MetaDrive, ≥25 mph.** Real pooled: err rms 1.03°, p90 1.40°, sign changes 0.32/s, mean |desired| 6.3°. MetaDrive baseline (stock model, torque mode, owner's tune, fitted EPS): rms 3.8–4.5°, sign changes 0.7–1.3/s, mean |desired| 17°. The sim roads (radius 120 m curves) demand about 3× the steering of the owner's drives, and the stack turns about 30 % of the sim's 120 s episodes into no-lateral time (paramsd/calibration and engagement gaps), so MetaDrive is a stability gate and a mistake generator, not a fidelity benchmark.

**CEM, PID kind (round 1, off-policy).** Hyperparameters were cut from the plan's 24/6/6 gens/6 plants to population 16, elites 4, 4 plants, 5 generations for wall-clock (~45 min with 8 workers). Seed (the car's driven settings): p [115,125,125,115], i [50,95,95,100], f [50,100,100,100]. Best: p [141.2,128.8,138.1,116.0], i [62.0,106.2,114.0,92.1]. Holdout cost 0.944 of seed (per plant 0.933–0.955; low 0.933, standard 0.946, highway 1.018). Verdict REJECTED under the frozen rule (highway band worse than 2 %). Generation bests 0.953, 0.953, 0.968, 0.933, 0.942: the search flattened after generation 3, the trust region (±40 points) was not reached, and the 6 % gain is mostly a stiffer 20–40 mph P with a little more I.

**James's `LatControlClarityEps` (STATUS 166–168) as a benchmark and a search space.** The controller ignores `LatGainSchedule`; its trims are constants in `nrdr_eps_firmware_ff.py` (CIVIC_P_SCALE, CIVIC_I_SCALE, OUTPUT_LPF_TAU), so the search over them (`--kind clarity_eps`, 9 dims, sim-only overrides into the core) can be scored off-policy but cannot be driven in MetaDrive or on the car. Four-way holdout comparison on the 5-plant ensemble, cost ratio to the PID seed, and per-band err rms / straight rms / curve ratio / sign changes per s / lag:

| variant | holdout cost | low <25 | standard 25–50 | highway >50 |
|---|---|---|---|---|
| PID seed | 1.000 | 8.51 / 1.80 / 0.882 / 0.68 / 0.25 | 0.76 / 0.49 / 0.964 / 0.86 / 0.30 | 0.53 / 0.48 / – / 0.77 / – |
| PID CEM best | 0.944 (0.933–0.955) | 7.98 / 1.61 / 0.902 / 0.74 / 0.22 | 0.72 / 0.47 / 0.970 / 0.89 / 0.28 | 0.53 / 0.47 / – / 0.78 / – |
| Clarity seed (fixed Civic trims) | 1.644 (1.35–1.94) | 8.43 / 1.53 / 0.985 / 1.54 / 0.22 | 0.57 / 0.36 / 1.006 / 1.74 / 0.16 | 0.42 / 0.36 / – / 1.08 / – |
| Clarity CEM best | 1.438 (1.22–1.67) | 8.21 / 1.43 / 0.988 / 1.70 / 0.21 | 0.60 / 0.39 / 1.006 / 1.52 / 0.17 | 0.44 / 0.38 / – / 1.01 / – |

Clarity tracks ~25 % tighter than the PID at 25–50 mph (0.57 vs 0.76°), holds curves at ratio 1.0 instead of 0.96, and lags half as much, which agrees with STATUS 167/168. It also changes sign twice as often (1.74 vs 0.86/s), and the cost's dither penalty (2·max(0, rate/seed − 1.10)) prices that at +0.6 to +0.9 per band, so under this cost it loses to the PID. The Clarity CEM (best p_scale [1.36,0.99,1.00], i_scale [0.80,0.96,1.15], out_tau [0.066,0.043,0.008] s) reached 0.372 of the Clarity seed's own cost on holdout but only by cutting the standard band's dither; highway got worse, so REJECTED as well. What the controller contributed to the training is a target, not a tune: it shows that ~0.57° at 25–50 mph is reachable on this plant, roughly 25 % beyond anything the PID search found inside its trust region.

**On-policy MetaDrive episodes (120 s, `lat_metrics.py` ≥25 mph).**

| episode | controller | engaged s | rms ° | osc /s | departure |
|---|---|---|---|---|---|
| base1 | PID seed | 81 | 4.46 | 0.68 | yes, 81 s |
| base2 | PID seed | 91 | 3.80 | 0.88 | no |
| pidbest1 | PID CEM best | 120 | 4.42 | 1.02 | no |
| pidbest2 | PID CEM best | 76 | 4.53 | 1.30 | yes, 72 s (err 35°, saturated 22 % of the last 2 s) |
| clar1 | Clarity seed | 59 | 7.84 | 1.52 | no (disengaged at 42 s, saturated, err 19°) |
| clar2 | Clarity seed | 50 | 5.44 | 1.53 | no (disengaged at 40 s, err 1.8°) |

The gate (no departure, rms and osc within +10 % of the baseline) FAILS for the PID CEM best: oscillation is 15–50 % higher, and it left the road once in two, the same as the seed. The 6 % off-policy gain did not transfer.

**Correction (2026-09-27, from the recorded frames).** MetaDrive's `out_of_road_done` defaults to True: the world terminates the moment the car leaves the road, after which the bridge keeps publishing the last state. clar2's last rendered frame is t = 42.3 s with the car on the left curb (outside of a right curve) while engaged, and pidbest2's last frame (t = 82.3 s) is past the curb line; base1's bridge log carries MetaDrive's out-of-road termination. clar1's record also ends (t = 44.6 s) but its last frame shows the car on the road, engaged, at 12.2 m/s, close to the solid yellow line on the inside of a left curve, and its log has no departure line. MetaDrive counts touching a solid line as out of road, so that is the likely end, but it is not confirmed. Rows after each record's end are stale. Read the table as: Clarity's record ended at ~40–45 s in 2 of 2 runs on the 120 m-radius default map (one confirmed departure), PID's in 2 of 4 at 72–81 s (both confirmed); the "disengaged" wording above was wrong (the world had ended, controlsd had not crashed). This is a stability observation on one map, not a controller ranking. The bridge now sets `out_of_road_done=False` and logs `metadrive: out_of_road / back_on_road at frame N pos (x, y)` so a departure is an event in the record, not the end of it. Rendering in the checked frames (sky, road surface, lane lines) was correct.

**CEM round 2 (PID kind, pool + the four PID MetaDrive episodes at weight 0.25, mistake penalty around base1 and pidbest2).** REJECTED on holdout: overall 0.959 but highway 1.065 (fails the every-band ≤ 1.02 rule; low band 0.945). Best theta p [138, 132, 126.8, 140.9], i [60.5, 83.9, 75.9, 96.3] at 20/30/40/50 mph. Not a recommendation. Its cost had no past-desired term, so it could also hide overshoot; `lat_cem_tune.py` now adds a `turn <25mph` band (trailing + 1.5 × past-desired error, |des| > 45°, v < 11.2 m/s) and `--pid-ff on|off` for round 3, and any winner must also pass `lat_score gate`. Episodes from ~02:12 to 02:57 on 2026-09-27 were lost to a persisted sim `Offroad_ExcessiveActuation` param (hardwared blocked onroad); `lat_episode.sh` now clears it before each episode.

**Verdict and recommendation.** No `LatGainSchedule` is recommended for the car from this work. The off-policy search finds a consistent but small (~6 %) improvement that is concentrated at 20–40 mph and does not survive the on-policy gate; the highway band cannot be tuned on this data (~8 minutes, no curves). What would change this: more highway minutes with curves in the pool, MetaDrive episodes long enough to hold ≥25 mph for the full 120 s, and the Clarity disengagement explained. If the owner wants to try a candidate anyway, the round-1 best is the one to try, at 25–50 mph only, on a drive that can be compared to 00000280/00000283 with `lat_pid_sim.py validate`. Sim evidence only; no device behaviour change.

## TSFDO as the default sim model (2026-09-27)

Episodes now run TSFDO, the owner's daily model, by default (`SIM_MODEL=stock` for comparison), and `SIM_CAMERA=mici` renders the comma 4's os04c10 geometry (1344x760, road hfov 61.0°, wide 115.4°). Three causes had made TSFDO look unreliable in MetaDrive. None of them was the model:

- **Bridge freeze (commit 2b2001f3).** The main loop and the 100 Hz `simulated_car` thread both called `SubMaster.update` on the same ZMQ sockets. On macOS that tripped libzmq's `fq.cpp:56` assertion and froze about half of all episodes ~30 s before their end, including clar1/clar2's ~40 s ends in the table above. Only the thread updates now.
- **Paint region (commit 0646d64f).** MetaDrive paints road and lane-line texture only in a 1024 m square around the origin. The old gentle S-road ran 1.5 km along x, and every gentle episode (4/4) left the road at x 539–590 m where the paint ended, with TSFDO's lane probs still ~0.9. The terrain region is now centred on the built map (darwin only), and `gentle` is a closed S-loop at R 250 m (3.1 km in a 945 m square). A 2048 m region drew the whole ground white on this Mac, so **maps must fit in 1024 m**.
- **Line rendering (same commit).** Lane lines were 1 px lines from truncated points, thresholded by the shader: curves came out as staircase blobs and the centre line zigzagged. They are now drawn anti-aliased at 0.15 m from sub-pixel points, with the `terrain.frag` thresholds at 50 % coverage.

Results, TSFDO + mici camera, owner's car config, torque mode through the fitted C020 EPS, 120 s episodes:

| Map | Speed | Episodes | Departures | Lane-line probs | Frame drop |
|---|---|---|---|---|---|
| gentle loop (R 250 m S-bends) | 25 mph set | 2 | 0 | 0.8–0.96 | 1–8 % |
| default (R 120 m, 90° curves) | 25 mph set | 1 | 0 | 0.4–0.87 | ≤ 9 % |
| default, `SIM_MAP_RADIUS=60` | 25 km/h | 1 | 0 | 0.13–0.5 | ≤ 9 % |
| default, `SIM_MAP_RADIUS=40` | 25 km/h | 1 | 0 | 0.15–0.33 | ≤ 9 % |

At R 40 m the wheel reached ~95° |desired|, so it replaces `intersection` as the low-speed turn scenario. The `intersection` preset's r20 corner reads to TSFDO as a T-junction (lane probs ~0.05 on the approach), and it drives straight on, as openpilot does at real intersections. Turn metrics from that preset are not valid under TSFDO.

**Remaining gaps versus the owner's car:**
- The wide camera is a rectilinear 115° render, not the real fisheye.
- Camera height and pitch are fixed rather than taken from the car's liveCalibration.
- MetaDrive's look: lane confidence is low on tight loops (R ≤ 60 m).
- MetaDrive's Bullet car stands in for the Civic's chassis. Only the EPS response is fitted from its rlogs.

Sim evidence only; no device behaviour change.
