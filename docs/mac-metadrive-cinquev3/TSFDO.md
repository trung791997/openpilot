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
