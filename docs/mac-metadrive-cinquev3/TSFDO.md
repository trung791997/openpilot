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

- **TSFDO barely sees MetaDrive lanes.** laneLineProbs stay at or below 0.05. It pulls away under engagement but does not steer into the first curve, and MetaDrive ends the episode with `out_of_road` after about 15 s. The rendered frame and the model input look right, so this is probably domain gap (MetaDrive's flat, untextured look). A pipeline fault is not excluded: the check that separates them is feeding a real comma frame through this exact ORT path.
- **Recording.** `SIM_RECORD_DIR=/tmp/simrec tools/sim/run_mac_tsfdo.sh` saves the road camera at 5 fps with speed and engagement drawn on it, for when the desktop cannot be screen-captured. `ffmpeg -framerate 5 -pattern_type glob -i '/tmp/simrec/*.jpg' -c:v libx264 -pix_fmt yuv420p out.mp4`. A 68 s run on 2026-09-26 stayed engaged at 2-3 m/s, took the first gentle bend, then left the road onto the grass.

## tinygrad METAL JIT bug (fixed locally, static)

One kernel in TSFDO's output head, `r_215_3_4_128_4_...`, binds 31 buffers, which is the ICB maximum (`setMaxKernelBufferBindCount(31)`). Replayed from the Metal indirect command buffer on this M1 (Apple7), its output is wrong. Run by itself, it is correct.

- Bisection: graph batches that include this kernel fail. `[304,320)`, which excludes it, passes.
- Fix: `MetalGraph.supports_uop` in `tinygrad_repo/tinygrad/runtime/graph/metal.py` keeps kernels with 31 or more buffers out of the graph.
- With the fix, the full compile passes the self-check. With the fix reverted, it fails with `outputs differ from baseline`.
- No upstream issue or PR was found for this. Searches covered tinygrad #6313, #10170, #15129, #15156 and #17685.
- The sim still uses ORT CoreML. Switching modeld back to the Metal pickle is untested.
