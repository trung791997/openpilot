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
| tinygrad METAL JIT | ~15 | wrong: up to 58 on ~1000 of 2580 outputs |

## Fixes needed to get here

- **MetaDrive terrain shader.** `terrain.frag.glsl` calls `texture2D`, which the macOS GL 4.1 core profile rejects. The shader then fails silently and the ground renders flat grey: no road surface, no lane lines. The model sees no road and plans a stop. Fixed in `tools/sim/bridge/metadrive/metadrive_process.py`, darwin only.
- **Dual camera.** TSFDO takes the wide camera. Without `--dual_camera`, the sim feeds it the narrow frame through the wide warp. The launcher now passes `--dual_camera`.
- **selfdrivedLagging.** On a thermally throttled laptop (`kernel_task` at about 84 %), the 100 Hz loop misses its deadline and soft-disables mid-drive. The check is skipped only when SIMULATION is set on darwin (`MAC_SIMULATION` in selfdrived.py).
- **rednose on macOS.** `rednose/helpers/ekf_sym_pyx.so` is the checked-in aarch64 Linux build, so locationd fails (posenetInvalid, locationdTemporaryError). Rebuild it locally with `scons rednose/helpers/ekf_sym_pyx.so`; the file is skip-worktree, so never commit it.

## Open

- **TSFDO barely sees MetaDrive lanes.** laneLineProbs stay at or below 0.05. It pulls away under engagement but does not steer into the first curve, and MetaDrive ends the episode with `out_of_road` after about 15 s. The rendered frame and the model input look right, so this is probably domain gap (MetaDrive's flat, untextured look). A pipeline fault is not excluded: the check that separates them is feeding a real comma frame through this exact ORT path.
- **tinygrad METAL JIT.** The fault is in the Metal graph (ICB) replay, not the pickle:
  - `JIT=2`, which runs kernels individually, matches eager exactly at ~26 ms.
  - A per-command-buffer test rules out barrier races.
  - Bisection puts it in the last 19 kernels of the output head.
  - Suspect: the kernel that binds 31 buffers, the ICB maximum (`setMaxKernelBufferBindCount(31)`). Not yet confirmed.
