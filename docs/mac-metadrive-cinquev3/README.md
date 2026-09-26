# Mac MetaDrive + CinqueV3 CoreML

A source handoff for reproducing the MetaDrive simulator and Jetlink/CoreML inference path on Apple Silicon, on top of the vfn branch. Tested on an M1 Pro Mac. The patch is based on vfn commit `35ddc44bdf673a4088a128bde86ef58878e34be9`; use that base because the simulator and modeld interfaces are branch-specific.

This package contains `README.md` and `mac-integration.patch`. It contains source text only: no ONNX/model files, compiled artifacts, logs, CoreML caches, credentials, or machine-specific paths.

## Setup

Use macOS on Apple Silicon, Xcode Command Line Tools, Git, `uv`, and Python 3.12. Use the VFN-derived repository/fork containing the pinned base commit below; this is not the separate radar checkout. Verify the checkout before applying the patch:

```sh
git rev-parse HEAD
# Must print: 35ddc44bdf673a4088a128bde86ef58878e34be9

git switch -c mac-metadrive-cinquev3
git apply /path/to/mac-integration.patch
git submodule update --init --recursive
uv python install 3.12
uv sync --python 3.12 --extra tools --extra simulation-coreml --extra testing
```

The extras install MetaDrive, pytest, Jetlink pinned to commit `40d7507148c3bb7568c39fd380eab05ed12e232a`, and ONNX Runtime 1.29. Keep the vfn base and its tinygrad submodule revision together.

## Get and verify CinqueV3

Supply the full ONNX model locally; it is not included. The source is:

```text
hf://commaai/openpilot_driving_models/f78ed37d-afad-4dbc-8050-40ea885eedde/12864/big_driving_supercombo.onnx
```

For example, with the Hugging Face CLI:

```sh
hf download commaai/openpilot_driving_models 12864/big_driving_supercombo.onnx \
  --revision f78ed37d-afad-4dbc-8050-40ea885eedde --local-dir "$HOME/Downloads"
MODEL="$HOME/Downloads/big_driving_supercombo.onnx"
shasum -a 256 "$MODEL"
```

Proceed only if SHA-256 is `404a18cfd86d29637d20c697dfde245bb47c666ae016730ab674c65f4d1e1aa4` (766,354,845 bytes). This guards against downloading a smaller or different model.

Build the local tinygrad runtime artifact from that ONNX. This artifact is machine-specific and must be generated on the Mac; it is not part of the handoff:

```sh
mkdir -p outputs/cinque_v3
DEV=METAL WARP_DEV=CPU FLOAT16=1 IMAGE=1 JIT_BATCH_SIZE=0 NOLOCALS=1 OPENPILOT_HACKS=1 \
PYTHONPATH="$PWD${PYTHONPATH:+:$PYTHONPATH}" \
.venv/bin/python selfdrive/modeld/compile_modeld.py \
  --model-type supercombo --model-size 512x256 \
  --camera-resolutions 1928x1208 1344x760 \
  --output outputs/cinque_v3/cinque_v3_metal_fp16.pkl \
  --frame-skip 1 --image-history-pipeline policy --out-of-band \
  --behavior-version v16 --supercombo-onnx "$MODEL"
```

## Run the simulator interactively

In each terminal, from the repository root, set the same environment. Change `OPENPILOT_PREFIX` and `OPENPILOT_ZMQ_NAMESPACE` to unique values if other openpilot/simulator processes are running:

```sh
export PATH="$PWD/.venv/bin:$PATH"
export OPENPILOT_PREFIX=cinquev3-mac
export OPENPILOT_ZMQ_NAMESPACE=cinquev3-mac
export SIMULATION_COREML_MODEL="$HOME/Downloads/big_driving_supercombo.onnx"
export SIMULATION_MODEL_ARTIFACT="$PWD/outputs/cinque_v3/cinque_v3_metal_fp16.pkl"
export SIMULATION_TINYGRAD_DEV=METAL
export SIMULATION_WARP_DEV=CPU
```

Start the openpilot manager in Terminal 1, then start MetaDrive in Terminal 2:

```sh
./tools/sim/launch_openpilot.sh
```

```sh
./tools/sim/run_bridge.py
```

Use `q` in the simulator to exit. The launch script sets simulation mode; keep both terminals in the same checkout and virtual environment.

## Automated smoke check (optional)

This is the automated full-stack smoke command used to check the setup. Use fresh, unique values for the Params prefix and ZMQ namespace if another openpilot/simulator process is running:

```sh
PATH="$PWD/.venv/bin:$PATH" \
SP_DISABLE_HOST_PYTEST_REDIRECT=1 \
OPENPILOT_PREFIX=cinquev3-mac-smoke \
OPENPILOT_ZMQ_NAMESPACE=cinquev3-mac-smoke \
SIMULATION_COREML_MODEL="$MODEL" \
SIMULATION_MODEL_ARTIFACT="$PWD/outputs/cinque_v3/cinque_v3_metal_fp16.pkl" \
SIMULATION_TINYGRAD_DEV=METAL SIMULATION_WARP_DEV=CPU \
PYTHONUNBUFFERED=1 \
.venv/bin/python -m pytest -q tools/sim/tests/test_metadrive_bridge.py
```

On first launch Jetlink prepares and caches CoreML engines under the user cache directory; allow the initial build to finish. The tested path used CPU for image warping and Tinygrad Metal for queues, while Jetlink/ONNX Runtime ran the vision network on the Apple Neural Engine and the policy network on the GPU. Keep `SIMULATION_WARP_DEV=CPU`: VisionIPC supplies host pointers, which cannot safely be wrapped as Metal buffers by this tinygrad path.

On the tested M1 Pro, policy inference was about 31.6 ms median / 39.7 ms p95; camera conversion, warp, and model together were about 47–53 ms. First-run engine compilation is separate and much slower. Set `PATH` so child processes use this checkout's venv. Keep `SP_DISABLE_HOST_PYTEST_REDIRECT=1` so the smoke exercises this checkout. A unique `OPENPILOT_ZMQ_NAMESPACE` prevents services from different checkouts talking to each other. The Panda3D macOS restore-dialog workaround is process-local; no persistent macOS preference is changed.

## What the experiment showed

The stock model passed the full-stack smoke; the fine-tuned candidate failed the lane-departure gate (`out_of_lane`). The candidate is rejected and is not included. A small head-only training experiment reduced error against its simplified steering labels, but that did not translate to the normal openpilot control stack. This patch is for reproducing simulator inference, not a claim that the model has improved.

## Patch contents

The focused patch contains changes to `pyproject.toml`, `uv.lock`, the ZMQ namespace handling, macOS simulator/manager setup, stateful supercombo artifact and queue handling, the Jetlink/CoreML adapter, and its related source checks. It omits generated libraries and model artifacts, unrelated working-tree edits, theme links, and all experimental SFT/DAgger scripts and outputs.
