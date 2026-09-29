#!/usr/bin/env bash
# Mac MetaDrive sim with the TSFDO driving model on ONNX Runtime (CoreML, CPU+GPU).
# Run from a Terminal window (the bridge reads keys from the tty). q in the bridge exits.
#   TSFDO_DIR  folder holding driving_supercombo.onnx and tsfdo_cpu.pkl (default ~/Downloads/tsfdo)
# See docs/mac-metadrive-cinquev3/README.md for how the model and artifact were made.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
TSFDO_DIR="${TSFDO_DIR:-$HOME/Downloads/tsfdo}"

export PATH="$ROOT/.venv/bin:$PATH" PYTHONPATH="$ROOT" PYTHONUNBUFFERED=1
export OPENPILOT_PREFIX="${OPENPILOT_PREFIX:-tsfdo-mac}" OPENPILOT_ZMQ_NAMESPACE="${OPENPILOT_ZMQ_NAMESPACE:-tsfdo-mac}"
# SIM_MODEL=stock runs the tree's own split driving model (selfdrive/modeld/models/driving_{vision,policy}.onnx)
# instead of TSFDO. STOCK_DIR holds its CPU artifact (metadata + warp), built with the same compile_modeld
# recipe as tsfdo_cpu.pkl but --model-type vision_policy --vision-onnx ... --policy-onnx ... (see TSFDO.md).
if [[ "${SIM_MODEL:-tsfdo}" == "stock" ]]; then
  STOCK_DIR="${STOCK_DIR:-$HOME/Downloads/stock}"
  export SIMULATION_MODEL_ARTIFACT="$STOCK_DIR/stock_cpu.pkl"
  export SIMULATION_VISION_ONNX="$ROOT/selfdrive/modeld/models/driving_vision.onnx" SIMULATION_POLICY_ONNX="$ROOT/selfdrive/modeld/models/driving_policy.onnx"
  export SIMULATION_MODEL_VERSION=v8  # plan-based policy (plan + desire_state, no action head); the car's param says v15
else
  export SIMULATION_MODEL_ARTIFACT="$TSFDO_DIR/tsfdo_cpu.pkl" SIMULATION_ONNX_MODEL="$TSFDO_DIR/driving_supercombo.onnx"
fi
export SIMULATION_TINYGRAD_DEV=CPU SIMULATION_WARP_DEV=CPU
# The network runs in tinygrad on METAL, not ORT CoreML: CoreML's ANECompilerService stuck at 100% CPU for 30+ min on
# 2026-09-28 and pulled the controls loop to 63-84 Hz. SIMULATION_ORT_PROVIDER=coreml restores the old path.
export SIMULATION_ORT_PROVIDER="${SIMULATION_ORT_PROVIDER:-metal}"
# Set speed the bridge raises cruise to after engaging (km/h); engagement alone leaves ~9 km/h.
export SIM_CRUISE_KPH="${SIM_CRUISE_KPH:-25}"
# SIM_CAR_CONFIG=DIR from tools/sim/sim_car_config.py extract: drive as that car (fingerprint, firmware, tune, toggles).
if [[ -n "${SIM_CAR_CONFIG:-}" ]]; then
  export SIM_CAR_CONFIG FINGERPRINT="$(python3 -c "import sys; from cereal import car; cp = car.CarParams.from_bytes(open(sys.argv[1], 'rb').read()).__enter__(); print(cp.carFingerprint)" "$SIM_CAR_CONFIG/carParams.bin")"
  echo "sim car: $FINGERPRINT from $SIM_CAR_CONFIG"
fi

"$ROOT/tools/sim/launch_openpilot.sh" > /tmp/tsfdo-mac-manager.log 2>&1 &
MANAGER=$!
trap 'pkill -TERM -P $MANAGER 2>/dev/null; kill -TERM $MANAGER 2>/dev/null' EXIT
echo "manager pid $MANAGER, log /tmp/tsfdo-mac-manager.log; first run builds the CoreML cache (~1 min)"
sleep 15
# TSFDO reads the wide camera too; single-camera mode feeds it the narrow frame through the wide warp.
"$ROOT/tools/sim/run_bridge.py" --dual_camera "$@"
