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
export SIMULATION_MODEL_ARTIFACT="$TSFDO_DIR/tsfdo_cpu.pkl" SIMULATION_ONNX_MODEL="$TSFDO_DIR/driving_supercombo.onnx"
export SIMULATION_TINYGRAD_DEV=CPU SIMULATION_WARP_DEV=CPU
# Set speed the bridge raises cruise to after engaging (km/h); engagement alone leaves ~9 km/h.
export SIM_CRUISE_KPH="${SIM_CRUISE_KPH:-25}"

"$ROOT/tools/sim/launch_openpilot.sh" > /tmp/tsfdo-mac-manager.log 2>&1 &
MANAGER=$!
trap 'pkill -TERM -P $MANAGER 2>/dev/null; kill -TERM $MANAGER 2>/dev/null' EXIT
echo "manager pid $MANAGER, log /tmp/tsfdo-mac-manager.log; first run builds the CoreML cache (~1 min)"
sleep 15
# TSFDO reads the wide camera too; single-camera mode feeds it the narrow frame through the wide warp.
"$ROOT/tools/sim/run_bridge.py" --dual_camera "$@"
