#!/usr/bin/env bash

export PASSIVE="0"
export NOBOARD="1"
export SIMULATION="1"
export FINGERPRINT="${FINGERPRINT:-HONDA_CIVIC_2022}"
# SIM_CAR_CONFIG (see tools/sim/sim_car_config.py): the bridge seeds CarParamsCache, so card takes the
# car's firmware from the cache instead of querying, and the interface picks the car's own tune.
if [[ -z "$SIM_CAR_CONFIG" ]]; then
  export SKIP_FW_QUERY="1"
fi

export BLOCK="${BLOCK},camerad,loggerd,encoderd,micd,logmessaged"
if [[ "$CI" ]]; then
  # TODO: offscreen UI should work
  export BLOCK="${BLOCK},ui"
fi

python3 -c "from openpilot.selfdrive.test.helpers import set_params_enabled; set_params_enabled()"
if [[ -n "$SIM_CAR_CONFIG" ]]; then
  python3 "$(dirname "$0")/sim_car_config.py" apply "$SIM_CAR_CONFIG"
fi

SCRIPT_DIR=$(dirname "$0")
OPENPILOT_DIR=$SCRIPT_DIR/../../

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
cd $OPENPILOT_DIR/system/manager && exec ./manager.py
