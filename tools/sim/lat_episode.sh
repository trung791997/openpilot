#!/bin/bash
# One on-policy MetaDrive episode for the lateral gain search (docs/superpowers/plans/2026-09-26-lat-tune-hybrid.md §2).
#
#   tools/sim/lat_episode.sh OUTDIR SECS [SCHEDULE_JSON] [KEY=VALUE ...]
#
# Launches the sim as the owner's car (SIM_CAR_CONFIG, stock model, fitted EPS response), waits for it to come up,
# applies SCHEDULE_JSON as LatGainSchedule plus any KEY=VALUE tuning params through tools/sim/sim_set_overrides.py
# (the controller re-reads its params every 300 frames), records SECS seconds with tools/sim/sim_lat_record.py into
# OUTDIR (a lat_pid_sim route directory), stores the 8 schedule values as "theta" in OUTDIR/episode.json so
# lat_cem_tune.py can penalise them if the episode left the road, and prints lat_metrics for the episode.
# Requires: ~/.openpilot-sim/civic (sim_car_config extract), tmux, tools/sim/run_mac_tsfdo.sh. Sim evidence only.
set -u
cd "$(dirname "$0")/../.."
out=$1; secs=$2; sched=${3:-}; shift 2; [ $# -gt 0 ] && shift
prefix=tsfdo-mac
export PATH=$PWD/.venv/bin:$PATH PYTHONPATH=$PWD
mkdir -p "$out"
# SIM_FAST_TEARDOWN=1: instead of the fixed 3 s / 5 s sleeps around a run, wait until the previous run's sim processes have
# exited (polled every 0.5 s, max 15 s). The launch -> record timing, and so the recorded window, is unchanged.
sim_gone() { if [ "${SIM_FAST_TEARDOWN:-0}" = 1 ]; then for _ in $(seq 1 30); do
    pgrep -f "run_bridge\.py|system/manager/manager\.py|selfdrive\.|run_mac_tsfdo" > /dev/null || return 0; sleep 0.5; done
    echo "lat_episode: sim processes still up after 15 s" >&2; else sleep "$1"; fi; }
tmux kill-session -t tsfdo-sim 2>/dev/null; sim_gone 3
# start-read toggles (NrdrLatEpsFirmwareFF picks LatControlPID vs LatControlClarityEps when controlsd starts) must be set
# before launch; launch_openpilot.sh only re-applies the car config's own keys, so these survive. Applied again after
# launch below for the live-read keys.
if [ $# -gt 0 ]; then
  OPENPILOT_PREFIX=$prefix python tools/sim/sim_set_overrides.py --prefix $prefix "$@"
fi
# selfdrived persists Offroad_ExcessiveActuation when a sim episode brakes/crashes hard (e.g. a road departure), and
# hardwared then blocks onroad (startup_conditions no_excessive_actuation) for every later episode: bridge runs, card and
# controlsd never start, sim_lat_record sees no carParams. Sim prefix params only; never touches a device.
OPENPILOT_PREFIX=$prefix python -c "from openpilot.common.params import Params; Params().remove('Offroad_ExcessiveActuation')"
# SIM_MODEL (tsfdo = the owner's daily model, default; stock = the tree's split model, for comparison) and SIM_CAMERA
# (tici; mici = the comma 4's os04c10 geometry, tools/sim/lib/common.py) are forwarded too.
# SIM_DRIVER (JSON, a scripted hand on the wheel: tools/sim/lib/driver_model.py) and NRDR_OVERRIDE_MODE (sim-only override
# candidates, read by a temporarily patched controller) are forwarded as well.
# SIM_MAP (default / intersection / gentle, metadrive_bridge.MAP_PRESETS), SIM_MAP_RADIUS / SIM_MAP_STRAIGHT and SIM_CRUISE_KPH are forwarded explicitly: a fresh
# tmux server does not inherit this shell's environment. SIM_PLANT (civic = the car's own bicycle model moves the car, default;
# metadrive = MetaDrive's Bullet chassis, the pre-2026-09-27 plant) too, and the car-as-driven settings: SIM_PLANT_SR /
# SIM_PLANT_STIFFNESS / SIM_PLANT_OFFSET_DEG, SIM_CAM_HEIGHT / SIM_CAM_RPY, SIM_MAP=route + SIM_MAP_FILE, SIM_LANE_WIDTH.
# The car as driven: $HOME/.openpilot-sim/civic/sim.env (not in the repo) sets the learned steer ratio, stiffness and angle
# offset, the camera height and calibration, and the lane width from the owner's routes. SIM_AS_DRIVEN=0 skips it.
[ "${SIM_AS_DRIVEN:-1}" = 1 ] && [ -f "$HOME/.openpilot-sim/civic/sim.env" ] && . "$HOME/.openpilot-sim/civic/sim.env"
tmux new-session -d -s tsfdo-sim -x 200 -y 50 "cd $PWD && env SIM_MODEL=${SIM_MODEL:-tsfdo} SIM_CAMERA=${SIM_CAMERA:-tici} SIM_MAP=${SIM_MAP:-default} SIM_CRUISE_KPH=${SIM_CRUISE_KPH:-25} ${SIM_MAP_RADIUS:+SIM_MAP_RADIUS=$SIM_MAP_RADIUS} ${SIM_MAP_STRAIGHT:+SIM_MAP_STRAIGHT=$SIM_MAP_STRAIGHT} ${SIM_BLINKER:+SIM_BLINKER=$SIM_BLINKER} ${SIM_STOP_BEFORE_TURN:+SIM_STOP_BEFORE_TURN=$SIM_STOP_BEFORE_TURN} ${SIM_STOP_M:+SIM_STOP_M=$SIM_STOP_M} ${SIM_KEYS:+SIM_KEYS=$SIM_KEYS} ${*:+SIM_PARAM_OVERRIDES='$*'} SIM_PLANT=${SIM_PLANT:-civic} ${SIM_PLANT_SR:+SIM_PLANT_SR=$SIM_PLANT_SR} ${SIM_PLANT_STIFFNESS:+SIM_PLANT_STIFFNESS=$SIM_PLANT_STIFFNESS} ${SIM_PLANT_VGR:+SIM_PLANT_VGR=$SIM_PLANT_VGR} ${SIM_PLANT_OFFSET_DEG:+SIM_PLANT_OFFSET_DEG=$SIM_PLANT_OFFSET_DEG} ${SIM_CAM_HEIGHT:+SIM_CAM_HEIGHT=$SIM_CAM_HEIGHT} ${SIM_CAM_RPY:+SIM_CAM_RPY=$SIM_CAM_RPY} ${SIM_MAP_FILE:+SIM_MAP_FILE=$SIM_MAP_FILE} ${SIM_LANE_WIDTH:+SIM_LANE_WIDTH=$SIM_LANE_WIDTH} ${SIM_DRIVER:+SIM_DRIVER='$SIM_DRIVER'} ${NRDR_OVERRIDE_MODE:+NRDR_OVERRIDE_MODE=$NRDR_OVERRIDE_MODE} ${SIM_FAST_LAUNCH:+SIM_FAST_LAUNCH=$SIM_FAST_LAUNCH} \
  SIM_STEER_MODEL=${SIM_STEER_MODEL:-$PWD/tools/lateral/plants/civic_bosch_c020.json} \
  SIM_CAR_CONFIG=$HOME/.openpilot-sim/civic SIM_RECORD_DIR=$out/frames tools/sim/run_mac_tsfdo.sh 2>&1 | tee $out/bridge.log"
# SIM_FAST_LAUNCH=1: run_mac_tsfdo.sh starts the bridge 2 s after the manager instead of 15 s, and this script waits for
# controlsd to be up (max 60 s) instead of a fixed 30 s, then starts recording after 1 s instead of 10 s. The car stands
# still until openpilot engages on its own; the recorder logs no rows until then, so episode.json seconds runs ~10 s longer than
# rows/100 Hz (rate checks must use the rows' own time span). The start-read
# toggles are in place before launch either way (SIM_PARAM_OVERRIDES). Changes when recording starts, so do not mix runs with
# and without it in one comparison.
if [ "${SIM_FAST_LAUNCH:-0}" = 1 ]; then
  for _ in $(seq 1 120); do pgrep -f selfdrive.controls.controlsd > /dev/null && break; sleep 0.5; done; sleep 1
else sleep 30; fi
if [ -n "$sched" ]; then
  OPENPILOT_PREFIX=$prefix python tools/sim/sim_set_overrides.py --prefix $prefix "LatGainSchedule=$sched" "$@"
elif [ $# -gt 0 ]; then
  OPENPILOT_PREFIX=$prefix python tools/sim/sim_set_overrides.py --prefix $prefix "$@"
fi
# controller check: the start-read toggles against controlsd's start time (SIM_PARAM_OVERRIDES applies them before it).
{ echo "lat_episode: controlsd started $(ps -o lstart= -p "$(pgrep -f selfdrive.controls.controlsd | head -1)" 2>/dev/null) now $(date)"
  OPENPILOT_PREFIX=$prefix python -c "from openpilot.common.params import Params; p=Params(); print('lat_episode: toggles', {k: p.get(k) for k in ('NrdrLatEpsFirmwareFF', 'NrdrLatPidFirmwareFF')})"; } > "$out/controller_check.log" 2>&1
if [ "${SIM_FAST_LAUNCH:-0}" = 1 ]; then sleep 1
  grep -q "Engaged: True" "$out/bridge.log" && echo "lat_episode: WARNING engaged before recording started" | tee -a "$out/controller_check.log"
else sleep 10; fi
# SIM_KEYS="30:3,50:1": bridge keyboard keys sent at those seconds after recording starts (3 = cruise cancel, 1 = cruise
# up / resume, s = brake, z/x = blinkers; tools/sim/lib/keyboard_ctrl.py). "30:3,50:1" is a stop-and-resume scenario.
# SIM_KEYS is also forwarded into the bridge env above (SCRIPTED_KEYS in bridge/common.py hands the resume to the keys).
if [ -n "${SIM_KEYS:-}" ]; then
  ( IFS=,; last=0; for kv in $SIM_KEYS; do at=${kv%%:*}; key=${kv#*:}; sleep $((at - last)); last=$at
      tmux send-keys -t tsfdo-sim "$key"; echo "lat_episode: key $key at ${at}s" >> "$out/bridge.log"; done ) &
  keys_pid=$!
fi
OPENPILOT_PREFIX=$prefix OPENPILOT_ZMQ_NAMESPACE=$prefix timeout $((secs + 60)) python tools/sim/sim_lat_record.py "$out" "$secs" --bridge-log "$out/bridge.log"
[ -n "${keys_pid:-}" ] && kill $keys_pid 2>/dev/null
tmux send-keys -t tsfdo-sim q; sim_gone 5; tmux kill-session -t tsfdo-sim 2>/dev/null
python - "$out" "$sched" "$@" <<'PY'
import json, os, sys
out, sched, overrides = sys.argv[1], sys.argv[2], sys.argv[3:]
ep = json.load(open(f"{out}/episode.json"))
ep["overrides"] = overrides
ep["model"] = os.environ.get("SIM_MODEL", "tsfdo")
ep["camera"] = os.environ.get("SIM_CAMERA", "tici")
if "NrdrLatEpsFirmwareFF=true" in overrides:
  # James's controller runs on its fixed trims; theta is those 9 values (lat_cem_tune --kind clarity_eps units)
  from openpilot.selfdrive.controls.lib import nrdr_eps_firmware_ff as ff
  ep["kind"] = "clarity_eps"
  ep["theta"] = [100.0 * v for v in ff.CIVIC_P_SCALE + ff.CIVIC_I_SCALE] + [1000.0 * v for v in ff.OUTPUT_LPF_TAU]
elif sched:
  s = json.loads(sched)
  ep["kind"] = "pid"
  ep["theta"] = [float(x) for x in s["p"]] + [float(x) for x in s["i"]]
  ep["schedule"] = sched
json.dump(ep, open(f"{out}/episode.json", "w"), indent=1)
print("episode:", {k: ep[k] for k in ("seconds", "rows", "engaged_s", "offroad")})
PY
python tools/sim/lat_metrics.py "$out/lat_pid_sim.npz"
