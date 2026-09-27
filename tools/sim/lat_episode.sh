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
tmux kill-session -t tsfdo-sim 2>/dev/null; sleep 3
tmux new-session -d -s tsfdo-sim -x 200 -y 50 "cd $PWD && env SIM_MODEL=stock SIM_STEER_MODEL=$PWD/tools/sim/eps_models/honda_civic_bosch_c020.json \
  SIM_CAR_CONFIG=$HOME/.openpilot-sim/civic SIM_RECORD_DIR=$out/frames tools/sim/run_mac_tsfdo.sh 2>&1 | tee $out/bridge.log"
sleep 30
if [ -n "$sched" ]; then
  OPENPILOT_PREFIX=$prefix python tools/sim/sim_set_overrides.py --prefix $prefix "LatGainSchedule=$sched" "$@"
elif [ $# -gt 0 ]; then
  OPENPILOT_PREFIX=$prefix python tools/sim/sim_set_overrides.py --prefix $prefix "$@"
fi
sleep 10
OPENPILOT_PREFIX=$prefix OPENPILOT_ZMQ_NAMESPACE=$prefix timeout $((secs + 60)) python tools/sim/sim_lat_record.py "$out" "$secs" --bridge-log "$out/bridge.log"
tmux send-keys -t tsfdo-sim q; sleep 5; tmux kill-session -t tsfdo-sim 2>/dev/null
python - "$out" "$sched" <<'PY'
import json, sys
out, sched = sys.argv[1], sys.argv[2]
ep = json.load(open(f"{out}/episode.json"))
if sched:
  s = json.loads(sched)
  ep["theta"] = [float(x) for x in s["p"]] + [float(x) for x in s["i"]]
  ep["schedule"] = sched
json.dump(ep, open(f"{out}/episode.json", "w"), indent=1)
print("episode:", {k: ep[k] for k in ("seconds", "rows", "engaged_s", "offroad")})
PY
python tools/sim/lat_metrics.py "$out/lat_pid_sim.npz"
