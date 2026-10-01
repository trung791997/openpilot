"""closed_loop.py with leads re-run from CAN (radard_from_can.py) instead of the logged radarState. REPLAY only (Job).

Candidates named RC_<variant> (e.g. RC_base, RC_s72, RC_s72b) take leadOne/leadTwo from the radard_from_can npz for the
event's route; any other candidate (e.g. BASE) runs closed_loop.py unchanged. Compare RC_base against RC_s72, not
against BASE: re-running radard from CAN already differs from the car's logged radar (293: BASE -2.94, RC_base -3.45).

RC_NPZ maps a substring of the route name to an npz: RC_NPZ=2a6=rc_2a6.npz,00000293=rc_293.npz
A frame whose logMono time is not in the npz falls back to the logged radarState; the count is printed at exit, check it.

  CAR_LP=$K/car/longitudinal_planner_d073.py CANDS=BASE,RC_base,RC_s72,RC_s72b PARAMS_ROOT=<car params> ROUTES=~/routes \
    RC_NPZ=... python $K/closed_loop_rc.py events.json out.pkl
"""
import atexit
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
import closed_loop as CL  # noqa: E402

_N = {}
for kv in os.environ.get("RC_NPZ", "").split(","):
  if kv:
    k, p = kv.split("=")
    z = np.load(p)
    _N[k] = (z, {round(float(t), 4): i for i, t in enumerate(z["t"])}, [str(f) for f in z["fields"]])
_cur = {"route": ""}
_miss = {"hit": 0, "miss": 0}
atexit.register(lambda: print(f"closed_loop_rc: RC frames matched {_miss['hit']}, fell back to logged radar {_miss['miss']}"))
_orig_run = CL.run_event


def run_event(ev):
  _cur["route"] = ev["route"]
  return _orig_run(ev)


CL.run_event = run_event
_orig_x = CL.xform


def xform(c, rs, v_ego, t):
  if not c.startswith("RC_"):
    return _orig_x(c, rs, v_ego, t)
  var = c[3:]
  for k, (z, idx, fl) in _N.items():
    if k in _cur["route"]:
      i = idx.get(round(t, 4))
      if i is None:
        _miss["miss"] += 1
        return rs
      _miss["hit"] += 1
      leads = []
      for j, ld in enumerate((rs.leadOne, rs.leadTwo)):
        a = z[var][i, j]
        over = {f: (bool(a[n]) if f in ("status", "radar") else float(a[n])) for n, f in enumerate(fl) if f != "radarTrackId"}
        over["radarTrackId"] = int(a[fl.index("radarTrackId")])
        leads.append(CL._O(ld, **over))
      return CL.LP._BoundedRadarState(rs, leads[0], leads[1])
  return rs


CL.xform = xform
CL.main()
