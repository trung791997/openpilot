#!/usr/bin/env python3
"""Per-sweep per-point Bosch-A table from rlogs for D-071 (LOG/REPLAY only; outputs stay outside the repo).

The current parser is re-run on the logged CAN; NORMALIZED_CLOSING raw and sigma are captured per measured point.
Usage: ncveto_extract.py <routes_dir> <out_dir> <route>   -> <out_dir>/pts_<route>.csv
"""
import csv
import sys
from pathlib import Path

from openpilot.common.swaglog import cloudlog
from openpilot.tools.lib.logreader import LogReader
from opendbc.car.can_definitions import CanData
from opendbc.car.honda import radar_interface as HRI
from opendbc.car.honda.interface import CarInterface
from opendbc.car.honda.values import CAR

for _l in ("info", "warning", "error", "exception", "event", "debug"):
  setattr(cloudlog, _l, lambda *a, **k: None)

NC: dict = {}
# Spy the always-called publisher (5d7be6e730+). _bosch_a_nc_vrel runs only with BOSCH_A_NC_RAIL_VREL on (D-069, off),
# so spying it left nc_raw empty on every row (found by Bob, 2026-09-30).
_orig_nc = HRI._bosch_a_nc_published


def _spy(nc_raw, nc_sigma_raw, d_rel):
  NC[float(d_rel)] = (nc_raw, nc_sigma_raw)
  return _orig_nc(nc_raw, nc_sigma_raw, d_rel)


HRI._bosch_a_nc_published = _spy
IDS = frozenset(HRI.BOSCH_A_ALL_IDS)


def main():
  routes, out, route = Path(sys.argv[1]), Path(sys.argv[2]), sys.argv[3]
  segs = sorted(int(p.name) for p in (routes / route).iterdir())
  t0 = next(m.logMonoTime for m in LogReader(str(routes / route / "0" / "rlog.zst"), sort_by_time=True)
            if m.which() == "initData")
  ri = None
  vego = aego = 0.0
  with open(out / f"pts_{route}.csv", "w") as fh:
    w = csv.writer(fh)
    w.writerow(["seg", "t", "tid", "d", "y", "v", "meas", "nc_raw", "nc_sig", "vego", "aego"])
    for seg in segs:
      if seg == 0 or (seg - 1) not in segs:
        ri = None
      if ri is None:
        cp = next(m.carParams for m in LogReader(str(routes / route / str(seg) / "rlog.zst"), sort_by_time=True)
                  if m.which() == "carParams")
        ocp = CarInterface.get_non_essential_params(getattr(CAR, str(cp.carFingerprint)))
        ocp.radarUnavailable = bool(cp.radarUnavailable)
        ocp.openpilotLongitudinalControl = bool(cp.openpilotLongitudinalControl)
        ri = CarInterface.RadarInterface(ocp)
        assert ri.bosch_a_radar
      for m in LogReader(str(routes / route / str(seg) / "rlog.zst"), sort_by_time=True):
        wh = m.which()
        if wh == "carState":
          vego, aego = m.carState.vEgo, m.carState.aEgo
          continue
        if wh != "can":
          continue
        b = [CanData(c.address, bytes(c.dat), c.src) for c in m.can if c.address in IDS]
        if not b:
          continue
        ri.v_ego = vego
        NC.clear()
        rr = ri.update([(m.logMonoTime, b)])
        if rr is None:
          continue
        t = (m.logMonoTime - t0) / 1e9
        for p in rr.points:
          r = NC.get(float(p.dRel), (None, None)) if p.measured else (None, None)
          w.writerow([seg, f"{t:.3f}", p.trackId, f"{p.dRel:.3f}", f"{p.yRel:.3f}", f"{p.vRel:.3f}", int(p.measured),
                      "" if r[0] is None else int(r[0]), "" if r[1] is None else int(r[1]), f"{vego:.3f}", f"{aego:.3f}"])
  print("done", route)


if __name__ == "__main__":
  main()
