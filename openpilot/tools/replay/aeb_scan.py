#!/usr/bin/env python3
"""Scan a route's rlogs for AEB / FCW activity and hard decelerations nobody commanded.

Reads what the logs carry. It does not re-run any controller (evidence word: logged, not replay).

Sources, each reported on its own:
  onroadEvents        aeb, stockAeb, fcw, stockFcw
  longitudinalPlan    fcw (planner crash prediction)
  modelV2.meta        hardBrakePredicted (the model's FCW input, selfdrived.py)
  carState            stockAeb, stockFcw
  hard decel          carState.aEgo <= --decel for >= --min-dur s, labelled by what explains it:
                      PEDAL (brakePressed), COMMANDED (op's actuators.accel reached it), UNCOMMANDED (neither)

Caveat on carState.stockAeb: honda/carstate.py forces it False on a Bosch car under openpilot longitudinal
(`not openpilotLongitudinalControl`), and Bosch has no stockFcw decode. On those cars a stock AEB brake can
only show up here as an UNCOMMANDED hard decel. The header line prints openpilotLongitudinalControl.

  aeb_scan.py ~/routes_drive1108/000000ac--138f4b8bb4 --out /tmp/ac_aeb.json
"""
import argparse
import json
import sys
from datetime import datetime
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
for sub in ("", "opendbc_repo", "msgq_repo", "rednose_repo", "tinygrad_repo", "jetlink_repo"):
  p = str(REPO / sub) if sub else str(REPO)
  if p not in sys.path:
    sys.path.insert(0, p)

from openpilot.tools.lib.logreader import LogReader

AEB_EVENTS = ("aeb", "stockAeb", "fcw", "stockFcw")
# A window is COMMANDED when op asked for at least this share of the measured decel.
COMMANDED_SHARE = 0.7


def seg_paths(route: Path, segs: str | None) -> list[tuple[int, Path]]:
  if segs:
    ids = []
    for part in segs.split(","):
      a, _, b = part.partition("-")
      ids.extend(range(int(a), int(b or a) + 1))
  else:
    ids = sorted(int(d.name) for d in route.iterdir() if d.name.isdigit())
  out = []
  for s in ids:
    found = next((route / str(s) / n for n in ("rlog.zst", "rlog.bz2", "rlog") if (route / str(s) / n).exists()), None)
    if found is None:
      print(f"segment {s}: no rlog, skipped", file=sys.stderr)
    else:
      out.append((s, found))
  return out


class Edges:
  """Collapses a per-message boolean into [start, end] runs."""
  def __init__(self):
    self.runs: list[dict] = []
    self.open: dict | None = None

  def step(self, on: bool, stamp: dict):
    if on and self.open is None:
      self.open = {"start": stamp, "end": stamp, "n": 0}
    if on:
      self.open["end"] = stamp
      self.open["n"] += 1
    elif self.open is not None:
      self.runs.append(self.open)
      self.open = None

  def close(self):
    if self.open is not None:
      self.runs.append(self.open)
      self.open = None


def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("route", help="route directory holding <seg>/rlog.zst")
  ap.add_argument("--segs", help="e.g. 0-37 or 3,7 (default: all)")
  ap.add_argument("--decel", type=float, default=-3.0, help="hard-decel threshold on aEgo, m/s^2 (default -3.0)")
  ap.add_argument("--min-dur", type=float, default=0.3, help="minimum hard-decel duration, s (default 0.3)")
  ap.add_argument("--out", help="write the full result as JSON")
  args = ap.parse_args()

  paths = seg_paths(Path(args.route).expanduser(), args.segs)
  if not paths:
    raise SystemExit("no rlogs found")

  CP = None
  events = {name: Edges() for name in AEB_EVENTS}
  sources = {k: Edges() for k in ("longitudinalPlan.fcw", "modelV2.hardBrakePredicted", "carState.stockAeb", "carState.stockFcw")}
  decels: list[dict] = []
  cur = None  # open hard-decel window
  t0 = None
  wall_off = None  # wall ns - mono ns, from the first clock source seen
  cc_accel = 0.0
  long_active = False
  enabled = False
  lead = None

  def stamp(seg, t):
    d = {"seg": seg, "t_s": round((t - t0) * 1e-9, 2)}
    if wall_off is not None:
      d["wall"] = datetime.fromtimestamp((t + wall_off) * 1e-9).strftime("%H:%M:%S.%f")[:-4]
    return d

  for seg, path in paths:
    for m in LogReader(str(path), sort_by_time=True):
      w = m.which()
      t = m.logMonoTime
      if t0 is None:
        t0 = t
      if w == "carParams" and CP is None:
        CP = m.carParams
      elif w == "clocks" and wall_off is None and m.clocks.wallTimeNanos:
        wall_off = m.clocks.wallTimeNanos - t
      elif w == "gpsLocationExternal" and wall_off is None and m.gpsLocationExternal.unixTimestampMillis:
        wall_off = m.gpsLocationExternal.unixTimestampMillis * 1_000_000 - t
      elif w == "onroadEvents":
        names = {str(e.name) for e in m.onroadEvents}
        for name, edge in events.items():
          edge.step(name in names, stamp(seg, t))
      elif w == "longitudinalPlan":
        sources["longitudinalPlan.fcw"].step(bool(m.longitudinalPlan.fcw), stamp(seg, t))
      elif w == "modelV2":
        sources["modelV2.hardBrakePredicted"].step(bool(m.modelV2.meta.hardBrakePredicted), stamp(seg, t))
      elif w == "carControl":
        cc_accel = m.carControl.actuators.accel
        long_active = m.carControl.longActive
      elif w == "selfdriveState":
        enabled = m.selfdriveState.enabled
      elif w == "radarState":
        l1 = m.radarState.leadOne
        lead = [round(l1.dRel, 1), round(l1.vRel, 2), bool(l1.radar)] if l1.present else None
      elif w == "carState":
        cs = m.carState
        st = stamp(seg, t)
        sources["carState.stockAeb"].step(bool(cs.stockAeb), st)
        sources["carState.stockFcw"].step(bool(cs.stockFcw), st)
        if cs.aEgo <= args.decel:
          if cur is None:
            cur = {"start": st, "v0": round(cs.vEgo, 2), "a_min": cs.aEgo, "cmd_min": cc_accel, "pedal": False,
                   "long_active": False, "enabled": False, "lead_at_start": lead, "t_start": t}
          cur["end"] = st
          cur["t_end"] = t
          cur["a_min"] = min(cur["a_min"], cs.aEgo)
          cur["cmd_min"] = min(cur["cmd_min"], cc_accel if long_active else 0.0)
          cur["pedal"] |= bool(cs.brakePressed)
          cur["long_active"] |= bool(long_active)
          cur["enabled"] |= bool(enabled)
        elif cur is not None:
          decels.append(cur)
          cur = None
  if cur is not None:
    decels.append(cur)
  for edge in list(events.values()) + list(sources.values()):
    edge.close()

  hard = []
  for d in decels:
    dur = (d.pop("t_end") - d.pop("t_start")) * 1e-9
    if dur < args.min_dur:
      continue
    d["dur_s"] = round(dur, 2)
    d["a_min"] = round(d["a_min"], 2)
    d["cmd_min"] = round(d["cmd_min"], 2)
    if d["pedal"]:
      d["label"] = "PEDAL"
    elif d["cmd_min"] <= COMMANDED_SHARE * d["a_min"]:
      d["label"] = "COMMANDED"
    else:
      d["label"] = "UNCOMMANDED"
    hard.append(d)

  op_long = None if CP is None else bool(CP.openpilotLongitudinalControl)
  result = {
    "route": str(args.route), "segments": [s for s, _ in paths], "fingerprint": None if CP is None else str(CP.carFingerprint),
    "openpilotLongitudinalControl": op_long, "decel_threshold": args.decel, "min_dur_s": args.min_dur,
    "events": {k: v.runs for k, v in events.items()}, "signals": {k: v.runs for k, v in sources.items()}, "hard_decel": hard,
  }

  print(f"{result['fingerprint']}  opLong={op_long}  segments={len(paths)}  wall_clock={'yes' if wall_off is not None else 'no'}")
  if op_long:
    print("  note: carState.stockAeb is forced False under op long on Bosch (honda/carstate.py); see UNCOMMANDED decels")
  for k, runs in list(result["events"].items()) + list(result["signals"].items()):
    print(f"  {k:28s} {len(runs)} run(s)" + "".join(f"\n    {r['start']} .. {r['end']}" for r in runs[:20]))
  counts = {lab: sum(d["label"] == lab for d in hard) for lab in ("PEDAL", "COMMANDED", "UNCOMMANDED")}
  print(f"  hard decel (aEgo<={args.decel} for >={args.min_dur}s): {len(hard)}  " + " ".join(f"{k}={v}" for k, v in counts.items()))
  for d in hard:
    print(f"    {d['label']:11s} {d['start']} dur={d['dur_s']}s v0={d['v0']} a_min={d['a_min']} cmd_min={d['cmd_min']} " +
          f"longActive={d['long_active']} lead={d['lead_at_start']}")
  if args.out:
    with open(args.out, "w") as f:
      json.dump(result, f, indent=1)
    print(f"wrote {args.out}", file=sys.stderr)


if __name__ == "__main__":
  main()
