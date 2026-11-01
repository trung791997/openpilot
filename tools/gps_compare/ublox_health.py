#!/usr/bin/env python3
"""Per-segment health of the comma's own u-blox GNSS from rlogs.

For each segment: ubloxRaw count, ubloxGnss message mix, antenna status (MON-HW aStatus/aPower), jamming,
noise, AGC, GPS week from RXM-RAWX, satellites seen / used (NAV-SAT), and gpsLocationExternal fix rate.
Also the monotonic time of the first message, which is seconds since the comma booted.

Usage (on the comma, offroad): /usr/local/venv/bin/python ublox_health.py /data/media/0/realdata ROUTE [ROUTE ...]
  ROUTE is e.g. 00000064--ca55382f07. Use --all-segs to read every segment (default: 0,1,2, every 5th, last).
Output: one JSON line per segment on stdout.
"""
import argparse
import collections
import json
import os
import sys

import zstandard

sys.path.insert(0, os.environ.get("OPENPILOT_ROOT", "/data/openpilot"))
try:
  from openpilot.cereal import log
except ImportError:
  from cereal import log

SERVICES = {"ubloxRaw", "ubloxGnss", "gpsLocationExternal", "initData", "deviceState", "managerState"}


def read_events(path):
  with open(path, "rb") as f:
    dat = f.read()
  if path.endswith(".zst"):
    dat = zstandard.ZstdDecompressor().decompressobj().decompress(dat)
  return log.Event.read_multiple_bytes(dat)


def seg_stats(path):
  s = {"raw": 0, "raw_bytes": 0, "gnss": collections.Counter(), "astatus": collections.Counter(),
       "apower": collections.Counter(), "jam": [], "noise": [], "agc": [], "weeks": collections.Counter(),
       "nummeas": [], "sats_seen": [], "sats_used": [], "sats_cno30": [], "cno": [], "gps": 0, "gps_fix": 0,
       "gps_sats": [], "gps_src": collections.Counter(), "first_mono": None, "pigeond_running": None,
       "pigeond_cpu": [], "cpu_max": []}
  for e in read_events(path):
    w = e.which()
    if s["first_mono"] is None:
      s["first_mono"] = e.logMonoTime / 1e9
    if w not in SERVICES:
      continue
    if w == "ubloxRaw":
      s["raw"] += 1
      s["raw_bytes"] += len(e.ubloxRaw)
    elif w == "ubloxGnss":
      g = e.ubloxGnss
      t = g.which()
      s["gnss"][t] += 1
      if t == "hwStatus":
        h = g.hwStatus
        s["astatus"][str(h.aStatus)] += 1
        s["apower"][str(h.aPower)] += 1
        s["jam"].append(h.jamInd)
        s["noise"].append(h.noisePerMS)
        s["agc"].append(h.agcCnt)
      elif t == "measurementReport":
        m = g.measurementReport
        s["weeks"][m.gpsWeek] += 1
        s["nummeas"].append(m.numMeas)
      elif t == "satReport":
        svs = g.satReport.svs
        s["sats_seen"].append(sum(1 for v in svs if v.cno > 0))
        s["sats_cno30"].append(sum(1 for v in svs if v.cno >= 30))
        s["cno"].extend(v.cno for v in svs if v.cno > 0)
        s["sats_used"].append(sum(1 for v in svs if v.flagsBitfield & 0x08))
    elif w == "gpsLocationExternal":
      gl = e.gpsLocationExternal
      s["gps"] += 1
      s["gps_src"][str(gl.source)] += 1
      if gl.hasFix:
        s["gps_fix"] += 1
      s["gps_sats"].append(gl.satelliteCount)
    elif w == "managerState":
      for p in e.managerState.processes:
        if p.name == "pigeond":
          s["pigeond_running"] = bool(p.running) if s["pigeond_running"] in (None, True) else False
    elif w == "deviceState":
      ds = e.deviceState
      if len(ds.cpuUsagePercent):
        s["cpu_max"].append(max(ds.cpuUsagePercent))
  return s


def summarize(v):
  if not v:
    return None
  v = sorted(v)
  return {"min": v[0], "med": v[len(v) // 2], "max": v[-1]}


def quantiles(v):
  if not v:
    return None
  v = sorted(v)
  return [v[len(v) // 2], v[int(.9 * (len(v) - 1))]]


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("root")
  ap.add_argument("routes", nargs="+")
  ap.add_argument("--all-segs", action="store_true")
  a = ap.parse_args()
  for r in a.routes:
    segs = sorted(int(d.rsplit("--", 1)[1]) for d in os.listdir(a.root) if d.startswith(r + "--"))
    if not segs:
      continue
    if not a.all_segs:
      pick = {segs[0], segs[-1]} | {x for x in segs if x <= 2 or x % 5 == 0}
      segs = sorted(pick)
    for n in segs:
      p = os.path.join(a.root, f"{r}--{n}", "rlog.zst")
      if not os.path.exists(p):
        continue
      try:
        s = seg_stats(p)
      except Exception as ex:  # truncated last segment, etc.
        print(json.dumps({"route": r, "seg": n, "error": repr(ex)[:120]}), flush=True)
        continue
      out = {"route": r, "seg": n, "boot_s": round(s["first_mono"] or 0), "raw": s["raw"], "raw_kB": s["raw_bytes"] // 1024,
             "gnss": dict(s["gnss"]), "aStatus": dict(s["astatus"]), "aPower": dict(s["apower"]),
             "jam": summarize(s["jam"]), "noise": summarize(s["noise"]), "agc": summarize(s["agc"]),
             "weeks": dict(s["weeks"]), "numMeas": summarize(s["nummeas"]), "sats_seen": summarize(s["sats_seen"]),
             "sats_cno30": summarize(s["sats_cno30"]), "cno_p50_p90": quantiles(s["cno"]), "sats_used": summarize(s["sats_used"]),
             "gps": s["gps"], "gps_fix_pct": round(100 * s["gps_fix"] / s["gps"]) if s["gps"] else None,
             "gps_sats": summarize(s["gps_sats"]), "gps_src": dict(s["gps_src"]), "pigeond_running": s["pigeond_running"],
             "cpu_max": summarize(s["cpu_max"])}
      print(json.dumps(out), flush=True)


if __name__ == "__main__":
  main()
