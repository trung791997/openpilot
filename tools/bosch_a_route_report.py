#!/usr/bin/env python3
"""Report the Bosch-A radar content of a route already on disk.

This is the offline twin of `tools/konik_preflight.py` step 8. The preflight answers
"can I reach Konik and pull a segment"; this answers the question that actually matters:

    does this route contain Bosch-A radar frames, and are any of them REAL TARGETS
    rather than the firmware's no-target sentinels?

Per STATUS.md, every clean Bosch-A replay this project has done so far contained only
sentinels. Parsing a sweep correctly is not the same as tracking a car, so a route full of
sentinels does not advance the radar work however cleanly it decodes. That distinction is
this tool's whole output.

Two passes, deliberately separate
---------------------------------
1. RAW CENSUS -- counts frames by CAN address and bus. Needs no DBC and no car params, so
   it works on any route and cannot be wrong about *presence*. Payloads are NOT hand-decoded
   here: AGENTS.md §5 forbids hand-decoding when a parser exists, and address counting needs
   no decode.
2. PARSER REPLAY -- feeds those frames through the real `RadarInterface` and reports what it
   published. Every semantic claim (targets, ranges, velocities) comes from the parser, never
   from this script's own arithmetic.

Accepts a route .zip (as downloaded from connect), a directory of segments, or a single
rlog / rlog.zst / rlog.bz2.

Usage
-----
    python tools/bosch_a_route_report.py <route.zip|dir|rlog>
    python tools/bosch_a_route_report.py <path> --segments 0-4      # only these segments
    python tools/bosch_a_route_report.py <path> --json report.json
    python tools/bosch_a_route_report.py <path> --fingerprint HONDA_CIVIC_BOSCH
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import tempfile
import zipfile
from collections import Counter

# realpath, not abspath: imported as `openpilot.tools.bosch_a_route_report`, __file__ runs through the
# repo's `openpilot/tools` symlink, and abspath would put `.../openpilot/tools/..` first on sys.path.
# Python resolves that `..` physically but capnp's kj filesystem resolves it lexically, so a later
# `import cereal` from there aborts looking for `openpilot/cereal/log.capnp` -- which is exactly how
# bosch_a_lifecycle_report.py crashed on its first real route.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.realpath(__file__))))

from openpilot.common.params import Params
from openpilot.tools.lib.logreader import _LogFileReader

RLOG_SUFFIXES = ("rlog", "rlog.zst", "rlog.bz2")


def _parse_segments(spec: str | None) -> set[int] | None:
  """'0-4,9' -> {0,1,2,3,4,9}. None means every segment."""
  if not spec:
    return None
  out: set[int] = set()
  for part in spec.split(","):
    part = part.strip()
    if "-" in part:
      lo, hi = part.split("-", 1)
      out.update(range(int(lo), int(hi) + 1))
    else:
      out.add(int(part))
  return out


def _segment_number(name: str) -> int:
  """Segment index from a path like '.../00000231--5782493b00--7/rlog.zst'."""
  for piece in reversed(name.replace("\\", "/").split("/")):
    if piece.isdigit():
      return int(piece)
    if "--" in piece:
      tail = piece.split("--")[-1]
      if tail.isdigit():
        return int(tail)
  return -1


def collect_inputs(path: str, tmpdir: str) -> list[tuple[int, str]]:
  """Return [(segment_number, local_path)] sorted by segment."""
  found: list[tuple[int, str]] = []

  if os.path.isfile(path) and path.endswith(".zip"):
    with zipfile.ZipFile(path) as zf:
      names = [n for n in zf.namelist() if n.endswith(RLOG_SUFFIXES)]
      if not names:
        qlogs = [n for n in zf.namelist() if "qlog" in n]
        raise SystemExit(
          f"No rlog files in {path}."
          + (f" It holds {len(qlogs)} qlog(s) -- qlogs are decimated and DROP the CAN data"
             + " this report needs. Re-download the rlogs." if qlogs else
             f" Archive holds {len(zf.namelist())} entries, none matching rlog*.")
        )
      # main() passes a TemporaryDirectory that already exists, so this only bites a direct
      # caller -- which is exactly what the test does. Create it rather than assume it.
      os.makedirs(tmpdir, exist_ok=True)
      for n in sorted(names):
        dest = os.path.join(tmpdir, n.replace("/", "_"))
        with zf.open(n) as src, open(dest, "wb") as dst:
          dst.write(src.read())
        found.append((_segment_number(n), dest))
    return sorted(found)

  if os.path.isdir(path):
    for root, _dirs, files in os.walk(path):
      for f in files:
        if f.endswith(RLOG_SUFFIXES):
          full = os.path.join(root, f)
          found.append((_segment_number(full), full))
    if not found:
      raise SystemExit(f"No rlog files under {path}")
    return sorted(found)

  if os.path.isfile(path):
    return [(_segment_number(path), path)]

  raise SystemExit(f"Not found: {path}")


def build_radar_interface(fingerprint: str):
  """The real RadarInterface for a Bosch-A car, or (None, reason)."""
  try:
    from opendbc.car.honda.interface import CarInterface
    from opendbc.car.honda.values import CAR, HONDA_BOSCH_A
    car = getattr(CAR, fingerprint)
    if car not in HONDA_BOSCH_A:
      return None, f"{fingerprint} is not in HONDA_BOSCH_A; parser replay skipped"
    Params().put_bool("BoschARadar", True)   # the parser is gated on this tester toggle
    CP = CarInterface.get_non_essential_params(car)
    return CarInterface.RadarInterface(CP), None
  except Exception as e:
    return None, f"could not build RadarInterface ({type(e).__name__}: {e})"


def analyse_segment(fn: str, bosch_ids: set[int], fingerprint: str) -> dict:
  """Raw census plus parser replay for one segment."""
  from opendbc.car.can_definitions import CanData

  res: dict = {
    "file": os.path.basename(fn),
    "msgs": 0,
    "bosch_frames": 0,
    "by_bus": Counter(),
    "by_addr": Counter(),
    "live_tracks": 0,
    "radar_state": 0,
    "radar_state_lead": 0,
    "car_fingerprint": None,
    "published_sweeps": 0,
    "published_points": 0,
    "track_ids": set(),
    "min_dRel": None,
    "max_dRel": None,
    "min_vRel": None,
    "max_vRel": None,
    "parser_note": None,
    "error": None,
  }

  ri, why = build_radar_interface(fingerprint)
  res["parser_note"] = why

  # Frames are handed to the parser grouped by logMonoTime, the same shape RadarInterface
  # sees on the car. Anything not in the Bosch-A address set is dropped: it cannot affect
  # this parser and passing it through only slows the replay.
  pending: list[CanData] = []
  pending_t = None

  def flush():
    nonlocal pending, pending_t
    if ri is None or not pending or pending_t is None:
      pending = []
      return
    try:
      rr = ri.update([(pending_t, pending)])
    except Exception:
      pending = []
      return
    if rr is not None and rr.points:
      res["published_sweeps"] += 1
      res["published_points"] += len(rr.points)
      for pt in rr.points:
        res["track_ids"].add(int(pt.trackId))
        for key, val in (("dRel", pt.dRel), ("vRel", pt.vRel)):
          lo, hi = res[f"min_{key}"], res[f"max_{key}"]
          res[f"min_{key}"] = val if lo is None else min(lo, val)
          res[f"max_{key}"] = val if hi is None else max(hi, val)
    pending = []

  try:
    for msg in _LogFileReader(fn):
      res["msgs"] += 1
      w = msg.which()
      if w == "can":
        t = msg.logMonoTime
        if pending_t is not None and t != pending_t:
          flush()
        pending_t = t
        for f in msg.can:
          if f.address in bosch_ids:
            res["bosch_frames"] += 1
            res["by_bus"][int(f.src)] += 1
            res["by_addr"][int(f.address)] += 1
            pending.append(CanData(f.address, bytes(f.dat), f.src))
      elif w == "liveTracks":
        res["live_tracks"] += 1
      elif w == "radarState":
        res["radar_state"] += 1
        try:
          if msg.radarState.leadOne.status:
            res["radar_state_lead"] += 1
        except Exception:
          pass
      elif w == "carParams" and res["car_fingerprint"] is None:
        res["car_fingerprint"] = str(msg.carParams.carFingerprint)
    flush()
  except Exception as e:
    res["error"] = f"{type(e).__name__}: {e}"

  return res


def main() -> int:
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("path", help="route .zip, a directory of segments, or a single rlog")
  ap.add_argument("--segments", help="only these segment numbers, e.g. '0-4,9'")
  ap.add_argument("--fingerprint", default="HONDA_CRV_5G", help="car fingerprint for parser replay")
  ap.add_argument("--json", dest="json_out", help="write the full report here")
  args = ap.parse_args()

  from opendbc.car.honda.radar_interface import BOSCH_A_ALL_IDS, BOSCH_A_MAIN_IDS
  bosch_ids = set(BOSCH_A_ALL_IDS)
  trigger = BOSCH_A_MAIN_IDS[15][3]
  first_f0 = BOSCH_A_MAIN_IDS[0][0]

  with tempfile.TemporaryDirectory() as tmp:
    inputs = collect_inputs(args.path, tmp)
    wanted = _parse_segments(args.segments)
    if wanted is not None:
      inputs = [(n, p) for n, p in inputs if n in wanted]
      if not inputs:
        raise SystemExit(f"No segments matched --segments {args.segments}")

    print(f"Bosch-A route report: {args.path}")
    print(f"{len(inputs)} segment(s); {len(bosch_ids)} Bosch-A addresses; "
          + f"trigger 0x{trigger:03X}, slot-0 f0 0x{first_f0:03X}")
    print("=" * 78)
    print(f"{'seg':>4}  {'msgs':>9}  {'boschA':>8}  {'bus':>6}  {'sweeps':>7}  "
          + f"{'points':>7}  {'tracks':>6}  note")
    print("-" * 78)

    segs = []
    totals: dict = {
      "bosch_frames": 0, "published_sweeps": 0, "published_points": 0,
      "live_tracks": 0, "radar_state": 0, "radar_state_lead": 0, "msgs": 0,
    }
    all_tracks: set[int] = set()
    all_addr: Counter = Counter()
    all_bus: Counter = Counter()
    fingerprints: set[str] = set()
    d_lo = d_hi = v_lo = v_hi = None

    for n, p in inputs:
      r = analyse_segment(p, bosch_ids, args.fingerprint)
      segs.append({"segment": n, **{k: (sorted(v) if isinstance(v, set) else
                                        dict(v) if isinstance(v, Counter) else v)
                                    for k, v in r.items()}})
      for k in totals:
        totals[k] += r[k]
      all_tracks |= r["track_ids"]
      all_addr.update(r["by_addr"])
      all_bus.update(r["by_bus"])
      if r["car_fingerprint"]:
        fingerprints.add(r["car_fingerprint"])
      if r["min_dRel"] is not None:
        d_lo = r["min_dRel"] if d_lo is None else min(d_lo, r["min_dRel"])
        d_hi = r["max_dRel"] if d_hi is None else max(d_hi, r["max_dRel"])
      if r["min_vRel"] is not None:
        v_lo = r["min_vRel"] if v_lo is None else min(v_lo, r["min_vRel"])
        v_hi = r["max_vRel"] if v_hi is None else max(v_hi, r["max_vRel"])

      note = r["error"] or r["parser_note"] or ""
      buses = ",".join(str(b) for b in sorted(r["by_bus"])) or "-"
      print(f"{n:>4}  {r['msgs']:>9}  {r['bosch_frames']:>8}  {buses:>6}  "
            + f"{r['published_sweeps']:>7}  {r['published_points']:>7}  "
            + f"{len(r['track_ids']):>6}  {note[:28]}")

    print("-" * 78)
    print(f"{'ALL':>4}  {totals['msgs']:>9}  {totals['bosch_frames']:>8}  "
          + f"{','.join(str(b) for b in sorted(all_bus)) or '-':>6}  "
          + f"{totals['published_sweeps']:>7}  {totals['published_points']:>7}  "
          + f"{len(all_tracks):>6}")
    print()

    # ---- verdict -----------------------------------------------------------
    print("Findings")
    print("-" * 78)
    if fingerprints:
      print(f"  car (from carParams): {', '.join(sorted(fingerprints))}")
    if totals["bosch_frames"] == 0:
      print("  NO Bosch-A object frames anywhere in this route.")
      print("  Either the car is not Bosch-A, the camera bus was not logged, or the radar")
      print("  was silent. This route cannot advance the Bosch-A work.")
    else:
      print(f"  Bosch-A frames: {totals['bosch_frames']} on bus(es) "
            + f"{sorted(all_bus)}, {len(all_addr)} distinct addresses of {len(bosch_ids)}")
      print(f"  Sweep triggers (0x{trigger:03X}): {all_addr.get(trigger, 0)}")
      if totals["published_points"] == 0:
        print("  The parser published NO radar points. Consistent with every prior replay:")
        print("  all objects were firmware no-target sentinels. Frames present, targets not.")
      else:
        print(f"  REAL TARGETS: the parser published {totals['published_points']} point(s) "
              + f"across {totals['published_sweeps']} sweep(s),")
        print(f"  {len(all_tracks)} distinct track id(s). "
              + f"dRel {d_lo:.1f}..{d_hi:.1f} m, vRel {v_lo:.2f}..{v_hi:.2f} m/s")
        print("  This is the first route evidence the radar work has been waiting on.")
    if totals["live_tracks"] or totals["radar_state"]:
      print(f"  Device-side: liveTracks {totals['live_tracks']}, radarState "
            + f"{totals['radar_state']} ({totals['radar_state_lead']} with a lead)")
    print()
    print("  Scope: frame counts and parser output only. Nothing here is road validation,")
    print("  and no gate threshold should be re-tuned from one route (D-042).")

    if args.json_out:
      with open(args.json_out, "w") as f:
        json.dump({
          "path": args.path, "fingerprint": args.fingerprint,
          "totals": totals, "addresses": {hex(k): v for k, v in sorted(all_addr.items())},
          "buses": dict(all_bus), "track_ids": sorted(all_tracks),
          "dRel_range": [d_lo, d_hi], "vRel_range": [v_lo, v_hi],
          "car_fingerprints": sorted(fingerprints), "segments": segs,
        }, f, indent=2, default=str)
      print(f"\n  wrote {args.json_out}")

  return 0


if __name__ == "__main__":
  sys.exit(main())
