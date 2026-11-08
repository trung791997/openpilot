#!/usr/bin/env python3
"""Raw-CAN trace of the Bosch-A radar parser, per sweep and per track.

Feeds a route's logged CAN through the CURRENT tree's BoschARadar (opendbc_repo/opendbc/car/honda/bosch_a_radar.py)
and writes one JSON line per sweep that touched a selected track:

  raws        what the radar SENT on CAN for that track this sweep (decoded raw fields, slot)
  pub         what the parser PUBLISHED for it (dRel, vRel, measured)
  track_state the parser's persistent state for it (holds, runs, anchors)
  coast/newborn  calls made this sweep
  label       per track: RADAR_SENT (in CAN and published), DROPPED (in CAN, not published),
              PARSER_MADE (published, not in CAN this sweep: coast/hold), NO_CAN (neither)

This is a parser replay on logged CAN (evidence word: replay). It does not run radard or the planner.

  bosch_a_can_trace.py ~/routes_drive1108/000000ac--138f4b8bb4 --segs 49-51 --tracks 24,62 --out /tmp/ac_trace.jsonl
  bosch_a_can_trace.py seg49/rlog.zst seg50/rlog.zst --out trace.jsonl          # explicit files, every track
"""
import argparse
import json
import sys
from collections import Counter
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
for sub in ("", "opendbc_repo", "msgq_repo", "rednose_repo", "tinygrad_repo", "jetlink_repo"):
  p = str(REPO / sub) if sub else str(REPO)
  if p not in sys.path:
    sys.path.insert(0, p)

from openpilot.tools.lib.logreader import LogReader
from opendbc.car.honda import bosch_a_radar as bar
from opendbc.car.honda.radar_interface import RadarInterface

TRACK_STATE_FIELDS = ("last_trusted_vrel", "rail_hold", "floor_vrel", "newborn_vrel", "recovered", "range_anchor", "status")
TRACK_STATE_LENS = ("rejoin_samples", "inconsistent_run", "rejected_run", "samples")

# An empty slot carries every field at its maximum (TRACK_ID 255, STATUS 15, RANGE_RAW 4095). Skipped unless asked for.
EMPTY_SLOT_TRACK_ID = 255

CALLLOG: list = []


def _wrap(name):
  orig = getattr(bar, name)

  def inner(*a, **kw):
    r = orig(*a, **kw)
    CALLLOG.append((name, a, r))
    return r
  setattr(bar, name, inner)
  return orig


def parse_segs(spec: str) -> list[int]:
  out: list[int] = []
  for part in spec.split(","):
    if "-" in part:
      a, b = part.split("-")
      out.extend(range(int(a), int(b) + 1))
    elif part:
      out.append(int(part))
  return out


def resolve_paths(inputs: list[str], segs: str | None) -> list[str]:
  if len(inputs) == 1 and Path(inputs[0]).is_dir():
    route = Path(inputs[0])
    seg_ids = parse_segs(segs) if segs else sorted(int(d.name) for d in route.iterdir() if d.name.isdigit())
    paths = []
    for s in seg_ids:
      cands = [route / str(s) / n for n in ("rlog.zst", "rlog.bz2", "rlog")]
      found = next((c for c in cands if c.exists()), None)
      if found is None:
        print(f"segment {s}: no rlog, skipped", file=sys.stderr)
        continue
      paths.append(str(found))
    return paths
  if segs:
    raise SystemExit("--segs only applies to a route directory")
  return inputs


def _round(x, nd=3):
  return round(x, nd) if isinstance(x, float) else x


def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("inputs", nargs="+", help="a route directory (with --segs) or rlog files in order")
  ap.add_argument("--segs", help="segments of a route directory, e.g. 49-51 or 3,7,9 (default: all)")
  ap.add_argument("--tracks", help="comma-separated track IDs (default: every track)")
  ap.add_argument("--counts-per-mps", type=float, help="U11 counts per m/s override (72 now; 64 for 1/64-era logs, D-074)")
  ap.add_argument("--out", required=True, help="output JSONL path")
  args = ap.parse_args()

  paths = resolve_paths(args.inputs, args.segs)
  if not paths:
    raise SystemExit("no rlogs found")
  tracks = {int(t) for t in args.tracks.split(",")} if args.tracks else None

  orig_direct = bar._bosch_a_direct_vrel
  _wrap("_bosch_a_coast_vrel")
  _wrap("_bosch_a_newborn_vrel")

  CP = None
  for m in LogReader(paths[0]):
    if m.which() == "carParams":
      CP = m.carParams
      break
  if CP is None:
    raise SystemExit(f"no carParams in {paths[0]}")
  ri = RadarInterface(CP)
  bosch = ri.bosch_a
  if bosch is None:
    raise SystemExit(f"{CP.carFingerprint} has no Bosch-A radar interface")
  if args.counts_per_mps:
    bosch.u11_counts_per_mps = args.counts_per_mps
  print(f"fingerprint {CP.carFingerprint} flags {CP.flags} segs {len(paths)} u11 {bosch.u11_counts_per_mps}", file=sys.stderr)

  labels: dict[int, Counter] = {}
  n_rows = 0
  t0 = None
  seen: set[int] = set()
  with open(args.out, "w") as f:
    for path in paths:
      seg_name = Path(path).parent.name
      for m in LogReader(path):
        if m.which() != "can":
          continue
        t = int(m.logMonoTime)
        if t0 is None:
          t0 = t
        frames = [(c.address, c.dat, c.src) for c in m.can]
        seen.update(a for a, _, src in frames if src < 128)
        CALLLOG.clear()
        rd = bosch.update([(t, frames)])
        if rd is None:
          continue
        if bar.BOSCH_A_TRIGGER_MSG not in seen:  # stale-bus RadarData, no sweep
          continue

        raws = {}
        for slot in range(bar.BOSCH_A_NUM_SLOTS):
          f0, f1, f2, f3 = bar.BOSCH_A_MAIN_IDS[slot]
          aux = bar.BOSCH_A_AUX_IDS[slot]
          # Only slots whose full frame set arrived this sweep (the parser's own rule): vl keeps stale values.
          if not all(a in seen for a in (f0, f1, f2, f3)):
            continue
          tid = int(bosch.rcp.vl[f3]["TRACK_ID"])
          if (tracks is None and tid == EMPTY_SLOT_TRACK_ID) or (tracks is not None and tid not in tracks):
            continue
          v0, v1, v2, av = (bosch.rcp.vl[a] for a in (f0, f1, f2, aux))
          range_raw = int(v0["RANGE_RAW"])
          vr_raw = int(av["REL_VELOCITY_RAW"])
          vr_unc = int(av["REL_VELOCITY_UNCERTAINTY_RAW"])
          raws[tid] = {
            "slot": slot, "status": int(v0["STATUS"]), "range_raw": range_raw,
            "dRel_raw": round(bar.BOSCH_A_RANGE_SCALE_M * range_raw + bosch.range_offset_m, 3),
            "life": int(v2["LIFECYCLE_RAW"]), "existence_raw": int(v1["OBJECT_EXISTENCE_PROBABILITY_RAW"]),
            "range_sigma_raw": int(v0["RANGE_SIGMA_RAW"]), "direct_vrel_raw": vr_raw, "direct_vrel_uncertainty_raw": vr_unc,
            "direct_vrel": _round(orig_direct(vr_raw, vr_unc, bosch.u11_counts_per_mps)), "range_ratio_raw": int(av["RANGE_RATIO_RAW"]),
          }

        pub = {}
        for tid, p in bosch.pts.items():
          if tracks is None or tid in tracks:
            pub[tid] = {"dRel": round(p.dRel, 3), "vRel": round(p.vRel, 3), "measured": bool(p.deprecated.measured)}

        track_state = {}
        for tid in set(raws) | set(pub):
          tr = bosch._tracks.get(tid)
          if tr is None:
            continue
          st = {k: _round(getattr(tr, k, None)) for k in TRACK_STATE_FIELDS}
          for k in TRACK_STATE_LENS:
            v = getattr(tr, k, None)
            st[k] = None if v is None else len(v)
          track_state[tid] = st

        row_labels = {}
        for tid in (tracks if tracks is not None else set(raws) | set(pub)):
          lab = ("RADAR_SENT" if tid in pub else "DROPPED") if tid in raws else ("PARSER_MADE" if tid in pub else "NO_CAN")
          row_labels[tid] = lab
          labels.setdefault(tid, Counter())[lab] += 1

        if raws or pub:
          coast = [(c[1][0].track_id, _round(c[2])) for c in CALLLOG if c[0] == "_bosch_a_coast_vrel"]
          newborn = [_round(c[2]) for c in CALLLOG if c[0] == "_bosch_a_newborn_vrel"]
          f.write(json.dumps({"seg": seg_name, "t_mono": t, "t_s": round((t - t0) * 1e-9, 3), "label": row_labels, "raws": raws,
                                  "pub": pub, "track_state": track_state, "coast": coast, "newborn": newborn}) + "\n")
          n_rows += 1
        seen.clear()

  print(f"wrote {n_rows} rows to {args.out}", file=sys.stderr)
  for tid in sorted(labels):
    if tracks is not None or sum(labels[tid].values()) >= 10:
      print(f"track {tid}: " + " ".join(f"{k}={v}" for k, v in sorted(labels[tid].items())))


if __name__ == "__main__":
  main()
