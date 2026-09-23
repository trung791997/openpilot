#!/usr/bin/env python3
"""Census of Bosch-A track-identity lifecycles on a real drive: D-049's validation gate.

Why this exists
---------------
D-049 settled, by construction, that a Bosch-A track-ID reuse resets radard's lead Kalman filter
only through a ONE-sweep absence of the CAN identity from RadarData, and it left three questions
that only route data can answer:

  1. how often lifecycle breaks actually occur on real routes, and whether any coincide with a
     hard brake;
  2. whether a SEAMLESS reuse exists -- a new object under an old ID with no lifecycle break --
     in which case nothing resets anywhere;
  3. a negative control over segments with no break.

This script answers 1 and 2 and supplies the counts 3 needs. It replays CAN through the real
`RadarInterface` (built by `bosch_a_route_report.build_radar_interface`, not copied) and watches
the parser's own per-identity state across each sweep, so a "break" here is a sweep on which the
parser itself saw a lifecycle discontinuity, not a re-derivation from raw frames.

What it measures
----------------
* **Breaks.** An identity observed on consecutive sweeps whose (frame index, lifecycle counter)
  step fails the parser's `life_delta == 2 * frame_delta` rule. For each: how many sweeps the
  identity is then absent from published points (D-049 predicts exactly 1), and whether a hard
  brake was commanded within +/-BRAKE_WINDOW_S while the device's `leadOne` was that identity.
* **Seamless-reuse candidates.** A published point that continues on the next sweep with NO break
  but jumps laterally by more than LATERAL_JUMP_M in one sweep. The innovation gate checks range
  only, so a new object at a similar range under the old ID would pass it and show up here and
  nowhere else. A candidate is a lead to inspect, not a confirmed reuse: azimuth noise at long
  range can produce it too, which is why the full per-sweep lateral-step distribution is reported
  alongside so the threshold can be judged.
* **Rejection runs.** Consecutive sweeps on which an identity is observed but its point is
  published unmeasured. A reuse at a DIFFERENT range is rejected by the innovation gate every
  sweep until the old point goes stale, so long runs are the other place a seamless reuse hides.

What it is NOT
--------------
Offline replay of recorded CAN through the current parser. It says what this commit's parser
does with that drive's frames, not what the device that recorded it did, and nothing about the
closed loop. The hard-brake coincidence uses the DEVICE's recorded `carControl`/`radarState`,
which came from whatever commit was on the car.

Usage
-----
    python tools/bosch_a_lifecycle_report.py <route.zip|dir|rlog> --fingerprint HONDA_CIVIC_BOSCH
    python tools/bosch_a_lifecycle_report.py <path> --segments 0-5 --json lifecycle.json
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.realpath(__file__))))  # realpath: see bosch_a_route_report

from opendbc.car.honda.radar_interface import BOSCH_A_LIFE_SATURATED
from openpilot.tools.bosch_a_route_report import _parse_segments, _segment_number, build_radar_interface, collect_inputs
from openpilot.tools.lib.logreader import _LogFileReader

# One Bosch-A sweep is ~70 ms (14.35 Hz). 1.5 m of lateral motion in one sweep is ~21 m/s
# sideways, which no vehicle does; below it, azimuth noise at range dominates. Reporting threshold
# only -- nothing consumes it -- and the full distribution is printed so it can be second-guessed.
LATERAL_JUMP_M = 1.5

# A lateral-step candidate with another candidate on the same track within this window is part of a
# smooth sweep, not an isolated step. ~4 sweeps.
ISOLATION_WINDOW_S = 0.3

# Same reporting threshold the corpus report uses for a hard brake.
HARD_BRAKE_ACCEL = -1.5
BRAKE_WINDOW_S = 2.0

# Breaks on one identity closer together than this belong to one chronic run (~3 sweeps).
CHRONIC_GAP_S = 0.25

# A rejection run shorter than this is ordinary single-sweep noise.
REJECTION_RUN_MIN_SWEEPS = 3

PERCENTILES = (50, 90, 95, 99)


def is_same_incarnation(prev_frame_idx: int, prev_life: int, frame_idx: int, life: int) -> bool:
  """The parser's lifecycle-continuity rule (radar_interface._update_bosch_a).

  Mirrored rather than imported because the parser computes it inline -- but the SATURATION value
  is imported, not copied, so the two cannot drift apart on the number that matters. The tests drive
  the real parser across a continuity step, a break and a saturated hold, and assert this agrees
  with what it did, so a change there breaks them instead of silently changing the census.
  """
  frame_delta = (frame_idx - prev_frame_idx) & 0xF
  life_delta = (life - prev_life) & 0xFFF
  if life_delta == 2 * frame_delta:
    return True
  # A counter pinned at its maximum cannot advance and so cannot testify to identity either way;
  # the parser treats saturated -> saturated as a continuation. Counting those as breaks reported
  # 1,991 "breaks" on 00000232--fc8dad0d18 for an object the parser was publishing normally.
  return life == BOSCH_A_LIFE_SATURATED and prev_life == BOSCH_A_LIFE_SATURATED


def percentiles(values: list[float]) -> dict:
  if not values:
    return {"n": 0}
  s = sorted(values)
  out = {"n": len(s), "max": s[-1]}
  for p in PERCENTILES:
    k = (len(s) - 1) * p / 100.0
    lo, hi = math.floor(k), math.ceil(k)
    out[f"p{p}"] = s[lo] + (s[hi] - s[lo]) * (k - lo)
  return out


class LifecycleCensus:
  """Feeds grouped CAN into a RadarInterface and records identity events per sweep."""

  def __init__(self, ri, *, lateral_jump_m: float = LATERAL_JUMP_M):
    self.ri = ri
    self.lateral_jump_m = lateral_jump_m
    self.sweeps = 0
    self.update_errors = 0
    self.first_error: str | None = None
    self.breaks: list[dict] = []
    self.fresh = 0                      # identity observed with no prior state (new, or after retirement)
    self.continuations = 0              # identity observed on consecutive state with no break
    self.lateral_steps: list[float] = []
    self.candidates: list[dict] = []
    self.rejection_runs: list[dict] = []
    self._prev_points: dict[int, tuple[float, float, float, float]] = {}  # id -> (t, dRel, yRel, vRel)
    self._pending_absence: dict[int, dict] = {}   # id -> break awaiting its reappearance
    self._reject_run: dict[int, dict] = {}

  def _snapshot(self) -> dict[int, tuple]:
    return {tid: (tr.prev_frame_idx, tr.prev_life) for tid, tr in self.ri._tracks.items()}

  def feed(self, t_nanos: int, t_rel: float, frames: list) -> None:
    before = self._snapshot()
    try:
      rr = self.ri.update([(t_nanos, frames)])
    except Exception as e:  # counted and reported, never swallowed silently
      self.update_errors += 1
      if self.first_error is None:
        self.first_error = f"{type(e).__name__}: {e}"
      return
    if rr is None:
      return
    self.sweeps += 1

    observed: set[int] = set()
    for tid, tr in self.ri._tracks.items():
      now_state = (tr.prev_frame_idx, tr.prev_life)
      prev_state = before.get(tid)
      if prev_state == now_state:
        continue
      observed.add(tid)
      if prev_state is None or None in prev_state:
        self.fresh += 1
        continue
      if is_same_incarnation(prev_state[0], prev_state[1], now_state[0], now_state[1]):
        self.continuations += 1
      else:
        prior = self._pending_absence.get(tid)
        if prior is not None:
          # Broken again before it was ever republished: the earlier break did not "never reappear",
          # it was superseded. Without this, a chronic break reads as N permanent losses.
          prior["superseded"] = True
        ev = {"t": t_rel, "trackId": tid, "absentSweeps": 0, "reappeared": False, "superseded": False,
              "life": now_state[1]}
        self.breaks.append(ev)
        self._pending_absence[tid] = ev

    points = {int(p.trackId): p for p in rr.points}

    for tid, ev in list(self._pending_absence.items()):
      if tid in points:
        ev["reappeared"] = True
        del self._pending_absence[tid]
      else:
        ev["absentSweeps"] += 1

    for tid, p in points.items():
      prev = self._prev_points.get(tid)
      broke = any(b["trackId"] == tid and b["t"] == t_rel for b in self.breaks[-4:])
      if prev is not None and not broke and p.measured:
        dy = abs(float(p.yRel) - prev[2])
        self.lateral_steps.append(dy)
        if dy > self.lateral_jump_m:
          self.candidates.append({"t": t_rel, "trackId": tid, "dRel": float(p.dRel),
                                  "yPrev": prev[2], "y": float(p.yRel), "dy": dy})

      run = self._reject_run.get(tid)
      if tid in observed and not p.measured:
        if run is None:
          self._reject_run[tid] = {"tStart": t_rel, "trackId": tid, "sweeps": 1, "dRel": float(p.dRel)}
        else:
          run["sweeps"] += 1
      elif run is not None:
        self._close_run(tid)

    for tid in list(self._reject_run):
      if tid not in points:
        self._close_run(tid)

    self._prev_points = {tid: (t_rel, float(p.dRel), float(p.yRel), float(p.vRel)) for tid, p in points.items()}

  def _close_run(self, tid: int) -> None:
    run = self._reject_run.pop(tid)
    if run["sweeps"] >= REJECTION_RUN_MIN_SWEEPS:
      self.rejection_runs.append(run)

  def finish(self) -> None:
    for tid in list(self._reject_run):
      self._close_run(tid)
    mark_isolated(self.candidates)


def mark_isolated(candidates: list[dict], window_s: float = ISOLATION_WINDOW_S) -> None:
  """Flag candidates with no other candidate on the same track within +/-window_s.

  An identity swap is one lateral step followed by steady readings. A run of consecutive large steps
  on one track is a smooth sweep -- on 00000231--5782493b00/4 track 32 walked y +10.3 -> -1.4 m over
  seven sweeps at a steady ~51 m, which is what ego yaw does to a real target in the car frame.
  Only isolated steps are candidates for a seamless reuse.
  """
  for c in candidates:
    c["isolated"] = not any(o is not c and o["trackId"] == c["trackId"] and abs(o["t"] - c["t"]) <= window_s
                            for o in candidates)


def chronic_runs(breaks: list[dict], max_gap_s: float = CHRONIC_GAP_S) -> list[dict]:
  """Group consecutive breaks on one identity into runs.

  A single break is D-049's case: one sweep of absence, then the identity returns. A run of breaks on
  consecutive sweeps is a different failure: the identity is observed every sweep and never
  published. On 00000232--fc8dad0d18/11 track 37 did this for >7 s with LIFECYCLE_RAW frozen at 4094.
  """
  runs: list[dict] = []
  open_runs: dict[int, dict] = {}
  for b in sorted(breaks, key=lambda b: b["t"]):
    run = open_runs.get(b["trackId"])
    if run is not None and b["t"] - run["tEnd"] <= max_gap_s:
      run["tEnd"] = b["t"]
      run["breaks"] += 1
      run["lives"].add(b.get("life"))
      if b.get("hardestBrakeNearby") is not None:
        run["hardestBrakeNearby"] = min(run["hardestBrakeNearby"] or 0.0, b["hardestBrakeNearby"])
      run["wasDeviceLead"] = run["wasDeviceLead"] or b.get("wasDeviceLead", False)
      continue
    run = {"trackId": b["trackId"], "tStart": b["t"], "tEnd": b["t"], "breaks": 1, "lives": {b.get("life")},
           "hardestBrakeNearby": b.get("hardestBrakeNearby"), "wasDeviceLead": b.get("wasDeviceLead", False)}
    runs.append(run)
    open_runs[b["trackId"]] = run
  for run in runs:
    run["durationS"] = run["tEnd"] - run["tStart"]
    run["lives"] = sorted(v for v in run["lives"] if v is not None)[:5]
  return runs


def attach_brakes(breaks: list[dict], brake_times: list[tuple[float, float]],
                  lead_ids: list[tuple[float, int]], window_s: float = BRAKE_WINDOW_S) -> None:
  """Mark each break with the hardest device brake within the window, and whether the device's
  leadOne was that identity at the break. Both lists are (t, value) sorted by t."""
  for ev in breaks:
    near = [a for t, a in brake_times if abs(t - ev["t"]) <= window_s]
    ev["hardestBrakeNearby"] = min(near) if near else None
    lead_at = None
    for t, tid in lead_ids:
      if t > ev["t"]:
        break
      lead_at = tid
    ev["wasDeviceLead"] = lead_at == ev["trackId"]


def analyse_segment(path: str, fingerprint: str, bosch_ids: set[int], *,
                    brake_threshold: float = HARD_BRAKE_ACCEL, lateral_jump_m: float = LATERAL_JUMP_M) -> dict:
  from opendbc.car.can_definitions import CanData

  ri, why = build_radar_interface(fingerprint)
  if ri is None:
    raise SystemExit(why)
  census = LifecycleCensus(ri, lateral_jump_m=lateral_jump_m)

  t0 = None
  pending: list = []
  pending_t = None
  brake_times: list[tuple[float, float]] = []
  lead_ids: list[tuple[float, int]] = []
  enabled = False

  for msg in _LogFileReader(path):
    w = msg.which()
    if w in ("initData", "sentinel"):
      # initData is stamped at logger start, ~600 s before a later segment's data; as the origin it
      # would report every event at route time (same defect as fixed in bosch_a_corpus_report.py).
      continue
    t = msg.logMonoTime
    if t0 is None:
      t0 = t
    if w == "can":
      if pending_t is not None and t != pending_t and pending:
        census.feed(pending_t, (pending_t - t0) * 1e-9, pending)
        pending = []
      pending_t = t
      for f in msg.can:
        if f.address in bosch_ids:
          pending.append(CanData(f.address, bytes(f.dat), f.src))
    elif w == "carControl":
      enabled = bool(msg.carControl.enabled)
      accel = float(msg.carControl.actuators.accel)
      if enabled and accel < brake_threshold:
        brake_times.append(((t - t0) * 1e-9, accel))
    elif w == "radarState":
      lead = msg.radarState.leadOne
      lead_ids.append(((t - t0) * 1e-9, int(lead.radarTrackId) if lead.status and lead.radar else -1))
  if pending and pending_t is not None:
    census.feed(pending_t, (pending_t - t0) * 1e-9, pending)
  census.finish()
  attach_brakes(census.breaks, brake_times, lead_ids)

  duration_min = census.sweeps / 14.35 / 60.0 if census.sweeps else 0.0
  absences = [b["absentSweeps"] for b in census.breaks if b["reappeared"]]
  return {
    "segment": _segment_number(path),
    "sweeps": census.sweeps,
    "updateErrors": census.update_errors,
    "firstError": census.first_error,
    "fresh": census.fresh,
    "continuations": census.continuations,
    "breaks": census.breaks,
    "breaksPerMin": len(census.breaks) / duration_min if duration_min else None,
    "absenceSweeps": {str(k): absences.count(k) for k in sorted(set(absences))},
    "neverReappeared": sum(1 for b in census.breaks if not b["reappeared"] and not b["superseded"]),
    "chronicRuns": chronic_runs(census.breaks),
    "lateralStep": percentiles(census.lateral_steps),
    "candidates": census.candidates,
    "rejectionRuns": census.rejection_runs,
    "_lateral": census.lateral_steps,
  }


def pool(segments: list[dict]) -> dict:
  breaks = [dict(b, segment=s["segment"]) for s in segments for b in s["breaks"]]
  absences: dict[str, int] = {}
  for s in segments:
    for k, v in s["absenceSweeps"].items():
      absences[k] = absences.get(k, 0) + v
  return {
    "segments": len(segments),
    "sweeps": sum(s["sweeps"] for s in segments),
    "updateErrors": sum(s["updateErrors"] for s in segments),
    "continuations": sum(s["continuations"] for s in segments),
    "fresh": sum(s["fresh"] for s in segments),
    "breaks": len(breaks),
    "segmentsWithNoBreak": [s["segment"] for s in segments if not s["breaks"]],
    "absenceSweeps": dict(sorted(absences.items(), key=lambda kv: int(kv[0]))),
    "neverReappeared": sum(s["neverReappeared"] for s in segments),
    "breaksOnDeviceLead": [b for b in breaks if b["wasDeviceLead"]],
    "breaksNearHardBrake": [b for b in breaks if b["hardestBrakeNearby"] is not None],
    "chronicRuns": [dict(r, segment=s["segment"]) for s in segments for r in s["chronicRuns"]],
    "lateralStep": percentiles([v for s in segments for v in s["_lateral"]]),
    "candidates": [dict(c, segment=s["segment"]) for s in segments for c in s["candidates"]],
    "rejectionRuns": sorted((dict(r, segment=s["segment"]) for s in segments for r in s["rejectionRuns"]),
                            key=lambda r: -r["sweeps"]),
  }


def render(segments: list[dict], pooled: dict) -> str:
  lines = ["Bosch-A lifecycle census (offline replay through the real RadarInterface)", "=" * 78]
  header = ("seg", "sweeps", "cont", "fresh", "breaks", "/min", "absent", "cand", "rejRuns", "err")
  widths = (4, 7, 7, 6, 6, 6, 14, 5, 7, 4)
  lines.append(" ".join(f"{h:>{w}}" for h, w in zip(header, widths, strict=True)))
  for s in segments:
    bpm = f"{s['breaksPerMin']:.2f}" if s["breaksPerMin"] is not None else "-"
    row = (s["segment"], s["sweeps"], s["continuations"], s["fresh"], len(s["breaks"]), bpm,
           json.dumps(s["absenceSweeps"]), len(s["candidates"]), len(s["rejectionRuns"]), s["updateErrors"])
    lines.append(" ".join(f"{v!s:>{w}}" for v, w in zip(row, widths, strict=True)))
  p = pooled
  lines.append("-" * 78)
  lines.append(f"segments {p['segments']}, sweeps {p['sweeps']}, continuations {p['continuations']}, " +
               f"fresh {p['fresh']}, parser update errors {p['updateErrors']}")
  lines.append(f"lifecycle breaks: {p['breaks']}  (absence sweeps -> count: {p['absenceSweeps']}; " +
               f"never reappeared: {p['neverReappeared']})")
  lines.append(f"segments with no break (negative-control set): {p['segmentsWithNoBreak']}")
  lines.append(f"breaks on the device's leadOne: {len(p['breaksOnDeviceLead'])}; " +
               f"breaks within {BRAKE_WINDOW_S:.0f} s of a hard brake: {len(p['breaksNearHardBrake'])}")
  runs = p["chronicRuns"]
  single = [r for r in runs if r["breaks"] == 1]
  chronic = [r for r in runs if r["breaks"] > 1]
  lines.append(f"break runs: {len(single)} single (D-049 one-sweep case), {len(chronic)} chronic " +
               "(identity observed every sweep, never published)")
  for r in sorted(chronic, key=lambda r: -r["durationS"])[:15]:
    brake = f"{r['hardestBrakeNearby']:.2f}" if r["hardestBrakeNearby"] is not None else "none"
    lines.append(f"    CHRONIC seg {r['segment']} id={r['trackId']} t={r['tStart']:.2f}-{r['tEnd']:.2f} " +
                 f"({r['durationS']:.1f} s, {r['breaks']} breaks) lifecycle={r['lives']} " +
                 f"deviceLead={r['wasDeviceLead']} hardestBrakeNearby={brake}")
  for r in single[:10]:
    lines.append(f"    single  seg {r['segment']} id={r['trackId']} t={r['tStart']:.2f} deviceLead={r['wasDeviceLead']}")
  lines.append(f"per-sweep lateral step on unbroken measured continuations: {p['lateralStep']}")
  isolated = [c for c in p["candidates"] if c.get("isolated")]
  lines.append(f"lateral steps > {LATERAL_JUMP_M} m with no break: {len(p['candidates'])}; " +
               f"isolated (seamless-reuse candidates): {len(isolated)}; the rest are smooth multi-sweep runs")
  for c in p["candidates"][:25]:
    kind = "ISOLATED" if c.get("isolated") else "run"
    lines.append(f"    {kind:<8} seg {c['segment']} t={c['t']:.2f} id={c['trackId']} d={c['dRel']:.1f} " +
                 f"y {c['yPrev']:+.2f} -> {c['y']:+.2f}")
  lines.append(f"rejection runs >= {REJECTION_RUN_MIN_SWEEPS} sweeps: {len(p['rejectionRuns'])}")
  for r in p["rejectionRuns"][:10]:
    lines.append(f"    seg {r['segment']} t={r['tStart']:.2f} id={r['trackId']} sweeps={r['sweeps']} d={r['dRel']:.1f}")
  return "\n".join(lines)


def main() -> int:
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("path", help="route .zip, directory of segments, or a single rlog")
  ap.add_argument("--fingerprint", required=True, help="car fingerprint for parser replay, e.g. HONDA_CIVIC_BOSCH")
  ap.add_argument("--segments", help="segment filter, e.g. 0-5,9")
  ap.add_argument("--json", dest="json_out", help="write the full report as JSON")
  ap.add_argument("--lateral-jump", type=float, default=LATERAL_JUMP_M)
  args = ap.parse_args()

  from opendbc.car.honda.radar_interface import BOSCH_A_ALL_IDS
  bosch_ids = set(BOSCH_A_ALL_IDS)

  wanted = _parse_segments(args.segments)
  with tempfile.TemporaryDirectory() as tmp:
    inputs = collect_inputs(args.path, tmp)
    if wanted is not None:
      inputs = [(n, p) for n, p in inputs if n in wanted]
    if not inputs:
      raise SystemExit("No segments selected")
    segments = []
    for _n, p in inputs:
      try:
        segments.append(analyse_segment(p, args.fingerprint, bosch_ids, lateral_jump_m=args.lateral_jump))
      except SystemExit:
        raise
      except Exception as e:  # one unreadable segment must not lose the rest
        print(f"  !! {p}: {type(e).__name__}: {e}", file=sys.stderr)

  if not segments:
    raise SystemExit("No segment could be read")
  pooled = pool(segments)
  print(render(segments, pooled))
  if args.json_out:
    public = [{k: v for k, v in s.items() if not k.startswith("_")} for s in segments]
    with open(args.json_out, "w") as f:
      json.dump({"segments": public, "pooled": pooled}, f, indent=2, default=str)
    print(f"\nJSON written to {args.json_out}")
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
