#!/usr/bin/env python3
"""Pool device-side Bosch-A behaviour across the segments of a drive.

Why this exists
---------------
The 6- and 16-segment analyses recorded in STATUS.md were produced by code that was never
committed (`5edbd59`, `69e1683`, `bdf98de` touch only STATUS.md and DECISIONS.md). Their
numbers therefore cannot be re-derived, re-checked, or re-run on a new route, which is
exactly what AGENTS.md §3 is about. This script is the missing harness.

It is also the validation gate for two designs that are blocked on it:
D-048 item 2 (a replay that asserts no authority reduction on the benign segments) and
D-049 item 3 (a negative control over segments with no incarnation break).

What it is NOT
--------------
This is **not** the parser replay -- that is `tools/bosch_a_route_report.py`, which feeds
CAN back through the real `RadarInterface` and answers "what would the parser publish".
This one reads what the **device actually did**: `radarState` leads, `modelV2` vision leads,
the commanded accel, and the plan source. The two answer different questions and neither
replaces the other; segment collection and the fingerprint helper are imported from the
route report rather than copied, so they cannot drift (AGENTS.md §7).

Geometry conventions are imported from `radard` and asserted against
`radard.track_matches_vision` in the tests, because a sign error here would silently invert
the lateral residual -- `track_matches_vision` compares `yRel + lead.y[0]` (a SUM: vision `y`
is device-frame, radar `yRel` is car-frame left-positive), while range is
`dRel - (lead.x[0] - RADAR_TO_CAMERA)`.

Status of its output
--------------------
**NOT VALIDATED AGAINST A REAL ROUTE.** The statistics here have not been reproduced against
the figures in STATUS.md, because no route is reachable from an agent session (the network
policy denies `konik.ai`). What *is* tested: the geometry conventions, pinned against
`radard.track_matches_vision`'s own accept/reject boundary; the arithmetic, against
hand-computed inputs; and the extraction and attribution logic, against a synthetic segment
built from capnp Events whose every expected answer is computable by hand. What is untested is
the only thing that matters in the end -- whether those mechanics produce the right numbers on
a real drive, with real message rates, dropouts and clock skew. Until this tool has re-derived
the recorded figures on `00000231--5782493b00`, treat its output as a fresh measurement and say
so, and treat any disagreement with STATUS.md as unresolved rather than as a correction in
either direction.

Usage
-----
    python tools/bosch_a_corpus_report.py <route.zip|dir|rlog>
    python tools/bosch_a_corpus_report.py <path> --segments 0-5
    python tools/bosch_a_corpus_report.py <path> --json corpus.json
    python tools/bosch_a_corpus_report.py <path> --brake-threshold -1.5
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import tempfile
from collections import defaultdict

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from openpilot.selfdrive.controls.radard import RADAR_TO_CAMERA
from openpilot.tools.bosch_a_route_report import _parse_segments, _segment_number, collect_inputs
from openpilot.tools.lib.logreader import _LogFileReader

# A vision lead below this probability is not evidence of anything; STATUS.md's pooled gap
# distribution was quoted at prob >= 0.9 and this default matches it.
VISION_PROB_MIN = 0.9

# Commanded accel below this, while engaged, is a "hard brake" event. STATUS.md's tables use
# -1.5; it is a reporting threshold only and gates no behaviour.
HARD_BRAKE_ACCEL = -1.5

PERCENTILES = (5, 25, 50, 75, 90, 95, 99)

# The recorded occupancy figures (ed6257ef's "confident vision lead 94.2%") match a
# leadsV3[0].prob >= 0.5 cut, measured on 00000231--5782493b00/25 -- not the 0.9 cut the gap
# distribution uses, under which that segment reads 1.4%. Occupancy is reported at both.
VISION_OCCUPANCY_PROB_MIN = 0.5

# U11-vs-range-rate pairs are built only inside contiguous runs of MEASURED samples. Nominal sweep
# period is ~70 ms, so one missing sweep -- the D-049 incarnation-break signature -- already ends a
# run. Without the split, a centred derivative spans dropouts and ID reuses: on
# 00000231--5782493b00 that manufactured range rates up to 55.8 m/s and 11.2% gross disagreement.
U11_RUN_GAP_S = 0.1

# A driver override belongs to a hard-brake run if a pedal is pressed from the run's start until
# this long after it ends. On 00000231--5782493b00/10 the gas press came after the commanded peak,
# so checking the peak frame alone reported the flagged false brake as "no override".
OVERRIDE_WINDOW_S = 1.0

# experimentalMode switches separated by less than this are reported individually. STATUS.md recorded
# 80 ms and 110 ms dwells on one segment; nothing ties them to a control fault, so this is a
# reporting threshold for a characterisation, not a gate.
EXPERIMENTAL_SHORT_DWELL_S = 0.5


# --- pure geometry and statistics -----------------------------------------------------------------
# Everything below is deliberately free of capnp and file I/O so it can be tested without a route.

def range_gap_m(radar_d_rel: float, vision_x: float) -> float:
  """Radar-versus-vision range disagreement, POSITIVE when vision reads longer.

  Vision `x` is measured from the camera, the radar's dRel from the radar, so the camera
  distance is moved back by RADAR_TO_CAMERA before differencing -- the same correction
  `track_matches_vision` applies. STATUS.md's pooled median of +4.0 m is in this sign: a
  monocular model reads systematically longer than the radar.
  """
  return (vision_x - RADAR_TO_CAMERA) - radar_d_rel


def lateral_residual_m(radar_y_rel: float, vision_y: float) -> float:
  """Signed lateral disagreement, in `track_matches_vision`'s convention.

  Note the SUM. Radar `yRel` is car-frame (left positive, per the sign fixed in
  radar_interface against real captures 2026-08-22) and model `y` is device-frame, so the
  matcher adds them. Differencing here would report ~2x the true residual on any off-centre
  track and roughly zero on a genuinely mismatched one.
  """
  return radar_y_rel + vision_y


def headway_s(d_rel: float, v_ego: float) -> float | None:
  """Time to reach the lead's current position at the current speed. None below 0.1 m/s."""
  if v_ego < 0.1:
    return None
  return d_rel / v_ego


def ttc_s(d_rel: float, v_rel: float) -> float | None:
  """Time to contact at the current closing rate. None when not closing."""
  if v_rel >= -0.1:
    return None
  return d_rel / -v_rel


def percentiles(values: list[float], ps: tuple[int, ...] = PERCENTILES) -> dict[str, float]:
  """Linear-interpolated percentiles. Empty input gives an empty dict, never a zero."""
  if not values:
    return {}
  ordered = sorted(values)
  out = {}
  for p in ps:
    if len(ordered) == 1:
      out[f"p{p}"] = ordered[0]
      continue
    pos = (p / 100.0) * (len(ordered) - 1)
    lo = math.floor(pos)
    hi = math.ceil(pos)
    out[f"p{p}"] = ordered[lo] if lo == hi else ordered[lo] + (ordered[hi] - ordered[lo]) * (pos - lo)
  return out


def pearson(xs: list[float], ys: list[float]) -> float | None:
  """Pearson correlation, or None when undefined (n < 2 or a constant series).

  Returning None rather than 0.0 matters: a constant series is "no information", and
  reporting it as zero correlation would read as evidence of independence.
  """
  n = len(xs)
  if n != len(ys) or n < 2:
    return None
  mx, my = sum(xs) / n, sum(ys) / n
  sxx = sum((x - mx) ** 2 for x in xs)
  syy = sum((y - my) ** 2 for y in ys)
  if sxx <= 0.0 or syy <= 0.0:
    return None
  sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys, strict=True))
  return sxy / math.sqrt(sxx * syy)


def centred_derivative(times: list[float], values: list[float]) -> list[tuple[float, float]]:
  """Centred finite difference -> [(t, dvalue/dt)] for interior samples.

  Centred rather than one-sided so the derivative is not shifted half a sample against the
  series it is compared with -- the lag measurement in `cross_correlation_lag` is the whole
  point of computing this, and a half-sample bias would show up there as real lag.
  """
  out = []
  for i in range(1, len(times) - 1):
    dt = times[i + 1] - times[i - 1]
    if dt > 1e-9:
      out.append((times[i], (values[i + 1] - values[i - 1]) / dt))
  return out


def contiguous_runs(samples: list[tuple], max_gap_s: float) -> list[list[tuple]]:
  """Split time-ordered (t, ...) samples wherever consecutive times differ by more than max_gap_s."""
  runs: list[list[tuple]] = []
  current: list[tuple] = []
  for s in samples:
    if current and s[0] - current[-1][0] > max_gap_s:
      runs.append(current)
      current = []
    current.append(s)
  if current:
    runs.append(current)
  return runs


def u11_range_rate_pairs(run: list[tuple[float, float, float]]) -> tuple[list[float], list[float]]:
  """(U11, centred range rate) pairs for one contiguous run of (t, dRel, vRel) samples.

  Paired by index in one loop. The previous pairing enumerated `centred_derivative`'s output against
  `vs[i + 1]`, which shifts every later pair by one sample whenever a zero-dt sample is skipped.
  """
  u11: list[float] = []
  rates: list[float] = []
  for i in range(1, len(run) - 1):
    dt = run[i + 1][0] - run[i - 1][0]
    if dt > 1e-9:
      u11.append(run[i][2])
      rates.append((run[i + 1][1] - run[i - 1][1]) / dt)
  return u11, rates


def attach_override(runs: list[dict], pressed_times: list[float], window_s: float = OVERRIDE_WINDOW_S) -> list[dict]:
  """Mark each brake run with whether a pedal was pressed from its start to `window_s` after its end."""
  for row in runs:
    row["overrideInRun"] = any(row["tStart"] <= t <= row["tEnd"] + window_s for t in pressed_times)
  return runs


def cross_correlation_lag(a: list[float], b: list[float], max_lag: int) -> tuple[int, float] | None:
  """Lag (in samples) maximising |corr(a shifted by lag, b)|, and that correlation.

  A NEGATIVE lag means `a` lags `b`. STATUS.md records U11 peaking at lag -4 (-0.28 s)
  against the range derivative, i.e. the velocity channel trails the geometry.
  """
  best = None
  for lag in range(-max_lag, max_lag + 1):
    if lag < 0:
      xs, ys = a[-lag:], b[:len(b) + lag]
    elif lag > 0:
      xs, ys = a[:len(a) - lag], b[lag:]
    else:
      xs, ys = a, b
    n = min(len(xs), len(ys))
    if n < 3:
      continue
    r = pearson(xs[:n], ys[:n])
    if r is not None and (best is None or abs(r) > abs(best[1])):
      best = (lag, r)
  return best


def summarise(values: list[float]) -> dict:
  """n / min / max / mean plus percentiles, the shape every table in STATUS.md wants."""
  if not values:
    return {"n": 0}
  out = {"n": len(values), "min": min(values), "max": max(values), "mean": sum(values) / len(values)}
  out.update(percentiles(values))
  return out


# --- log extraction --------------------------------------------------------------------------------

def segment_label(path: str) -> str:
  """A readable segment name. The file is always called `rlog`, so the name is its directory.

  STATUS.md labels segments by the hash in the directory name (`3a4e0842`, `9e21cac9`), and a
  table of rows all labelled "rlog" cannot be cross-referenced with anything.
  """
  base = os.path.basename(path.rstrip("/"))
  if base.startswith(("rlog", "qlog")):
    parent = os.path.basename(os.path.dirname(path.rstrip("/")))
    if parent:
      return parent
  return base


def collapse_brake_runs(frames: list[dict]) -> list[dict]:
  """Group contiguous over-threshold frames into runs, one row per run at its peak.

  A 1.5 s brake at the radarState rate is ~30 frames. STATUS.md counts RUNS -- "only four runs
  with commanded accel < -1.5 m/s^2 while engaged" -- so emitting a row per frame would both
  bury the table and inflate the event count by an order of magnitude, which is exactly the
  kind of miscount that makes a threshold look supported.
  """
  runs = []
  current: list[dict] = []
  for f in frames:
    if current and f["_frame"] != current[-1]["_frame"] + 1:
      runs.append(current)
      current = []
    current.append(f)
  if current:
    runs.append(current)

  out = []
  for run in runs:
    peak = min(run, key=lambda f: f["accel"])
    row = {k: v for k, v in peak.items() if not k.startswith("_")}
    row["tStart"] = run[0]["t"]
    row["tEnd"] = run[-1]["t"]
    row["runFrames"] = len(run)
    row["runDuration"] = run[-1]["t"] - run[0]["t"]
    out.append(row)
  return out


def _lead_record(lead) -> dict | None:
  """The fields of a radarState lead this report reads, or None when there is no lead."""
  if not lead.status:
    return None
  return {
    "dRel": float(lead.dRel), "yRel": float(lead.yRel), "vRel": float(lead.vRel),
    "vLeadK": float(lead.vLeadK), "aLeadK": float(lead.aLeadK),
    "radar": bool(lead.radar), "radarTrackId": int(lead.radarTrackId),
    "vRelRangeDerived": float(lead.vRelRangeDerived),
  }


def _vision_lead(model) -> dict | None:
  """leadsV3[0] reduced to the three numbers this report uses."""
  leads = model.leadsV3
  if len(leads) < 1 or not len(leads[0].x):
    return None
  return {"prob": float(leads[0].prob), "x": float(leads[0].x[0]), "y": float(leads[0].y[0])}


def analyse_segment(path: str, *, brake_threshold: float = HARD_BRAKE_ACCEL,
                    vision_prob_min: float = VISION_PROB_MIN) -> dict:
  """Walk one segment's rlog and pull out everything the corpus tables need.

  State is carried forward between messages (each message type arrives on its own clock), so
  every derived quantity is paired against the most recent value of the others rather than
  assuming synchronised arrival.
  """
  t0 = None
  latest = {"model": None, "vEgo": 0.0, "aEgo": 0.0, "gasPressed": False, "brakePressed": False,
            "enabled": False, "experimental": None, "planSource": None, "accel": None}

  frames = 0
  radar_lead_frames = 0
  vision_lead_frames = 0
  vision_lead_frames_50 = 0
  pressed_times: list[float] = []
  experimental_frames = 0
  experimental_seen = 0
  experimental_switch_times: list[float] = []

  gaps: list[float] = []
  gap_series: list[tuple[float, float]] = []   # (t, gap) while a radar lead is selected
  brake_events: list[dict] = []
  track_samples: dict[int, list[tuple[float, float, float]]] = defaultdict(list)  # id -> (t, dRel, vRel)
  lateral_pairs: list[tuple[float, float]] = []   # (range residual, lateral residual), nearest-in-range
  live_track_frames = 0
  live_track_count = 0

  lr = _LogFileReader(path)
  for msg in lr:
    which = msg.which()
    if which in ("initData", "sentinel"):
      # initData is stamped at logger start -- ~601.5 s before the first sample of segment 10 of
      # 00000231--5782493b00 -- so as the clock origin it put every event at route time, not
      # segment time (the flagged brake read t=612.76 instead of ~11.3).
      continue
    t = msg.logMonoTime * 1e-9
    if t0 is None:
      t0 = t
    rel = t - t0

    if which == "carState":
      latest["vEgo"] = float(msg.carState.vEgo)
      latest["aEgo"] = float(msg.carState.aEgo)
      latest["gasPressed"] = bool(msg.carState.gasPressed)
      latest["brakePressed"] = bool(msg.carState.brakePressed)
      if latest["gasPressed"] or latest["brakePressed"]:
        pressed_times.append(rel)
    elif which == "modelV2":
      latest["model"] = _vision_lead(msg.modelV2)
    elif which == "selfdriveState":
      experimental = bool(msg.selfdriveState.experimentalMode)
      if latest["experimental"] is not None and experimental != latest["experimental"]:
        experimental_switch_times.append(rel)
      latest["experimental"] = experimental
      latest["enabled"] = bool(msg.selfdriveState.enabled)
    elif which == "longitudinalPlan":
      latest["planSource"] = str(msg.longitudinalPlan.longitudinalPlanSource)
    elif which == "carControl":
      latest["accel"] = float(msg.carControl.actuators.accel)
      latest["enabled"] = bool(msg.carControl.enabled)
    elif which == "liveTracks":
      pts = msg.liveTracks.points
      live_track_frames += 1
      live_track_count += len(pts)
      for p in pts:
        # Measured only: a coasted point carries a held vRel, not U11 (the recorded figure was
        # "9,390 measured track samples").
        if p.measured:
          track_samples[int(p.trackId)].append((rel, float(p.dRel), float(p.vRel)))
      # The inverse failure mode (D-048): tracks that agree with vision in RANGE but are
      # rejected on LATERAL. Take the nearest-in-range track to the vision lead, as the
      # 16-segment analysis did, and record both residuals for it.
      vis = latest["model"]
      if vis is not None and vis["prob"] >= vision_prob_min and len(pts):
        target = vis["x"] - RADAR_TO_CAMERA
        nearest = min(pts, key=lambda p: abs(float(p.dRel) - target))
        lateral_pairs.append((float(nearest.dRel) - target,
                              abs(lateral_residual_m(float(nearest.yRel), vis["y"]))))
    elif which == "radarState":
      frames += 1
      if latest["experimental"] is not None:
        experimental_seen += 1
        experimental_frames += int(latest["experimental"])
      lead = _lead_record(msg.radarState.leadOne)
      vis = latest["model"]
      if vis is not None and vis["prob"] >= vision_prob_min:
        vision_lead_frames += 1
      if vis is not None and vis["prob"] >= VISION_OCCUPANCY_PROB_MIN:
        vision_lead_frames_50 += 1
      if lead is None:
        continue
      if not lead["radar"]:
        continue
      radar_lead_frames += 1
      if vis is not None and vis["prob"] >= vision_prob_min:
        gap = range_gap_m(lead["dRel"], vis["x"])
        gaps.append(gap)
        gap_series.append((rel, gap))
      # A hard brake is attributed to the lead that was selected when it was commanded.
      if latest["accel"] is not None and latest["accel"] < brake_threshold and latest["enabled"]:
        brake_events.append({
          "_frame": frames, "t": rel, "accel": latest["accel"], "dRel": lead["dRel"], "vRel": lead["vRel"],
          "vEgo": latest["vEgo"], "aEgo": latest["aEgo"],
          "headway": headway_s(lead["dRel"], latest["vEgo"]), "ttc": ttc_s(lead["dRel"], lead["vRel"]),
          "gap": range_gap_m(lead["dRel"], vis["x"]) if vis else None,
          "visionProb": vis["prob"] if vis else None,
          "source": latest["planSource"], "override": latest["gasPressed"] or latest["brakePressed"],
          "radarTrackId": lead["radarTrackId"], "experimental": latest["experimental"],
        })

  # d(gap)/dt -- refuted as a gate by the corpus (p95 31 m/s, max 130 m/s), so it is reported to
  # keep that refutation reproducible rather than because anything should consume it.
  gap_rates = [abs(r) for _t, r in centred_derivative([t for t, _ in gap_series], [g for _, g in gap_series])]

  # U11 versus a centred range derivative, per track, pooled. Establishes that U11 is an
  # independent channel rather than a differentiated range (STATUS.md: r=0.807 at zero lag,
  # peaking at lag -4). Tracks are kept separate so one track's death cannot create a step in
  # the other's derivative.
  u11: list[float] = []
  drange: list[float] = []
  for samples in track_samples.values():
    for run in contiguous_runs(samples, U11_RUN_GAP_S):
      run_u11, run_rates = u11_range_rate_pairs(run)
      u11.extend(run_u11)
      drange.extend(run_rates)

  return {
    "segment": _segment_number(path),
    "path": segment_label(path),
    "frames": frames,
    "radarLeadPct": 100.0 * radar_lead_frames / frames if frames else 0.0,
    "visionLeadPct": 100.0 * vision_lead_frames / frames if frames else 0.0,
    "visionLeadPct50": 100.0 * vision_lead_frames_50 / frames if frames else 0.0,
    "experimentalPct": 100.0 * experimental_frames / experimental_seen if experimental_seen else None,
    "experimentalSwitches": len(experimental_switch_times),
    "experimentalShortDwells": [
      {"t": b, "dwell": b - a} for a, b in zip(experimental_switch_times, experimental_switch_times[1:], strict=False)
      if b - a < EXPERIMENTAL_SHORT_DWELL_S
    ],
    "tracksPerFrame": live_track_count / live_track_frames if live_track_frames else 0.0,
    "gap": summarise(gaps),
    "gapRate": summarise(gap_rates),
    "lateralResidual": summarise([lat for _r, lat in lateral_pairs]),
    "rangeResidual": summarise([r for r, _lat in lateral_pairs]),
    "hardBrakes": attach_override(collapse_brake_runs(brake_events), pressed_times),
    "_u11": u11,
    "_drange": drange,
    "_gaps": gaps,
    "_gapRates": gap_rates,
    "_lateral": [lat for _r, lat in lateral_pairs],
    "_range": [r for r, _lat in lateral_pairs],
  }


def pool(segments: list[dict]) -> dict:
  """Pooled distributions and the cross-segment correlations."""
  cat = lambda key: [v for s in segments for v in s[key]]  # noqa: E731
  u11, drange = cat("_u11"), cat("_drange")

  disagreement = [abs(a - b) for a, b in zip(u11, drange, strict=True)]
  gross = sum(1 for d in disagreement if d > 5.0)

  exp_pairs = [(s["experimentalPct"], s["radarLeadPct"]) for s in segments if s["experimentalPct"] is not None]

  return {
    "segments": len(segments),
    "gap": summarise(cat("_gaps")),
    "gapRate": summarise(cat("_gapRates")),
    "lateralResidual": summarise(cat("_lateral")),
    "rangeResidual": summarise(cat("_range")),
    "u11VsRangeRate": {
      "n": len(u11),
      "rZeroLag": pearson(u11, drange),
      "peak": cross_correlation_lag(u11, drange, max_lag=10),
      "absDisagreement": summarise(disagreement),
      "grossDisagreePct": 100.0 * gross / len(disagreement) if disagreement else 0.0,
    },
    "corrExperimentalRadarLead": pearson([e for e, _ in exp_pairs], [r for _, r in exp_pairs]),
    "corrVisionRadarLead": pearson([s["visionLeadPct"] for s in segments], [s["radarLeadPct"] for s in segments]),
    "corrVisionRadarLead50": pearson([s["visionLeadPct50"] for s in segments], [s["radarLeadPct"] for s in segments]),
    "hardBrakes": sorted((b for s in segments for b in [dict(e, segment=s["path"]) for e in s["hardBrakes"]]),
                         key=lambda b: b["accel"]),
  }


# --- reporting -------------------------------------------------------------------------------------

def _fmt_dist(label: str, d: dict) -> str:
  if not d.get("n"):
    return f"  {label:<18} (no samples)"
  cells = "  ".join(f"{d[f'p{p}']:+7.1f}" for p in PERCENTILES if f"p{p}" in d)
  return f"  {label:<18} n={d['n']:<6d} {cells}   max {d['max']:+7.1f}"


def render(segments: list[dict], pooled: dict, *, brake_threshold: float) -> str:
  out = []
  out.append("=" * 100)
  out.append("Bosch-A corpus report -- device-side behaviour pooled across segments")
  out.append("=" * 100)
  out.append("")
  out.append("UNVALIDATED: this harness has not yet reproduced the figures recorded in STATUS.md on")
  out.append("a known route. Read its numbers as a fresh measurement, not as a confirmation.")
  out.append("")

  out.append("Per segment")
  out.append(f"  {'segment':<28} {'frames':>7} {'radarLead%':>11} {'visLead%':>9} {'vis50%':>7} {'exp%':>7} {'trk/frame':>10} {'gap p50':>9}")
  for s in segments:
    exp = f"{s['experimentalPct']:.1f}" if s["experimentalPct"] is not None else "n/a"
    gap = f"{s['gap'].get('p50', float('nan')):+.1f}" if s["gap"].get("n") else "n/a"
    row = f"  {s['path'][:28]:<28} {s['frames']:>7} {s['radarLeadPct']:>10.1f}% {s['visionLeadPct']:>8.1f}%"
    row += f" {s['visionLeadPct50']:>6.1f}%"
    out.append(f"{row} {exp:>7} {s['tracksPerFrame']:>10.2f} {gap:>9}")
  out.append("")

  hdr = "  ".join(f"p{p:<6}" for p in PERCENTILES)
  out.append(f"Pooled distributions ({pooled['segments']} segments)")
  out.append(f"  {'':<18} {'':<9} {hdr}")
  out.append(_fmt_dist("radar/vision gap", pooled["gap"]))
  out.append(_fmt_dist("|d(gap)/dt|", pooled["gapRate"]))
  out.append(_fmt_dist("range residual", pooled["rangeResidual"]))
  out.append(_fmt_dist("lateral residual", pooled["lateralResidual"]))
  out.append("")

  u = pooled["u11VsRangeRate"]
  out.append("U11 versus a centred range derivative")
  if u["n"]:
    peak = u["peak"]
    out.append(f"  n={u['n']}  r(zero lag)={u['rZeroLag']:.3f}" if u["rZeroLag"] is not None else f"  n={u['n']}")
    if peak:
      trend = 'U11 lags the range' if peak[0] < 0 else 'U11 leads the range' if peak[0] > 0 else 'no lag'
      out.append(f"  peak |r|={peak[1]:.3f} at lag {peak[0]:+d} samples  ({trend})")
    dis = u['absDisagreement']
    out.append(f"  |U11 - range rate|: p50 {dis.get('p50', 0):.2f}  p90 {dis.get('p90', 0):.2f}  max {dis.get('max', 0):.2f} m/s")
    out.append(f"  disagreeing by >5 m/s: {u['grossDisagreePct']:.2f}% of samples")
  else:
    out.append("  (no track samples)")
  out.append("")

  out.append("Cross-segment correlations")
  for label, key in (("corr(experimental%, radarLead%)", "corrExperimentalRadarLead"),
                     ("corr(visionLead%, radarLead%)", "corrVisionRadarLead"),
                     ("corr(visionLead%@0.5, radarLead%)", "corrVisionRadarLead50")):
    v = pooled[key]
    out.append(f"  {label:<34} {v:+.3f}" if v is not None else f"  {label:<34} undefined")
  out.append("")

  out.append(f"Hard-brake events (commanded accel < {brake_threshold} m/s^2 while engaged)")
  if pooled["hardBrakes"]:
    out.append(f"  {'segment':<24} {'t':>7} {'accel':>7} {'dRel':>7} {'vRel':>7} {'headway':>8} {'TTC':>7} {'gap':>7} {'vprob':>6} {'source':>8} {'ovr':>4}")
    for b in pooled["hardBrakes"]:
      f2 = lambda v, w=7, p=2: f"{v:>{w}.{p}f}" if v is not None else f"{'n/a':>{w}}"  # noqa: E731
      ovr = 'yes' if b.get('overrideInRun', b['override']) else 'no'
      head = f"  {b['segment'][:24]:<24} {b['t']:>7.2f} {b['accel']:>7.2f} {b['dRel']:>7.1f} {b['vRel']:>7.2f}"
      mid = f"{f2(b['headway'], 8)} {f2(b['ttc'])} {f2(b['gap'])} {f2(b['visionProb'], 6)}"
      out.append(f"{head} {mid} {str(b['source'])[:8]:>8} {ovr:>4}")
  else:
    out.append("  (none)")
  out.append("")
  out.append(f"experimentalMode switching (dwell < {EXPERIMENTAL_SHORT_DWELL_S} s listed)")
  switching = [s for s in segments if s.get("experimentalSwitches")]
  if switching:
    for s in switching:
      short = ", ".join(f"t={d['t']:.2f} ({d['dwell'] * 1000:.0f} ms)" for d in s["experimentalShortDwells"]) or "none short"
      out.append(f"  {s['path'][:24]:<24} {s['experimentalSwitches']:>3} switches; {short}")
  else:
    out.append("  (no switches)")
  out.append("")
  out.append("Reminder: offline replay of a recorded drive. It says nothing about how the car behaves")
  out.append("when this radar drives the planner (AGENTS.md §2).")
  return "\n".join(out)


def main() -> int:
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("path", help="route .zip, directory of segments, or a single rlog")
  ap.add_argument("--segments", help="segment filter, e.g. 0-5,9")
  ap.add_argument("--json", dest="json_out", help="write the full report as JSON")
  ap.add_argument("--brake-threshold", type=float, default=HARD_BRAKE_ACCEL,
                  help=f"commanded accel below this counts as a hard brake (default {HARD_BRAKE_ACCEL})")
  ap.add_argument("--vision-prob", type=float, default=VISION_PROB_MIN,
                  help=f"minimum vision lead probability (default {VISION_PROB_MIN})")
  args = ap.parse_args()

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
        segments.append(analyse_segment(p, brake_threshold=args.brake_threshold,
                                        vision_prob_min=args.vision_prob))
      except Exception as e:  # one unreadable segment must not lose the rest of the corpus
        print(f"  !! {os.path.basename(p)}: {type(e).__name__}: {e}", file=sys.stderr)

  if not segments:
    raise SystemExit("No segment could be read")

  pooled = pool(segments)
  print(render(segments, pooled, brake_threshold=args.brake_threshold))

  if args.json_out:
    public = [{k: v for k, v in s.items() if not k.startswith("_")} for s in segments]
    with open(args.json_out, "w") as f:
      json.dump({"segments": public, "pooled": pooled}, f, indent=2, default=str)
    print(f"\nJSON written to {args.json_out}")
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
