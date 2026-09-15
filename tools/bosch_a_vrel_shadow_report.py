#!/usr/bin/env python3
"""Compare published radar lead velocity (U11) with shadow range-derived telemetry.

Why this exists
---------------
Decision D-044 introduced shadow range-derived velocity telemetry
(radarState.leadOne.vRelRangeDerived) alongside the radar's own published lead velocity
radarState.leadOne.vRel (U11) to measure lag, attenuation, and saturation on real routes.
This tool compares the two signals across real drives.

Device-side telemetry only; it replays nothing. Results are bound to the device commit
in initData and the route ID. This tool has NOT been validated against a real route by
its author.

Usage
-----
  python tools/bosch_a_vrel_shadow_report.py <route.zip|dir|rlog>
  python tools/bosch_a_vrel_shadow_report.py <path> --segments 0-4
  python tools/bosch_a_vrel_shadow_report.py <path> --json report.json
  python tools/bosch_a_vrel_shadow_report.py <path> --onset-threshold -2.0
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import tempfile

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

# Loads the capnp schemas from the physical cereal path before anything imports them through the
# `openpilot/tools` symlink (see the realpath note in bosch_a_route_report.py).
import cereal  # noqa: F401
from openpilot.tools.bosch_a_route_report import _parse_segments, _segment_number, collect_inputs
from openpilot.tools.lib.logreader import _LogFileReader

PERCENTILES = (5, 25, 50, 75, 95, 99)


def percentiles(values: list[float]) -> dict:
  """Linear-interpolated percentiles with n, min, max, mean, p5, p25, p50, p75, p95, p99.

  Returns {'n': 0} when empty.
  """
  if not values:
    return {"n": 0}
  ordered = sorted(values)
  n = len(ordered)
  out: dict = {
    "n": n,
    "min": ordered[0],
    "max": ordered[-1],
    "mean": sum(ordered) / n,
  }
  for p in PERCENTILES:
    if n == 1:
      out[f"p{p}"] = ordered[0]
      continue
    pos = (p / 100.0) * (n - 1)
    lo = math.floor(pos)
    hi = math.ceil(pos)
    out[f"p{p}"] = ordered[lo] if lo == hi else ordered[lo] + (ordered[hi] - ordered[lo]) * (pos - lo)
  return out


def pearson(xs: list[float], ys: list[float]) -> float | None:
  """Pearson correlation coefficient. Returns None when n < 3 or zero variance."""
  n = len(xs)
  if n != len(ys) or n < 3:
    return None
  mx = sum(xs) / n
  my = sum(ys) / n
  sxx = sum((x - mx) ** 2 for x in xs)
  syy = sum((y - my) ** 2 for y in ys)
  if sxx <= 0.0 or syy <= 0.0:
    return None
  sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys, strict=True))
  return sxy / math.sqrt(sxx * syy)


def diff_stats(samples: list[dict]) -> dict:
  """Summary statistics of vRel - vRelRangeDerived.

  samples are dicts with keys vRel, vRelRangeDerived, measuredRadar.
  Uses only samples whose vRelRangeDerived is finite.
  """
  finite_samples = [s for s in samples if math.isfinite(s["vRelRangeDerived"])]
  n = len(finite_samples)
  nan_count = len(samples) - n
  diffs = [s["vRel"] - s["vRelRangeDerived"] for s in finite_samples]
  abs_diffs = [abs(d) for d in diffs]
  frac_over_2 = (sum(1 for d in abs_diffs if d > 2.0) / n) if n > 0 else 0.0
  frac_over_5 = (sum(1 for d in abs_diffs if d > 5.0) / n) if n > 0 else 0.0
  r = pearson([s["vRel"] for s in finite_samples], [s["vRelRangeDerived"] for s in finite_samples])
  return {
    "n": n,
    "nanCount": nan_count,
    "diff": percentiles(diffs),
    "absDiff": percentiles(abs_diffs),
    "fracAbsOver2": frac_over_2,
    "fracAbsOver5": frac_over_5,
    "r": r,
  }


def rail_stats(samples: list[dict], rail: float = -13.49) -> dict:
  """Statistics for samples on the saturation rail (vRel <= rail).

  Positive understatement means the rail understated closing.
  """
  rail_samples = [s for s in samples if s["vRel"] <= rail and math.isfinite(s["vRelRangeDerived"])]
  diffs = [s["vRel"] - s["vRelRangeDerived"] for s in rail_samples]
  return {
    "n": len(rail_samples),
    "understatement": percentiles(diffs),
  }


ONSET_SUSTAIN_SAMPLES = 5

# A U11 onset and a range onset further apart than this are two different events inside one long
# track run, not one onset seen late: with the sustain rule alone, 00000232--fc8dad0d18 still paired
# crossings up to 26 s apart. STATUS.md's recorded U11 onset lag is 0.88-1.28 s; 3 s leaves margin.
ONSET_PAIR_MAX_S = 3.0


def _first_sustained_crossing(run: list[dict], key: str, threshold: float, sustain: int) -> float | None:
  """t of the first sample that starts `sustain` consecutive finite samples below `threshold`."""
  streak = 0
  for i, s in enumerate(run):
    v = s[key]
    if math.isfinite(v) and v < threshold:
      streak += 1
      if streak == sustain:
        return run[i - sustain + 1]["t"]
    else:
      streak = 0
  return None


def onset_lags(samples: list[dict], threshold: float, sustain: int = ONSET_SUSTAIN_SAMPLES) -> list[float]:
  """Braking onset lag (t_vrel - t_range) across contiguous lead tracks.

  Positive = U11 lags the range-derived signal. An onset is the first crossing that HOLDS for
  `sustain` consecutive samples: on 00000232--fc8dad0d18 a first-sample crossing reported lags up to
  30 s, because a single noisy range-LSQ sample dips below -2 m/s long before any real closing.
  """
  if not samples:
    return []

  runs: list[list[dict]] = []
  current_run: list[dict] = []
  for s in samples:
    if not current_run:
      current_run.append(s)
    else:
      prev = current_run[-1]
      dt = s["t"] - prev["t"]
      if s["trackId"] == prev["trackId"] and 0.0 <= dt <= 0.2:
        current_run.append(s)
      else:
        runs.append(current_run)
        current_run = [s]
  if current_run:
    runs.append(current_run)

  lags: list[float] = []
  for run in runs:
    first = run[0]
    if not (first["vRel"] > -1.0 and math.isfinite(first["vRelRangeDerived"]) and first["vRelRangeDerived"] > -1.0):
      continue

    t_vrel = _first_sustained_crossing(run, "vRel", threshold, sustain)
    t_range = _first_sustained_crossing(run, "vRelRangeDerived", threshold, sustain)
    if t_vrel is not None and t_range is not None and abs(t_vrel - t_range) <= ONSET_PAIR_MAX_S:
      lags.append(t_vrel - t_range)

  return lags


def analyse_segment(path: str, onset_threshold: float = -2.0) -> dict:
  """Extract and compare vRel and vRelRangeDerived for one segment rlog."""
  t0 = None
  latest_v_ego = 0.0
  samples: list[dict] = []

  lr = _LogFileReader(path)
  for msg in lr:
    which = msg.which()
    if which in ("initData", "sentinel"):
      # initData is stamped at logger start, ~600 s before a later segment's data; as the origin it
      # would put every sample at route time (same defect as fixed in bosch_a_corpus_report.py).
      continue
    if t0 is None:
      t0 = msg.logMonoTime

    if which == "carState":
      latest_v_ego = float(msg.carState.vEgo)
    elif which == "radarState":
      lead = msg.radarState.leadOne
      if not (lead.status and lead.radar):
        continue
      samples.append({
        "t": (msg.logMonoTime - t0) * 1e-9,
        "trackId": int(lead.radarTrackId),
        "dRel": float(lead.dRel),
        "vRel": float(lead.vRel),
        "vRelRangeDerived": float(lead.vRelRangeDerived),
        "measuredRadar": bool(lead.measuredRadar),
        "vEgo": latest_v_ego,
      })

  measured = [s for s in samples if s["measuredRadar"]]
  unmeasured = [s for s in samples if not s["measuredRadar"]]
  return {
    "segment": _segment_number(path),
    "radarLeadFrames": len(samples),
    "all": diff_stats(samples),
    "measured": diff_stats(measured),
    "unmeasured": diff_stats(unmeasured),
    "rail": rail_stats(samples),
    "onsetLags": onset_lags(samples, onset_threshold),
    "_samples": samples,
  }


def pool(segments: list[dict]) -> dict:
  """Pool statistics across multiple segments."""
  all_samples: list[dict] = []
  all_onset_lags: list[float] = []
  for s in segments:
    all_samples.extend(s.get("_samples", []))
    all_onset_lags.extend(s.get("onsetLags", []))

  measured = [s for s in all_samples if s["measuredRadar"]]
  unmeasured = [s for s in all_samples if not s["measuredRadar"]]
  return {
    "segments": len(segments),
    "radarLeadFrames": len(all_samples),
    "all": diff_stats(all_samples),
    "measured": diff_stats(measured),
    "unmeasured": diff_stats(unmeasured),
    "rail": rail_stats(all_samples),
    "onsetLag": percentiles(all_onset_lags),
  }


def render(segments: list[dict], pooled: dict, onset_threshold: float = -2.0) -> str:
  """Render a readable terminal text report."""
  out = [
    "Bosch-A vRel (U11) vs vRelRangeDerived (D-044) Report",
    "=" * 55,
    f"Segments processed: {pooled['segments']}  |  Total radar lead frames: {pooled['radarLeadFrames']}",
    "",
    "Difference statistics: vRel - vRelRangeDerived (m/s)",
    f"  {'subset':<12} {'n':>6} {'NaN':>5} {'mean':>7} {'p5':>7} {'p25':>7} {'p50':>7} {'p75':>7} {'p95':>7} {'p99':>7} {'|d|>2':>7} {'|d|>5':>7} {'r':>6}",
  ]
  for label, key in (("all", "all"), ("measured", "measured"), ("unmeasured", "unmeasured")):
    st = pooled[key]
    n = st["n"]
    nan_cnt = st["nanCount"]
    d = st["diff"]
    f2 = f"{st['fracAbsOver2'] * 100.0:5.1f}%" if n else "   n/a"
    f5 = f"{st['fracAbsOver5'] * 100.0:5.1f}%" if n else "   n/a"
    r_str = f"{st['r']:+.3f}" if st["r"] is not None else "   n/a"
    if n:
      out.append(
        f"  {label:<12} {n:>6} {nan_cnt:>5} {d['mean']:>+7.2f} " +
        f"{d['p5']:>+7.2f} {d['p25']:>+7.2f} {d['p50']:>+7.2f} {d['p75']:>+7.2f} {d['p95']:>+7.2f} {d['p99']:>+7.2f} " +
        f"{f2:>7} {f5:>7} {r_str:>6}"
      )
    else:
      out.append(f"  {label:<12} {n:>6} {nan_cnt:>5}    (no finite samples)")

  out.append("")
  out.append("Saturation rail (vRel <= -13.49 m/s)")
  rail = pooled["rail"]
  if rail["n"]:
    u = rail["understatement"]
    out.append(f"  Samples: {rail['n']}")
    out.append("  Understatement (vRel - vRelRangeDerived, pos = rail understated closing):")
    out.append(f"    mean={u['mean']:+.2f} m/s  p50={u['p50']:+.2f} m/s  p95={u['p95']:+.2f} m/s  max={u['max']:+.2f} m/s")
  else:
    out.append("  (no samples on saturation rail)")

  out.append("")
  out.append(f"Braking onset lag at threshold {onset_threshold:.1f} m/s (t_vrel - t_range, pos = U11 lags)")
  ol = pooled["onsetLag"]
  if ol["n"]:
    out.append(f"  Onset events: {ol['n']}")
    out.append(f"    mean={ol['mean']:+.3f} s  p50={ol['p50']:+.3f} s  p95={ol['p95']:+.3f} s  min={ol['min']:+.3f} s  max={ol['max']:+.3f} s")
  else:
    out.append("  (no onset events detected)")

  out.append("")
  out.append("Notice: Device-side telemetry only; it replays nothing.")
  out.append("Results are bound to the device commit in initData and the route ID.")
  out.append("NOT validated against a real route by its author.")
  return "\n".join(out)


def main() -> int:
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("path", help="route .zip, directory of segments, or a single rlog")
  ap.add_argument("--segments", help="segment filter, e.g. 0-5,9")
  ap.add_argument("--json", dest="json_out", help="write the full report as JSON")
  ap.add_argument("--onset-threshold", type=float, default=-2.0,
                  help="velocity threshold (m/s) to define braking onset (default -2.0)")
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
        segments.append(analyse_segment(p, onset_threshold=args.onset_threshold))
      except Exception as e:
        print(f"  !! {os.path.basename(p)}: {type(e).__name__}: {e}", file=sys.stderr)

  if not segments:
    raise SystemExit("No segment could be read")

  pooled = pool(segments)
  print(render(segments, pooled, onset_threshold=args.onset_threshold))

  if args.json_out:
    public = [{k: v for k, v in s.items() if not k.startswith("_")} for s in segments]
    with open(args.json_out, "w") as f:
      json.dump({"segments": public, "pooled": pooled}, f, indent=2, default=str)
    print(f"\nJSON written to {args.json_out}")
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
