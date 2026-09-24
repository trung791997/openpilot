#!/usr/bin/env python3
"""Off-device Lateral Tune trial: latest N Konik/route dirs -> proposed LatGainSchedule.

Usage (inside the oprad-test container, routes volume mounted at /routes):
  python tools/lateral/lat_tune_cli.py --routes-root /routes/konik --latest 8 [--json trial.json] [--current-schedule '<json>']

Segment dirs are `<route>--<seg>` or `<route>/<seg>` (the tools/konik_fetch.py layout), optionally under a
`<dongle>/` folder, holding rlog.zst/rlog.bz2/rlog. `<route>` is a date name or a counter name (`0000026b--92b1979afa`).
Prints a per-knot table and a final `LatGainSchedule = ...` line to paste into Galaxy (Toggles -> LatGainSchedule).
Unit-test/replay evidence only; nothing here is driven.
"""
import argparse
import json
import re
import sys
from pathlib import Path

from openpilot.selfdrive.controls.lib import lat_tune_analyzer as lat

MAX_ROUTES = 8
ROUTE_PAT = r"(?:\d{4}-\d{2}-\d{2}--\d{2}-\d{2}-\d{2}|[0-9a-f]{8}--[0-9a-f]{10})"   # date or counter route name
SEG_RE = re.compile(rf"^(?P<route>{ROUTE_PAT})--(?P<seg>\d+)$")                   # <route>--<seg>/rlog
ROUTE_RE = re.compile(rf"^{ROUTE_PAT}$")                                            # <route>/<seg>/rlog (tools/konik_fetch.py)
LOG_NAMES = ("rlog.zst", "rlog.bz2", "rlog")


def _log_in(seg_dir):
  for name in LOG_NAMES:
    p = seg_dir / name
    if p.is_file():
      return p
  return None


def discover_routes(root, latest=MAX_ROUTES):
  """[(route_name, [log paths sorted by segment])], oldest first, keeping only the `latest` newest routes."""
  root = Path(root)
  groups = {}
  for seg_dir in root.glob("**/"):
    m = SEG_RE.match(seg_dir.name)
    if m:
      route, seg, parent = m["route"], m["seg"], seg_dir.parent
    elif seg_dir.name.isdigit() and ROUTE_RE.match(seg_dir.parent.name):
      route, seg, parent = seg_dir.parent.name, seg_dir.name, seg_dir.parent.parent
    else:
      continue
    log = _log_in(seg_dir)
    if log is None:
      continue
    dongle = parent.name if parent != root else ""
    name = f"{dongle}|{route}" if dongle else route
    groups.setdefault(name, []).append((int(seg), log))
  ordered = sorted(groups, key=lambda n: n.split("|")[-1])          # route time string sorts chronologically
  keep = ordered[-latest:] if latest else ordered
  return [(n, [p for _, p in sorted(groups[n])]) for n in keep]


def _table(trial):
  lines = [f"{'knot':>6} {'min':>6} {'ready':>5} {'sign/s':>7} {'curve':>6} {'ovr/min':>7} {'factor':>6} {'P%':>6}  decision"]
  for k, p in zip(trial["knots"], trial["proposedPPct"], strict=True):
    f = lambda x, w: f"{x:{w}.2f}" if isinstance(x, (int, float)) else f"{'-':>{w}}"
    lines.append(f"{k['mph']:>4.0f}mph {k['minutes']:>6.1f} {'yes' if k['ready'] else 'no':>5} {f(k['signRate'], 7)} "
                 f"{f(k['curveRatio'], 6)} {f(k['pressRate'], 7)} {k['factor']:>6.2f} {p:>6.1f}  {k['reason']}")
  return "\n".join(lines)


def main(argv=None):
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("--routes-root", required=True, help="directory holding <route>--<seg> dirs (Konik layout ok)")
  ap.add_argument("--latest", type=int, default=MAX_ROUTES, help=f"newest N routes, 1..{MAX_ROUTES}")
  ap.add_argument("--json", help="write the trial JSON here")
  ap.add_argument("--current-schedule", default=None, help="LatGainSchedule currently on the device, to carry i/f")
  args = ap.parse_args(argv)
  if not 1 <= args.latest <= MAX_ROUTES:
    print(f"--latest must be at most {MAX_ROUTES}", file=sys.stderr)
    return 2
  routes = discover_routes(args.routes_root, latest=args.latest)
  if not routes:
    print(f"no route segments with rlogs under {args.routes_root}", file=sys.stderr)
    return 1
  sources = [lat.RouteLog(name, str(i), str(p)) for name, logs in routes for i, p in enumerate(logs)]
  print(f"analyzing {len(routes)} route(s), {len(sources)} segment(s): " + ", ".join(n for n, _ in routes))
  trial = lat.analyze_sources(sources, on_progress=lambda i, n, s: print(f"  [{i + 1}/{n}] {s.route}--{s.segment}", flush=True))
  print(_table(trial))
  for w in trial["warnings"]:
    print(f"warning: {w}")
  if args.json:
    Path(args.json).write_text(json.dumps(trial, indent=2, sort_keys=True))
  print(f"baseline P% (from the newest route's logs): {trial['baseline']['pPct']}")
  print(f"LatGainSchedule = {lat.build_schedule(trial, args.current_schedule)}")
  return 0


if __name__ == "__main__":
  sys.exit(main())
