#!/usr/bin/env python3
"""Off-device Lateral Tune trial: latest N Konik/route dirs -> proposed NRDR PID band P scales.

Usage (inside the oprad-test container, routes volume mounted at /routes):
  python tools/lateral/lat_tune_cli.py --routes-root /routes/konik --latest 8 [--json trial.json]

Segment dirs are `<route>--<seg>` or `<route>/<seg>` (the tools/konik_fetch.py layout), optionally under a
`<dongle>/` folder, holding rlog.zst/rlog.bz2/rlog. `<route>` is a date name or a counter name (`0000026b--92b1979afa`).
Prints a per-band table (LowSpeed < 25 mph, Standard 25-50, Highway 50+, as LatControlPID bins them) and
`LatPScaleLowSpeed = N` lines to set in Galaxy. Only P is proposed; I and F are shown and left alone.
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


def _band_range(b):
  return f"{b['lowMph']:g}-{b['highMph']:g}" if b.get("highMph") is not None else f"{b['lowMph']:g}+"


def _table(trial):
  lines = [f"{'band':<9} {'mph':>6} {'min':>6} {'ready':>5} {'sign/s':>7} {'curve':>6} {'entry':>6} {'steady':>6} {'exit':>6} {'ovr/min':>7} {'factor':>6} "
           f"{'P/I/F now':>12} {'P new':>5}  decision"]
  for b in trial["bands"]:
    f = lambda x, w: f"{x:{w}.2f}" if isinstance(x, (int, float)) else f"{'-':>{w}}"
    cur = b["current"]
    pif = f"{cur['p']}/{cur['i']}/{cur['f']}"
    lines.append(f"{b['name']:<9} {_band_range(b):>6} {b['minutes']:>6.1f} {'yes' if b['ready'] else 'no':>5} "
                 f"{f(b['signRate'], 7)} {f(b['curveRatio'], 6)} "
                 f"{f(b.get('curveRatioEntry'), 6)} {f(b.get('curveRatioSteady'), 6)} {f(b.get('curveRatioExit'), 6)} "
                 f"{f(b['pressRate'], 7)} {b['factor']:>6.2f} "
                 f"{pif:>12} {b['proposed']['p']:>5}  {b['reason']}")
  return "\n".join(lines)


def main(argv=None):
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("--routes-root", required=True, help="directory holding <route>--<seg> dirs (Konik layout ok)")
  ap.add_argument("--latest", type=int, default=MAX_ROUTES, help=f"newest N routes, 1..{MAX_ROUTES}")
  ap.add_argument("--json", help="write the trial JSON here")
  ap.add_argument("--baseline", action="append", default=[], metavar="KEY=N",
                  help="what-if: start the proposal from this band value instead of the logged one, e.g. LatPScaleStandard=115")
  ap.add_argument("--mixed-tuning", action="store_true",
                  help="pool routes driven on different lateral tuning (default: only routes on the newest route's tuning)")
  args = ap.parse_args(argv)
  if not 1 <= args.latest <= MAX_ROUTES:
    print(f"--latest must be at most {MAX_ROUTES}", file=sys.stderr)
    return 2
  overrides = {}
  band_keys = set(lat.P_KEYS) | set(lat.I_KEYS) | set(lat.F_KEYS)
  for item in args.baseline:
    key, _, val = item.partition("=")
    if key not in band_keys or not val.strip().isdigit():
      print(f"--baseline takes KEY=N with KEY one of {', '.join(sorted(band_keys))}", file=sys.stderr)
      return 2
    overrides[key] = val.strip()
  routes = discover_routes(args.routes_root, latest=args.latest)
  if not routes:
    print(f"no route segments with rlogs under {args.routes_root}", file=sys.stderr)
    return 1
  sources = [lat.RouteLog(name, str(i), str(p)) for name, logs in routes for i, p in enumerate(logs)]
  print(f"analyzing {len(routes)} route(s), {len(sources)} segment(s): " + ", ".join(n for n, _ in routes))
  trial = lat.analyze_sources(sources, on_progress=lambda i, n, s: print(f"  [{i + 1}/{n}] {s.route}--{s.segment}", flush=True),
                              baseline_overrides=overrides or None, mixed_tuning=args.mixed_tuning)
  for r in trial["perRoute"]:
    if "used" in r:
      print(f"  {r['route']}: {'used' if r['used'] else 'left out'}  tuning {r.get('tuning') or '-'}  "
            f"min low/std/hwy {'/'.join(f'{m:.1f}' for m in r['minutes'])}")
  print(_table(trial))
  for w in trial["warnings"]:
    print(f"warning: {w}")
  if args.json:
    Path(args.json).write_text(json.dumps(trial, indent=2, sort_keys=True))
  print("Galaxy > Lateral > PID speed bands (P only; I and F unchanged). Values from the newest route's logs:")
  for b in trial["bands"]:
    mark = "" if b["proposed"]["p"] == b["current"]["p"] else f"   (was {b['current']['p']})"
    print(f"{b['pKey']} = {b['proposed']['p']}{mark}")
  if "p" in trial["baseline"].get("scheduleTerms", []):
    print("note: LatGainSchedule has a p term, which overrides these bands; remove it (or apply from Galaxy, which does when the controller accepts the schedule)")
  return 0


if __name__ == "__main__":
  sys.exit(main())
