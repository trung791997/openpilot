"""Pick one (model, on, off) from select_queue.py's qualifying candidates, using a held-out road route. Never looks at test.

  python tools/vasm_finetune/pick.py ft/run12_select.json --road-crops cropsroad --rule highest --out ft/run12_pick.json

Rule (owner, 2026-10-14, fixed before run11's test): the candidates are select_queue.py's qualifying (model, on) on the
queue and validation groups. Keep those that, on the held-out road route (owner road labels in <road-crops>/labels.json),
publish no more time on labelled no-car crops AND cover no less labelled car time than the shipped model at 0.80 / 0.65.
Then order by fewest missed events, lowest false-occupied s/h, lowest latency, on closest to 0.80.
--rule highest (owner, 2026-10-14, after run11 failed false-occupied at a low pick of 0.73; the run12 rule) puts the
highest on first, then the same order. Moved here from the scratch diag/run11_pick.py; same logic.
Prints "MODEL ON OFF" or "NONE" as its last line.
"""

import argparse
import csv
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from benchmark import FOCUS_LEAD_DIST, default_model, infer, night_gate, simulate  # noqa: E402

ROAD_ROUTE = "00000329--2afda26f0e"


def road_scorer(crops, route):
  with open(os.path.join(crops, "labels.json")) as f:
    labels = json.load(f)
  with open(os.path.join(crops, "manifest.csv")) as f:
    rows = [r for r in csv.DictReader(f) if r["route"] == route]
  if not rows:
    sys.exit(f"no crops for road route {route} in {crops}")
  by = {}
  for i, r in enumerate(rows):
    by.setdefault(r["seg"], []).append(i)
  cache = {}

  def road(model, on, off):
    if model not in cache:
      cache[model] = night_gate(rows, infer(model, crops, rows)[:, 1])
    P = cache[model]
    false = car = 0.0
    for idx in by.values():
      t = np.array(sorted({float(rows[k]["t"]) for k in idx}))
      pos = {x: n for n, x in enumerate(t)}
      dt = float(np.median(np.diff(t)))
      p = {s: np.zeros(len(t)) for s in ("left", "right")}
      foc = {s: np.zeros(len(t), bool) for s in p}
      y = {s: np.full(len(t), -1) for s in p}
      for i in idx:
        r = rows[i]
        n, s = pos[float(r["t"])], r["image_side"]
        p[s][n] = P[i]
        foc[s][n] = r["blinker"] == "1" or (r["radar_status"] == "1" and float(r["radar_drel"]) <= FOCUS_LEAD_DIST)
        lab = labels.get(r["file"], {}).get("label")
        y[s][n] = lab if lab in (0, 1, 2) else -1
      pub = simulate(t, p, on, off, "left", t[0] - 0.5, focus=foc)
      for s in p:
        false += (pub[s] & ((y[s] == 0) | (y[s] == 2))).sum() * dt
        car += (pub[s] & (y[s] == 1)).sum() * dt
    return round(false, 2), round(car, 2)

  return road


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("select", help="select_queue.py --out JSON")
  ap.add_argument("--road-crops", required=True)
  ap.add_argument("--road-route", default=ROAD_ROUTE)
  ap.add_argument("--rule", choices=("highest", "order"), default="highest")
  ap.add_argument("--out", default=None)
  args = ap.parse_args()

  road = road_scorer(args.road_crops, args.road_route)
  ref = road(default_model(), 0.80, 0.65)
  print(f"road reference upstream@0.80: no-car published {ref[0]} s, car covered {ref[1]} s")
  with open(args.select) as f:
    ok = [r for r in json.load(f)["results"] if r["ok"]]
  print(len(ok), "queue-qualifying candidates")
  keep = []
  for r in ok:
    f, c = road(r["model"], r["on"], r["off"])
    if f <= ref[0] and c >= ref[1]:
      keep.append({**r, "road": (f, c)})
  print(len(keep), "also pass the road route")
  print("rule:", args.rule)
  hi = args.rule == "highest"
  key = lambda r: ((-r["on"],) if hi else ()) + (r["events"]["missed"], r["events"]["false_occupied_s_per_h"],  # noqa: E731
                                                 r["events"]["latency_p50"], abs(r["on"] - 0.80))
  pick = min(keep, key=key) if keep else None
  if args.out:
    with open(args.out, "w") as f:
      json.dump({"road_reference": ref, "rule": args.rule, "kept": keep, "pick": pick}, f, indent=1)
  print(f"{pick['model']} {pick['on']} {pick['off']}" if pick else "NONE")


if __name__ == "__main__":
  main()
