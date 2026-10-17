"""Independent check of an autolabel.py output, required before pipeline.py lets any of it into training.

  python tools/vasm_finetune/autolabel_check.py --labels crops12/autolabels.json --crops cropsroad --route 00000329--2afda26f0e

The labeller's thresholds come from validation groups that share routes and scenery with the models' training, so its
own validation numbers are optimistic (v1: 0.98 there, 0.70 class-1 precision on route 329). This re-applies the exact
decision (autolabel.apply, same models, smoothing, thresholds, agreement, classes, edge rule) to a route that is in
neither the labeller's crops nor the models' training, and scores it against the owner's labels there. A class passes
when the 95% Wilson lower bound of its precision is >= --min-lower. Nothing here is tuned: thresholds are read from
the report, and the check route is only ever a pass/fail gate. Every failed attempt that leads to a redesign spends
some of that route's independence, so record the attempts and move to a fresh owner-labelled route when it runs out.

Before scoring it proves the re-application is faithful: the report's validation result is recomputed from the
labeller's own crops and must match exactly. Writes <labels>.check.json with the sha256 of the labels it judged.
"""

import argparse
import collections
import csv
import hashlib
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import autolabel as A  # noqa: E402
from train import is_val  # noqa: E402


def wilson_lo(k, n, z=1.96):
  if n == 0:
    return 0.0
  p = k / n
  return (p + z * z / (2 * n) - z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n))) / (1 + z * z / n)


def owner_y(rows, labels):
  y = []
  for r in rows:
    e = labels.get(r["file"], {})
    y.append(e["label"] if "by" not in e and e.get("label") in (0, 1, 2) else -1)
  return np.array(y)


def decisions(rep, crops, rows, cache):
  a = rep["args"]
  seqs = A.sides(rows)
  S = A.smooth(A.scores(rep["models"], crops, rows, cache), rows, seqs, a["smooth"])
  Ss = [A.smooth(A.scores([m], crops, rows, cache), rows, seqs, a["smooth"]) for m in rep["models"]] if a.get("agree") else None
  keep = {int(c) for c in a.get("classes", "0,1,2").split(",") if c != ""}
  return A.apply(S, Ss, rep["thresholds"], a.get("agree", False), keep, rows, seqs, a["edge_gap"],
                 a.get("min_speed", 0.0))


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("--labels", required=True, help="autolabel.py output; its .report.json must sit next to it")
  ap.add_argument("--crops", required=True, help="owner-labelled crops holding the check route")
  ap.add_argument("--route", required=True)
  ap.add_argument("--min-lower", type=float, default=0.98)
  args = ap.parse_args()
  rep = json.load(open(args.labels + ".report.json"))
  digest = hashlib.sha256(open(args.labels, "rb").read()).hexdigest()
  if rep.get("sha256") not in (None, digest):
    sys.exit("labels file differs from the one its report describes")
  for m, h in zip(rep["models"], rep.get("model_sha256", [])):
    if hashlib.sha256(open(m, "rb").read()).hexdigest() != h:
      sys.exit(f"{m} changed since labelling")
  src = rep["args"]["crops"]
  cache = os.path.dirname(os.path.abspath(args.labels))

  src_rows = [r for r in csv.DictReader(open(os.path.join(src, "manifest.csv"))) if r["split"] == "train"]
  if any(r["route"] == args.route for r in src_rows):
    sys.exit(f"{args.route} is in the labeller's own crops ({src}); it cannot check them")

  # 1. faithfulness: recompute the report's validation numbers from the labeller's crops
  src_labels = json.load(open(os.path.join(src, "labels.json")))
  val = np.array([is_val(f"{r['route']}/{r['seg']}", rep["args"]["val_frac"]) for r in src_rows])
  yv = owner_y(src_rows, src_labels)
  cal = val & (yv >= 0)
  dv = decisions(rep, src, src_rows, cache)
  again = json.loads(json.dumps(A.report(dv[cal], yv[cal])))
  if again != rep["val"]:
    sys.exit(f"re-applied decision does not reproduce the report's validation result:\n{again}\n{rep['val']}")

  # 2. the check route
  rows = [r for r in csv.DictReader(open(os.path.join(args.crops, "manifest.csv"))) if r["route"] == args.route]
  y = owner_y(rows, json.load(open(os.path.join(args.crops, "labels.json"))))
  dec = decisions(rep, args.crops, rows, os.path.join(cache, "check_" + args.route[:8]))
  m = y >= 0
  emitted = sorted({e["label"] for e in json.load(open(args.labels)).values()})
  res = {"labels": os.path.abspath(args.labels), "labels_sha256": digest, "route": args.route, "crops": args.crops,
         "min_lower": args.min_lower, "owner_crops": int(m.sum()),
         "owner_classes": dict(collections.Counter(int(c) for c in y[m])), "emitted_classes": emitted, "classes": {}}
  for c in (0, 1, 2):
    sel = m & (dec == c)
    n, k = int(sel.sum()), int((y[sel] == c).sum())
    res["classes"][c] = {"auto": n, "correct": k, "precision": round(k / n, 4) if n else None,
                         "wilson_lo": round(wilson_lo(k, n), 4),
                         "wrong_as": {int(w): int(((y == w) & sel).sum()) for w in (0, 1, 2) if w != c},
                         "recall": round(k / max(int((m & (y == c)).sum()), 1), 4),
                         "passed": n > 0 and wilson_lo(k, n) >= args.min_lower}
  res["errors"] = [{"file": rows[i]["file"], "owner": int(y[i]), "auto": int(dec[i])} for i in np.where(m & (dec >= 0) & (dec != y))[0]]
  res["passed"] = bool(emitted) and all(res["classes"][c]["passed"] for c in emitted)
  out = args.labels + ".check.json"
  with open(out, "w") as f:
    json.dump(res, f, indent=1)
  print(json.dumps({k: res[k] for k in ("owner_crops", "owner_classes", "emitted_classes", "classes", "passed")}, indent=1))
  print(len(res["errors"]), "errors listed in", out)
  sys.exit(0 if res["passed"] else 1)


if __name__ == "__main__":
  main()
