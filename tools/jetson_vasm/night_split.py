#!/usr/bin/env python3
"""Day/night split of a V-ASM model on labelled crops, scored on the Mac (onnxruntime) or from a Jetson run.

  build:  python night_split.py build --crops ~/oprad-scratch/vasm/crops12nr --onnx run12.onnx --outdir /tmp/jnight
  score:  python night_split.py score --outdir /tmp/jnight [--pred-suffix trt_fp16]

build writes, for each of night and day, crops_<g>.npy (uint8 [N,352,352,3] RGB), labels_<g>.npy and the
onnxruntime outputs ref_<g>.npy. Night is mean_luma < NIGHT_LUMA, as tools/vasm_finetune/benchmark.py's night
subset; day is mean_luma >= DAY_LUMA. Each group takes every car (1) and distant (2) crop and NEG_N no-car (0)
crops (seed 0), owner labels only; 'x' (unlabellable) is left out and counted. Both groups come from the same split, so a model
trained on that split is scored on crops it has seen: an upper bound, not a held-out result.

score reports, per group, recall and precision for label 1 (car) at the car's threshold, p_car >= ON.
Route data: keep --outdir outside the repo.
"""
import argparse
import collections
import csv
import json
import os

import numpy as np

NIGHT_LUMA = 40.0
DAY_LUMA = 100.0
NEG_N = 1500
ON = 0.9
GROUPS = ("night", "day")


def build(args):
  import cv2
  import onnxruntime as ort
  labels = json.load(open(os.path.join(args.crops, "labels.json")))
  # owner labels only (no "by"), as train.py --owner-only: the agents put the adjacent lane one dashed line out
  labels = {k: v for k, v in labels.items() if "by" not in v}
  rows = [r for r in csv.DictReader(open(os.path.join(args.crops, "manifest.csv")))
          if r["file"] in labels and r["split"] == args.split]
  sess = ort.InferenceSession(args.onnx, providers=["CPUExecutionProvider"])
  name = sess.get_inputs()[0].name
  rng = np.random.default_rng(0)
  os.makedirs(args.outdir, exist_ok=True)
  for g in GROUPS:
    sel = [r for r in rows if (float(r["mean_luma"]) < NIGHT_LUMA if g == "night" else float(r["mean_luma"]) >= DAY_LUMA)]
    lab = [labels[r["file"]]["label"] for r in sel]
    print(f"{g}: {len(sel)} labelled crops from {len({r['route'] for r in sel})} routes, {collections.Counter(map(str, lab))}")
    pos = [r for r, l in zip(sel, lab) if l in (1, 2)]
    neg = [r for r, l in zip(sel, lab) if l == 0]
    neg = [neg[i] for i in sorted(rng.choice(len(neg), min(NEG_N, len(neg)), replace=False))]
    pick = pos + neg
    crops = np.stack([cv2.cvtColor(cv2.imread(os.path.join(args.crops, r["file"])), cv2.COLOR_BGR2RGB) for r in pick])
    y = np.array([labels[r["file"]]["label"] for r in pick], np.int8)
    ref = np.stack([sess.run(None, {name: (c.astype(np.float32) / 255.0).transpose(2, 0, 1)[None]})[0].reshape(-1) for c in crops])
    np.save(os.path.join(args.outdir, f"crops_{g}.npy"), crops)
    np.save(os.path.join(args.outdir, f"labels_{g}.npy"), y)
    np.save(os.path.join(args.outdir, f"ref_{g}.npy"), ref)
    print(f"  kept {len(pick)}: car {int((y == 1).sum())} distant {int((y == 2).sum())} nocar {int((y == 0).sum())}")


def predict(args):
  """Another model's onnxruntime outputs on the crops build kept, saved as <suffix>_<g>.npy for score."""
  import onnxruntime as ort
  sess = ort.InferenceSession(args.onnx, providers=["CPUExecutionProvider"])
  name = sess.get_inputs()[0].name
  for g in GROUPS:
    crops = np.load(os.path.join(args.outdir, f"crops_{g}.npy"))
    out = np.stack([sess.run(None, {name: (c.astype(np.float32) / 255.0).transpose(2, 0, 1)[None]})[0].reshape(-1) for c in crops])
    np.save(os.path.join(args.outdir, f"{args.suffix}_{g}.npy"), out)


def score(args):
  for g in GROUPS:
    y = np.load(os.path.join(args.outdir, f"labels_{g}.npy"))
    p = np.load(os.path.join(args.outdir, f"{args.pred_suffix}_{g}.npy"))[:, 1]
    alert = p >= ON
    car, nocar = y == 1, y == 0
    tp, fn, fp = int((alert & car).sum()), int((~alert & car).sum()), int((alert & nocar).sum())
    dist_alert = int((alert & (y == 2)).sum())
    print(f"{g:5s} car {int(car.sum()):4d}  recall {tp / max(1, tp + fn):.3f} ({fn} missed)  "
          f"false alerts {fp}/{int(nocar.sum())} ({fp / max(1, nocar.sum()):.4f})  distant alerted {dist_alert}/{int((y == 2).sum())}  "
          f"median p_car on missed cars {np.median(p[~alert & car]) if fn else float('nan'):.3f}")


def main():
  ap = argparse.ArgumentParser()
  sub = ap.add_subparsers(dest="cmd", required=True)
  b = sub.add_parser("build")
  b.add_argument("--crops", required=True)
  b.add_argument("--onnx", required=True)
  b.add_argument("--split", default="train")
  b.add_argument("--outdir", required=True)
  s = sub.add_parser("score")
  s.add_argument("--outdir", required=True)
  s.add_argument("--pred-suffix", default="ref")
  pr = sub.add_parser("predict")
  pr.add_argument("--outdir", required=True)
  pr.add_argument("--onnx", required=True)
  pr.add_argument("--suffix", required=True)
  args = ap.parse_args()
  {"build": build, "predict": predict, "score": score}[args.cmd](args)


if __name__ == "__main__":
  main()
