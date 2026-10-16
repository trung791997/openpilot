"""Upstream vasm-op's training route on our labels: Ultralytics yolo26n-cls (ImageNet weights), 3 classes, WITHOUT fliplr.

  cd ~/oprad-scratch/vasm
  venv-ul/bin/python <repo>/tools/vasm_finetune/ul_scratch.py --crops crops12 --out ft/ul13

The comparison asked for on 2026-10-16 (owner): does the upstream recipe, started from ImageNet rather than from the
shipped V-ASM model, beat our fine-tune? Same data and split as train.py's run12 recipe so only the training route
differs: owner labels only, accepted hard negatives in train, hard negatives within --hardneg-gap s of a car dropped,
queue segments never trained on, validation = train.py's hash groups. fliplr is 0 because the left and right crops are
not mirror images of each other (different polygons, and the driver camera is mirrored). Other augmentation is
Ultralytics' default, as upstream. Exports <out>/epN.onnx for the chosen epochs; pipeline.py judges them with recipe
"models" (select, pick, test once, gate).
"""

import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from train import drop_hardneg_near_cars, in_segments, is_val, load_rows, queue_segments  # noqa: E402

NAMES = ("0_nocar", "1_car", "2_distant")


def build(crops, out, hardneg, queue, gap, frac):
  import json
  rows = load_rows(crops, True, set(json.load(open(hardneg))) if hardneg and os.path.exists(hardneg) else ())
  if gap > 0:
    rows = drop_hardneg_near_cars(rows, crops, gap)
  segs = queue_segments(queue) if queue and os.path.exists(queue) else set()
  n = {}
  for r in rows:
    val = is_val(r["group"], frac)
    if val and r.get("hardneg"):
      continue
    if not val and in_segments(r, segs):
      continue
    split = "val" if val else "train"
    d = os.path.join(out, split, NAMES[r["y"]])
    os.makedirs(d, exist_ok=True)
    dst = os.path.join(d, r["file"].replace("/", "_"))
    if not os.path.exists(dst):
      os.symlink(os.path.abspath(os.path.join(crops, r["file"])), dst)
    n[(split, r["y"])] = n.get((split, r["y"]), 0) + 1
  return n


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("--crops", required=True)
  ap.add_argument("--out", required=True)
  ap.add_argument("--hardneg", default="hardneg/accepted.json")
  ap.add_argument("--hardneg-gap", type=float, default=1.0)
  ap.add_argument("--val-frac", type=float, default=0.1)
  ap.add_argument("--weights", default="yolo26n-cls.pt")
  ap.add_argument("--epochs", type=int, default=50)
  ap.add_argument("--batch", type=int, default=64)
  ap.add_argument("--export-epochs", default="10,20,30,40,50", help="also export these epochs' weights (save_period)")
  ap.add_argument("--device", default="mps")
  args = ap.parse_args()

  data = os.path.join(os.path.abspath(args.out), "data")
  if not os.path.isdir(os.path.join(data, "val")):
    print("dataset:", build(args.crops, data, args.hardneg, os.path.join(args.crops, "queue.json"), args.hardneg_gap, args.val_frac))

  from ultralytics import YOLO
  proj, name = os.path.abspath(args.out), "train"
  last = os.path.join(proj, name, "weights", "last.pt")
  if os.path.exists(last) and not os.path.exists(os.path.join(proj, name, "weights", "best.pt")):
    YOLO(last).train(resume=True)
  elif not os.path.exists(last):
    YOLO(args.weights).train(data=data, epochs=args.epochs, imgsz=352, batch=args.batch, device=args.device, fliplr=0.0,
                             project=proj, name=name, exist_ok=True, save_period=10, seed=0, deterministic=False, plots=False)
  wdir = os.path.join(proj, name, "weights")
  for e in [int(x) for x in args.export_epochs.split(",") if x]:
    src = os.path.join(wdir, "last.pt" if e == args.epochs else f"epoch{e}.pt")
    dst = os.path.join(proj, f"ep{e}.onnx")
    if os.path.exists(src) and not os.path.exists(dst):
      os.replace(YOLO(src).export(format="onnx", imgsz=352, opset=17, simplify=True, dynamic=True), dst)
  for f in ("best",):
    dst = os.path.join(proj, f"{f}.onnx")
    if not os.path.exists(dst):
      os.replace(YOLO(os.path.join(wdir, f"{f}.pt")).export(format="onnx", imgsz=352, opset=17, simplify=True, dynamic=True), dst)
  print("exported:", sorted(x for x in os.listdir(proj) if x.endswith(".onnx")))


if __name__ == "__main__":
  main()
