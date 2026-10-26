#!/usr/bin/env python3
"""Mac side of stage 1: build a uint8 crop set and its onnxruntime reference outputs.

  python make_testset.py --crops ~/oprad-scratch/vasm/crops12nr --onnx run12_soup_ep3.onnx \
      --n 512 --outdir /tmp/jvasm

Writes crops.npy (uint8 [N,352,352,3], RGB, the comma's crop_sq) and ref_ort.npy
(float32 [N,3], CPU fp32). The crops stay outside the repo: they are route data.
The PNGs are already 352x352 letterboxed; cv2 loads BGR, so they are flipped to RGB
here as the comma's cvtColor(..., COLOR_YUV2RGB_NV12) produces RGB.
"""
import argparse
import csv
import os

import cv2
import numpy as np
import onnxruntime as ort


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("--crops", required=True)
  ap.add_argument("--onnx", required=True)
  ap.add_argument("--n", type=int, default=512)
  ap.add_argument("--split", default="test")
  ap.add_argument("--outdir", required=True)
  args = ap.parse_args()

  with open(os.path.join(args.crops, "manifest.csv")) as f:
    rows = [r for r in csv.DictReader(f) if r["split"] == args.split]
  rng = np.random.default_rng(0)
  rows = [rows[i] for i in sorted(rng.choice(len(rows), min(args.n, len(rows)), replace=False))]

  crops, p_car_manifest = [], []
  for r in rows:
    img = cv2.imread(os.path.join(args.crops, r["file"]))
    if img is None or img.shape != (352, 352, 3):
      raise SystemExit(f"bad crop {r['file']}: {None if img is None else img.shape}")
    crops.append(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    p_car_manifest.append(float(r["p_car"]))
  crops = np.stack(crops)

  sess = ort.InferenceSession(args.onnx, providers=["CPUExecutionProvider"])
  name = sess.get_inputs()[0].name
  ref = np.stack([sess.run(None, {name: (c.astype(np.float32) / 255.0).transpose(2, 0, 1)[None]})[0].reshape(-1)
                  for c in crops])

  os.makedirs(args.outdir, exist_ok=True)
  np.save(os.path.join(args.outdir, "crops.npy"), crops)
  np.save(os.path.join(args.outdir, "ref_ort.npy"), ref)
  d = np.abs(ref[:, 1] - np.array(p_car_manifest))
  print(f"n {len(crops)}  crops.npy {crops.nbytes / 1e6:.1f} MB")
  print(f"ORT p_car vs manifest p_car (channel-order sanity): median |d| {np.median(d):.4f}  max {d.max():.4f}")


if __name__ == "__main__":
  main()
