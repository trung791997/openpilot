#!/usr/bin/env python3
"""Compare TensorRT outputs from the Jetson with the onnxruntime reference.

  python compare.py --ref /tmp/jvasm/ref_ort.npy --trt /tmp/jvasm/trt_fp16.npy --threshold 0.9
"""
import argparse

import numpy as np


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("--ref", required=True)
  ap.add_argument("--trt", required=True)
  ap.add_argument("--threshold", type=float, default=0.9, help="p_car alert threshold on the car")
  args = ap.parse_args()

  ref, got = np.load(args.ref), np.load(args.trt)
  if ref.shape != got.shape:
    raise SystemExit(f"shape mismatch {ref.shape} vs {got.shape}")
  d = np.abs(ref - got)
  flips = int(np.sum((ref[:, 1] >= args.threshold) != (got[:, 1] >= args.threshold)))
  print(f"{args.trt}: n {len(ref)}")
  print(f"  |d| all outputs: mean {d.mean():.2e}  p99 {np.percentile(d, 99):.2e}  max {d.max():.2e}")
  print(f"  |d| p_car:       max {d[:, 1].max():.2e}")
  print(f"  argmax agree {int(np.sum(ref.argmax(1) == got.argmax(1)))}/{len(ref)}")
  print(f"  p_car >= {args.threshold} decision flips: {flips}  (ref alerts {int(np.sum(ref[:, 1] >= args.threshold))})")


if __name__ == "__main__":
  main()
