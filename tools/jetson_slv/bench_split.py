#!/usr/bin/env python3
"""Where speed-limit vision's CPU goes per frame: NV12->BGR, ROI letterbox+blob, detector, one classifier call.

  python tools/jetson_slv/bench_split.py [--frames 60] [--core 2]

Synthetic road-camera-sized NV12 frame (1928x1208, camerad's road stream); single OpenCV thread as the
producer runs. Only the detector and classifier would move to a Jetson; the rest stays on the comma.
"""
import argparse
import os
import time

import numpy as np

from openpilot.starpilot.speed_limits.vision.model import ROI_WINDOWS, VisionModelCore, cv2

W, H = 1928, 1208


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("--frames", type=int, default=60)
  ap.add_argument("--core", type=int)
  a = ap.parse_args()
  if a.core is not None:
    os.sched_setaffinity(0, {a.core})
  cv2.setNumThreads(1)
  core = VisionModelCore(is_metric=False)
  rng = np.random.default_rng(0)
  nv12 = rng.integers(0, 255, (H * 3 // 2, W), dtype=np.uint8)
  t = {"nv12_to_bgr": 0.0, "roi_letterbox_blob": 0.0, "detector": 0.0, "classifier": 0.0}
  size = core.DETECTOR_SIZE
  for _ in range(a.frames):
    t0 = time.perf_counter()
    frame = cv2.cvtColor(nv12, cv2.COLOR_YUV2BGR_NV12)
    t1 = time.perf_counter()
    left, top, right, bottom = ROI_WINDOWS[-1]["bounds"]
    roi = frame[int(H * top):int(H * bottom), int(W * left):int(W * right)]
    lb, *_ = core._letterbox(roi, shape=(size, size))
    blob = cv2.dnn.blobFromImage(lb, scalefactor=1 / 255.0, size=(size, size), swapRB=True, crop=False)
    t2 = time.perf_counter()
    core.net.setInput(blob)
    core.net.forward()
    t3 = time.perf_counter()
    crop = core._square_resize(roi[:80, :80], size=core.CLASSIFIER_SIZE)
    core.classifier_net.setInput(cv2.dnn.blobFromImage(crop, 1 / 255.0, (core.CLASSIFIER_SIZE,) * 2, swapRB=True, crop=False))
    core.classifier_net.forward()
    t4 = time.perf_counter()
    for k, d in zip(t, (t1 - t0, t2 - t1, t3 - t2, t4 - t3)):
      t[k] += d
  for k, v in t.items():
    print(f"{k:20s} {1000 * v / a.frames:7.2f} ms/frame")


if __name__ == "__main__":
  main()
