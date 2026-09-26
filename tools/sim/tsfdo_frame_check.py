#!/usr/bin/env python3
"""Run recorded road/wide camera frames through the sim's modeld path (TSFDO on ONNX Runtime).

Separates "the model does not see MetaDrive lanes" (domain gap) from "the Mac sim pipeline is
broken": the frames go through the same NV12 packing as the sim camerad, the same ModelState,
warp and ORT policy, and the lane-line probabilities are printed next to the recorded modelV2.

  tools/sim/tsfdo_frame_check.py --dir SEG_DIR [--frames 400]

SEG_DIR holds fcamera.hevc, ecamera.hevc and rlog.zst of one segment. Uses the same env as
tools/sim/run_mac_tsfdo.sh (SIMULATION_ONNX_MODEL, SIMULATION_MODEL_ARTIFACT, TSFDO_DIR).
"""
import argparse
import os
import subprocess
from types import SimpleNamespace

import numpy as np

TSFDO_DIR = os.path.expanduser(os.getenv("TSFDO_DIR", "~/Downloads/tsfdo"))
os.environ.setdefault("SIMULATION", "1")
os.environ.setdefault("SIMULATION_MODEL_ARTIFACT", f"{TSFDO_DIR}/tsfdo_cpu.pkl")
os.environ.setdefault("SIMULATION_ONNX_MODEL", f"{TSFDO_DIR}/driving_supercombo.onnx")
os.environ.setdefault("SIMULATION_TINYGRAD_DEV", "CPU")
os.environ.setdefault("SIMULATION_WARP_DEV", "CPU")

from openpilot.common.transformations.camera import DEVICE_CAMERAS
from openpilot.common.transformations.model import get_warp_matrix
from openpilot.selfdrive.modeld import modeld
from openpilot.system.camerad.cameras.nv12_info import get_nv12_info
from openpilot.tools.lib.logreader import LogReader


def nv12_frames(path, w, h):
  proc = subprocess.Popen(["ffmpeg", "-v", "error", "-i", path, "-f", "rawvideo", "-pix_fmt", "nv12", "-"], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
  size = w * h * 3 // 2
  while (data := proc.stdout.read(size)) and len(data) == size:
    yield np.frombuffer(data, dtype=np.uint8)
  proc.kill()


def pack_into(buf, packed, w, h, stride, y_height, uv_height):
  uv_offset = stride * y_height
  buf[:stride * y_height].reshape(y_height, stride)[:h, :w] = packed[:w * h].reshape(h, w)
  buf[uv_offset:uv_offset + uv_height * stride].reshape(uv_height, stride)[:h // 2, :w] = packed[w * h:].reshape(h // 2, w)


def reproject_nv12(packed, src_w, src_h, src_k, dst_w, dst_h, dst_k):
  """Re-render a pinhole frame with another camera's intrinsics (same pose), as packed NV12."""
  import cv2
  bgr = cv2.cvtColor(packed.reshape(src_h * 3 // 2, src_w), cv2.COLOR_YUV2BGR_NV12)
  bgr = cv2.warpPerspective(bgr, dst_k @ np.linalg.inv(src_k), (dst_w, dst_h), flags=cv2.INTER_LINEAR)
  i420 = cv2.cvtColor(bgr, cv2.COLOR_BGR2YUV_I420).reshape(-1)
  y, u, v = i420[:dst_w * dst_h], i420[dst_w * dst_h:dst_w * dst_h * 5 // 4], i420[dst_w * dst_h * 5 // 4:]
  return np.concatenate([y, np.stack([u, v], axis=1).reshape(-1)])


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("--dir", required=True)
  ap.add_argument("--frames", type=int, default=400)
  ap.add_argument("--as-tici", action="store_true", help="reproject into tici/ar0231 1928x1208 intrinsics, the sim's camera")
  args = ap.parse_args()

  lr = list(LogReader(os.path.join(args.dir, "rlog.zst")))
  init = next(m.initData for m in lr if m.which() == "initData")
  sensor = next(m.roadCameraState.sensor for m in lr if m.which() == "roadCameraState")
  dc = DEVICE_CAMERAS[(str(init.deviceType), str(sensor))]
  rpy = np.array(next(m.liveCalibration.rpyCalib for m in reversed(lr) if m.which() == "liveCalibration"), dtype=np.float32)
  recorded = np.array([list(m.modelV2.laneLineProbs) for m in lr if m.which() == "modelV2"])
  v_ego = np.array([m.carState.vEgo for m in lr if m.which() == "carState"])
  src_dc = dc
  if args.as_tici:
    dc = DEVICE_CAMERAS[("tici", "ar0231")]
  w, h = dc.fcam.width, dc.fcam.height

  model = modeld.ModelState(w, h, False)
  stride, y_height, uv_height, buf_size = get_nv12_info(w, h)
  bufs = {key: SimpleNamespace(data=np.zeros(buf_size, dtype=np.uint8)) for key in ("main", "extra")}
  tfm_main = get_warp_matrix(rpy, dc.fcam.intrinsics, False).astype(np.float32)
  tfm_extra = get_warp_matrix(rpy, dc.ecam.intrinsics, True).astype(np.float32)
  prev_action = SimpleNamespace(desiredCurvature=0.0, desiredAcceleration=0.0)

  probs = []
  sw, sh = src_dc.fcam.width, src_dc.fcam.height
  frames = zip(nv12_frames(os.path.join(args.dir, "fcamera.hevc"), sw, sh), nv12_frames(os.path.join(args.dir, "ecamera.hevc"), sw, sh), strict=False)
  for i, (road, wide) in enumerate(frames):
    if i >= args.frames:
      break
    if args.as_tici:
      road = reproject_nv12(road, sw, sh, src_dc.fcam.intrinsics, w, h, dc.fcam.intrinsics)
      wide = reproject_nv12(wide, sw, sh, src_dc.ecam.intrinsics, w, h, dc.ecam.intrinsics)
    pack_into(bufs["main"].data, road, w, h, stride, y_height, uv_height)
    pack_into(bufs["extra"].data, wide, w, h, stride, y_height, uv_height)
    v = float(v_ego[min(i * len(v_ego) // max(len(recorded), 1), len(v_ego) - 1)])
    frame_bufs, transforms, inputs = modeld._runner_frame_args(
      model, bufs["main"], bufs["extra"], tfm_main, tfm_extra,
      np.zeros(modeld.ModelConstants.DESIRE_LEN, dtype=np.float32), np.array([1, 0], dtype=np.float32),
      0.1, 0.1, prev_action, v, np.array([v, 0.2], dtype=np.float32))
    out = model.run(frame_bufs, transforms, inputs, False)
    probs.append(out["lane_lines_prob"][0, 1::2])
    if i % 40 == 0:
      rec = recorded[min(i, len(recorded) - 1)]
      print(f"frame {i:4d} v={v:4.1f}  tsfdo {np.round(probs[-1], 2)}  recorded {np.round(rec, 2)}", flush=True)

  probs = np.array(probs)
  print(f"device {init.deviceType}/{sensor} {w}x{h} rpyCalib {np.round(rpy, 4)}")
  print(f"TSFDO laneLineProbs mean   {np.round(probs[20:].mean(0), 3)} over {len(probs) - 20} frames")
  print(f"recorded laneLineProbs mean {np.round(recorded[20:len(probs)].mean(0), 3)}")


if __name__ == "__main__":
  main()
