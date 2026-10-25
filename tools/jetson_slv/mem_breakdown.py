#!/usr/bin/env python3
"""Where speed-limit vision's and V-ASM's memory goes on the comma, stage by stage, in fresh processes.

  cd /data/openpilot && PYTHONPATH=/data/openpilot:/data/openpilot/openpilot \
      /usr/local/venv/bin/python tools/jetson_slv/mem_breakdown.py

SLV_OVERLAY=DIR loads model.py and jetson_net.py from DIR instead (DIR/assets -> the installed assets), to
measure a change before it is deployed.

Run it offroad: each child adds what the real process would (about 130 MB at most).
Each row is the process's own /proc/self/smaps_rollup (summed smaps where there is none) after that stage: Rss, Pss, and Private (clean + dirty),
the memory that goes away when the process does. Pss/Private matter more than Rss, which counts every
shared library page in full.

Children:
  slv        imports, both OpenCV nets, their ABI forward, one camera-sized frame through VisionModelCore.infer,
             then the nets deleted and malloc_trim(0)
  slv-lazy   VisionModelCore(load_nets=False) behind JetsonNet with a stand-in Jetson that answers every pass
             (zeros of the right shape): what the producer holds while the real Jetson answers
  vasm       imports, V-ASM's verified OpenCV net (VASMInference.load), malloc_trim after deleting it
"""
import ctypes
import gc
import os
import subprocess
import sys
import time

FRAME_W, FRAME_H = 1928, 1208


def mem() -> dict:
  out: dict[str, int] = {}
  path = '/proc/self/smaps_rollup' if os.path.exists('/proc/self/smaps_rollup') else '/proc/self/smaps'  # comma: no rollup
  with open(path) as f:
    for line in f:
      parts = line.split()
      if len(parts) >= 2 and parts[0].endswith(':') and parts[1].isdigit():
        out[parts[0][:-1]] = out.get(parts[0][:-1], 0) + int(parts[1])
  return {'rss': out.get('Rss', 0), 'pss': out.get('Pss', 0),
          'private': out.get('Private_Clean', 0) + out.get('Private_Dirty', 0)}


def report(stage: str):
  m = mem()
  print(f"  {stage:<34} rss {m['rss'] / 1024:7.1f}  pss {m['pss'] / 1024:7.1f}  private {m['private'] / 1024:7.1f} MB", flush=True)


def trim():
  gc.collect()
  try:
    ctypes.CDLL('libc.so.6').malloc_trim(0)
  except OSError:
    pass


def _overlay():
  import importlib.util
  overlay = os.environ.get('SLV_OVERLAY')
  if not overlay:
    return
  for name in ('model', 'jetson_net'):
    full = f'openpilot.starpilot.speed_limits.vision.{name}'
    spec = importlib.util.spec_from_file_location(full, os.path.join(overlay, f'{name}.py'))
    module = importlib.util.module_from_spec(spec)
    sys.modules[full] = module
    spec.loader.exec_module(module)


def slv(lazy: bool):
  report('python')
  import numpy as np
  report('+ numpy')
  import cv2
  report('+ cv2')
  import cereal.messaging  # noqa: F401
  from msgq.visionipc import VisionIpcClient  # noqa: F401
  report('+ messaging, visionipc')
  _overlay()
  from openpilot.starpilot.speed_limits.vision import model as M
  report('+ vision model module')
  if lazy:
    from openpilot.starpilot.speed_limits.vision import jetson_net as J
    core = M.VisionModelCore(is_metric=False, load_nets=False)

    class Answers:   # the Jetson answering every pass
      def infer(self, kind, image_u8):
        return np.zeros(J.OUT_SHAPES[kind], np.float32)
    core.net = J.JetsonNet(None, J.DETECTOR, Answers(), loader=core.load_detector)
    core.classifier_net = J.JetsonNet(None, J.CLASSIFIER, Answers(), loader=core.load_classifier)
    report('+ core, load_nets=False')
  else:
    det = M.VisionModelCore._load_net('speed_limit_us_detector.onnx', M.VisionModelCore.DETECTOR_SHA256)
    report('+ detector net')
    cls_net = M.VisionModelCore._load_net('speed_limit_us_value_classifier.onnx', M.VisionModelCore.CLASSIFIER_SHA256)
    report('+ classifier net')
    del det, cls_net
    core = M.VisionModelCore(is_metric=False)   # loads both again and runs the ABI forward
    report('+ core (nets + ABI forward)')
  rng = np.random.default_rng(0)
  frame = rng.integers(0, 255, (FRAME_H, FRAME_W, 3), np.uint8)
  for _ in range(3):
    core.infer(frame)
  report('+ 3 camera-sized frames')
  if not lazy:
    core.net = core.classifier_net = None
    trim()
    report('nets deleted, malloc_trim')
    t0 = time.monotonic()
    core.load_detector(), core.load_classifier()
    print(f"  load_detector + load_classifier took {(time.monotonic() - t0) * 1000:.0f} ms (the lazy fallback's gap)")
  del cv2


def vasm():
  report('python')
  import numpy as np  # noqa: F401
  import cv2  # noqa: F401
  report('+ numpy, cv2')
  from openpilot.starpilot.spot_monitor.inference import DEFAULT_MODEL_PATH, VASMInference
  report('+ V-ASM inference module')
  model = VASMInference(DEFAULT_MODEL_PATH)
  ok = model.load()
  report(f'+ net loaded ({"ok" if ok else model.last_error})')
  model.net = None
  trim()
  report('net deleted, malloc_trim')


def main():
  if len(sys.argv) > 1:
    {'slv': lambda: slv(False), 'slv-lazy': lambda: slv(True), 'vasm': vasm}[sys.argv[1]]()
    return
  for child in ('slv', 'slv-lazy', 'vasm'):
    print(child, flush=True)
    subprocess.run([sys.executable, __file__, child], check=False)


if __name__ == '__main__':
  main()
