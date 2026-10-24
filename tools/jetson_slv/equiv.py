#!/usr/bin/env python3
"""Does speed-limit vision read the same signs with the Jetson's TensorRT engines as with the comma's OpenCV nets?

Three steps, on recorded road video (fcamera.hevc) of drives with signs; route data stays outside the repo.

  dump     (Mac or comma): every 3rd frame (the producer's ~6 Hz) through VisionModelCore.observe() with the
           OpenCV nets. Saves every net input as the u8 the Jetson would get, every OpenCV output, and each
           frame's published read.
             python3 tools/jetson_slv/equiv.py dump --out /tmp/jslv/dump SEG_DIR [SEG_DIR ...]
  trt      (Jetson, next to slv_server.py): the same u8 inputs through the engines.
             python3 equiv.py trt --dump DIR --detector det.engine --classifier cls.engine --tag fp16
  compare  (Mac): raw output differences, then observe() again over the same frames with the nets answering
           from the TensorRT outputs (looked up by input hash), frame by frame against the OpenCV reads.
           An input the TensorRT run never saw (the sign logic took another path) is counted, never guessed.
             python3 tools/jetson_slv/equiv.py compare --dump DIR --tag fp16 SEG_DIR [SEG_DIR ...]
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import subprocess
import sys

import numpy as np

DETECTOR, CLASSIFIER = 0, 1
NAMES = {DETECTOR: 'detector', CLASSIFIER: 'classifier'}
FRAME_STEP = 3          # 20 fps camera, the producer infers every 166 ms
FRAME_DT = 0.05


def _key(u8: np.ndarray) -> str:
  return hashlib.sha1(np.ascontiguousarray(u8).tobytes()).hexdigest()


def frames(seg_dir: str):
  """BGR frames of seg_dir/fcamera.hevc, every FRAME_STEP-th, with their index. The size is the file's own
  (1344x760 on the comma 4's road camera)."""
  path = os.path.join(seg_dir, 'fcamera.hevc')
  wh = subprocess.check_output(['ffprobe', '-v', 'error', '-f', 'hevc', '-select_streams', 'v:0', '-show_entries',
                                'stream=width,height', '-of', 'csv=p=0', path], text=True).strip()
  w, h = (int(x) for x in wh.split(','))
  cmd = ['ffmpeg', '-v', 'error', '-f', 'hevc', '-i', path, '-f', 'rawvideo', '-pix_fmt', 'bgr24', '-']
  p = subprocess.Popen(cmd, stdout=subprocess.PIPE)
  n = w * h * 3
  i = 0
  while True:
    buf = p.stdout.read(n)
    if len(buf) < n:
      break
    if i % FRAME_STEP == 0:
      yield i, np.frombuffer(buf, np.uint8).reshape(h, w, 3)
    i += 1
  p.wait()


def _core():
  from openpilot.starpilot.speed_limits.vision.model import VisionModelCore
  return VisionModelCore(is_metric=False)


def _observe_all(core, seg_dirs, on_frame=None):
  reads = []
  t = 1.0
  for seg in seg_dirs:
    core.reset()
    for i, frame in frames(seg):
      if on_frame:
        on_frame(seg, i)
      r = core.observe(frame, now=t)
      reads.append([os.path.basename(os.path.normpath(seg)), i] + (list(r) if r else [0, 0.0, 0, 0]))
      t += FRAME_STEP * FRAME_DT
  return reads


class _Recorder:
  def __init__(self, net, kind, store):
    self.net, self.kind, self.store = net, kind, store
    self.blob = None

  def setInput(self, blob):  # noqa: N802 (cv2's name)
    self.blob = blob

  def forward(self):
    from openpilot.starpilot.speed_limits.vision.jetson_net import blob_to_u8
    u8 = blob_to_u8(self.blob)
    self.net.setInput(self.blob)
    out = self.net.forward()
    k = _key(u8)
    if k not in self.store['seen']:
      self.store['seen'].add(k)
      self.store['u8'].append(u8)
      self.store['cv2'].append(out.reshape(-1).copy())
      # the OpenCV net on the blob the Jetson would rebuild (u8 * float32(1/255)): the input path alone
      self.net.setInput((u8[None].astype(np.float32) * np.float32(1 / 255.0)))
      self.store['cv2_from_u8'].append(self.net.forward().reshape(-1).copy())
    return out


def cmd_dump(args):
  core = _core()
  stores = {k: {'seen': set(), 'u8': [], 'cv2': [], 'cv2_from_u8': []} for k in NAMES}
  core.net = _Recorder(core.net, DETECTOR, stores[DETECTOR])
  core.classifier_net = _Recorder(core.classifier_net, CLASSIFIER, stores[CLASSIFIER])
  reads = _observe_all(core, args.segs)
  os.makedirs(args.out, exist_ok=True)
  for kind, name in NAMES.items():
    s = stores[kind]
    np.savez(os.path.join(args.out, f'{name}.npz'), u8=np.stack(s['u8']), cv2=np.stack(s['cv2']),
             cv2_from_u8=np.stack(s['cv2_from_u8']))
    print(f'{name}: {len(s["u8"])} distinct inputs')
  with open(os.path.join(args.out, 'reads_cv2.json'), 'w') as f:
    json.dump(reads, f)
  print(f'{len(reads)} frames, {sum(r[2] > 0 for r in reads)} with a published read')


def cmd_trt(args):
  sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
  from slv_server import Engine, SIZES, OUT_SHAPES
  from vasm_trt import _Cuda
  cuda = _Cuda()
  for kind, path in ((DETECTOR, args.detector), (CLASSIFIER, args.classifier)):
    eng = Engine(path, cuda, SIZES[kind], OUT_SHAPES[kind])
    d = np.load(os.path.join(args.dump, f'{NAMES[kind]}.npz'))
    out = np.stack([eng.infer(u8) for u8 in d['u8']])
    np.save(os.path.join(args.dump, f'{NAMES[kind]}_trt_{args.tag}.npy'), out)
    print(f'{NAMES[kind]}: {len(out)} outputs')


class _Answer:
  def __init__(self, table, kind, misses):
    self.table, self.kind, self.misses = table, kind, misses
    self.blob = None

  def setInput(self, blob):  # noqa: N802
    self.blob = blob

  def forward(self):
    from openpilot.starpilot.speed_limits.vision.jetson_net import blob_to_u8
    from openpilot.starpilot.speed_limits.vision.jetson_net import OUT_SHAPES
    k = _key(blob_to_u8(self.blob))
    if k not in self.table:
      self.misses[self.kind] += 1
      return np.zeros(OUT_SHAPES[self.kind], np.float32)
    return self.table[k].reshape(OUT_SHAPES[self.kind])


def _raw_report(name, ref, other, label):
  diff = np.abs(ref - other)
  line = f'{name} {label}: max |diff| {diff.max():.3g}, p99 {np.percentile(diff, 99):.3g}'
  if name == 'classifier':
    agree = (ref.argmax(1) == other.argmax(1)).mean()
    line += f', top class agrees {agree:.2%} of {len(ref)}'
  else:
    conf_ref, conf = ref.reshape(-1, 5, 1344)[:, 4].max(1), other.reshape(-1, 5, 1344)[:, 4].max(1)
    line += f', best-box conf max |diff| {np.abs(conf_ref - conf).max():.3g}'
  print(line)


def cmd_compare(args):
  tables = {}
  for kind, name in NAMES.items():
    d = np.load(os.path.join(args.dump, f'{name}.npz'))
    _raw_report(name, d['cv2'], d['cv2_from_u8'], 'OpenCV on the rebuilt u8 blob')
    if args.tag:
      trt = np.load(os.path.join(args.dump, f'{name}_trt_{args.tag}.npy'))
      _raw_report(name, d['cv2'], trt, f'TensorRT {args.tag}')
      tables[kind] = {_key(u8): o for u8, o in zip(d['u8'], trt)}
    else:
      tables[kind] = {_key(u8): o for u8, o in zip(d['u8'], d['cv2_from_u8'])}
  core = _core()
  misses = {DETECTOR: 0, CLASSIFIER: 0}
  core.net = _Answer(tables[DETECTOR], DETECTOR, misses)
  core.classifier_net = _Answer(tables[CLASSIFIER], CLASSIFIER, misses)
  reads = _observe_all(core, args.segs)
  with open(os.path.join(args.dump, 'reads_cv2.json')) as f:
    ref = json.load(f)
  if len(ref) != len(reads):
    print(f'FRAME COUNT DIFFERS: {len(ref)} vs {len(reads)}')
    return 1
  speed_diff = [(a, b) for a, b in zip(ref, reads) if a[2] != b[2]]
  any_diff = [(a, b) for a, b in zip(ref, reads)
              if a[2] != b[2] or a[4:] != b[4:] or abs(a[3] - b[3]) > 1e-6]   # mph, support/episode, confidence
  print(f'{len(reads)} frames: published mph differs on {len(speed_diff)}, any field on {len(any_diff)}; '
        f'inputs the other run never saw: detector {misses[DETECTOR]}, classifier {misses[CLASSIFIER]}')
  for a, b in speed_diff[:20]:
    print(f'  seg {a[0]} frame {a[1]}: OpenCV {a[2]} mph, other {b[2]} mph')
  return 0 if not speed_diff and not any(misses.values()) else 1


def main():
  ap = argparse.ArgumentParser()
  sub = ap.add_subparsers(dest='cmd', required=True)
  d = sub.add_parser('dump')
  d.add_argument('--out', required=True)
  d.add_argument('segs', nargs='+')
  t = sub.add_parser('trt')
  t.add_argument('--dump', required=True)
  t.add_argument('--detector', required=True)
  t.add_argument('--classifier', required=True)
  t.add_argument('--tag', required=True)
  c = sub.add_parser('compare')
  c.add_argument('--dump', required=True)
  c.add_argument('--tag', help='TensorRT run to compare; omitted: the rebuilt-u8 OpenCV run (input path only)')
  c.add_argument('segs', nargs='+')
  args = ap.parse_args()
  return {'dump': cmd_dump, 'trt': cmd_trt, 'compare': cmd_compare}[args.cmd](args) or 0


if __name__ == '__main__':
  sys.exit(main())
