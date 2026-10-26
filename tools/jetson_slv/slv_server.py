#!/usr/bin/env python3
"""Jetson side: answer speed-limit vision's detector and classifier passes with TensorRT, over the comma's
ffs.slv interface (class FF/53/4C), next to Jetlink's interface 0 and V-ASM's.

  python3 slv_server.py --detector detector_fp32.engine --classifier value_classifier_fp32.engine \
      --detector-sha256 82408b68... --classifier-sha256 07c6696e...

fp32, not fp16: on route 21 sign frames it differs from the comma's OpenCV about a third as much (equiv.py, 2026-10-24).
Copy next to tools/jetson_vasm's vasm_trt.py, vasm_usb_server.py and yield_gate.py (it reuses their CUDA, USB
and yield code). It yields the GPU to the driving model as V-ASM's server does (yield_gate.py).
Like V-ASM's server it claims only its own interface and never sets the configuration or resets the device.

Wire format: the comma's openpilot/starpilot/speed_limits/vision/jetson_net.py, kept in step by hand:
  request  SLQ1: <4sIBxxxI magic, seq, kind (0 detector, 1 classifier), nbytes> + u8 [3, S, S]
  response SLR1: <4sIiII magic, seq, status, server_us, nfloats> + f32 [nfloats]
  hello:   seq 0xFFFFFFFF, status 100, server_us = detector tag, nfloats = classifier tag (sha256[:8])
"""
import argparse
import struct
import sys
import time

import numpy as np
import tensorrt as trt

from vasm_trt import _Cuda
from vasm_usb_server import UsbLink
from yield_gate import YieldGate

REQ = struct.Struct("<4sIBxxxI")
RESP = struct.Struct("<4sIiII")
REQ_MAGIC, RESP_MAGIC = b"SLQ1", b"SLR1"
STATUS_OK, STATUS_BAD_REQUEST, STATUS_INFER_FAILED, STATUS_HELLO, HELLO_SEQ = 0, 1, 2, 100, 0xFFFFFFFF
STATUS_YIELD = 101   # yield_gate.py: the driving model needs the GPU
DETECTOR, CLASSIFIER = 0, 1
SIZES = {DETECTOR: 256, CLASSIFIER: 128}
OUT_SHAPES = {DETECTOR: (1, 5, 1344), CLASSIFIER: (1, 19)}
SCALE = np.float32(1 / 255.0)   # blobFromImage's scalefactor, as the comma's OpenCV applies it


def log(msg: str):
  print(f"{time.strftime('%H:%M:%S')} slv: {msg}", flush=True)


class SlvUsbLink(UsbLink):
  IFACE_CLASS = (0xFF, 0x53, 0x4C)
  REQ_SIZE = REQ.size + 3 * 256 * 256


class Engine:
  """One float input [1, 3, S, S] and one float output, as the two ONNX models."""

  def __init__(self, path: str, cuda: _Cuda, size: int, out_shape: tuple):
    rt = trt.Runtime(trt.Logger(trt.Logger.WARNING))
    with open(path, "rb") as f:
      self.engine = rt.deserialize_cuda_engine(f.read())
    if self.engine is None:
      raise RuntimeError(f"could not deserialize {path}")
    self.cuda, self.ctx = cuda, self.engine.create_execution_context()
    names = [self.engine.get_tensor_name(i) for i in range(self.engine.num_io_tensors)]
    ins = [n for n in names if self.engine.get_tensor_mode(n) == trt.TensorIOMode.INPUT]
    outs = [n for n in names if self.engine.get_tensor_mode(n) == trt.TensorIOMode.OUTPUT]
    if len(ins) != 1 or len(outs) != 1:
      raise RuntimeError(f"{path}: expected 1 input and 1 output, got {ins} {outs}")
    if tuple(self.engine.get_tensor_shape(ins[0])) != (1, 3, size, size) or \
       tuple(self.engine.get_tensor_shape(outs[0])) != out_shape:
      raise RuntimeError(f"{path}: shapes {tuple(self.engine.get_tensor_shape(ins[0]))} -> "
                         f"{tuple(self.engine.get_tensor_shape(outs[0]))}, expected (1, 3, {size}, {size}) -> {out_shape}")
    for n in (ins[0], outs[0]):
      if self.engine.get_tensor_dtype(n) != trt.DataType.FLOAT:
        raise RuntimeError(f"{path}: {n} is {self.engine.get_tensor_dtype(n)}, expected FLOAT")
    self.in_host = np.empty((1, 3, size, size), np.float32)
    self.out_host = np.empty(out_shape, np.float32)
    self.in_dev, self.out_dev = cuda.alloc(self.in_host.nbytes), cuda.alloc(self.out_host.nbytes)
    self.ctx.set_tensor_address(ins[0], self.in_dev)
    self.ctx.set_tensor_address(outs[0], self.out_dev)

  def infer(self, image_u8: np.ndarray) -> np.ndarray:
    """image_u8: [3, S, S] RGB, the comma's blob before scaling. Returns the flat float32 output."""
    np.multiply(image_u8, SCALE, out=self.in_host[0], casting="unsafe")
    self.cuda.h2d(self.in_dev, self.in_host)
    if not self.ctx.execute_async_v3(self.cuda.stream.value):
      raise RuntimeError("execute_async_v3 failed")
    self.cuda.d2h(self.out_host, self.out_dev)
    self.cuda.sync()
    return self.out_host.reshape(-1).copy()


def unpack_request(buf) -> tuple[int, int, np.ndarray]:
  if len(buf) < REQ.size:
    raise ValueError(f"short request {len(buf)}")
  magic, seq, kind, n = REQ.unpack_from(buf)
  if magic != REQ_MAGIC or kind not in SIZES:
    raise ValueError(f"bad header {magic!r} kind {kind}")
  size = SIZES[kind]
  if n != 3 * size * size or len(buf) != REQ.size + n:
    raise ValueError(f"kind {kind}: {len(buf)} bytes, header says {n}")
  return seq, kind, np.frombuffer(bytes(buf), np.uint8, offset=REQ.size).reshape(3, size, size)


def pack_response(seq: int, status: int, server_us: int, out: np.ndarray | None = None) -> bytes:
  out = np.zeros(0, np.float32) if out is None else out.astype(np.float32)
  return RESP.pack(RESP_MAGIC, seq, status, server_us, out.size) + out.tobytes()


def pack_hello(detector_sha256: str, classifier_sha256: str, status: int = STATUS_HELLO) -> bytes:
  return RESP.pack(RESP_MAGIC, HELLO_SEQ, status, int(detector_sha256[:8], 16), int(classifier_sha256[:8], 16))


def serve(link, engines: dict, hello: bytes, gate=None, yield_hello: bytes = b''):
  n, t_log = 0, time.monotonic()
  while True:
    if not link.open():
      time.sleep(1.0)
      continue
    try:
      while True:
        buf = link.recv()
        if buf is None:
          link.hello(yield_hello if gate is not None and gate.yielding() else hello)
          continue
        t0 = time.perf_counter()
        try:
          seq, kind, image = unpack_request(buf)
        except ValueError as e:
          log(f"dropped: {e}")
          link.send(pack_response(0, STATUS_BAD_REQUEST, 0))
          continue
        if gate is not None and gate.yielding():
          link.send(pack_response(seq, STATUS_YIELD, 0))
          continue
        try:
          out, status = engines[kind].infer(image), STATUS_OK
        except Exception as e:  # report, keep serving
          log(f"infer failed: {e}")
          out, status = None, STATUS_INFER_FAILED
        us = int((time.perf_counter() - t0) * 1e6)
        if gate is not None and status == STATUS_OK:
          gate.note(kind, us)
        link.send(pack_response(seq, status, us, out))
        n += 1
        if time.monotonic() - t_log > 60:
          log(f"{n} passes served")
          t_log = time.monotonic()
    except OSError as e:
      log(f"link lost: {e}")
      link.close()


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("--detector", required=True)
  ap.add_argument("--classifier", required=True)
  ap.add_argument("--detector-sha256", required=True)
  ap.add_argument("--classifier-sha256", required=True)
  ap.add_argument("--no-yield", action="store_true", help="never give the GPU back to the driving model (bench only)")
  args = ap.parse_args()
  cuda = _Cuda()
  engines = {DETECTOR: Engine(args.detector, cuda, SIZES[DETECTOR], OUT_SHAPES[DETECTOR]),
             CLASSIFIER: Engine(args.classifier, cuda, SIZES[CLASSIFIER], OUT_SHAPES[CLASSIFIER])}
  for kind, e in engines.items():
    e.infer(np.zeros((3, SIZES[kind], SIZES[kind]), np.uint8))  # warm before the first real pass
  link = SlvUsbLink()
  log("ready on usb class %02x/%02x/%02x" % SlvUsbLink.IFACE_CLASS)
  try:
    gate = None if args.no_yield else YieldGate(log=log)
    serve(link, engines, pack_hello(args.detector_sha256, args.classifier_sha256), gate,
          pack_hello(args.detector_sha256, args.classifier_sha256, STATUS_YIELD))
  except KeyboardInterrupt:
    link.close()
    sys.exit(0)


if __name__ == "__main__":
  main()
