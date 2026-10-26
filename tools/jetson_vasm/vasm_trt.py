#!/usr/bin/env python3
"""V-ASM inference on the Jetson with TensorRT, for the second-USB-interface plan.

Runs on the Jetson only. The comma sends the 352x352x3 uint8 crop it already builds in
adj_spot_monitor_vision_inference.py (crop_sq, before /255), 371,712 bytes, and gets
back the model's 3 floats. Normalization (/255, HWC->CHW) happens here so the cable
carries uint8, a quarter of the float32 blob.

The Jetson has the TensorRT python bindings but no cuda-python/pycuda and no
libcudart, so device memory goes through the CUDA driver API (libcuda) by ctypes.

Stage 1 (no comma, no USB):
  python3 vasm_trt.py --engine run12_fp16_b1.engine --npy crops.npy --out trt_fp16.npy
crops.npy is uint8 [N,352,352,3] from make_testset.py; compare.py checks the outputs
against onnxruntime on the Mac.
"""
import argparse
import ctypes
import time

import numpy as np
import tensorrt as trt

INPUT_SIZE = 352
CUDA_PATHS = ("libcuda.so.1", "/opt/nvidia/l4t-gpu-libs/nvgpu/libcuda.so.1")


def _load_cuda():
  for p in CUDA_PATHS:
    try:
      return ctypes.CDLL(p)
    except OSError:
      pass
  raise OSError(f"libcuda not found in {CUDA_PATHS}")


class _Cuda:
  def __init__(self):
    self.lib = _load_cuda()
    self._ck(self.lib.cuInit(0), "cuInit")
    dev = ctypes.c_int()
    self._ck(self.lib.cuDeviceGet(ctypes.byref(dev), 0), "cuDeviceGet")
    self.ctx = ctypes.c_void_p()
    self._ck(self.lib.cuDevicePrimaryCtxRetain(ctypes.byref(self.ctx), dev), "cuDevicePrimaryCtxRetain")
    self._ck(self.lib.cuCtxSetCurrent(self.ctx), "cuCtxSetCurrent")
    self.stream = ctypes.c_void_p()
    self._ck(self.lib.cuStreamCreate(ctypes.byref(self.stream), 0), "cuStreamCreate")

  @staticmethod
  def _ck(rc, what):
    if rc != 0:
      raise RuntimeError(f"{what} failed: CUresult {rc}")

  def alloc(self, nbytes: int) -> int:
    p = ctypes.c_uint64()
    self._ck(self.lib.cuMemAlloc_v2(ctypes.byref(p), ctypes.c_size_t(nbytes)), "cuMemAlloc")
    return p.value

  def h2d(self, dst: int, src: np.ndarray):
    self._ck(self.lib.cuMemcpyHtoDAsync_v2(ctypes.c_uint64(dst), src.ctypes.data_as(ctypes.c_void_p),
                                           ctypes.c_size_t(src.nbytes), self.stream), "cuMemcpyHtoD")

  def d2h(self, dst: np.ndarray, src: int):
    self._ck(self.lib.cuMemcpyDtoHAsync_v2(dst.ctypes.data_as(ctypes.c_void_p), ctypes.c_uint64(src),
                                           ctypes.c_size_t(dst.nbytes), self.stream), "cuMemcpyDtoH")

  def sync(self):
    self._ck(self.lib.cuStreamSynchronize(self.stream), "cuStreamSynchronize")


class VasmTrt:
  def __init__(self, engine_path: str):
    self.cuda = _Cuda()
    self.logger = trt.Logger(trt.Logger.WARNING)
    with open(engine_path, "rb") as f:
      self.engine = trt.Runtime(self.logger).deserialize_cuda_engine(f.read())
    if self.engine is None:
      raise RuntimeError(f"could not deserialize {engine_path}")
    self.ctx = self.engine.create_execution_context()
    names = [self.engine.get_tensor_name(i) for i in range(self.engine.num_io_tensors)]
    ins = [n for n in names if self.engine.get_tensor_mode(n) == trt.TensorIOMode.INPUT]
    outs = [n for n in names if self.engine.get_tensor_mode(n) == trt.TensorIOMode.OUTPUT]
    if len(ins) != 1 or len(outs) != 1:
      raise RuntimeError(f"expected 1 input and 1 output, got {ins} {outs}")
    self.in_name, self.out_name = ins[0], outs[0]
    in_shape = tuple(self.engine.get_tensor_shape(self.in_name))
    if in_shape != (1, 3, INPUT_SIZE, INPUT_SIZE):
      raise RuntimeError(f"engine input {in_shape}, expected (1, 3, {INPUT_SIZE}, {INPUT_SIZE})")
    for n in (self.in_name, self.out_name):
      if self.engine.get_tensor_dtype(n) != trt.DataType.FLOAT:
        raise RuntimeError(f"{n} is {self.engine.get_tensor_dtype(n)}, expected FLOAT")
    self.in_host = np.empty(in_shape, dtype=np.float32)
    self.out_host = np.empty(tuple(self.engine.get_tensor_shape(self.out_name)), dtype=np.float32)
    self.in_dev = self.cuda.alloc(self.in_host.nbytes)
    self.out_dev = self.cuda.alloc(self.out_host.nbytes)
    self.ctx.set_tensor_address(self.in_name, self.in_dev)
    self.ctx.set_tensor_address(self.out_name, self.out_dev)

  def infer(self, crop_u8: np.ndarray, timing: dict | None = None) -> np.ndarray:
    """crop_u8: uint8 [352,352,3], the comma's crop_sq. Returns float32 [3]."""
    if crop_u8.shape != (INPUT_SIZE, INPUT_SIZE, 3) or crop_u8.dtype != np.uint8:
      raise ValueError(f"crop must be uint8 [{INPUT_SIZE},{INPUT_SIZE},3], got {crop_u8.dtype} {crop_u8.shape}")
    t0 = time.perf_counter()
    np.multiply(crop_u8.transpose(2, 0, 1), np.float32(1 / 255.0), out=self.in_host[0], casting="unsafe")
    t1 = time.perf_counter()
    self.cuda.h2d(self.in_dev, self.in_host)
    if not self.ctx.execute_async_v3(self.cuda.stream.value):
      raise RuntimeError("execute_async_v3 failed")
    self.cuda.d2h(self.out_host, self.out_dev)
    self.cuda.sync()
    t2 = time.perf_counter()
    if timing is not None:
      timing["norm_ms"] = (t1 - t0) * 1e3
      timing["gpu_ms"] = (t2 - t1) * 1e3
    return self.out_host.reshape(-1).copy()


def _pct(a, q):
  return float(np.percentile(a, q))


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("--engine", required=True)
  ap.add_argument("--npy", required=True, help="uint8 [N,352,352,3]")
  ap.add_argument("--out", required=True)
  ap.add_argument("--warmup", type=int, default=20)
  args = ap.parse_args()

  crops = np.load(args.npy)
  m = VasmTrt(args.engine)
  for i in range(min(args.warmup, len(crops))):
    m.infer(crops[i])

  outs, norm, gpu, total = [], [], [], []
  for c in crops:
    t = {}
    s = time.perf_counter()
    outs.append(m.infer(c, t))
    total.append((time.perf_counter() - s) * 1e3)
    norm.append(t["norm_ms"])
    gpu.append(t["gpu_ms"])
  np.save(args.out, np.stack(outs))
  print(f"engine {args.engine}  n {len(crops)}  output {m.out_name} {m.out_host.shape}")
  for k, a in (("normalize", norm), ("h2d+infer+d2h", gpu), ("total", total)):
    print(f"  {k:14s} mean {np.mean(a):.3f}  p50 {_pct(a, 50):.3f}  p99 {_pct(a, 99):.3f}  max {np.max(a):.3f} ms")


if __name__ == "__main__":
  main()
