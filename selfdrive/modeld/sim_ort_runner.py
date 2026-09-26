"""ONNX Runtime model runner for the Mac MetaDrive simulator (SIMULATION only).

Drop-in for tinygrad's OnnxRunner inside compile_modeld.make_run_supercombo: the history
queues stay in tinygrad, only the network itself runs in ONNX Runtime. Used because the
tinygrad METAL JIT replay of TSFDO disagrees with CPU (see STATUS), while ORT's CoreML
provider on CPU+GPU matches the ORT CPU reference within 0.07 at ~15 ms/frame on the owner's Mac.
"""
import os
from pathlib import Path

import numpy as np
from tinygrad.dtype import dtypes
from tinygrad.tensor import Tensor

ORT_TO_TINYGRAD = {
  "tensor(uint8)": dtypes.uint8,
  "tensor(float16)": dtypes.float16,
  "tensor(float)": dtypes.float32,
}


class _InputSpec:
  def __init__(self, dtype):
    self.dtype = dtype


class OrtModelRunner:
  def __init__(self, onnx_path: str | Path, provider: str | None = None, compute_units: str | None = None):
    import onnxruntime as ort

    provider = provider or os.getenv("SIMULATION_ORT_PROVIDER", "coreml")
    if provider == "coreml":
      cache = Path(os.getenv("SIMULATION_ORT_CACHE", Path.home() / "Library/Caches/openpilot-coreml"))
      cache.mkdir(parents=True, exist_ok=True)
      # CPUAndGPU by default: the Neural Engine ("ALL") is faster but drifts up to 0.6 on the outputs.
      options = {
        "ModelFormat": "MLProgram",
        "MLComputeUnits": compute_units or os.getenv("SIMULATION_ORT_COMPUTE_UNITS", "CPUAndGPU"),
        "ModelCacheDirectory": str(cache),
      }
      providers = [("CoreMLExecutionProvider", options), "CPUExecutionProvider"]
    elif provider == "cpu":
      providers = ["CPUExecutionProvider"]
    else:
      raise ValueError(f"Unknown SIMULATION_ORT_PROVIDER {provider!r}")

    self.session = ort.InferenceSession(str(onnx_path), providers=providers)
    self.graph_inputs = {i.name: _InputSpec(ORT_TO_TINYGRAD[i.type]) for i in self.session.get_inputs()}
    self.output_names = [o.name for o in self.session.get_outputs()]

  def __call__(self, inputs: dict[str, Tensor]) -> dict[str, Tensor]:
    feed = {name: np.ascontiguousarray(inputs[name].numpy()) for name in self.graph_inputs}
    outputs = self.session.run(self.output_names, feed)
    return {name: Tensor(value) for name, value in zip(self.output_names, outputs, strict=True)}


ORT_TO_NUMPY = {"tensor(uint8)": np.uint8, "tensor(float16)": np.float16, "tensor(float)": np.float32}


class _NumpyQueue:
  """Host-side stand-in for a tinygrad history queue; modeld only ever calls assign() on one."""
  def __init__(self, tensor: Tensor):
    self.array = np.zeros_like(tensor.numpy())

  def assign(self, value):
    self.array[...] = value
    return self


def numpy_queues(input_queues) -> dict[str, _NumpyQueue]:
  return {key: _NumpyQueue(input_queues[key]) for key in ("img_q", "big_img_q", "feat_q", "desire_q")}


def _shift(queue: _NumpyQueue, new_value: np.ndarray) -> np.ndarray:
  queue.array[:-1] = queue.array[1:]
  queue.array[-1:] = new_value
  return queue.array


def make_numpy_run_supercombo(runner: OrtModelRunner, metadata, frame_skip, input_queues):
  """The policy-pipeline run_supercombo with the queue shifts in numpy instead of eager tinygrad.

  Mirrors compile_modeld.make_run_supercombo (IMAGE_HISTORY_IN_POLICY): shift_and_sample with
  sample_skip for images/features and sample_desire (max over each skip window) for desire.
  Eager tinygrad on CPU cost ~33 ms per frame here, which kept modelV2 under 20 Hz.
  Returns the run_policy function and the queues to put in modeld's input_queues.
  """
  from openpilot.selfdrive.modeld.compile_modeld import _detect_desire_key, _detect_vision_keys, _packed_policy_shapes

  input_shapes = metadata["model"]["input_shapes"]
  desire_key = _detect_desire_key(input_shapes)
  road_key, wide_key = _detect_vision_keys(input_shapes)
  packed_shapes, packed_sizes = _packed_policy_shapes(input_shapes, include_prev_feature=True)
  split_at = np.cumsum(packed_sizes[:-1])
  dtypes_np = {i.name: ORT_TO_NUMPY[i.type] for i in runner.session.get_inputs()}

  def sample_skip(array):
    sampled = array[::frame_skip]
    return sampled.reshape(1, sampled.shape[0] * sampled.shape[1], *sampled.shape[2:])

  def sample_desire(array):
    pooled = array.reshape(-1, frame_skip, *array.shape[1:]).max(1)
    return pooled.reshape(1, pooled.shape[0] * pooled.shape[1], *pooled.shape[2:])

  def run_policy(warped, img_q, big_img_q, feat_q, desire_q, packed_npy_inputs):
    warped = warped.numpy()
    packed = packed_npy_inputs.numpy()
    unpacked = {key: value.reshape(shape) for (key, shape), value in zip(packed_shapes.items(), np.split(packed, split_at), strict=True)}
    model_inputs = {
      road_key: sample_skip(_shift(img_q, warped[0:1])),
      wide_key: sample_skip(_shift(big_img_q, warped[1:2])),
      desire_key: sample_desire(_shift(desire_q, unpacked.pop("desire").reshape(1, 1, -1))),
      "features_buffer": sample_skip(_shift(feat_q, unpacked.pop("prev_feat").reshape(1, 1, -1))).reshape(input_shapes["features_buffer"]),
      **unpacked,
    }
    feed = {name: np.ascontiguousarray(value, dtype=dtypes_np[name]) for name, value in model_inputs.items()}
    output = runner.session.run(runner.output_names[:1], feed)[0]
    return Tensor(output.astype(np.float32)),

  return run_policy, numpy_queues(input_queues)


def make_numpy_run_split(vision_runner: OrtModelRunner, policy_runner: OrtModelRunner, metadata, frame_skip, input_queues):
  """Split vision + policy model (the stock openpilot model) on ONNX Runtime, queues in numpy.

  Mirrors compile_modeld.make_run_split_policy (IMAGE_HISTORY_IN_POLICY): vision on the shifted image
  queues, its hidden state pushed into the feature queue, then the policy. Returns (vision, policy) outputs.
  """
  from openpilot.selfdrive.modeld.compile_modeld import _detect_desire_key, _detect_vision_keys, _packed_policy_shapes

  vision_shapes = metadata["vision"]["input_shapes"]
  policy_shapes = metadata["policy"]["input_shapes"]
  features_slice = metadata["vision"]["output_slices"]["hidden_state"]
  desire_key = _detect_desire_key(policy_shapes)
  road_key, wide_key = _detect_vision_keys(vision_shapes)
  packed_shapes, packed_sizes = _packed_policy_shapes(policy_shapes)
  split_at = np.cumsum(packed_sizes[:-1])
  vision_dtypes = {i.name: ORT_TO_NUMPY[i.type] for i in vision_runner.session.get_inputs()}
  policy_dtypes = {i.name: ORT_TO_NUMPY[i.type] for i in policy_runner.session.get_inputs()}

  def sample_skip(array):
    sampled = array[::frame_skip]
    return sampled.reshape(1, sampled.shape[0] * sampled.shape[1], *sampled.shape[2:])

  def sample_desire(array):
    pooled = array.reshape(-1, frame_skip, *array.shape[1:]).max(1)
    return pooled.reshape(1, pooled.shape[0] * pooled.shape[1], *pooled.shape[2:])

  def run_policy(warped, img_q, big_img_q, feat_q, desire_q, packed_npy_inputs):
    warped = warped.numpy()
    packed = packed_npy_inputs.numpy()
    unpacked = {key: value.reshape(shape) for (key, shape), value in zip(packed_shapes.items(), np.split(packed, split_at), strict=True)}
    vision_inputs = {road_key: sample_skip(_shift(img_q, warped[0:1])), wide_key: sample_skip(_shift(big_img_q, warped[1:2]))}
    feed = {name: np.ascontiguousarray(value, dtype=vision_dtypes[name]) for name, value in vision_inputs.items()}
    vision_output = vision_runner.session.run(vision_runner.output_names[:1], feed)[0].astype(np.float32)
    new_feature = vision_output[:, features_slice].reshape(1, 1, -1)
    policy_inputs = {
      "features_buffer": sample_skip(_shift(feat_q, new_feature)).reshape(policy_shapes["features_buffer"]),
      desire_key: sample_desire(_shift(desire_q, unpacked.pop("desire").reshape(1, 1, -1))),
      **unpacked,
    }
    feed = {name: np.ascontiguousarray(value, dtype=policy_dtypes[name]) for name, value in policy_inputs.items()}
    policy_output = policy_runner.session.run(policy_runner.output_names[:1], feed)[0].astype(np.float32)
    return Tensor(vision_output), Tensor(policy_output)

  return run_policy, numpy_queues(input_queues)
