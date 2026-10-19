"""jetlink v0.8.7's fallback reset (warp.prepare_reset) over this fork's
ModelState: SmallModel's state_pairs and packed_input on queues made by
compile_modeld's own makers, on tinygrad's CPU device. Static: no comma GPU."""
import numpy as np
import pytest

tinygrad = pytest.importorskip("tinygrad")
from tinygrad import Tensor  # noqa: E402

from openpilot.selfdrive.modeld.compile_modeld import (  # noqa: E402
  make_fused_supercombo_input_queues, make_split_input_queues, make_supercombo_input_queues)
from openpilot.selfdrive.modeld.jetlink_join import SmallModel  # noqa: E402

DEV = 'CPU'
HISTORY = {'img_q', 'big_img_q', 'feat_q', 'desire_q'}

# shapes as modeld's metadata gives them, shrunk: image (1, 12, h, w) is two
# frames of 6 channels, features (1, T, F), desire (1, T, 8)
SUPERCOMBO = {
  'input_imgs': (1, 12, 4, 8), 'big_input_imgs': (1, 12, 4, 8),
  'desire_pulse': (1, 5, 8), 'traffic_convention': (1, 2), 'features_buffer': (1, 4, 6),
}
VISION = {'img': (1, 12, 4, 8), 'big_img': (1, 12, 4, 8)}
POLICY = {'desire_pulse': (1, 5, 8), 'traffic_convention': (1, 2), 'features_buffer': (1, 4, 6)}


class FakeModelState:
  """The parts of modeld.ModelState the reset reads, as its __init__ and
  _reset_state lay them out."""
  def __init__(self, kind):
    self.frame_views = None
    if kind == 'split':
      self.input_queues, self.npy = make_split_input_queues(VISION, POLICY, 1, DEV)
    elif kind == 'supercombo':
      self.input_queues, self.npy = make_supercombo_input_queues(SUPERCOMBO, 1, DEV)
    else:
      self.input_queues, self.npy, self.frame_views = make_fused_supercombo_input_queues(SUPERCOMBO, 1, DEV, 64)
    self.numpy_inputs = {'desire_pulse': np.zeros((1, 5, 8), dtype=np.float32),
                         'traffic_convention': np.zeros((1, 2), dtype=np.float32)}
    self.prev_desire = np.zeros(8, dtype=np.float32)
    self.prev_desired_curv_key = 'prev_desired_curv'
    self.full_prev_desired_curv = np.zeros((1, 10, 1), dtype=np.float32)


def dirty(model):
  for name, q in model.input_queues.items():
    if q.device != 'NPY':
      q.assign(Tensor.ones(*q.shape, dtype=q.dtype, device=q.device)).realize()
  for array in [*model.npy.values(), *(model.frame_views or {}).values(), *model.numpy_inputs.values(),
                model.full_prev_desired_curv, model.prev_desire]:
    array.fill(1)


def host_arrays(model):
  return [*model.npy.values(), *(model.frame_views or {}).values(), *model.numpy_inputs.values(),
          model.full_prev_desired_curv, model.prev_desire]


@pytest.mark.parametrize('kind', ['split', 'supercombo', 'fused'])
def test_state_pairs_are_the_gpu_history(kind):
  model = FakeModelState(kind)
  small = SmallModel(model)
  assert set(small.state_pairs) == HISTORY
  assert all(model.input_queues[name].device == DEV for name in small.state_pairs)
  # what is left is host memory, which packed_input covers
  assert {n for n, q in model.input_queues.items() if q.device == 'NPY'} == set(model.input_queues) - HISTORY


@pytest.mark.parametrize('kind', ['split', 'supercombo', 'fused'])
def test_packed_input_fill_zeroes_every_host_input(kind):
  model = FakeModelState(kind)
  small = SmallModel(model)
  dirty(model)
  npy_tensors = {n: q for n, q in model.input_queues.items() if q.device == 'NPY'}
  assert all(q.to('CPU').numpy().any() for q in npy_tensors.values())
  small.packed_input.fill(0)
  for array in host_arrays(model)[:-1]:   # all but prev_desire, which prepare_reset zeroes itself
    assert not array.any()
  # the NPY tensors the graph reads share that memory
  assert not any(q.to('CPU').numpy().any() for q in npy_tensors.values())


@pytest.mark.parametrize('kind', ['split', 'supercombo', 'fused'])
def test_prepare_reset_zeroes_history_in_place(kind):
  from jetlink.openpilot.warp import prepare_reset

  model = FakeModelState(kind)
  small = SmallModel(model)
  buffers = {n: model.input_queues[n].uop.buffer for n in small.state_pairs}
  reset = prepare_reset(small)
  dirty(model)
  assert all(model.input_queues[n].numpy().any() for n in small.state_pairs)
  reset()
  for name in small.state_pairs:
    q = model.input_queues[name]
    assert q.uop.buffer is buffers[name]   # the JIT's buffer identities kept
    assert not q.numpy().any(), name
  for array in host_arrays(model):
    assert not array.any()
