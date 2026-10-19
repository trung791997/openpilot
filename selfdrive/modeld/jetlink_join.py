"""
modeld's side of jetlink: this fork's ModelState behind the face jetlink's
joining model drives (upstream zoompilot's stock modeld ModelState), and the
joined model behind the face this modeld reads.

jetlink calls run(bufs, transforms, inputs, after_enqueue) with bufs and
transforms keyed 'img'/'big_img' and reads desire*, traffic_convention and
action_t from inputs. This ModelState runs on its own road/wide keys, takes
prepare_only, blinker_on and after_output_sync, and modeld reads generation
flags (is_v*, mlsim) off the model to pick the action law. Static only: no
replay or road evidence yet.

jetlink's fallback reset (warp.prepare_reset, v0.8.7) zeroes the small
model's history the way upstream modeld's warmup does: the recurrent GPU
state named by `state_pairs`, one host buffer `packed_input`, and
`prev_desire`. SmallModel carries those names over this fork's ModelState,
which keeps its history in more pieces; see HostInputs.
"""
from __future__ import annotations

from collections.abc import Callable

import numpy as np


class HostInputs:
  """Upstream modeld's `packed_input` (one uint8 buffer of every host input
  and both frames, zeroed after warmup) over this fork's ModelState, which
  keeps them in several arrays. fill(v) writes each of them in place, as
  ModelState._reset_state zeroes them after warmup:

  - every array in `npy`: the views the NPY tensors read (tfm, big_tfm and the
    packed policy inputs: desire, traffic_convention, prev_feat, ...)
  - `frame_views`, the fused graph's frame copies, when there are any
  - every array in `numpy_inputs`, which run() copies into `npy`
  - `full_prev_desired_curv`, the desired-curvature history, when the model has one

  Not touched, unlike _reset_state: _blob_cache (camera buffer wrappers, not
  history) and last_warp_output (overwritten by the next run). The arrays are
  looked up on each fill, so one re-made by _reset_state is the one filled."""

  def __init__(self, model):
    self.model = model

  def arrays(self) -> list[np.ndarray]:
    m = self.model
    found = list(m.npy.values())
    found += list((getattr(m, 'frame_views', None) or {}).values())
    found += list(m.numpy_inputs.values())
    if getattr(m, 'prev_desired_curv_key', None) is not None and getattr(m, 'full_prev_desired_curv', None) is not None:
      found.append(m.full_prev_desired_curv)
    return found

  def fill(self, value) -> None:
    for array in self.arrays():
      array.fill(value)


class SmallModel:
  """The small ModelState as jetlink's joining model calls it. modeld sets
  prepare_only and blinker_on before each run(); jetlink sets in_control,
  frame_drop_ratio and lat_delay here. Anything else reads through."""

  # stock modeld's ModelState, to jetlink's prepare_reset: no modeld_v2
  # bundle adapter in between, so it reads state_pairs and packed_input
  adapter = None

  def __init__(self, model):
    self.model = model
    self.prepare_only = False
    self.blinker_on = False
    self.in_control = True
    self.frame_drop_ratio = 0.0
    self.lat_delay = 0.0
    self.PLANPLUS_CONTROL = 1.0
    self.chestnut = False
    self.packed_input = HostInputs(model)

  @property
  def state_pairs(self) -> dict[str, str]:
    """Upstream's {state: next_state} for the recurrent state on the GPU.
    prepare_reset reads only the keys, as names in input_queues to zero. Here
    that is every queue off the host (img_q, big_img_q, feat_q, desire_q on
    QUEUE_DEV), the set jetlink v0.8.5 cleared; the NPY tensors are host
    memory and packed_input zeroes them. This fork's queues advance in place,
    so each pairs with itself."""
    return {name: name for name, q in self.model.input_queues.items() if q.device != 'NPY'}

  def run(self, bufs: dict, transforms: dict[str, np.ndarray], inputs: dict[str, np.ndarray],
          after_enqueue: Callable[[], None] | None = None):
    m = self.model
    # jetlink's aliases are the same buffers; this ModelState caches a blob per key
    bufs = {m.road_key: bufs[m.road_key], m.wide_key: bufs[m.wide_key]}
    transforms = {m.road_key: transforms[m.road_key], m.wide_key: transforms[m.wide_key]}
    # inputs it has no slot for (action_t, for a model without one) it skips
    return m.run(bufs, transforms, inputs, self.prepare_only, after_output_sync=after_enqueue,
                 blinker_on=self.blinker_on)

  def __getattr__(self, name):
    return getattr(self.model, name)


def big_flags(output_slices) -> dict[str, bool]:
  """The action law for comma's large model, from its outputs, as upstream's
  get_action_from_model picks it: an 'action' head is curvature * v^2 and
  acceleration (this fork's v15/v16 law), otherwise the plan (the mlsim plan
  law, never v9's desired_curvature)."""
  action = 'action' in output_slices
  return {'is_v9': False, 'is_v14': False, 'is_v15': False, 'is_v16': action, 'mlsim': True}


class Joined:
  """The joining model as modeld reads it. While the small model drives every
  read is the small model's. While the large one drives, the generation flags
  follow its outputs and nothing is prepare-only; model_id and
  policy_generation stay the small model's, the slot the params describe."""

  def __init__(self, joined, small: SmallModel):
    self.__dict__['joined'] = joined
    self.__dict__['small'] = small

  @property
  def big(self) -> bool:
    return bool(self.joined.chestnut)

  def _big_flag(self, name: str):
    return big_flags(self.joined.output_slices)[name]

  is_v9 = property(lambda self: self._big_flag('is_v9') if self.big else self.small.is_v9)
  is_v14 = property(lambda self: self._big_flag('is_v14') if self.big else self.small.is_v14)
  is_v15 = property(lambda self: self._big_flag('is_v15') if self.big else self.small.is_v15)
  is_v16 = property(lambda self: self._big_flag('is_v16') if self.big else self.small.is_v16)
  mlsim = property(lambda self: self._big_flag('mlsim') if self.big else self.small.mlsim)
  can_prepare_only = property(lambda self: False if self.big else self.small.can_prepare_only)
  uses_external_gpu = property(lambda self: False)
  last_warp_output = property(lambda self: None if self.big else self.small.last_warp_output)
  model_id = property(lambda self: self.small.model_id)
  policy_generation = property(lambda self: self.small.policy_generation)

  def run(self, bufs, transforms, inputs, prepare_only=False, after_output_sync=None, *, blinker_on=False):
    m = self.small.model
    bufs = {**bufs, 'img': bufs[m.road_key], 'big_img': bufs[m.wide_key]}
    transforms = {**transforms, 'img': transforms[m.road_key], 'big_img': transforms[m.wide_key]}
    self.small.prepare_only = prepare_only
    self.small.blinker_on = blinker_on
    return self.joined.run(bufs, transforms, inputs, after_output_sync)

  def __getattr__(self, name):
    return getattr(self.joined, name)

  def __setattr__(self, name, value):
    setattr(self.joined, name, value)
