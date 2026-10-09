import numpy as np

from jetlink.openpilot.interface import Openpilot, conformance

from openpilot.selfdrive.modeld.jetlink_join import Joined, SmallModel, big_flags
from openpilot.starpilot import jetlink_adapter


class FakeModelState:
  road_key, wide_key = 'input_imgs', 'big_input_imgs'
  desire_key = 'desire_pulse'
  numpy_inputs = {'desire_pulse': np.zeros(8), 'traffic_convention': np.zeros(2)}
  is_v9 = is_v14 = is_v15 = False
  is_v16 = True
  mlsim = True
  can_prepare_only = True
  uses_external_gpu = False
  last_warp_output = 'warp'
  model_id = 'rdf43'
  policy_generation = 'v16'

  def __init__(self):
    self.calls = []

  def run(self, bufs, transforms, inputs, prepare_only, after_output_sync=None, shared_warp=None, *, blinker_on=False):
    self.calls.append((bufs, transforms, inputs, prepare_only, after_output_sync, blinker_on))
    return {'ok': True}


class FakeJoining:
  """jetlink's JoiningModelState as modeld sees it: chestnut is True while the
  large model drives, and reads it does not define follow the active model."""
  def __init__(self, small):
    self.small = small
    self.chestnut = False
    self.output_slices = {'action': slice(0, 2), 'plan': slice(2, 10)}
    self.handovers = 0
    self.in_control = True
    self.frame_drop_ratio = 0.0

  def run(self, bufs, transforms, inputs, after_enqueue=None):
    assert {'img', 'big_img'} <= set(bufs) and {'img', 'big_img'} <= set(transforms)
    return self.small.run(bufs, transforms, inputs, after_enqueue)


def make():
  inner = FakeModelState()
  small = SmallModel(inner)
  joining = FakeJoining(small)
  return inner, small, joining, Joined(joining, small)


def frame_args(inner):
  bufs = {inner.road_key: 'main', inner.wide_key: 'extra'}
  transforms = {inner.road_key: np.eye(3), inner.wide_key: np.eye(3) * 2}
  inputs = {'desire_pulse': np.zeros(8), 'traffic_convention': np.zeros(2), 'action_t': np.zeros(2, dtype=np.float32)}
  return bufs, transforms, inputs


def test_adapter_conforms():
  assert conformance(jetlink_adapter.Adapter(), Openpilot) == []


def test_in_control_reads_carcontrol():
  class CC:
    enabled = latActive = longActive = False

  class SM(dict):
    def __init__(self, alive, cc):
      super().__init__(carControl=cc)
      self.alive = alive

    def all_alive(self, _):
      return self.alive

    def all_valid(self, _):
      return self.alive

  assert jetlink_adapter.in_control(SM(False, CC())) is True
  assert jetlink_adapter.in_control(SM(True, CC())) is False
  for name in ('enabled', 'latActive', 'longActive'):
    cc = CC()
    setattr(cc, name, True)
    assert jetlink_adapter.in_control(SM(True, cc)) is True


def test_small_model_runs_on_its_own_keys():
  inner, _, _, joined = make()
  bufs, transforms, inputs = frame_args(inner)
  sync = object()
  assert joined.run(bufs, transforms, inputs, True, after_output_sync=sync, blinker_on=True) == {'ok': True}
  run_bufs, run_tfms, run_inputs, prepare_only, after, blinker = inner.calls[-1]
  assert set(run_bufs) == {inner.road_key, inner.wide_key} == set(run_tfms)
  assert run_bufs[inner.road_key] == 'main' and run_bufs[inner.wide_key] == 'extra'
  assert prepare_only is True and after is sync and blinker is True
  assert run_inputs is inputs


def test_flags_follow_the_driving_model():
  _, _, joining, joined = make()
  assert joined.can_prepare_only is True and joined.last_warp_output == 'warp'
  joining.chestnut = True
  assert joined.big
  assert (joined.is_v9, joined.is_v14, joined.is_v15, joined.is_v16, joined.mlsim) == (False, False, False, True, True)
  assert joined.can_prepare_only is False and joined.last_warp_output is None and joined.uses_external_gpu is False
  assert joined.model_id == 'rdf43' and joined.policy_generation == 'v16'
  joining.output_slices = {'plan': slice(0, 8)}
  assert joined.is_v16 is False and joined.mlsim is True


def test_big_flags():
  assert big_flags({'action': 0})['is_v16'] and not big_flags({'plan': 0})['is_v16']
  assert all(big_flags(s)['mlsim'] for s in ({'action': 0}, {'plan': 0}))


def test_writes_reach_the_joining_model():
  _, _, joining, joined = make()
  joined.in_control = False
  joined.frame_drop_ratio = 0.25
  assert joining.in_control is False and joining.frame_drop_ratio == 0.25
  assert joined.handovers == 0


def _bundle(ref, name, index, selector=19):
  return {'ref': ref, 'display_name': name, 'index': index, 'minimum_selector_version': str(selector), 'is_big': True}


def test_model_picker_lists_and_selects(monkeypatch):
  from openpilot.common.params import Params
  params = Params()
  a, b, old = 'a' * 40, 'b' * 40, 'c' * 40
  params.put(jetlink_adapter.KEYS.catalog, {'bundles': [_bundle(a, 'Older', 1), _bundle(b, 'Newer', 2), _bundle(old, 'Other runtime', 3, 14),
                                                        {'ref': 'not-a-ref', 'index': 4, 'minimum_selector_version': '19'}]})
  params.remove(jetlink_adapter.KEYS.big_model)
  rows = jetlink_adapter.models()
  assert [r['ref'] for r in rows] == [b, a] and [r['name'] for r in rows] == ['Newer', 'Older']
  assert not any(r['selected'] for r in rows)

  assert jetlink_adapter.select_model(a) is True
  assert params.get(jetlink_adapter.KEYS.big_model) == {'ref': a, 'displayName': 'Older'}
  assert [r['selected'] for r in jetlink_adapter.models()] == [False, True]
  assert jetlink_adapter.select_model(old) is False   # not listed at this selector
  assert jetlink_adapter.select_model(None) is True
  assert params.get(jetlink_adapter.KEYS.big_model) is None


def test_refresh_catalog_keeps_the_last_on_failure(monkeypatch):
  from openpilot.common.params import Params
  params = Params()
  cached = {'bundles': [_bundle('d' * 40, 'Cached', 1)]}
  params.put(jetlink_adapter.KEYS.catalog, cached)
  monkeypatch.setattr(jetlink_adapter, 'should_extend_catalog', lambda: True)
  monkeypatch.setattr(jetlink_adapter, 'extend_catalog', lambda catalog: catalog)   # a failed fetch keeps what it had
  assert jetlink_adapter.refresh_catalog() is False and params.get(jetlink_adapter.KEYS.catalog) == cached
  fresh = {'bundles': [*cached['bundles'], _bundle('e' * 40, 'Fresh', 2)]}
  monkeypatch.setattr(jetlink_adapter, 'extend_catalog', lambda catalog: fresh)
  assert jetlink_adapter.refresh_catalog() is True and params.get(jetlink_adapter.KEYS.catalog) == fresh
  monkeypatch.setattr(jetlink_adapter, 'should_extend_catalog', lambda: False)   # a chestnut: StarPilot's own big models
  assert jetlink_adapter.refresh_catalog() is False
