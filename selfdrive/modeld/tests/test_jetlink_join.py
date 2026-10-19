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
  assert conformance(jetlink_adapter.adapter(), Openpilot) == []


class FakeMsg:
  def __init__(self, **fields):
    for name, value in fields.items():
      setattr(self, name, value)


class FakeSubMaster(dict):
  """cereal's SubMaster as Stock.in_control() and the adapter's AOL reader use it."""
  def __init__(self, services, alive=True, update_error=None, **msgs):
    super().__init__(msgs)
    self.services = services
    self.ok = alive
    self.update_error = update_error
    self.updates = []
    self.seen = {s: s in msgs for s in services}
    self.alive = {s: alive for s in services}

  def update(self, timeout):
    self.updates.append(timeout)
    if self.update_error is not None:
      raise self.update_error

  def all_alive(self, _):
    return self.ok

  def all_valid(self, _):
    return self.ok


def _engaged_state(**active):
  cc = FakeMsg(enabled=False, latActive=False, longActive=False)
  ss = FakeMsg(enabled=False, active=False)
  for name, value in active.items():
    setattr(ss if name.startswith('ss_') else cc, name.removeprefix('ss_'), value)
  return {'carControl': cc, 'selfdriveState': ss}


def _aol(allowed=False, enabled=False):
  return FakeMsg(alwaysOnLateralAllowed=allowed, alwaysOnLateralEnabled=enabled)


def test_in_control_is_openpilot_or_aol():
  # the PR's rule: openpilot engaged, or AOL armed (paused included) or engaged
  adapter = jetlink_adapter.adapter()
  adapter._sm = sm = FakeSubMaster(adapter.IN_CONTROL_SERVICES, **_engaged_state())
  adapter._aol = aol = FakeSubMaster([jetlink_adapter.AOL_SERVICE], starpilotCarState=_aol())
  assert adapter.in_control() is False
  assert sm.updates == [0] and aol.updates == [0]   # polled, never waits on modeld's frame thread
  for name in ('enabled', 'latActive', 'longActive', 'ss_enabled', 'ss_active'):
    dict.update(sm, _engaged_state(**{name: True}))
    assert adapter.in_control() is True, name
  dict.update(sm, _engaged_state())
  for allowed, enabled in ((True, False), (False, True), (True, True)):
    aol['starpilotCarState'] = _aol(allowed, enabled)
    assert adapter.in_control() is True
  aol['starpilotCarState'] = _aol()
  assert adapter.in_control() is False
  aol.alive[jetlink_adapter.AOL_SERVICE] = False   # AOL's state gone stale: in control
  assert adapter.in_control() is True


def test_aol_not_heard_yet_waits_its_grace():
  assert jetlink_adapter.aol_in_control(None, False, waiting=True) is True
  assert jetlink_adapter.aol_in_control(None, False, waiting=False) is False
  assert jetlink_adapter.aol_in_control(_aol(), False) is True
  assert jetlink_adapter.aol_in_control(_aol(), True) is False


def test_in_control_fails_closed(monkeypatch):
  import sys
  import types

  adapter = jetlink_adapter.adapter()
  adapter._sm = FakeSubMaster(adapter.IN_CONTROL_SERVICES, update_error=RuntimeError('socket'), **_engaged_state())
  assert adapter.in_control() is True
  adapter._sm = FakeSubMaster(adapter.IN_CONTROL_SERVICES, alive=False, **_engaged_state())   # late or invalid
  assert adapter.in_control() is True

  # the SubMaster cannot be made: in control, and not tried again on every frame
  def broken(services):
    raise OSError('no msgq')
  fake = types.ModuleType('cereal.messaging')
  fake.SubMaster = broken
  cereal = types.ModuleType('cereal')
  cereal.messaging = fake
  monkeypatch.setitem(sys.modules, 'cereal', cereal)
  monkeypatch.setitem(sys.modules, 'cereal.messaging', fake)
  adapter = jetlink_adapter.adapter()
  assert adapter.in_control() is True and adapter._sm is None
  made = []
  fake.SubMaster = lambda services: made.append(tuple(services)) or FakeSubMaster(services, **_engaged_state())
  assert adapter.in_control() is True and made == []   # inside the retry wait
  adapter._sm_after = 0.
  # the AOL reader is made from the same fake: not heard yet, inside its grace, counts as armed
  assert adapter.in_control() is True
  assert made[0] == tuple(adapter.IN_CONTROL_SERVICES) and made[1] == (jetlink_adapter.AOL_SERVICE,)
  adapter._aol_since = -10.
  assert adapter.in_control() is False and len(made) == 2   # each made once


def test_hooks_fall_back_to_link_off(monkeypatch):
  # no usable jetlink, or an unsupported device: every hook answers link off and modeld runs the small model
  import builtins
  real_import = builtins.__import__

  def no_jetlink(name, *args, **kwargs):
    if name.startswith('jetlink'):
      raise ImportError('no jetlink')
    return real_import(name, *args, **kwargs)
  jetlink_adapter.hooks.cache_clear()
  monkeypatch.setattr(builtins, '__import__', no_jetlink)
  try:
    link = jetlink_adapter.hooks()
  finally:
    monkeypatch.setattr(builtins, '__import__', real_import)
    jetlink_adapter.hooks.cache_clear()
  assert isinstance(link, jetlink_adapter._NoJetlink)
  assert link.prepare(False) is False and link.should_run(True, None, None) is False and link.swap_events(0.01) is None
  assert link.shutdown_step(True) is True and link.status() is None

  from jetlink.openpilot.hooks import Off
  monkeypatch.setattr(jetlink_adapter, 'supported_device', lambda device_type=None: False)
  try:
    link = jetlink_adapter.hooks()
    assert isinstance(link, Off) and link.prepare(False) is False
    small = object()
    assert link.attach(None, small, 1928, 1208) is None   # not joined: modeld keeps the small model
  finally:
    jetlink_adapter.hooks.cache_clear()


def test_small_model_is_stock_modeld_to_prepare_reset():
  inner, small, _, _ = make()
  assert small.adapter is None
  # the names v0.8.7's prepare_reset reads; their contents are test_jetlink_reset's
  assert isinstance(SmallModel.state_pairs, property) and hasattr(small.packed_input, 'fill')


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
  link = FakeMsg(should_extend_catalog=lambda: True, extend_catalog=lambda catalog: catalog)   # a failed fetch keeps what it had
  monkeypatch.setattr(jetlink_adapter, 'hooks', lambda: link)
  assert jetlink_adapter.refresh_catalog() is False and params.get(jetlink_adapter.KEYS.catalog) == cached
  fresh = {'bundles': [*cached['bundles'], _bundle('e' * 40, 'Fresh', 2)]}
  link.extend_catalog = lambda catalog: fresh
  assert jetlink_adapter.refresh_catalog() is True and params.get(jetlink_adapter.KEYS.catalog) == fresh
  link.should_extend_catalog = lambda: False   # a chestnut: StarPilot's own big models
  assert jetlink_adapter.refresh_catalog() is False
