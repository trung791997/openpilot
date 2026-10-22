from types import SimpleNamespace as NS

import pytest

from opendbc.car.structs import car
from openpilot.cereal import messaging
from openpilot.common.params import Params
from openpilot.selfdrive.car.card import Car
from openpilot.selfdrive.car.tests.publication_fixture import initialize_publication_sources
from openpilot.selfdrive.car.cruise import CRUISE_LONG_PRESS, VCruiseHelper
from openpilot.selfdrive.car.tests.test_hyundai_aol import candidate
from openpilot.starpilot.controllers.wheel_actions import ACTIONS, KEYS, WheelConsumer, WheelPublisher, capture


TYPE = car.CarState.ButtonEvent.Type
DRIVE = 1_000_000_000
NOW = DRIVE + 2_000_000_000


def state(*pressed, button=TYPE.gapAdjustCruise):
  return car.CarState(canValid=True, buttonEvents=[car.CarState.ButtonEvent(type=button, pressed=value) for value in pressed])


class Publisher:
  def __init__(self):
    self.events = []

  def send(self, service, event):
    self.events.append((service, event.to_bytes()))


def services(cp, now=NOW):
  names = ('deviceState', 'carState', 'carControl')
  data = {'deviceState': NS(started=True, startedMonoTime=DRIVE), 'carState': state(),
          'carControl': car.CarControl(enabled=True, longActive=True)}
  class Services(dict):
    pass
  sm = Services(data)
  sm.logMonoTime = dict.fromkeys(names, now)
  sm.recv_time = dict.fromkeys(names, now / 1e9)
  sm.seen = sm.alive = sm.valid = dict.fromkeys(names, True)
  sm.frame = 1
  sm.all_checks = lambda _: True
  return sm


@pytest.fixture
def context(tmp_path):
  params = Params(str(tmp_path))
  _, cp = candidate(False)
  return params, cp


def test_exact_catalog_and_missing_vs_saved_off(context):
  params, _ = context
  assert sorted(ACTIONS.values()) == list(range(15))
  assert len(KEYS) == 14
  assert capture(params)['DistanceButtonControl'] == 1
  assert capture(params)['LongDistanceButtonControl'] == 5
  assert capture(params)['VeryLongDistanceButtonControl'] == 6
  params.put('DistanceButtonControl', 0, block=True)
  assert capture(params)['DistanceButtonControl'] == 0


@pytest.mark.parametrize('key,code', [('DistanceButtonControl', code) for code in (1, 2, 5, 6, 7, 8, 11, 12, 13, 14)])
def test_real_distance_short_delivers_once_and_claims_serialized_release(context, key, code):
  params, cp = context
  params.put(key, code, block=True)
  owner = WheelPublisher()
  owner.observe(params, cp, state(), now_ns=NOW, drive_id=DRIVE)
  assert not any(action for _, action in owner.observe(params, cp, state(True), now_ns=NOW + 10_000_000, drive_id=DRIVE))
  commands = owner.observe(params, cp, state(False), now_ns=NOW + 20_000_000, drive_id=DRIVE)
  assert commands == ((key, code),)
  assert owner.suppress_distance_release
  assert not any(action for _, action in owner.observe(params, cp, state(), now_ns=NOW + 30_000_000, drive_id=DRIVE))


def test_held_long_then_very_long_exactly_once_no_short_release(context):
  params, cp = context
  owner = WheelPublisher()
  owner.observe(params, cp, state(), now_ns=NOW, drive_id=DRIVE)
  observed = []
  for frame in range(1, CRUISE_LONG_PRESS * 5 + 3):
    commands = owner.observe(params, cp, state(True) if frame == 1 else state(),
                             now_ns=NOW + frame * 10_000_000, drive_id=DRIVE)
    observed.extend((frame, key, action) for key, action in commands if action)
  assert observed == [(CRUISE_LONG_PRESS, 'LongDistanceButtonControl', 5),
                      (CRUISE_LONG_PRESS * 5, 'VeryLongDistanceButtonControl', 6)]
  assert not any(action for _, action in owner.observe(params, cp, state(False),
                now_ns=NOW + (CRUISE_LONG_PRESS * 5 + 4) * 10_000_000, drive_id=DRIVE))


def test_published_receipt_exact_cp_session_replay_and_delayed_source(context):
  params, cp = context
  params.put('DistanceButtonControl', 8, block=True)
  publisher = Publisher()
  owner = WheelPublisher()
  owner.publish((('DistanceButtonControl', 8),), cp, publisher, now_ns=NOW, drive_id=DRIVE,
                source_car_ns=NOW, source_control_ns=NOW)
  event = messaging.log_from_bytes(publisher.events[0][1])
  sm = services(cp)
  consumer = WheelConsumer()
  assert consumer.accept(event, params, cp, sm, now_ns=NOW).action == 8
  assert consumer.accept(event, params, cp, sm, now_ns=NOW) is None
  changed = cp.as_reader().as_builder()
  changed.pcmCruise = not cp.pcmCruise
  assert WheelConsumer().accept(event, params, changed.as_reader(), sm, now_ns=NOW) is None
  sm.recv_time['carState'] = (NOW - 300_000_000) / 1e9
  assert WheelConsumer().accept(event, params, cp, sm, now_ns=NOW) is None


def test_actual_card_published_distance_off_cannot_trigger_unconditional_personality(context):
  params, cp = context
  params.put('DistanceButtonControl', 0, block=True)
  owner = WheelPublisher()
  sm = services(cp)
  owner.observe(params, cp, state(), now_ns=NOW, drive_id=DRIVE)
  owner.observe(params, cp, state(True), now_ns=NOW + 10_000_000, drive_id=DRIVE)
  physical = state(False)
  commands = owner.observe(params, cp, physical, now_ns=NOW + 20_000_000, drive_id=DRIVE)
  instance = Car.__new__(Car)
  instance.CP, instance.params, instance.sm = cp, params, sm
  vars(instance)['pm'] = Publisher()
  instance.wheel_publisher, instance.wheel_commands = owner, commands
  instance.slc_replay = instance.curve_replay = instance.conditional_replay = instance.aol_replay = False
  instance.last_actuators_output = car.CarControl.Actuators()
  instance.can_rcv_cum_timeout_counter = 0
  vars(instance)['rk'] = NS(remaining=0.0)
  instance.v_cruise_helper = VCruiseHelper(cp)
  instance.slc_receipts = []
  initialize_publication_sources(instance)
  instance.state_publish(physical, None)
  serialized = next(messaging.log_from_bytes(raw).carState for name, raw in instance.pm.events if name == 'carState')
  assert len(serialized.buttonEvents) == 0
  assert len(physical.buttonEvents) == 1


@pytest.mark.parametrize('starpilot_planner', (False, True))
def test_distance_long_press_reaches_experimental_unless_starpilot_conditional_claims_it(context, monkeypatch, starpilot_planner):
  # 2026-10-22: with UseStarPilotLongitudinalPlanner=0 a configured CEM used to claim the
  # long press (Card blocked the key) although upstream plannerd never consumes conditionalMode.
  import json
  from openpilot.starpilot.conditional_mode.card_input import conditional_manual_candidate
  from openpilot.starpilot.conditional_mode.manual import ButtonTracker
  from openpilot.starpilot.conditional_mode.policy import ModeChoice
  from openpilot.starpilot.conditional_mode.preferences import SavedPreferences, encode_preferences
  from openpilot.starpilot.conditional_mode.runtime_settings import ConditionalSettingsOwner
  from openpilot.starpilot.controllers.wheel_actions import key_for
  from openpilot.starpilot.feature_runtime import card_conditional_runtime, enabled
  from openpilot.starpilot.ui import runtime_app
  params, cp = context
  assert cp.openpilotLongitudinalControl
  params.put('ConditionalModeConfig', json.loads(encode_preferences(SavedPreferences(mode=ModeChoice.CEM))), block=True)
  params.put('UseStarPilotLongitudinalPlanner', starpilot_planner, block=True)
  params.put('LongDistanceButtonControl', 5, block=True)  # Saved as on the device (Experimental).
  # Card, selfdrived and the UI all read the same planner-aware gate.
  gate = card_conditional_runtime(params, {})
  assert gate is starpilot_planner
  assert enabled(params, cp, 'conditional', {}) is starpilot_planner
  monkeypatch.setattr(runtime_app, 'ui_state', NS(params=params, CP=cp))
  session = runtime_app.StarShellSession.__new__(runtime_app.StarShellSession)
  assert session._conditional_favorite_active() is starpilot_planner
  # Mirror of Car.update(): the Conditional manual claim runs only when gated, then blocks its key.
  tracker, settings = (ButtonTracker(), ConditionalSettingsOwner(params)) if gate else (None, None)
  sm, owner = services(cp), WheelPublisher()
  receipts, commands = [], []
  for frame in range(CRUISE_LONG_PRESS + 3):
    now = NOW + frame * 10_000_000
    sm.logMonoTime = dict.fromkeys(sm.logMonoTime, now)
    physical = state(True) if frame == 1 else state()
    blocked = set()
    if gate:
      receipt = conditional_manual_candidate(physical, tracker, params, settings, cp, sm, now_ns=now)
      if receipt is not None:
        tracker.claim(receipt[0], receipt[1], receipt[2])
        receipts.append(receipt)
        blocked.add(key_for(receipt[0]))
    commands.extend(command for command in owner.observe(params, cp, physical, now_ns=now, drive_id=DRIVE,
                                                         blocked_keys=blocked) if command[1])
  if starpilot_planner:
    # Unchanged: StarPilot plannerd owns the CEM gesture (Card publishes a conditionalMode event for it).
    assert len(receipts) == 1 and receipts[0][3] == ModeChoice.CEM
    assert ('LongDistanceButtonControl', 5) not in commands
    return
  assert receipts == []  # No manual receipt -> Card publishes no conditionalMode event.
  assert commands == [('LongDistanceButtonControl', 5)]
  publisher = Publisher()
  owner.publish(tuple(commands), cp, publisher, now_ns=NOW, drive_id=DRIVE, source_car_ns=NOW, source_control_ns=NOW)
  kinds = [str(messaging.log_from_bytes(raw).slcCruiseEvent.kind) for _, raw in publisher.events]
  assert kinds == ['wheelAction']
  received = WheelConsumer().accept(messaging.log_from_bytes(publisher.events[0][1]), params, cp, services(cp), now_ns=NOW)
  assert (received.key, received.action) == ('LongDistanceButtonControl', 5)  # The UI's plain Experimental toggle.
