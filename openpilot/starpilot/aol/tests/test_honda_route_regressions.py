"""Civic route regressions: zero brake pause, EPS intent and native receipt order."""
from dataclasses import replace
from types import SimpleNamespace as NS
from typing import cast

import pytest

from opendbc.car import structs
from opendbc.car.honda.tests.test_stock_aol import final_cp
from opendbc.car.honda.values import CAR
from openpilot.cereal import log, messaging
from openpilot.selfdrive.selfdrived.events import Events
from openpilot.selfdrive.selfdrived.selfdrived import SelfdriveD
from openpilot.selfdrive.car.card import Car
from openpilot.starpilot.aol.transport_pause import TransportPauseFeedback
from openpilot.starpilot.aol.intent import AolSettings
from openpilot.starpilot.tests.test_ford_reverse_intent import NOW, PRODUCER, change, native_change, sample


def civic():
  cp = final_cp(CAR.HONDA_CIVIC_2022, alpha=True, hybrid=True)
  cp.alternativeExperience = 32
  return cp


class Peers(dict):
  def all_checks(self, _services):
    return True


@pytest.mark.parametrize('speed', [-0.0142488591, -0.0008664753, 0.0, 0.002777777845])
@pytest.mark.parametrize('threshold', [0.0, 1.0])
@pytest.mark.parametrize('grant', [False, True])
@pytest.mark.parametrize('history_only', [False, True])
def test_actual_caller_zero_brake_pause_matches_current_and_queued_samples(speed, threshold, grant, history_only):
  sd = SelfdriveD.__new__(SelfdriveD)
  sd.CP = civic()
  sd.sm = cast(messaging.SubMaster, Peers(driverMonitoringState=log.DriverMonitoringState.new_message(),
                extrinsicsCalibration=log.ExtrinsicsCalibration.new_message(calStatus='calibrated')))
  sd.enabled = sd.active = sd.aol_dm_lateral_inhibit = sd.nostalgia_paddle_cancel = False
  sd.initialized = True
  sd.events = Events()
  sd.aol_settings = AolSettings(True, threshold, 9, 0, (0, 0, 0), (0, 0, 0))
  state = structs.CarState(canValid=True, brakePressed=True, vEgo=speed, standstill=False, gearShifter='drive')
  current = structs.CarState(**state.to_dict())
  if history_only:
    current.standstill = True  # The latest sample must not erase an earlier valid veto.
  sd.car_state_samples = ((state, True), (current, True))
  intent = NS(allowedLatch=True, pauseLateral=False, pauseLongitudinal=True, optionalSetRelease=False)
  native = NS(requestedLateral=True, lateralAllowed=grant, requestedLongitudinal=False, longitudinalAllowed=False)
  result = sd._decide_aol_axes(current, intent, native, now_ns=NOW)
  assert result.desired_lateral == (threshold == 0.0)
  assert result.lateral_active == (threshold == 0.0 and grant)
  assert not result.desired_longitudinal and not result.longitudinal_active
  state.steerFaultTemporary = True
  result = sd._decide_aol_axes(current, intent, native, now_ns=NOW)
  assert not result.desired_lateral and not result.lateral_active


def baseline():
  cp = civic()
  state = structs.CarState(canValid=True, gearShifter='drive', vEgo=15.)
  state.cruiseState.available = True
  feedback = TransportPauseFeedback(cp)
  sm, native = sample(cp, NOW, NOW, 1)
  change(sm, 'onroadEvents', lambda events: setattr(events[0], 'userDisable', False))
  assert not feedback.observe(sm, cp, state, NOW, latched=True, intent_producer_session=PRODUCER)
  assert feedback.lateral_only_baseline
  return cp, state, feedback, sm, native


def eps_sample(cp, stamp, *, withdrawn, brake=False, gas=False):
  sm, native = sample(cp, stamp, stamp, 2, active=not withdrawn, allowed=False,
                      event_stamp=stamp if withdrawn else NOW)
  event = sm.messages['onroadEvents'].as_builder()
  event.onroadEvents = [{'name': 'steerOverride', 'overrideLateral': True}]
  if brake:
    event.onroadEvents = [*(e.to_dict() for e in event.onroadEvents), {'name': 'pedalPressed', 'userDisable': True}]
  if gas:
    event.onroadEvents = [*(e.to_dict() for e in event.onroadEvents), {'name': 'gasPressedOverride', 'overrideLongitudinal': True}]
  sm.messages['onroadEvents'] = event.as_reader()
  return sm, native


@pytest.mark.parametrize('fault', ['unarmed', 'no_baseline', 'no_eps', 'permanent_eps', 'can_invalid', 'timeout',
  'door', 'sensors', 'disengage', 'main_off', 'acc_fault', 'cancel', 'reverse', 'wrong_profile', 'wrong_session',
  'wrong_producer', 'wrong_epoch', 'expired_axis', 'native_missing', 'native_invalid', 'native_stale',
  'native_fault', 'native_heartbeat', 'long_request', 'long_desired', 'long_active', 'hard_event', 'other_user'])
def test_honda_eps_intent_retention_preserves_independent_vetoes(fault):
  cp, state, feedback, _, _ = baseline()
  state.steerFaultTemporary = state.steeringPressed = state.brakePressed = True
  now = NOW + 10_000_000
  sm, native = eps_sample(cp, now, withdrawn=True, brake=True)
  fields = {'no_eps': ('steerFaultTemporary', False), 'permanent_eps': ('steerFaultPermanent', True),
    'can_invalid': ('canValid', False), 'timeout': ('canTimeout', True), 'door': ('doorOpen', True),
    'sensors': ('vehicleSensorsInvalid', True), 'disengage': ('steeringDisengage', True), 'acc_fault': ('accFaulted', True)}
  if fault in fields:
    setattr(state, *fields[fault])
  elif fault == 'no_baseline':
    feedback.lateral_baseline_ns = 0
  elif fault == 'main_off':
    state.cruiseState.available = False
  elif fault == 'cancel':
    state.buttonEvents = [structs.CarState.ButtonEvent(type='cancel', pressed=True)]
  elif fault == 'reverse':
    state.gearShifter = 'reverse'
  elif fault == 'wrong_profile':
    cp.alternativeExperience = 0
  elif fault == 'wrong_session':
    native_change(sm, native, axisSessionId='different')
  elif fault == 'wrong_epoch':
    change(sm, 'aolAxisState', lambda axis: setattr(axis, 'intentIntegrityEpoch', 2))
  elif fault == 'expired_axis':
    change(sm, 'aolAxisState', lambda axis: setattr(axis, 'validUntilMonoTime', now - 1))
  elif fault in ('long_desired', 'long_active'):
    change(sm, 'aolAxisState', lambda axis: setattr(axis, 'desiredLongitudinal' if fault == 'long_desired' else 'longitudinalActive', True))
  elif fault == 'long_request':
    native_change(sm, native, requestedLongitudinal=True)
  elif fault == 'native_missing':
    sm.seen['aolSafetyWire'] = False
  elif fault == 'native_invalid':
    sm.valid['aolSafetyWire'] = False
  elif fault == 'native_stale':
    native_change(sm, native, validUntilMonoTime=now - 1)
  elif fault in ('native_fault', 'native_heartbeat'):
    native_change(sm, native, pandaInventory=(replace(native.pandaInventory[0],
      **({'faults': 1} if fault == 'native_fault' else {'heartbeatLost': True})),))
  elif fault == 'hard_event':
    change(sm, 'onroadEvents', lambda events: setattr(events[0], 'immediateDisable', True))
  elif fault == 'other_user':
    change(sm, 'onroadEvents', lambda events: setattr(events[1], 'name', 'buttonCancel'))
  rejected = feedback.observe(sm, cp, state, now, latched=fault != 'unarmed',
    intent_producer_session='different' if fault == 'wrong_producer' else PRODUCER)
  if fault != 'unarmed':
    assert rejected
  assert not feedback.temporary_host_qualified


