from contextlib import ExitStack
from unittest import mock

from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.values import CAR

import pytest

from openpilot.cereal import log, messaging
from openpilot.starpilot.aol.wire import encode_intent
from openpilot.starpilot.aol.tests.test_stock_location_family_intent import stock_family_callers


def callers(cleanup, family):
  if family != 'ev6_camera':
    return stock_family_callers(cleanup, family)
  factory = CarInterface.get_params

  def ev6_params(identity, *args, **kwargs):
    return factory(CAR.KIA_EV6 if identity == CAR.KIA_NIRO_EV_2ND_GEN else identity, *args, **kwargs)

  with mock.patch.object(CarInterface, 'get_params', side_effect=ev6_params):
    result = stock_family_callers(cleanup, 'canfd_camera')
  assert result[0].CP.carFingerprint == CAR.KIA_EV6
  assert result[0].CP.safetyConfigs[-1].safetyParam == 0x809
  return result


@pytest.mark.parametrize('fault', ['requested_denial', 'receive_checks', 'relay', 'interrupt_rate',
                                  'native_session', 'native_invalid', 'producer', 'stale_baseline',
                                  'permanent_eps', 'invalid_can'])
def test_source_expiry_does_not_hide_faults_or_changed_provenance(fault):
  from dataclasses import replace
  from openpilot.starpilot.aol.runtime import current_native
  from openpilot.starpilot.aol.wire import encode_safety

  with ExitStack() as cleanup:
    sd, drive, card, receive, state = callers(cleanup, 'ev6_camera')
    sd.aol_settings = card.aol_card_intent.settings
    for tick in range(2, 5):
      axis, _, command = drive(tick, observed_state=state, gas=True,
        native_request_long=False, panda_controls_allowed=False)
      assert axis.lateralActive and command.latActive
      assert receive(axis.observedMonoTime, state=state,
        axis=sd.pm.messages['aolAxisState'], event=sd.pm.messages['onroadEvents'])
    old_native = current_native(sd.sm, sd.CP, now_ns=axis.observedMonoTime, axis_session_id=sd.aol_session_id)
    assert old_native is not None
    intent = sd.aol_last_intent
    if fault == 'producer':
      intent = replace(intent, producerSessionId='changed-producer')
    held = messaging.new_message('aolIntentWire', 0, valid=True, logMonoTime=intent.carStateLogMonoTime)
    held.aolIntentWire = encode_intent(intent)
    authority = messaging.new_message('starpilotCarState', valid=True, logMonoTime=sd.sm.logMonoTime['starpilotCarState'])
    authority.starpilotCarState = sd.sm['starpilotCarState']
    tick = 30 if fault == 'stale_baseline' else 10
    now = axis.observedMonoTime + (tick - 4) * 10_000_000
    faults = ['interruptRateCan1'] if fault == 'interrupt_rate' else ['relayMalfunction'] if fault == 'relay' else []
    mask = sum(1 << log.PandaState.FaultType.schema.enumerants[name] for name in faults)
    native = replace(old_native, observedMonoTime=now, validUntilMonoTime=now + 200_000_000,
      sourcePandaStatesMonoTime=now, requestedLateral=fault == 'requested_denial', lateralAllowed=False,
      requestedLongitudinal=False, longitudinalAllowed=False,
      axisSessionId='changed-native-session' if fault == 'native_session' else old_native.axisSessionId,
      pandaInventory=tuple(replace(slot, faults=mask, safetyRxChecksInvalid=fault == 'receive_checks')
                           for slot in old_native.pandaInventory))
    wire = messaging.new_message('aolSafetyWire', 0, valid=fault != 'native_invalid', logMonoTime=now)
    wire.aolSafetyWire = encode_safety(native)
    pandas = messaging.new_message('pandaStates', len(sd.CP.safetyConfigs), valid=True, logMonoTime=now)
    pandas.pandaStates = [panda.to_dict() for panda in sd.sm['pandaStates']]
    for panda in pandas.pandaStates:
      panda.faults = faults
      panda.safetyRxChecksInvalid = fault == 'receive_checks'
    state.steerFaultPermanent = fault == 'permanent_eps'
    state.canValid = fault != 'invalid_can'
    paused, _, command = drive(tick, timeout=True, observed_state=state, gas=True, intent_message=held,
      authority_message=authority, safety_message=wire, panda_state_message=pandas,
      native_request_lat=False, allowed=False, native_request_long=False, panda_controls_allowed=False)
    assert not paused.lateralActive and not paused.longitudinalActive
    assert not command.latActive and not command.longActive
    assert not receive(now, state=state, axis=sd.pm.messages['aolAxisState'], event=sd.pm.messages['onroadEvents'])
    assert not card.aol_card_intent.allowed_latch
