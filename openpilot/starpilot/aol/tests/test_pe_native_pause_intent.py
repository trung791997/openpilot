"""PE independent LAT intent survives only qualified native withdrawals."""
from contextlib import ExitStack

from openpilot.common.prefix import OpenpilotPrefix
from openpilot.starpilot.aol.tests import test_ioniq_availability as fixture


def state(*, gas=False, eps=False):
  cs = fixture.car_state()
  cs.vEgoRaw = cs.vEgo
  cs.gasPressed = gas
  cs.steerFaultTemporary = eps
  cs.cruiseState.available = True
  return cs


def setup_case(cleanup):
  case = fixture.IoniqAvailabilityTests(methodName='runTest')
  cleanup.callback(case.doCleanups)
  cleanup.enter_context(OpenpilotPrefix())
  from opendbc.car.hyundai.tests.test_ioniq5pe_pending_startup import TestPEPendingStartup
  startup = TestPEPendingStartup(methodName='runTest')
  cleanup.callback(startup.doCleanups)
  startup.setUp()
  owner, ci, _ = startup.instance(seal=False)
  cleanup.callback(owner.close)
  sd, controls, drive, card, receive = case._ioniq_availability_callers(full_op=False, cp=ci.CP, startup_owner=owner)
  assert sd.CP.safetyConfigs[0].safetyParam == 0x5495
  for tick in range(2, 152):
    axis, _, command = drive(tick, native_request_long=False, gas=tick >= 150, observed_state=state(gas=tick >= 150))
    assert axis.lateralActive and command.latActive and not command.longActive
    assert receive(10_000_000_000 + tick * 10_000_000, axis=sd.pm.messages['aolAxisState'],
                   event=sd.pm.messages['onroadEvents'], state=state(gas=tick >= 150))
  return sd, controls, drive, card, receive


def test_pe_native_first_eps_preserves_fault_identity_and_source_vetoes():
  from dataclasses import replace
  from openpilot.cereal import log
  for fault in ('native_invalid', 'rx_invalid', 'native_fault', 'permanent_eps', 'can_invalid', 'can_timeout',
                'producer', 'epoch', 'session', 'fault_session', 'source_reordered', 'hard_event', 'settings_disabled'):
    with ExitStack() as cleanup:
      sd, _, drive, card, receive = setup_case(cleanup)
      from openpilot.cereal import messaging
      pandas = None
      if fault == 'native_fault':
        pandas = messaging.new_message('pandaStates', len(sd.sm['pandaStates']), valid=True, logMonoTime=11_520_000_000)
        pandas.pandaStates = sd.sm['pandaStates']
        pandas.pandaStates[0].faults = ['interruptRateCan1']
      drive(152, gas=True, observed_state=state(gas=True), allowed=False, native_request_lat=True,
            panda_state_message=pandas,
            native_request_long=False, native_valid=fault != 'native_invalid',
            panda_fault=fault == 'rx_invalid',
            critical_event=log.OnroadEvent.EventName.canError if fault == 'hard_event' else None)
      proof = sd.pm.messages['aolAxisState'].as_builder()
      eps = state(gas=True, eps=True)
      if fault == 'permanent_eps':
        eps.steerFaultPermanent = True
      elif fault == 'can_invalid':
        eps.canValid = False
      elif fault == 'can_timeout':
        eps.canTimeout = True
      elif fault == 'producer':
        proof.aolAxisState.intentProducerSessionId = 'other-card'
      elif fault == 'epoch':
        proof.aolAxisState.intentIntegrityEpoch += 1
      elif fault == 'session':
        proof.aolAxisState.sessionId = 'other-session'
      elif fault == 'fault_session':
        proof.aolAxisState.faultSessionId = 'other-session'
      elif fault == 'source_reordered':
        proof.aolAxisState.sourceCarStateMonoTime = card.aol_transport_feedback.lateral_source_ns - 1
      elif fault == 'settings_disabled':
        card.aol_card_intent.settings = replace(card.aol_card_intent.settings, enabled=False)
      assert not receive(11_520_000_000, axis=proof.as_reader(),
                         event=sd.pm.messages['onroadEvents'], state=eps), fault
      assert not card.aol_card_intent.allowed_latch, fault
