"""Ford native brake ordering with steering already paused; no denied output."""
from contextlib import ExitStack
from unittest.mock import patch

import pytest
from opendbc.car.ford.interface import CarInterface
from opendbc.car.ford.values import CAR
from openpilot.cereal import log
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.common.realtime import DT_CTRL
from openpilot.starpilot.aol.tests import test_ioniq_availability as fixture


def callers(cleanup):
  cleanup.enter_context(OpenpilotPrefix())
  cleanup.enter_context(patch.dict('os.environ', {'SIMULATION': '0', 'REPLAY': '0'}))
  cp = CarInterface.get_params(CAR.FORD_MUSTANG_MACH_E_MK1, {0: {}, 1: {}, 2: {}}, [], True, False, False)
  case = fixture.IoniqAvailabilityTests(methodName='runTest')
  cleanup.callback(case.doCleanups)
  sd, controls, step = case._authority_callers(True, cp=cp, genuine_events=True)
  def drive(tick, **options):
    state = fixture.car_state(brake=options.get('brake', False))
    state.cruiseState.available = state.cruiseState.enabled = True
    state.cruiseState.speed = state.cruiseState.speedCluster = 25.
    return step(tick, observed_state=state, **options)
  for tick in range(10):
    axis, host, command = drive(tick, pause=True, latched=False, allowed=False,
      native_request_lat=False, native_request_long=True)
    assert host.enabled and command.longActive and not command.latActive
    assert axis.longitudinalActive and not axis.lateralActive
  return sd, controls, drive


@pytest.mark.parametrize('fault', ('native_invalid', 'rx_fault', 'relay_fault', 'proof_missing', 'expired', 'lat_requested'))
def test_ford_long_only_health_and_proof_refusals_remain_immediate(fault):
  with ExitStack() as cleanup:
    sd, _, drive = callers(cleanup)
    options = {'pause': True, 'latched': False, 'allowed': False, 'native_request_lat': False,
      'native_request_long': True, 'long_allowed': False, 'panda_controls_allowed': False}
    if fault == 'native_invalid':
      options['native_valid'] = False
    elif fault == 'rx_fault':
      options['panda_fault'] = True
    elif fault == 'relay_fault':
      options['relay_fault'] = True
    elif fault == 'proof_missing':
      sd.aol_published_input_proof = None
    elif fault == 'expired':
      options['offset_ns'] = 200_000_000
    else:
      options['native_request_lat'] = True
    axis, host, command = drive(10, **options)
    assert log.OnroadEvent.EventName.controlsMismatch in sd.events.names
    assert not host.enabled and not command.latActive and not command.longActive
    assert not axis.lateralActive and not axis.longitudinalActive


@pytest.mark.parametrize('timer', (40, 0))
def test_started_or_expired_countdown_keeps_immediate_takeover(timer):
  with ExitStack() as cleanup:
    sd, _, _ = callers(cleanup)
    now = sd.aol_car_state_log_ns + 30_000_001
    with patch('openpilot.selfdrive.selfdrived.selfdrived.time.monotonic_ns', return_value=now):
      assert sd.retain_ordinary_source_state(sd.CS_prev, now)
      sd.state_machine.state = log.SelfdriveState.OpenpilotState.softDisabling
      sd.state_machine.soft_disable_timer = timer
      sd.update_alerts(sd.CS_prev)
    assert sd.state_machine.soft_disable_timer == timer
    assert sd.AM.current_alert.alert_status == log.SelfdriveState.AlertStatus.critical
    assert sd.AM.current_alert.audible_alert == log.SelfdriveState.AudibleAlert.warningImmediate
