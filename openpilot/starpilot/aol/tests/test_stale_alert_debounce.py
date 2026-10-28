"""The red commIssue alert waits for 100 ms of stale carState; commands still go at 30 ms.

Route 0000003c--8a8cfed3f7 segment 9: one 28 ms pandad gap aged carState past the
30 ms intent bound and sounded commIssue/softDisable for a single frame.
"""
from contextlib import ExitStack
from unittest import mock

import pytest

from openpilot.cereal import log
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.selfdrive.selfdrived import selfdrived
from openpilot.starpilot.aol.tests import test_ioniq_availability as fixture
from openpilot.starpilot.aol.tests.test_input_deadline_continuity import BASE
from openpilot.starpilot.aol.tests.test_ordinary_source_availability import settings

COMM_ISSUE = log.OnroadEvent.EventName.commIssue
STATE = log.SelfdriveState.OpenpilotState


def expire(mode, offset_ns):
  case = fixture.IoniqAvailabilityTests(methodName='runTest')
  cleanup = ExitStack()
  cleanup.callback(case.doCleanups)
  cleanup.enter_context(OpenpilotPrefix())
  cleanup.enter_context(mock.patch.object(selfdrived, 'AOL_STALE_ALERT_NS', 100_000_000))
  sd, controls, drive, _, _ = case._ioniq_availability_callers(full_op=True)
  drive(2, state_stamp=BASE, offset_ns=4_497_535, **settings(mode, BASE, 3))
  axis, state, command = drive(5, timeout=True, state_stamp=BASE, offset_ns=offset_ns, **settings(mode, BASE, 3))
  return cleanup, sd, drive, axis, state, command


@pytest.mark.parametrize('mode', ['long_only', 'combined', 'combined_unarmed'])
def test_short_stale_gap_withdraws_commands_without_alert(mode):
  cleanup, sd, drive, axis, state, command = expire(mode, 5_399_000)  # carState 35 ms old
  with cleanup:
    assert not axis.lateralActive and not axis.longitudinalActive
    assert not command.latActive and not command.longActive
    assert COMM_ISSUE not in sd.events.names
    assert state.enabled and state.state == STATE.enabled and state.alertType == ''
    assert sd.aol_stale_alert_deferred and sd.aol_fault_reason == 'availabilityPause'
    fresh = BASE + 40_000_000
    recovered, recovered_state, recovered_command = drive(6, state_stamp=fresh, **settings(mode, fresh, 4))
    assert recovered.longitudinalActive and recovered_command.longActive
    assert recovered_state.enabled and recovered_state.state == STATE.enabled
    assert not sd.aol_stale_alert_deferred


@pytest.mark.parametrize('mode', ['long_only', 'combined', 'combined_unarmed'])
def test_stale_gap_past_100ms_still_sounds_soft_disable(mode):
  cleanup, sd, _, axis, state, command = expire(mode, 75_399_000)  # carState 105 ms old
  with cleanup:
    assert not axis.longitudinalActive and not command.longActive
    assert COMM_ISSUE in sd.events.names
    assert state.state == STATE.softDisabling and state.alertType == 'commIssue/softDisable'
    assert not sd.aol_stale_alert_deferred


@pytest.mark.parametrize('age_ns, due', [(0, True), (29_000_000, False), (99_999_999, False), (100_000_000, True), (150_000_000, True)])
def test_alert_due_boundary(age_ns, due):
  sd = selfdrived.SelfdriveD.__new__(selfdrived.SelfdriveD)
  now = BASE + 200_000_000
  intent = mock.Mock(carStateLogMonoTime=now - age_ns if age_ns else 0)
  with mock.patch.object(selfdrived, 'AOL_STALE_ALERT_NS', 100_000_000):
    assert sd.aol_stale_alert_due(intent, now) == due
    assert sd.aol_stale_alert_due(None, now)
