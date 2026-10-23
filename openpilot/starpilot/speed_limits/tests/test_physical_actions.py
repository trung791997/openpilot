"""Pending choices are source independent; selected-speed commands are not."""

import math

import pytest

from openpilot.cereal import messaging
from openpilot.selfdrive.car.cruise import SlcPendingConfirmation
from openpilot.starpilot.speed_limits import physical_actions
from openpilot.starpilot.speed_limits.selection import Source
from openpilot.starpilot.speed_limits.tests.test_runtime_replay import ReplaySM


NOW = 2_000_000_000


@pytest.fixture
def sm():
  state = messaging.new_message('slcState').slcState
  state.enabled, state.observationKind, state.source = True, 'valid', 'map'
  state.sessionId, state.frameMonoTime = 'drive', NOW
  state.hasPending, state.pendingSpeedLimit, state.speedLimit = True, 25.0, 25.0
  state.decisionId, state.presentationId = 1, 2
  return ReplaySM({'slcState': state}, NOW)


@pytest.mark.parametrize('source', tuple(Source))
def test_confirm_any_source_without_dashboard_evidence(sm, source):
  sm['slcState'].source = source.value
  assert physical_actions.pending_confirmation(sm, NOW, long_active=True) == SlcPendingConfirmation('drive', 1, 2)
  # A pending choice alone must never qualify a selected-speed command.
  current = physical_actions.state_current(sm, None, 'card', NOW)
  assert (current is not None) == (source is not Source.DASHBOARD)


@pytest.mark.parametrize('field,value', [
  ('enabled', False), ('displayOnly', True), ('observationKind', 'unknown'), ('observationKind', 'absent'),
  ('observationKind', 'stale'), ('source', 'none'), ('source', ''), ('source', 'unsupported'), ('sessionId', ''),
  ('frameMonoTime', 0), ('frameMonoTime', NOW + 1), ('frameMonoTime', NOW - physical_actions.STATE_MAX_AGE_NS - 1),
  ('hasPending', False), ('decisionId', 0), ('presentationId', 0), ('pendingSpeedLimit', 0.0),
  ('pendingSpeedLimit', math.nan), ('pendingSpeedLimit', math.inf), ('pendingSpeedLimit', 26.0), ('speedLimit', math.nan),
])
def test_reject_invalid_or_unavailable_pending_decision(sm, field, value):
  setattr(sm['slcState'], field, value)
  assert physical_actions.pending_confirmation(sm, NOW, long_active=True) is None


@pytest.mark.parametrize('valid,alive,stamp', [
  (False, True, NOW), (True, False, NOW), (True, True, 0), (True, True, NOW + 1),
  (True, True, NOW - physical_actions.STATE_MAX_AGE_NS - 1),
])
def test_reject_invalid_or_stale_transport(sm, valid, alive, stamp):
  sm.valid['slcState'], sm.alive['slcState'], sm.logMonoTime['slcState'] = valid, alive, stamp
  assert physical_actions.pending_confirmation(sm, NOW, long_active=True) is None


@pytest.mark.parametrize('long_active,after_ns', [(False, 0), (True, NOW)])
def test_preserve_authority_and_configuration_barriers(sm, long_active, after_ns):
  assert physical_actions.pending_confirmation(sm, NOW, long_active=long_active, after_ns=after_ns) is None


def test_fresh_post_configuration_decision_qualifies(sm):
  assert physical_actions.pending_confirmation(sm, NOW, long_active=True, after_ns=NOW - 1) is not None


def test_engage_target_before_engagement_uses_accepted_limit_plus_offset(sm):
  # No set speed yet: the domain has no context, so there is no effective target.
  state = sm['slcState']
  state.hasAccepted, state.acceptedSpeedLimit = True, 25.0
  state.hasAcceptedAdjustedLimit, state.acceptedAdjustedLimit = True, 27.0
  assert not state.hasEffectiveClusterTarget
  assert physical_actions.engage_target_mps(sm, NOW) == pytest.approx(27.0)
  # Once engaged, the domain-qualified effective target still takes precedence.
  state.hasEffectiveClusterTarget, state.effectiveClusterTarget = True, 26.0
  assert physical_actions.engage_target_mps(sm, NOW) == pytest.approx(26.0)


@pytest.mark.parametrize('has_accepted,has_adjusted,adjusted', [
  (False, True, 27.0), (True, False, 0.0), (True, True, 0.0), (True, True, math.nan), (True, True, -1.0),
])
def test_engage_target_absent_without_accepted_limit_or_band(sm, has_accepted, has_adjusted, adjusted):
  state = sm['slcState']
  state.hasAccepted, state.acceptedSpeedLimit = has_accepted, 25.0 if has_accepted else 0.0
  state.hasAcceptedAdjustedLimit, state.acceptedAdjustedLimit = has_adjusted, adjusted
  assert physical_actions.engage_target_mps(sm, NOW) is None


def test_engage_target_from_runtime_before_engagement():
  from dataclasses import replace
  from openpilot.starpilot.speed_limits.runtime import Runtime
  from openpilot.starpilot.speed_limits.runtime_settings import parse
  from openpilot.starpilot.speed_limits.speed_domain import OffsetBand, OffsetSchedule
  from openpilot.starpilot.speed_limits.tests import test_runtime_replay as replay_fixture
  from openpilot.starpilot.speed_limits.tests.test_set_speed import tick

  fixture = replay_fixture.RuntimeReplayTests()
  fixture.setUp()
  replay_sm, cp = fixture.sm, fixture.cp
  # Before SET: no set speed, so the selected-speed pair is not valid.
  replay_sm['carState'].vCruise = replay_sm['carState'].vCruiseCluster = 0.
  replay_sm['slcDashboardObservation'].speedMps = 25.
  offset = 5 / 3.6
  settings = replace(parse({'SpeedLimitController': True, 'SLCPriority1': 'Dashboard'}),
                     offsets=OffsetSchedule((OffsetBand(0., None, offset),)))
  runtime = Runtime(settings)
  tick(runtime, replay_sm, cp, 0)
  output = tick(runtime, replay_sm, cp, 3)
  state = output.message.slcState
  assert state.hasAccepted and not state.hasEffectiveClusterTarget
  slc_sm = ReplaySM({'slcState': state}, int(state.frameMonoTime))
  assert physical_actions.engage_target_mps(slc_sm, int(state.frameMonoTime)) == pytest.approx(25. + offset)
