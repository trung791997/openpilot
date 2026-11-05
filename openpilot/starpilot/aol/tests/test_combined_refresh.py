"""A held LAT+LONG proof near its lease end takes an already-queued replacement; no proof gains time.

Route 00000080--bdf63ac92f (2026-11-03; limited road evidence): with AOL on, lat and long dropped for
one 10 ms frame about every 2.3 s. The 30 ms axis lease could lapse between controlsd's top-of-cycle
refresh and its AOL admission check while a newer axis was already queued. Port of origin 8fd1fdc46.
"""
from types import SimpleNamespace
from typing import Any
from unittest.mock import patch

import pytest

from openpilot.starpilot.aol.runtime import (AXIS_MAX_AGE_NS, COMBINED_REFRESH_MARGIN_NS, axis_drop_reason,
                                            refresh_expired_lateral_snapshot)

T0 = 1_000_000_000
LEASE = 30_000_000
NON_GM: dict[str, Any] = {'allow_combined': True, 'margin_ns': COMBINED_REFRESH_MARGIN_NS, 'wait_for_new': False}


class FakeSM:
  def __init__(self, axis, log_ns=T0):
    self.data = {'aolAxisState': axis}
    self.valid = {'aolAxisState': True}
    self.alive = {'aolAxisState': True}
    self.seen = {'aolAxisState': True}
    self.logMonoTime = {'aolAxisState': log_ns}
    self.sock = {'aolAxisState': object()}
    self.updates = 0

  def __getitem__(self, key):
    return self.data[key]

  def update(self, timeout):
    assert timeout == 0
    self.updates += 1


def axis(*, lat=True, long=True, observed=T0, lease=LEASE):
  return SimpleNamespace(qualified=True, nativeAcknowledged=True, sessionId='s', lateralActive=lat, desiredLateral=lat,
                         longitudinalActive=long, desiredLongitudinal=long, observedMonoTime=observed,
                         validUntilMonoTime=observed + lease)


@pytest.fixture
def poller():
  with patch('openpilot.cereal.messaging.Poller') as p:
    yield p


INSIDE_MARGIN = T0 + LEASE - COMBINED_REFRESH_MARGIN_NS + 1
OUTSIDE_MARGIN = T0 + LEASE - COMBINED_REFRESH_MARGIN_NS - 1
EXPIRED = T0 + LEASE + 1


def test_non_gm_combined_axis_inside_margin_receives_queued_inputs_once(poller):
  # Route 80 seg 26: the held axis lapses mid-cycle while a newer one waits.
  a = axis()
  sm = FakeSM(a)
  assert refresh_expired_lateral_snapshot(sm, now_ns=INSIDE_MARGIN, **NON_GM)
  assert sm.updates == 1
  poller.assert_not_called()  # the GM 10 ms wait never runs on this path
  assert a.validUntilMonoTime == T0 + LEASE  # the held proof gains no time


def test_non_gm_combined_axis_outside_margin_is_left_alone(poller):
  sm = FakeSM(axis())
  assert not refresh_expired_lateral_snapshot(sm, now_ns=OUTSIDE_MARGIN, **NON_GM)
  assert not refresh_expired_lateral_snapshot(sm, now_ns=T0 + 5_000_000, **NON_GM)
  assert sm.updates == 0
  poller.assert_not_called()


def test_non_gm_combined_axis_already_expired_refreshes_without_waiting(poller):
  sm = FakeSM(axis())
  assert refresh_expired_lateral_snapshot(sm, now_ns=EXPIRED, **NON_GM)
  assert sm.updates == 1
  poller.assert_not_called()


def test_publish_age_alone_also_refreshes_inside_margin():
  sm = FakeSM(axis(observed=T0 + 20_000_000), log_ns=T0)
  assert refresh_expired_lateral_snapshot(sm, now_ns=T0 + AXIS_MAX_AGE_NS - COMBINED_REFRESH_MARGIN_NS + 1, **NON_GM)
  sm = FakeSM(axis(observed=T0 + 20_000_000), log_ns=T0)
  assert not refresh_expired_lateral_snapshot(sm, now_ns=T0 + AXIS_MAX_AGE_NS - COMBINED_REFRESH_MARGIN_NS - 1, **NON_GM)


@pytest.mark.parametrize('kwargs', [{}, NON_GM], ids=['lat_only_form', 'margin_form'])
def test_lat_only_axis_keeps_expired_only_test(kwargs, poller):
  # The margin applies to LAT+LONG proofs only; a LAT-only proof refreshes exactly as before.
  sm = FakeSM(axis(long=False))
  assert not refresh_expired_lateral_snapshot(sm, now_ns=INSIDE_MARGIN, **kwargs)
  assert sm.updates == 0
  assert refresh_expired_lateral_snapshot(sm, now_ns=EXPIRED, **kwargs)
  assert sm.updates == 1
  poller.assert_not_called()


def test_lat_only_form_never_takes_a_combined_axis():
  sm = FakeSM(axis())
  assert not refresh_expired_lateral_snapshot(sm, now_ns=EXPIRED)
  assert not refresh_expired_lateral_snapshot(sm, now_ns=EXPIRED, margin_ns=COMBINED_REFRESH_MARGIN_NS)
  assert sm.updates == 0


def test_gm_form_is_unchanged(poller):
  # GM late refresh: allow_combined=True, no margin. Expired only, and it waits one tick when nothing arrives.
  sm = FakeSM(axis())
  assert not refresh_expired_lateral_snapshot(sm, now_ns=INSIDE_MARGIN, allow_combined=True)
  assert sm.updates == 0
  poller.assert_not_called()
  assert refresh_expired_lateral_snapshot(sm, now_ns=EXPIRED, allow_combined=True)
  assert sm.updates == 2
  poller.return_value.poll.assert_called_once_with(10)


def test_unqualified_or_invalid_axis_never_refreshes():
  a = axis()
  a.qualified = False
  assert not refresh_expired_lateral_snapshot(FakeSM(a), now_ns=INSIDE_MARGIN, **NON_GM)
  sm = FakeSM(axis())
  sm.valid['aolAxisState'] = False
  assert not refresh_expired_lateral_snapshot(sm, now_ns=INSIDE_MARGIN, **NON_GM)
  a = axis()
  a.desiredLongitudinal = False  # long active without desire is neither LAT-only nor LAT+LONG
  assert not refresh_expired_lateral_snapshot(FakeSM(a), now_ns=EXPIRED, **NON_GM)


def test_drop_reason_names_an_expired_axis():
  sm = FakeSM(axis())
  assert axis_drop_reason(sm, None, now_ns=T0 + 31_000_000) == 'axis_expired'
  sm.alive['aolAxisState'] = False
  assert axis_drop_reason(sm, None, now_ns=T0 + 1_000_000) == 'axis_service'


def test_controlsd_non_gm_combined_axis_survives_lease_crossing_mid_cycle():
  # Actual Card/SelfdriveD/Controls callers (Ioniq fixture, non-GM). The clock is inside the margin at the
  # top-of-cycle refresh and past the held axis's lease at the AOL admission check, as on route 80.
  from contextlib import ExitStack
  from openpilot.common.prefix import OpenpilotPrefix
  from openpilot.starpilot.aol.tests import test_ioniq_availability as fixture

  case = fixture.IoniqAvailabilityTests(methodName='runTest')
  with OpenpilotPrefix(), ExitStack() as cleanup:
    cleanup.callback(case.doCleanups)
    sd, controls, step, card, _ = case._ioniq_availability_callers(full_op=True)
    cleanup.callback(card.vehicle_startup.close)
    old_axis, _, old_command = step(2)
    assert old_axis.lateralActive and old_axis.longitudinalActive
    assert old_command.latActive and old_command.longActive
    assert controls.CP.brand != 'gm'
    old_deadline = int(old_axis.validUntilMonoTime)
    pending = []
    with (
      patch.object(controls.sm, 'update_msgs', side_effect=lambda stamp, rows: pending.extend(rows)),
      patch.object(controls, 'state_control', return_value=(old_command, None)),
      patch.object(controls, 'publish'),
    ):
      fresh_axis, _, _ = step(5, offset_ns=-5_000_000)
    assert fresh_axis.lateralActive and fresh_axis.longitudinalActive
    assert int(fresh_axis.validUntilMonoTime) > old_deadline
    top = old_deadline - COMBINED_REFRESH_MARGIN_NS + 1_000_000
    late = old_deadline + 1_000_000
    clock = iter([top])

    def receive(timeout):
      assert timeout == 0
      controls.sm.update_msgs(top / 1e9, pending)

    with (
      patch.object(controls.sm, 'update', side_effect=receive) as receive_latest,
      patch('openpilot.selfdrive.controls.controlsd.time.monotonic_ns', side_effect=lambda: next(clock, late)),
      patch('openpilot.cereal.messaging.Poller') as poller,
    ):
      command, _ = controls.state_control()
    receive_latest.assert_called_once_with(0)
    poller.assert_not_called()
    assert command.latActive and command.longActive
    assert int(old_axis.validUntilMonoTime) == old_deadline


def test_controlsd_combined_refresh_keeps_this_cycles_update_flags():
  # The firing combined refresh runs sm.update(0), which clears every `updated` flag. Flags already set this
  # cycle must survive it (the Honda model ramp reads updated['modelV2'] later), and newly received ones stay set.
  from contextlib import ExitStack
  from openpilot.common.prefix import OpenpilotPrefix
  from openpilot.starpilot.aol.tests import test_ioniq_availability as fixture

  case = fixture.IoniqAvailabilityTests(methodName='runTest')
  with OpenpilotPrefix(), ExitStack() as cleanup:
    cleanup.callback(case.doCleanups)
    sd, controls, step, card, _ = case._ioniq_availability_callers(full_op=True)
    cleanup.callback(card.vehicle_startup.close)
    old_axis, _, old_command = step(2)
    assert controls.CP.brand != 'gm'
    old_deadline = int(old_axis.validUntilMonoTime)
    pending = []
    with (
      patch.object(controls.sm, 'update_msgs', side_effect=lambda stamp, rows: pending.extend(rows)),
      patch.object(controls, 'state_control', return_value=(old_command, None)),
      patch.object(controls, 'publish'),
    ):
      step(5, offset_ns=-5_000_000)
    # modelV2 arrived earlier this cycle only; the refresh receives the queued rows without it.
    pending = [row for row in pending if row.which() != 'modelV2']
    received = {row.which() for row in pending}
    assert 'aolAxisState' in received
    untouched = next(s for s in controls.sm.updated if s != 'modelV2' and s not in received)
    controls.sm.updated = dict.fromkeys(controls.sm.updated, False)
    controls.sm.updated['modelV2'] = True
    top = old_deadline - COMBINED_REFRESH_MARGIN_NS + 1_000_000
    late = old_deadline + 1_000_000
    clock = iter([top])
    ramp_reads = []

    def receive(timeout):
      assert timeout == 0
      controls.sm.update_msgs(top / 1e9, pending)
      assert not controls.sm.updated['modelV2']  # the receive itself clears it

    ramp = SimpleNamespace(update=lambda curvature, model_updated, active: ramp_reads.append(model_updated) or curvature)
    with (
      patch.object(controls.sm, 'update', side_effect=receive) as receive_latest,
      patch.object(controls, 'model_action_ramp', ramp, create=True),
      patch('openpilot.selfdrive.controls.controlsd.time.monotonic_ns', side_effect=lambda: next(clock, late)),
      patch('openpilot.cereal.messaging.Poller'),
    ):
      controls.state_control()
    receive_latest.assert_called_once_with(0)
    assert ramp_reads == [True]
    assert controls.sm.updated['modelV2']
    assert all(controls.sm.updated[s] for s in received)
    assert not controls.sm.updated[untouched]
