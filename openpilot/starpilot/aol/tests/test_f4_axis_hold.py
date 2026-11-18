"""Route 000000f4 (limited road evidence, logged gaps): silent pause (seg 12) is bridged; late axis (A seg 5, B seg 16) is not.

x86-runnable: AxisHold and the native-withdrawal classification are exercised with stubs; the Honda
lifecycle fixture, pandad and panda re-grant timing are aarch64 items (not covered here).
"""
from types import SimpleNamespace as NS
from unittest import mock

from openpilot.cereal import log
from openpilot.selfdrive.selfdrived import selfdrived
from openpilot.selfdrive.selfdrived.selfdrived import SelfdriveD
from openpilot.starpilot.aol.runtime import AXIS_MAX_AGE_NS, STALE_ALERT_NS, AxisHold

import pytest

MS = 1_000_000


@pytest.fixture(autouse=True)
def shipped_stale_alert(monkeypatch):
  # tests/conftest.py zeroes AOL_STALE_ALERT_NS (legacy 30 ms contracts); these tests need the shipped 100 ms bound
  monkeypatch.setattr(selfdrived, 'AOL_STALE_ALERT_NS', 100 * MS)
T0 = 50_000 * 1_000_000_000
SESSION = 'sess'


def axis_msg(*, lat=True, lng=True, fault='none', source=1, session=SESSION, valid_until=None):
  return NS(validUntilMonoTime=valid_until if valid_until is not None else T0 + 10**12, sessionId=session, faultReason=fault, nativeAcknowledged=True, lateralActive=lat, longitudinalActive=lng,
            desiredLateral=True, desiredLongitudinal=lng or fault == 'availabilityPause' and False or True,
            sourceCarStateMonoTime=source, qualified=True)


class FakeSM:
  def __init__(self, axis, log_ns):
    self.d = {'aolAxisState': axis}
    self.logMonoTime = {'aolAxisState': log_ns}
    self.seen = {'aolAxisState': True}
    self.valid = {'aolAxisState': True}
    self.alive = {'aolAxisState': True}

  def __getitem__(self, key):
    return self.d[key]


def hold_after(published_at, now, *, active=None, allowed=True):
  """Feed one active axis at published_at, then resolve at `now` with the lapsed (None) axis."""
  hold = AxisHold()
  active = active or axis_msg()
  sm = FakeSM(active, T0 + published_at * MS)
  assert hold.resolve(sm, active, now_ns=T0 + published_at * MS, allowed=True) is active
  return hold, sm, hold.resolve(sm, None, now_ns=T0 + now * MS, allowed=allowed)


def test_A_late_axis_40ms_and_50ms_drops_as_on_base():
  # seg 5: last axis -61.1 ms, controlsd ticks at -20.7 ms (40.4 ms old) and -10.7 ms (50.4 ms old): axis is None, no bridge
  for age in (40.4, 50.4):
    hold, _, out = hold_after(0, age)
    assert out is None
    assert hold.last is None
  assert 40.4 * MS > AXIS_MAX_AGE_NS


def test_B_late_axis_44ms_drops_as_on_base():
  hold, _, out = hold_after(0, 44.5)
  assert out is None and hold.last is None


def test_missing_axis_is_never_held_even_inside_stale_alert_bound():
  _, _, out = hold_after(0, 1.0)
  assert out is None
  _, _, out = hold_after(0, STALE_ALERT_NS / MS - 0.1)
  assert out is None


def test_seg12_silent_pause_is_held_then_recovers():
  hold = AxisHold()
  active = axis_msg()
  sm = FakeSM(active, T0)
  hold.resolve(sm, active, now_ns=T0, allowed=True)
  pause = axis_msg(lat=False, lng=False, fault='availabilityPause', source=0)
  sm = FakeSM(pause, T0 + 30 * MS)  # pause published 30 ms after the last active axis
  held = hold.resolve(sm, pause, now_ns=T0 + 35 * MS, allowed=True)
  assert held is not None and held.lateralActive and held.longitudinalActive
  later = hold.resolve(sm, pause, now_ns=T0 + STALE_ALERT_NS + 1, allowed=True)
  assert later is pause  # silent pause longer than 100 ms is not held
  # a fresh active axis restarts the hold
  new = axis_msg()
  sm = FakeSM(new, T0 + 120 * MS)
  assert hold.resolve(sm, new, now_ns=T0 + 120 * MS, allowed=True) is new


def test_silent_pause_past_its_own_valid_until_is_not_held():
  hold = AxisHold()
  active = axis_msg()
  sm = FakeSM(active, T0)
  hold.resolve(sm, active, now_ns=T0, allowed=True)
  pause = axis_msg(lat=False, lng=False, fault='availabilityPause', source=0, valid_until=T0 + 40 * MS)
  sm = FakeSM(pause, T0 + 30 * MS)
  assert hold.resolve(sm, pause, now_ns=T0 + 40 * MS, allowed=True) is not pause  # still valid: held
  hold = AxisHold()
  hold.resolve(FakeSM(active, T0), active, now_ns=T0, allowed=True)
  out = hold.resolve(sm, pause, now_ns=T0 + 41 * MS, allowed=True)
  assert out is pause and hold.last is None  # past the paused axis's validUntilMonoTime: no hold


def active_hold():
  """A hold primed by one active axis at T0, with no further resolve."""
  hold = AxisHold()
  active = axis_msg()
  sm = FakeSM(active, T0)
  assert hold.resolve(sm, active, now_ns=T0, allowed=True) is active
  return hold, sm, None


def test_hold_refuses_faults_sessions_and_driver_conditions():
  hold, sm, _ = active_hold()
  crit = axis_msg(lat=False, lng=False, fault='critical', source=0)
  assert hold.resolve(sm, crit, now_ns=T0 + 20 * MS, allowed=True) is crit
  assert hold.last is None
  hold, sm, _ = active_hold()
  other = axis_msg(lat=False, lng=False, fault='availabilityPause', source=0, session='other')
  assert hold.resolve(sm, other, now_ns=T0 + 20 * MS, allowed=True) is other
  hold, sm, _ = active_hold()
  pause = axis_msg(lat=False, lng=False, fault='availabilityPause', source=0)
  assert hold.resolve(sm, pause, now_ns=T0 + 20 * MS, allowed=False) is pause  # pedal/cancel/override: no hold
  hold, sm, _ = active_hold()
  pause_with_source = axis_msg(lat=False, lng=False, fault='availabilityPause', source=5)
  assert hold.resolve(sm, pause_with_source, now_ns=T0 + 20 * MS, allowed=True) is pause_with_source


class Stub:
  honda_native_withdrawn_by_lease_lapse = SelfdriveD.honda_native_withdrawn_by_lease_lapse
  honda_native_lateral_grant_lost = SelfdriveD.honda_native_lateral_grant_lost

  def __init__(self, native, prior):
    self.CP = NS()
    self.initialized = True
    self.aol_session_id = SESSION
    self.aol_fault_reason = 'none'
    self.aol_availability_pause = NS(active=False)
    self.aol_transport_pause = NS(observed_native=prior, active=False, cp='cp', session=SESSION)
    self.events = NS(contains=lambda e: False)
    self.sm = NS(all_checks=lambda names: True,
                 __getitem__=None)
    self.sm = SmStub()
    self.native = native


class SmStub:
  def __getitem__(self, key):
    return NS(calStatus=log.ExtrinsicsCalibration.Status.calibrated)

  def all_checks(self, names):
    return True


def native_state(observed, req, allowed):
  return NS(requestedLateral=req, lateralAllowed=allowed, observedMonoTime=observed, axisSessionId=SESSION,
            validUntilMonoTime=observed + 200 * MS,
            pandaSerial='s', safetyModel=1, safetyParam=2, pandaInventory=[])


def grant_lost(stub, native, now_ns, intent_source):
  CS = NS(brakePressed=False, regenBraking=False, gasPressed=False, steeringPressed=False)
  previous = NS(lateral_active=True)
  requested = NS(desired_lateral=True)
  intent = NS(producerSessionId='p', carStateLogMonoTime=intent_source)
  with mock.patch.object(selfdrived, 'honda_aol_qualified', return_value=True), \
       mock.patch.object(selfdrived, 'honda_temporary_restriction', return_value=False), \
       mock.patch.object(selfdrived, 'cp_identity', return_value='cp'), \
       mock.patch.object(selfdrived, 'current_native', return_value=native), \
       mock.patch.object(selfdrived, 'healthy_transport', return_value=True):
    return type(stub).honda_native_lateral_grant_lost(stub, CS, native, previous, requested, source_current=True,
                                                previous_intent=intent, intent=intent, now_ns=now_ns)


def scenario(axis_gap_ms, wire_ms_after_prev, now_ms_after_prev, publish_successor, prior_age_ms=80):
  """Logged shape: axis at 0, safety wire sampled inside the gap, step at now (A: gap 53, wire +50.1, step +53)."""
  prior = native_state(T0 - prior_age_ms * MS, True, True)
  native = native_state(T0 + wire_ms_after_prev * MS, False, False)
  stub = Stub(native, prior)
  stub.aol_last_axis_lease = (T0, T0 + 30 * MS)
  now = T0 + now_ms_after_prev * MS
  if publish_successor:
    stub.aol_axis_lease_lapse = (T0 + 30 * MS, T0 + axis_gap_ms * MS, T0)
    stub.aol_last_axis_lease = (T0 + axis_gap_ms * MS, T0 + (axis_gap_ms + 30) * MS)
  return stub, native, now


class BaseStub(Stub):
  """Reproduces base classification: no lease-lapse exemption."""
  def honda_native_withdrawn_by_lease_lapse(self, native, now_ns):
    return False


def base_behaviour(stub, native, now, source):
  base = BaseStub.__new__(BaseStub)
  base.__dict__.update(stub.__dict__)
  return grant_lost(base, native, now, source)


def test_A_native_withdrawn_inside_lease_lapse_is_not_a_lost_grant():
  # seg 5: axis gap 53.0 ms, wire sampled 50.1 ms after the old axis (inside the lapse)
  stub, native, now = scenario(53.0, 50.1, 53.0, publish_successor=False)
  assert base_behaviour(stub, native, now, T0 + 40 * MS) is True  # base: lost grant -> authority_lost
  assert grant_lost(stub, native, now, T0 + 40 * MS) is False
  # next step: successor axis published, same wire still the latest
  stub, native, now = scenario(53.0, 50.1, 56.0, publish_successor=True)
  assert grant_lost(stub, native, now, T0 + 40 * MS) is False


def test_B_native_withdrawn_inside_lease_lapse_is_not_a_lost_grant():
  # seg 16: gap 44.5 ms, wire 40 ms after the old axis
  stub, native, now = scenario(44.5, 40.0, 44.5, publish_successor=False)
  assert base_behaviour(stub, native, now, T0 + 30 * MS) is True
  assert grant_lost(stub, native, now, T0 + 30 * MS) is False


def test_unchanged_paths_still_fire():
  # panda revokes while the lease is valid (wire at +20 ms, lease to +30 ms): still a lost grant
  stub, native, now = scenario(25.0, 20.0, 25.0, publish_successor=False)
  stub.aol_last_axis_lease = (T0, T0 + 30 * MS)
  assert grant_lost(stub, native, now, T0 + 10 * MS) is True
  # lapse longer than STALE_ALERT_NS: genuinely stale, fires
  stub, native, now = scenario(130.0, 110.0, 130.0, publish_successor=False, prior_age_ms=-100)
  assert grant_lost(stub, native, now, T0 + 100 * MS) is True
  # wire sampled after the successor axis and still withdrawn: fires
  stub, native, now = scenario(53.0, 60.0, 62.0, publish_successor=True)
  assert grant_lost(stub, native, now, T0 + 50 * MS) is True
