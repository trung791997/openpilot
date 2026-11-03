from types import SimpleNamespace

from openpilot.starpilot.aol.runtime import (AXIS_MAX_AGE_NS, COMBINED_REFRESH_MARGIN_NS, axis_drop_reason,
                                            refresh_expiring_combined_snapshot)

T0 = 1_000_000_000


class FakeSM:
  def __init__(self, axis, log_ns=T0):
    self.data = {'aolAxisState': axis}
    self.valid = {'aolAxisState': True}
    self.alive = {'aolAxisState': True}
    self.seen = {'aolAxisState': True}
    self.logMonoTime = {'aolAxisState': log_ns}
    self.updates = 0

  def __getitem__(self, key):
    return self.data[key]

  def update(self, timeout):
    assert timeout == 0
    self.updates += 1


def axis(*, lat=True, long=True, observed=T0, lease=30_000_000):
  return SimpleNamespace(qualified=True, nativeAcknowledged=True, sessionId='s', lateralActive=lat, desiredLateral=lat,
                         longitudinalActive=long, desiredLongitudinal=long, observedMonoTime=observed,
                         validUntilMonoTime=observed + lease)


def test_fresh_combined_axis_is_left_alone():
  sm = FakeSM(axis())
  assert not refresh_expiring_combined_snapshot(sm, now_ns=T0 + 5_000_000)
  assert sm.updates == 0


def test_combined_axis_about_to_lapse_receives_queued_inputs_once():
  # Route 80 seg 26: the held axis lapses mid-cycle while a newer one waits.
  sm = FakeSM(axis())
  assert refresh_expiring_combined_snapshot(sm, now_ns=T0 + 30_000_000 - COMBINED_REFRESH_MARGIN_NS + 1)
  assert sm.updates == 1


def test_publish_age_alone_also_refreshes():
  sm = FakeSM(axis(observed=T0 + 20_000_000), log_ns=T0)
  assert refresh_expiring_combined_snapshot(sm, now_ns=T0 + AXIS_MAX_AGE_NS - COMBINED_REFRESH_MARGIN_NS + 1)


def test_lat_only_axis_is_not_this_path():
  sm = FakeSM(axis(long=False))
  assert not refresh_expiring_combined_snapshot(sm, now_ns=T0 + 40_000_000)
  assert sm.updates == 0


def test_unqualified_or_invalid_axis_never_refreshes():
  a = axis()
  a.qualified = False
  assert not refresh_expiring_combined_snapshot(FakeSM(a), now_ns=T0 + 40_000_000)
  sm = FakeSM(axis())
  sm.valid['aolAxisState'] = False
  assert not refresh_expiring_combined_snapshot(sm, now_ns=T0 + 40_000_000)


def test_drop_reason_names_an_expired_axis():
  sm = FakeSM(axis())
  assert axis_drop_reason(sm, None, now_ns=T0 + 31_000_000) == 'axis_expired'
  sm.alive['aolAxisState'] = False
  assert axis_drop_reason(sm, None, now_ns=T0 + 1_000_000) == 'axis_service'
