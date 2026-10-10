"""D-104: the coast range bound fits the last BOSCH_A_COAST_FIT_WINDOW_S of the coast's own ranges, not the whole
run. Route 00000311 84:21: track 33 coasted -1.5 for 14 s while its range went from near steady to closing ~8 m/s."""
from types import SimpleNamespace

import pytest

from opendbc.car.honda import radar_interface as ri
from opendbc.car.honda.radar_interface import BOSCH_A_VREL_RATE_CHECK_MAX_DISAGREEMENT_MPS as MAXDIS

DT = 0.07


def coast_track(slow_s=13.0, slow_rate=-1.0, fast_s=1.0, fast_rate=-8.0, vrel=-1.5):
  run, d, t = [], 100.0, 0.0
  for dur, rate in ((slow_s, slow_rate), (fast_s, fast_rate)):
    for _ in range(round(dur / DT)):
      t += DT
      d += rate * DT
      run.append((t, d, None, True))
  return SimpleNamespace(last_trusted_vrel=vrel, rejoin_samples=None, inconsistent_run=run, rail_hold=False)


@pytest.fixture(autouse=True)
def restore():
  w = ri.BOSCH_A_COAST_FIT_WINDOW_S
  yield
  ri.BOSCH_A_COAST_FIT_WINDOW_S = w


def test_long_coast_follows_the_recent_range_rate():
  track = coast_track()
  v = ri._bosch_a_coast_vrel(track, rail_interval=False, range_bound=True, v_ego=30.0)
  assert v == pytest.approx(-8.0 + MAXDIS, abs=0.2)
  ri.BOSCH_A_COAST_FIT_WINDOW_S = 0.0  # whole-run fit: averages the old steady ranges in and lags
  lagged = ri._bosch_a_coast_vrel(track, rail_interval=False, range_bound=True, v_ego=30.0)
  assert lagged > v + 2.0


def test_bound_stays_one_sided_when_the_recent_range_opens():
  # the range opens in the last second: the coast is never made less closing outside a rail hold
  track = coast_track(slow_rate=-6.0, fast_rate=+3.0, vrel=-5.0)
  assert ri._bosch_a_coast_vrel(track, rail_interval=False, range_bound=True, v_ego=30.0) == -5.0


def test_short_run_is_unchanged():
  track = coast_track(slow_s=0.0, fast_s=0.7)
  w = ri._bosch_a_coast_vrel(track, rail_interval=False, range_bound=True, v_ego=30.0)
  ri.BOSCH_A_COAST_FIT_WINDOW_S = 0.0
  assert ri._bosch_a_coast_vrel(track, rail_interval=False, range_bound=True, v_ego=30.0) == w
