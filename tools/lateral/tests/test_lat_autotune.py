import json
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import lat_autotune as at

from openpilot.selfdrive.controls.lib.latcontrol_pid import parse_lat_gain_schedule

BANDS = {"LatPScaleLowSpeed": "90", "LatPScaleStandard": "115", "LatPScaleHighway": "105",
         "LatIScaleLowSpeed": "50", "LatIScaleStandard": "75", "LatIScaleHighway": "0"}


def test_seed_reproduces_bands_at_knots():
  assert at.seed_values(BANDS, [20, 30, 45, 55], "p") == pytest.approx([90, 115, 115, 105])
  assert at.seed_values(BANDS, [20, 30, 45, 55], "i") == pytest.approx([50, 75, 75, 0])


def test_seed_defaults_when_band_params_absent():
  assert at.seed_values({}, [20, 30, 55], "i") == pytest.approx([20, 100, 0])


def test_seed_follows_existing_schedule():
  params = dict(BANDS, LatGainSchedule=json.dumps({"v_mph": [20, 40], "p": [100, 140]}))
  assert at.seed_values(params, [10, 30, 50], "p") == pytest.approx([100, 120, 140])
  assert at.seed_values(params, [10, 30, 50], "i") == pytest.approx([50, 75, 0])  # term not in schedule


def test_schedule_json_is_accepted_by_controller_parser():
  s = at.schedule_json([20, 30, 45, 55], ["p", "i"], [100, 115, 115, 105, 50, 75, 75, 0])
  sched = parse_lat_gain_schedule(s)
  assert sched is not None
  assert sched["p"] == pytest.approx([1.0, 1.15, 1.15, 1.05])


def test_limits_mirror_controller():
  from openpilot.selfdrive.controls.lib.latcontrol_pid import LAT_GAIN_SCHEDULE_LIMITS
  assert at.LIMITS == LAT_GAIN_SCHEDULE_LIMITS


def test_band_cost_penalises_curve_shortfall_and_added_oscillation():
  base = {"err_rms": 1.0, "straight_rms": 0.5, "curve_ratio": 1.0, "zero_cross": 1.0}
  assert at.band_cost(base, base) == pytest.approx(1.5)
  assert at.band_cost(dict(base, curve_ratio=0.90), base) == pytest.approx(1.5 + 5 * 0.07)
  assert at.band_cost(dict(base, zero_cross=1.05), base) == pytest.approx(1.5)  # inside the 10 % allowance
  assert at.band_cost(dict(base, zero_cross=1.30), base) == pytest.approx(1.5 + 2 * 0.20)


def test_band_of():
  assert at.band_of(20).startswith("low")
  assert at.band_of(30).startswith("standard")
  assert at.band_of(55).startswith("highway")


def _res(**bands):
  names = [b[0] for b in at.sim.BANDS]
  row = dict.fromkeys(names)
  for key, m in bands.items():
    row[next(n for n in names if n.startswith(key))] = m
  return [{"fit": row, "holdout": row, "all": row}]


M = {"min": 5.0, "err_rms": 1.0, "straight_rms": 0.5, "curve_ratio": 1.0, "zero_cross": 0.3}


def test_untrusted_band_may_not_oscillate_more():
  trusted = {b[0]: not b[0].startswith("highway") for b in at.sim.BANDS}
  ref = _res(highway=M)
  assert at.untrusted_ok(_res(highway=dict(M, zero_cross=0.32)), ref, trusted) == []
  assert at.untrusted_ok(_res(highway=dict(M, zero_cross=0.42)), ref, trusted)
  assert at.untrusted_ok(_res(highway=dict(M, err_rms=1.005)), ref, trusted) == []
  assert at.untrusted_ok(_res(highway=dict(M, err_rms=1.02)), ref, trusted)


def test_costs_are_relative_to_seed():
  seed = _res(low=dict(M, err_rms=12.0), standard=M)
  total, per_band = at.costs(seed, seed, "fit")
  assert total == pytest.approx(1.0)
  better = _res(low=dict(M, err_rms=11.0), standard=dict(M, err_rms=0.9))
  _, per_band = at.costs(better, seed, "fit")
  low = next(v for k, v in per_band.items() if k.startswith("low"))
  std = next(v for k, v in per_band.items() if k.startswith("standard"))
  assert low[0] == pytest.approx(11.5 / 12.5)
  assert std[0] == pytest.approx(1.4 / 1.5)
