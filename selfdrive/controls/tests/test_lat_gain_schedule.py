import json
import math

import pytest

from openpilot.selfdrive.controls.lib.latcontrol_pid import (
  _MPH_TO_MS,
  lat_gain_schedule_scale,
  parse_lat_gain_schedule,
)


def _s(**kw):
  return json.dumps(kw)


class TestParseLatGainSchedule:
  def test_absent_or_empty_falls_back(self):
    assert parse_lat_gain_schedule(None) is None
    assert parse_lat_gain_schedule("") is None
    assert parse_lat_gain_schedule(b"  ") is None

  def test_valid_string_bytes_and_dict(self):
    raw = _s(v_mph=[15, 35, 60], p=[110, 115, 105], i=[50, 75, 0], f=[50, 100, 100])
    for value in (raw, raw.encode(), json.loads(raw)):
      sched = parse_lat_gain_schedule(value)
      assert sched is not None
      assert sched["v"] == pytest.approx([15 * _MPH_TO_MS, 35 * _MPH_TO_MS, 60 * _MPH_TO_MS])
      assert sched["p"] == pytest.approx([1.10, 1.15, 1.05])
      assert sched["i"] == pytest.approx([0.5, 0.75, 0.0])

  def test_omitted_term_is_absent(self):
    sched = parse_lat_gain_schedule(_s(v_mph=[10, 40], p=[100, 120]))
    assert "p" in sched and "i" not in sched and "f" not in sched

  @pytest.mark.parametrize("raw", [
    "not json",
    "[1, 2]",
    _s(p=[100, 100]),                                   # no speeds
    _s(v_mph=[20], p=[100]),                            # one knot
    _s(v_mph=list(range(0, 90, 10)), p=[100] * 9),      # nine knots
    _s(v_mph=[30, 20], p=[100, 100]),                   # decreasing
    _s(v_mph=[20, 20], p=[100, 100]),                   # repeated
    _s(v_mph=[-5, 20], p=[100, 100]),                   # negative speed
    _s(v_mph=[20, 120], p=[100, 100]),                  # past 100 mph
    _s(v_mph=[20, 40]),                                 # no terms
    _s(v_mph=[20, 40], p=[100]),                        # length mismatch
    _s(v_mph=[20, 40], p=[100, 20]),                    # P under 25%
    _s(v_mph=[20, 40], p=[100, 301]),                   # P over 300%
    _s(v_mph=[20, 40], i=[-1, 50]),                     # negative I
    _s(v_mph=[20, 40], f=[100, 250]),                   # F over 200%
    _s(v_mph=[20, 40], p=[100, "x"]),                   # non-numeric
    '{"v_mph": [20, 40], "p": [100, NaN]}',             # non-finite
    _s(v_mph=[20, 40], p=[100, 110], i=[50, 400]),      # one bad term rejects all
  ])
  def test_invalid_rejects_whole_schedule(self, raw):
    assert parse_lat_gain_schedule(raw) is None


class TestLatGainScheduleScale:
  SCHED = parse_lat_gain_schedule(_s(v_mph=[20, 40, 60], p=[100, 120, 90]))

  def test_none_uses_band(self):
    assert lat_gain_schedule_scale(None, "p", 10.0, 0.7) == 0.7

  def test_omitted_term_uses_band(self):
    assert lat_gain_schedule_scale(self.SCHED, "i", 10.0, 0.33) == 0.33

  def test_clamped_past_end_knots(self):
    assert lat_gain_schedule_scale(self.SCHED, "p", 0.0, 9.0) == pytest.approx(1.0)
    assert lat_gain_schedule_scale(self.SCHED, "p", 80 * _MPH_TO_MS, 9.0) == pytest.approx(0.9)

  def test_linear_between_knots(self):
    assert lat_gain_schedule_scale(self.SCHED, "p", 30 * _MPH_TO_MS, 9.0) == pytest.approx(1.10)
    assert lat_gain_schedule_scale(self.SCHED, "p", 50 * _MPH_TO_MS, 9.0) == pytest.approx(1.05)
    assert lat_gain_schedule_scale(self.SCHED, "p", 40 * _MPH_TO_MS, 9.0) == pytest.approx(1.20)

  def test_continuous_across_speed(self):
    prev = None
    for k in range(3001):
      v = k * 0.01
      y = lat_gain_schedule_scale(self.SCHED, "p", v, 9.0)
      assert math.isfinite(y)
      if prev is not None:
        assert abs(y - prev) < 1e-3
      prev = y
