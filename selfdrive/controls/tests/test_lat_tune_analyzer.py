import json

import pytest

from openpilot.selfdrive.controls.lib import lat_tune_analyzer as lat

MPH = lat.MPH_TO_MS


class FakeParams:
  def __init__(self, **values):
    self.values = {k: (v if isinstance(v, str) else json.dumps(v)) for k, v in values.items()}
    self.puts = []

  def get(self, key, *args, **kwargs):
    return self.values.get(key)

  def put(self, key, value):
    self.puts.append(key)
    self.values[key] = value

  put_nonblocking = put


def drive(minutes, v_mph, *, straight_err=0.5, flip_every=100, curve_des=None, curve_ratio=1.0, press_every=None, carry=None):
  """Synthetic engaged drive at one speed. The straight-frame error alternates sign every
  `flip_every` straight frames; with curve_des, one second in four is a curve, so straight runs
  are 300 frames and a flip that lands on a run boundary is not counted (the learner resets there)."""
  st = lat.DriveStats(carry)
  n = int(minutes * 6000)
  j = 0
  for k in range(n):
    in_curve = curve_des is not None and (k // 100) % 4 == 3
    if in_curve:
      des, ang = curve_des, curve_des * curve_ratio
    else:
      sgn = 1.0 if (j // flip_every) % 2 == 0 else -1.0
      j += 1
      des, ang = 0.0, -sgn * straight_err
    pressed = press_every is not None and k % press_every == 0
    st.observe(v_mph * MPH, des, ang, pressed, False, False)
  return st


def fp(params):
  return lat.tuning_fingerprint({k: params.get(k) for k in lat.TUNING_KEYS})


def saved(params, stats):
  """Stats as the previous drive would have saved them under `params`' current tuning."""
  stats.tuning = fp(params)
  params.values["LatAdaptiveStats"] = stats.to_json()
  return params


def knot_index(v_mph):
  return lat.KNOTS_MPH.index(v_mph)


class TestWeights:
  def test_weights_sum_to_one_and_are_flat_past_ends(self):
    for v in (0.0, 5.0, 10.0, 15.0, 20.0, 30.0):
      assert sum(w for _, w in lat.knot_weights(v)) == pytest.approx(1.0)
    assert lat.knot_weights(0.0) == ((0, 1.0),)
    assert lat.knot_weights(40.0) == ((len(lat.KNOTS) - 1, 1.0),)

  def test_interp_factor(self):
    f = [1.0, 1.1, 1.0, 0.9]
    assert lat.interp_factor(f, 25 * MPH) == pytest.approx(1.05)
    assert lat.interp_factor(f, 10 * MPH) == pytest.approx(1.0)
    assert lat.interp_factor(f, 70 * MPH) == pytest.approx(0.9)


class TestMetrics:
  def test_sign_rate_straight_rms_and_curve_ratio(self):
    st = drive(4, 30, straight_err=0.5, flip_every=100, curve_des=10.0, curve_ratio=0.9)  # 2 flips per 300-frame run
    m = lat.knot_metrics(st.acc[knot_index(30)])
    assert m["min"] == pytest.approx(4.0, rel=1e-3)
    assert m["straight_rms"] == pytest.approx(0.5)
    assert m["sign_rate"] == pytest.approx(2 / 3, rel=0.01)
    assert m["curve_ratio"] == pytest.approx(0.9)

  def test_excluded_frames_do_not_count(self):
    st = lat.DriveStats()
    for _ in range(1000):
      st.observe(30 * MPH, 0.0, 1.0, True, False, False)
      st.observe(30 * MPH, 0.0, 1.0, False, True, False)
      st.observe(30 * MPH, 0.0, 1.0, False, False, True)
      st.observe(2.0, 0.0, 1.0, False, False, False)
    assert all(a["n"] == 0 for a in st.acc)

  def test_json_round_trip_and_rejects_garbage(self):
    st = drive(1, 40)
    back = lat.DriveStats.from_json(st.to_json())
    assert back.acc[knot_index(40)]["n"] == pytest.approx(st.acc[knot_index(40)]["n"])
    for bad in ("", "x", "[]", json.dumps({"version": 99}), json.dumps({"version": 1, "knots_mph": [1, 2], "acc": []})):
      assert lat.DriveStats.from_json(bad) is None


class TestRules:
  def test_short_drive_holds_and_carries(self):
    s1 = lat.update_state(lat.default_state(), drive(2, 40, curve_des=10, curve_ratio=0.9), lat.APPLY_MODE)
    assert s1["factor"] == [1.0] * len(lat.KNOTS)
    assert s1["carry"][knot_index(40)] is not None
    s2 = lat.update_state(s1, drive(2, 40, curve_des=10, curve_ratio=0.9, carry=s1["carry"]), lat.APPLY_MODE)
    assert s2["factor"][knot_index(40)] == pytest.approx(1.0 + lat.STEP)
    assert s2["carry"][knot_index(40)] is None

  def test_curve_shortfall_steps_up_one_step_per_drive_and_is_bounded(self):
    s = lat.default_state()
    for _ in range(10):
      s = lat.update_state(s, drive(4, 30, curve_des=10, curve_ratio=0.9), 0)
    assert s["factor"][knot_index(30)] == pytest.approx(1.0 + lat.MAX_NEIGHBOUR_GAP)  # neighbours had no data
    for _ in range(10):
      st = lat.DriveStats()
      for v in lat.KNOTS_MPH:
        st.acc[knot_index(v)] = drive(4, v, curve_des=10, curve_ratio=0.9).acc[knot_index(v)]
      s = lat.update_state(s, st, 0)
    assert s["factor"] == pytest.approx([lat.FACTOR_MAX] * len(lat.KNOTS))

  def test_one_step_per_drive(self):
    s = lat.update_state(lat.default_state(), drive(20, 30, curve_des=10, curve_ratio=0.5), 0)
    assert s["factor"][knot_index(30)] == pytest.approx(1.0 + lat.STEP)

  def test_oscillation_steps_down_to_bound(self):
    s = lat.default_state()
    for _ in range(10):
      s = lat.update_state(s, drive(4, 40, flip_every=50), 0)  # 2 sign changes /s
    assert s["factor"][knot_index(40)] == pytest.approx(1.0 - lat.MAX_NEIGHBOUR_GAP)

  def test_overshoot_steps_down(self):
    s = lat.update_state(lat.default_state(), drive(4, 40, curve_des=10, curve_ratio=1.1), 0)
    assert s["factor"][knot_index(40)] == pytest.approx(1.0 - lat.STEP)

  def test_no_step_up_when_override_onsets_high(self):
    s = lat.update_state(lat.default_state(), drive(4, 20, curve_des=10, curve_ratio=0.85, press_every=2000), lat.APPLY_MODE)
    assert s["factor"][knot_index(20)] == pytest.approx(1.0)  # 3 onsets/min > PRESS_RATE_UP_MAX

  def test_no_step_up_when_already_near_oscillation_limit(self, monkeypatch):
    monkeypatch.setattr(lat, "SIGN_RATE_UP_MAX", 0.5)  # 0.67 /s: above the step-up limit, below SIGN_RATE_MAX
    s = lat.update_state(lat.default_state(), drive(4, 30, flip_every=100, curve_des=10, curve_ratio=0.85), lat.APPLY_MODE)
    assert s["factor"][knot_index(30)] == pytest.approx(1.0)

  def test_revert_after_increase_that_raised_oscillation_only_in_apply(self):
    s = lat.update_state(lat.default_state(), drive(4, 30, flip_every=150, curve_des=10, curve_ratio=0.9), lat.APPLY_MODE)
    assert s["factor"][knot_index(30)] == pytest.approx(1.05)
    worse = drive(4, 30, flip_every=100, curve_des=10, curve_ratio=0.9)  # 0.33 -> 0.67 /s, still under the limits
    assert lat.update_state(s, worse, lat.APPLY_MODE)["factor"][knot_index(30)] == pytest.approx(1.0)
    assert lat.update_state(s, worse, 0)["factor"][knot_index(30)] == pytest.approx(1.10)

  def test_revert_after_increase_that_raised_override_onsets(self):
    s = lat.update_state(lat.default_state(), drive(4, 30, curve_des=10, curve_ratio=0.9), lat.APPLY_MODE)
    worse = drive(4, 30, curve_des=10, curve_ratio=0.9, press_every=6000)  # 1 onset/min, from 0
    assert lat.update_state(s, worse, lat.APPLY_MODE)["factor"][knot_index(30)] == pytest.approx(1.0)

  def test_neighbour_gap_is_bounded(self):
    s = lat.default_state()
    s["factor"] = [0.85, 1.15, 1.0, 1.0]
    s = lat.update_state(s, lat.DriveStats(), 0)
    f = s["factor"]
    assert all(abs(a - b) <= lat.MAX_NEIGHBOUR_GAP + 1e-9 for a, b in zip(f[:-1], f[1:], strict=True))
    assert all(lat.FACTOR_MIN <= x <= lat.FACTOR_MAX for x in f)

  @pytest.mark.parametrize("raw", ["", "junk", "[]", json.dumps({"version": 1, "knots_mph": [20, 30, 40, 50], "factor": [2, 1, 1, 1]}),
                                   json.dumps({"version": 1, "knots_mph": [10, 30], "factor": [1, 1]})])
  def test_bad_state_resets(self, raw):
    assert lat.parse_state(raw)["factor"] == [1.0] * len(lat.KNOTS)


class TestTrialAndSchedule:
  BASELINE = {"fingerprint": "deadbeef", "pPct": [100.0, 100.0, 100.0, 100.0], "raw": {}}

  def test_build_trial_reports_readiness_factors_and_reasons(self):
    stats = drive(4, 30, curve_des=8.0, curve_ratio=0.9)   # 4 min at 30 mph, curve shortfall -> up
    trial = lat.build_trial(stats, self.BASELINE, ["r1"], [{"route": "r1", "minutes": [0, 4, 0, 0]}], [])
    assert trial["schemaVersion"] == 1
    assert trial["knotsMph"] == [20.0, 30.0, 40.0, 50.0]
    k30 = trial["knots"][1]
    assert k30["ready"] is True and k30["minutes"] >= 3.9
    assert k30["decision"] == "up" and k30["factor"] == 1.05
    assert trial["knots"][0]["ready"] is False and trial["knots"][0]["factor"] == 1.0
    assert trial["proposedPPct"] == [100.0, 105.0, 100.0, 100.0]
    assert trial["baseline"]["fingerprint"] == "deadbeef"
    assert trial["routeNames"] == ["r1"] and trial["perRoute"][0]["route"] == "r1"
    assert trial["applied"] is None

  def test_build_trial_scales_the_baseline_p(self):
    stats = drive(4, 30, curve_des=8.0, curve_ratio=0.9)
    base = dict(self.BASELINE, pPct=[110.0, 120.0, 130.0, 140.0])
    trial = lat.build_trial(stats, base, ["r1"], [], [])
    assert trial["proposedPPct"] == [110.0, 126.0, 130.0, 140.0]

  def test_build_schedule_is_parseable_and_carries_i_f(self):
    from openpilot.selfdrive.controls.lib.latcontrol_pid import parse_lat_gain_schedule
    stats = drive(4, 30, curve_des=8.0, curve_ratio=0.9)
    trial = lat.build_trial(stats, self.BASELINE, ["r1"], [], [])
    raw = lat.build_schedule(trial, '{"v_mph": [10, 60], "p": [100, 100], "i": [40, 90], "f": [100, 100]}')
    sched = parse_lat_gain_schedule(raw)
    assert sched is not None
    d = json.loads(raw)
    assert d["v_mph"] == [20.0, 30.0, 40.0, 50.0]
    assert d["p"] == [100.0, 105.0, 100.0, 100.0]
    assert d["i"] == [50.0, 60.0, 70.0, 80.0]      # linear 40@10 -> 90@60
    assert d["f"] == [100.0, 100.0, 100.0, 100.0]

  def test_build_schedule_without_current_schedule_has_only_p(self):
    stats = drive(4, 30)
    trial = lat.build_trial(stats, self.BASELINE, [], [], [])
    d = json.loads(lat.build_schedule(trial, None))
    assert set(d) == {"v_mph", "p"}

  def test_build_schedule_clamps_p_to_schedule_limits(self):
    stats = drive(4, 30, curve_des=8.0, curve_ratio=0.9)
    trial = lat.build_trial(stats, dict(self.BASELINE, pPct=[290.0, 290.0, 290.0, 290.0]), [], [], [])
    d = json.loads(lat.build_schedule(trial, None))
    assert max(d["p"]) == 300.0

  def test_effective_p_pct_from_bands_and_schedule(self):
    raw = {"LatPScaleLowSpeed": "80", "LatPScaleStandard": "120", "LatPScaleHighway": "90", "LatGainSchedule": ""}
    assert lat.effective_p_pct(raw) == [80.0, 120.0, 120.0, 90.0]
    raw["LatGainSchedule"] = '{"v_mph": [20, 50], "p": [100, 200]}'
    assert lat.effective_p_pct(raw) == [100.0, 133.3, 166.7, 200.0]
