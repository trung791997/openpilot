import json

import pytest

from openpilot.selfdrive.controls.lib import lat_adaptive_tune as A

MPH = A.MPH_TO_MS


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
  st = A.DriveStats(carry)
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
  return A.tuning_fingerprint({k: params.get(k) for k in A.TUNING_KEYS})


def saved(params, stats):
  """Stats as the previous drive would have saved them under `params`' current tuning."""
  stats.tuning = fp(params)
  params.values["LatAdaptiveStats"] = stats.to_json()
  return params


def knot_index(v_mph):
  return A.KNOTS_MPH.index(v_mph)


class TestWeights:
  def test_weights_sum_to_one_and_are_flat_past_ends(self):
    for v in (0.0, 5.0, 10.0, 15.0, 20.0, 30.0):
      assert sum(w for _, w in A.knot_weights(v)) == pytest.approx(1.0)
    assert A.knot_weights(0.0) == ((0, 1.0),)
    assert A.knot_weights(40.0) == ((len(A.KNOTS) - 1, 1.0),)

  def test_interp_factor(self):
    f = [1.0, 1.1, 1.0, 0.9]
    assert A.interp_factor(f, 25 * MPH) == pytest.approx(1.05)
    assert A.interp_factor(f, 10 * MPH) == pytest.approx(1.0)
    assert A.interp_factor(f, 70 * MPH) == pytest.approx(0.9)


class TestMetrics:
  def test_sign_rate_straight_rms_and_curve_ratio(self):
    st = drive(4, 30, straight_err=0.5, flip_every=100, curve_des=10.0, curve_ratio=0.9)  # 2 flips per 300-frame run
    m = A.knot_metrics(st.acc[knot_index(30)])
    assert m["min"] == pytest.approx(4.0, rel=1e-3)
    assert m["straight_rms"] == pytest.approx(0.5)
    assert m["sign_rate"] == pytest.approx(2 / 3, rel=0.01)
    assert m["curve_ratio"] == pytest.approx(0.9)

  def test_excluded_frames_do_not_count(self):
    st = A.DriveStats()
    for _ in range(1000):
      st.observe(30 * MPH, 0.0, 1.0, True, False, False)
      st.observe(30 * MPH, 0.0, 1.0, False, True, False)
      st.observe(30 * MPH, 0.0, 1.0, False, False, True)
      st.observe(2.0, 0.0, 1.0, False, False, False)
    assert all(a["n"] == 0 for a in st.acc)

  def test_json_round_trip_and_rejects_garbage(self):
    st = drive(1, 40)
    back = A.DriveStats.from_json(st.to_json())
    assert back.acc[knot_index(40)]["n"] == pytest.approx(st.acc[knot_index(40)]["n"])
    for bad in ("", "x", "[]", json.dumps({"version": 99}), json.dumps({"version": 1, "knots_mph": [1, 2], "acc": []})):
      assert A.DriveStats.from_json(bad) is None


class TestRules:
  def test_short_drive_holds_and_carries(self):
    s1 = A.update_state(A.default_state(), drive(2, 40, curve_des=10, curve_ratio=0.9), A.MODE_APPLY)
    assert s1["factor"] == [1.0] * len(A.KNOTS)
    assert s1["carry"][knot_index(40)] is not None
    s2 = A.update_state(s1, drive(2, 40, curve_des=10, curve_ratio=0.9, carry=s1["carry"]), A.MODE_APPLY)
    assert s2["factor"][knot_index(40)] == pytest.approx(1.0 + A.STEP)
    assert s2["carry"][knot_index(40)] is None

  def test_curve_shortfall_steps_up_one_step_per_drive_and_is_bounded(self):
    s = A.default_state()
    for _ in range(10):
      s = A.update_state(s, drive(4, 30, curve_des=10, curve_ratio=0.9), A.MODE_SHADOW)
    assert s["factor"][knot_index(30)] == pytest.approx(1.0 + A.MAX_NEIGHBOUR_GAP)  # neighbours had no data
    for _ in range(10):
      st = A.DriveStats()
      for v in A.KNOTS_MPH:
        st.acc[knot_index(v)] = drive(4, v, curve_des=10, curve_ratio=0.9).acc[knot_index(v)]
      s = A.update_state(s, st, A.MODE_SHADOW)
    assert s["factor"] == pytest.approx([A.FACTOR_MAX] * len(A.KNOTS))

  def test_one_step_per_drive(self):
    s = A.update_state(A.default_state(), drive(20, 30, curve_des=10, curve_ratio=0.5), A.MODE_SHADOW)
    assert s["factor"][knot_index(30)] == pytest.approx(1.0 + A.STEP)

  def test_oscillation_steps_down_to_bound(self):
    s = A.default_state()
    for _ in range(10):
      s = A.update_state(s, drive(4, 40, flip_every=50), A.MODE_SHADOW)  # 2 sign changes /s
    assert s["factor"][knot_index(40)] == pytest.approx(1.0 - A.MAX_NEIGHBOUR_GAP)

  def test_overshoot_steps_down(self):
    s = A.update_state(A.default_state(), drive(4, 40, curve_des=10, curve_ratio=1.1), A.MODE_SHADOW)
    assert s["factor"][knot_index(40)] == pytest.approx(1.0 - A.STEP)

  def test_no_step_up_when_override_onsets_high(self):
    s = A.update_state(A.default_state(), drive(4, 20, curve_des=10, curve_ratio=0.85, press_every=2000), A.MODE_APPLY)
    assert s["factor"][knot_index(20)] == pytest.approx(1.0)  # 3 onsets/min > PRESS_RATE_UP_MAX

  def test_no_step_up_when_already_near_oscillation_limit(self, monkeypatch):
    monkeypatch.setattr(A, "SIGN_RATE_UP_MAX", 0.5)  # 0.67 /s: above the step-up limit, below SIGN_RATE_MAX
    s = A.update_state(A.default_state(), drive(4, 30, flip_every=100, curve_des=10, curve_ratio=0.85), A.MODE_APPLY)
    assert s["factor"][knot_index(30)] == pytest.approx(1.0)

  def test_revert_after_increase_that_raised_oscillation_only_in_apply(self):
    s = A.update_state(A.default_state(), drive(4, 30, flip_every=150, curve_des=10, curve_ratio=0.9), A.MODE_APPLY)
    assert s["factor"][knot_index(30)] == pytest.approx(1.05)
    worse = drive(4, 30, flip_every=100, curve_des=10, curve_ratio=0.9)  # 0.33 -> 0.67 /s, still under the limits
    assert A.update_state(s, worse, A.MODE_APPLY)["factor"][knot_index(30)] == pytest.approx(1.0)
    assert A.update_state(s, worse, A.MODE_SHADOW)["factor"][knot_index(30)] == pytest.approx(1.10)

  def test_revert_after_increase_that_raised_override_onsets(self):
    s = A.update_state(A.default_state(), drive(4, 30, curve_des=10, curve_ratio=0.9), A.MODE_APPLY)
    worse = drive(4, 30, curve_des=10, curve_ratio=0.9, press_every=6000)  # 1 onset/min, from 0
    assert A.update_state(s, worse, A.MODE_APPLY)["factor"][knot_index(30)] == pytest.approx(1.0)

  def test_neighbour_gap_is_bounded(self):
    s = A.default_state()
    s["factor"] = [0.85, 1.15, 1.0, 1.0]
    s = A.update_state(s, A.DriveStats(), A.MODE_SHADOW)
    f = s["factor"]
    assert all(abs(a - b) <= A.MAX_NEIGHBOUR_GAP + 1e-9 for a, b in zip(f[:-1], f[1:], strict=True))
    assert all(A.FACTOR_MIN <= x <= A.FACTOR_MAX for x in f)

  @pytest.mark.parametrize("raw", ["", "junk", "[]", json.dumps({"version": 1, "knots_mph": [20, 30, 40, 50], "factor": [2, 1, 1, 1]}),
                                   json.dumps({"version": 1, "knots_mph": [10, 30], "factor": [1, 1]})])
  def test_bad_state_resets(self, raw):
    assert A.parse_state(raw)["factor"] == [1.0] * len(A.KNOTS)


class TestTuner:
  def test_off_by_default_reads_nothing_else_and_writes_nothing(self):
    p = FakeParams(LatAdaptiveState={"version": 1, "knots_mph": list(A.KNOTS_MPH), "factor": [1.15] * 4})
    t = A.LatAdaptiveTuner(p)
    assert t.mode == A.MODE_OFF
    assert t.p_factor(30 * MPH) == 1.0
    for _ in range(A.SAVE_EVERY_FRAMES * 2):
      t.observe(30 * MPH, 0.0, 0.5, False, False, False)
    assert p.puts == []

  def test_shadow_learns_but_never_applies(self):
    p = saved(FakeParams(LatAdaptiveTune="1"), drive(4, 30, curve_des=10, curve_ratio=0.9))
    t = A.LatAdaptiveTuner(p)
    assert json.loads(p.values["LatAdaptiveState"])["factor"][knot_index(30)] == pytest.approx(1.05)
    assert p.values["LatAdaptiveStats"] == ""
    assert t.p_factor(30 * MPH) == 1.0

  def test_apply_uses_the_stored_factor_for_the_whole_drive(self):
    p = saved(FakeParams(LatAdaptiveTune="2"), drive(4, 30, curve_des=10, curve_ratio=0.9))
    t = A.LatAdaptiveTuner(p)
    assert t.p_factor(30 * MPH) == pytest.approx(1.05)
    for _ in range(A.SAVE_EVERY_FRAMES):
      t.observe(30 * MPH, 10.0, 5.0, False, False, False)  # badly under-following
    assert t.p_factor(30 * MPH) == pytest.approx(1.05)  # unchanged until the next drive
    assert A.DriveStats.from_json(p.values["LatAdaptiveStats"]).acc[knot_index(30)]["n"] > 0

  def test_params_failure_turns_it_off(self):
    class Broken(FakeParams):
      def put(self, key, value):
        raise OSError("disk")
    b = Broken(LatAdaptiveTune="2")
    b.values["LatAdaptiveStats"] = drive(4, 30).to_json()
    t = A.LatAdaptiveTuner(b)
    assert t.mode == A.MODE_OFF
    assert t.p_factor(30 * MPH) == 1.0

  def test_manual_tuning_change_resets_the_learned_factor(self):
    p = FakeParams(LatAdaptiveTune="2", LatIScaleStandard="25")
    s = A.default_state(fp(p))
    s["factor"] = [1.0, 1.05, 1.15, 1.05]
    p.values["LatAdaptiveState"] = json.dumps(s)
    assert A.LatAdaptiveTuner(p).p_factor(40 * MPH) == pytest.approx(1.15)  # same tuning: kept
    p.values["LatIScaleStandard"] = "75"
    t = A.LatAdaptiveTuner(p)
    assert t.p_factor(40 * MPH) == pytest.approx(1.0)
    state = json.loads(p.values["LatAdaptiveState"])
    assert state["factor"] == [1.0] * len(A.KNOTS)
    assert state["tuning"] == fp(p)
    assert state["last"] == ["reset: manual lateral tuning changed"]

  def test_stats_from_other_tuning_are_dropped(self):
    p = saved(FakeParams(LatAdaptiveTune="2", LatPScaleStandard="100"), drive(4, 30, curve_des=10, curve_ratio=0.9))
    p.values["LatPScaleStandard"] = "115"  # changed after (or during) the drive that produced the stats
    t = A.LatAdaptiveTuner(p)
    assert t.p_factor(30 * MPH) == pytest.approx(1.0)
    assert p.values["LatAdaptiveStats"] == ""

  def test_live_stats_carry_the_drive_start_fingerprint(self):
    p = FakeParams(LatAdaptiveTune="1", LatPScaleStandard="100")
    t = A.LatAdaptiveTuner(p)
    for _ in range(A.SAVE_EVERY_FRAMES):
      t.observe(30 * MPH, 0.0, 0.5, False, False, False)
    assert A.DriveStats.from_json(p.values["LatAdaptiveStats"]).tuning == fp(p)

  def test_fingerprint_ignores_unrelated_keys_and_bytes(self):
    a = A.tuning_fingerprint({"LatPScaleStandard": b"100", "SomethingElse": "1"})
    assert a == A.tuning_fingerprint({"LatPScaleStandard": "100"})
    assert a != A.tuning_fingerprint({"LatPScaleStandard": "105"})

  def test_rack_map_switch_resets_the_learned_factor(self):
    p = FakeParams(LatAdaptiveTune="2", NrdrLatUseFirmwareVgr="1")
    s = A.default_state(fp(p))
    s["factor"] = [1.0, 1.05, 1.10, 1.05]
    p.values["LatAdaptiveState"] = json.dumps(s)
    assert A.LatAdaptiveTuner(p).p_factor(40 * MPH) == pytest.approx(1.10)
    p.values["NrdrLatUseFirmwareVgr"] = "0"  # firmware VGR table -> road-measured curve
    assert A.LatAdaptiveTuner(p).p_factor(40 * MPH) == pytest.approx(1.0)
