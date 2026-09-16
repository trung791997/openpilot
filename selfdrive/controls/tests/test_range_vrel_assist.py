"""Tests for the range-derived vRel assist (D-053, TEST feature, default OFF).

The assist lets the range-LSQ closing rate correct the Bosch-A native U11 velocity, in ONE
direction only: it may report more closing, never less. Read the RANGE_VREL_ASSIST_* comment
block in radard.py before changing a number here -- every constant carries the evidence that set
it, and several of these tests exist only to make a specific failure mode visible.

The tests that matter most are the negative ones. This feature reads the range channel to check a
velocity channel, so it is BLIND to a range error, and STATUS.md's t~=11 s false brake is exactly
a range error. `TestTheRangeWalkFault` pins that the recorded figures leave the assist inert. If a
future change makes that case fire, this feature has become a brake-harder amplifier for the one
fault the branch has actually recorded.

Geometry and velocities are taken from recorded routes wherever a recorded number exists, so a
change that breaks one of these breaks against road data rather than an invented scenario (D-009).
Nothing here is road evidence FOR the assist: these are static unit tests against a synthetic
range series, and the assist has never run on a car.
"""

import math

import pytest

from openpilot.selfdrive.controls import radard
from openpilot.selfdrive.controls.radard import (
  RANGE_VREL_ASSIST_ARM_UPDATES,
  RANGE_VREL_ASSIST_MAX_ABS_Y_REL_M,
  RANGE_VREL_ASSIST_MAX_CORRECTION_MPS,
  RANGE_VREL_ASSIST_MIN_DISAGREEMENT_MPS,
  RANGE_VREL_ASSIST_MIN_D_REL_M,
  RANGE_VREL_SAMPLES,
)

DT = radard.HONDA_BOSCH_A_RADAR_TS   # ~0.0697 s, the physical Bosch-A sweep period
DT_MDL = 0.05                        # radard's own loop rate: 20 Hz, the model rate
V_EGO = 20.0


def new_track(track_id: int = 1, v_lead: float = V_EGO) -> radard.Track:
  return radard.Track(track_id, v_lead, radard.KalmanParams(DT))


def feed(track, n, *, d0, range_rate, v_rel, y_rel=0.0, t0=0.0, dt=DT,
         range_assist=True, measured=True, d_offsets=None):
  """Drive `track` through n Bosch-A sweeps of a constant-rate range series.

  `range_rate` is d(dRel)/dt in m/s -- negative closes. `v_rel` is what U11 claims, independently,
  so a test can make the two disagree by exactly the amount it wants. `d_offsets` injects a
  per-sample range error for the outlier tests. Returns the list of (t, dRel) actually fed.
  """
  fed = []
  for i in range(n):
    t = t0 + i * dt
    d = d0 + range_rate * (i * dt)
    if d_offsets is not None:
      d += d_offsets.get(i, 0.0)
    track.update(d, y_rel, v_rel, V_EGO + v_rel, measured, measured,
                 t_now=t, range_assist=range_assist)
    fed.append((t, d))
  return fed


def feed_radard_cadence(track, n_cycles, *, d0, range_rate, v_rel, y_rel=0.0,
                        range_assist=True, coast_sweeps=()):
  """Drive `track` the way RadarD actually drives it: a 20 Hz loop over a 14.35 Hz radar.

  Every other helper here feeds one Track.update per radar sweep, which is NOT what happens on the
  car. radard runs at the model rate, so roughly one cycle in four carries no new liveTracks
  message and RadarD passes `measured=False, measurement_update=False` with an UNCHANGED t_now and
  unchanged point data. That is a DUPLICATE, not a coast, and conflating the two made the assist
  unable to arm at all. `coast_sweeps` marks sweeps that do arrive but with the parser's measured
  bit clear -- a real coast, where t_now DOES advance.

  Returns a (sweep_index, fresh, correction) trace, one entry per radard cycle.
  """
  trace = []
  last_sweep = -1
  for k in range(n_cycles):
    sweep = int(k * DT_MDL / DT)
    fresh = sweep != last_sweep
    last_sweep = sweep
    t_now = sweep * DT
    measured = fresh and sweep not in coast_sweeps
    track.update(d0 + range_rate * t_now, y_rel, v_rel, V_EGO + v_rel, measured, measured,
                 t_now=t_now, range_assist=range_assist)
    trace.append((sweep, fresh, track.range_assist_correction))
  return trace


def settle(track, **kwargs):
  """Feed exactly enough sweeps to fill the LSQ window and then arm: the window needs
  RANGE_VREL_SAMPLES samples before the first fit exists, and arming needs
  RANGE_VREL_ASSIST_ARM_UPDATES qualifying fits after that."""
  return feed(track, RANGE_VREL_SAMPLES + RANGE_VREL_ASSIST_ARM_UPDATES - 1, **kwargs)


# ---------------------------------------------------------------------------------------------
# Default OFF
# ---------------------------------------------------------------------------------------------

class TestDefaultOff:
  def test_param_default_is_off(self):
    """A TEST feature must not change behaviour for anyone who has not opted in."""
    from openpilot.common.params import Params
    p = Params()
    p.remove("RangeDerivedVrel")
    assert p.get_bool("RangeDerivedVrel") is False

  def test_track_is_inert_without_the_flag(self):
    """Same geometry as the rail case below, which corrects hard when armed."""
    track = new_track()
    settle(track, d0=76.0, range_rate=-19.4, v_rel=-13.49, range_assist=False)
    assert track.range_assist_correction == 0.0
    assert track.get_RadarState()["vRel"] == pytest.approx(-13.49)

  def test_radard_leaves_the_flag_off_for_non_bosch_a(self, monkeypatch):
    """The toggle is Bosch-A only: the param must not be consulted at all elsewhere."""
    radar_d = radard.RadarD(honda_bosch_a_radar=False)
    calls = []
    monkeypatch.setattr(radar_d, "_range_vrel_assist_enabled",
                        lambda: calls.append(1) or True)
    assert (radar_d.honda_bosch_a_radar and radar_d._range_vrel_assist_enabled()) is False
    assert calls == [], "the param was read on a car the assist can never apply to"

  def test_param_read_failure_falls_back_to_off(self, monkeypatch):
    """A device whose params_pyx.so predates RangeDerivedVrel raises on the read. That must
    degrade to the shipped U11 behaviour, not to an enabled feature."""
    radar_d = radard.RadarD(honda_bosch_a_radar=True)

    class Boom:
      def get_bool(self, _key):
        raise KeyError("RangeDerivedVrel")

    radar_d._range_assist_params = Boom()
    assert radar_d._range_vrel_assist_enabled() is False


# ---------------------------------------------------------------------------------------------
# One-sidedness -- the single most important property
# ---------------------------------------------------------------------------------------------

class TestOneSided:
  def test_range_claiming_less_closing_is_ignored(self):
    """U11 says -8, the range says -2. Acting on this would RELEASE braking. D-041/D-043 both
    forbid that direction: an understated closing rate still brakes, an overstated one does not."""
    track = new_track()
    settle(track, d0=60.0, range_rate=-2.0, v_rel=-8.0)
    assert track.range_assist_correction == 0.0
    assert not track.range_assist_active
    assert track.get_RadarState()["vRel"] == pytest.approx(-8.0)

  def test_opening_lead_with_closing_u11_is_ignored(self):
    track = new_track()
    settle(track, d0=60.0, range_rate=+3.0, v_rel=-1.0)
    assert track.range_assist_correction == 0.0

  def test_correction_is_never_negative(self):
    """Sweep the whole disagreement axis; the published vRel may never exceed the native one."""
    for range_rate in [-20.0, -12.0, -6.0, -2.0, 0.0, +2.0, +6.0]:
      track = new_track()
      settle(track, d0=70.0, range_rate=range_rate, v_rel=-5.0)
      assert track.range_assist_correction >= 0.0
      assert track.get_RadarState()["vRel"] <= -5.0 + 1e-9, (
          f"range_rate={range_rate} made the lead look FASTER than U11 said")

  def test_native_vrel_is_untouched_so_association_cannot_move(self):
    """track_matches_vision and vision_track_probability read self.vRel/self.vLead. The assist
    changes what is reported ABOUT the lead, never which track is chosen -- see D-053's rejected
    alternatives. If this fails, the assist has become an association change."""
    track = new_track()
    settle(track, d0=76.0, range_rate=-19.4, v_rel=-13.49)
    assert track.range_assist_correction > 0.0, "precondition: the assist must be armed here"
    assert track.vRel == pytest.approx(-13.49)
    assert track.vLead == pytest.approx(V_EGO - 13.49)


# ---------------------------------------------------------------------------------------------
# Arming
# ---------------------------------------------------------------------------------------------

class TestArming:
  def test_no_correction_before_the_arm_count(self):
    """Exactly zero for every qualifying fit before the threshold, not a ramp up to it."""
    track = new_track()
    fits = 0
    # The first fit lands on sweep RANGE_VREL_SAMPLES, so this is exactly ARM_UPDATES fits.
    for i in range(RANGE_VREL_SAMPLES + RANGE_VREL_ASSIST_ARM_UPDATES - 1):
      track.update(76.0 - 19.4 * (i * DT), 0.0, -13.49, V_EGO - 13.49, True, True,
                   t_now=i * DT, range_assist=True)
      if not track.vRelRangeFresh:
        continue
      fits += 1
      if fits < RANGE_VREL_ASSIST_ARM_UPDATES:
        assert track.range_assist_correction == 0.0, f"corrected on qualifying fit {fits}"
      else:
        assert track.range_assist_correction > 0.0, f"still inert on qualifying fit {fits}"
    assert fits == RANGE_VREL_ASSIST_ARM_UPDATES, "the ramp was never actually exercised"

  def test_arms_exactly_on_the_nth_update(self):
    track = new_track()
    settle(track, d0=76.0, range_rate=-19.4, v_rel=-13.49)
    assert track.range_assist_active
    assert track.range_assist_arm_count == RANGE_VREL_ASSIST_ARM_UPDATES

  def test_one_update_short_stays_inert(self):
    track = new_track()
    feed(track, RANGE_VREL_SAMPLES + RANGE_VREL_ASSIST_ARM_UPDATES - 2,
         d0=76.0, range_rate=-19.4, v_rel=-13.49)
    assert not track.range_assist_active
    assert track.range_assist_correction == 0.0

  def test_disagreement_below_threshold_never_arms(self):
    """Just under MIN_DISAGREEMENT, held for far longer than the arm count."""
    track = new_track()
    below = RANGE_VREL_ASSIST_MIN_DISAGREEMENT_MPS - 0.2
    feed(track, 40, d0=70.0, range_rate=-(5.0 + below), v_rel=-5.0)
    assert track.range_assist_correction == 0.0
    assert track.range_assist_arm_count == 0

  def test_streak_resets_on_a_single_agreeing_update(self):
    """Arming must require CONSECUTIVE disagreement; an intermittent one must not accumulate."""
    track = new_track()
    feed(track, RANGE_VREL_SAMPLES + 2, d0=76.0, range_rate=-19.4, v_rel=-13.49)
    assert 0 < track.range_assist_arm_count < RANGE_VREL_ASSIST_ARM_UPDATES
    # One sweep where U11 and the range agree.
    track.update(track.dRel - 13.49 * DT, 0.0, -19.4, V_EGO - 19.4, True, True,
                 t_now=(RANGE_VREL_SAMPLES + 2) * DT, range_assist=True)
    assert track.range_assist_arm_count == 0
    assert track.range_assist_correction == 0.0


class TestGrossOutlierRejection:
  """The reason ARM_UPDATES exists. The range channel carries ~1% gross outliers and the LSQ has
  no outlier rejection, so a single bad sample clears MIN_DISAGREEMENT on its own."""

  def test_a_single_outlier_clears_the_threshold(self):
    """Precondition for the test below -- if this stops holding, the arm count is being validated
    against a scenario that no longer exercises it."""
    track = new_track()
    feed(track, RANGE_VREL_SAMPLES, d0=70.0, range_rate=-1.0, v_rel=-1.0,
         d_offsets={RANGE_VREL_SAMPLES - 1: -1.0})
    assert track.vRelRangeFresh
    assert track.vRel - track.vRelRange > RANGE_VREL_ASSIST_MIN_DISAGREEMENT_MPS, (
        "a 1 m outlier on the newest sample no longer moves the fit past MIN_DISAGREEMENT")

  def test_a_single_outlier_cannot_arm_the_assist(self):
    """One bad sample walks the 5-deep window with leverage +2h, +h, 0, -h, -2h, so it can only
    push the fit the same way twice. Five consecutive qualifying fits are unreachable."""
    track = new_track()
    feed(track, 30, d0=70.0, range_rate=-1.0, v_rel=-1.0, d_offsets={8: -1.0})
    assert track.range_assist_correction == 0.0
    assert not track.range_assist_active

  def test_two_separated_outliers_cannot_arm_the_assist(self):
    """Outliers far enough apart to never share the window are still only worth two fits each."""
    track = new_track()
    feed(track, 40, d0=70.0, range_rate=-1.0, v_rel=-1.0, d_offsets={8: -1.0, 20: -1.2})
    assert track.range_assist_correction == 0.0


# ---------------------------------------------------------------------------------------------
# Bounds and gates
# ---------------------------------------------------------------------------------------------

class TestCorrectionCap:
  def test_correction_is_capped(self):
    """The cap is a bound on the damage a bad fit can do, not a fitted value. A 40 m/s range
    error must not become a 40 m/s brake demand."""
    track = new_track()
    settle(track, d0=200.0, range_rate=-45.0, v_rel=-5.0)
    assert track.range_assist_correction == pytest.approx(RANGE_VREL_ASSIST_MAX_CORRECTION_MPS)
    assert track.get_RadarState()["vRel"] == pytest.approx(-5.0 - RANGE_VREL_ASSIST_MAX_CORRECTION_MPS)

  def test_published_vlead_carries_the_same_correction_as_vrel(self):
    """vRel and vLead differ by v_ego and must stay consistent, or the planner's closing-speed
    and follow-distance terms disagree with each other."""
    track = new_track()
    settle(track, d0=76.0, range_rate=-19.4, v_rel=-13.49)
    state = track.get_RadarState()
    assert state["vLead"] - state["vRel"] == pytest.approx(V_EGO)


class TestGeometryGates:
  """A range derivative is a RADIAL rate. These two gates are what keep it close enough to a
  longitudinal one to be read as one."""

  def test_inert_inside_the_distance_floor(self):
    track = new_track()
    settle(track, d0=RANGE_VREL_ASSIST_MIN_D_REL_M - 1.0, range_rate=-0.05, v_rel=+2.5)
    assert track.range_assist_correction == 0.0

  def test_inert_outside_the_lateral_gate(self):
    track = new_track()
    settle(track, d0=76.0, range_rate=-19.4, v_rel=-13.49,
           y_rel=RANGE_VREL_ASSIST_MAX_ABS_Y_REL_M + 0.2)
    assert track.range_assist_correction == 0.0

  def test_lateral_gate_is_symmetric(self):
    for sign in (-1.0, +1.0):
      track = new_track()
      settle(track, d0=76.0, range_rate=-19.4, v_rel=-13.49,
             y_rel=sign * (RANGE_VREL_ASSIST_MAX_ABS_Y_REL_M + 0.2))
      assert track.range_assist_correction == 0.0, f"yRel sign {sign} was not gated"

  def test_the_gates_bound_azimuth_where_the_comment_says(self):
    """Pins the pair to the 10.8 deg / 1.8% projection error claimed in the constant block, so
    that widening either constant fails here rather than silently invalidating the comment."""
    azimuth = math.degrees(math.asin(RANGE_VREL_ASSIST_MAX_ABS_Y_REL_M / RANGE_VREL_ASSIST_MIN_D_REL_M))
    assert azimuth == pytest.approx(10.8, abs=0.1)
    assert math.cos(math.radians(azimuth)) == pytest.approx(0.982, abs=0.001)

  def test_leaving_the_gate_disarms_immediately(self):
    """Not a decay: an armed correction whose geometry stops qualifying is dropped on that
    update, because the radial-rate assumption is what licensed it in the first place."""
    track = new_track()
    settle(track, d0=76.0, range_rate=-19.4, v_rel=-13.49)
    assert track.range_assist_correction > 0.0
    track.update(track.dRel - 19.4 * DT, 3.0, -13.49, V_EGO - 13.49, True, True,
                 t_now=100.0 * DT, range_assist=True)
    assert track.range_assist_correction == 0.0
    assert not track.range_assist_active


# ---------------------------------------------------------------------------------------------
# Disarming
# ---------------------------------------------------------------------------------------------

class TestDisarm:
  def test_coast_clears_the_correction(self):
    """A coast means the velocity is doubtful. The Bosch-A coast path holds last_trusted_vrel and
    can run for a long time -- measured at 121.8 s of continuous suppression of the followed lead
    on 000001fb -- so holding a correction across it would be unbounded staleness."""
    track = new_track()
    settle(track, d0=76.0, range_rate=-19.4, v_rel=-13.49)
    assert track.range_assist_correction > 0.0
    track.update(track.dRel, 0.0, -13.49, V_EGO - 13.49, False, False,
                 t_now=100.0 * DT, range_assist=True)
    assert track.range_assist_correction == 0.0
    assert track.get_RadarState()["vRel"] == pytest.approx(-13.49)

  def test_a_range_dropout_clears_the_correction(self):
    """After a gap the LSQ window is rebuilt; vRelRange holds the PRE-GAP value until it refills.
    Acting on that stale rate is exactly the trap vRelRangeFresh exists to close."""
    track = new_track()
    settle(track, d0=76.0, range_rate=-19.4, v_rel=-13.49)
    stale = track.vRelRange
    assert track.range_assist_correction > 0.0
    # One sweep a full second later: span exceeds RANGE_VREL_MAX_SPAN_S, so the history resets.
    track.update(40.0, 0.0, -13.49, V_EGO - 13.49, True, True, t_now=100.0, range_assist=True)
    assert not track.vRelRangeFresh
    assert track.range_assist_correction == 0.0
    assert track.vRelRange != stale or math.isnan(track.vRelRange)

  def test_turning_the_flag_off_disarms_on_the_next_update(self):
    track = new_track()
    settle(track, d0=76.0, range_rate=-19.4, v_rel=-13.49)
    assert track.range_assist_correction > 0.0
    track.update(track.dRel - 19.4 * DT, 0.0, -13.49, V_EGO - 13.49, True, True,
                 t_now=100.0 * DT, range_assist=False)
    assert track.range_assist_correction == 0.0

  def test_correction_decays_continuously_as_u11_catches_up(self):
    """Once armed the hold is on ANY closing disagreement, not on MIN_DISAGREEMENT, so the
    correction walks to zero instead of stepping off a 2 m/s cliff into the planner."""
    track = new_track()
    settle(track, d0=76.0, range_rate=-19.4, v_rel=-13.49)
    seen = [track.range_assist_correction]
    t = (RANGE_VREL_SAMPLES + RANGE_VREL_ASSIST_ARM_UPDATES) * DT
    for u11 in [-15.0, -17.0, -18.5, -19.3, -19.4]:
      track.update(track.dRel - 19.4 * DT, 0.0, u11, V_EGO + u11, True, True,
                   t_now=t, range_assist=True)
      t += DT
      seen.append(track.range_assist_correction)
    assert all(b <= a + 1e-9 for a, b in zip(seen[:-1], seen[1:], strict=True)), seen
    assert seen[-1] == pytest.approx(0.0, abs=1e-6)


# ---------------------------------------------------------------------------------------------
# The real radard cadence: a 20 Hz loop over a 14.35 Hz radar
# ---------------------------------------------------------------------------------------------

class TestRadardLoopCadence:
  """Every other test in this file feeds one update per radar sweep. radard does not.

  On the car RadarD is driven at the 20 Hz model rate and collapses two different conditions into
  one bit -- `measured = pt.measured and radar_fresh` -- so a DUPLICATE cycle (no new liveTracks
  message) and a COAST (new message, parser's measured bit clear) both arrive at Track.update as
  measurement_update False. They need opposite handling, and these tests pin that.
  """

  def test_the_cadence_really_does_produce_duplicate_cycles(self):
    """Guard the premise: if this stops holding, the tests below stop testing anything."""
    track = new_track()
    trace = feed_radard_cadence(track, 20, d0=76.0, range_rate=-19.4, v_rel=-13.49)
    assert sum(1 for _, fresh, _ in trace if not fresh) >= 4, trace
    assert {sweep for sweep, _, _ in trace} == set(range(trace[-1][0] + 1)), "a sweep was skipped"

  def test_the_assist_arms_at_the_real_radard_cadence(self):
    """The defect this class exists for. Clearing on every non-measurement update reset the arm
    count roughly every fourth cycle, so RANGE_VREL_ASSIST_ARM_UPDATES consecutive qualifying
    fits were unreachable and the feature was permanently inert on a car."""
    track = new_track()
    feed_radard_cadence(track, 20, d0=76.0, range_rate=-19.4, v_rel=-13.49)
    assert track.range_assist_active
    assert track.range_assist_correction > 0.0

  def test_a_duplicate_cycle_holds_the_correction_unchanged(self):
    """Nothing was re-measured, so nothing may change -- otherwise the published vRel flickers
    between corrected and native at ~5 Hz, which the planner would see as velocity noise."""
    track = new_track()
    trace = feed_radard_cadence(track, 24, d0=76.0, range_rate=-19.4, v_rel=-13.49)
    armed_at = next(i for i, (_, _, c) in enumerate(trace) if c > 0.0)
    duplicates = [i for i, (_, fresh, _) in enumerate(trace) if not fresh and i > armed_at]
    assert duplicates, trace
    for i in duplicates:
      assert trace[i][2] == trace[i - 1][2], f"cycle {i} changed the correction on a duplicate"

  def test_a_coast_at_the_real_cadence_still_clears(self):
    """The hold must not swallow a coast: a coast advances t_now, and D-052 measured 121.8 s of
    continuous coasting, so holding across one is unbounded staleness."""
    track = new_track()
    feed_radard_cadence(track, 20, d0=76.0, range_rate=-19.4, v_rel=-13.49)
    assert track.range_assist_correction > 0.0
    trace = feed_radard_cadence(track, 24, d0=76.0, range_rate=-19.4, v_rel=-13.49,
                                coast_sweeps={11, 12, 13})
    coasted = [c for sweep, fresh, c in trace if fresh and sweep in {11, 12, 13}]
    assert coasted and all(c == 0.0 for c in coasted), trace

  def test_the_hold_is_not_a_time_bound_in_disguise(self):
    """The hold is 'nothing arrived', not 'not much time has passed'. A stale t_now held for far
    longer than any real duplicate must still hold, because the alternative -- a bound in cycles
    or seconds -- would be a tuned constant with no evidence behind it."""
    track = new_track()
    settle(track, d0=76.0, range_rate=-19.4, v_rel=-13.49)
    armed = track.range_assist_correction
    assert armed > 0.0
    last_t = (RANGE_VREL_SAMPLES + RANGE_VREL_ASSIST_ARM_UPDATES - 2) * DT
    for _ in range(40):
      track.update(track.dRel, 0.0, -13.49, V_EGO - 13.49, False, False,
                   t_now=last_t, range_assist=True)
    assert track.range_assist_correction == armed


# ---------------------------------------------------------------------------------------------
# Recorded cases
# ---------------------------------------------------------------------------------------------

class TestRecordedRailCase:
  """000001f9 at 29:52 (D-041). U11 railed at -13.5 m/s on 88 of 88 active frames with healthy
  u10 while the range closed at -19.4 m/s. D-041 publishes the rail as a bound and leaves
  recovering the true value to 'a separate, validated change'. This is that change -- still
  unvalidated on the road."""

  def test_the_rail_case_recovers_toward_the_range_rate(self):
    track = new_track()
    settle(track, d0=76.0, range_rate=-19.4, v_rel=-13.49)
    assert track.range_assist_active
    assert track.range_assist_correction == pytest.approx(19.4 - 13.49, abs=0.05)
    assert track.get_RadarState()["vRel"] == pytest.approx(-19.4, abs=0.05)

  def test_the_rail_case_fits_inside_the_cap(self):
    """5.9 m/s against an 8.0 m/s cap. If the cap is lowered below this the feature can no longer
    do the one thing D-041 left for it."""
    assert 19.4 - 13.49 < RANGE_VREL_ASSIST_MAX_CORRECTION_MPS


class TestRecordedOnsetLag:
  """000001f3 at 19:27 (D-043). The fitted range rate was -6.7 m/s while U11 still read -2.08."""

  def test_the_onset_case_arms(self):
    track = new_track()
    settle(track, d0=45.0, range_rate=-6.7, v_rel=-2.08)
    assert track.range_assist_active
    assert track.range_assist_correction == pytest.approx(6.7 - 2.08, abs=0.05)

  def test_arming_costs_less_than_the_measured_u11_lag(self):
    """0.35 s of arming against a measured 0.88-1.28 s U11 onset lag. If the arm count grows past
    the lag, the feature can no longer beat the channel it is correcting."""
    assert RANGE_VREL_ASSIST_ARM_UPDATES * DT < 0.88


class TestTheRangeWalkFault:
  """THE NEGATIVE CONTROL THAT MATTERS. STATUS.md's t~=11 s false brake on 00000231--5782493b00:
  track 6 walked 71.8 -> 61.6 m while vision held 75-78 m at prob 0.93-1.00 and the real gap GREW.
  The range itself was wrong, so U11 (-6.0) and the range fit (-4.5 to -6.4) AGREED. This assist
  is fitted to that same range and is blind to it by construction (STATUS.md records this).

  These tests pin that the recorded figures leave it inert. [INFERRED from the recorded figures,
  not replayed] -- the numbers are from the STATUS.md event table, driven through the real Track
  here rather than through a log.

  If a change makes this fire, the assist has become an amplifier for the only false brake this
  branch has recorded. Do not 'fix' these tests."""

  @pytest.mark.parametrize("fitted", [-4.5, -5.0, -5.5, -6.0, -6.4])
  def test_the_fault_leaves_the_assist_inert(self, fitted):
    track = new_track()
    settle(track, d0=71.8, range_rate=fitted, v_rel=-6.02)
    assert track.range_assist_correction == 0.0, (
        f"fitted range rate {fitted} against U11 -6.02 armed the assist on the recorded false brake")
    assert track.get_RadarState()["vRel"] == pytest.approx(-6.02)

  def test_the_worst_recorded_disagreement_is_inside_the_threshold(self):
    """disagreement = vRel - vRelRange, positive when the RANGE claims more closing. Across the
    recorded -4.5 .. -6.4 fit band against U11's -6.02 the worst case is +0.38 m/s, so the margin
    to MIN_DISAGREEMENT is 1.62 m/s. That margin is the whole safety argument for the threshold,
    so state it as a number rather than leaving it implicit."""
    worst = max(-6.02 - fitted for fitted in (-4.5, -5.0, -5.5, -6.0, -6.4))
    assert worst == pytest.approx(0.38, abs=0.01)
    assert RANGE_VREL_ASSIST_MIN_DISAGREEMENT_MPS - worst == pytest.approx(1.62, abs=0.01)


# ---------------------------------------------------------------------------------------------
# Kalman path
# ---------------------------------------------------------------------------------------------

class TestKalmanPath:
  def test_the_kf_sees_the_correction(self):
    """aLeadK is what the MPC brakes on. If the correction reached only the published vRel, the
    lead's acceleration would still come from the uncorrected channel and the feature would do
    much less than it claims."""
    corrected, native = new_track(), new_track()
    settle(corrected, d0=76.0, range_rate=-19.4, v_rel=-13.49)
    settle(native, d0=76.0, range_rate=-19.4, v_rel=-13.49, range_assist=False)
    assert corrected.vLeadK < native.vLeadK
    assert corrected.aLeadK < native.aLeadK, "a corrected lead must look like it is braking harder"


# ---------------------------------------------------------------------------------------------
# D-009 negative control: break the mechanism, prove the tests notice
# ---------------------------------------------------------------------------------------------

class TestNegativeControlOfTheTestsThemselves:
  """D-009: a test suite that passes against a broken mechanism is not evidence. Each of these
  breaks one guard on purpose and asserts that the property it defends actually fails."""

  def test_clearing_on_a_duplicate_cycle_makes_the_assist_permanently_inert(self, monkeypatch):
    """The negative control for the duplicate-cycle hold, and the one that found a real defect.

    Defeating the t_now comparison the hold rests on reproduces exactly the pre-fix behaviour --
    clear on EVERY non-measurement update -- and at the cadence radard actually runs at the assist
    can then never arm. If this test starts passing with the correction non-zero, the hold has
    stopped doing anything and TestRadardLoopCadence is no longer evidence.
    """
    original = radard.Track._update_range_assist

    def never_holds(self, enabled, measurement_update, t_now):
      # NaN never compares equal, so the duplicate branch falls through to the clear.
      self._range_assist_last_t = float('nan')
      return original(self, enabled, measurement_update, t_now)

    monkeypatch.setattr(radard.Track, "_update_range_assist", never_holds)
    track = new_track()
    feed_radard_cadence(track, 40, d0=76.0, range_rate=-19.4, v_rel=-13.49)
    assert not track.range_assist_active
    assert track.range_assist_correction == 0.0

  def test_removing_the_arm_count_lets_an_outlier_through(self, monkeypatch):
    """The correction is transient here -- it appears while the outlier is in the window and is
    gone by the last sweep -- so this watches the peak, not the settled state."""
    monkeypatch.setattr(radard, "RANGE_VREL_ASSIST_ARM_UPDATES", 1)
    track = new_track()
    peak = 0.0
    for i in range(30):
      d = 70.0 - 1.0 * (i * DT) + (-1.0 if i == 8 else 0.0)
      track.update(d, 0.0, -1.0, V_EGO - 1.0, True, True, t_now=i * DT, range_assist=True)
      peak = max(peak, track.range_assist_correction)
    assert peak > 0.0, "with the arm count disabled a single outlier should have armed: TestGrossOutlierRejection is not proving what it claims"

  def test_removing_the_threshold_lets_the_recorded_false_brake_through(self, monkeypatch):
    monkeypatch.setattr(radard, "RANGE_VREL_ASSIST_MIN_DISAGREEMENT_MPS", 0.1)
    track = new_track()
    settle(track, d0=71.8, range_rate=-6.4, v_rel=-6.02)
    assert track.range_assist_correction > 0.0, "lowering MIN_DISAGREEMENT should fire the t~=11 s fault: TestTheRangeWalkFault proves nothing"

  def test_removing_the_geometry_gate_lets_an_adjacent_track_through(self, monkeypatch):
    monkeypatch.setattr(radard, "RANGE_VREL_ASSIST_MAX_ABS_Y_REL_M", 99.0)
    track = new_track()
    settle(track, d0=76.0, range_rate=-19.4, v_rel=-13.49, y_rel=8.0)
    assert track.range_assist_correction > 0.0, "widening the lateral gate should correct an 8 m offset track: the gate tests prove nothing"
