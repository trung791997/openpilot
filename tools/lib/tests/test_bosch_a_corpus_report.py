"""Regression tests for tools/bosch_a_corpus_report.py.

Two jobs, and the first matters more than the second.

1. **Pin the geometry conventions against production.** The corpus report's whole purpose is
   to produce numbers that get quoted in STATUS.md and used to justify (or refute) threshold
   changes. A sign error in the lateral residual would not crash anything -- it would quietly
   produce a plausible-looking distribution that is wrong, and the previous corpus numbers are
   already unreproducible, so there would be nothing to check it against. `track_matches_vision`
   compares `yRel + lead.y[0]`, a SUM, because radar `yRel` is car-frame left-positive and model
   `y` is device-frame. Differencing instead would report ~2x the residual on an off-centre track
   and ~0 on a genuinely mismatched one -- exactly backwards. So these tests find the boundary
   where the real matcher flips and assert the report's functions read the tolerance there.

2. **Prove the arithmetic and the extraction**, because no route is reachable from an agent
   session. The statistics are checked against hand-computable inputs rather than against the
   tool's own output (AGENTS.md §5: a reference derived from the thing under test is circular),
   and the extraction is driven end to end over a synthetic segment of real capnp Events. That
   still does not validate the tool on a real drive -- message rates, dropouts and clock skew
   are exactly what a synthetic segment does not have.
"""

import math
import os
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0, os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "..", "..")))

from openpilot.selfdrive.controls import radard
from openpilot.tools.bosch_a_corpus_report import (
  attach_override,
  centred_derivative,
  contiguous_runs,
  cross_correlation_lag,
  u11_range_rate_pairs,
  headway_s,
  lateral_residual_m,
  pearson,
  percentiles,
  range_gap_m,
  summarise,
  ttc_s,
)

# The tolerances radard actually passes for a Bosch-A strict match, per STATUS.md's account of
# the inverse failure mode: range gated loosely, lateral tightly.
MATCH_KW = dict(dist_scale=0.25, dist_floor=5.0, vel_limit=10.0, y_std_scale=1.0, y_floor=1.0)


def make_track(d_rel, y_rel, v_rel=0.0):
  track = radard.Track(1, v_rel, radard.KalmanParams(radard.HONDA_BOSCH_A_RADAR_TS))
  track.update(d_rel, y_rel, v_rel, v_rel, True, True)
  return track


def make_lead(x, y, v=0.0, y_std=0.2):
  return SimpleNamespace(x=[x], y=[y], v=[v], yStd=[y_std], prob=1.0)


# --- 1. conventions pinned against the production matcher -------------------------------------

def test_range_gap_matches_the_matcher_boundary():
  # At 80 m the range tolerance is max(80*0.25, 5.0) = 20 m. Walk the radar range outward until
  # the real matcher stops accepting, and assert the report reads exactly that tolerance there.
  vision_x = 80.0
  offset = vision_x - radard.RADAR_TO_CAMERA
  tolerance = max(abs(offset) * MATCH_KW["dist_scale"], MATCH_KW["dist_floor"])

  inside = make_track(offset - (tolerance - 0.01), 0.0)
  outside = make_track(offset - (tolerance + 0.01), 0.0)
  lead = make_lead(vision_x, 0.0)

  assert radard.track_matches_vision(inside, lead, 0.0, **MATCH_KW)
  assert not radard.track_matches_vision(outside, lead, 0.0, **MATCH_KW)
  assert abs(range_gap_m(inside.dRel, vision_x)) == pytest.approx(tolerance - 0.01, abs=1e-6)
  assert abs(range_gap_m(outside.dRel, vision_x)) == pytest.approx(tolerance + 0.01, abs=1e-6)


def test_range_gap_is_positive_when_vision_reads_longer():
  # STATUS.md's pooled median is +4.0 m in the sense "vision reads longer than radar". Pin the
  # sign, because the whole distribution changes meaning if it is inverted.
  assert range_gap_m(60.0, 70.0) > 0
  assert range_gap_m(70.0, 60.0) < 0
  assert range_gap_m(60.0, 60.0 + radard.RADAR_TO_CAMERA) == pytest.approx(0.0, abs=1e-9)


def test_lateral_residual_is_a_sum_not_a_difference():
  # The case that separates the two: a track well off boresight whose vision counterpart is off
  # the other way. In the matcher's convention these describe the SAME object (residual 0.5 m,
  # inside the 1.0 m floor); differencing would call it 3.5 m away and reject it.
  track = make_track(50.0, 2.0)
  lead = make_lead(50.0 + radard.RADAR_TO_CAMERA, -1.5)

  assert lateral_residual_m(2.0, -1.5) == pytest.approx(0.5)
  assert abs(2.0 - (-1.5)) == pytest.approx(3.5)   # what the wrong convention would report
  assert radard.track_matches_vision(track, lead, 0.0, **MATCH_KW), "matcher accepts it, so the report must agree"


def test_lateral_residual_matches_the_matcher_boundary():
  # yStd 0.2 -> tolerance max(1.0, 1.0*0.2) = 1.0 m, the tight gate STATUS.md blames for good
  # tracks being thrown away at a 4.3 m median lateral residual.
  vision_y = -1.5
  for delta, expected in ((-0.01, True), (+0.01, False)):
    y_rel = (1.0 + delta) - vision_y
    track = make_track(50.0, y_rel)
    lead = make_lead(50.0 + radard.RADAR_TO_CAMERA, vision_y)
    assert radard.track_matches_vision(track, lead, 0.0, **MATCH_KW) is expected
    assert abs(lateral_residual_m(y_rel, vision_y)) == pytest.approx(1.0 + delta, abs=1e-6)


# --- 2. the statistics ------------------------------------------------------------------------

def test_percentiles_interpolate_against_hand_computed_values():
  values = list(range(1, 101))            # 1..100, so p50 sits between 50 and 51
  out = percentiles(values, (5, 50, 95))
  assert out["p50"] == pytest.approx(50.5)
  assert out["p5"] == pytest.approx(5.95)
  assert out["p95"] == pytest.approx(95.05)


def test_percentiles_on_empty_and_single_inputs():
  # Empty must not silently report 0.0 -- a zero percentile reads as a measurement.
  assert percentiles([]) == {}
  single = percentiles([7.5], (5, 50, 95))
  assert set(single.values()) == {7.5}


def test_summarise_reports_n_zero_rather_than_fabricating_stats():
  assert summarise([]) == {"n": 0}
  s = summarise([1.0, 2.0, 3.0])
  assert (s["n"], s["min"], s["max"], s["mean"]) == (3, 1.0, 3.0, 2.0)


def test_pearson_known_values():
  assert pearson([1, 2, 3, 4], [2, 4, 6, 8]) == pytest.approx(1.0)
  assert pearson([1, 2, 3, 4], [8, 6, 4, 2]) == pytest.approx(-1.0)


def test_pearson_is_none_when_undefined_rather_than_zero():
  # A constant series carries no information; reporting 0.0 would read as proven independence,
  # which is the kind of claim AGENTS.md §5 exists to stop.
  assert pearson([1, 1, 1, 1], [1, 2, 3, 4]) is None
  assert pearson([1.0], [2.0]) is None
  assert pearson([1, 2, 3], [1, 2]) is None


def test_centred_derivative_recovers_a_known_slope_without_offset():
  times = [i * 0.1 for i in range(6)]
  values = [3.0 * t + 5.0 for t in times]          # slope 3.0
  out = centred_derivative(times, values)
  assert len(out) == 4                              # interior samples only
  for t, rate in out:
    assert rate == pytest.approx(3.0)
    assert t in times[1:-1]                         # timestamps are the sample's own, not shifted


def test_centred_derivative_handles_short_and_duplicate_timestamps():
  assert centred_derivative([0.0, 0.1], [1.0, 2.0]) == []
  assert centred_derivative([0.0, 0.1, 0.0], [1.0, 2.0, 3.0]) == []   # zero span is dropped


def test_cross_correlation_recovers_a_known_negative_lag():
  # Build b as a wave and a as the SAME wave delayed by 3 samples, which is the shape STATUS.md
  # reports for U11 against the range derivative (peak at lag -4). A delayed series must come
  # back as a negative lag; a sign slip here would invert "U11 lags" into "U11 leads".
  base = [math.sin(i * 0.4) for i in range(60)]
  delay = 3
  b = base
  a = [0.0] * delay + base[:-delay]
  found = cross_correlation_lag(a, b, max_lag=8)
  assert found is not None
  lag, r = found
  assert lag == -delay
  assert r == pytest.approx(1.0, abs=1e-6)


def test_cross_correlation_returns_none_when_too_short():
  assert cross_correlation_lag([1.0, 2.0], [1.0, 2.0], max_lag=5) is None


def test_headway_and_ttc_decline_to_answer_rather_than_dividing_by_zero():
  assert headway_s(50.0, 25.0) == pytest.approx(2.0)
  assert headway_s(50.0, 0.0) is None               # stopped: headway is meaningless, not infinite
  assert ttc_s(50.0, -10.0) == pytest.approx(5.0)
  assert ttc_s(50.0, +2.0) is None                  # opening: no contact to come
  assert ttc_s(50.0, 0.0) is None


# --- 3. the extraction path, against a synthetic segment ---------------------------------------
# No route is reachable from an agent session, but `_LogFileReader` reads concatenated capnp
# Events, so a segment can be built here with every expected answer computable by hand. This
# exercises the message-pairing and attribution logic; it does NOT validate the tool against a
# real drive, and the report says so in its own docstring.

def _event(which, t_ns, fill):
  from cereal import messaging
  msg = messaging.new_message(which)
  msg.logMonoTime = t_ns
  fill(getattr(msg, which))
  return msg.to_bytes()


def write_segment(path, n=5, *, vision_x=64.0, vision_prob=1.0, d_rel=60.0, v_rel=-5.0,
                  y_rel=0.3, accel=-2.0, enabled=True, radar_lead=True, v_ego=20.0,
                  measured=True, experimental=None, gas=True):
  """A segment where every statistic is hand-computable. Returns the path written."""
  exps = list(experimental) if experimental is not None else [False] * n
  assert len(exps) == n, "one experimentalMode value per frame"

  def cs(m):
    m.vEgo = v_ego
    m.gasPressed = gas
  def model(m):
    lead = m.init('leadsV3', 1)[0]
    lead.prob = vision_prob
    lead.x, lead.y, lead.v, lead.yStd = [vision_x], [0.0], [15.0], [0.2]
  def sds(m):
    m.experimentalMode = exps[frame["i"]]
    m.enabled = enabled
  def plan(m):
    m.longitudinalPlanSource = 'lead0'
  accels = list(accel) if isinstance(accel, (list, tuple)) else [accel] * n
  assert len(accels) == n, "one accel per frame"
  frame = {"i": 0}

  def cc(m):
    m.enabled = enabled
    m.actuators.accel = accels[frame["i"]]
  def lt(m):
    p = m.init('points', 1)[0]
    p.trackId, p.dRel, p.yRel, p.vRel, p.measured = 7, d_rel, y_rel, v_rel, measured
  def rs(m):
    lead = m.leadOne
    lead.status, lead.radar, lead.radarTrackId = True, radar_lead, 7
    lead.dRel, lead.yRel, lead.vRel = d_rel, y_rel, v_rel

  blobs = []
  for i in range(n):
    t = 10**9 + int(i * 0.05 * 1e9)
    # carState/model/state first: the report pairs each radarState against the most recent of
    # the others, so ordering within a cycle is part of what is under test here.
    frame["i"] = i
    for which, fill in (('carState', cs), ('modelV2', model), ('selfdriveState', sds),
                        ('longitudinalPlan', plan), ('carControl', cc), ('liveTracks', lt),
                        ('radarState', rs)):
      blobs.append(_event(which, t, fill))
  with open(path, 'wb') as f:
    f.write(b''.join(blobs))
  return str(path)


def test_analyse_segment_extracts_hand_computable_values(tmp_path):
  from openpilot.tools.bosch_a_corpus_report import analyse_segment
  report = analyse_segment(write_segment(tmp_path / "rlog"))

  assert report["frames"] == 5
  assert report["radarLeadPct"] == pytest.approx(100.0)
  assert report["visionLeadPct"] == pytest.approx(100.0)
  assert report["experimentalPct"] == pytest.approx(0.0)
  # (64.0 - RADAR_TO_CAMERA) - 60.0
  assert report["gap"]["p50"] == pytest.approx(64.0 - radard.RADAR_TO_CAMERA - 60.0, abs=1e-4)
  assert report["lateralResidual"]["p50"] == pytest.approx(0.3, abs=1e-4)

  # Five contiguous over-threshold frames are ONE brake run, not five events.
  assert len(report["hardBrakes"]) == 1
  brake = report["hardBrakes"][0]
  assert brake["runFrames"] == 5
  assert brake["runDuration"] == pytest.approx(4 * 0.05, abs=1e-6)
  assert brake["accel"] == pytest.approx(-2.0)
  assert brake["headway"] == pytest.approx(60.0 / 20.0)      # dRel / vEgo
  assert brake["ttc"] == pytest.approx(60.0 / 5.0)           # dRel / -vRel
  assert brake["source"] == "lead0"
  assert brake["override"] is True                            # gasPressed
  assert brake["radarTrackId"] == 7


def test_analyse_segment_ignores_a_vision_lead_below_the_probability_floor(tmp_path):
  # A low-probability vision lead is not evidence, so it must not enter the gap distribution --
  # otherwise the pooled percentiles quoted in STATUS.md would be diluted by frames where the
  # model was not actually reporting a lead.
  from openpilot.tools.bosch_a_corpus_report import analyse_segment
  report = analyse_segment(write_segment(tmp_path / "rlog", vision_prob=0.3))
  assert report["gap"]["n"] == 0
  assert report["visionLeadPct"] == pytest.approx(0.0)
  assert report["radarLeadPct"] == pytest.approx(100.0)       # the radar lead is still there


def test_analyse_segment_does_not_count_a_vision_lead_as_a_radar_lead(tmp_path):
  # leadOne.status with radar=False is the vision fallback. Counting it as a radar lead would
  # invent radar-lead occupancy on exactly the segments where the radar was NOT driving.
  from openpilot.tools.bosch_a_corpus_report import analyse_segment
  report = analyse_segment(write_segment(tmp_path / "rlog", radar_lead=False))
  assert report["radarLeadPct"] == pytest.approx(0.0)
  assert report["gap"]["n"] == 0
  assert report["hardBrakes"] == []


def test_analyse_segment_ignores_a_brake_while_disengaged(tmp_path):
  # Accel below the threshold while not engaged is the driver braking, not a commanded brake.
  from openpilot.tools.bosch_a_corpus_report import analyse_segment
  report = analyse_segment(write_segment(tmp_path / "rlog", enabled=False))
  assert report["hardBrakes"] == []


def test_hard_brakes_are_grouped_into_runs_and_reported_at_their_peak(tmp_path):
  """The count STATUS.md quotes is runs, not frames, and each row is the run's hardest moment.

  Reporting per frame would turn a corpus's four brake events into hundreds of rows and make
  any "only N hard brakes" claim meaningless -- the exact situation D-042 warns about, where a
  threshold looks supported by a sample size that was never really there.
  """
  from openpilot.tools.bosch_a_corpus_report import analyse_segment
  # brake, release, brake harder: two runs, peaks -2.0 then -3.4.
  accels = [-2.0, -1.9, -2.0] + [-0.2, -0.1] + [-3.0, -3.4, -3.1]
  report = analyse_segment(write_segment(tmp_path / "rlog", n=len(accels), accel=accels))

  runs = report["hardBrakes"]
  assert len(runs) == 2
  assert [r["runFrames"] for r in runs] == [3, 3]
  assert runs[0]["accel"] == pytest.approx(-2.0)
  assert runs[1]["accel"] == pytest.approx(-3.4)
  # the row carries the peak frame's own timestamp, not the run's start
  assert runs[1]["t"] == pytest.approx(6 * 0.05, abs=1e-6)


# --- fixes made when the harness first met a real drive (00000231--5782493b00) ---------------------

def test_contiguous_runs_split_at_a_gap():
  samples = [(0.0,), (0.07,), (0.14,), (0.40,), (0.47,)]
  assert contiguous_runs(samples, 0.1) == [[(0.0,), (0.07,), (0.14,)], [(0.40,), (0.47,)]]
  assert contiguous_runs([], 0.1) == []


def test_u11_pairs_stay_aligned_past_a_skipped_zero_dt_sample():
  """Regression: the old pairing enumerated centred_derivative() against vs[i + 1], so a skipped
  zero-dt sample shifted every later U11 onto the wrong range rate."""
  run = [(0.0, 10.0, -1.0), (0.0, 10.0, -2.0), (0.0, 10.0, -3.0), (0.1, 9.0, -4.0), (0.2, 8.0, -5.0)]
  u11, rates = u11_range_rate_pairs(run)
  # i=1 skipped (dt 0); i=2 -> (9-10)/0.1; i=3 -> (8-10)/0.2
  assert u11 == [-3.0, -4.0]
  assert rates == [pytest.approx(-10.0), pytest.approx(-10.0)]


def test_override_counts_a_press_after_the_peak_but_not_one_before_the_run():
  runs = [{"tStart": 1.0, "tEnd": 1.5}]
  assert attach_override([dict(runs[0])], [2.3])[0]["overrideInRun"]        # within 1 s of the end
  assert not attach_override([dict(runs[0])], [2.6])[0]["overrideInRun"]    # too late
  assert not attach_override([dict(runs[0])], [0.9])[0]["overrideInRun"]    # before the brake


def test_analyse_segment_clock_ignores_initdata(tmp_path):
  """initData is stamped at logger start. As the origin it put segment 10's false brake at
  t=612.76 instead of ~11.3."""
  from openpilot.tools.bosch_a_corpus_report import analyse_segment
  path = write_segment(tmp_path / "rlog")
  with open(path, "rb") as f:
    body = f.read()
  with open(path, "wb") as f:
    f.write(_event("initData", 0, lambda m: None) + body)   # 1 s before the first real sample
  report = analyse_segment(path)
  assert report["hardBrakes"][0]["t"] == pytest.approx(0.0, abs=1e-6)


def test_vision_occupancy_is_reported_at_both_probability_cuts(tmp_path):
  from openpilot.tools.bosch_a_corpus_report import analyse_segment
  report = analyse_segment(write_segment(tmp_path / "rlog", vision_prob=0.6))
  assert report["visionLeadPct"] == pytest.approx(0.0)
  assert report["visionLeadPct50"] == pytest.approx(100.0)


def test_unmeasured_points_never_enter_the_u11_series(tmp_path):
  from openpilot.tools.bosch_a_corpus_report import analyse_segment
  measured = analyse_segment(write_segment(tmp_path / "a", measured=True))
  coasted = analyse_segment(write_segment(tmp_path / "b", measured=False))
  assert len(measured["_u11"]) == 3          # 5 samples -> 3 interior centred pairs
  assert coasted["_u11"] == []


def test_experimental_switches_and_short_dwells(tmp_path):
  from openpilot.tools.bosch_a_corpus_report import analyse_segment
  report = analyse_segment(write_segment(tmp_path / "rlog", experimental=[False, True, False, False, False]))
  assert report["experimentalSwitches"] == 2
  assert len(report["experimentalShortDwells"]) == 1
  assert report["experimentalShortDwells"][0]["dwell"] == pytest.approx(0.05, abs=1e-6)
  steady = analyse_segment(write_segment(tmp_path / "steady"))
  assert steady["experimentalSwitches"] == 0 and steady["experimentalShortDwells"] == []


def test_override_in_run_negative_control(tmp_path):
  from openpilot.tools.bosch_a_corpus_report import analyse_segment
  assert analyse_segment(write_segment(tmp_path / "a", gas=True))["hardBrakes"][0]["overrideInRun"]
  assert not analyse_segment(write_segment(tmp_path / "b", gas=False))["hardBrakes"][0]["overrideInRun"]


def test_segment_label_uses_the_directory_not_the_rlog_filename(tmp_path):
  from openpilot.tools.bosch_a_corpus_report import analyse_segment, segment_label
  seg = tmp_path / "00000231--5782493b00--7"
  seg.mkdir()
  assert segment_label(str(seg / "rlog")) == "00000231--5782493b00--7"
  assert segment_label(str(seg / "rlog.zst")) == "00000231--5782493b00--7"
  assert analyse_segment(write_segment(seg / "rlog"))["path"] == "00000231--5782493b00--7"
