"""Tests for tools/bosch_a_lifecycle_report.py.

The census is only worth running if a "break" in its output is a sweep on which the REAL parser
saw a lifecycle discontinuity. So these tests drive the real RadarInterface with the Bosch-A test
module's own frame builders and check the census against what the parser actually did -- its
cleared range history -- rather than against the census's own arithmetic.

Each detector has a negative control (D-009): a continuous identity must produce no break and no
seamless-reuse candidate, or "found one" and "always finds one" look identical.
"""

import importlib.util
import os

from opendbc.car.honda.tests.test_bosch_a_radar import make_radar_interface, sweep

_HERE = os.path.dirname(__file__)
_SCRIPT = os.path.normpath(os.path.join(_HERE, "..", "..", "bosch_a_lifecycle_report.py"))

SWEEP_NS = int((1.0 / 14.35) * 1e9)
TRACK_ID = 7


def _load():
  spec = importlib.util.spec_from_file_location("bosch_a_lifecycle_report", _SCRIPT)
  module = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(module)
  return module


report = _load()


def _feed(census, i, life, angle_raw=1024):
  """One sweep of a slowly closing object with a qualified U11 -- the same real-target pattern the
  route report's tests use. Without AUX the parser has no trusted velocity and coasts every sweep,
  which never matures a point and so cannot exercise lifecycle handling at all."""
  t = i * SWEEP_NS
  (t_nanos, frames), = sweep(0, i % 16, 0x7, 1000 - 3 * i, angle_raw, life, t, with_aux=True,
                             track_id=TRACK_ID, direct_vrel_raw=800, direct_vrel_uncertainty_raw=40)
  census.feed(t_nanos, t * 1e-9, frames)


def test_same_incarnation_rule_wraps_like_the_parser():
  assert report.is_same_incarnation(3, 100, 4, 102)
  assert not report.is_same_incarnation(3, 100, 4, 180)
  # A saturated counter cannot advance, and the parser treats that as a continuation.
  assert report.is_same_incarnation(3, 0xFFE, 4, 0xFFE)
  # Frozen anywhere else is still a break.
  assert not report.is_same_incarnation(3, 4000, 4, 4000)
  # Both counters wrap: frame index is 4 bits, lifecycle 12 bits.
  assert report.is_same_incarnation(15, 4094, 0, 0)
  assert report.is_same_incarnation(14, 4094, 0, 2)


def test_continuous_identity_is_not_a_break():
  """Negative control: a well-formed identity is all continuations."""
  census = report.LifecycleCensus(make_radar_interface())
  for i in range(20):
    _feed(census, i, life=1 + 2 * i)
  census.finish()
  assert census.update_errors == 0
  assert census.breaks == []
  assert census.fresh == 1
  assert census.continuations == 19
  assert census.candidates == []


def test_break_matches_the_parser_and_counts_the_absence():
  ri = make_radar_interface()
  census = report.LifecycleCensus(ri)
  for i in range(10):
    _feed(census, i, life=1 + 2 * i)
  history_before = len(ri._tracks[TRACK_ID].samples)
  assert history_before > 1

  # New object under the same CAN identity: lifecycle restarts.
  for k, i in enumerate(range(10, 20)):
    _feed(census, i, life=500 + 2 * k)
    if k == 0:
      # The parser itself treated this sweep as a new incarnation: its range history was cleared
      # (and may hold at most the one new sample). This is what makes the census's break real.
      assert len(ri._tracks[TRACK_ID].samples) <= 1
  census.finish()

  assert len(census.breaks) == 1
  ev = census.breaks[0]
  assert ev["trackId"] == TRACK_ID
  assert abs(ev["t"] - 10 * SWEEP_NS * 1e-9) < 1e-9
  assert ev["reappeared"]
  # D-049: the identity is missing from RadarData for exactly one sweep.
  assert ev["absentSweeps"] == 1


def test_lateral_jump_without_break_is_a_candidate():
  census = report.LifecycleCensus(make_radar_interface(), lateral_jump_m=1.5)
  for i in range(8):
    _feed(census, i, life=1 + 2 * i, angle_raw=1024)
  assert census.candidates == []
  for i in range(8, 12):
    _feed(census, i, life=1 + 2 * i, angle_raw=1024 + 400)
  census.finish()
  assert census.breaks == []
  assert len(census.candidates) == 1
  assert census.candidates[0]["trackId"] == TRACK_ID
  assert census.candidates[0]["dy"] > 1.5


def test_mark_isolated_separates_single_steps_from_sweeps():
  sweep_run = [{"t": 10.0 + 0.07 * k, "trackId": 32} for k in range(5)]
  single = {"t": 20.0, "trackId": 34}
  other_track_nearby = {"t": 10.1, "trackId": 99}
  cands = [*sweep_run, single, other_track_nearby]
  report.mark_isolated(cands, window_s=0.3)
  assert not any(c["isolated"] for c in sweep_run)
  assert single["isolated"]
  # Negative control: proximity on a DIFFERENT track does not de-isolate.
  assert other_track_nearby["isolated"]


def test_saturated_counter_is_not_a_break_and_keeps_publishing():
  """The parser's saturation carve-out: LIFECYCLE_RAW pins at 0xFFE after ~137 s of tracking.

  Counting those sweeps as breaks is what made 00000232--fc8dad0d18 read as 1,991 lost identities
  for an object the parser was publishing normally.
  """
  ri = make_radar_interface()
  census = report.LifecycleCensus(ri)
  for i in range(6):
    _feed(census, i, life=0xFFE - 10 + 2 * i)   # counts up into saturation
  for i in range(6, 20):
    _feed(census, i, life=0xFFE)                # and pins there
  census.finish()
  assert census.breaks == []
  assert census.continuations == 19
  assert ri.pts[TRACK_ID].measured                # still published on the last sweep


def test_chronic_break_is_one_run_not_n_permanent_losses():
  """Frozen but UNSATURATED counter: a real discontinuity on every sweep."""
  ri = make_radar_interface()
  census = report.LifecycleCensus(ri)
  for i in range(6):
    _feed(census, i, life=3990 + 2 * i)      # counts up to 4000
  for i in range(6, 20):
    _feed(census, i, life=4000)              # then freezes below saturation
  census.finish()
  assert len(census.breaks) >= 13
  # Every break but the last was superseded by the next -- none is a "never reappeared" loss.
  assert sum(1 for b in census.breaks if not b["reappeared"] and not b["superseded"]) == 1
  runs = report.chronic_runs(census.breaks)
  assert len(runs) == 1
  assert runs[0]["breaks"] == len(census.breaks)
  assert runs[0]["lives"] == [4000]
  # The parser never republished it once frozen -- the failure this run exposes.
  assert not any(b["reappeared"] for b in census.breaks)


def test_chronic_runs_negative_control_separates_distant_breaks():
  breaks = [{"t": 1.0, "trackId": 5}, {"t": 9.0, "trackId": 5}, {"t": 1.07, "trackId": 6}]
  runs = report.chronic_runs(breaks, max_gap_s=0.25)
  assert sorted((r["trackId"], r["breaks"]) for r in runs) == [(5, 1), (5, 1), (6, 1)]


def test_attach_brakes_uses_window_and_device_lead():
  breaks = [{"t": 10.0, "trackId": 6}, {"t": 30.0, "trackId": 9}]
  brakes = [(9.0, -2.0), (11.5, -2.9), (40.0, -3.0)]
  leads = [(0.0, 6), (20.0, 4)]
  report.attach_brakes(breaks, brakes, leads, window_s=2.0)
  assert breaks[0]["hardestBrakeNearby"] == -2.9
  assert breaks[0]["wasDeviceLead"]
  assert breaks[1]["hardestBrakeNearby"] is None
  assert not breaks[1]["wasDeviceLead"]


def test_percentiles_interpolate():
  out = report.percentiles([0.0, 1.0, 2.0, 3.0, 4.0])
  assert out["n"] == 5 and out["max"] == 4.0 and out["p50"] == 2.0
  assert report.percentiles([]) == {"n": 0}
