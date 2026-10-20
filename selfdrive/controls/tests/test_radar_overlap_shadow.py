import math
from types import SimpleNamespace

import pytest

from cereal import car, log, messaging
from openpilot.selfdrive.controls import radard
from openpilot.selfdrive.controls.tests.test_radard_bosch import (
  FakeSubMaster, make_lead, make_radar_data, make_empty_radar_data, make_toggles,
)


@pytest.fixture(autouse=True)
def fixed_toggles(monkeypatch):
  monkeypatch.setattr(radard, 'get_starpilot_toggles', lambda *args: make_toggles())


def make_sm(frame=1):
  sm = FakeSubMaster(frame)
  sm._data['carState'].vEgo = 10.0
  sm._data['modelV2'].leadsV3 = [make_lead(30), make_lead(60)]
  sm._data['modelV2'].position = SimpleNamespace(x=[0.0, 100.0], y=[0.0, 0.0])
  sm.logMonoTime['liveTracks'] += frame * 70_000_000
  return sm


def make_rr(*, center=1.6, left=2.4, right=0.8, measured=True, track_id=1, distance=30.0):
  rr = make_radar_data(track_id=track_id, d_rel=distance, y_rel=center, measured=measured)
  rr.points[0].yLeft = left
  rr.points[0].yRight = right
  return rr


def run(sm=None, rr=None, bosch=True):
  rd = radard.RadarD(honda_bosch_a_radar=bosch)
  rd.starpilot_toggles = make_toggles()
  rd.update(sm or make_sm(), rr or make_rr())
  return rd


def test_published_overlap_and_counterfactual_match():
  rd = run()
  shadow = rd.radar_state.boschAOverlapShadow
  row = shadow.tracks[0]
  assert shadow.radarFresh and row.edgeValid and row.pathValid
  assert shadow.liveTracksMonoTime == make_sm().logMonoTime['liveTracks']
  assert shadow.pathHalfWidth == 1.0
  assert row.pathY == 0.0
  assert row.centerOffset == pytest.approx(1.6)
  assert row.overlapM == pytest.approx(0.2)
  assert row.overlapFraction == pytest.approx(0.125)
  assert row.edgeDistance == pytest.approx(0.8)
  assert row.centerScore == pytest.approx(math.exp(-1.6))
  assert row.edgeScore == pytest.approx(math.exp(-0.8))
  match = shadow.matches[0]
  assert match.evaluated and match.changed
  assert match.existingTrackId == match.selectedTrackId == -1
  assert match.shadowTrackId == 1
  assert not match.candidates[0].centerStrict and match.candidates[0].edgeStrict
  assert match.candidates[0].edgeProbability == pytest.approx(math.exp(-10.8))  # velocity PDF also contributes
  assert not rd.radar_state.leadOne.radar


def test_curve_uses_model_x_origin_and_lateral_sign():
  sm = make_sm()
  # At mesh x=30+1.52, model-right y=-3.152 means radar-left path y=+3.152.
  sm._data['modelV2'].position.y = [0.0, -10.0]
  row = run(sm, make_rr(center=3.152, left=3.652, right=2.652)).radar_state.boschAOverlapShadow.tracks[0]
  assert row.pathY == pytest.approx(3.152)
  assert row.centerOffset == pytest.approx(0, abs=1e-6)
  assert row.overlapFraction == pytest.approx(1.0)
  assert row.edgeScore == 1.0


@pytest.mark.parametrize('x,y', [([], []), ([0], [0]), ([0, 50], [0]), ([0, 0], [0, 0]),
                                  ([50, 0], [0, 0]), ([0, math.nan], [0, 0]), ([0, 50], [0, math.inf]),
                                  ([0, 30], [0, 0]), ([40, 50], [0, 0])])
def test_invalid_or_out_of_horizon_path_is_unavailable(x, y, monkeypatch):
  # Exercise diagnostic input validation separately from the older onpath adopter,
  # whose np.interp assumes equal-length model arrays.
  monkeypatch.setattr(radard, 'ONPATH_RADAR_ADOPT', False)
  sm = make_sm()
  sm._data['modelV2'].position = SimpleNamespace(x=x, y=y)
  row = run(sm).radar_state.boschAOverlapShadow.tracks[0]
  assert row.edgeValid and not row.pathValid
  assert math.isnan(row.edgeScore) and math.isnan(row.overlapFraction)


@pytest.mark.parametrize('left,right,measured', [(0.8, 2.4, True), (1.0, 1.0, True), (2.4, 1.7, True),
                                              (math.nan, 0.8, True), (2.4, math.inf, True), (2.4, 0.8, False)])
def test_invalid_edges_use_centre_match_and_do_not_score_path(left, right, measured):
  shadow = run(rr=make_rr(left=left, right=right, measured=measured)).radar_state.boschAOverlapShadow
  assert not shadow.tracks[0].edgeValid and not shadow.tracks[0].pathValid
  assert math.isnan(shadow.tracks[0].overlapM)
  for match in shadow.matches:
    assert match.existingTrackId == match.shadowTrackId
    assert not match.changed
    for candidate in match.candidates:
      assert candidate.edgeProbability == candidate.centerProbability
      assert candidate.edgeStrict == candidate.centerStrict


def test_no_overlap_and_bracket_spanning_the_corridor():
  outside = run(rr=make_rr(center=-3, left=-2, right=-4)).radar_state.boschAOverlapShadow.tracks[0]
  assert outside.overlapM == outside.overlapFraction == 0
  assert outside.edgeDistance == 2
  spanning = run(rr=make_rr(center=0, left=2, right=-2)).radar_state.boschAOverlapShadow.tracks[0]
  assert spanning.overlapM == 2 and spanning.overlapFraction == 0.5
  assert spanning.edgeDistance == 0 and spanning.edgeScore == 1


def test_duplicate_coast_and_retirement_clear_diagnostics():
  rd = run()
  rd.update(make_sm(), make_rr())
  assert not rd.radar_state.boschAOverlapShadow.radarFresh
  assert not rd.radar_state.boschAOverlapShadow.tracks[0].edgeValid
  rd.update(make_sm(2), make_rr(measured=False))
  assert not rd.radar_state.boschAOverlapShadow.tracks[0].pathValid
  rd.update(make_sm(3), make_empty_radar_data())
  assert len(rd.radar_state.boschAOverlapShadow.tracks) == 0
  rd.update(make_sm(4), make_rr(center=0, left=0.5, right=-0.5))
  assert rd.radar_state.boschAOverlapShadow.tracks[0].overlapFraction == 1


def test_no_bosch_and_model_not_ready():
  assert len(run(bosch=False).radar_state.boschAOverlapShadow.tracks) == 0
  sm = make_sm()
  sm.seen['modelV2'] = False
  shadow = run(sm).radar_state.boschAOverlapShadow
  assert not shadow.tracks[0].pathValid
  assert not any(m.evaluated for m in shadow.matches)


def test_probability_gate_and_distance_gate_still_apply():
  sm = make_sm()
  sm._data['modelV2'].leadsV3[0].prob = 0.1
  match = run(sm).radar_state.boschAOverlapShadow.matches[0]
  assert not match.evaluated and match.shadowTrackId == -1 and not match.changed
  match = run(rr=make_rr(distance=10)).radar_state.boschAOverlapShadow.matches[0]
  assert not match.candidates[0].edgeStrict and match.shadowTrackId == -1


def test_shadow_is_serialized_on_actual_radar_state_topic():
  rd = run()
  messages = {}
  rd.publish(SimpleNamespace(send=lambda name, msg: messages.update({name: msg.to_bytes()})))
  with log.Event.from_bytes(messages['radarState']) as event:
    assert event.radarState.boschAOverlapShadow.matches[0].changed
    assert event.radarState.boschAOverlapShadow.tracks[0].overlapM == pytest.approx(0.2)
    assert not event.radarState.leadOne.radar


def test_shadow_does_not_change_actual_leads_or_track_state(monkeypatch):
  def sequence():
    rd = radard.RadarD(honda_bosch_a_radar=True)
    rd.starpilot_toggles = make_toggles()
    results = []
    for frame in range(1, 11):
      rr = make_rr(center=0.2 * frame, measured=frame != 8)
      rd.update(make_sm(frame), rr)
      state = rd.radar_state.to_dict()
      state.pop('boschAOverlapShadow', None)
      # Compare serialized selected-lead fields and independent internal state, including
      # preferred IDs, sample counts, KF state, and the assist/onpath histories.
      results.append((state, rd.starpilot_radar_state.to_dict(), list(rd.prev_lead_track_ids),
                      [(t.identifier, t.cnt, t.dRel, t.yRel, t.vRel, t.vLeadK, t.aLeadK,
                        list(t.range_hist), list(t.range_hist_long), list(t.onpath_hist)) for t in rd.tracks.values()]))
    return repr(results)  # NaNs are intentional unavailable telemetry in both executions

  observed = sequence()
  monkeypatch.setattr(radard, 'bosch_a_overlap_shadow', lambda *args: {})
  assert sequence() == observed


def test_capnp_model_reader_is_supported():
  md = messaging.new_message('modelV2').modelV2
  md.position.x = [0, 100]
  md.position.y = [0, 0]
  md.velocity.x = [10]
  md.leadsV3 = [vars(make_lead(30)), vars(make_lead(60))]
  sm = make_sm()
  sm._data['modelV2'] = md.as_reader()
  assert run(sm).radar_state.boschAOverlapShadow.matches[0].shadowTrackId == 1


def test_low_probability_empty_model_lead_is_not_scored():
  sm = make_sm()
  sm._data['modelV2'].leadsV3[0] = make_lead(30, probability=0.0)
  sm._data['modelV2'].leadsV3[0].yStd = []
  match = run(sm).radar_state.boschAOverlapShadow.matches[0]
  assert not match.evaluated and len(match.candidates) == 0


def test_shadow_preferred_fallback_and_competing_track_ranking():
  rd = run(rr=make_rr(center=0, left=0.5, right=-0.5))
  # Keep the existing track mature and preferred; expand its current extent toward
  # vision while its centre misses the strict gate. Relaxed preference still applies.
  for frame in range(2, 5):
    rd.update(make_sm(frame), make_rr(center=0, left=0.5, right=-0.5))
  rd.update(make_sm(5), make_rr(center=1.2, left=1.6, right=0.9))
  match = rd.radar_state.boschAOverlapShadow.matches[0]
  assert match.preferredTrackId == match.existingTrackId == match.shadowTrackId == 1
  assert not match.changed

  # A second, wider return can win the interval PDF although its centre is farther
  # from vision. It is observable as a change, never installed in the actual lead.
  rr = car.RadarData.new_message()
  rr.points = [make_rr(center=0.5, left=0.6, right=0.4).points[0].to_dict(),
               dict(trackId=2, dRel=30, yRel=1.6, vRel=0, measured=True, yLeft=2.4, yRight=-0.1)]
  rd.update(make_sm(6), rr)
  match = rd.radar_state.boschAOverlapShadow.matches[0]
  assert match.existingTrackId == match.selectedTrackId == 1
  assert match.shadowTrackId == 2 and match.changed
