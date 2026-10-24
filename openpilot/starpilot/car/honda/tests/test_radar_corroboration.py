"""D-048 corroboration bound (static). Scenarios are shaped on 231 t=9.1-11.35 and route 20 T285-287."""
from types import SimpleNamespace

import pytest

from openpilot.starpilot.car.honda.radar_corroboration import RADAR_TO_CAMERA, W_MAX, RadarCorroboration

V_EGO = 25.0


class FakeTrack(SimpleNamespace):
  def state(self, prob=0.95):
    return {'dRel': self.dRel, 'yRel': 0.0, 'vRel': self.vLead - V_EGO, 'vLead': self.vLead, 'vLeadK': self.vLead,
            'aLeadK': self.aLeadK, 'aLeadTau': 0.6, 'present': True, 'modelProb': prob, 'radar': True,
            'radarTrackId': self.identifier}


def vision(d, v, a=0.0):
  return SimpleNamespace(x=[d + RADAR_TO_CAMERA], v=[v], a=[a])


def run(corr, track, lead_msg, frames, prob=0.95, index=0):
  out = None
  for _ in range(frames):
    corr.observe(track, lead_msg, V_EGO, V_EGO, prob)
    out = corr.bound(index, track.state(prob), lead_msg, V_EGO, V_EGO, prob)
  return out


def test_agreeing_lead_is_never_touched():
  # Negative control: ordinary following with vision noise inside AGREE_DV changes nothing, at any range.
  corr = RadarCorroboration()
  for d in (20.0, 45.0, 70.0, 90.0):
    track = FakeTrack(identifier=int(d), dRel=d, vLead=22.0, aLeadK=-0.4, measured=True)
    for k in range(200):
      jitter = 1.5 if k % 2 else -1.5
      lead_msg = vision(d + 8.0 * (1 if k % 3 else -1), 22.0 + jitter, -0.2)
      corr.observe(track, lead_msg, V_EGO, V_EGO, 0.95)
      assert corr.bound(0, track.state(), lead_msg, V_EGO, V_EGO, 0.95) == track.state()
      assert corr.weights[0] == 0.0


def test_route_20_fresh_far_track_reading_slow_is_bounded_not_dropped():
  corr = RadarCorroboration()
  track = FakeTrack(identifier=23, dRel=58.0, vLead=16.0, aLeadK=-3.0, measured=True)
  out = run(corr, track, vision(66.0, 22.0, -0.3), 20)
  assert out['present'] and out['radar'] and out['radarTrackId'] == 23
  assert corr.weights[0] == pytest.approx(W_MAX, abs=0.05)
  assert 16.0 < out['vLead'] < 22.0
  assert out['vRel'] == pytest.approx(out['vLead'] - V_EGO)
  assert -3.0 < out['aLeadK'] < -0.3
  assert 58.0 < out['dRel'] < 66.0


def test_urgent_geometry_bypasses_the_bound():
  corr = RadarCorroboration()
  close = FakeTrack(identifier=1, dRel=30.0, vLead=16.0, aLeadK=-3.0, measured=True)
  assert run(corr, close, vision(38.0, 22.0), 40) == close.state()
  short_ttc = FakeTrack(identifier=2, dRel=60.0, vLead=10.0, aLeadK=-3.0, measured=True)  # 15 m/s closing, TTC 4 s
  assert run(corr, short_ttc, vision(66.0, 20.0), 40) == short_ttc.state()


def test_never_moves_toward_the_more_severe_side():
  corr = RadarCorroboration()
  track = FakeTrack(identifier=3, dRel=66.0, vLead=24.0, aLeadK=0.0, measured=True)
  assert run(corr, track, vision(58.0, 18.0, -2.0), 40) == track.state()


def test_trusted_lead_that_starts_braking_keeps_its_authority():
  # Followed in agreement for 5 s, then the lead brakes; radar sees it first and vision catches up over 1.5 s.
  corr = RadarCorroboration()
  track = FakeTrack(identifier=4, dRel=60.0, vLead=24.0, aLeadK=0.0, measured=True)
  run(corr, track, vision(62.0, 24.0), 100)
  for k in range(40):
    track.vLead, track.aLeadK = max(24.0 - 0.3 * k, 14.0), -3.0
    lagged = max(24.0 - 0.3 * max(k - 20, 0) * 2, track.vLead)
    out = run(corr, track, vision(62.0, lagged), 1)
    assert corr.weights[0] < 0.05 * W_MAX
    assert out['vLead'] - track.vLead < 0.1


def test_trusted_track_keeps_full_authority_through_the_grace_then_loses_trust():
  corr = RadarCorroboration()
  track = FakeTrack(identifier=8, dRel=70.0, vLead=22.0, aLeadK=0.0, measured=True)
  run(corr, track, vision(74.0, 22.0), 100)
  track.vLead, track.aLeadK = 16.0, -2.5  # 231-like migration: vision never agrees
  run(corr, track, vision(76.0, 22.0), 20)
  assert corr.weights[0] == 0.0
  run(corr, track, vision(76.0, 22.0), 50)
  assert corr.weights[0] > 0.5 * W_MAX


def test_weak_vision_is_not_a_witness():
  corr = RadarCorroboration()
  track = FakeTrack(identifier=5, dRel=58.0, vLead=16.0, aLeadK=-3.0, measured=True)
  assert run(corr, track, vision(66.0, 22.0), 40, prob=0.6) == track.state(0.6)


def test_onset_is_continuous_as_the_track_walks_away_from_vision():
  corr = RadarCorroboration()
  track = FakeTrack(identifier=6, dRel=72.0, vLead=22.0, aLeadK=0.0, measured=True)
  lead_msg = vision(76.0, 22.0)
  prev = None
  for k in range(60):  # 231-like: U11 ramps to -6 m/s over 3 s
    track.vLead = 22.0 - 6.0 * min(k / 60.0, 1.0)
    out = run(corr, track, lead_msg, 1)
    if prev is not None:
      assert abs(out['vLead'] - prev['vLead']) < 0.5
      assert abs(out['dRel'] - prev['dRel']) < 0.5
    prev = out
  assert corr.weights[0] > 0.3 * W_MAX


def test_vision_only_and_absent_leads_pass_through():
  corr = RadarCorroboration()
  vision_lead = {'present': True, 'radar': False, 'radarTrackId': -1, 'dRel': 50.0, 'vRel': -5.0}
  assert corr.bound(0, vision_lead, vision(50.0, 20.0), V_EGO, V_EGO, 0.95) is vision_lead
  assert corr.bound(1, {'present': False}, vision(50.0, 20.0), V_EGO, V_EGO, 0.95) == {'present': False}


def test_prune_drops_state_for_vanished_tracks():
  corr = RadarCorroboration()
  run(corr, FakeTrack(identifier=7, dRel=60.0, vLead=20.0, aLeadK=0.0, measured=True), vision(60.0, 20.0), 1)
  corr.prune({})
  assert corr.state == {}
