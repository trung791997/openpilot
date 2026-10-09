"""RAIL_RANGE_VEL_CHECK (radard.py, D-090): a Bosch-A track whose vRel sits on the U11 rail is matched to the camera
lead by its own clean range slope when that slope lies beyond the rail, and gets no loose preferred-track hold.
Route 000002f8 1:30.4: track 52 at -12.00 closing ~16.5 m/s against a 12 m/s camera car."""
from types import SimpleNamespace

import pytest

from openpilot.selfdrive.controls import radard

DT = radard.HONDA_BOSCH_A_RADAR_TS
V_EGO = 13.38


@pytest.fixture(autouse=True)
def check_on():
  radard.set_bosch_a_newborn_leads(True)
  yield
  radard.RAIL_RANGE_VEL_CHECK = True


def make_track(ranges, v_rel, measured=True, t0=100.0, y=-0.8):
  track = radard.Track(52, v_rel + V_EGO, radard.KalmanParams(DT))
  for i, d in enumerate(ranges):
    track.update(d, y, v_rel, v_rel + V_EGO, measured, measurement_update=measured, t_now=t0 + i * DT)
  return track


def linear(d0, d1, n):
  return [d0 + (d1 - d0) * i / (n - 1) for i in range(n)]


def cam(x, v, y=0.75):
  return SimpleNamespace(x=[x + radard.RADAR_TO_CAMERA], v=[v], y=[y], yStd=[0.5], xStd=[2.0], vStd=[1.0], prob=0.98)


STRICT = dict(dist_scale=0.25, dist_floor=5.0, vel_limit=10.0, y_std_scale=1.0, y_floor=1.0, honda_bosch_a=True)
# 2f8: ~16.5 m/s closing over ~1 s, ending at the camera range; camera car at 11.24 m/s (the 9.86 m/s strict pass)
RAILED_RANGES = linear(73.0 + 16.5 * 19 * DT, 73.0, 20)


def test_railed_track_uses_its_range_slope():
  track = make_track(RAILED_RANGES, radard.BOSCH_A_U11_LOW_RAIL_MPS)
  slope = radard.rail_range_vrel(track)
  assert slope is not None and slope < -15.0
  assert not radard.track_matches_vision(track, cam(73.5, 11.24), V_EGO, **STRICT)
  radard.RAIL_RANGE_VEL_CHECK = False
  assert radard.track_matches_vision(track, cam(73.5, 11.24), V_EGO, **STRICT)  # the clamp passed by 0.14 m/s


def test_unrailed_or_inside_rail_tracks_are_unchanged():
  assert radard.rail_range_vrel(make_track(RAILED_RANGES, -10.0)) is None              # not on the rail
  assert radard.rail_range_vrel(make_track(linear(72.0 + 10.0 * 19 * DT, 72.0, 20), -12.0)) is None    # slope inside the rail
  noisy = [d + (3.0 if i % 2 else -3.0) for i, d in enumerate(RAILED_RANGES)]
  assert radard.rail_range_vrel(make_track(noisy, -12.0)) is None                     # no clean fit


def test_opening_rail_is_still_inert():
  # high rail, opening faster than the clamp: the slope is used but vEgo + slope > 3 keeps vel_sane, as before
  track = make_track(linear(40.0, 40.0 + 15.0 * 19 * DT, 20), radard.BOSCH_A_U11_HIGH_RAIL_MPS)
  assert radard.rail_range_vrel(track) is not None
  assert radard.track_matches_vision(track, cam(40.0 + 15.0 * 19 * DT, 13.0, y=0.8), V_EGO, **STRICT)


def test_railed_track_gets_no_loose_preferred_hold():
  track = make_track(RAILED_RANGES, radard.BOSCH_A_U11_LOW_RAIL_MPS)
  # lead 73.5 m at 11.74 m/s: strict fails on velocity either way; the old loose hold (vel 13) kept the clamp
  lead = cam(73.5, 11.74)
  toggles = SimpleNamespace()
  assert radard.match_vision_to_track(V_EGO, lead, None, {52: track}, toggles, preferred_track_id=52, honda_bosch_a=True) is None
  radard.RAIL_RANGE_VEL_CHECK = False
  assert radard.match_vision_to_track(V_EGO, lead, None, {52: track}, toggles, preferred_track_id=52, honda_bosch_a=True) is track
