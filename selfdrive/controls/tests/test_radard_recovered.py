"""D-089: a Bosch-A point recovered by the range-scaled far-range re-anchor (RadarPoint.recovered) is a lead candidate
only after the camera AND its own range slope confirm it, and loses that after a run of disagreement."""
from types import SimpleNamespace

from openpilot.selfdrive.controls import radard

DT = 1.0 / 14.35  # Bosch-A sweep period
V_EGO = 25.0


def new_track(identifier=57):
  return radard.Track(identifier, V_EGO - 3.5, radard.KalmanParams(radard.HONDA_BOSCH_A_RADAR_TS))


def drive(track, n, *, v_rel, closing, d0=70.0, t0=0.0):
  for i in range(n):
    track.update(d0 - closing * i * DT, 0.0, v_rel, V_EGO + v_rel, True, True, t_now=t0 + i * DT)
  return t0 + n * DT


def cam(x, v, y=0.0, prob=0.9):
  return SimpleNamespace(x=[x], y=[y], v=[v], prob=prob)


def test_camera_agreement_needs_range_lateral_and_speed():
  rpt = [65.0, 0.5, -3.5]
  assert radard.recovered_cam_agrees(rpt, [cam(65.0 + radard.RADAR_TO_CAMERA, V_EGO - 3.0, y=-0.5)], V_EGO) is True
  assert radard.recovered_cam_agrees(rpt, [cam(85.0, V_EGO - 3.0)], V_EGO) is False             # range
  assert radard.recovered_cam_agrees(rpt, [cam(66.5, V_EGO - 3.0, y=2.5)], V_EGO) is False      # lateral
  assert radard.recovered_cam_agrees(rpt, [cam(66.5, V_EGO - 8.0)], V_EGO) is False             # speed
  assert radard.recovered_cam_agrees(rpt, [cam(66.5, V_EGO - 3.0, prob=0.3)], V_EGO) is None    # not confident
  # the second camera lead counts too
  assert radard.recovered_cam_agrees(rpt, [cam(85.0, V_EGO), cam(66.5, V_EGO - 3.5)], V_EGO) is True


def test_range_slope_verdict_needs_the_long_window():
  track = new_track()
  drive(track, radard.RANGE_VREL_LONG_SAMPLES - 1, v_rel=-3.5, closing=3.5)
  assert track.recovered_range_slope_agrees(-3.5) is None
  drive(track, 1, v_rel=-3.5, closing=3.5, d0=track.dRel - 3.5 * DT, t0=(radard.RANGE_VREL_LONG_SAMPLES - 1) * DT)
  assert track.recovered_range_slope_agrees(-3.5) is True
  assert track.recovered_range_slope_agrees(-3.5 - radard.RECOVERED_RANGE_SLOPE_TOL_MPS - 0.5) is False


def test_confirms_only_when_camera_and_range_slope_agree():
  track = new_track()
  drive(track, radard.RANGE_VREL_LONG_SAMPLES, v_rel=-3.5, closing=3.5)
  for _ in range(radard.RECOVERED_CAM_CONFIRM_FRAMES - 1):
    track.update_recovered(True, True)
  assert not track.recover_ok
  track.update_recovered(True, None)  # no confident camera breaks the run
  for _ in range(radard.RECOVERED_CAM_CONFIRM_FRAMES - 1):
    track.update_recovered(True, True)
  assert not track.recover_ok
  track.update_recovered(True, True)
  assert track.recover_ok


def test_wrong_u11_is_never_confirmed_even_when_the_camera_agrees():
  # 2f2-type: U11 claims 8 m/s more closing than the range shows; the camera's noisy speed happens to agree
  track = new_track()
  drive(track, radard.RANGE_VREL_LONG_SAMPLES, v_rel=-9.0, closing=1.0)
  for _ in range(20):
    track.update_recovered(True, True)
  assert not track.recover_ok


def test_no_confirmation_before_the_range_window_fills():
  track = new_track()
  drive(track, 5, v_rel=-3.5, closing=3.5)
  for _ in range(10):
    track.update_recovered(True, True)
  assert not track.recover_ok


def test_drops_after_a_run_of_disagreement_and_resets_when_no_longer_recovered():
  track = new_track()
  drive(track, radard.RANGE_VREL_LONG_SAMPLES, v_rel=-3.5, closing=3.5)
  for _ in range(radard.RECOVERED_CAM_CONFIRM_FRAMES):
    track.update_recovered(True, True)
  assert track.recover_ok
  for _ in range(radard.RECOVERED_CAM_DROP_FRAMES - 1):
    track.update_recovered(True, False)
  assert track.recover_ok
  track.update_recovered(True, False)
  assert not track.recover_ok
  track.update_recovered(False, None)
  assert not track.recovered and track.recover_agree == 0 and track.recover_disagree == 0
