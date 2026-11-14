from types import SimpleNamespace

from openpilot.starpilot.longitudinal import bosch_a_ghost as G

DT = 0.05


def lead(tid=22, d=20.0, v=-5.0, y=-0.8, v_lead=8.0, a_lead=-1.0, present=True):
  return SimpleNamespace(present=present, radarTrackId=tid, dRel=d, vRel=v, yRel=y, vLeadK=v_lead, aLeadK=a_lead)


GONE = lead(present=False)
CAM_SEES = (0.9, 21.5)   # camera lead 0 still at the ghost
CAM_CLEAR = (0.1, 80.0)


def born(gl, v_ego=13.0, cam=CAM_SEES, **kw):
  assert gl.update(lead(**kw), -3.0, v_ego, {22: -0.8}, cam) == (None, None)
  return gl.update(GONE, -3.0, v_ego, {22: -0.9}, cam)


def test_a_req_physics():
  assert G.ghost_a_req(30.0, 15.0, 0.0, 0.0) < -3.0      # stopped car 30 m ahead at 15 m/s
  assert G.ghost_a_req(30.0, 10.0, 12.0, 0.0) == 0.0     # pulling away
  assert G.ghost_a_req(3.0, 10.0, 10.0, 0.0) == -10.0    # inside the stop gap


def test_dropout_close_keeps_base():
  # 84 s7 shape: 12.5 m, closing 8 m/s, camera still sees it, raw track gone
  gl = G.GhostLetoff(DT)
  jerk, floor = born(gl, v_ego=14.0, d=12.5, v=-8.0, v_lead=6.0)
  assert jerk == G.FAST_JERK and floor < -3.0 and gl.no_gas


def test_lateral_exit_clears():
  gl = G.GhostLetoff(DT)
  born(gl)
  assert gl.update(GONE, -2.5, 12.0, {22: -1.6}, CAM_SEES) == (G.FAST_JERK, 0.0)
  assert gl.no_gas
  gl.update(GONE, -1.0, 12.0, {22: -3.1}, CAM_SEES)
  assert not gl.no_gas


def test_camera_clear_needs_dwell():
  gl = G.GhostLetoff(DT)
  born(gl, tid=-1, cam=CAM_CLEAR)  # camera-only lead: the camera clear applies
  n = int(G.CAM_CLEAR_S / DT)
  outs = [gl.update(GONE, -2.0, 12.0, {}, CAM_CLEAR) for _ in range(n + 1)]
  assert outs[-1] == (G.FAST_JERK, 0.0)


def test_camera_clear_blocked_while_radar_track_near_lane():
  # 84 s7 shape: camera loses it, raw track still at 1.27 m, then vanishes there
  gl = G.GhostLetoff(DT)
  born(gl, v_ego=14.0, d=13.9, v=-8.0, v_lead=6.0, y=-1.19, cam=CAM_CLEAR)
  for i in range(int(1.0 / DT)):
    jerk, floor = gl.update(GONE, -3.0, 14.0, {22: -1.27} if i < 6 else {}, CAM_CLEAR)
    assert floor < 0.0 and not gl.ghost.letoff_clear
  assert gl.cleared == 0


def test_camera_clear_allowed_for_far_off_centre_track():
  # 80 s3 shape: leadOne track already 4.3 m out and steady; the camera clears it
  gl = G.GhostLetoff(DT)
  far = lead(d=37.0, v=-11.7, v_lead=10.0, y=-4.3)
  assert gl.update(far, -3.0, 22.0, {22: -4.3}, CAM_CLEAR) == (None, None)
  outs = [gl.update(GONE, -3.0, 22.0, {22: -4.3}, CAM_CLEAR) for _ in range(int(G.CAM_CLEAR_S / DT) + 2)]
  assert outs[-1] == (G.FAST_JERK, 0.0)


def test_lead_back_kills_ghost():
  gl = G.GhostLetoff(DT)
  born(gl)
  assert gl.update(lead(d=19.5), -2.0, 13.0, {22: -0.8}, CAM_SEES) == (None, None)
  assert gl.ghost is None and not gl.no_gas


def test_blinker_skips():
  gl = G.GhostLetoff(DT)
  gl.update(lead(), -3.0, 13.0, {22: -0.8}, CAM_SEES)
  assert gl.update(GONE, -3.0, 13.0, {}, CAM_SEES, skip=True) == (None, None)
  assert not gl.no_gas


def test_not_braking_no_ghost():
  gl = G.GhostLetoff(DT)
  gl.update(lead(), 0.2, 13.0, {22: -0.8}, CAM_SEES)
  assert gl.update(GONE, 0.2, 13.0, {}, CAM_SEES) == (None, None)


def test_ghost_expires():
  gl = G.GhostLetoff(DT)
  born(gl, d=60.0, v=-1.0, v_lead=12.0, a_lead=0.0)
  for _ in range(int(G.GHOST_MAX_S / DT) + 2):
    out = gl.update(GONE, -1.0, 13.0, {}, CAM_SEES)
  assert out == (None, None) and not gl.no_gas
