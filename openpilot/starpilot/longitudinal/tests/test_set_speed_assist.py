from types import SimpleNamespace as NS

from openpilot.starpilot.longitudinal.set_speed_assist import SetSpeedAssist, LIFT_FALL

BP, VALS = [0., 10., 25., 40.], [1.6, 1.2, 0.8, 0.6]


def sm(v_ego=25., gas=0.9, curv=0., yaw=0., lane_change=0):
  md = NS(velocity=NS(x=[v_ego] * 33), orientationRate=NS(z=[yaw] * 33),
          meta=NS(disengagePredictions=NS(gasPressProbs=[0.9, gas]), laneChangeState=lane_change),
          action=NS(desiredCurvature=curv))
  return {'modelV2': md, 'carState': NS(vEgo=v_ego), 'controlsState': NS(curvature=curv)}


def assist(radar=True):
  return SetSpeedAssist(NS(radarUnavailable=not radar), BP, VALS)


def run(a, s, a_model, a_floor, n=100):
  out = None
  for _ in range(n):
    out = a.update(s, a_model, a_floor, False)
  return out


def test_lifts_slow_model_toward_floor_but_not_past():
  a = assist()
  out = run(a, sm(gas=1.0), 0.0, 0.5)
  assert 0.49 < out <= 0.5


def test_builds_at_cruise_jerk():
  a = assist()
  out = a.update(sm(v_ego=25., gas=1.0), 0.0, 0.5, False)
  assert abs(out - 0.8 * 0.05) < 1e-6


def test_model_slowing_gets_nothing():
  assert run(assist(), sm(), -0.3, 0.5) == -0.3


def test_never_below_model_and_drops_fast():
  a = assist()
  run(a, sm(gas=1.0), 0.0, 0.5)
  out = a.update(sm(gas=1.0), -0.5, 0.5, False)  # model starts braking
  assert out >= -0.5 and a.lift <= 0.5 - LIFT_FALL * 0.05 + 1e-9


def test_low_speed_bend_lane_change_and_vision_only_untouched():
  assert run(assist(), sm(v_ego=4.), 0.0, 0.5) == 0.0
  assert run(assist(), sm(curv=0.002), 0.0, 0.5) == 0.0  # 0.002 * 625 = 1.25 m/s^2
  assert run(assist(), sm(lane_change=1), 0.0, 0.5) == 0.0
  assert run(assist(radar=False), sm(), 0.0, 0.5) == 0.0


def test_floor_below_model_adds_nothing():
  assert run(assist(), sm(), 0.3, -1.0) == 0.3
