from types import SimpleNamespace as NS

from openpilot.starpilot.longitudinal.set_speed_assist import SetSpeedAssist, LIFT_FALL, CLEAR_S

BP, VALS = [0., 10., 25., 40.], [1.6, 1.2, 0.8, 0.6]


def sm(v_ego=25., gas=0.9, curv=0., yaw=0., lane_change='off', lead=False, model_a=0.):
  md = NS(velocity=NS(x=[v_ego] * 33), orientationRate=NS(z=[yaw] * 33),
          meta=NS(disengagePredictions=NS(gasPressProbs=[0.9, gas]), laneChangeState=lane_change),
          action=NS(desiredCurvature=curv, desiredAcceleration=model_a))
  rs = NS(leadOne=NS(present=lead), leadTwo=NS(present=False))
  return {'modelV2': md, 'carState': NS(vEgo=v_ego), 'controlsState': NS(curvature=curv), 'radarState': rs}


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


def test_builds_at_cruise_jerk_after_clear_hold():
  a = assist()
  n = round(CLEAR_S / 0.05)
  outs = [a.update(sm(v_ego=25., gas=1.0), 0.0, 0.5, False) for _ in range(n)]
  assert outs[-2] == 0.0 and abs(outs[-1] - 0.8 * 0.05) < 1e-6


def test_model_slowing_gets_nothing():
  assert run(assist(), sm(model_a=-0.3), -0.3, 0.5) == -0.3


def test_never_below_model_and_drops_fast():
  a = assist()
  run(a, sm(gas=1.0), 0.0, 0.5)
  out = a.update(sm(gas=1.0, model_a=-0.5), -0.5, 0.5, False)  # model starts braking
  assert out >= -0.5 and a.lift <= 0.5 - LIFT_FALL * 0.05 + 1e-9


def test_low_speed_bend_lane_change_and_vision_only_untouched():
  assert run(assist(), sm(v_ego=4.), 0.0, 0.5) == 0.0
  assert run(assist(), sm(curv=0.002), 0.0, 0.5) == 0.0  # 0.002 * 625 = 1.25 m/s^2
  assert run(assist(), sm(lane_change='laneChangeStarting'), 0.0, 0.5) == 0.0
  assert run(assist(radar=False), sm(), 0.0, 0.5) == 0.0


def test_floor_below_model_adds_nothing():
  assert run(assist(), sm(), 0.3, -1.0) == 0.3


def test_any_lead_drops_lift_and_restarts_hold():
  # route 71 rt 118-300: the lift ran into a lead slowing from 25 to 16.6 m/s
  a = assist()
  run(a, sm(gas=1.0), 0.0, 0.5)
  out = a.update(sm(gas=1.0, lead=True), 0.0, 0.5, False)
  assert abs(out - (0.5 - LIFT_FALL * 0.05)) < 1e-6
  assert run(a, sm(gas=1.0, lead=True), 0.0, 0.5) == 0.0
  a.update(sm(gas=1.0), 0.0, 0.5, False)  # lead gone: no lift until the hold passes
  assert a.update(sm(gas=1.0), 0.0, 0.5, False) == 0.0


def test_braking_model_raised_by_encouragement_gets_nothing():
  # the candidate is lifted to -0.05 by Encouragement, but the model itself brakes at -2.0
  assert run(assist(), sm(gas=1.0, model_a=-2.0), -0.05, 0.5) == -0.05
