from types import SimpleNamespace

from openpilot.starpilot.longitudinal import bosch_a_far_slow_lead as fs

DT = 0.05


def lead(d, v, radar=True):
  return SimpleNamespace(present=True, radar=radar, dRel=d, vLead=v)


def planner(monkeypatch):
  monkeypatch.setattr(fs, 'vehicle_supported', lambda CP: True)
  return fs.BoschAFarSlowLead(SimpleNamespace(radarUnavailable=False), DT)


def step(p, ld, target=-0.2, v_ego=22.5, follow_s=1.25, lead_two=None):
  return p.step(lead_one=ld, lead_two=lead_two, target=target, v_ego=v_ego, follow_s=follow_s, reset_state=False)


def approach(p, d0, closing, v_read, ticks, v_ego=22.5, target=-0.2, jitter=0.0):
  out = []
  for i in range(ticks):
    d = d0 - closing * DT * i + (jitter if i % 2 else -jitter)
    out.append(step(p, lead(d, v_read), target=target, v_ego=v_ego))
  return out


def test_route_84_452_near_stopped_car_read_as_traffic_slows_early(monkeypatch):
  # seg 4 4:49.1-4:50.5: the gap closes ~20 m/s from 120 m while vLead reads 15
  p = planner(monkeypatch)
  out = approach(p, 120.0, 20.0, 15.0, 30)
  assert out[0] == -0.2 and out[fs.BoschAFarSlowLead(None, DT).window.maxlen - 2] == -0.2  # needs a full window
  assert out[-1] == fs.MAX_EARLY_ACCEL  # capped gentle
  assert p.v_est is not None and p.v_est < 4.0


def test_route_84_1233_slowing_lead_gets_a_light_early_brake(monkeypatch):
  # seg 12 12:33: ~9 m/s closing at 110 m, vLead reads 19 (gap implies ~15.6)
  p = planner(monkeypatch)
  out = approach(p, 112.0, 9.1, 19.0, 30, v_ego=24.7)
  assert fs.MAX_EARLY_ACCEL < out[-1] < -0.3


def test_matched_reading_is_left_alone(monkeypatch):
  p = planner(monkeypatch)
  assert approach(p, 120.0, 7.5, 15.0, 40) == [-0.2] * 40  # vLead agrees with the gap


def test_only_lowers_the_target(monkeypatch):
  p = planner(monkeypatch)
  out = approach(p, 120.0, 20.0, 15.0, 30, target=-2.0)
  assert out[-1] == -2.0


def test_identity_jump_restarts_the_window(monkeypatch):
  # 84 12:34.56: 103.5 -> 112.4 m between ticks
  p = planner(monkeypatch)
  approach(p, 125.0, 20.0, 15.0, 15)
  assert step(p, lead(125.0 - 20.0 * DT * 15 + 9.0, 15.0)) == -0.2
  assert len(p.window) == 1


def test_noisy_range_never_triggers(monkeypatch):
  p = planner(monkeypatch)
  assert approach(p, 120.0, 20.0, 15.0, 40, jitter=3.5) == [-0.2] * 40  # 7 m tick-to-tick swings: restarts, never a fit
  p = planner(monkeypatch)
  out = approach(p, 120.0, 0.0, 15.0, 40, jitter=2.9)  # flat but noisy: no closing
  assert out == [-0.2] * 40


def test_inside_60m_and_low_speed_are_not_this_layer(monkeypatch):
  p = planner(monkeypatch)
  assert approach(p, 59.0, 20.0, 15.0, 30) == [-0.2] * 30
  p = planner(monkeypatch)
  assert approach(p, 120.0, 20.0, 15.0, 30, v_ego=14.0) == [-0.2] * 30


def test_bound_holds_then_releases(monkeypatch):
  p = planner(monkeypatch)
  approach(p, 130.0, 20.0, 15.0, 25)
  assert p.active
  d = 130.0 - 20.0 * DT * 25
  held = [step(p, lead(d - 0.1 * i, 4.0)) for i in range(int(fs.HOLD_S / DT) + 2)]  # reading now agrees
  assert held[0] == fs.MAX_EARLY_ACCEL and held[-1] == -0.2


def test_disabled_without_radar():
  p = fs.BoschAFarSlowLead(SimpleNamespace(radarUnavailable=True), DT)
  assert approach(p, 120.0, 20.0, 15.0, 30) == [-0.2] * 30
