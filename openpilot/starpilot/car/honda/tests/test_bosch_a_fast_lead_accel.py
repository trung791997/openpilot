"""Bosch-A Track fast lead braking: the vLead window slope leads the KF accel on a hard-braking lead, and only then."""
from openpilot.selfdrive.controls.radard import KalmanParams
from openpilot.starpilot.car.honda import radar
from openpilot.starpilot.car.honda.radar import BOSCH_A_LEAD_ACCEL_MIN, FAST_ACCEL_ON, Track

DT = 1.0 / 14.35  # Bosch-A measurement cadence (D-046)


def _run(lead_accel, *, ego_accel=0.0, v_lead0=19.0, v_ego0=19.0, brake_at=20, steps=40, vrel_rail=None, noise=None):
  """Returns (track, [aLeadK per step], [KF accel per step])."""
  track = Track(15, v_lead0, KalmanParams(DT))
  v_lead, v_ego, published, kf = v_lead0, v_ego0, [], []
  for i in range(steps):
    if i >= brake_at:
      v_lead = max(v_lead + lead_accel * DT, 0.0)
      v_ego = max(v_ego + ego_accel * DT, 0.0)
    v_rel = v_lead - v_ego if vrel_rail is None else vrel_rail
    meas_lead = v_rel + v_ego + (noise[i] if noise else 0.0)
    track.update(48.0, 0.0, v_rel, meas_lead, True)
    published.append(track.aLeadK)
    kf.append(float(track.kf.x[radar.ACCEL][0]))
  return track, published, kf


def _first(values, below):
  return next(i for i, v in enumerate(values) if v <= below)


def test_hard_braking_lead_reaches_brake_well_before_kf():
  # route 82 rt 359: radar-locked lead at ~48 m braking about -4 from 19 m/s
  _, published, kf = _run(-4.0)
  fast_i, kf_i = _first(published, -2.0), _first(kf, -2.0)
  assert (kf_i - fast_i) * DT >= 0.3
  assert (fast_i - 20) * DT <= 0.5
  assert min(published) == BOSCH_A_LEAD_ACCEL_MIN  # still ends at the clamp


def test_only_moves_toward_braking():
  _, published, kf = _run(-4.0)
  assert all(p <= max(k, BOSCH_A_LEAD_ACCEL_MIN) + 1e-9 for p, k in zip(published, kf, strict=True))


def test_gentle_slowdown_stays_kf():
  track, published, kf = _run(-1.5)
  assert track.fast_accel is None
  assert all(p == max(k, BOSCH_A_LEAD_ACCEL_MIN) for p, k in zip(published, kf, strict=True))


def test_railed_vrel_with_own_braking_does_not_engage():
  # vRel railed -> vLead follows our own speed; our own -4 must not read as lead braking
  track, published, kf = _run(0.0, ego_accel=-4.0, vrel_rail=0.0)
  assert track.fast_accel is None
  assert all(p == max(k, BOSCH_A_LEAD_ACCEL_MIN) for p, k in zip(published, kf, strict=True))


def test_rail_step_restarts_window():
  track = Track(3, 20.0, KalmanParams(DT))
  for _ in range(10):
    track.update(40.0, 0.0, 0.0, 20.0, True)
  track.update(40.0, 0.0, -8.0, 12.0, True)  # a single 8 m/s jump is a rail/rejoin, not braking
  assert track.fast_accel is None
  assert len(track.fast_hist) == 1


def test_noisy_window_stays_kf():
  noise = [0.6 if i % 2 else -0.6 for i in range(40)]
  track, _, _ = _run(-2.5, noise=noise, steps=30)
  assert track.fast_accel is None


def test_brief_tap_releases_back_to_kf():
  track = Track(4, 20.0, KalmanParams(DT))
  v = 20.0
  for i in range(30):
    if 10 <= i < 14:
      v -= 3.0 * DT
    track.update(40.0, 0.0, v - 20.0, v, True)
  assert track.fast_accel is None
  assert track.aLeadK > FAST_ACCEL_ON


def test_coasted_update_holds_last_value():
  track, published, _ = _run(-4.0, steps=27)
  held = track.fast_accel
  track.update(40.0, 0.0, -5.0, 14.0, False, measurement_update=False)
  assert track.fast_accel == held
