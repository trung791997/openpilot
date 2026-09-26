import numpy as np
import pytest

from openpilot.tools.lateral import lat_accel_space_study as study

WHEELBASE = 2.7


def _drive(n=4000, seed=0):
  rng = np.random.default_rng(seed)
  v = rng.uniform(5.0, 33.0, n)
  theta = rng.uniform(-120.0, 120.0, n) * rng.uniform(0.0, 1.0, n) ** 2
  roll = rng.normal(0.0, 0.03, n)
  return rng, v, theta, roll


def test_fit_bicycle_recovers_sr_and_k():
  rng, v, theta, roll = _drive()
  kappa = study.predict_curvature(theta, v, roll, WHEELBASE, 15.3, 0.0012) + rng.normal(0, 2e-5, len(v))
  sr, k = study.fit_bicycle(theta, v, roll, kappa, WHEELBASE)
  assert sr == pytest.approx(15.3, rel=0.01)
  assert k == pytest.approx(0.0012, abs=5e-5)


def test_vgr_linearisation_roundtrips_the_firmware_map():
  from opendbc.car.honda.steer_ratio import HONDA_VGR_INVERSE_BY_PROFILE, HONDA_VGR_CIVIC_TBA_C020, vgr_physical_to_linear
  inverse = HONDA_VGR_INVERSE_BY_PROFILE[HONDA_VGR_CIVIC_TBA_C020]
  angles = np.array([-200.0, -45.0, -3.0, 0.0, 7.5, 90.0, 300.0])
  expected = [vgr_physical_to_linear(a, inverse) for a in angles]
  assert np.allclose(study.vgr_physical_to_linear_np(angles, inverse), expected)
  # a car whose curvature is linear in the linearised angle is fitted by M1 exactly, not by M0
  _, v, theta, roll = _drive(seed=1)
  kappa = study.predict_curvature(study.vgr_physical_to_linear_np(theta, inverse), v, roll, WHEELBASE, 15.0, 0.0007)
  sr1, _ = study.fit_bicycle(study.vgr_physical_to_linear_np(theta, inverse), v, roll, kappa, WHEELBASE)
  sr0, k0 = study.fit_bicycle(theta, v, roll, kappa, WHEELBASE)
  res0 = kappa - study.predict_curvature(theta, v, roll, WHEELBASE, sr0, k0)
  assert sr1 == pytest.approx(15.0, rel=1e-3)
  assert np.sqrt(np.mean(res0 ** 2)) > 1e-5


def test_sr_curve_angle_is_constant_ratio_at_centre():
  curve = ([0.0, 50.0, 200.0], [15.0, 15.0, 12.0])
  assert study.sr_curve_eff_angle(np.array([10.0]), curve)[0] == pytest.approx(10.0)
  assert study.sr_curve_eff_angle(np.array([-200.0]), curve)[0] == pytest.approx(-250.0)


def test_fit_torque_recovers_factor_friction_offset():
  rng = np.random.default_rng(2)
  la = rng.uniform(-2.5, 2.5, 5000)
  jerk = rng.normal(0.0, 0.6, 5000)
  torque = la / 11.0 + 0.04 * study.friction_shape(jerk) + 0.015 + rng.normal(0, 0.005, 5000)
  laf, fric, off, r2, rms = study.fit_torque(la, jerk, torque)
  assert laf == pytest.approx(11.0, rel=0.01)
  assert fric == pytest.approx(0.04, abs=0.002)
  assert off == pytest.approx(0.015, abs=0.001)
  assert r2 > 0.99 and rms < 0.006
  assert study.torque_residual_with(la, jerk, torque, laf, fric, off) == pytest.approx(rms, rel=1e-6)
