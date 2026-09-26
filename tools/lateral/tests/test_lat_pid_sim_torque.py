import math

import numpy as np
import pytest

from opendbc.car.car_helpers import interfaces
from opendbc.car.honda.steer_ratio import get_honda_vgr_inverse, vgr_linear_to_physical
from opendbc.car.honda.values import CAR as HONDA, HondaFlags
from openpilot.tools.lateral import lat_pid_sim as sim

# plant_d5 (fitted on 260-263) without its offset and friction terms: stable, and a positive command turns the wheel
# to a positive angle.
PLANT = sim.Plant([-8.544, 0.0716, -1.635, -4.022, 1785.5, -19.756, -60.5, 0.0, 0.0], delay=5)


def _cp_bytes(vgr=True):
  CP = interfaces[HONDA.HONDA_CIVIC_BOSCH].get_non_essential_params(HONDA.HONDA_CIVIC_BOSCH)
  CP.flags |= int(HondaFlags.EPS_MODIFIED) | (int(HondaFlags.VGR_CIVIC_TBA_C020) if vgr else 0)
  CP.dashcamOnly = True
  return CP.to_bytes()


def _route(curv, v=20.0, n=1500, vgr=True):
  """Engaged, hands off, constant speed; the desired curvature steps to curv at 3 s."""
  d = {k: np.zeros(n) for k in sim.FIELDS}
  d["t"] = np.arange(n) * sim.DT
  d["v"][:] = v
  d["active"][:] = 1.0
  d["sr"][:] = 16.0
  d["stiff"][:] = 1.0
  d["des_curv"][300:] = curv
  d["cp_bytes"] = _cp_bytes(vgr)
  d["params"] = {"HondaLateralPidKpScale": "1.0", "HondaLateralPidKiScale": "1.0"}
  d["route"] = "synthetic"
  return d


@pytest.mark.parametrize("kind", ["torque_upstream", "torque_starpilot"])
@pytest.mark.parametrize("vgr", [False, True])
def test_torque_kinds_close_the_loop_onto_their_own_target(kind, vgr):
  d = _route(0.004)   # right turn in this fork's convention: a negative wheel angle
  ang, des, out, deliv, raw = sim.simulate(d, PLANT, with_raw=True, kind=kind, torque={"vgr": vgr})
  assert raw[-1] < -5.0
  assert np.all(np.isfinite(ang)) and np.max(np.abs(deliv)) <= 1.0 + 1e-9
  assert ang[-1] == pytest.approx(raw[-1], abs=0.1 * abs(raw[-1]))
  assert out[-1] < 0.0


def test_vgr_target_is_the_map_applied_to_the_linear_angle():
  d = _route(0.02, v=8.0)
  _, _, _, _, raw_off = sim.simulate(d, PLANT, with_raw=True, kind="torque_upstream", torque={"vgr": False})
  _, _, _, _, raw_on = sim.simulate(d, PLANT, with_raw=True, kind="torque_upstream", torque={"vgr": True})
  ctl = sim.Controller(d["cp_bytes"], d["params"], kind="torque_upstream", torque={"vgr": True})
  ctl.VM.update_params(1.0, sim.VGR_SR)
  lin = math.degrees(ctl.VM.get_steer_from_curvature(-0.02, 8.0, 0.0))
  ctl.close()
  assert raw_on[-1] == pytest.approx(vgr_linear_to_physical(lin, get_honda_vgr_inverse(ctl.CP.flags)), abs=1e-6)
  assert abs(raw_on[-1]) != pytest.approx(abs(raw_off[-1]), abs=0.5)   # the map and sR change the target at 8 m/s


def test_vgr_without_the_firmware_flag_is_refused():
  d = _route(0.004, vgr=False)
  with pytest.raises(ValueError):
    sim.Controller(d["cp_bytes"], d["params"], kind="torque_upstream", torque={"vgr": True})


def test_starpilot_kind_restores_the_testing_ground_hooks():
  from openpilot.selfdrive.controls.lib import latcontrol_torque
  before = (latcontrol_torque.civic_bosch_modified_lateral_testing_ground_active,
            latcontrol_torque.civic_bosch_modified_a_lateral_testing_ground_active)
  sim.simulate(_route(0.002, n=400), PLANT, kind="torque_starpilot")
  assert (latcontrol_torque.civic_bosch_modified_lateral_testing_ground_active,
          latcontrol_torque.civic_bosch_modified_a_lateral_testing_ground_active) == before


def test_parse_variant_splits_torque_keys_from_param_overrides():
  assert sim.parse_variant("x=torque_starpilot,vgr=1,friction_low=0.08,NrdrLatRateFF=0.5") == \
    ("x", "torque_starpilot", {"vgr": True, "friction_low": 0.08}, {"NrdrLatRateFF": "0.5"})
  with pytest.raises(ValueError):
    sim.parse_variant("x=nnff")
