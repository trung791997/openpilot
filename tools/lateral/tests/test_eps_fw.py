import os

import numpy as np
import pytest

from openpilot.tools.lateral import eps_fw
from openpilot.tools.lateral import lat_pid_sim as sim
from openpilot.tools.lateral.eps_fw import EpsTable

RWD = os.path.join(eps_fw.REPO_ROOT, "eps_tools", "rwd")
OWNER = os.path.join(RWD, "39990-TBA,C020-20260805-ClarityPminus5-P117to265-D737-KFF45-Norm1650-Trk4500-TargetMapD-Telem-" +
                     "SpeedClamp0-Pclamp7373.rwd")
OWNER_TORQUE = [0, 1926, 4938, 8455, 12036, 15926, 20138, 26955, 30000]


def test_owner_image_decodes_to_the_documented_words():
  t = EpsTable.from_rwd(OWNER)
  assert t.axis.tolist() == [0, 99, 258, 447, 643, 862, 1111, 1549, 1774]
  assert t.torque.tolist() == OWNER_TORQUE
  assert (t.words["tracker"], t.words["norm"], t.words["speed_clamp"]) == (4500, 1650, 0)
  assert t.words["p_row0"] == [117, 148, 184, 220, 245, 257, 263, 265, 265]
  assert t.words["d_row0"] == [737] * 9
  # every row carries the same torque row in this image; only the axes differ
  assert EpsTable.from_rwd(OWNER, row=3).torque.tolist() == OWNER_TORQUE
  assert EpsTable.from_rwd(OWNER, row=3).axis.tolist() == [0, 222, 333, 495, 656, 887, 1108, 1552, 1774]
  assert t.controller_diff(EpsTable.from_rwd(OWNER, row=3)) == {}


def test_other_images_and_rows_are_refused():
  with pytest.raises(ValueError):
    EpsTable.from_rwd(os.path.join(RWD, "39990-TLA-A040-stock.rwd"))   # CR-V: different layout
  with pytest.raises(ValueError):
    EpsTable.from_rwd(OWNER, row=7)


def test_drive_is_the_signed_table_torque():
  t = EpsTable.from_rwd(OWNER)
  u = np.linspace(-1, 1, 41)
  y = t.drive(u)
  assert np.allclose(y, -y[::-1]) and np.all(np.diff(y) > 0)
  assert t.drive(1.0) == pytest.approx(30000 / eps_fw.TORQUE_UNIT)
  assert t.drive(99 / 1774) == pytest.approx(1926 / eps_fw.TORQUE_UNIT)


def test_plant_keeps_its_table_through_json_and_refuses_a_swap_without_one():
  t = EpsTable.from_rwd(OWNER)
  p = sim.Plant([-8.5, 0.07, -1.6, -4.0, 800.0, 0.0, 0.0, 0.0, 0.0], delay=5, quant=0.1, eps=t)
  q = sim.Plant.from_json(p.to_json())
  assert q.eps.torque.tolist() == OWNER_TORQUE and q.eps.sha1 == t.sha1 and q.delay == 5
  assert q.drive(0.5) == pytest.approx(t.drive(0.5))
  raw = sim.Plant.from_json(sim.Plant([0.0] * 9).to_json())
  assert raw.eps is None and raw.drive(0.3) == 0.3
  with pytest.raises(ValueError):
    raw.with_eps(t)


def test_the_owner_table_reproduces_the_raw_plant_and_a_weaker_table_tracks_worse():
  from openpilot.tools.lateral.tests.test_lat_pid_sim_torque import PLANT, _modified_cp_bytes, _route
  owner = EpsTable.from_rwd(OWNER)
  weak = EpsTable(owner.axis, owner.torque * 0.3, source="weak")
  # plant_d5's raw-command gain, re-expressed per table-torque unit at the owner's first segment
  k = 1785.5 / (owner.torque[1] / owner.axis[1] * eps_fw.AXIS_MAX / eps_fw.TORQUE_UNIT)
  plant = sim.Plant([*PLANT.c[:4], k, PLANT.c[5] * k / 1785.5, PLANT.c[6] * k / 1785.5, 0.0, 0.0], delay=5, eps=owner)
  d = _route(0.004)
  d["cp_bytes"] = _modified_cp_bytes()
  runs = {name: sim.simulate(d, p)[:2] for name, p in (("raw", PLANT), ("owner", plant), ("weak", plant.with_eps(weak)))}
  err = {name: float(np.sqrt(np.mean((ang[300:700] - des[300:700]) ** 2))) for name, (ang, des) in runs.items()}
  # the owner's table is near-linear over the range this step uses, so it matches the raw-command plant
  assert err["owner"] == pytest.approx(err["raw"], rel=0.05), err
  assert err["weak"] > 1.4 * err["owner"], err
