from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

from openpilot.tools.lateral.lat_pid_sim import FIELDS, TUNING_KEYS, extract
from openpilot.tools.sim.sim_lat_record import build_row, check_offroad, long_plan_row, raw_rows, read_toggles, write_episode, write_npz

REPO_ROOT = Path(__file__).resolve().parents[3]
SET_OVERRIDES_SCRIPT = REPO_ROOT / "tools" / "sim" / "sim_set_overrides.py"


def test_record_synthetic(tmp_path: Path) -> None:
  rows = []
  for i in range(50):
    sm = SimpleNamespace(
      controlsState=SimpleNamespace(
        lateralControlState=SimpleNamespace(
          which=lambda: "pidState",
          pidState=SimpleNamespace(
            active=True,
            output=0.1 * i,
            steeringAngleDesiredDeg=1.0 + 0.1 * i,
            p=0.01 * i,
            i=0.02 * i,
            f=0.03 * i,
          ),
        ),
        desiredCurvature=0.001 * i,
      ),
      carState=SimpleNamespace(
        vEgo=25.0 + 0.1 * i,
        aEgo=0.05,
        steeringAngleDeg=1.0 + 0.05 * i,
        steeringRateDeg=0.2,
        steeringPressed=(i % 10 == 0),
        steeringTorque=0.5,
        leftBlinker=(i > 40),
        rightBlinker=False,
      ),
      carControl=SimpleNamespace(
        latActive=True,
        actuators=SimpleNamespace(torque=0.15),
      ),
      carOutput=SimpleNamespace(
        actuatorsOutput=SimpleNamespace(torque=0.14),
      ),
      liveParameters=SimpleNamespace(
        steerRatio=15.5,
        stiffnessFactor=1.0,
        angleOffsetDeg=0.1,
        roll=0.01,
      ),
      t=float(i) * 0.01,
    )
    row = build_row(sm)
    assert len(row) == len(FIELDS)
    rows.append(row)

  assert len(rows) == 50
  params = {k: f"val_{k}_{idx}" for idx, k in enumerate(TUNING_KEYS)}
  outdir = str(tmp_path)

  write_npz(outdir, rows, cp_bytes=b"xyz", params=params)

  data = extract(outdir)
  for k in FIELDS:
    assert k in data, f"Missing field {k}"
    assert len(data[k]) == 50, f"Field {k} has length {len(data[k])}, expected 50"

  assert data["cp_bytes"] == b"xyz"
  assert data["params"] == params


def test_t_is_seconds_near_t0() -> None:
  # logMonoTime is ns: a row 0.5 ms after t0 must read 0.0005 s, not 500000 (the t-row-1 bug, PR 10 Amendment 4)
  t0 = 1_000_000_000_000
  for dt_ns in (0, 500_000, 10_000_000):
    row = build_row({"logMonoTime": {"controlsState": t0 + dt_ns}}, t0=t0)
    assert abs(row[FIELDS.index("t")] - dt_ns * 1e-9) < 1e-12


def test_read_toggles() -> None:
  class FakeParams:
    def get(self, k):
      if k == "NrdrLatEpsFfAngleGate":
        raise KeyError(k)  # a Params build that does not know the key
      return {"NrdrLatEpsFirmwareFF": b"1"}.get(k)
  assert read_toggles(FakeParams()) == {"NrdrLatEpsFirmwareFF": "1", "NrdrLatPidFirmwareFF": "unset", "NrdrLatEpsFfAngleGate": "unknown"}
  assert read_toggles(SimpleNamespace(get=lambda k: True), ("A",)) == {"A": "1"}


def test_long_plan_row() -> None:
  mv = SimpleNamespace(action=SimpleNamespace(desiredAcceleration=0.4))
  lp = SimpleNamespace(aTarget=0.3, speeds=[24.5, 24.6], hasLead=False)
  assert long_plan_row(mv, lp, True, True) == [0.4, 0.3, 24.5, 0.0]
  r = long_plan_row(mv, lp, False, False)
  assert all(x != x for x in r)
  empty = long_plan_row(mv, SimpleNamespace(aTarget=0.0, speeds=[], hasLead=True), True, True)
  assert empty[2] != empty[2] and empty[3] == 1.0


def test_episode_offroad(tmp_path: Path) -> None:
  bridge_log = tmp_path / "bridge.log"
  bridge_log.write_text("""frame 10: normal
frame 20: vehicle out_of_road detected
frame 30: normal
""")

  rows = [[1.0 if idx == FIELDS.index("active") else 0.0 for idx in range(len(FIELDS))] for _ in range(50)]
  outdir = str(tmp_path)

  ep = write_episode(outdir, seconds=0.5, rows=rows, bridge_log=str(bridge_log))
  ep_file = tmp_path / "episode.json"
  assert ep_file.exists()

  with open(ep_file) as f:
    loaded = json.load(f)

  assert ep == loaded
  assert ep["seconds"] == 0.5
  assert ep["rows"] == 50
  assert ep["engaged_s"] == 0.5
  assert ep["offroad"] is True

  # Clean bridge log test
  clean_log = tmp_path / "clean_bridge.log"
  clean_log.write_text("frame 10: normal\nframe 20: normal\n")
  clean_dir = str(tmp_path / "clean")
  ep_clean = write_episode(clean_dir, seconds=0.5, rows=rows, bridge_log=str(clean_log))
  assert ep_clean["offroad"] is False

  # Additional offroad keyword checks
  lane_log = tmp_path / "lane.log"
  lane_log.write_text("frame 5: vehicle out_of_lane\n")
  assert check_offroad(str(lane_log)) is True

  crash_log = tmp_path / "crash.log"
  crash_log.write_text("frame 8: simulation crash reported\n")
  assert check_offroad(str(crash_log)) is True


def test_set_overrides() -> None:
  # set_overrides() sets os.environ["OPENPILOT_PREFIX"] itself (matching how the standalone
  # script is invoked from a shell) and Params() resolves the prefix at instantiation time. The
  # repo's global pytest fixture (root conftest.py's OpenpilotPrefix) already put this test
  # process under its own prefix and asserts on teardown that OPENPILOT_PREFIX is unchanged, so
  # calling set_overrides() in-process here would clobber that invariant for every other test in
  # this worker. Run the script (and the read-back) in subprocesses instead: each gets its own
  # env with a private OPENPILOT_PREFIX, and the parent's os.environ is never touched.
  prefix = f"test_sim_prefix_{os.getpid()}"
  child_env = dict(os.environ)
  child_env["OPENPILOT_PREFIX"] = prefix
  child_env["PYTHONPATH"] = os.pathsep.join([str(REPO_ROOT), child_env.get("PYTHONPATH", "")])

  run = subprocess.run(
    [sys.executable, str(SET_OVERRIDES_SCRIPT), "--prefix", prefix,
     "LatIScaleStandard=75", "ExperimentalMode=true", "ForceOffroad=false"],
    cwd=str(REPO_ROOT), env=child_env, capture_output=True, text=True, check=False,
  )
  assert run.returncode == 0, f"stdout={run.stdout!r} stderr={run.stderr!r}"
  assert run.stdout.split() == ["LatIScaleStandard", "ExperimentalMode", "ForceOffroad"]
  assert "OPENPILOT_PREFIX" not in os.environ or os.environ["OPENPILOT_PREFIX"] != prefix

  readback_code = "\n".join([
    "from openpilot.common.params import Params",
    "p = Params()",
    "print(p.get('LatIScaleStandard', encoding='utf-8'))",
    "print(p.get_bool('ExperimentalMode'))",
    "print(p.get_bool('ForceOffroad'))",
  ])
  readback = subprocess.run(
    [sys.executable, "-c", readback_code],
    cwd=str(REPO_ROOT), env=child_env, capture_output=True, text=True, check=False,
  )
  assert readback.returncode == 0, f"stdout={readback.stdout!r} stderr={readback.stderr!r}"
  lines = readback.stdout.strip().splitlines()
  assert lines == ["75", "True", "False"]


def test_raw_rows() -> None:
  msgs = [SimpleNamespace(logMonoTime=int(1e9) + 10_000_000 * k, carControl=SimpleNamespace(latActive=k != 1)) for k in range(3)]
  assert raw_rows(msgs, "carControl", "latActive") == [[1.0, 1.0], [1.01, 0.0], [1.02, 1.0]]
  assert raw_rows([], "carState", "steerFaultTemporary") == []
