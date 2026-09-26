import json

import numpy as np
import pytest

from openpilot.selfdrive.controls.lib import lat_tune_analyzer as lat
from openpilot.tools.lateral import lat_autotune as at
from openpilot.tools.lateral import lat_pid_sim as sim
from openpilot.tools.lateral import lat_tune_sim as lts
from openpilot.tools.lateral.tests.test_lat_pid_sim_torque import PLANT, _cp_bytes

GAINS = [{"p": 100, "i": 50, "f": 50}, {"p": 105, "i": 75, "f": 100}, {"p": 105, "i": 0, "f": 100}]


def test_gate_and_cost_constants_match_lat_autotune():
  assert (lts.TRUST_REL, lts.TRUST_CURVE, lts.MIN_BAND_MIN, lts.HOLDOUT_BLOCK_S) == \
         (at.TRUST_REL, at.TRUST_CURVE, at.MIN_BAND_MIN, at.HOLDOUT_BLOCK_S)


def test_shipped_plant_loads_banded_with_the_owner_table():
  plant, meta = lts.load_plant()
  assert isinstance(plant, sim.BandedPlant) and plant.delay == 5 and plant.quant == pytest.approx(0.1)
  assert meta["car"] == "HONDA_CIVIC_BOSCH" and "39990-TBA,C020" in meta["eps_fw"]
  assert "Trk4500" in meta["image"] and plant.eps is not None and len(plant.low.c) == 10


def test_banded_plant_blends_and_round_trips():
  low = sim.Plant(list(PLANT.c) + [-300.0], delay=5, quant=0.1)
  high = sim.Plant(PLANT.c, delay=5, quant=0.1)
  b = sim.BandedPlant(low, high, (22.0, 28.0))
  args = (4.0, 1.0, 0.01)
  assert b.accel(*args, 20 * sim.MPH) == low.accel(*args, 20 * sim.MPH)
  assert b.accel(*args, 30 * sim.MPH) == high.accel(*args, 30 * sim.MPH)
  mid = 25 * sim.MPH
  assert b.accel(*args, mid) == pytest.approx(0.5 * low.accel(*args, mid) + 0.5 * high.accel(*args, mid))
  back = sim.Plant.from_json(json.loads(json.dumps(b.to_json())))
  assert isinstance(back, sim.BandedPlant) and back.accel(*args, mid) == pytest.approx(b.accel(*args, mid))
  with pytest.raises(ValueError):
    sim.BandedPlant(low, sim.Plant(PLANT.c, delay=4, quant=0.1))


def test_centring_term_is_a_constant_pull_past_a_few_degrees():
  p = sim.Plant([0.0] * 9 + [-100.0])
  assert p.accel(10.0, 0.0, 0.0, 10.0) == pytest.approx(p.accel(20.0, 0.0, 0.0, 10.0), rel=1e-3)
  assert p.accel(-10.0, 0.0, 0.0, 10.0) == pytest.approx(100.0, rel=1e-3)
  assert p.accel(0.0, 0.0, 0.0, 10.0) == 0.0
  assert "c9 tanh" in p.to_json()["model"] and "c9" not in sim.Plant([0.0] * 9).to_json()["model"]


def test_candidate_grid_is_p_10pct_by_i_25_and_holds_i_under_a_schedule_i_term():
  c = lts.candidate_gains(GAINS, [])
  assert len(c) == 9 and c[(0, 0)] == [{"p": g["p"], "i": g["i"]} for g in GAINS]
  assert [g["p"] for g in c[(1, 0)]] == [110, 115, 115] and [g["p"] for g in c[(-1, 0)]] == [90, 95, 95]
  assert [g["i"] for g in c[(0, -1)]] == [25, 50, 0]          # clamped at 0
  assert set(lts.candidate_gains(GAINS, ["i"])) == {(-1, 0), (0, 0), (1, 0)}


def test_overrides_strip_the_schedule_p_term():
  o = lts.overrides_for([{"p": 110, "i": 75}] * 3, '{"v_mph":[20,50],"p":[100,100],"i":[50,50]}')
  assert o["LatPScaleStandard"] == "110" and o["LatIScaleHighway"] == "75"
  assert "p" not in json.loads(o["LatGainSchedule"])
  assert "LatGainSchedule" not in lts.overrides_for([{"p": 110, "i": 75}] * 3, "")


def test_plant_match_needs_the_car_and_eps_firmware():
  ok, why = lts.plant_matches(_cp_bytes(), {"car": "HONDA_CIVIC_BOSCH", "eps_fw": ["39990-TBA,C020"]})
  assert not ok and "EPS firmware" in why          # the synthetic CarParams carry no firmware
  ok, why = lts.plant_matches(_cp_bytes(), {"car": "HONDA_ACCORD"})
  assert not ok and "HONDA_CIVIC_BOSCH" in why
  assert lts.plant_matches(_cp_bytes(), {"car": "HONDA_CIVIC_BOSCH"})[0]


def _m(err, straight, curve=0.95, sign=0.3, minutes=4.0):
  return {"min": minutes, "err_rms": err, "straight_rms": straight, "curve_ratio": curve, "sign_hyst": sign, "zero_cross": 1.0}


def _res(band, **kw):
  m = _m(**kw)
  return {mask: {name: (m if name == band else None) for name, _, _ in sim.BANDS} for mask in ("fit", "holdout", "all")}


def test_trust_gate_matches_autotune_thresholds():
  band = sim.BANDS[1][0]
  assert lts.trust([_res(band, err=1.0, straight=0.5)], [_res(band, err=0.9, straight=0.5)], band)[0]
  ok, why = lts.trust([_res(band, err=1.3, straight=0.5)], [_res(band, err=1.0, straight=0.5)], band)
  assert not ok and "err rms" in why
  ok, why = lts.trust([_res(band, err=1.0, straight=0.5, curve=0.95)], [_res(band, err=1.0, straight=0.5, curve=0.88)], band)
  assert not ok and "curve ratio" in why
  ok, why = lts.trust([_res(band, err=1.0, straight=0.5, minutes=2.0)], [_res(band, err=1.0, straight=0.5, minutes=2.0)], band)
  assert not ok and "min of data" in why


def test_pick_takes_the_best_pair_unless_vetoed():
  band = sim.BANDS[1][0]
  cands = lts.candidate_gains(GAINS, [])
  results = {k: [_res(band, err=1.0, straight=0.5)] for k in cands}
  results[(1, 1)] = [_res(band, err=0.8, straight=0.45)]
  results[(-1, 0)] = [_res(band, err=0.9, straight=0.5)]
  assert lts._pick(1, band, results, cands, (0, 0), {"pressRate": 0.5, "signRate": 0.3})[0] == (1, 1)
  # the logs veto a step up (override onsets), so the best step down is taken instead
  assert lts._pick(1, band, results, cands, (0, 0), {"pressRate": 2.0, "signRate": 0.3})[0] == (-1, 0)
  # a pair that sets the car weaving past the analyzer limit is never proposed
  results[(1, 1)] = [_res(band, err=0.8, straight=0.45, sign=lat.SIGN_RATE_MAX + 0.1)]
  assert lts._pick(1, band, results, cands, (0, 0), {"pressRate": 0.5, "signRate": 0.3})[0] == (-1, 0)
  # nothing 2 % better: hold
  flat = {k: [_res(band, err=0.99, straight=0.5)] for k in cands}
  key, note = lts._pick(1, band, flat, cands, (0, 0), {})
  assert key == (0, 0) and "no pair" in note


def _logged_route(minutes=3.3, v=18.0):
  """A 40 mph drive whose 'log' is the sim itself at the driven gains, so the Standard band is trusted by construction."""
  n = int(minutes * 6000)
  t = np.arange(n) * sim.DT
  d = {k: np.zeros(n) for k in sim.FIELDS}
  d.update(t=t, v=np.full(n, v), active=np.ones(n), sr=np.full(n, 16.0), stiff=np.ones(n),
           des_curv=0.004 * np.sin(2 * np.pi * t / 20.0) * (np.sin(2 * np.pi * t / 90.0) > -0.3))
  d["cp_bytes"] = _cp_bytes(False)
  d["params"] = {"HondaLateralPidKpScale": "1.0", "HondaLateralPidKiScale": "1.0", "LatPScaleStandard": "105",
                 "LatIScaleStandard": "75"}
  d["route"] = "synthetic"
  ang, des, _, _ = sim.simulate(d, PLANT)
  d["angle"], d["des_angle"] = ang, des
  return d


def test_refine_trial_trusts_a_band_the_sim_reproduces_and_keeps_rules_elsewhere(monkeypatch):
  monkeypatch.setattr(lts, "plant_matches", lambda cp, meta: (True, "test"))
  d = _logged_route()
  bands = [{"name": n, "current": dict(GAINS[i]), "proposed": dict(GAINS[i]), "factor": 1.0, "decision": "hold",
            "reason": "rules", "pressRate": 0.2, "signRate": 0.2} for i, n in enumerate(lat.BAND_NAMES)]
  bands[0]["factor"], bands[0]["proposed"]["p"] = 1.05, 105       # a rule step in a band with no data
  trial = {"bands": bands, "baseline": {"gains": GAINS, "raw": dict(d["params"]), "scheduleTerms": []}}
  lts.refine_trial(trial, [d], PLANT, {"path": "test"})
  assert trial["sim"]["status"] == "used" and trial["sim"]["routes"] == ["synthetic"]
  low, std, hwy = trial["bands"]
  assert not low["sim"]["trusted"] and low["source"] == "rules" and low["factor"] == 1.05   # the rule step stands
  assert not hwy["sim"]["trusted"] and hwy["source"] == "rules"
  assert std["sim"]["trusted"], std["sim"]["trust"]
  assert std["source"] == "sim" and len(std["sim"]["grid"]) == 9
  assert std["sim"]["simDriven"]["err_rms"] == pytest.approx(std["sim"]["log"]["err_rms"], rel=1e-6)
  assert std["proposed"]["f"] == 100 and std["factor"] == pytest.approx(std["proposed"]["p"] / 105, abs=1e-4)
  assert std["iDelta"] == std["proposed"]["i"] - 75


def test_refine_trial_skips_a_plant_for_another_car():
  trial = {"bands": [], "baseline": {"gains": GAINS, "raw": {}}}
  lts.refine_trial(trial, [{"cp_bytes": _cp_bytes(), "route": "x"}], PLANT, {"car": "HONDA_ACCORD"})
  assert trial["sim"]["status"].startswith("skipped")
