import json
import os
import sys
import tempfile
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import lat_cem_tune as cem
import lat_pid_sim as sim
import lat_autotune as at

from opendbc.car.honda.values import CAR as HONDA, HondaFlags
from opendbc.car.car_helpers import interfaces

def _modified_cp_bytes():
    from types import SimpleNamespace
    from opendbc.car import gen_empty_fingerprint
    from opendbc.car.structs import CarParams
    fw = [CarParams.CarFw(ecu=CarParams.Ecu.eps, fwVersion=b'39990-TBA,C020\x00\x00', address=0x18DA30F1, subAddress=0)]
    toggles = SimpleNamespace(force_torque_controller=False, nnff=False, nnff_lite=False)
    CP = interfaces[HONDA.HONDA_CIVIC_BOSCH].get_params(HONDA.HONDA_CIVIC_BOSCH, gen_empty_fingerprint(), fw, False, False, False, toggles)
    CP.flags |= int(HondaFlags.EPS_MODIFIED) | int(HondaFlags.VGR_CIVIC_TBA_C020)
    CP.dashcamOnly = True
    return CP.to_bytes()

def _route(curv=0.004, v=20.0, n=1500, active_minutes=5.0):
    d = {k: np.zeros(n) for k in sim.FIELDS}
    d["t"] = np.arange(n) * sim.DT
    d["v"][:] = v
    d["active"][:] = 1.0
    d["sr"][:] = 16.0
    d["stiff"][:] = 1.0
    d["des_curv"][300:] = curv
    d["cp_bytes"] = _modified_cp_bytes()
    d["params"] = {"LatPScaleStandard": "100", "LatIScaleStandard": "20"}
    d["route"] = "synthetic"

    # We want valid metrics to be extracted. We will use a realistic angle.
    d["angle"] = np.zeros(n)
    d["angle"][350:] = 0.5 * curv * 1000
    d["des_angle"] = d["des_curv"] * 1000
    return d

def test_lat_cem_tune_runs(monkeypatch):
    with tempfile.TemporaryDirectory() as tmpdir:
        plant_path = os.path.join(tmpdir, "plant.json")
        out_path = os.path.join(tmpdir, "results.json")
        route1 = os.path.join(tmpdir, "route1")
        sim_route = os.path.join(tmpdir, "sim_route")

        # plant
        plant = sim.Plant([-8.0, 0.0, -2.0, 0.0, 300.0, 0.0, 0.0, 0.0, 0.0], 5, 0.1)
        with open(plant_path, "w") as f:
            json.dump(plant.to_json(), f)

        os.makedirs(sim_route, exist_ok=True)
        with open(os.path.join(sim_route, "episode.json"), "w") as f:
            json.dump({"offroad": True}, f)

        params_path = os.path.join(tmpdir, "params.json")
        with open(params_path, "w") as f:
            json.dump({"LatPScaleHighway": "105", "LatIScaleHighway": "5"}, f)

        # mock route will give a v > 50 mph so it ends up in "highway >50" band
        def mock_extract(path):
            return _route(v=30.0) # 30 m/s = 67 mph

        monkeypatch.setattr(sim, "extract", mock_extract)
        args = [
            "--plant", plant_path,
            route1,
            "--sim-route", sim_route,
            "--seed-params", params_path,
            "--pop", "4",
            "--elites", "2",
            "--gens", "1",
            "--std", "15",
            "--trust", "40",
            "--rng", "42",
            "--plants", "2",
            "--out", out_path
        ]

        # mock trust_gate so we don't have to perfectly tune the synthetic data to be trusted
        # mock metrics to ensure we have min > 0

        def mock_metrics(*args, **kwargs):
            band = {"min": 5.0, "err_rms": 1.0, "straight_rms": 0.5, "curve_ratio": 1.0, "curve_s": 1.0,
                    "zero_cross": 0.3, "sign_hyst": 0.0, "bias": 0.0, "lag_s": 0.0}
            # sim.metrics returns a dict keyed by band name; mock the same shape so lat_cem_tune's
            # route[mask][band] lookups succeed regardless of which mask/band is requested.
            return {name: band for name, _, _ in sim.BANDS}

        monkeypatch.setattr(at, "trust_gate", lambda *a, **kw: {"highway >50": (True, "mocked")})
        monkeypatch.setattr(sim, "metrics", mock_metrics)
        ret = cem.main(args)
        assert ret == 0 or ret is None

        with open(out_path) as f:
            res = json.load(f)

        assert "seed" in res
        assert "generations" in res
        assert len(res["generations"]) == 1
        assert "best_theta" in res
        assert "holdout" in res
        assert "plants" in res
        assert len(res["plants"]) == 2 # args.plants = 2

        # Test 1: Seed evaluates to cost ratio 1.0 on the seed plant.
        # (Verified by logic checking seed_cost vs seed_cost returning 1.0)
        assert res["holdout"]["seed_cost"] == 1.0

        # Test 2: Writes results.json with every key and best_theta in bounds
        theta = res["best_theta"]
        assert len(theta) == 8 # 4 p, 4 i

        seed_sched = json.loads(res["seed"])
        seed_p = seed_sched["p"]
        seed_i = seed_sched["i"]
        seed_theta = seed_p + seed_i

        # check trust region bounds
        for j, (t, s) in enumerate(zip(theta, seed_theta, strict=False)):
            assert s - 40 <= t <= s + 40, f"theta[{j}] = {t} not in [{s-40}, {s+40}]"
            if j < 4:
                assert 25 <= t <= 300, "p bounds"
            else:
                assert 0 <= t <= 300, "i bounds"

        # Test 3: a sim route carries weight 0.25 and an off-road flag never changes the replay cost ratio;
        # the off-road episode's theta penalises candidates near it instead.
        band = {"min": 4.0, "err_rms": 1.0, "straight_rms": 0.5, "curve_ratio": 1.0, "zero_cross": 0.3}
        route_reg = {"holdout": {"highway >50": band}, "is_sim_route": False, "offroad": False}
        route_sim = {"holdout": {"highway >50": band}, "is_sim_route": True, "offroad": True}
        c_list = [[route_reg, route_sim]]
        trusted = ["highway >50"]
        assert cem.compute_cost(c_list, c_list, trusted, "holdout") == 1.0
        fail = [100.0] * 8
        assert cem.mistake_penalty(fail, [fail]) == 5.0
        assert abs(cem.mistake_penalty([110.0] + [100.0] * 7, [fail]) - 5.0 * np.exp(-0.5)) < 1e-9
        assert cem.mistake_penalty([200.0] * 8, [fail]) < 1e-6
        assert cem.mistake_penalty(fail, []) == 0.0

        # test deterministic ensemble
        rng = np.random.default_rng(42)
        base = sim.Plant([1,2,3,4,5,6,7,8,9], 5, 0.1)
        p1 = cem.perturb_plant(base, rng)

        rng = np.random.default_rng(42)
        p2 = cem.perturb_plant(base, rng)

        assert np.array_equal(p1.c, p2.c)
        assert p1.delay == p2.delay


def test_lat_cem_tune_clarity_kind(monkeypatch):
    """--kind clarity_eps searches James's controller's per-band trims (sim-only) instead of LatGainSchedule."""
    from openpilot.selfdrive.controls.lib import nrdr_eps_firmware_ff as ff
    with tempfile.TemporaryDirectory() as tmpdir:
        plant_path = os.path.join(tmpdir, "plant.json")
        out_path = os.path.join(tmpdir, "results.json")
        with open(plant_path, "w") as f:
            json.dump(sim.Plant([-8.0, 0.0, -2.0, 0.0, 300.0, 0.0, 0.0, 0.0, 0.0], 5, 0.1).to_json(), f)
        monkeypatch.setattr(sim, "extract", lambda path: _route(v=30.0))
        band = {"min": 5.0, "err_rms": 1.0, "straight_rms": 0.5, "curve_ratio": 1.0, "curve_s": 1.0,
                "zero_cross": 0.3, "sign_hyst": 0.0, "bias": 0.0, "lag_s": 0.0}
        monkeypatch.setattr(sim, "metrics", lambda *a, **kw: {name: band for name, _, _ in sim.BANDS})
        monkeypatch.setattr(at, "trust_gate", lambda *a, **kw: {"highway >50": (True, "mocked")})
        ret = cem.main(["--plant", plant_path, os.path.join(tmpdir, "r"), "--kind", "clarity_eps", "--pop", "3", "--elites", "1",
                        "--gens", "1", "--plants", "1", "--rng", "1", "--out", out_path])
        assert ret == 0
        with open(out_path) as f:
            res = json.load(f)
        assert res["kind"] == "clarity_eps" and len(res["best_theta"]) == 9
        seed = json.loads(res["seed"])
        assert tuple(seed["p_scale"]) == ff.CIVIC_P_SCALE and tuple(seed["out_tau"]) == ff.OUTPUT_LPF_TAU
        # the trims reach the controller core (the pool workers are forked, so check the constructor directly)
        d = _route(v=30.0)
        c = sim.Controller(d["cp_bytes"], d["params"], kind="clarity_eps", torque={"p_scale": [1.0, 1.1, 1.2], "i_scale": "0.5,0.6,0.7",
                                                                                    "out_tau": [0.1, 0.05, 0.02]})
        assert (c.lac.core.p_scale, c.lac.core.i_scale, c.lac.core.output_lpf_tau) == ((1.0, 1.1, 1.2), (0.5, 0.6, 0.7), (0.1, 0.05, 0.02))
        c0 = sim.Controller(d["cp_bytes"], d["params"], kind="clarity_eps")
        assert (c0.lac.core.p_scale, c0.lac.core.i_scale, c0.lac.core.output_lpf_tau) == (ff.CIVIC_P_SCALE, ff.CIVIC_I_SCALE, ff.OUTPUT_LPF_TAU)
        for j, (t, s) in enumerate(zip(res["best_theta"], [100 * v for v in ff.CIVIC_P_SCALE + ff.CIVIC_I_SCALE]
                                       + [1000 * v for v in ff.OUTPUT_LPF_TAU], strict=True)):
            assert s - 40 <= t <= s + 40 and (t >= 1.0 if j >= 6 else True)

