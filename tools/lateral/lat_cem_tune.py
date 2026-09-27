#!/usr/bin/env python3
"""
Cross-entropy-method search for lateral gain schedule.
Plant parameters mapped for perturbations:
  - delay: plant.delay
  - gain: c[4], c[5], c[6] (coefficients for torque command u)
  - damping: c[0], c[1], c[8] (coefficients for steering rate)
MetaDrive episodes (--sim-route) enter the replay pool at weight 0.25. An episode that left the road does not
change the replay cost (every candidate would pay the same constant); instead its episode.json "theta" (the 8
schedule values it was driven with) becomes a mistake memory: mistake_penalty adds 5.0 * exp(-|x - theta|^2 / (2 * 10^2))
to every candidate near it, so the search moves away from gains that failed on-policy.
"""
import argparse
import json
import math
import multiprocessing
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import lat_pid_sim as sim
import lat_autotune as at

def perturb_plant(base_plant, rng):
    delay_delta = rng.integers(-1, 3) # -1, 0, 1, 2
    gain_mult = rng.uniform(0.8, 1.2)
    damp_mult = rng.uniform(0.8, 1.2)
    c = base_plant.c.copy()
    c[4:7] *= gain_mult
    c[0:2] *= damp_mult
    c[8] *= damp_mult
    delay = max(0, base_plant.delay + delay_delta)
    return sim.Plant(c, delay, base_plant.quant, base_plant.eps)

def evaluate_plants(args):
    overrides, plants_json, kind, torque = args
    plants = [sim.Plant.from_json(pj) for pj in plants_json]
    res_list = []
    for p in plants:
        res = []
        for d in at._DS:
            ang, des, _, _ = sim.simulate(d, p, overrides, kind=kind, torque=torque)
            res.append({name: sim.metrics(d, ang, des, m) for name, m in at._masks(d).items()})
            res[-1]["is_sim_route"] = d.get("is_sim_route", False)
            res[-1]["offroad"] = d.get("offroad", False)
            res[-1]["route"] = d["route"]
        res_list.append(res)
    return res_list

def compute_cost(res_list, seed_res_list, trusted, mask):
    def get_plant_costs(r_list, ref_list):
        costs = []
        for p in range(len(r_list)):
            c_tot, w_tot = 0.0, 0.0
            for i in range(len(r_list[p])):
                route = r_list[p][i]
                ref = ref_list[p][i]
                is_sim = route.get("is_sim_route", False)
                for b in trusted:
                    r = route[mask][b]
                    s = ref[mask][b]
                    if r is None or s is None:
                        continue
                    w = r["min"]
                    if is_sim:
                        w *= 0.25
                    c = at.band_cost(r, s)
                    c_tot += c * w
                    w_tot += w
            costs.append(c_tot / w_tot if w_tot > 0 else 0.0)
        return np.array(costs)

    C = get_plant_costs(res_list, seed_res_list)
    S = get_plant_costs(seed_res_list, seed_res_list)
    cand_c = np.mean(C) + 0.5 * np.std(C)
    seed_c = np.mean(S) + 0.5 * np.std(S)
    return cand_c / seed_c if seed_c > 0 else float('inf')

def mistake_penalty(x, fail_thetas, width=10.0, weight=5.0):
    """Penalty for candidate x near schedules that left the road in MetaDrive (RBF of `width` schedule points)."""
    x = np.asarray(x, dtype=float)
    return sum(weight * math.exp(-float(np.sum((x - np.asarray(t, dtype=float)) ** 2)) / (2 * width ** 2)) for t in fail_thetas)

def check_verdict(cand_res_list, seed_res_list, trusted):
    for p in range(len(cand_res_list)):
        c_tot, s_tot = 0.0, 0.0
        c_band = dict.fromkeys(trusted, 0.0)
        s_band = dict.fromkeys(trusted, 0.0)
        w_band = dict.fromkeys(trusted, 0.0)
        for i in range(len(cand_res_list[p])):
            route = cand_res_list[p][i]
            s_route = seed_res_list[p][i]
            is_sim = route.get("is_sim_route", False)

            for b in trusted:
                r = route["holdout"][b]
                s = s_route["holdout"][b]
                if r is None or s is None:
                    continue
                w = r["min"]
                if is_sim:
                    w *= 0.25

                c = at.band_cost(r, s)
                s_c = at.band_cost(s, s)

                c_band[b] += c * w
                s_band[b] += s_c * w
                w_band[b] += w
                c_tot += c * w
                s_tot += s_c * w

        if s_tot > 0 and c_tot / s_tot >= 0.97:
            return False
        for b in trusted:
            if w_band[b] > 0 and (c_band[b] / s_band[b]) > 1.02:
                return False
    return True

def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--plant", required=True)
    ap.add_argument("routes", nargs="+")
    ap.add_argument("--sim-route", action="append", default=[])
    ap.add_argument("--seed-params", default=None)
    ap.add_argument("--pop", type=int, default=24)
    ap.add_argument("--elites", type=int, default=6)
    ap.add_argument("--gens", type=int, default=6)
    ap.add_argument("--std", type=float, default=15.0)
    ap.add_argument("--trust", type=float, default=40.0)
    ap.add_argument("--rng", type=int, default=0)
    ap.add_argument("--plants", type=int, default=6)
    ap.add_argument("-j", "--jobs", type=int, default=1)
    ap.add_argument("--kind", choices=("pid", "clarity_eps"), default="pid",
                    help="pid: theta = LatGainSchedule p,i at 4 knots (percent); clarity_eps: theta = James's controller's "
                    + "per-band P trim x3, I trim x3 (percent) and output LPF tau x3 (ms), sim-only trims")
    ap.add_argument("--out", required=True)
    args = ap.parse_args(argv)

    with open(args.plant) as f:
        base_plant = sim.Plant.from_json(json.load(f))

    at._DS = []
    for r in args.routes:
        d = sim.extract(r)
        d["is_sim_route"] = False
        d["offroad"] = False
        at._DS.append(d)

    fail_thetas = []
    for r in args.sim_route:
        d = sim.extract(r)
        d["is_sim_route"] = True
        d["offroad"] = False
        ep_path = os.path.join(r, "episode.json")
        if os.path.exists(ep_path):
            with open(ep_path) as f:
                ep = json.load(f)
            d["offroad"] = bool(ep.get("offroad", False))
            if d["offroad"] and len(ep.get("theta", [])) == (8 if args.kind == "pid" else 9):
                fail_thetas.append(ep["theta"])
        at._DS.append(d)

    if args.seed_params:
        with open(args.seed_params) as f:
            seed_params = json.load(f)
    else:
        seed_params = dict(at._DS[0]["params"])

    knots = [20.0, 30.0, 40.0, 50.0]

    driven_res = [ {name: sim.metrics(d, *sim.simulate(d, base_plant)[:2], m) for name, m in at._masks(d).items()} for d in at._DS ]
    log_res = at.logged_metrics()
    gate = at.trust_gate(driven_res, log_res)
    trusted_bands = [name for name, (ok, _) in gate.items() if ok]

    if not trusted_bands:
        print("No trusted bands.")
        return 1

    if args.kind == "pid":
        seed_p = at.seed_values(seed_params, knots, "p")
        seed_i = at.seed_values(seed_params, knots, "i")
        seed_f = at.seed_values(seed_params, knots, "f")
        x_seed = np.array(seed_p + seed_i, dtype=float)
        lim_lo = [at.LIMITS["p"][0]] * 4 + [at.LIMITS["i"][0]] * 4
        lim_hi = [at.LIMITS["p"][1]] * 4 + [at.LIMITS["i"][1]] * 4

        def make_job(x):
            return {"LatGainSchedule": at.schedule_json(knots, ["p", "i", "f"], list(x) + seed_f)}, None
    else:
        from openpilot.selfdrive.controls.lib import nrdr_eps_firmware_ff as ff
        x_seed = np.array([100.0 * v for v in ff.CIVIC_P_SCALE] + [100.0 * v for v in ff.CIVIC_I_SCALE]
                          + [1000.0 * v for v in ff.OUTPUT_LPF_TAU], dtype=float)
        lim_lo = [at.LIMITS["p"][0]] * 3 + [at.LIMITS["i"][0]] * 3 + [1.0] * 3
        lim_hi = [at.LIMITS["p"][1]] * 3 + [at.LIMITS["i"][1]] * 3 + [300.0] * 3

        def make_job(x):
            x = [float(v) for v in x]
            return {}, {"p_scale": [v / 100.0 for v in x[0:3]], "i_scale": [v / 100.0 for v in x[3:6]],
                        "out_tau": [v / 1000.0 for v in x[6:9]]}

    bounds_lo = np.array([max(lo, v - args.trust) for lo, v in zip(lim_lo, x_seed, strict=True)])
    bounds_hi = np.array([min(hi, v + args.trust) for hi, v in zip(lim_hi, x_seed, strict=True)])

    rng = np.random.default_rng(args.rng)

    mu = x_seed.copy()
    std = np.full_like(mu, args.std)

    generations_out = []

    ctx = multiprocessing.get_context("fork")
    with ctx.Pool(args.jobs) as pool:
        for g in range(args.gens):
            plants = [perturb_plant(base_plant, rng) for _ in range(args.plants)]
            plants_json = [p.to_json() for p in plants]

            seed_overrides, seed_torque = make_job(x_seed)

            cand_xs = []
            for _ in range(args.pop):
                x = rng.normal(mu, std)
                x = np.clip(x, bounds_lo, bounds_hi)
                cand_xs.append(x)

            jobs = [(seed_overrides, plants_json, args.kind, seed_torque)]
            for x in cand_xs:
                o, tq = make_job(x)
                jobs.append((o, plants_json, args.kind, tq))

            results = pool.map(evaluate_plants, jobs)
            seed_res_list = results[0]
            cand_res_lists = results[1:]

            costs = []
            for x, crl in zip(cand_xs, cand_res_lists, strict=True):
                costs.append(compute_cost(crl, seed_res_list, trusted_bands, "fit") + mistake_penalty(x, fail_thetas))

            costs = np.array(costs)
            elites_idx = np.argsort(costs)[:args.elites]
            elite_xs = np.array([cand_xs[i] for i in elites_idx])

            best_cost = costs[elites_idx[0]]
            elite_mean = np.mean(elite_xs, axis=0)
            elite_std = np.std(elite_xs, axis=0)

            mu = 0.7 * mu + 0.3 * elite_mean
            std = np.maximum(2.0, 0.7 * std + 0.3 * elite_std)

            generations_out.append({
                "mean": mu.tolist(),
                "std": std.tolist(),
                "elites": elite_xs.tolist(),
                "best_cost": best_cost
            })
            print(f"Gen {g}: best cost {best_cost:.4f}")

        best_x = elite_xs[0]
        best_res_list = cand_res_lists[elites_idx[0]]
        verdict = check_verdict(best_res_list, seed_res_list, trusted_bands)

        best_over, best_tq = make_job(best_x)
        final_sched = best_over["LatGainSchedule"] if args.kind == "pid" else json.dumps(best_tq)

        holdout_out = {
            "seed_cost": 1.0,
            "best_cost": compute_cost(best_res_list, seed_res_list, trusted_bands, "holdout"),
            "per_band": {}
        }
        for b in trusted_bands:
            c_tot, s_tot = 0.0, 0.0
            for p in range(args.plants):
                for route_idx, route in enumerate(best_res_list[p]):
                    s_route = seed_res_list[p][route_idx]
                    r = route["holdout"][b]
                    s = s_route["holdout"][b]
                    if r and s:
                        w = r["min"] * (0.25 if route.get("is_sim_route") else 1.0)
                        c_tot += at.band_cost(r, s) * w
                        s_tot += at.band_cost(s, s) * w
            holdout_out["per_band"][b] = c_tot / s_tot if s_tot > 0 else 1.0

    res_json = {
        "kind": args.kind,
        "seed": seed_overrides["LatGainSchedule"] if args.kind == "pid" else json.dumps(seed_torque),
        "generations": generations_out,
        "best_theta": best_x.tolist(),
        "holdout": holdout_out,
        "plants": plants_json,
        "routes": [os.path.basename(r.rstrip("/")) for r in args.routes],
        "sim_routes": [os.path.basename(r.rstrip("/")) for r in args.sim_route],
        "fail_thetas": fail_thetas,
        "schedule": final_sched
    }

    with open(args.out, "w") as f:
        json.dump(res_json, f, indent=2)

    print("\nVerdict:", "RECOMMENDED" if verdict else "REJECTED")
    print(final_sched)
    return 0

if __name__ == "__main__":
    sys.exit(main())
