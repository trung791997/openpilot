#!/usr/bin/env python3
"""Episode analysis of frames_<route>.json written by ab_nccap.py."""
import json
import os
from pathlib import Path

SP = Path(os.environ["SP"])  # REQUIRED, same dir as nccap_ab.py
EPISODES = [("00000271--4e9b9502db", 566.0), ("00000236--60bfb34cb1", 771.0), ("00000236--60bfb34cb1", 774.0),
            ("00000237--77313c5a66", 600.0), ("00000298--c4d2a4acbc", 250.0), ("00000297--f971b5896f", 2892.0)]
SERIES = {"00000297--f971b5896f": (2891.5, 2893.6), "00000298--c4d2a4acbc": (248.5, 251.5)}


def mmss(t):
  return f"{int(t // 60)}:{t - 60 * int(t // 60):05.2f}"


def r(x, n=2):
  return None if x is None else round(x, n)


def ep_stats(frames, v):
  out = {}
  leads = [(f, f["v"][v]["lead"]) for f in frames if f["v"][v]["lead"] is not None]
  if leads:
    f, l = min(leads, key=lambda p: p[1]["vRel"])
    trk = f["v"][v].get("trk") or {}
    out["min_vRel"] = {"vRel": r(l["vRel"]), "t": mmss(f["t"]), "tid": l["tid"], "d": r(l["d"]),
                       "native": r(trk.get("vRel")), "ncVRel": r(trk.get("ncVRel")), "ncValid": trk.get("ncValid"),
                       "corr": r(trk.get("corr")), "on_rail": trk.get("on_rail"),
                       "nc_raw": trk.get("nc_raw"), "nc_sigma": trk.get("nc_sigma"), "nc_diag": r(trk.get("nc_diag_vrel"))}
    ds = [l["d"] for _, l in leads]
    out["dRel_range"] = [r(min(ds)), r(max(ds))]
  fr = [f for f in frames if f["fresh"] and f["v"][v].get("trk")]
  out["lead_sweeps"] = len(fr)
  out["ncValid_frac"] = r(sum(f["v"][v]["trk"]["ncValid"] for f in fr) / len(fr), 3) if fr else None
  rail = [f for f in frames if f["v"][v].get("trk") and f["v"][v]["trk"]["on_rail"]]
  if rail:
    f = max(rail, key=lambda f: f["v"][v]["trk"]["corr"])
    out["max_rail_corr"] = {"corr": r(f["v"][v]["trk"]["corr"]), "t": mmss(f["t"]), "tid": f["v"][v]["lead"]["tid"],
                            "ncValid": f["v"][v]["trk"]["ncValid"], "ncVRel": r(f["v"][v]["trk"]["ncVRel"])}
  out["rail_frames"] = len(rail)
  allc = [f for f in frames if f["v"][v].get("trk")]
  if allc:
    f = max(allc, key=lambda f: f["v"][v]["trk"]["corr"])
    out["max_corr_any"] = {"corr": r(f["v"][v]["trk"]["corr"]), "t": mmss(f["t"]), "on_rail": f["v"][v]["trk"]["on_rail"]}
  f = min(frames, key=lambda f: f["v"][v]["a"])
  out["planner_min"] = {"a": r(f["v"][v]["a"]), "t": mmss(f["t"]), "src": f["v"][v]["src"]}
  return out


def diffs(frames, a, b):
  dv = []
  da = []
  sel = 0
  for f in frames:
    la, lb = f["v"][a]["lead"], f["v"][b]["lead"]
    if (la is None) != (lb is None) or (la and lb and la["tid"] != lb["tid"]):
      sel += 1
    elif la and lb and abs(la["vRel"] - lb["vRel"]) > 0.01:
      dv.append((abs(la["vRel"] - lb["vRel"]), f["t"], la["vRel"], lb["vRel"]))
    x = abs(f["v"][a]["a"] - f["v"][b]["a"])
    if x > 0.01:
      da.append((x, f["t"], f["v"][a]["a"], f["v"][b]["a"]))
  o = {"lead_select_diff": sel, "vRel_diff_frames": len(dv), "accel_diff_frames": len(da)}
  if dv:
    m = max(dv)
    o["vRel_max"] = {"absdiff": r(m[0]), "t": mmss(m[1]), a: r(m[2]), b: r(m[3])}
    o["vRel_diff_span"] = [mmss(min(x[1] for x in dv)), mmss(max(x[1] for x in dv))]
  if da:
    m = max(da)
    o["accel_max"] = {"absdiff": r(m[0], 3), "t": mmss(m[1]), a: r(m[2]), b: r(m[3])}
  return o


def series(frames, lo, hi):
  rows = []
  for f in frames:
    if not (lo <= f["t_lt"] <= hi) or not f["fresh"]:
      continue
    lo_, ln = f["v"]["OFF"]["lead"], f["v"]["ON"]["lead"]
    to, tn = f["v"]["OFF"].get("trk") or {}, f["v"]["ON"].get("trk") or {}
    rows.append({"t": mmss(f["t_lt"]), "t_s": r(f["t_lt"], 3), "tid": lo_["tid"] if lo_ else None,
                 "tid_ON": ln["tid"] if ln else None, "d": r(to.get("d")), "native": r(to.get("vRel")),
                 "ncVRel": r(to.get("ncVRel")), "ncValid": to.get("ncValid"), "nc_raw": to.get("nc_raw"),
                 "nc_sigma": to.get("nc_sigma"), "nc_diag": r(to.get("nc_diag_vrel")), "on_rail": to.get("on_rail"),
                 "corr_OFF": r(to.get("corr")), "corr_ON": r(tn.get("corr")),
                 "pub_OFF": r(lo_["vRel"]) if lo_ else None, "pub_ON": r(ln["vRel"]) if ln else None,
                 "a_OFF": r(f["v"]["OFF"]["a"]), "a_ON": r(f["v"]["ON"]["a"])})
  return rows


def main():
  res = {"episodes": [], "series": {}, "meta": {}}
  cache = {}
  for route, ep in EPISODES:
    if route not in cache:
      cache[route] = json.loads((SP / f"frames_{route}.json").read_text())
      res["meta"][route] = cache[route]["meta"]
    fr = [f for f in cache[route]["frames"] if ep - 3 <= f["t"] <= ep + 5]
    e = {"route": route, "episode": mmss(ep), "window": [mmss(ep - 3), mmss(ep + 5)], "frames": len(fr),
         "aEgo_min": {"a": r(min(f["aEgo"] for f in fr)), "t": mmss(min(fr, key=lambda f: f["aEgo"])["t"])},
         "OFF": ep_stats(fr, "OFF"), "ON": ep_stats(fr, "ON"),
         "ON_vs_OFF": diffs(fr, "OFF", "ON"), "AA_OFF_vs_OFF2": diffs(fr, "OFF", "OFF2")}
    res["episodes"].append(e)
  for route, (lo, hi) in SERIES.items():
    res["series"][route] = series(cache[route]["frames"], lo, hi)
  # ON check: published more closing than ncVRel - 3 while ncValid and on rail (ON lead track, all window frames)
  viol = []
  for route in cache:
    for f in cache[route]["frames"]:
      l, t = f["v"]["ON"]["lead"], f["v"]["ON"].get("trk")
      if l and t and t["ncValid"] and t["on_rail"] and l["vRel"] < t["ncVRel"] - 3.0 - 1e-6:
        viol.append({"route": route, "t": mmss(f["t"]), "pub": r(l["vRel"]), "nc": r(t["ncVRel"]), "corr": r(t["corr"])})
  res["ON_cap_violations"] = viol
  (SP / "ab_results.json").write_text(json.dumps(res, indent=1))
  print(json.dumps(res, indent=1))


if __name__ == "__main__":
  main()
