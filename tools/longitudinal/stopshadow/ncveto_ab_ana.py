#!/usr/bin/env python3
"""Summarise ncveto_ab.py frames (REPLAY only): per episode OFF vs ON lead1 vRel / rail corr / planner, A/A diffs.

Usage: ncveto_ab_ana.py <frames.json> <label> <ep_t> [<ep_t> ...]   (window ep-3..ep+5 s, t on the seg-0 clock)
"""
import json
import sys


def fmt(t):
  return f"{int(t // 60)}:{t % 60:05.2f}"


def g(fr, v, *ks):
  x = fr["v"][v]
  for k in ks:
    if x is None:
      return None
    x = x.get(k)
  return x


def main():
  data = json.load(open(sys.argv[1]))
  label = sys.argv[2]
  for ep in (float(e) for e in sys.argv[3:]):
    F = [f for f in data["frames"] if ep - 3 <= f["t"] <= ep + 5]
    row = {}
    for v in ("OFF", "ON"):
      vs = [(g(f, v, "lead", "vRel"), f["t"], g(f, v, "lead", "tid"), g(f, v, "lead", "d")) for f in F if g(f, v, "lead") is not None]
      mn = min(vs) if vs else None
      corr = max(((g(f, v, "trk", "corr") or 0.0), f["t"]) for f in F)
      pmin = min((f["v"][v]["a"], f["t"]) for f in F)
      row[v] = (mn, corr, pmin)
    dv = [f for f in F if g(f, "OFF", "lead", "vRel") != g(f, "ON", "lead", "vRel")]
    da = [f for f in F if f["v"]["OFF"]["a"] != f["v"]["ON"]["a"]]
    aa = [f for f in F if g(f, "OFF", "lead") != g(f, "OFF2", "lead") or f["v"]["OFF"]["a"] != f["v"]["OFF2"]["a"]]
    maxda = max((abs(f["v"]["ON"]["a"] - f["v"]["OFF"]["a"]) for f in da), default=0.0)
    harder = max((f["v"]["OFF"]["a"] - f["v"]["ON"]["a"] for f in da), default=0.0)
    o, n = row["OFF"], row["ON"]
    if o[0] is None or n[0] is None:
      head = f"| {label} {fmt(ep)} | no lead1 (OFF {o[0]}, ON {n[0]}) | "
      print(head + f"| {o[1][0]:.2f} / {n[1][0]:.2f} | {o[2][0]:.2f} / {n[2][0]:.2f} | {len(dv)} | {len(da)} | {len(aa)} |")
      continue
    head = f"| {label} {fmt(ep)} | {o[0][0]:.2f} @ {fmt(o[0][1])} (tid {o[0][2]}, d {o[0][3]:.0f}) | {n[0][0]:.2f} @ {fmt(n[0][1])} | "
    body = f"{o[1][0]:.2f} / {n[1][0]:.2f} | {o[2][0]:.2f} / {n[2][0]:.2f} | {len(dv)} | "
    tail = f"{len(da)} (max |da| {maxda:.2f}, ON harder by <= {max(harder, 0):.2f}) | {len(aa)} |"
    print(head + body + tail)
    for f in dv:
      a = f"    {fmt(f['t'])} tid {g(f, 'OFF', 'lead', 'tid')} d {g(f, 'OFF', 'lead', 'd'):.1f} "
      b = f"vRel OFF {g(f, 'OFF', 'lead', 'vRel'):.2f} ON {g(f, 'ON', 'lead', 'vRel'):.2f} "
      c = f"corr OFF {g(f, 'OFF', 'trk', 'corr')} ON {g(f, 'ON', 'trk', 'corr')} nc_med {g(f, 'ON', 'trk', 'nc_med')} "
      print(a + b + c + f"a OFF {f['v']['OFF']['a']:.2f} ON {f['v']['ON']['a']:.2f}")


if __name__ == "__main__":
  main()
