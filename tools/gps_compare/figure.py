#!/usr/bin/env python3
"""compare.png: a map of one stretch with both traces over the OSM roads, the pooled cross-track histogram, and the
share of driving time each source had a usable fix. Also prints pooled stats. Usage: figure.py DIR ROUTE MM:SS-MM:SS"""

import sys
import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, sys.path[0])
import analyze as A

DIR, MAP_ROUTE, SPAN = sys.argv[1], sys.argv[2], sys.argv[3]
CAR, COMMA, ROAD, INK, INK2 = "#2a78d6", "#eb6834", "#c9c8c2", "#0b0b0b", "#52514e"
A.DIR = DIR
routes = sorted({f.rsplit("_", 1)[0] for f in __import__("os").listdir(DIR) if f.endswith("_hu.csv")})
pool = {"comma": [], "car": []}
both = {"comma": [], "car": []}
usable = []
data = {}
for r in routes:
  res, (c, h, v, roads, proj, t0) = A.analyze(r)
  data[r] = (c, h, v, roads, proj, t0)
  for name, s in (("comma", c), ("car", h)):
    pool[name].append(s["xt"][s["mv"] & s["ok"]])
  usable.append((r[:8], res["comma"]["usable_frac_moving_time"], res["car"]["usable_frac_moving_time"]))
  # paired: car fixes with a usable comma fix within 0.25 s
  cm = c["ok"] & np.isfinite(c["x"])
  if cm.sum() > 300:
    tc = c["t"][cm]
    j = np.clip(np.searchsorted(tc, h["t"]), 1, len(tc) - 1)
    near = np.minimum(np.abs(tc[j] - h["t"]), np.abs(tc[j - 1] - h["t"])) < 0.25e9
    m = h["ok"] & h["mv"] & near
    cx = np.interp(h["t"][m], tc, c["x"][cm])
    cy = np.interp(h["t"][m], tc, c["y"][cm])
    both["comma"].append(roads.match(cx, cy, h["bearing"][m])[0])
    both["car"].append(h["xt"][m])
for k in pool:
  pool[k] = np.concatenate(pool[k])
  both[k] = np.concatenate(both[k])
  print(
    k,
    "pooled usable xtrack p50/p95/max",
    A.pct(pool[k]),
    "n",
    len(pool[k]),
    "| paired same instants",
    A.pct(both[k]),
    "n",
    len(both[k]),
    f"| share > 10 m: all {100 * np.mean(pool[k] > 10):.1f}% paired {100 * np.mean(both[k] > 10):.1f}%",
  )

fig = plt.figure(figsize=(13, 6.2), dpi=130)
gs = fig.add_gridspec(2, 2, width_ratios=[1.25, 1], height_ratios=[1.3, 1], wspace=0.22, hspace=0.45)
# map
c, h, v, roads, proj, t0 = data[MAP_ROUTE]
a, b = [int(x.split(":")[0]) * 60 + int(x.split(":")[1]) for x in SPAN.split("-")]
ax = fig.add_subplot(gs[:, 0])
cw = ((c["t"] - t0) / 1e9 >= a) & ((c["t"] - t0) / 1e9 <= b) & np.isfinite(c["x"])
hw = ((h["t"] - t0) / 1e9 >= a) & ((h["t"] - t0) / 1e9 <= b)
xs = np.concatenate([c["x"][cw], h["x"][hw]])
ys = np.concatenate([c["y"][cw], h["y"][hw]])
pad = 60
x0, x1, y0, y1 = xs.min() - pad, xs.max() + pad, ys.min() - pad, ys.max() + pad
span = max(x1 - x0, y1 - y0)
cxm, cym = (x0 + x1) / 2, (y0 + y1) / 2
x0, x1, y0, y1 = cxm - span / 2, cxm + span / 2, cym - span / 2, cym + span / 2
s = roads.s
inb = ((s[:, 0] > x0) & (s[:, 0] < x1) & (s[:, 1] > y0) & (s[:, 1] < y1)) | ((s[:, 2] > x0) & (s[:, 2] < x1) & (s[:, 3] > y0) & (s[:, 3] < y1))
for seg in s[inb]:
  ax.plot([seg[0], seg[2]], [seg[1], seg[3]], color=ROAD, lw=3, solid_capstyle="round", zorder=1)
ax.plot(c["x"][cw], c["y"][cw], color=COMMA, lw=2, zorder=3, label="comma GNSS (10 Hz)")
ax.plot(h["x"][hw], h["y"][hw], color=CAR, lw=2, zorder=4, label="car GPS via head unit (1 Hz)")
ax.scatter(h["x"][hw], h["y"][hw], s=14, color=CAR, edgecolor="white", linewidth=1, zorder=5)
for tt in range(a, b + 1, 5 if b - a <= 40 else 10):  # time ticks on the car trace
  k = np.argmin(np.abs((h["t"] - t0) / 1e9 - tt))
  if h["v"][k] < 2:
    continue
  ax.annotate(A.mmss(tt), (h["x"][k], h["y"][k]), xytext=(6, 4), textcoords="offset points", fontsize=7, color=INK2, zorder=6)
ax.set_xlim(x0, x1)
ax.set_ylim(y0, y1)
ax.set_aspect("equal")
ax.set_xticks([])
ax.set_yticks([])
for sp in ax.spines.values():
  sp.set_color(ROAD)
bar = 100 if span > 300 else 50
ax.plot([x0 + 20, x0 + 20 + bar], [y0 + 20, y0 + 20], color=INK, lw=2)
ax.text(x0 + 20 + bar / 2, y0 + 28, f"{bar} m", ha="center", fontsize=8, color=INK)
ax.set_title(f"Route {MAP_ROUTE}  {SPAN}  (grey = OpenStreetMap roads)", fontsize=10, color=INK, loc="left")
ax.legend(loc="upper right", fontsize=8, frameon=False)
# histogram
ax2 = fig.add_subplot(gs[0, 1])
bins = np.arange(0, 31, 1)
for k, col, lab in (("comma", COMMA, "comma GNSS"), ("car", CAR, "car GPS")):
  x = np.clip(both[k], 0, 30)
  ax2.hist(
    x,
    bins=bins,
    weights=np.full(len(x), 100 / len(x)),
    histtype="step",
    lw=2,
    color=col,
    label=f"{lab}: median {np.median(both[k]):.1f} m, 95% {np.percentile(both[k], 95):.1f} m",
  )
ax2.set_xlabel("distance from road centreline (m; last bin = 30+)", fontsize=8, color=INK2)
ax2.set_ylabel("% of fixes", fontsize=8, color=INK2)
ax2.set_title(f"Same instants, both sources have a fix (n={len(both['car'])}, routes 64/65/72/75)", fontsize=9, color=INK, loc="left")
ax2.legend(fontsize=8, frameon=False)
# usable share
ax3 = fig.add_subplot(gs[1, 1])
yy = np.arange(len(usable))
ax3.barh(yy + 0.2, [u[2] * 100 for u in usable], height=0.38, color=CAR, label="car GPS")
ax3.barh(yy - 0.2, [u[1] * 100 for u in usable], height=0.38, color=COMMA, label="comma GNSS")
for i, u in enumerate(usable):
  ax3.text(u[2] * 100 + 1, i + 0.2, f"{u[2] * 100:.0f}%", va="center", fontsize=7, color=INK2)
  ax3.text(u[1] * 100 + 1, i - 0.2, f"{u[1] * 100:.0f}%", va="center", fontsize=7, color=INK2)
ax3.set_yticks(yy, [u[0] for u in usable], fontsize=8)
ax3.set_xlim(0, 112)
ax3.set_xlabel("% of driving time with a usable fix (accuracy <= 25 m)", fontsize=8, color=INK2)
ax3.invert_yaxis()
ax3.legend(fontsize=7, frameon=False, loc="lower left", bbox_to_anchor=(0, 1.0), ncol=2)
for axx in (ax2, ax3):
  for sp in ("top", "right"):
    axx.spines[sp].set_visible(False)
  axx.tick_params(labelsize=8, colors=INK2)
  axx.grid(axis="x" if axx is ax3 else "y", color="#e6e5e0", lw=0.6)
  axx.set_axisbelow(True)
fig.savefig(f"{DIR}/compare.png", bbox_inches="tight", facecolor="white")
print("wrote", f"{DIR}/compare.png")
