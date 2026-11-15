#!/usr/bin/env python3
"""SYNTHETIC pass along a real tile centreline (US-75 / N Central Expy, tile line 21 as printed by geom.py), at constant speed,
one fix per second. usage: synth_pass.py out.csv speed_mps [rev]   (rev = drive the line backwards, i.e. northbound)"""
import sys, math
la0, lo0 = 33.1168, -96.6703
PTS = [(130, 326), (-34, 2), (-203, -326), (-226, -354), (-275, -402), (-302, -448), (-313, -456)]  # (east, north) m from (la0, lo0)
out, v = sys.argv[1], float(sys.argv[2])
pts = PTS[::-1] if len(sys.argv) > 3 and sys.argv[3] == "rev" else PTS
seg = [math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:])]
def at(s):
    for i, l in enumerate(seg):
        if s <= l or i == len(seg) - 1:
            t = min(1.0, s / l); a, b = pts[i], pts[i + 1]
            return a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]), math.degrees(math.atan2(b[0] - a[0], b[1] - a[1])) % 360
        s -= l
with open(out, "w") as f:
    f.write("t_ms,lat,lon,bearing_deg,speed_mps\n")
    k = 0
    while k * v <= sum(seg):
        e, n, b = at(k * v)
        f.write("%d,%.7f,%.7f,%.1f,%.2f\n" % (k * 1000, la0 + n / 111320.0, lo0 + e / (111320.0 * math.cos(math.radians(la0))), b, v))
        k += 1
