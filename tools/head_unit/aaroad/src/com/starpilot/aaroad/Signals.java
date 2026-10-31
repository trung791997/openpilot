package com.starpilot.aaroad;

// Traffic signals on the dash map, as Google Maps marks a signalised intersection with a small traffic light (Peter
// 2026-10-31: "for intersection with stoplight or stop sign, can you draw it like this?"). The points come in the road
// tiles from format 4 (road_tiles.py: Mapbox Streets' traffic_signals points, OSM highway=traffic_signals). Mapbox
// has no stop or give-way points, so STOP is drawn when a tile carries one but no tile does today.
//
// Which ones show, and where; plain Java, no Android: test/SignalsTest.java runs it on the desktop. SignalIcon draws
// them, the same for the view and the GPU layer.
//
// OSM maps a signal at every node where the roads cross, so a divided road meeting another is four signals 20 to 40 m
// apart, and one with turn lanes more. They are merged into one icon within CLUSTER_M of the group's centre. Only the
// MAX nearest within AHEAD_M are drawn, none more than BEHIND_M behind the car, farthest first so a near one is on top.
// With a route, the nearest one on it ahead (within ROUTE_M of the line, more than NEXT_MIN_M ahead) is drawn larger:
// the next light the car will stop at.
final class Signals {
  static final int SIGNAL = 1, STOP = 2;  // road_tiles.SIGNAL, STOP
  static final float CLUSTER_M = 30, AHEAD_M = 300, BEHIND_M = 15, ROUTE_M = 25, NEXT_MIN_M = 5;
  static final int MAX = 6;
  // signals farther from the scene's anchor are left out: the scene is rebuilt every 600 m, so this is 300 m ahead of
  // the car at the farthest it gets from the anchor, with room
  static final double KEEP_M = 1500;

  // The scene's icons from the tiles' points: pts is x, y, kind in threes, x and y in zoom-14 tile units (Feed.world),
  // ax, ay the scene's anchor and mpt its metres per tile unit. route and along are the scene's route runs (e, n
  // pairs) and the metres along it at each point, both empty with no route. Returns e, n, kind, along in fours: e, n
  // in metres east and north of the anchor, kind SIGNAL when any merged point is one, along the metres along the
  // route at the point of it nearest the icon, or NaN when the icon is not within ROUTE_M of the route.
  static float[] cluster(double[] pts, int count, double ax, double ay, double mpt, float[][] route, float[][] along) {
    float[] e = new float[count], n = new float[count];
    int[] kind = new int[count], members = new int[count];
    int groups = 0;
    for (int i = 0; i < count; i++) {
      double pe = (pts[3 * i] - ax) * mpt, pn = -(pts[3 * i + 1] - ay) * mpt;
      if (Math.abs(pe) > KEEP_M || Math.abs(pn) > KEEP_M) continue;
      int k = (int) pts[3 * i + 2];
      if (k != SIGNAL && k != STOP) continue;
      int best = -1;
      double bestD = CLUSTER_M;
      for (int g = 0; g < groups; g++) {
        double d = Math.hypot(e[g] - pe, n[g] - pn);
        if (d <= bestD) {
          bestD = d;
          best = g;
        }
      }
      if (best < 0) {
        e[groups] = (float) pe;
        n[groups] = (float) pn;
        kind[groups] = k;
        members[groups] = 1;
        groups++;
      } else {
        int m = ++members[best];
        e[best] += (float) ((pe - e[best]) / m);  // the group's centre
        n[best] += (float) ((pn - n[best]) / m);
        if (k == SIGNAL) kind[best] = SIGNAL;
      }
    }
    float[] out = new float[groups * 4];
    for (int g = 0; g < groups; g++) {
      out[4 * g] = e[g];
      out[4 * g + 1] = n[g];
      out[4 * g + 2] = kind[g];
      out[4 * g + 3] = onRoute(e[g], n[g], route, along);
    }
    return out;
  }

  // Metres along the route at its point nearest (e, n), or NaN when that is farther than ROUTE_M.
  static float onRoute(float e, float n, float[][] route, float[][] along) {
    double best = ROUTE_M;
    float at = Float.NaN;
    for (int r = 0; r < route.length; r++) {
      float[] p = route[r], al = r < along.length ? along[r] : null;
      if (al == null) continue;
      for (int i = 0; i + 3 < p.length; i += 2) {
        double se = p[i + 2] - p[i], sn = p[i + 3] - p[i + 1], len2 = se * se + sn * sn;
        double t = len2 <= 0 ? 0 : Math.max(0, Math.min(1, ((e - p[i]) * se + (n - p[i + 1]) * sn) / len2));
        double d = Math.hypot(p[i] + t * se - e, p[i + 1] + t * sn - n);
        if (d <= best) {
          best = d;
          at = (float) (al[i / 2] + t * (al[i / 2 + 1] - al[i / 2]));
        }
      }
    }
    return at;
  }

  // One frame's choice: idx the chosen icons (indices into the fours) farthest first, next the one drawn larger or -1.
  static final class Choice {
    final int[] idx = new int[MAX];
    final float[] dist = new float[MAX];
    int count, next = -1;
  }

  // The icons to draw with the car at (ce, cn) heading ch = cos h, sh = sin h (h clockwise from north), into c.
  // Allocates nothing: the view and the GPU layer each call it every frame.
  static void choose(float[] sig, float ce, float cn, float ch, float sh, Choice c) {
    c.count = 0;
    c.next = -1;
    float nextAt = Float.MAX_VALUE;
    for (int g = 0; g + 3 < sig.length; g += 4) {
      float de = sig[g] - ce, dn = sig[g + 1] - cn;
      float fwd = de * sh + dn * ch;
      float d = (float) Math.hypot(de, dn);
      if (fwd < -BEHIND_M || d > AHEAD_M) continue;
      // keep the MAX nearest, nearest at the end, by insertion
      int at = c.count;
      if (at == MAX) {
        if (d >= c.dist[0]) continue;
        System.arraycopy(c.idx, 1, c.idx, 0, MAX - 1);
        System.arraycopy(c.dist, 1, c.dist, 0, MAX - 1);
        at = MAX - 1;
      } else {
        c.count++;
      }
      while (at > 0 && c.dist[at - 1] < d) {
        c.idx[at] = c.idx[at - 1];
        c.dist[at] = c.dist[at - 1];
        at--;
      }
      c.idx[at] = g / 4;
      c.dist[at] = d;
    }
    for (int i = 0; i < c.count; i++) {
      int g = c.idx[i] * 4;
      float along = sig[g + 3];
      if (Float.isNaN(along)) continue;
      float fwd = (sig[g] - ce) * sh + (sig[g + 1] - cn) * ch;
      if (fwd > NEXT_MIN_M && along < nextAt) {
        nextAt = along;
        c.next = c.idx[i];
      }
    }
  }
}
