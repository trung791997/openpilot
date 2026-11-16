package com.starpilot.aaroad;

// The white arrow MapView draws on the route at a roundabout or U-turn (Peter 2026-11-15): the route from BACK_M
// before the maneuver point to FWD_M after it (for a roundabout up to where the route leaves the ring, at most
// RING_FWD_M), with an arrowhead at the end. Plain Java so it can be tested on the desktop; MapView draws it.
// Positions are metres east, north in the scene's frame, and distances are metres along the route as
// Scene.routeAlong has them (runs end to end), the same measure as Frame.cut, the car's distance along.
final class ManeuverArrow {
  static final double BACK_M = 25, FWD_M = 35, RING_FWD_M = 60, SHOW_M = 300;

  // Roundabout, rotary or the turn at a small roundabout or its exit: the route goes round a ring.
  static boolean ring(String type) {
    return "roundabout".equals(type) || "rotary".equals(type) || "roundabout turn".equals(type);
  }

  // The maneuvers that get the arrow: the roundabout family, and any U-turn.
  static boolean wants(String type, String modifier) {
    return Turn.isRoundabout(type) || Turn.isUturn(type, modifier);
  }

  // The route from `at` - back to `at` + fwd metres along it, in the run that holds `at` (a run's own points, so
  // never across a gap between runs), as e, n pairs; null when no run holds it or the piece is under 2 points.
  static float[] slice(float[][] runs, float[][] along, double at, double back, double fwd) {
    for (int r = 0; r < runs.length; r++) {
      float[] p = runs[r], a = along[r];
      int n = p.length / 2;
      if (n < 2 || a.length < n || at < a[0] || at > a[n - 1]) continue;
      double from = Math.max(a[0], at - back), to = Math.min(a[n - 1], at + fwd);
      float[] out = new float[2 * (n + 2)];
      int m = 0;
      for (int i = 0; i + 1 < n; i++) {
        double d0 = a[i], d1 = a[i + 1];
        if (d1 < from || d0 > to) continue;
        if (m == 0) {
          m = put(out, m, lerp(p, i, d0, d1, Math.max(from, d0)));
        }
        if (d1 <= to) m = put(out, m, new float[] {p[2 * i + 2], p[2 * i + 3]});
        else m = put(out, m, lerp(p, i, d0, d1, to));
      }
      if (m < 4) return null;
      float[] cut = new float[m];
      System.arraycopy(out, 0, cut, 0, m);
      return cut;
    }
    return null;
  }

  private static float[] lerp(float[] p, int i, double d0, double d1, double d) {
    double t = d1 > d0 ? (d - d0) / (d1 - d0) : 0;
    return new float[] {(float) (p[2 * i] + t * (p[2 * i + 2] - p[2 * i])), (float) (p[2 * i + 1] + t * (p[2 * i + 3] - p[2 * i + 1]))};
  }

  private static int put(float[] out, int m, float[] pt) {
    if (m >= 2 && out[m - 2] == pt[0] && out[m - 1] == pt[1]) return m;
    out[m] = pt[0];
    out[m + 1] = pt[1];
    return m + 2;
  }

  static double length(float[] p) {
    double l = 0;
    for (int i = 2; i + 1 < p.length; i += 2) l += Math.hypot(p[i] - p[i - 2], p[i + 1] - p[i - 1]);
    return l;
  }

  // The first `len` metres of the line, its end interpolated.
  static float[] prefix(float[] p, double len) {
    double l = 0;
    for (int i = 2; i + 1 < p.length; i += 2) {
      double d = Math.hypot(p[i] - p[i - 2], p[i + 1] - p[i - 1]);
      if (l + d >= len) {
        double t = d > 0 ? (len - l) / d : 0;
        float[] out = new float[i + 2];
        System.arraycopy(p, 0, out, 0, i);
        out[i] = (float) (p[i - 2] + t * (p[i] - p[i - 2]));
        out[i + 1] = (float) (p[i - 1] + t * (p[i + 1] - p[i - 1]));
        return out;
      }
      l += d;
    }
    return p;
  }

  // Where a roundabout's ring ends along the line `p`, which has the maneuver point `man` metres from its start: the
  // route turns while it is on the ring, so after at least RING_TURN degrees of turning, the first place from which
  // the next STRAIGHT_M metres turn less than STRAIGHT_DEG in all, plus a little run-out. At most the line's end.
  static final double RING_TURN = 25, STRAIGHT_M = 12, STRAIGHT_DEG = 3, RUN_OUT_M = 8;

  static double ringEnd(float[] p, double man) {
    int n = p.length / 2;
    double[] at = new double[n], turn = new double[n];  // metres from the start, the turn (degrees) at each vertex
    for (int i = 1; i < n; i++) at[i] = at[i - 1] + Math.hypot(p[2 * i] - p[2 * i - 2], p[2 * i + 1] - p[2 * i - 1]);
    for (int i = 1; i + 1 < n; i++) {
      double h0 = Math.atan2(p[2 * i] - p[2 * i - 2], p[2 * i + 1] - p[2 * i - 1]);
      double h1 = Math.atan2(p[2 * i + 2] - p[2 * i], p[2 * i + 3] - p[2 * i + 1]);
      double d = Math.toDegrees(h1 - h0);
      while (d > 180) d -= 360;
      while (d < -180) d += 360;
      turn[i] = Math.abs(d);
    }
    double turned = 0;
    for (int i = 1; i + 1 < n; i++) {
      if (at[i] < man) continue;
      turned += turn[i];
      if (turned < RING_TURN) continue;
      double ahead = 0;
      for (int j = i + 1; j + 1 < n && at[j] - at[i] <= STRAIGHT_M; j++) ahead += turn[j];
      if (ahead < STRAIGHT_DEG) return Math.min(at[n - 1], at[i] + RUN_OUT_M);
    }
    return at[n - 1];
  }

  // The unit direction of the line's last `back` metres (e, n), for the head; out[0], out[1].
  static void endDirection(float[] p, double back, float[] out) {
    int n = p.length / 2;
    float ex = p[2 * n - 2], en = p[2 * n - 1];
    double l = 0;
    float sx = p[0], sn = p[1];
    for (int i = n - 1; i > 0; i--) {
      l += Math.hypot(p[2 * i] - p[2 * i - 2], p[2 * i + 1] - p[2 * i - 1]);
      sx = p[2 * i - 2];
      sn = p[2 * i - 1];
      if (l >= back) break;
    }
    double d = Math.hypot(ex - sx, en - sn);
    out[0] = d > 0 ? (float) ((ex - sx) / d) : 0;
    out[1] = d > 0 ? (float) ((en - sn) / d) : 1;
  }

  // The arrow's line for a maneuver `at` metres along the route: BACK_M before to FWD_M after, a roundabout's to where
  // the route leaves the ring (at most RING_FWD_M). null when there is nothing to draw.
  static float[] line(String type, float[][] runs, float[][] along, double at) {
    boolean ring = ring(type);
    float[] p = slice(runs, along, at, BACK_M, ring ? RING_FWD_M : FWD_M);
    if (p == null || !ring) return p;
    double man = at - Math.max(along[runOf(runs, along, at)][0], at - BACK_M);  // the maneuver point's distance from the line's start
    return prefix(p, ringEnd(p, man));
  }

  private static int runOf(float[][] runs, float[][] along, double at) {
    for (int r = 0; r < runs.length; r++) {
      int n = runs[r].length / 2;
      if (n >= 2 && along[r].length >= n && at >= along[r][0] && at <= along[r][n - 1]) return r;
    }
    return 0;
  }

  private ManeuverArrow() {}
}
