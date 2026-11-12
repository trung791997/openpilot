package com.starpilot.aaroad;

// Display-only road snapping for the zoomed-out map (Peter 2026-11-08). GPS puts the car a few metres off its road, and
// beside a freeway's frontage road that is the difference between "on the frontage road" and "on the freeway"; the
// map draws the car and centres the camera so the car sits on the road's drawn surface instead. Nothing here is sent
// to the car: it moves only where the map is drawn.
//
// v2 (Peter 2026-11-08): a clamp to the road's edge, not a pull to its centreline. With hw the road's half width
// (one-way lines half of that) the car is moved toward the line only as far as it is outside hw - EDGE_MARGIN_M:
// a car already on the road does not move, one on a frontage road 7 m from a freeway carriageway goes to just
// inside that carriageway's edge, and a car in its lane on a two-way road stays where it is.
//
// Pure Java (no Android types) so it can be tested on the desktop (test/SnapTest.java).
//   choose(): at 10 Hz, picks the line -- the nearest within MAX_M whose direction matches the heading within
//   MAX_DEG (a one-way line only in its digitised direction, a two-way line either way), keeping the current line
//   unless another is more than SWITCH_M closer, and holding it below MIN_SPEED m/s.
//   step(): every frame, eases the offset toward the foot point (or back to zero) over about BLEND_S s.
// Positions are metres east (e) and north (n) in the scene's frame; headings compass radians (0 north, east = +).
// choose() and step() allocate nothing.
final class Snap {
  static final double MAX_M = 15, MAX_DEG = 30, SWITCH_M = 3, MIN_SPEED = 2, BLEND_S = 0.5, EDGE_MARGIN_M = 1;
  // first-order lag: 95% of the way in BLEND_S
  private static final double TAU = BLEND_S / 3;
  private static final double MAX_COS = Math.cos(Math.toRadians(MAX_DEG));

  // the chosen line (run index in the arrays given to choose), -1 for none; its distance (m), heading difference
  // (deg, 0 to 90 for a two-way line) and foot point, as of the last choose()
  int run = -1;
  double dist, diffDeg;
  float footE, footN;
  // the offset to add to the car's position now (eased), and the one it is easing toward
  float offE, offN;
  private float wantE, wantN;
  private long stepAt;
  private Object key;  // the scene the run index belongs to

  void reset() {
    run = -1;
    offE = offN = wantE = wantN = 0;
    stepAt = 0;
    key = null;
  }

  // Picks the line for the car at (e, n) heading h (rad) at spd m/s. scene identifies the arrays: a new one drops
  // the current line (its indices mean nothing), not the eased offset. one[r] is whether run r is one-way, its
  // points in the digitised direction. hw[r] is run r's drawn half width in metres (one-way already halved); 0 or less
  // means the run is not drawn (a hidden class) and is never snapped to. Returns whether a line is snapped to.
  boolean choose(Object scene, float[][] runs, float[][] box, boolean[] one, float[] hw, double e, double n, double h,
                 double spd) {
    if (scene != key) {
      key = scene;
      run = -1;
    }
    if (runs == null || box == null || one == null || hw == null) {
      run = -1;
      aim(e, n, null);
      return false;
    }
    if (spd < MIN_SPEED) {
      // no switching: only follow the held line, which the car may still be creeping along
      if (run >= 0 && run < runs.length) {
        if (hw[run] <= 0) run = -1;
        else if (!nearest(runs[run], e, n, Double.NaN, Double.NaN, false)) run = -1;
        if (run >= 0 && dist > 2 * MAX_M) run = -1;
      }
      aim(e, n, hw);
      return run >= 0;
    }
    double hx = Math.sin(h), hy = Math.cos(h);
    double curD = Double.MAX_VALUE, bestD = Double.MAX_VALUE;
    int best = -1;
    float curE = 0, curN = 0, bestE = 0, bestN = 0;
    double curDiff = 0, bestDiff = 0;
    for (int r = 0; r < runs.length; r++) {
      float[] b = box[r];
      if (hw[r] <= 0) continue;
      if (e < b[0] - MAX_M || e > b[2] + MAX_M || n < b[1] - MAX_M || n > b[3] + MAX_M) continue;
      if (!nearest(runs[r], e, n, hx, hy, one[r])) continue;
      if (r == run) {
        curD = dist;
        curE = footE;
        curN = footN;
        curDiff = diffDeg;
      }
      if (dist < bestD) {
        bestD = dist;
        best = r;
        bestE = footE;
        bestN = footN;
        bestDiff = diffDeg;
      }
    }
    int pick = best;
    if (run >= 0 && curD != Double.MAX_VALUE && !(bestD < curD - SWITCH_M)) pick = run;  // hysteresis
    run = pick;
    if (pick < 0) {
      dist = diffDeg = 0;
    } else if (pick == best) {
      dist = bestD;
      footE = bestE;
      footN = bestN;
      diffDeg = bestDiff;
    } else {
      dist = curD;
      footE = curE;
      footN = curN;
      diffDeg = curDiff;
    }
    aim(e, n, hw);
    return run >= 0;
  }

  // Nearest qualifying point of one run to (e, n): fills dist, footE, footN, diffDeg and returns true when a
  // segment within MAX_M has a direction matching (hx, hy), the heading's unit vector (NaN: any direction).
  private boolean nearest(float[] p, double e, double n, double hx, double hy, boolean oneWay) {
    boolean any = false;
    double bestD2 = MAX_M * MAX_M;
    boolean ignoreDir = Double.isNaN(hx);
    if (ignoreDir) bestD2 = Double.MAX_VALUE;
    for (int i = 0; i + 3 < p.length; i += 2) {
      double ax = p[i], ay = p[i + 1], dx = p[i + 2] - ax, dy = p[i + 3] - ay;
      double len2 = dx * dx + dy * dy;
      if (len2 <= 0) continue;
      double cosd = 1;
      if (!ignoreDir) {
        cosd = (dx * hx + dy * hy) / Math.sqrt(len2);
        if (oneWay ? cosd < MAX_COS : Math.abs(cosd) < MAX_COS) continue;
      }
      double t = ((e - ax) * dx + (n - ay) * dy) / len2;
      t = t < 0 ? 0 : t > 1 ? 1 : t;
      double fx = ax + t * dx, fy = ay + t * dy, ex = e - fx, ey = n - fy, d2 = ex * ex + ey * ey;
      if (d2 < bestD2) {
        bestD2 = d2;
        any = true;
        footE = (float) fx;
        footN = (float) fy;
        diffDeg = ignoreDir ? 0 : Math.toDegrees(Math.acos(Math.min(1, Math.abs(cosd))));
      }
    }
    if (any) dist = Math.sqrt(bestD2);
    return any;
  }

  // the offset from the unsnapped position (e, n) choose() was given: toward the foot point by how far the car is
  // outside the road's edge less the margin (0 when it is inside)
  private void aim(double e, double n, float[] hw) {
    if (run >= 0 && hw != null && dist > 0) {
      double allow = Math.max(0, hw[run] - EDGE_MARGIN_M), move = Math.max(0, dist - allow), f = move / dist;
      wantE = (float) ((footE - e) * f);
      wantN = (float) ((footN - n) * f);
    } else {
      wantE = wantN = 0;
    }
  }

  // Eases the offset toward the aim, dt from the last call (a first call, or a clock gone back, moves nothing).
  void step(long nowMs) {
    double dt = stepAt == 0 ? 0 : Math.max(0, (nowMs - stepAt) / 1000.0);
    stepAt = nowMs;
    float k = (float) (1 - Math.exp(-dt / TAU));
    offE += (wantE - offE) * k;
    offN += (wantN - offN) * k;
  }
}
