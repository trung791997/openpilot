package com.starpilot.aaroad;

// Which roads the scene keeps with no route, and when the car has outrun them (Peter 2026-10-31: "sometimes the map
// doesn't load fully, like I have to drive up to a certain point then it can load the upcoming portion").
//
// The scene used to keep only the roads within 700 m of its anchor, a square, and was rebuilt only once the car was
// 600 m from it. At 70 mph the free-drive view reaches about 790 m ahead and 1200 m to the side at its far edge, so
// even right after a rebuild the top of the screen was bare, and just before the next one only about 100 m ahead of
// the car had roads. The comma had the tiles all along; the head unit dropped them.
//
// Now the scene keeps the roads in a rectangle turned to the car's heading when it was built: the view box at that
// speed's zoom (with HEADROOM for speeding up), SIDE and BACK metres more around it, and AHEAD_S seconds of driving
// (at least AHEAD_MIN m) more ahead. Each tick checks the view box now and LEAD_S seconds on, at the zoom the view will
// have, and the view turned TURN either way; once a corner of any falls outside the rectangle the scene is rebuilt,
// with the car still well inside.
//
// Pure Java so test/FreeViewTest.java runs it on a desktop without Android.
final class FreeView {
  // The view box in display metres in the car's frame (x right, z ahead), a little past the screen. The free-drive
  // view, the car at the meter's centre, sees x -158..158 at the bottom edge (z -19) and -284..284 at the top (z 177);
  // with a route the car sits right of centre and the view sees -359..209 at the top. 290 covers both: the box was
  // 240 on the right before, which left the top right corner of the free-drive map bare.
  static final float X0 = -380, X1 = 290, Z0 = -45, Z1 = 185;
  static final double R0 = 70.03;  // MapView's: display metres from the car to the range's far edge
  // with no route the view shows about 12 s of road, 120 to 300 m (MapView.zoomTarget)
  static final double RNG_MIN = 120, RNG_MAX = 300, RNG_S = 12;
  static final double HEADROOM = 1.25, SIDE = 250, BACK = 250, AHEAD_S = 30, AHEAD_MIN = 600, LEAD_S = 5;
  // and the view turned TURN either way where the car is: a corner or a ramp swings the far edge of the view sideways
  // faster than the car moves, and the new scene takes a moment to build
  static final double TURN = Math.toRadians(30);
  // StarPilot Auto's map hides service roads and tracks once it is zoomed out past 2.2 m a pixel (about 45 mph); the
  // dash does the same: past MINOR_HIDE m of range (2.2 m/px at the car, PX_DM 1.7156 px per display metre) they
  // go, and they come back under MINOR_SHOW, so a speed near the line does not flip them.
  static final double MINOR_HIDE = 264, MINOR_SHOW = 240;
  // A road's points closer than SIMPLIFY m to the line through its neighbours are dropped (Douglas-Peucker): under
  // 0.6 px at the closest free-drive zoom, and well under at highway zoom. The tile keeps a point every few metres on a
  // curve; on the Tegra each point is a quad to cull, pack and draw every frame.
  static final double SIMPLIFY = 1.0;

  private FreeView() {}

  static double range(double spd) {
    return Math.max(RNG_MIN, Math.min(RNG_MAX, spd * RNG_S));
  }

  // whether service roads show at range rng, given whether they showed
  static boolean minor(double rng, boolean shown) {
    return shown ? rng <= MINOR_HIDE : rng < MINOR_SHOW;
  }

  // The rectangle the scene keeps for a car at speed spd whose view is at zoom kView (display to world metres, the
  // zoom smooths towards range(spd) / R0): {x0, x1, z0, z1} in metres in the frame of the build's heading, from the
  // anchor. It holds the view box turned up to TURN either way, SIDE and BACK metres around that, and the drive ahead.
  static float[] area(double spd, double kView) {
    double k = Math.max(kView, Math.min(RNG_MAX, range(spd) * HEADROOM) / R0), ahead = Math.max(AHEAD_MIN, spd * AHEAD_S);
    double x0 = 0, x1 = 0, z0 = 0, z1 = 0;
    for (int turn = -1; turn <= 1; turn++) {
      double c = Math.cos(turn * TURN), s = Math.sin(turn * TURN);
      for (int corner = 0; corner < 4; corner++) {
        double x = (corner < 2 ? X0 : X1) * k, z = (corner % 2 == 0 ? Z0 : Z1) * k;
        // the car's frame turned by turn TURN, in the build's frame (as in covered)
        double bx = x * c - z * s, bz = x * s + z * c;
        x0 = Math.min(x0, bx);
        x1 = Math.max(x1, bx);
        z0 = Math.min(z0, bz);
        z1 = Math.max(z1, bz);
      }
    }
    return new float[] {(float) (x0 - SIDE), (float) (x1 + SIDE), (float) (z0 - BACK), (float) (z1 + ahead)};
  }

  // Whether a box (min e, min n, max e, max n, metres from the anchor) may reach into area a built at heading h
  // (cos ch, sin sh): its corners turned into the build's frame, and their box tested.
  static boolean touches(float[] b, float[] a, double ch, double sh) {
    double x0 = Double.MAX_VALUE, x1 = -Double.MAX_VALUE, z0 = Double.MAX_VALUE, z1 = -Double.MAX_VALUE;
    for (int c = 0; c < 4; c++) {
      double e = b[c == 0 || c == 3 ? 0 : 2], n = b[c < 2 ? 1 : 3];
      double x = e * ch - n * sh, z = e * sh + n * ch;
      x0 = Math.min(x0, x);
      x1 = Math.max(x1, x);
      z0 = Math.min(z0, z);
      z1 = Math.max(z1, z);
    }
    return !(x1 < a[0] || x0 > a[1] || z1 < a[2] || z0 > a[3]);
  }

  // Whether the view of a car at (e, n) from the anchor, heading h, at speed spd and zoom k (display to world metres)
  // stays inside area a built at heading bh: now, LEAD_S seconds on, and turned TURN either way.
  static boolean covered(float[] a, double bh, double e, double n, double h, double spd, double k) {
    double bc = Math.cos(bh), bs = Math.sin(bh);
    for (int step = 0; step < 4; step++) {
      double sh = h + (step == 2 ? TURN : step == 3 ? -TURN : 0), c = Math.cos(sh), s = Math.sin(sh);
      double ce = e + (step == 1 ? s * spd * LEAD_S : 0), cn = n + (step == 1 ? c * spd * LEAD_S : 0);
      for (int corner = 0; corner < 4; corner++) {
        double x = (corner < 2 ? X0 : X1) * k, z = (corner % 2 == 0 ? Z0 : Z1) * k;
        // the car's frame to world metres (MapView: x = de ch - dn sh, z = de sh + dn ch), then to the build's frame
        double we = ce + x * c + z * s, wn = cn - x * s + z * c;
        double bx = we * bc - wn * bs, bz = we * bs + wn * bc;
        if (bx < a[0] || bx > a[1] || bz < a[2] || bz > a[3]) return false;
      }
    }
    return true;
  }

  // Douglas-Peucker on a run of x, y pairs (n points): the points kept, in order, ends always kept.
  static float[] simplify(float[] p, int n, double tol) {
    if (n <= 2) return java.util.Arrays.copyOf(p, 2 * n);
    boolean[] keep = new boolean[n];
    keep[0] = keep[n - 1] = true;
    int[] stack = new int[2 * n];
    int top = 0, kept = 2;
    stack[top++] = 0;
    stack[top++] = n - 1;
    double tol2 = tol * tol;
    while (top > 0) {
      int j = stack[--top], i = stack[--top];
      double ax = p[2 * i], ay = p[2 * i + 1], dx = p[2 * j] - ax, dy = p[2 * j + 1] - ay, len2 = dx * dx + dy * dy;
      double worst = tol2;
      int at = -1;
      for (int m = i + 1; m < j; m++) {
        double px = p[2 * m] - ax, py = p[2 * m + 1] - ay, d2;
        if (len2 == 0) {
          d2 = px * px + py * py;
        } else {
          double t = Math.max(0, Math.min(1, (px * dx + py * dy) / len2)), qx = px - t * dx, qy = py - t * dy;
          d2 = qx * qx + qy * qy;
        }
        if (d2 > worst) {
          worst = d2;
          at = m;
        }
      }
      if (at < 0) continue;
      keep[at] = true;
      kept++;
      stack[top++] = i;
      stack[top++] = at;
      stack[top++] = at;
      stack[top++] = j;
    }
    float[] out = new float[2 * kept];
    for (int m = 0, o = 0; m < n; m++) {
      if (!keep[m]) continue;
      out[o++] = p[2 * m];
      out[o++] = p[2 * m + 1];
    }
    return out;
  }
}
