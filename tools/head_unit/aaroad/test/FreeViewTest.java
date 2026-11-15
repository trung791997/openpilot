package com.starpilot.aaroad;

import java.util.Random;

// Desktop check of FreeView, the free-drive scene's road rectangle and when it is rebuilt.
//   javac -d /tmp/freeview src/com/starpilot/aaroad/FreeView.java test/FreeViewTest.java
//   java -cp /tmp/freeview com.starpilot.aaroad.FreeViewTest
// It drives MapView's rule (rebuild once the view now or LEAD_S s on leaves the rectangle, or after 600 m) with the
// build landing BUILD_S late at the pose it was asked at, and fails (exit 1) if the view's box is ever outside the
// roads kept: 70 mph straight, 70 mph through curves and a ramp, speeding up from a stop to 80 mph, and a city drive
// with right-angle turns. Then Douglas-Peucker: the ends kept, nothing dropped farther than the tolerance.
public class FreeViewTest {
  static final double MOVE_R = 600, HOLD_SPEED = 1.0, TICK = 0.05, BUILD_S = 1.0, ZOOM_TAU = 1.5;
  static int fails = 0;

  interface Drive {
    // speed (m/s) and turn rate (rad/s) at time t
    double[] at(double t);
  }

  static void drive(String name, double secs, Drive d) {
    double e = 0, n = 0, h = 0, k = FreeView.RNG_MIN / FreeView.R0;
    double[] v0 = d.at(0);
    k = FreeView.range(v0[0]) / FreeView.R0;
    // the scene: its anchor, rectangle and heading
    double ae = 0, an = 0, bh = 0;
    float[] area = FreeView.area(v0[0], k);
    double pendingAt = -1, pe = 0, pn = 0, ph = 0, pspd = 0, pk = 0;
    int builds = 1, bad = 0;
    double worst = Double.MAX_VALUE, metres = 0;
    for (double t = 0; t < secs; t += TICK) {
      double[] v = d.at(t);
      double spd = v[0];
      h += v[1] * TICK;
      e += Math.sin(h) * spd * TICK;
      n += Math.cos(h) * spd * TICK;
      metres += spd * TICK;
      double target = FreeView.range(spd) / FreeView.R0;
      k += (target - k) * Math.min(1, TICK / ZOOM_TAU);
      double kView = Math.max(k, target);
      if (pendingAt >= 0 && t >= pendingAt) {
        ae = pe;
        an = pn;
        bh = ph;
        area = FreeView.area(pspd, pk);
        pendingAt = -1;
        builds++;
      }
      // the view as drawn this tick must be inside the roads kept
      double m = margin(area, bh, e - ae, n - an, h, k);
      if (m < 0) bad++;
      worst = Math.min(worst, m);
      if (pendingAt < 0) {
        boolean need = Math.hypot(e - ae, n - an) > MOVE_R
            || spd >= HOLD_SPEED && !FreeView.covered(area, bh, e - ae, n - an, h, spd, kView);
        if (need) {
          pendingAt = t + BUILD_S;
          pe = e;
          pn = n;
          ph = h;
          pspd = spd;
          pk = kView;
        }
      }
    }
    System.out.printf("%-14s %6.0f m  %3d builds (one per %4.0f m)  least margin %5.0f m  ticks uncovered %d%n", name,
        metres, builds, metres / builds, worst, bad);
    if (bad > 0) fails++;
  }

  // metres from the view's box (now) to the rectangle's nearest edge, negative when outside
  static double margin(float[] a, double bh, double e, double n, double h, double k) {
    double bc = Math.cos(bh), bs = Math.sin(bh), c = Math.cos(h), s = Math.sin(h), m = Double.MAX_VALUE;
    for (int corner = 0; corner < 4; corner++) {
      double x = (corner < 2 ? FreeView.X0 : FreeView.X1) * k, z = (corner % 2 == 0 ? FreeView.Z0 : FreeView.Z1) * k;
      double we = e + x * c + z * s, wn = n - x * s + z * c;
      double bx = we * bc - wn * bs, bz = we * bs + wn * bc;
      m = Math.min(m, Math.min(Math.min(bx - a[0], a[1] - bx), Math.min(bz - a[2], a[3] - bz)));
    }
    return m;
  }

  static void check(boolean ok, String what) {
    if (!ok) {
      System.out.println("FAIL " + what);
      fails++;
    }
  }

  public static void main(String[] args) {
    final double mph70 = 70 * 0.44704;
    drive("70mph straight", 600, t -> new double[] {mph70, 0});
    // sweeping curves of 600 m radius each way, and a cloverleaf ramp: 270 degrees at 15 m/s
    drive("70mph curves", 600, t -> new double[] {mph70, Math.sin(t / 20) * mph70 / 600});
    drive("ramp", 120, t -> t < 30 ? new double[] {mph70, 0} : t < 30 + 4.71 * 60 / 15
        ? new double[] {15, 15 / 60.0} : new double[] {mph70, 0});
    drive("0 to 80mph", 300, t -> new double[] {Math.min(80 * 0.44704, t * 2.5), 0});
    // city: 13 m/s, a right-angle turn every 400 m, stopping at some
    drive("city turns", 900, t -> {
      double ph = t % 40;
      if (ph < 3) return new double[] {Math.max(0, ph * 4), 0};
      if (ph < 33) return new double[] {13, 0};
      return new double[] {8, ((int) (t / 40) % 3 == 0 ? -1 : 1) * Math.PI / 2 / 7};
    });

    // a heading that wanders at speed, 30 runs
    Random rnd = new Random(3);
    for (int run = 0; run < 30; run++) {
      final double[] turns = new double[200];
      for (int i = 0; i < turns.length; i++) turns[i] = rnd.nextGaussian() * 0.04;
      final double spd = 15 + rnd.nextDouble() * 22;
      drive("random " + run, 300, t -> new double[] {spd, turns[(int) (t / 1.5) % turns.length]});
    }

    // service roads go above MINOR_HIDE m of range and come back under MINOR_SHOW, with no flicker between
    check(FreeView.minor(FreeView.range(15), true), "service roads at 15 m/s");
    check(!FreeView.minor(FreeView.range(mph70), true), "no service roads at 70 mph");
    check(FreeView.minor(250, true) && !FreeView.minor(250, false), "service road hysteresis");

    // Douglas-Peucker: ends kept, every dropped point within the tolerance of the line kept, a straight run to 2
    float[] line = new float[200];
    for (int i = 0; i < 100; i++) {
      line[2 * i] = i * 3;
      line[2 * i + 1] = 0.3f * (float) Math.sin(i);
    }
    check(FreeView.simplify(line, 100, 1.0).length == 4, "straight run to its ends");
    for (int run = 0; run < 200; run++) {
      int n = 2 + rnd.nextInt(80);
      float[] p = new float[2 * n];
      double x = 0, y = 0, a = rnd.nextDouble() * 6;
      for (int i = 0; i < n; i++) {
        a += rnd.nextGaussian() * 0.3;
        x += Math.cos(a) * 5;
        y += Math.sin(a) * 5;
        p[2 * i] = (float) x;
        p[2 * i + 1] = (float) y;
      }
      float[] q = FreeView.simplify(p, n, 1.0);
      check(q[0] == p[0] && q[1] == p[1] && q[q.length - 2] == p[2 * n - 2] && q[q.length - 1] == p[2 * n - 1],
          "ends kept, run " + run);
      for (int i = 0; i < n; i++) {
        double best = Double.MAX_VALUE;
        for (int j = 0; j + 3 < q.length; j += 2) best = Math.min(best, seg(p[2 * i], p[2 * i + 1], q, j));
        if (q.length == 2) best = Math.hypot(p[2 * i] - q[0], p[2 * i + 1] - q[1]);
        check(best <= 1.0 + 1e-4, "point " + i + " within 1 m, run " + run);
      }
    }
    System.out.println(fails == 0 ? "ok" : fails + " failed");
    if (fails > 0) System.exit(1);
  }

  static double seg(double px, double py, float[] q, int j) {
    double ax = q[j], ay = q[j + 1], dx = q[j + 2] - ax, dy = q[j + 3] - ay, l2 = dx * dx + dy * dy;
    double t = l2 == 0 ? 0 : Math.max(0, Math.min(1, ((px - ax) * dx + (py - ay) * dy) / l2));
    return Math.hypot(px - ax - t * dx, py - ay - t * dy);
  }
}
