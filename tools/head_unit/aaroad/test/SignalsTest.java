package com.starpilot.aaroad;

// Desktop check of Signals, which traffic signals the dash map shows and which is the next on the route.
//   javac -d /tmp/signals src/com/starpilot/aaroad/Signals.java test/SignalsTest.java
//   java -cp /tmp/signals com.starpilot.aaroad.SignalsTest
// Fails (exit 1) unless: the signals OSM maps at each corner of a divided intersection merge into one icon at its
// centre, two intersections a block apart stay two, a stop and a signal together are a signal, points far from the
// anchor and unknown kinds are left out; only icons within AHEAD_M and not more than BEHIND_M behind are chosen, at
// most MAX of them, the nearest, farthest first; and the next one on the route ahead is the larger one, with none
// off the route or behind the car.
public class SignalsTest {
  static int fails = 0;
  static final double MPT = 2445.98 / 4096 * 4096;  // metres per zoom-14 tile unit at the equator, near enough

  static void check(boolean ok, String what) {
    if (!ok) {
      fails++;
      System.out.println("FAIL " + what);
    }
  }

  // points in metres east, north of the anchor (0, 0) to the tile units cluster() takes
  static double[] pts(double... enk) {
    double[] o = new double[enk.length];
    for (int i = 0; i < enk.length; i += 3) {
      o[i] = enk[i] / MPT;
      o[i + 1] = -enk[i + 1] / MPT;
      o[i + 2] = enk[i + 2];
    }
    return o;
  }

  static float[] cluster(float[][] route, double... enk) {
    double[] p = pts(enk);
    float[][] along = new float[route.length][];
    double a = 0;
    for (int r = 0; r < route.length; r++) {
      along[r] = new float[route[r].length / 2];
      for (int j = 0; j < along[r].length; j++) {
        if (j > 0) a += Math.hypot(route[r][2 * j] - route[r][2 * j - 2], route[r][2 * j + 1] - route[r][2 * j - 1]);
        along[r][j] = (float) a;
      }
    }
    return Signals.cluster(p, p.length / 3, 0, 0, MPT, route, along);
  }

  public static void main(String[] args) {
    int S = Signals.SIGNAL, T = Signals.STOP;
    float[][] none = new float[0][];

    // a divided road crossing another: four signals 24 m apart, one icon at the middle
    float[] g = cluster(none, 88, 112, S, 112, 112, S, 88, 88, S, 112, 88, S);
    check(g.length == 4, "four corners are one icon, got " + g.length / 4);
    check(g.length == 4 && Math.abs(g[0] - 100) < 0.5 && Math.abs(g[1] - 100) < 0.5, "the icon at the centre");
    check(g.length == 4 && Float.isNaN(g[3]), "no route, no along");

    // a block apart (200 m) stays two; 31 m apart is two too
    g = cluster(none, 0, 0, S, 0, 200, S);
    check(g.length == 8, "a block apart is two icons");
    g = cluster(none, 0, 0, S, 0, 31, S);
    check(g.length == 8, "31 m apart is two icons");

    // a stop and a signal at one corner draw as a signal; a stop alone stays a stop
    g = cluster(none, 0, 0, T, 5, 5, S, 300, 0, T);
    check(g.length == 8 && g[2] == S && g[6] == T, "signal wins over stop");

    // far from the anchor and unknown kinds are dropped
    g = cluster(none, Signals.KEEP_M + 10, 0, S, 0, 0, 7, 0, 0, 0);
    check(g.length == 0, "far and unknown dropped, got " + g.length / 4);

    // a route north along e = 0, then east along n = 400: along at a signal on it, NaN off it
    float[][] route = {{0, -100, 0, 400, 300, 400}};
    g = cluster(route, 0, 100, S, 10, 300, S, 200, 410, S, 100, 100, S);
    check(g.length == 16, "four apart");
    check(Math.abs(g[3] - 200) < 0.5, "along at 100 m north is 200, got " + g[3]);
    check(Math.abs(g[7] - 400) < 0.5, "along 10 m beside the route is the nearest point's, got " + g[7]);
    check(Math.abs(g[11] - 700) < 0.5, "along after the turn, got " + g[11]);
    check(Float.isNaN(g[15]), "100 m off the route has no along");

    // choose: the car at (0, 0) heading north (ch 1, sh 0)
    Signals.Choice c = new Signals.Choice();
    float[] sig = {
        0, 100, S, 200,          // 0: 100 ahead, on the route
        0, 250, S, 350,          // 1: 250 ahead, on the route
        0, 320, S, 420,          // 2: beyond AHEAD_M
        0, -10, S, 90,           // 3: 10 behind: shown, never the next
        0, -40, S, 60,           // 4: 40 behind: not shown
        50, 20, S, Float.NaN,    // 5: off the route, nearest ahead
        0, 3, S, 103,            // 6: on the route but under NEXT_MIN_M ahead: shown, not the next
    };
    Signals.choose(sig, 0, 0, 1, 0, c);
    check(c.count == 5, "five chosen, got " + c.count);
    boolean has2 = false, has4 = false;
    for (int i = 0; i < c.count; i++) {
      has2 |= c.idx[i] == 2;
      has4 |= c.idx[i] == 4;
      if (i > 0) check(c.dist[i - 1] >= c.dist[i], "farthest first");
    }
    check(!has2, "beyond AHEAD_M left out");
    check(!has4, "more than BEHIND_M behind left out");
    check(c.next == 0, "the next is the first on the route ahead, got " + c.next);

    // heading south, the same points: those north are behind and out; the next is the earliest on the route of those
    // now ahead (the 40 m one at 60 m along before the 10 m one at 90 m)
    Signals.choose(sig, 0, 0, -1, 0, c);
    for (int i = 0; i < c.count; i++) check(c.idx[i] != 0 && c.idx[i] != 1, "behind heading south is out");
    check(c.next == 4, "heading south the next is the earliest along ahead, got " + c.next);

    // a city grid: twenty in reach, only MAX, the nearest
    float[] many = new float[4 * 20];
    for (int i = 0; i < 20; i++) {
      many[4 * i] = 0;
      many[4 * i + 1] = 10 + 14 * i;
      many[4 * i + 2] = S;
      many[4 * i + 3] = Float.NaN;
    }
    Signals.choose(many, 0, 0, 1, 0, c);
    check(c.count == Signals.MAX, "capped at MAX, got " + c.count);
    for (int i = 0; i < c.count; i++) check(c.idx[i] < Signals.MAX, "the nearest kept, got " + c.idx[i]);
    check(c.count > 0 && c.idx[c.count - 1] == 0, "the nearest last, drawn on top");
    check(c.next == -1, "no route, no next");

    if (fails > 0) {
      System.out.println(fails + " failed");
      System.exit(1);
    }
    System.out.println("SignalsTest ok");
  }
}
