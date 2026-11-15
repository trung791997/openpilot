package com.starpilot.aaroad;

// Desktop check of RoundaboutGeom: exit angle from degrees or modifier, ring positions, the way round either side, head.
//   javac -d /tmp/rg src/com/starpilot/aaroad/RoundaboutGeom.java test/RoundaboutGeomTest.java && java -cp /tmp/rg com.starpilot.aaroad.RoundaboutGeomTest
public class RoundaboutGeomTest {
  static int bad;

  static void check(boolean ok, String what) {
    System.out.println((ok ? "ok   " : "FAIL ") + what);
    if (!ok) bad++;
  }

  static boolean near(double a, double b) {
    return Math.abs(a - b) < 1e-3;
  }

  public static void main(String[] x) {
    check(RoundaboutGeom.exitDegrees(120, "left") == 120, "degrees win over the modifier");
    check(RoundaboutGeom.exitDegrees(0, "sharp right") == 45 && RoundaboutGeom.exitDegrees(0, "right") == 90
        && RoundaboutGeom.exitDegrees(0, "slight right") == 135 && RoundaboutGeom.exitDegrees(0, "straight") == 180
        && RoundaboutGeom.exitDegrees(0, "slight left") == 225 && RoundaboutGeom.exitDegrees(0, "left") == 270
        && RoundaboutGeom.exitDegrees(0, "sharp left") == 315 && RoundaboutGeom.exitDegrees(0, "uturn") == 360, "modifier table");
    check(RoundaboutGeom.exitDegrees(0, "slightLeft") == 225 && RoundaboutGeom.exitDegrees(0, "sharpRight") == 45, "camel-case modifiers");
    check(RoundaboutGeom.exitDegrees(0, null) == 180 && RoundaboutGeom.exitDegrees(-5, "wat") == 180 && RoundaboutGeom.exitDegrees(999, "") == 180,
        "missing or bad: straight on");

    float[] p = new float[2];
    RoundaboutGeom.at(10, 10, 5, 0, p);
    check(near(p[0], 10) && near(p[1], 15), "angle 0 is the bottom (entry)");
    RoundaboutGeom.at(10, 10, 5, 90, p);
    check(near(p[0], 15) && near(p[1], 10), "angle 90 is on the right");
    RoundaboutGeom.at(10, 10, 5, 180, p);
    check(near(p[0], 10) && near(p[1], 5), "angle 180 is the top");
    RoundaboutGeom.at(10, 10, 5, 270, p);
    check(near(p[0], 5) && near(p[1], 10), "angle 270 is on the left");

    check(RoundaboutGeom.sweep(90, false) == 90 && RoundaboutGeom.sweep(90, true) == 270, "sweep: counter-clockwise the angle, clockwise the rest");
    check(RoundaboutGeom.sweep(360, false) == 360 && RoundaboutGeom.sweep(0, false) == 360, "a U-turn goes all the way round");

    double[] ang = new double[2 + 360 / 6];
    int n = RoundaboutGeom.path(90, false, 6, ang);
    check(n == 16 && ang[0] == 0 && near(ang[n - 1], 90) && ang[1] > 0, "counter-clockwise path 0 to 90, rising");
    n = RoundaboutGeom.path(90, true, 6, ang);
    check(ang[0] == 0 && near(ang[n - 1], -270) && ang[1] < 0, "clockwise path 0 to -270 (the other way round)");
    boolean small = true;
    for (int i = 1; i < n; i++) small &= Math.abs(ang[i] - ang[i - 1]) <= 6.0001;
    check(small, "no step over 6 degrees");

    float[] h = new float[6];
    RoundaboutGeom.head(0, 0, 0, -3, 4, 2, h);
    check(near(h[0], 0) && near(h[1], 0) && near(h[2], 2) && near(h[3], 4) && near(h[4], -2) && near(h[5], 4), "head pointing up: tip, base corners 4 back, 2 each side");
    RoundaboutGeom.head(1, 1, 0, 0, 4, 2, h);
    check(near(h[0], 1) && h[3] > 1, "no direction: points up");

    System.out.println(bad == 0 ? "all ok" : bad + " FAILED");
    if (bad != 0) System.exit(1);
  }
}
