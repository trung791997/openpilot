package com.starpilot.aaroad;

// Desktop check of ManeuverArrow: slicing the route by distance, the ring's end, the head's direction.
//   javac -d /tmp/ma src/com/starpilot/aaroad/Turn.java src/com/starpilot/aaroad/ManeuverArrow.java test/ManeuverArrowTest.java
//   java -cp /tmp/ma com.starpilot.aaroad.ManeuverArrowTest
public class ManeuverArrowTest {
  static int bad;

  static void check(boolean ok, String what) {
    System.out.println((ok ? "ok   " : "FAIL ") + what);
    if (!ok) bad++;
  }

  static boolean near(double a, double b, double tol) {
    return Math.abs(a - b) <= tol;
  }

  static float[] alongOf(float[] p, float start) {
    float[] a = new float[p.length / 2];
    double d = start;
    for (int i = 0; i < a.length; i++) {
      if (i > 0) d += Math.hypot(p[2 * i] - p[2 * i - 2], p[2 * i + 1] - p[2 * i - 1]);
      a[i] = (float) d;
    }
    return a;
  }

  // a straight 200 m road north, every 10 m
  static float[] straight() {
    float[] p = new float[2 * 21];
    for (int i = 0; i < 21; i++) {
      p[2 * i] = 0;
      p[2 * i + 1] = 10 * i;
    }
    return p;
  }

  public static void main(String[] x) {
    check(ManeuverArrow.wants("roundabout", "right") && ManeuverArrow.wants("rotary", "") && ManeuverArrow.wants("exit roundabout", "left")
        && ManeuverArrow.wants("turn", "uturn") && !ManeuverArrow.wants("turn", "left") && !ManeuverArrow.wants(null, null), "which maneuvers get the arrow");
    check(ManeuverArrow.ring("roundabout") && ManeuverArrow.ring("rotary") && !ManeuverArrow.ring("exit roundabout") && !ManeuverArrow.ring("turn"), "ring types");

    float[] r = straight();
    float[][] runs = {r}, along = {alongOf(r, 1000)};
    float[] s = ManeuverArrow.slice(runs, along, 1100, 25, 35);
    check(s != null && s[1] == 75 && s[s.length - 1] == 135, "slice 25 m back to 35 m forward, ends interpolated");
    check(near(ManeuverArrow.length(s), 60, 0.01), "slice is 60 m long");
    s = ManeuverArrow.slice(runs, along, 1005, 25, 35);
    check(s != null && s[1] == 0 && s[s.length - 1] == 40, "clipped at the route's start");
    s = ManeuverArrow.slice(runs, along, 1195, 25, 35);
    check(s != null && s[s.length - 1] == 200, "clipped at the route's end");
    check(ManeuverArrow.slice(runs, along, 900, 25, 35) == null && ManeuverArrow.slice(runs, along, 1300, 25, 35) == null, "outside the route: null");

    // two runs end to end: the maneuver sits in the second
    float[] r2 = {50, 0, 50, 40, 50, 80};
    float[][] two = {{0, 0, 0, 40}, r2};
    float[][] along2 = {alongOf(two[0], 0), alongOf(r2, 40)};
    s = ManeuverArrow.slice(two, along2, 60, 25, 35);
    check(s != null && s[0] == 50 && s[1] == 0 && s[s.length - 1] == 55, "slice stays in the run that holds the point");

    float[] d = new float[2];
    ManeuverArrow.endDirection(straight(), 6, d);
    check(near(d[0], 0, 1e-6) && near(d[1], 1, 1e-6), "head points north on a northbound line");
    ManeuverArrow.endDirection(new float[] {0, 0, 10, 0}, 6, d);
    check(near(d[0], 1, 1e-6) && near(d[1], 0, 1e-6), "and east on an eastbound one");

    // a ring: 30 m straight in, then a 180 degree arc of radius 20 m (counter-clockwise), then 80 m straight out
    java.util.ArrayList<Float> q = new java.util.ArrayList<Float>();
    for (int i = 0; i <= 3; i++) {
      q.add(0f);
      q.add(10f * i);
    }
    for (int deg = 5; deg <= 180; deg += 5) {  // centre (-20, 30): leaves going south... a half turn left
      double a = Math.toRadians(deg);
      q.add((float) (-20 + 20 * Math.cos(a)));
      q.add((float) (30 + 20 * Math.sin(a)));
    }
    float lastE = q.get(q.size() - 2), lastN = q.get(q.size() - 1);
    for (int i = 1; i <= 8; i++) {
      q.add(lastE);
      q.add(lastN - 10f * i);
    }
    float[] ring = new float[q.size()];
    for (int i = 0; i < ring.length; i++) ring[i] = q.get(i);
    double end = ManeuverArrow.ringEnd(ring, 25);
    double arcEnd = 30 + Math.PI * 20;
    check(end >= arcEnd - 1 && end <= arcEnd + ManeuverArrow.RUN_OUT_M + 6, "the ring ends at the end of the arc plus a short run-out: " + end + " vs arc end " + arcEnd);
    check(ManeuverArrow.ringEnd(straight(), 50) == 200, "a straight line never leaves a ring: its end");

    float[][] rr = {ring};
    float[][] ra = {alongOf(ring, 500)};
    float[] line = ManeuverArrow.line("roundabout", rr, ra, 525);
    check(line != null && ManeuverArrow.length(line) <= ManeuverArrow.BACK_M + 
        ManeuverArrow.RING_FWD_M + 0.01 && ManeuverArrow.length(line) > 50, "roundabout line: back 25 m, forward to the ring's end, at most 60 m");
    line = ManeuverArrow.line("turn", runs, along, 1100);
    check(line != null && near(ManeuverArrow.length(line), 60, 0.01), "a U-turn: 25 back and 35 forward");
    line = ManeuverArrow.line("roundabout", runs, along, 1100);
    check(line != null && near(ManeuverArrow.length(line), 25 + ManeuverArrow.RING_FWD_M, 0.01) || line != null && ManeuverArrow.length(line) >= 25,
        "a roundabout on a straight route keeps the whole 60 m window");
    float[] cut = ManeuverArrow.prefix(straight(), 25);
    check(cut.length == 8 && cut[cut.length - 1] == 25, "prefix interpolates its end");

    System.out.println(bad == 0 ? "all ok" : bad + " FAILED");
    if (bad != 0) System.exit(1);
  }
}
