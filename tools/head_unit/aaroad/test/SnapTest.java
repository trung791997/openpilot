package com.starpilot.aaroad;

// Desktop check of Snap: the frontage road beside a freeway, wrong-way and far lines, hysteresis, a crawl, the blend.
//   javac -d /tmp/snaptest src/com/starpilot/aaroad/Snap.java test/SnapTest.java
//   java -cp /tmp/snaptest com.starpilot.aaroad.SnapTest
// Exits 1 on the first failed check. Frame: x east, y north; the car heads north (compass 0) unless a test says so.
public class SnapTest {
  static int fails;

  static void check(String what, boolean ok) {
    System.out.println((ok ? "ok   " : "FAIL ") + what);
    if (!ok) fails++;
  }

  static float[][] box(float[][] runs) {
    float[][] b = new float[runs.length][];
    for (int r = 0; r < runs.length; r++) {
      float[] p = runs[r];
      float x0 = 1e9f, y0 = 1e9f, x1 = -1e9f, y1 = -1e9f;
      for (int i = 0; i + 1 < p.length; i += 2) {
        x0 = Math.min(x0, p[i]); x1 = Math.max(x1, p[i]);
        y0 = Math.min(y0, p[i + 1]); y1 = Math.max(y1, p[i + 1]);
      }
      b[r] = new float[] {x0, y0, x1, y1};
    }
    return b;
  }

  // a straight north-south line at x, running up (north) or down (south), 400 m long, in 50 m segments
  static float[] vline(float x, boolean up) {
    float[] p = new float[18];
    for (int i = 0; i < 9; i++) {
      float y = -200 + 50 * (up ? i : 8 - i);
      p[2 * i] = x;
      p[2 * i + 1] = y;
    }
    return p;
  }

  // every run drawn with half width 4.5 m (a one-way major street, ROAD_M[1] 36 / ROAD_SCALE 2 / 2 * 0.5)
  static float[] hw(float[][] runs) {
    float[] h = new float[runs.length];
    java.util.Arrays.fill(h, 4.5f);
    return h;
  }

  // the offset the snap settles on after choose(): a long step so the lag has run its course
  static double[] settle(Snap s) {
    s.step(1000);
    s.step(1000000);
    return new double[] {s.offE, s.offN};
  }

  public static void main(String[] args) {
    // the car at the origin heading north. Frontage road 7 m east (one-way north), freeway carriageways at 21 m
    // (north) and 39 m (south) east.
    float[][] runs = {vline(7, true), vline(21, true), vline(39, false)};
    boolean[] one = {true, true, true};
    float[][] box = box(runs);
    Snap s = new Snap();
    boolean has = s.choose(runs, runs, box, one, hw(runs), 0, 0, 0, 25);
    check("frontage road chosen over the freeway carriageways", has && s.run == 0);
    check("perpendicular distance 7 m", Math.abs(s.dist - 7) < 1e-3);
    check("foot point on the frontage road", Math.abs(s.footE - 7) < 1e-3 && Math.abs(s.footN) < 1e-3);
    check("heading difference 0", s.diffDeg < 1e-3);

    // 10 degrees off still matches, 40 does not
    check("10 deg off matches", s.choose(runs, runs, box, one, hw(runs), 0, 0, Math.toRadians(10), 25) && s.run == 0);
    Snap t = new Snap();
    check("40 deg off: no snap", !t.choose(runs, runs, box, one, hw(runs), 0, 0, Math.toRadians(40), 25) && t.run == -1);

    // opposite-direction one-way rejected (only the south-bound carriageway at 39 m within reach? it is 39 m: out)
    float[][] r2 = {vline(7, false)};
    Snap o = new Snap();
    check("opposite-direction one-way rejected", !o.choose(r2, r2, box(r2), new boolean[] {true}, hw(r2), 0, 0, 0, 25));
    check("same line heading south is accepted", o.choose(r2, r2, box(r2), new boolean[] {true}, hw(r2), 0, 0, Math.PI, 25) && o.run == 0);
    // a two-way line takes either direction
    Snap tw = new Snap();
    check("two-way line, heading with its digitised direction", tw.choose(r2, r2, box(r2), new boolean[] {false}, hw(r2), 0, 0, Math.PI, 25));
    check("two-way line, heading against it", tw.choose(r2, r2, box(r2), new boolean[] {false}, hw(r2), 0, 0, 0, 25) && tw.diffDeg < 1e-3);

    // more than 15 m away: no snap; just inside: snap
    float[][] far = {vline(15.5f, true)};
    check("15.5 m: no snap", !new Snap().choose(far, far, box(far), new boolean[] {true}, hw(far), 0, 0, 0, 25));
    float[][] near = {vline(14.5f, true)};
    check("14.5 m: snap", new Snap().choose(near, near, box(near), new boolean[] {true}, hw(near), 0, 0, 0, 25));

    // hysteresis: on A (x=7), B (x=-1) is 6 m closer-ish. Keep A unless B is more than 3 m closer.
    float[][] two = {vline(7, true), vline(4.5f, true)};  // B is 2.5 m closer: stay
    Snap h = new Snap();
    h.choose(two, two, box(two), new boolean[] {true, true}, hw(two), 0, 0, 0, 25);
    check("first choice is the nearest (B)", h.run == 1);
    // car moves so that A is now nearer by 2 m: stay on B
    h.choose(two, two, box(two), new boolean[] {true, true}, hw(two), 3.5, 0, 0, 25);  // A 3.5 m, B 1 m
    check("stays on B", h.run == 1);
    h.choose(two, two, box(two), new boolean[] {true, true}, hw(two), 8.5, 0, 0, 25);   // A 1.5 m, B 4 m: A 2.5 closer, stay
    check("another line 2.5 m closer: kept", h.run == 1);
    h.choose(two, two, box(two), new boolean[] {true, true}, hw(two), 9.5, 0, 0, 25);   // A 2.5 m, B 5 m ... 2.5 closer
    check("another line 2.5 m closer (again): kept", h.run == 1);
    h.choose(two, two, box(two), new boolean[] {true, true}, hw(two), 11.0, 0, 0, 25);  // A 4 m, B 6.5 m ... 2.5
    check("still kept", h.run == 1);
    h.choose(two, two, box(two), new boolean[] {true, true}, hw(two), 12.5, 0, 0, 25);  // A 5.5 m, B 8 m; both within 15 m
    check("kept at 2.5 m difference", h.run == 1);
    // now make A decisively closer: a third line far from B
    float[][] three = {vline(7, true), vline(-5, true)};
    Snap h2 = new Snap();
    h2.choose(three, three, box(three), new boolean[] {true, true}, hw(three), 0, 0, 0, 25);
    check("starts on A (7 m) against B (5 m, 2 m closer than A means B nearer)", h2.run == 1);
    h2.choose(three, three, box(three), new boolean[] {true, true}, hw(three), 6, 0, 0, 25);  // A 1 m, B 11 m: A 10 m closer
    check("another line more than 3 m closer: switched", h2.run == 0);
    // drop when none qualifies
    check("dropped when none qualifies", !h2.choose(three, three, box(three), new boolean[] {true, true}, hw(three), 100, 0, 0, 25) && h2.run == -1);

    // low speed: the choice is held, not switched, even when another line is much closer
    Snap l = new Snap();
    l.choose(three, three, box(three), new boolean[] {true, true}, hw(three), -4, 0, 0, 25);  // B 1 m
    check("fast: B", l.run == 1);
    l.choose(three, three, box(three), new boolean[] {true, true}, hw(three), 6.5, 0, 0, 1.0);  // slow: A would be 0.5 m
    check("below 2 m/s: no switching", l.run == 1);
    // and with the heading no longer matching at a stop
    l.choose(three, three, box(three), new boolean[] {true, true}, hw(three), -4, 0, Math.PI, 0.2);
    check("below 2 m/s: held whatever the heading", l.run == 1);
    // and nothing is picked up while slow
    Snap n0 = new Snap();
    check("below 2 m/s with nothing held: no snap", !n0.choose(three, three, box(three), new boolean[] {true, true}, hw(three), 0, 0, 0, 1.0));

    // a new scene drops the held line
    Snap sc = new Snap();
    sc.choose(runs, runs, box, one, hw(runs), 0, 0, 0, 25);
    sc.choose(three, three, box(three), new boolean[] {true, true}, hw(three), 6.5, 0, 0, 1.0);
    check("a new scene while slow: held line dropped", sc.run == -1);

    // the blend: starts at 0, 95% in 0.5 s, no step bigger than the foot; back out to 0
    Snap b = new Snap();
    long t0 = 1000;
    b.step(t0);
    b.choose(runs, runs, box, one, hw(runs), 0, 0, 0, 25);
    double prev = 0, maxStep = 0;
    for (int i = 1; i <= 15; i++) {  // 30 fps for 0.5 s
      b.step(t0 + i * 1000 / 30);
      maxStep = Math.max(maxStep, b.offE - prev);
      prev = b.offE;
    }
    check("offset starts eased, not jumped (first frame step < 1 m of 3.5)", maxStep < 1.0);
    check("offset is 94% or more of the clamp (3.5 m) after 0.5 s", b.offE > 0.94 * 3.5 && b.offE <= 3.5001);
    b.choose(runs, runs, box, one, hw(runs), 7, 0, Math.PI, 25);  // wrong way now: dropped
    long t1 = t0 + 500;
    for (int i = 1; i <= 15; i++) b.step(t1 + i * 1000 / 30);
    check("offset eases back out to under 6% after 0.5 s", Math.abs(b.offE) < 0.06 * 3.5);

    // v2: the clamp to the road's edge (hw - 1 m margin), toward the line
    Snap c1 = new Snap();
    c1.choose(runs, runs, box, one, hw(runs), 0, 0, 0, 25);
    double[] o1 = settle(c1);
    check("frontage d=7, one-way hw 4.5: offset 3.5 m east", Math.abs(o1[0] - 3.5) < 1e-3 && Math.abs(o1[1]) < 1e-3);
    // car 3 m from a line of hw 4.5: inside, no offset
    float[][] in1 = {vline(3, true)};
    Snap c2 = new Snap();
    check("car inside the road: chosen", c2.choose(in1, in1, box(in1), new boolean[] {true}, hw(in1), 0, 0, 0, 25));
    double[] o2 = settle(c2);
    check("car inside the road: zero offset", Math.abs(o2[0]) < 1e-6 && Math.abs(o2[1]) < 1e-6);
    // just beyond the margin line (3.5 m): 4 m gives 0.5 m
    float[][] in2 = {vline(4, true)};
    Snap c3 = new Snap();
    c3.choose(in2, in2, box(in2), new boolean[] {true}, hw(in2), 0, 0, 0, 25);
    check("4 m from the line, allow 3.5: offset 0.5 m", Math.abs(settle(c3)[0] - 0.5) < 1e-3);
    // a two-way road (hw 9: ROAD_M[1] / 2 / 2): the car 4 m off the centre, in its lane, does not move
    float[][] tw1 = {vline(4, true)};
    float[] hw9 = {9f};
    Snap c4 = new Snap();
    check("two-way road, car in its lane: chosen", c4.choose(tw1, tw1, box(tw1), new boolean[] {false}, hw9, 0, 0, 0, 25));
    double[] o4 = settle(c4);
    check("two-way road, car in its lane: position unchanged", Math.abs(o4[0]) < 1e-6 && Math.abs(o4[1]) < 1e-6);
    // the offset is toward the line from either side
    float[][] west = {vline(-7, true)};
    Snap c5 = new Snap();
    c5.choose(west, west, box(west), new boolean[] {true}, hw(west), 0, 0, 0, 25);
    check("line to the west: offset -3.5 m", Math.abs(settle(c5)[0] + 3.5) < 1e-3);
    // a road that is not drawn (hw 0: a hidden class) is never snapped to, and the next best is
    float[][] pair = {vline(7, true), vline(12, true)};
    float[] hidden = {0f, 4.5f};
    Snap c6 = new Snap();
    check("hidden road skipped for the drawn one behind it", c6.choose(pair, pair, box(pair), new boolean[] {true, true}, hidden, 0, 0, 0, 25) && c6.run == 1);
    float[] allHidden = {0f, 0f};
    Snap c7 = new Snap();
    check("only hidden roads: no snap", !c7.choose(pair, pair, box(pair), new boolean[] {true, true}, allHidden, 0, 0, 0, 25) && c7.run == -1);
    check("only hidden roads: zero offset", settle(c7)[0] == 0);
    // a held line that turns hidden (the minor streets fade out) is let go while slow, too
    Snap c8 = new Snap();
    c8.choose(pair, pair, box(pair), new boolean[] {true, true}, new float[] {4.5f, 4.5f}, 0, 0, 0, 25);
    c8.choose(pair, pair, box(pair), new boolean[] {true, true}, allHidden, 0, 0, 0, 1.0);
    check("held line gone hidden: dropped while slow", c8.run == -1);

    if (fails > 0) {
      System.out.println(fails + " FAILED");
      System.exit(1);
    }
    System.out.println("SnapTest: all passed");
  }
}
