package com.starpilot.aaroad;

// Desktop test of Perf: the pacing state machine, the kerb-skip width math, the Douglas-Peucker band and its
// hysteresis, the heap-drop counter. Static only -- nothing here touches a device.
public final class PerfTest {
  static int fails;

  static void check(boolean ok, String what) {
    if (!ok) {
      fails++;
      System.out.println("FAIL " + what);
    }
  }

  static long t;

  // one closed window: 5 frames 250 ms apart, each costing cost ms
  static void win(Perf.Pace p, double cost) {
    for (int i = 0; i < 5; i++) p.frame(t += 250, cost);
  }

  public static void main(String[] a) {
    // pacing: fast at start; a second of 35 ms frames goes slow; stays slow under 22 ms for two windows, back on the third
    Perf.Pace p = new Perf.Pace();
    check(p.frameMs() == 33, "starts at 33");
    win(p, 35);
    check(p.frameMs() == 42 && p.switches == 1, "slow after avg 35: " + p.frameMs());
    for (int w = 0; w < 2; w++) win(p, 15);
    check(p.frameMs() == 42, "two calm windows are not enough");
    win(p, 25);  // 22..30: resets the calm count
    for (int w = 0; w < 2; w++) win(p, 15);
    check(p.frameMs() == 42, "a middling window resets the count");
    win(p, 15);
    check(p.frameMs() == 33 && p.switches == 2, "back after three calm windows: " + p.frameMs());
    Perf.Pace q = new Perf.Pace();
    win(q, 30);  // exactly 30 is not above 30
    check(q.frameMs() == 33, "avg 30 stays fast");
    // next: normal, and late (no back-to-back)
    check(p.next(1000, 1010) == 1033, "next on time");
    check(p.next(1000, 1060) == 1064, "late frame waits 4 ms");
    check(p.next(1000, 1029) == 1033, "close to due keeps the cap");
    p.rest();

    // kerb: off below 0.15, full at 0.18, ramp between; the surface takes what the kerb gives up
    check(Perf.KerbLod.alpha(0.10f) == 0 && Perf.KerbLod.alpha(0.15f) == 0, "kerb off");
    check(Perf.KerbLod.alpha(0.18f) == 1 && Perf.KerbLod.alpha(0.5f) == 1, "kerb full");
    check(Math.abs(Perf.KerbLod.alpha(0.165f) - 0.5f) < 1e-4, "kerb half");
    check(Math.abs(Perf.KerbLod.surfaceExtra(2f, 0) - 2f) < 1e-6, "surface +kerb when off");
    check(Perf.KerbLod.surfaceExtra(2f, 1) == 0, "surface +0 when kerb full");
    for (float ik = 0.05f; ik < 0.4f; ik += 0.01f) {  // kerb + extra is constant: the silhouette does not move
      float al = Perf.KerbLod.alpha(ik);
      float total = 2f * al + Perf.KerbLod.surfaceExtra(2f, al);
      check(Math.abs(total - 2f) < 1e-5, "silhouette constant at " + ik);
    }

    // DP band: boundaries 0.25 and 0.15 with 8% margins
    check(Perf.DpBand.next(0, 0.30f) == 0, "fine at 0.30");
    check(Perf.DpBand.next(0, 0.24f) == 0, "inside the margin stays 0");
    check(Perf.DpBand.next(0, 0.22f) == 1, "coarser at 0.22");
    check(Perf.DpBand.next(1, 0.26f) == 1, "inside the margin stays 1");
    check(Perf.DpBand.next(1, 0.28f) == 0, "finer at 0.28");
    check(Perf.DpBand.next(1, 0.13f) == 2, "coarsest at 0.13");
    check(Perf.DpBand.next(2, 0.15f) == 2 && Perf.DpBand.next(2, 0.16f) == 2, "inside margin stays 2");
    check(Perf.DpBand.next(2, 0.17f) == 1, "back to 1 at 0.17");
    check(Perf.DpBand.next(0, 0.05f) == 2, "jumps two bands at once");
    check(Perf.DpBand.next(2, 0.9f) == 0, "and back");
    int flips = 0, b = 1;  // a zoom wobbling +-2% around a boundary never flips
    for (int i = 0; i < 1000; i++) {
      int n = Perf.DpBand.next(b, 0.25f * (1 + 0.02f * (i % 2 == 0 ? 1 : -1)));
      if (n != b) flips++;
      b = n;
    }
    check(flips == 0, "no flapping at the boundary: " + flips);
    check(Perf.DpBand.tolerance(0) == 0 && Perf.DpBand.tolerance(1) == 2 && Perf.DpBand.tolerance(2) == 4, "tolerances");

    // heap drops
    Perf.GcWatch g = new Perf.GcWatch();
    g.sample(10_000_000);
    g.sample(10_100_000);
    g.sample(9_000_000);
    g.sample(9_050_000);
    check(g.drops == 1 && g.worstDrop == 1_100_000, "one drop counted");
    g.sample(9_050_000 - 1000);
    check(g.drops == 1, "a small dip is not a GC");
    g.clear();
    check(g.drops == 0 && g.worstDrop == 0, "clear");

    System.out.println(fails == 0 ? "PerfTest OK" : "PerfTest FAILED " + fails);
    if (fails != 0) System.exit(1);
  }
}
