package com.starpilot.aaroad;

// Desktop check of Liveness, when the dash map shows over Honda Hack's page.
//   javac -d /tmp/liveness src/com/starpilot/aaroad/Liveness.java test/LivenessTest.java
//   java -cp /tmp/liveness com.starpilot.aaroad.LivenessTest
// Fails (exit 1) unless: drive 00000071's 25.9 s car-GPS gap (the comma's state still coming once a second) keeps the
// map up the whole way, where the old 10 s rule hid it for about 15 s; the page going hides it at once; the comma's
// feed going quiet hides it once the position is also past the old 10 s hold; the hold stops at 60 s; and the map
// still comes up only on a fresh position.
public class LivenessTest {
  static int fails = 0;

  static void check(boolean ok, String what) {
    if (!ok) {
      fails++;
      System.out.println("FAIL " + what);
    }
  }

  // Runs the map tick at 10 Hz from t=0 to end (ms) with a pose each second outside [gapFrom, gapTo) and the comma's
  // state each second until feedUntil; returns the ms the map was hidden after it first came up, by the new rule
  // (old = false) or the 10 s rule it replaced (old = true).
  static long hiddenMs(long end, long gapFrom, long gapTo, long feedUntil, boolean old) {
    boolean live = false;
    long poseAt = -1, stateAt = -1, hidden = 0;
    boolean up = false;
    for (long t = 0; t <= end; t += 100) {
      if (t % 1000 == 0 && (t < gapFrom || t >= gapTo)) poseAt = t;
      if (t % 1000 == 0 && t < feedUntil) stateAt = t;
      long poseAge = poseAt >= 0 ? t - poseAt : Long.MAX_VALUE;
      long feedAge = stateAt >= 0 ? t - stateAt : Long.MAX_VALUE;
      live = old ? poseAge < (live ? Liveness.GAP_HOLD_MS : Liveness.STALE_MS)
          : Liveness.hideReason(true, live, poseAge, feedAge) == null;
      up |= live;
      if (up && !live) hidden += 100;
    }
    return hidden;
  }

  public static void main(String[] a) {
    // drive 00000071--37216af48f (2026-10-31), ~320 s: the car's fixes stopped for 25.9 s at 67 mph
    long gapFrom = 320000, gapTo = 345900;
    long oldHidden = hiddenMs(400000, gapFrom, gapTo, Long.MAX_VALUE, true);
    check(oldHidden >= 14000 && oldHidden <= 17000, "the old rule hid the map ~15 s in the drive 71 gap, got " + oldHidden);
    check(hiddenMs(400000, gapFrom, gapTo, Long.MAX_VALUE, false) == 0, "the drive 71 gap keeps the map up");
    // and drive 71's shorter gaps (5.0, 4.2, 3.1 s) never took it down either way
    check(hiddenMs(60000, 20000, 25000, Long.MAX_VALUE, false) == 0, "a 5 s gap keeps the map");

    // the page going hides at once, whatever the position
    check("page not shown".equals(Liveness.hideReason(false, true, 0, 0)), "page off hides at once");
    check("no position yet".equals(Liveness.hideReason(true, false, Long.MAX_VALUE, 0)), "no position yet");

    // the comma gone: up to the old 10 s on its own, then hidden, with the reason saying the feed went
    check(Liveness.hideReason(true, true, 9900, 9900) == null, "a short silence is ridden out as before");
    String r = Liveness.hideReason(true, true, 10000, 10000);
    check(r != null && r.startsWith("feed stale: comma silent 10 s"), "silent comma hides past 10 s: " + r);
    check(hiddenMs(60000, 30000, Long.MAX_VALUE, 30000, false) == 60000 - 39000 + 100,  // last pose and state at 29 s
        "the comma gone after 29 s takes the map down at 39 s");
    // a hello pause the comma answers by dropping the feed: state stops with the poses, same 10 s as before
    check(Liveness.hideReason(true, true, 12000, 4900) == null, "feed alive keeps the map at 12 s");
    check(Liveness.hideReason(true, true, 12000, 5000) != null, "feed 5 s quiet and no position hides");

    // the hold stops at 60 s even with the feed alive
    check(Liveness.hideReason(true, true, 59900, 500) == null, "held to 60 s");
    check("no position for 60 s".equals(Liveness.hideReason(true, true, 60000, 500)), "not past 60 s");

    // coming up needs a fresh position, feed or not
    check(Liveness.hideReason(true, false, 1900, 0) == null, "comes up on a fresh position");
    check(Liveness.hideReason(true, false, 2000, 0) != null, "a 2 s old position does not bring it up");
    check(Liveness.hideReason(true, false, 20000, 0) != null, "the feed alone does not bring it up");

    // holding: only the part the old rule would have hidden
    check(!Liveness.holding(true, 9900, 0) && Liveness.holding(true, 10000, 0), "holding starts at 10 s");
    check(!Liveness.holding(true, 10000, 5000) && !Liveness.holding(false, 10000, 0), "holding needs live and the feed");

    if (fails > 0) {
      System.out.println(fails + " failed");
      System.exit(1);
    }
    System.out.println("LivenessTest ok");
  }
}
