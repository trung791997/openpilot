package com.starpilot.aaroad;

// Whether the dash map shows over Honda Hack's page, and why not when it does not. Plain Java, so the rule can be
// checked on a desktop (test/LivenessTest.java).
//
// The map comes up once Honda Hack's nav page is on screen and the comma's position is fresh. Once up, it used to go
// back to Honda Hack's page whenever the position was GAP_HOLD_MS old (Peter 2026-11-01: the whole cluster map drops
// to the Honda Hack view now and then and comes back). The comma sends a position only while its GPS fix is under
// MAX_CARRY (1 s) old (hu_feed.py carry), and the car's GPS goes quiet for longer than that: drive
// 00000071--37216af48f (2026-10-31) had a 25.9 s gap at 67 mph with no route, so the map would have gone for
// about 15 s and come back. Now, once up, the map rides out a gap like that while the comma's feed still talks
// (its state comes once a second whether or not it has a fix), with the car held where it was, for up to
// POSITION_HOLD_MS. It still goes at once when Honda Hack's page goes, and once the comma's feed stops.
final class Liveness {
  static final long STALE_MS = 2000;           // to come up, the position has to be newer than this
  static final long GAP_HOLD_MS = 10000;       // once up, a position gap this long is ridden out on its own
  static final long FEED_STALE_MS = 5000;      // the comma's state datagram comes once a second; past this it is gone
  static final long POSITION_HOLD_MS = 60000;  // once up, the longest position gap ridden out while the feed talks

  // Why the map should be hidden now, or null when it should show. Ages in ms, Long.MAX_VALUE for never.
  static String hideReason(boolean pageShown, boolean live, long poseAge, long feedAge) {
    if (!pageShown) return "page not shown";
    if (poseAge == Long.MAX_VALUE) return "no position yet";
    if (poseAge < (live ? GAP_HOLD_MS : STALE_MS)) return null;
    if (live && feedAge < FEED_STALE_MS && poseAge < POSITION_HOLD_MS) return null;  // a GPS gap, the comma still here
    if (feedAge >= FEED_STALE_MS) {
      return "feed stale: comma silent " + (feedAge == Long.MAX_VALUE ? "since start" : feedAge / 1000 + " s")
          + ", no position for " + poseAge / 1000 + " s";
    }
    return "no position for " + poseAge / 1000 + " s";
  }

  // Whether the map is up only because of the feed hold (it would have gone before 2026-11-01).
  static boolean holding(boolean live, long poseAge, long feedAge) {
    return live && poseAge != Long.MAX_VALUE && poseAge >= GAP_HOLD_MS && feedAge < FEED_STALE_MS
        && poseAge < POSITION_HOLD_MS;
  }

  private Liveness() {}
}
