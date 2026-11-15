package com.starpilot.aaroad;

// Desktop check that the under-deck outline never shows while the car drives ON the bridge (owner 2026-11-15:
// Exchange Pkwy over US-75), only when it crosses beneath it.
//   javac -d OUT src/com/starpilot/aaroad/CloseUp.java src/com/starpilot/aaroad/Under.java test/UnderOnBridgeTest.java
//   java -cp OUT com.starpilot.aaroad.UnderOnBridgeTest
// Exits 1 on a failure.
public class UnderOnBridgeTest {
  static final float[] ROAD_M = {56, 36, 20, 12, 10};
  static final float PX_DM = 1.7156f;
  static final float FLOOR = 2 / PX_DM, CASING = 8 / PX_DM, CAR = Under.CAR_PX / PX_DM;
  static int bad;

  static void check(boolean ok, String what) {
    System.out.println((ok ? "ok   " : "FAIL ") + what);
    if (!ok) bad++;
  }

  static int over(CloseUp.Detail d, float e, float n, float hdg) {
    float ch = (float) Math.cos(Math.toRadians(hdg)), sh = (float) Math.sin(Math.toRadians(hdg));
    return Under.covering(d, e, n, ch, sh, 1 / 3f, 1, ROAD_M, FLOOR, CASING, CAR, new int[48]);
  }

  public static void main(String[] a) {
    // Exchange: layer-1 bridge east-west at n = 0 over US-75 (ground, north-south at e = 0)
    CloseUp.Lines in = new CloseUp.Lines();
    in.road(new float[] {0, -800, 0, 800}, 0, false, 0, 0, false);
    in.road(new float[] {-900, 0, -300, 0}, 0, false, 0, 0, false);
    in.road(new float[] {-300, 0, 0, 0, 300, 0}, 0, true, 1, 0, false);
    in.road(new float[] {300, 0, 900, 0}, 0, false, 0, 0, false);
    CloseUp.Detail d = CloseUp.build(in);
    int wrong = 0;
    for (float e = -120; e <= 120; e += 10)
      for (float h : new float[] {90, 270, 75, 105, 255, 285})  // along the bridge, and noisy within 15 deg
        if (over(d, e, 0, h) != 0) wrong++;
    check(wrong == 0, "driving along the bridge, either way, +-15 deg noise: never an outline (" + wrong + " wrong)");
    check(over(d, 0, 0, 90) == 0, "on the bridge over US-75, heading along it: no outline");
    check(over(d, 0, 5, 270) == 0, "on the bridge, 5 m off the centreline, heading back: no outline");
    check(over(d, 0, 0, 0) > 0, "on US-75 crossing under at 90 deg: outline");
    check(over(d, 0, 0, 180) > 0, "same going south: outline");
    check(over(d, 0, 0, 45) > 0, "below, crossing at 45 deg: outline");
    check(over(d, 0, 0, 135) > 0, "below, crossing at 45 deg the other way: outline");
    System.out.println(bad == 0 ? "all ok" : bad + " failed");
    System.exit(bad == 0 ? 0 : 1);
  }
}
