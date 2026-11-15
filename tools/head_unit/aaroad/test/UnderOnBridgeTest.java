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

  static float IK = 1 / 3f;  // world m to display m, as MapView's f.ik

  static int over(CloseUp.Detail d, float e, float n, float hdg) {
    float ch = (float) Math.cos(Math.toRadians(hdg)), sh = (float) Math.sin(Math.toRadians(hdg));
    return Under.covering(d, e, n, ch, sh, IK, 1, ROAD_M, FLOOR, CASING, CAR, new int[48]);
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
    // The car's own road is one piece after another (2026-11-15, found replaying real Exchange Pkwy tiles + GPS): the
    // deck and its 60 m ramp pieces. A raised end of the next piece, 25 to 50 m along, must not draw the outline.
    // Bridge e -300..300 over US-75: ramps -300..-240 and 240..300, flat deck between.
    // the replay's failing case: a ground road (a service lane, a ramp tail) runs under the deck 3 m off its line and is
    // nearer the car than the deck, so carHeight reads 0 on the deck and every raised piece looks "over" it
    in.road(new float[] {-300, -3, 300, -3}, 3, false, 0, 0, false);
    d = CloseUp.build(in);
    check(Under.carHeight(d, 200, -3, 0, 1, ROAD_M) == 0, "scene check: carHeight reads 0 on the deck beside the ground lane (the replay's case)");
    int wrongNext = 0, nextTried = 0, wrongRamp = 0, rampTried = 0;
    for (float ik : new float[] {1 / 6f, 1 / 3f, 1 / 2f, 1f, 2f}) {  // the zoom changes how far a raised end lifts
      IK = ik;
      for (float e : new float[] {190, 200, 205, 210, 215}) {  // 25 to 50 m before the east ramp (starts at 240)
        for (float off : new float[] {0, 3, -3}) {
          wrongNext += over(d, e, off, 90) != 0 ? 1 : 0;
          wrongNext += over(d, -e, off, 270) != 0 ? 1 : 0;
          nextTried += 2;
        }
      }
      for (float e = 245; e <= 295; e += 10) {
        for (float off : new float[] {0, 3, -3}) {
          wrongRamp += over(d, e, off, 270) != 0 ? 1 : 0;  // rising towards the deck
          wrongRamp += over(d, -e, off, 90) != 0 ? 1 : 0;
          rampTried += 2;
        }
      }
    }
    IK = 1 / 3f;
    check(wrongNext == 0, "on the flat deck 25-50 m short of the next ramp piece, either way, 0-3 m off, 5 zooms: no outline (" + wrongNext + " of " + nextTried + " wrong)");
    check(wrongRamp == 0, "on the ramp rising to the deck, either way, 5 zooms: no outline (" + wrongRamp + " of " + rampTried + " wrong)");
    check(over(d, 0, 0, 0) > 0 && over(d, 0, 30, 180) > 0, "the crossing road under the same bridge still draws (grown chain does not eat crossings)");

    // a DIFFERENT road beside it: a raised frontage bridge 18 m to the north over US-75, the car on its own ground road
    CloseUp.Lines side = new CloseUp.Lines();
    side.road(new float[] {0, -800, 0, 800}, 0, false, 0, 0, false);
    side.road(new float[] {-900, 0, 900, 0}, 3, false, 0, 0, false);  // the car's road
    side.road(new float[] {-300, 18, 0, 18, 300, 18}, 3, true, 1, 0, false);  // raised, parallel, not touching it
    CloseUp.Detail ds = CloseUp.build(side);
    int sideOld = 0;
    for (float e = -30; e <= 30; e += 5) sideOld += over(ds, e, 0, 90) > 0 ? 1 : 0;
    System.out.println("info side road: " + sideOld + " of 13 frames draw the outline");
    check(sideOld > 0, "car on its own road with a different, unconnected raised road 18 m beside it: still outline (" + sideOld + ")");

    System.out.println(bad == 0 ? "all ok" : bad + " failed");
    System.exit(bad == 0 ? 0 : 1);
  }
}
