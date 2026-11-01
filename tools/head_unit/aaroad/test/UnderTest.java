package com.starpilot.aaroad;

// Desktop check of Under: which decks are over the car, never the car's own deck, and the outline's ease.
//   javac -d /tmp/under src/com/starpilot/aaroad/CloseUp.java src/com/starpilot/aaroad/Under.java test/UnderTest.java
//   java -cp /tmp/under com.starpilot.aaroad.UnderTest
// Exits 1 on a failure.
public class UnderTest {
  static final float[] ROAD_M = {56, 36, 20, 12, 10};  // as MapView
  static final float PX_DM = 1.7156f;
  static final float FLOOR = 2 / PX_DM, CASING = 8 / PX_DM, CAR = Under.CAR_PX / PX_DM;
  static final float IK = 1 / 3f;  // range 210 m: the close-up fully in
  static int bad;

  static void check(boolean ok, String what) {
    System.out.println((ok ? "ok   " : "FAIL ") + what);
    if (!ok) bad++;
  }

  static float[] line(float... p) {
    return p;
  }

  // decks over a car at (e, n) heading hdg degrees (0 north, 90 east), the close-up at hs
  static int over(CloseUp.Detail d, float e, float n, float hdg, float hs) {
    float ch = (float) Math.cos(Math.toRadians(hdg)), sh = (float) Math.sin(Math.toRadians(hdg));
    return Under.covering(d, e, n, ch, sh, IK, hs, ROAD_M, FLOOR, CASING, CAR, new int[48]);
  }

  public static void main(String[] a) {
    // a ground motorway north-south at e = 0, crossed by a layer-1 motorway bridge east-west at n = 0
    CloseUp.Lines in = new CloseUp.Lines();
    in.road(line(0, -800, 0, 800), 0, false, 0, 0, false);
    in.road(line(-900, 0, -300, 0), 0, false, 0, 0, false);
    in.road(line(-300, 0, 0, 0, 300, 0), 0, true, 1, 0, false);
    in.road(line(300, 0, 900, 0), 0, false, 0, 0, false);
    // and a layer-2 flyover north-south at e = 150, over the bridge
    in.road(line(150, -400, 150, 0, 150, 400), 3, true, 2, 0, false);
    CloseUp.Detail d = CloseUp.build(in);

    check(over(d, 0, 0, 0, 1) > 0, "on the ground under the bridge: the bridge is over the car");
    check(over(d, 0, 0, 180, 1) > 0, "the same heading south");
    check(over(d, 0, 0, 90, 1) == 0, "on the bridge itself heading east: nothing over the car");
    check(over(d, 0, 0, 270, 1) == 0, "on the bridge heading west: nothing over the car");
    check(over(d, 0, 4, 85, 1) == 0, "on the bridge a little off its line and its way: still on it");
    check(over(d, 0, 0, 0, 0) == 0, "cruising (close-up shut): no outline");
    check(over(d, 0, -300, 0, 1) == 0 && over(d, 0, 300, 0, 1) == 0, "well before and after the bridge: nothing over");
    check(over(d, 150, 0, 90, 1) > 0, "on the bridge under the flyover: the flyover is over the car");
    check(over(d, 150, 0, 0, 1) == 0, "on the flyover heading north: nothing over the car");

    // driving north under the bridge: one stretch under, no flicker, around the deck
    int changes = 0;
    boolean was = false;
    float first = Float.NaN, last = Float.NaN;
    for (float n = -150; n <= 150; n += 0.5f) {
      boolean u = over(d, 0, n, 0, 1) > 0;
      if (u != was) changes++;
      if (u && Float.isNaN(first)) first = n;
      if (u) last = n;
      was = u;
    }
    check(changes == 2, "one stretch under the deck driving through: " + changes + " changes");
    // the deck is 56 m wide plus casing and the car's body; drawn lifted, it shows a little toward the car
    check(first > -60 && first < -30 && last > 45 && last < 75 && last + first > 0,
          "the stretch is the drawn deck over the car's body, shifted ahead: n " + first + " to " + last);

    // a half-open close-up still finds it (the deck is drawn lower)
    check(over(d, 0, 0, 0, 0.3f) > 0, "close-up part way in: still under");

    // no raised decks at all
    in = new CloseUp.Lines();
    in.road(line(0, -800, 0, 800), 0, false, 0, 0, false);
    check(over(CloseUp.build(in), 0, 0, 0, 1) == 0, "no bridges: nothing over");
    check(Under.covering(null, 0, 0, 1, 0, IK, 1, ROAD_M, FLOOR, CASING, CAR, new int[4]) == 0, "no detail: nothing");

    // the ease: 150 ms each way, turning back from part way
    Under.Fade f = new Under.Fade();
    check(f.level(0) == 0 && !f.moving(0), "starts off");
    f.set(true, 1000);
    check(Math.abs(f.level(1075) - 0.5f) < 1e-3, "half in after 75 ms");
    check(f.level(1150) == 1 && !f.moving(1150) && f.moving(1100), "fully in at 150 ms, then still");
    f.set(true, 1200);
    check(f.level(1200) == 1, "set again on: stays in");
    f.set(false, 1300);
    check(Math.abs(f.level(1375) - 0.5f) < 1e-3 && f.level(1450) == 0 && !f.moving(1450), "out over 150 ms");
    f.set(true, 2000);
    f.set(false, 2060);  // a deck edge crossed back after 60 ms
    check(Math.abs(f.level(2060) - 0.4f) < 1e-3, "turned back at 0.4");
    check(Math.abs(f.level(2090) - 0.2f) < 1e-3 && f.level(2120) == 0, "and out from there, not from 1");

    System.out.println(bad == 0 ? "all ok" : bad + " failed");
    System.exit(bad == 0 ? 0 : 1);
  }
}
