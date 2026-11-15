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
    // 2026-11-07: the outline now works at every zoom, not only the close-up (hs 1 to 0); zoomed out (hs 0) the
    // bridge is drawn flat, and the car's flat clip still covers it, so the outline still shows.
    check(over(d, 0, 0, 0, 0) > 0, "zoomed out (hs 0): the flat deck still shows the outline");
    check(over(d, 0, 0, 90, 0) == 0, "zoomed out, on the bridge itself: still nothing over the car");
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

    // a ground frontage road north-south at e = 20 (20 m from the viaduct's centreline, inside its reach), beside a
    // layer-1 viaduct north-south at e = 0: a car on the frontage road is nearer the ground road than the viaduct,
    // so it reads as on the ground and the viaduct over it shows an outline; a car on the viaduct itself reads as
    // on the viaduct and shows none.
    in = new CloseUp.Lines();
    in.road(line(0, -800, 0, 800), 1, true, 1, 0, false);  // the viaduct, layer 1, north-south
    in.road(line(20, -800, 20, 800), 0, false, 0, 0, false);  // the frontage road, ground, 20 m off its centre
    // a ground cross-street well up the road, away from both test points below, so overpasses() marks the viaduct
    // raised (a bridge needs some lower or ground line actually crossing it; a same-or-higher bridge does not count)
    in.road(line(-50, 600, 50, 600), 0, false, 0, 0, false);
    CloseUp.Detail dv = CloseUp.build(in);
    check(over(dv, 20, 0, 0, 1) > 0, "frontage road 20 m off a viaduct's centre: on the ground, the viaduct shows");
    check(over(dv, 0, 0, 0, 1) == 0, "on the viaduct itself: nothing over (it is the car's own road)");
    // the viaduct's own reach (half width + ON_DECK_M) extends well past the frontage road's position; it is the
    // nearer-candidate rule (Under.carHeight), not any gap between the two reaches, that gives the ground the win
    float viaductHalf = ROAD_M[1] / 2 + Under.ON_DECK_M;
    check(viaductHalf > 20, "the viaduct's own reach already covers the frontage road's position: " + viaductHalf);

    // Part A fix (2026-11-08): a route edge that rides its own layer-1 deck for most of its length but passes
    // under a layer-2 deck partway along it must split there, not read as under (or not) for its whole length.
    in = new CloseUp.Lines();
    in.road(line(0, -500, 0, 500), 0, true, 1, 0, false);       // the route's own deck, layer 1, north-south at e = 0
    in.road(line(-50, -480, 50, -480), 0, false, 0, 0, false);  // a ground cross street, away from the test: makes it an overpass
    in.road(line(-50, 100, 50, 100), 0, true, 2, 0, false);     // a layer-2 flyover crossing it at n = 100
    CloseUp.Detail dp = CloseUp.build(in);
    Under.Cuts cuts = Under.underCuts(dp, 0, -400, 0, 400);
    check(!cuts.first, "the own-deck part of the edge, away from the flyover, is not under anything higher");
    check(cuts.at.length == 2, "two transitions: into the flyover's crossing and back out of it, found " + cuts.at.length);
    if (cuts.at.length == 2) {
      float into = -400 + cuts.at[0], out = -400 + cuts.at[1];
      check(into > 55 && into < 95, "splits just before the flyover's reach: n " + into);
      check(out > 105 && out < 145, "and just after it: n " + out);
    }

    // Split-harness fix (2026-11-08): a route edge drawn as a straight chord across its own curving connector deck
    // (the deck's segments 35-60 deg off the chord, past carHeight's ON_DECK_DEG) used to fall back to ground height,
    // so the deck it rides read as a higher deck over it and hid the route. A raised segment along the route
    // (within ALONG_DEG) never covers it.
    in = new CloseUp.Lines();
    in.road(line(0, 0, 60, 40, 140, 60), 3, true, 2, 0, false);  // the connector deck, layer 2, curving
    in.road(line(-50, 20, 200, 20), 0, false, 0, 0, false);              // ground road under it: makes it raised
    CloseUp.Detail dc = CloseUp.build(in);
    Under.Cuts own = Under.underCuts(dc, 0, 0, 140, 60);
    check(!own.first && own.at.length == 0, "a chord across its own curving deck is never under it, cuts " + own.at.length);
    // and a real flyover still hides a ground route crossing beneath it
    in = new CloseUp.Lines();
    in.road(line(-50, 0, 50, 0), 0, true, 1, 0, false);   // the flyover, east-west
    in.road(line(0, -300, 0, 300), 0, false, 0, 0, false);  // the route's ground road, north-south
    CloseUp.Detail df = CloseUp.build(in);
    Under.Cuts gr = Under.underCuts(df, 0, -200, 0, 200);
    check(!gr.first && gr.at.length == 2, "a ground route still goes under a crossing flyover, cuts " + gr.at.length);

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

    // the ease scales down with speed: 150 ms at or under 15 m/s, 80 ms by about 28 m/s and above
    check(Under.fadeMs(5) == 150, "5 m/s: still the full 150 ms");
    check(Under.fadeMs(15) == 150, "15 m/s: still the full 150 ms (the reference speed)");
    check(Under.fadeMs(20) == 112 || Under.fadeMs(20) == 113, "20 m/s: part way down, " + Under.fadeMs(20) + " ms");
    check(Under.fadeMs(28) == 80, "28 m/s: at the floor, " + Under.fadeMs(28) + " ms");
    check(Under.fadeMs(35) == 80, "35 m/s: still at the 80 ms floor");
    Under.Fade fs = new Under.Fade();
    fs.set(true, 3000, Under.fadeMs(35));  // a new transition at 35 m/s: the 80 ms pace takes hold
    check(fs.level(3040) == 0.5f, "half in after 40 ms at the 80 ms pace: " + fs.level(3040));
    check(fs.level(3080) == 1, "fully in at 80 ms at that pace");

    System.out.println(bad == 0 ? "all ok" : bad + " failed");
    System.exit(bad == 0 ? 0 : 1);
  }
}
