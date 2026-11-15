package com.starpilot.aaroad;

// Desktop test for Lanes (no Android):
//   javac -d /tmp/lt src/com/starpilot/aaroad/Lanes.java test/LanesTest.java && java -cp /tmp/lt com.starpilot.aaroad.LanesTest
public final class LanesTest {
  private static int fails;

  private static void eq(String what, Object got, Object want) {
    boolean ok = got == null ? want == null : got.equals(want);
    if (!ok) {
      fails++;
      System.out.println("FAIL " + what + ": got " + got + ", want " + want);
    }
  }

  private static void yes(String what, boolean ok) {
    eq(what, ok, true);
  }

  public static void main(String[] a) {
    int S = Lanes.STRAIGHT, SR = Lanes.SLIGHT_RIGHT, R = Lanes.RIGHT, L = Lanes.LEFT, U = Lanes.UTURN;

    // the arrow lit: the comma's when the lane carries it
    eq("exit lane, comma's arrow", Lanes.lit(S | SR, true, SR, "slight right"), SR);
    eq("a lane not to take", Lanes.lit(S | SR, false, SR, "slight right"), 0);
    eq("no arrows", Lanes.lit(0, true, S, "straight"), 0);
    // else the maneuver's way, if the lane carries it
    eq("no comma arrow, by modifier", Lanes.lit(L | U, true, 0, "left"), L);
    eq("modifier spelt as the comma does", Lanes.lit(S | SR, true, 0, "slightRight"), SR);
    eq("an arrow the lane lacks", Lanes.lit(S | R, true, L, "left"), S);
    eq("two bits are not one arrow", Lanes.lit(S | R, true, S | R, "right"), R);
    // else the lane's first
    eq("modifier the lane lacks", Lanes.lit(R | S, true, 0, "left"), S);
    eq("bits past the last arrow", Lanes.lit(S | 4096, true, 4096, ""), S);

    eq("bit uturn", Lanes.bit("uturn"), U);
    eq("bit sharp left", Lanes.bit("sharp left"), Lanes.SHARP_LEFT);
    eq("bit sharpRight", Lanes.bit("sharpRight"), Lanes.SHARP_RIGHT);
    eq("bit unknown", Lanes.bit("sideways"), S);
    eq("bit null", Lanes.bit(null), S);

    // shows within 1 km of the maneuver, with lanes
    int[] four = {S, S, S | SR, SR};
    yes("at 800 m", Lanes.shown(four, 800));
    yes("at the maneuver", Lanes.shown(four, 0));
    yes("not at 1.2 km", !Lanes.shown(four, 1200));
    yes("not with the distance unknown", !Lanes.shown(four, -1));
    yes("not without lanes", !Lanes.shown(new int[0], 300));
    yes("not with null", !Lanes.shown(null, 300));

    // the lit arrow is drawn last, over the dim ones
    int[] o = new int[10];
    int n = Lanes.order(L | U | S, L, o);
    eq("order count", n, 3);
    eq("order lit last", o[2], L);
    eq("order none lit", Lanes.order(S | R, 0, o), 2);

    eq("cell shrinks to fit", Lanes.cell(10, 160, 34), 16f);
    eq("cell at most", Lanes.cell(2, 160, 34), 34f);
    eq("cell of none", Lanes.cell(0, 160, 34), 0f);

    // shapes: every arrow fits its cell, starts at the foot, and points its way
    float[] line = new float[2 * Lanes.POINTS], head = new float[6];
    float s = 26, cx = 100, cy = 50;
    for (int bit = 1; bit <= Lanes.MERGE_RIGHT; bit <<= 1) {
      for (boolean left : new boolean[] {false, true}) {
        int p = Lanes.arrow(bit, left, cx, cy, s, line, head);
        yes("points fit " + bit, p >= 2 && p <= Lanes.POINTS);
        for (int i = 0; i < 2 * p; i += 2) {
          yes("inside x " + bit, Math.abs(line[i] - cx) <= 0.5f * s);
          yes("inside y " + bit, Math.abs(line[i + 1] - cy) <= 0.5f * s);
        }
        for (int i = 0; i < 6; i += 2) {
          yes("head inside x " + bit, Math.abs(head[i] - cx) <= 0.55f * s);
          yes("head inside y " + bit, Math.abs(head[i + 1] - cy) <= 0.55f * s);
        }
      }
    }
    Lanes.arrow(S, false, cx, cy, s, line, head);
    yes("straight points up", head[1] < cy && Math.abs(head[0] - cx) < 0.01f);
    Lanes.arrow(R, false, cx, cy, s, line, head);
    yes("right points right", head[0] > cx + 0.25f * s);
    Lanes.arrow(Lanes.SLIGHT_LEFT, false, cx, cy, s, line, head);
    yes("slight left up and left", head[0] < cx && head[1] < cy);
    Lanes.arrow(Lanes.SHARP_RIGHT, false, cx, cy, s, line, head);
    yes("sharp right points right and back down", head[0] > cx && head[1] > cy - 0.1f * s);
    Lanes.arrow(U, false, cx, cy, s, line, head);
    yes("uturn goes round to the left where traffic keeps right", head[0] < cx && head[1] > cy);
    Lanes.arrow(U, true, cx, cy, s, line, head);
    yes("and to the right where it keeps left", head[0] > cx);
    Lanes.arrow(Lanes.MERGE_LEFT, false, cx, cy, s, line, head);
    yes("merge left comes in from the right", line[0] > cx && head[1] < cy);
    Lanes.arrow(Lanes.MERGE_RIGHT, false, cx, cy, s, line, head);
    yes("merge right comes in from the left", line[0] < cx);

    if (fails > 0) {
      System.out.println(fails + " failed");
      System.exit(1);
    }
    System.out.println("LanesTest ok");
  }
}
