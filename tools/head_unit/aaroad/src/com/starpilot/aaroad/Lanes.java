package com.starpilot.aaroad;

// Lane guidance as Google Maps shows it: a row of the lanes before the next maneuver, left to right, each with the
// arrows painted on it, the ones to take in white and the rest dim (LaneStrip draws it). The comma sends them from
// Mapbox (route_engine.lanes) in the turn step's "lanes". Display only.
//
// No Android here, so test/LanesTest.java runs on a desktop JVM: which arrow lights, when the row shows, and the
// arrows' shapes as points.
final class Lanes {
  // one bit per arrow, as the comma sends them (Navigation.Instruction.Lane)
  static final int STRAIGHT = 1, SLIGHT_RIGHT = 2, RIGHT = 4, SHARP_RIGHT = 8, UTURN = 16, SHARP_LEFT = 32, LEFT = 64,
      SLIGHT_LEFT = 128, MERGE_LEFT = 256, MERGE_RIGHT = 512, ALL = 1023;
  static final int MAX = 12;  // the comma sends no more
  static final double SHOW_M = 1000;  // the row shows from 1 km before the maneuver, as Google Maps does

  // Where the row goes, Peter's two choices (2026-11-01): under the turn card, or under the car with the map lifted
  // and zoomed in a little to make room. AT is the one that ships; not final so the render harness can draw both.
  static final int AT_CARD = 0, AT_CAR = 1;
  static int at = AT_CARD;

  private Lanes() {
  }

  // Whether the row shows: there are lanes, and the maneuver is known to be within SHOW_M.
  static boolean shown(int[] lanes, double meters) {
    return lanes != null && lanes.length > 0 && meters >= 0 && meters <= SHOW_M;
  }

  // The one arrow lit in a lane, 0 for a lane not to take. The comma's arrow when the lane carries it; else the one
  // nearest the maneuver's way (modifier as the comma spells it, "slightRight"); else the lane's first.
  static int lit(int arrows, boolean take, int arrow, String modifier) {
    arrows &= ALL;
    if (!take || arrows == 0) return 0;
    if (arrow != 0 && (arrow & (arrow - 1)) == 0 && (arrow & arrows) != 0) return arrow;
    int want = bit(modifier);
    if ((want & arrows) != 0) return want;
    return Integer.lowestOneBit(arrows);
  }

  // The arrow for a maneuver modifier ("sharpLeft", "slight right", "uturn", ...); straight when unknown.
  static int bit(String modifier) {
    if (modifier == null) return STRAIGHT;
    String m = modifier.toLowerCase().replace(" ", "").replace("_", "");
    if (m.equals("uturn")) return UTURN;
    if (m.equals("sharpright")) return SHARP_RIGHT;
    if (m.equals("right")) return RIGHT;
    if (m.equals("slightright")) return SLIGHT_RIGHT;
    if (m.equals("sharpleft")) return SHARP_LEFT;
    if (m.equals("left")) return LEFT;
    if (m.equals("slightleft")) return SLIGHT_LEFT;
    return STRAIGHT;
  }

  // The arrows of a lane in the order they are drawn: the dim ones first, the lit one last, on top.
  static int order(int arrows, int lit, int[] out) {
    int n = 0;
    for (int b = 1; b <= MERGE_RIGHT; b <<= 1) {
      if ((arrows & b) != 0 && b != lit) out[n++] = b;
    }
    if (lit != 0 && (arrows & lit) != 0) out[n++] = lit;
    return n;
  }

  // The width of one lane's cell when n of them share w px, no wider than most.
  static float cell(int n, float w, float most) {
    return n <= 0 ? 0 : Math.min(most, w / n);
  }

  // One arrow of size s centred on (cx, cy), y down: its shaft as a polyline into line (x, y pairs, returns the point
  // count) and its head, a triangle, into head (6 floats). All of a lane's arrows leave the same foot at the bottom
  // centre. A U-turn goes round to the left, to the right where traffic drives on the left.
  static int arrow(int bit, boolean leftDriving, float cx, float cy, float s, float[] line, float[] head) {
    float footY = cy + 0.45f * s;
    int n = 0;
    line[n++] = cx;
    line[n++] = footY;
    float tx, ty, dx, dy;  // where the head's tip is and the way it points
    if (bit == UTURN) {
      float side = leftDriving ? 1 : -1, r = 0.16f * s, top = cy - 0.12f * s, ox = cx + side * r;
      line[n++] = cx;
      line[n++] = top;
      for (int i = 1; i <= 8; i++) {  // over the top, a half circle
        double a = Math.PI * i / 8;
        line[n++] = (float) (ox - side * r * Math.cos(a));
        line[n++] = (float) (top - r * Math.sin(a));
      }
      tx = cx + side * 2 * r;
      ty = cy + 0.22f * s;
      dx = 0;
      dy = 1;
    } else if (bit == MERGE_LEFT || bit == MERGE_RIGHT) {
      // from a foot off to the side, into the middle and on up
      float side = bit == MERGE_LEFT ? 1 : -1;
      n = 0;
      line[n++] = cx + side * 0.26f * s;
      line[n++] = footY;
      for (int i = 1; i <= 6; i++) {
        float t = i / 6f;
        line[n++] = cx + side * 0.26f * s * (1 - t) * (1 - t);
        line[n++] = footY - t * 0.5f * s;
      }
      tx = cx;
      ty = cy - 0.45f * s;
      dx = 0;
      dy = -1;
    } else if (bit == STRAIGHT || bit == 0) {
      tx = cx;
      ty = cy - 0.45f * s;
      dx = 0;
      dy = -1;
    } else {
      // up to the bend, then off at the turn's angle: 45 slight, 90 turn, 135 sharp
      double deg = bit == SLIGHT_RIGHT || bit == SLIGHT_LEFT ? 45 : bit == RIGHT || bit == LEFT ? 90 : 135;
      float side = bit == SLIGHT_RIGHT || bit == RIGHT || bit == SHARP_RIGHT ? 1 : -1;
      float bendY = cy + (deg == 45 ? 0.02f : deg == 90 ? -0.02f : -0.1f) * s;
      float arm = (deg == 45 ? 0.42f : deg == 90 ? 0.34f : 0.3f) * s;
      line[n++] = cx;
      line[n++] = bendY;
      dx = (float) (side * Math.sin(Math.toRadians(deg)));
      dy = (float) -Math.cos(Math.toRadians(deg));
      tx = cx + dx * arm;
      ty = bendY + dy * arm;
    }
    // the shaft stops inside the head so its round cap does not poke out past the tip
    float hl = 0.24f * s, hw = 0.2f * s;
    line[n++] = tx - dx * hl * 0.7f;
    line[n++] = ty - dy * hl * 0.7f;
    float bx = tx - dx * hl, by = ty - dy * hl;
    head[0] = tx;
    head[1] = ty;
    head[2] = bx - dy * hw;
    head[3] = by + dx * hw;
    head[4] = bx + dy * hw;
    head[5] = by - dx * hw;
    return n / 2;
  }

  // Room for arrow()'s points.
  static final int POINTS = 24;
}
