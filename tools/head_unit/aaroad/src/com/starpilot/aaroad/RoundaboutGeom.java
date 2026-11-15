package com.starpilot.aaroad;

// The geometry of the turn card's roundabout picture (RoundaboutIcon draws it), kept free of Android so it can be
// tested on the desktop.
//
// Angles are in degrees, measured from the entry arm round the ring in the counter-clockwise direction as seen on the
// map: 0 (or 360) is the entry itself (a U-turn), 90 the first exit to the right of a car that entered counter-
// clockwise (US), 180 straight on, 270 to the left. That is the convention the nav `degrees` field is read in
// (assumed from the car screen's step data and the fallback table below; not checked against the left-hand
// traffic data). The picture puts the entry arm at the bottom, so an angle a is at (sin a, cos a) of the ring in
// screen coordinates (y down): 90 is on the right, 180 at the top, 270 on the left.
final class RoundaboutGeom {
  // The exit's angle: `degrees` when the comma sent one (1..360), else from the maneuver's modifier. Mapbox's
  // modifiers are Mapbox's own words ("slight right", "sharp left") or the camel-case ones navigation_icons also
  // accepts ("slightRight"). Missing or unknown: straight on.
  static double exitDegrees(int degrees, String modifier) {
    if (degrees >= 1 && degrees <= 360) return degrees;
    if (modifier == null) return 180;
    String m = modifier.replace(" ", "").toLowerCase(java.util.Locale.US);
    if (m.equals("sharpright")) return 45;
    if (m.equals("right")) return 90;
    if (m.equals("slightright")) return 135;
    if (m.equals("straight")) return 180;
    if (m.equals("slightleft")) return 225;
    if (m.equals("left")) return 270;
    if (m.equals("sharpleft")) return 315;
    if (m.equals("uturn")) return 360;
    return 180;
  }

  // The ring's point at angle a (degrees) on a ring of radius r about (cx, cy): out[0] = x, out[1] = y (down).
  static void at(double cx, double cy, double r, double a, float[] out) {
    double t = Math.toRadians(a);
    out[0] = (float) (cx + r * Math.sin(t));
    out[1] = (float) (cy + r * Math.cos(t));
  }

  // How far round the ring the car drives to the exit at angle `exit`: counter-clockwise it is the angle itself,
  // clockwise (left-hand traffic) the rest of the turn. Always in (0, 360].
  static double sweep(double exit, boolean clockwise) {
    double a = exit % 360;
    if (a <= 0) a += 360;
    return clockwise ? (a == 360 ? 360 : 360 - a) : a;
  }

  // The car's way round as ring angles, from the entry to the exit, every `step` degrees at most: the first angle is
  // 0 and the last is the exit's angle (counter-clockwise the angles go up, clockwise they go down). n is how many
  // were written; angles must hold at least 2 + 360 / step.
  static int path(double exit, boolean clockwise, double step, double[] angles) {
    double total = sweep(exit, clockwise);
    int n = Math.max(1, (int) Math.ceil(total / step));
    for (int i = 0; i <= n; i++) {
      double a = total * i / n;
      angles[i] = clockwise ? -a : a;
    }
    return n + 1;
  }

  // The arrowhead at tip (x, y) pointing along (dx, dy) (need not be unit): the tip, then the two base corners,
  // len back and half to either side. out holds 6 floats.
  static void head(float x, float y, float dx, float dy, float len, float half, float[] out) {
    float d = (float) Math.hypot(dx, dy);
    if (d <= 0) {
      dx = 0;
      dy = -1;
      d = 1;
    }
    dx /= d;
    dy /= d;
    float bx = x - dx * len, by = y - dy * len;
    out[0] = x;
    out[1] = y;
    out[2] = bx - dy * half;
    out[3] = by + dx * half;
    out[4] = bx + dy * half;
    out[5] = by - dx * half;
  }

  private RoundaboutGeom() {}
}
