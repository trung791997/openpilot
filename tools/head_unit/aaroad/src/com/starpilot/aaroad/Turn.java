package com.starpilot.aaroad;

import java.math.BigDecimal;
import java.math.RoundingMode;
import java.util.Set;

// What the turn card says, worked out exactly as the StarPilot Auto car screen works it out, so the dash and the car
// screen agree: the arrow's file (navigation_icons.icon_name), the distance (navigation_state.distance_text) and the
// instruction's text (navigation_state). Plain Java, no Android, so the Python tests can run it against the Python
// (openpilot/starpilot/system/starport/tests/test_dash_turn_card.py).
final class Turn {
  static final String FALLBACK = "direction_turn_straight.png";

  // icon_name: the file for a Mapbox maneuver type and modifier; have is the files there are (the manifest), and
  // anything else gets the straight arrow.
  static String icon(String type, String modifier, Set<String> have) {
    if (type == null) type = "";
    if (modifier == null) modifier = "";
    String kind = type.equals("rotary") ? "roundabout" : type.equals("new name") || type.equals("continue") ? "turn" : type;
    kind = kind.replace(' ', '_');
    String suffix = modifier.equals("slightLeft") ? "slight_left" : modifier.equals("slightRight") ? "slight_right"
        : modifier.equals("sharpLeft") ? "sharp_left" : modifier.equals("sharpRight") ? "sharp_right" : modifier;
    suffix = suffix.replace(' ', '_');
    String candidate = suffix.equals("uturn") ? "direction_uturn.png"
        : "direction_" + kind + (suffix.isEmpty() ? "" : "_" + suffix) + ".png";
    return have.contains(candidate) ? candidate : FALLBACK;
  }

  // distance_text: 10 ft steps to 100 ft, 50 ft steps to 1000 ft, then tenths of a mile under 10 mi; 25 m steps to
  // 500 m, 50 m steps to 1 km, then tenths of a km under 10 km. Python's round() and format() round half to even on
  // the double's exact value, so these do too (Math.round and String.format would round 2.5 steps and x.x5 up).
  static String distance(double meters, boolean metric) {
    if (metric) {
      if (meters < 1000) {
        int step = meters < 500 ? 25 : 50;
        return (long) Math.rint(meters / step) * step + " m";
      }
      double km = meters / 1000;
      return fixed(km, km >= 10 ? 0 : 1) + " km";
    }
    double feet = meters * 3.28084;
    if (feet < 1000) {
      int step = feet <= 100 ? 10 : 50;
      return (long) Math.rint(feet / step) * step + " ft";
    }
    double mi = feet / 5280;
    return fixed(mi, mi >= 10 ? 0 : 1) + " mi";
  }

  private static String fixed(double v, int places) {
    return new BigDecimal(v).setScale(places, RoundingMode.HALF_EVEN).toPlainString();
  }

  // The instruction as the car screen shows it: its words with the spaces between them collapsed, at most 240 chars.
  static String text(String t) {
    if (t == null) return "";
    StringBuilder b = new StringBuilder(t.length());
    for (String w : t.trim().split("\\s+")) {
      if (w.isEmpty()) continue;
      if (b.length() > 0) b.append(' ');
      b.append(w);
    }
    return b.length() > 240 ? b.substring(0, 240) : b.toString();
  }

  private Turn() {}
}
