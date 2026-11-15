package com.starpilot.aaroad;

import java.util.HashSet;
import java.util.Set;

// Desktop check of Turn: the arrow file for the Mapbox types that used to fall back to the straight arrow, the
// roundabout and U-turn predicates, and the U-turn text. (test_dash_turn_card.py checks Turn.icon against Python.)
//   javac -d /tmp/turn src/com/starpilot/aaroad/Turn.java test/TurnTest.java && java -cp /tmp/turn com.starpilot.aaroad.TurnTest
public class TurnTest {
  static int bad;

  static void check(boolean ok, String what) {
    System.out.println((ok ? "ok   " : "FAIL ") + what);
    if (!ok) bad++;
  }

  public static void main(String[] a) {
    Set<String> have = new HashSet<String>();
    for (String f : new String[] {"direction_turn_left.png", "direction_turn_sharp_right.png", "direction_turn_straight.png",
        "direction_roundabout.png", "direction_roundabout_right.png", "direction_roundabout_slight_left.png", "direction_uturn.png"}) {
      have.add(f);
    }
    check(Turn.icon("roundabout turn", "left", have).equals("direction_turn_left.png"), "roundabout turn left is the left turn");
    check(Turn.icon("roundabout turn", "sharp right", have).equals("direction_turn_sharp_right.png"), "roundabout turn sharp right");
    check(Turn.icon("exit roundabout", "right", have).equals("direction_roundabout_right.png"), "exit roundabout right is the roundabout");
    check(Turn.icon("exit rotary", "slight left", have).equals("direction_roundabout_slight_left.png"), "exit rotary slight left");
    check(Turn.icon("exit roundabout", "", have).equals("direction_roundabout.png"), "exit roundabout, no modifier");
    check(Turn.icon("exit rotary", null, have).equals("direction_roundabout.png"), "exit rotary, null modifier");
    check(Turn.icon("rotary", "", have).equals("direction_roundabout.png"), "rotary still the roundabout");
    check(Turn.icon("turn", "uturn", have).equals("direction_uturn.png"), "uturn modifier");
    check(Turn.icon("nonsense", "left", have).equals(Turn.FALLBACK), "unknown type still falls back");

    for (String t : new String[] {"roundabout", "rotary", "roundabout turn", "exit roundabout", "exit rotary"}) check(Turn.isRoundabout(t), t + " is a roundabout");
    check(!Turn.isRoundabout("turn") && !Turn.isRoundabout(null) && !Turn.isRoundabout("fork"), "others are not");
    check(Turn.isUturn("turn", "uturn") && !Turn.isUturn("turn", "left") && !Turn.isUturn("turn", null), "uturn by modifier");
    check(Turn.uturnText(null).equals("Make a U-turn") && Turn.uturnText("  ").equals("Make a U-turn"), "U-turn text alone");
    check(Turn.uturnText(" Main   St ").equals("Make a U-turn onto Main St"), "U-turn text with the road, spaces collapsed");

    System.out.println(bad == 0 ? "all ok" : bad + " FAILED");
    if (bad != 0) System.exit(1);
  }
}
