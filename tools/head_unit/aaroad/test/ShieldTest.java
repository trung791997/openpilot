package com.starpilot.aaroad;

// Desktop test for Shield (no Android):
//   javac -d /tmp/st src/com/starpilot/aaroad/Shield.java test/ShieldTest.java && java -cp /tmp/st com.starpilot.aaroad.ShieldTest
public final class ShieldTest {
  private static int fails;

  private static void eq(String what, Object got, Object want) {
    boolean ok = got == null ? want == null : got.equals(want);
    if (!ok) {
      fails++;
      System.out.println("FAIL " + what + ": got " + got + ", want " + want);
    }
  }

  private static void split(String text, String want) {
    eq("split(" + text + ")", Shield.split(text).toString(), want);
  }

  public static void main(String[] a) {
    // a whole name that is a route number
    eq("US 75", Shield.ref("US 75"), "US 75");
    eq("US-75", Shield.ref("US-75"), "US 75");
    eq("U.S. 75", Shield.ref("U.S. 75"), "US 75");
    eq("US Highway 75", Shield.ref("US Highway 75"), "US 75");
    eq("I-635", Shield.ref("I-635"), "I-635");
    eq("I 635", Shield.ref("I 635"), "I-635");
    eq("I-635 E", Shield.ref("I-635 E"), "I-635");
    eq("IH 35E", Shield.ref("IH 35E"), "I-35E");
    eq("I 35E", Shield.ref("I 35E"), "I-35E");
    eq("Interstate 30", Shield.ref("Interstate 30"), "I-30");
    eq("SH 121", Shield.ref("SH 121"), "SH 121");
    eq("TX 121", Shield.ref("TX 121"), "TX 121");
    eq("TX-121 North", Shield.ref("TX-121 North"), "TX 121");
    eq("State Highway 190", Shield.ref("State Highway 190"), "SH 190");
    eq("State Route 1", Shield.ref("State Route 1"), "SR 1");
    eq("Loop 12", Shield.ref("Loop 12"), "Loop 12");
    eq("Spur 366", Shield.ref("Spur 366"), "Spur 366");
    eq("FM 1709", Shield.ref("FM 1709"), "FM 1709");
    eq("Farm to Market Road 544", Shield.ref("Farm to Market Road 544"), "FM 544");
    eq("Beltway 8", Shield.ref("Beltway 8"), "Beltway 8");
    eq("I\\u2011635", Shield.ref("I‑635"), "I-635");
    // not route numbers
    eq("Main St", Shield.ref("Main St"), null);
    eq("US Highway", Shield.ref("US Highway"), null);
    eq("Interstate", Shield.ref("Interstate"), null);
    eq("Dallas North Tollway", Shield.ref("Dallas North Tollway"), null);
    eq("North Central Expressway", Shield.ref("North Central Expressway"), null);
    eq("Loop Road", Shield.ref("Loop Road"), null);
    eq("Interstate Drive", Shield.ref("Interstate Drive"), null);
    eq("US 75 Frontage Road", Shield.ref("US 75 Frontage Road"), null);
    eq("empty", Shield.ref(""), null);
    eq("null", Shield.ref(null), null);
    // a Mapbox ref list
    eq("firstRef list", Shield.firstRef("US 75; TX 289"), "US 75");
    eq("firstRef I", Shield.firstRef("I 635;US 75"), "I-635");
    eq("firstRef none", Shield.firstRef("Dallas North Tollway"), null);
    eq("firstRef empty", Shield.firstRef(""), null);
    // exit numbers
    eq("exit 34", Shield.exit("Exit 34 McDermott Drive"), "34");
    eq("exit 19A", Shield.exit("Exit 19A Belt Line Rd"), "19A");
    eq("exit none", Shield.exit("Turn right onto Main St"), null);
    eq("exit word", Shield.exit("Exit toward Garland"), null);
    eq("exit later", Shield.exit("Take the Exit 5 ramp"), null);
    // whole instructions
    split("Exit 34 McDermott Drive", "[[exit 34], McDermott Drive]");
    split("Exit 19A Belt Line Rd", "[[exit 19A], Belt Line Rd]");
    split("Exit 19A I-635 East / Lyndon B Johnson Freeway toward Garland",
        "[[exit 19A], [I-635], East / Lyndon B Johnson Freeway toward Garland]");
    split("Keep left onto US 75 South", "[Keep left onto, [US 75], South]");
    split("Merge onto I 35E N", "[Merge onto, [I-35E], N]");
    split("Take the ramp onto SH 121/Sam Rayburn Tollway", "[Take the ramp onto, [SH 121], /Sam Rayburn Tollway]");
    split("Turn left onto State Highway 190", "[Turn left onto, [SH 190]]");
    split("Continue on Loop 12 toward Spur 366", "[Continue on, [Loop 12], toward, [Spur 366]]");
    split("Turn right onto FM 1709", "[Turn right onto, [FM 1709]]");
    split("Turn right onto Main St", "[Turn right onto Main St]");
    split("Keep right toward Dallas North Tollway", "[Keep right toward Dallas North Tollway]");
    split("Continue on US Highway", "[Continue on US Highway]");
    split("Turn onto US-75", "[Turn onto, [US 75]]");
    split("Head north on I-635 E", "[Head north on, [I-635], E]");
    split("Turn right onto Interstate Drive", "[Turn right onto Interstate Drive]");
    split("Turn right onto I-635E-Express", "[Turn right onto I-635E-Express]");
    split("", "[]");
    if (fails > 0) {
      System.out.println(fails + " failed");
      System.exit(1);
    }
    System.out.println("ShieldTest: all passed");
  }
}
