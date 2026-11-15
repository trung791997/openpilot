package com.starpilot.aaroad;

// Desktop check of Blink: first flash starts lit, 50% duty at 1.5 Hz, repeats every period, a clock before the start.
//   javac -d /tmp/blinktest src/com/starpilot/aaroad/Blink.java test/BlinkTest.java
//   java -cp /tmp/blinktest com.starpilot.aaroad.BlinkTest
// Exits 1 on the first failed check.
public class BlinkTest {
  static int fails;

  static void check(String what, boolean ok) {
    System.out.println((ok ? "ok   " : "FAIL ") + what);
    if (!ok) fails++;
  }

  public static void main(String[] a) {
    long t0 = 5000;
    check("lit the instant the signal comes on", Blink.lit(t0, t0));
    check("lit just before half a period", Blink.lit(t0, t0 + 332));
    check("dark from half a period", !Blink.lit(t0, t0 + 333));
    check("dark just before the period ends", !Blink.lit(t0, t0 + 665));
    check("lit again at one period", Blink.lit(t0, t0 + 666));
    check("lit in the second period's first half", Blink.lit(t0, t0 + 666 + 332));
    check("dark in the second period's second half", !Blink.lit(t0, t0 + 666 + 400));
    check("a clock before the start counts as lit", Blink.lit(t0, t0 - 10));
    int on = 0, n = 6660;
    for (int i = 0; i < n; i++) if (Blink.lit(t0, t0 + i)) on++;
    check("50% duty over ten periods (" + on + "/" + n + ")", on == n / 2);
    check("1.5 Hz: period 666 ms is 90 flashes a minute", Blink.PERIOD_MS == 666 && 60000 / Blink.PERIOD_MS == 90);
    long late = 123456789L;
    check("phase is relative to the start, any start", Blink.lit(late, late + 10) && !Blink.lit(late, late + 400));
    if (fails > 0) {
      System.out.println(fails + " failed");
      System.exit(1);
    }
    System.out.println("all passed");
  }
}
