package com.starpilot.aaroad;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;

// Desktop check of Events, the map's rolling show/hide log on the head unit.
//   javac -d /tmp/events src/com/starpilot/aaroad/Events.java test/EventsTest.java
//   java -cp /tmp/events com.starpilot.aaroad.EventsTest
// Fails (exit 1) unless: lines append in order; at MAX_BYTES the file moves to events.log.1 and a new one starts,
// one older copy only, so the two never pass about 2 x MAX_BYTES; record() writes timestamped lines on its own
// thread; and an unwritable first directory falls back to the next.
public class EventsTest {
  static int fails = 0;

  static void check(boolean ok, String what) {
    if (!ok) {
      fails++;
      System.out.println("FAIL " + what);
    }
  }

  static String last(File f) throws Exception {
    BufferedReader r = new BufferedReader(new FileReader(f));
    String l, out = null;
    while ((l = r.readLine()) != null) out = l;
    r.close();
    return out;
  }

  public static void main(String[] a) throws Exception {
    File dir = new File(System.getProperty("java.io.tmpdir"), "aaroad-events-test-" + System.nanoTime());
    check(dir.mkdirs(), "temp dir");
    File f = new File(dir, Events.NAME), old = new File(dir, Events.NAME + ".1");

    check(Events.append(f, "one", 100) && Events.append(f, "two", 100), "append");
    check(f.length() == 8 && !old.exists(), "two lines, no rotation yet");

    // a gl parts line is ~200 bytes; 3000 of them at the real cap
    String line = "aaroad: gl parts 30 s: frames 900 tiles 41 roads 3.2 ms labels 0.8 ms swap 4.1 ms "
        + "upload 0.3 ms total 9.0 ms | p50 8 p90 12 p99 21 max 34 slow 2 skipped 0 xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx";
    for (int i = 0; i < 3000; i++) check(Events.append(f, line + " " + i, Events.MAX_BYTES), "append " + i);
    check(f.length() <= Events.MAX_BYTES && old.exists() && old.length() <= Events.MAX_BYTES,
        "rotated at the cap: " + f.length() + " / " + old.length());
    check(f.length() + old.length() > Events.MAX_BYTES, "the older copy is kept");
    check(last(f).endsWith(" 2999"), "newest line in the live file");
    File[] all = dir.listFiles();
    check(all != null && all.length == 2, "one older copy only");

    // record(): on its own thread, timestamped, to the first directory it can write (a file in the way first)
    File blocker = new File(dir, "blocked");
    check(blocker.createNewFile(), "blocker");
    File good = new File(dir, "good");
    Events.dirs = new File[] {blocker, good};
    Events.record("aaroad: hidden reason=page not shown");
    File g = new File(good, Events.NAME);
    for (int i = 0; i < 100 && Events.where() == null; i++) Thread.sleep(20);
    check(g.equals(Events.where()), "falls back to the writable dir: " + Events.where());
    String l = g.exists() ? last(g) : "";
    check(l.matches("\\d{4}-\\d\\d-\\d\\d \\d\\d:\\d\\d:\\d\\d\\.\\d{3} aaroad: hidden reason=page not shown"),
        "timestamped line: " + l);

    if (fails > 0) {
      System.out.println(fails + " failed");
      System.exit(1);
    }
    System.out.println("EventsTest ok");
  }
}
