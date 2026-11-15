package com.starpilot.aaroad;

import java.io.ByteArrayOutputStream;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Collections;
import java.util.zip.Deflater;

// Desktop check of TrafficLayer: the SPTQ payload (levels, points, deltas restarting per line), line_count 0 clearing a
// tile, expiry 15 min after receipt on an injected clock, unknown version ignored, and the width class of a line.
//   javac -d /tmp/tq src/com/starpilot/aaroad/TrafficLayer.java test/TrafficLayerTest.java
//   java -cp /tmp/tq com.starpilot.aaroad.TrafficLayerTest
// Exits 1 on a failure.
public class TrafficLayerTest {
  static int bad;
  static long now;

  static void check(boolean ok, String what) {
    System.out.println((ok ? "ok   " : "FAIL ") + what);
    if (!ok) bad++;
  }

  // a line is {level, x0, y0, x1, y1, ...} in absolute tile units; the deltas restart at (0, 0) for each line
  static byte[] payload(int version, int[]... lines) throws Exception {
    ByteArrayOutputStream body = new ByteArrayOutputStream();
    ByteBuffer c = ByteBuffer.allocate(2).order(ByteOrder.LITTLE_ENDIAN);
    c.putShort((short) lines.length);
    body.write(c.array());
    for (int[] l : lines) {
      int n = (l.length - 1) / 2;
      ByteBuffer b = ByteBuffer.allocate(3 + n * 8).order(ByteOrder.LITTLE_ENDIAN);
      b.put((byte) l[0]).putShort((short) n);
      int px = 0, py = 0;
      for (int i = 0; i < n; i++) {
        b.putInt(l[1 + 2 * i] - px).putInt(l[2 + 2 * i] - py);
        px = l[1 + 2 * i];
        py = l[2 + 2 * i];
      }
      body.write(b.array());
    }
    Deflater d = new Deflater();
    d.setInput(body.toByteArray());
    d.finish();
    byte[] buf = new byte[65536];
    int n = d.deflate(buf);
    d.end();
    ByteBuffer h = ByteBuffer.allocate(10 + n).order(ByteOrder.LITTLE_ENDIAN);
    h.put((byte) 'S').put((byte) 'P').put((byte) 'T').put((byte) 'Q').put((byte) version).put((byte) 0).putInt(1700000000);
    h.put(buf, 0, n);
    return h.array();
  }

  public static void main(String[] a) throws Exception {
    TrafficLayer.clock = new TrafficLayer.Clock() {
      @Override
      public long now() {
        return now;
      }
    };
    TrafficLayer.clear();

    byte[] two = payload(1, new int[] {2, 100, 200, 300, 250}, new int[] {4, 4000, 4000, 4090, 3900, 4095, 3800});
    TrafficLayer.Tile t = TrafficLayer.decode(3, 4, two, 5);
    check(t != null && t.level.length == 2, "two lines decoded");
    check(t.level[0] == 2 && t.level[1] == 4, "levels 2 and 4");
    check(t.pts[0].length == 4 && t.pts[0][0] == 100 && t.pts[0][1] == 200 && t.pts[0][2] == 300 && t.pts[0][3] == 250, "line 1 points");
    check(t.pts[1].length == 6 && t.pts[1][0] == 4000 && t.pts[1][1] == 4000 && t.pts[1][2] == 4090 && t.pts[1][3] == 3900
        && t.pts[1][4] == 4095 && t.pts[1][5] == 3800, "line 2 points: deltas restart per line, not chained from line 1");
    check(t.fetchedSec == 1700000000L, "fetched seconds");

    now = 1000;
    check(TrafficLayer.receive("3/4", two) && TrafficLayer.count() == 1, "receive stores a tile");
    int v = TrafficLayer.version;
    check(TrafficLayer.receive("3/4", payload(1, new int[] {3, 0, 0, 10, 10})) && TrafficLayer.count() == 1
        && TrafficLayer.snapshot().get(0).level[0] == 3, "a new message replaces the tile");
    check(TrafficLayer.version > v, "version bumps");

    check(!TrafficLayer.receive("9/9", payload(2, new int[] {3, 0, 0, 10, 10})) && TrafficLayer.count() == 1, "unknown version ignored");
    boolean threw = false;
    try {
      TrafficLayer.receive("9/9", new byte[] {'X', 'X', 'X', 'X', 1, 0, 0, 0, 0, 0, 0});
    } catch (Exception e) {
      threw = true;
    }
    check(threw && TrafficLayer.count() == 1, "bad magic rejected, store untouched");

    check(TrafficLayer.receive("3/4", payload(1)) && TrafficLayer.count() == 0, "line_count 0 clears the tile");
    check(!TrafficLayer.receive("3/4", payload(1)), "clearing an empty tile changes nothing");

    now = 0;
    TrafficLayer.receive("1/1", two);
    now = TrafficLayer.EXPIRY_MS - 1;
    TrafficLayer.sweep();
    check(TrafficLayer.count() == 1, "kept just under 15 min after receipt");
    now = TrafficLayer.EXPIRY_MS;
    int v2 = TrafficLayer.version;
    TrafficLayer.sweep();
    check(TrafficLayer.count() == 0 && TrafficLayer.version > v2, "dropped 15 min after receipt");
    now = 5000000;
    TrafficLayer.receive("1/1", two);  // its fetched time is 2023, but the receipt time rules
    TrafficLayer.sweep();
    check(TrafficLayer.count() == 1, "an old fetched time does not expire a fresh receipt");

    // geometry: mpt 4096 makes one tile unit one metre; the anchor is the top-left of tile (1, 1)
    TrafficLayer.Tile q = TrafficLayer.decode(1, 1, payload(1, new int[] {2, 0, 0, 100, 0}, new int[] {3, 0, 50, 100, 50}), 0);
    float[][] road = {{0, 0, 100, 0}, {0, -50, 100, -50}};
    float[][] box = {{0, 0, 100, 0}, {0, -50, 100, -50}};
    int[] cls = {0, 1 + TrafficLayer.BRIDGE};  // a motorway, and a class 1 on a deck
    TrafficLayer.Lines ln = TrafficLayer.lines(Collections.singletonList(q), 1, 1, 4096, road, box, cls);
    check(ln.runs.length == 2 && ln.runs[0][2] == 100f && ln.runs[0][3] == 0f && ln.runs[1][1] == -50f, "tile units to metres east/north of the anchor");
    check(ln.group[0] == 0 * TrafficLayer.CLASSES + 0, "moderate line on the motorway: group (moderate, class 0)");
    check(ln.group[1] == 1 * TrafficLayer.CLASSES + 1, "heavy line on a deck of class 1: group (heavy, class 1)");
    float[][] far = {{0, 1000, 100, 1000}};
    TrafficLayer.Lines lf = TrafficLayer.lines(Collections.singletonList(q), 1, 1, 4096, far, far, new int[] {0});
    check(lf.group[0] == TrafficLayer.FALLBACK_CLASS, "no road within range: street width");

    System.out.println(bad == 0 ? "all ok" : bad + " FAILED");
    if (bad != 0) System.exit(1);
  }
}
