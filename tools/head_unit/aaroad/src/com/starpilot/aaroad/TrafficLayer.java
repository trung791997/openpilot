package com.starpilot.aaroad;

import java.io.ByteArrayOutputStream;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.zip.Inflater;

// Live traffic on the roads around the car, with or without navigation (2026-11-15). The comma sends a "q" message
// per zoom-14 tile ("x/y", chunked like the road tile "m"); Feed reassembles it and hands the bytes here.
//
// Payload (all integers little-endian, as the road tile body): "SPTQ", u8 version (1), u8 reserved, u32 fetched unix
// seconds, then a zlib body: u16 line count; per line u8 level (2 moderate, 3 heavy, 4 severe), u16 point count and
// the points as Feed.decode reads a road line's: <i4 x, y deltas in tile units (0..4096 over the tile).
// A line count of 0 clears the tile. A tile is dropped EXPIRY_MS after it was received (the head unit's clock may be
// wrong, so the fetched time is not used). An unknown version is ignored.
//
// Thread safety as the road tiles: Feed's thread puts whole immutable Tiles into a concurrent map; the scene builder
// thread reads a snapshot. Nothing here runs per frame.
final class TrafficLayer {
  static final int EXPIRY_MS = 15 * 60 * 1000;
  static final int VERSION = 1;
  static final int LEVELS = 3;                 // moderate, heavy, severe: MapView.TRAFFIC[0..2]
  static final int CLASSES = 5;                // the road class a line is drawn at the width of
  static final int GROUPS = LEVELS * CLASSES;  // group = (level - 2) * CLASSES + class
  static final float WIDTH_K = 0.6f;           // of the road's drawn width
  static final double EXTENT = 4096;           // = Feed.EXTENT, here so the desktop test needs no Android
  static final int BRIDGE = 5;                 // = MapView.BRIDGE: a road class + BRIDGE per deck layer

  interface Clock {
    long now();
  }

  // monotonic milliseconds; a test swaps in its own
  static volatile Clock clock = new Clock() {
    @Override
    public long now() {
      return System.nanoTime() / 1000000L;
    }
  };

  static final class Tile {
    final int x, y;
    final long receivedMs;
    final int[] level;      // per line, 2..4
    final float[][] pts;    // per line, x, y pairs in tile units
    final long fetchedSec;

    Tile(int x, int y, long receivedMs, long fetchedSec, int[] level, float[][] pts) {
      this.x = x;
      this.y = y;
      this.receivedMs = receivedMs;
      this.fetchedSec = fetchedSec;
      this.level = level;
      this.pts = pts;
    }
  }

  private static final Map<String, Tile> tiles = new ConcurrentHashMap<String, Tile>();
  static volatile int version;  // bumped on every change, so the scene knows to rebuild

  // Decodes the payload; null for an unknown version (ignore the tile), an exception for a damaged one.
  static Tile decode(int x, int y, byte[] raw, long nowMs) throws Exception {
    if (raw.length < 10 || raw[0] != 'S' || raw[1] != 'P' || raw[2] != 'T' || raw[3] != 'Q') {
      throw new IllegalArgumentException("not a traffic tile");
    }
    if (raw[4] != VERSION) return null;
    long fetched = ByteBuffer.wrap(raw, 6, 4).order(ByteOrder.LITTLE_ENDIAN).getInt() & 0xffffffffL;
    Inflater inflater = new Inflater();
    ByteArrayOutputStream body = new ByteArrayOutputStream(raw.length * 4);
    try {
      inflater.setInput(raw, 10, raw.length - 10);
      byte[] buf = new byte[4096];
      while (!inflater.finished()) {
        int n = inflater.inflate(buf);
        if (n == 0 && (inflater.needsInput() || inflater.needsDictionary())) throw new IllegalArgumentException("truncated traffic tile");
        body.write(buf, 0, n);
        if (body.size() > 4 * 1024 * 1024) throw new IllegalArgumentException("oversized traffic tile");
      }
    } finally {
      inflater.end();
    }
    ByteBuffer b = ByteBuffer.wrap(body.toByteArray()).order(ByteOrder.LITTLE_ENDIAN);
    int count = b.getShort() & 0xffff;
    List<Integer> level = new ArrayList<Integer>();
    List<float[]> pts = new ArrayList<float[]>();
    for (int l = 0; l < count; l++) {
      int lv = b.get() & 0xff, n = b.getShort() & 0xffff;
      if ((long) n * 8 > b.remaining()) throw new IllegalArgumentException("bad traffic tile points");
      float[] p = new float[n * 2];
      int ax = 0, ay = 0;
      for (int i = 0; i < n; i++) {
        ax += b.getInt();
        ay += b.getInt();
        p[2 * i] = ax;
        p[2 * i + 1] = ay;
      }
      if (lv < 2 || lv > 4 || n < 2) continue;  // not a drawable line; keep the rest
      level.add(lv);
      pts.add(p);
    }
    int[] lv = new int[level.size()];
    for (int i = 0; i < lv.length; i++) lv[i] = level.get(i);
    return new Tile(x, y, nowMs, fetched, lv, pts.toArray(new float[0][]));
  }

  // Feed's thread: a complete message for tile key "x/y". Returns whether it changed anything.
  static boolean receive(String key, byte[] raw) throws Exception {
    String[] xy = key.split("/");
    Tile t = decode(Integer.parseInt(xy[0]), Integer.parseInt(xy[1]), raw, clock.now());
    if (t == null) return false;
    if (t.level.length == 0) {
      if (tiles.remove(key) == null) return false;
    } else {
      tiles.put(key, t);
    }
    version++;
    return true;
  }

  // Drops tiles older than EXPIRY_MS since receipt. Cheap; the scene tick calls it.
  static void sweep() {
    long now = clock.now();
    boolean any = false;
    for (Map.Entry<String, Tile> e : tiles.entrySet()) {
      if (now - e.getValue().receivedMs >= EXPIRY_MS) {
        tiles.remove(e.getKey());
        any = true;
      }
    }
    if (any) version++;
  }

  static List<Tile> snapshot() {
    return new ArrayList<Tile>(tiles.values());
  }

  static void clear() {
    tiles.clear();
    version++;
  }

  static int count() {
    return tiles.size();
  }

  // The scene's traffic: runs of e, n pairs in metres from the anchor (ax, ay in tile units, mpt metres per tile
  // unit), as MapView's roads; group[i] = (level - 2) * CLASSES + the class of the nearest road.
  static final class Lines {
    float[][] runs = new float[0][];
    int[] group = new int[0];
  }

  // roadRuns/roadBox/roadCls: the scene's roads in the same frame (class may carry + MapView.BRIDGE per deck layer).
  static Lines lines(List<Tile> from, double ax, double ay, double mpt, float[][] roadRuns, float[][] roadBox, int[] roadCls) {
    List<float[]> runs = new ArrayList<float[]>();
    List<Integer> group = new ArrayList<Integer>();
    for (Tile t : from) {
      for (int l = 0; l < t.pts.length; l++) {
        float[] p = t.pts[l], out = new float[p.length];
        for (int i = 0; i < p.length; i += 2) {
          out[i] = (float) ((t.x + p[i] / EXTENT - ax) * mpt);
          out[i + 1] = (float) (-(t.y + p[i + 1] / EXTENT - ay) * mpt);
        }
        runs.add(out);
        group.add((t.level[l] - 2) * CLASSES + nearestClass(out, roadRuns, roadBox, roadCls));
      }
    }
    Lines o = new Lines();
    o.runs = runs.toArray(new float[0][]);
    o.group = new int[group.size()];
    for (int i = 0; i < o.group.length; i++) o.group[i] = group.get(i);
    return o;
  }

  static final float NEAR_M = 20;
  static final int FALLBACK_CLASS = 2;  // a street

  // The class of the road closest to the run's points (at most 8 sampled), within NEAR_M; a street when none is.
  static int nearestClass(float[] run, float[][] roads, float[][] box, int[] cls) {
    int n = run.length / 2, step = Math.max(1, n / 8);
    double best = (double) NEAR_M * NEAR_M;
    int out = FALLBACK_CLASS;
    for (int i = 0; i < n; i += step) {
      float px = run[2 * i], py = run[2 * i + 1];
      for (int r = 0; r < roads.length; r++) {
        float[] b = box[r];
        if (px < b[0] - NEAR_M || px > b[2] + NEAR_M || py < b[1] - NEAR_M || py > b[3] + NEAR_M) continue;
        float[] q = roads[r];
        for (int j = 0; j + 3 < q.length; j += 2) {
          double d = segDist2(px, py, q[j], q[j + 1], q[j + 2], q[j + 3]);
          if (d < best) {
            best = d;
            out = cls[r] % BRIDGE;
          }
        }
      }
    }
    return out;
  }

  private static double segDist2(double px, double py, double ax, double ay, double bx, double by) {
    double dx = bx - ax, dy = by - ay, l2 = dx * dx + dy * dy;
    double t = l2 <= 0 ? 0 : Math.max(0, Math.min(1, ((px - ax) * dx + (py - ay) * dy) / l2));
    double ex = ax + t * dx - px, ey = ay + t * dy - py;
    return ex * ex + ey * ey;
  }

  private TrafficLayer() {}
}
