package com.starpilot.aaroad;

import java.io.File;
import java.lang.reflect.Constructor;
import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.nio.file.Files;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Date;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.TimeZone;
import java.util.TreeMap;

// Desktop replay of a real drive through the dash map's scene build and the GPU layer's cull (2026-11-08, for the
// I-35 North slowdown: cull 1-8 ms -> 15-21 ms at ~13:32). It runs the app's own MapView.frame() and
// rebuildIfNeeded() on an instance made without its constructor, the app's MapView.build() on real road tiles, and
// the app's GlCull on the scene the way GlRoads.packLoop builds it, at 20 Hz along a recorded GPS track.
//
// Not built or run by build.sh; see /tmp/hu/bench for the exporter and the android stubs it needs:
//   /tmp/hu/bench/build_bench.sh
//   java [-Xint] -cp /tmp/hu/bench/benchcls:/tmp/hu/bench/app:/tmp/hu/bench/stubs:/tmp/hu/api-82.jar:<android.jar> \
//     com.starpilot.aaroad.MapReplayBench track.csv route.csv tilesDirs navStartMs mode warmPasses
//
// What is NOT the Tegra: the numbers are desktop JVM milliseconds (Xint is the closest to Dalvik's cost per
// bytecode, still only a proxy). Draw calls and quads are counted from the passes drawFrame makes (PASSES below,
// read off GlRoads.drawFrame); they are the proxy for the GPU's work, not a GPU measurement.
public final class MapReplayBench {
  static final double DT = 0.05;                         // 20 Hz
  static final int ROUTE = 2 * MapView.DIM_OFF, LEVELS = 4, MAX_QUADS = 16384;
  static final long MIN = 60000;

  static double[] T, LA, LO, BR, SP;
  static final TimeZone TZ = TimeZone.getTimeZone("America/Chicago");

  public static void main(String[] a) throws Exception {
    String trackF = a[0], routeF = a[1];
    String[] tileDirs = a[2].split(",");
    long navStart = Long.parseLong(a[3]);
    String mode = a[4];
    int warm = Integer.parseInt(a[5]);
    long limitMin = a.length > 6 ? Long.parseLong(a[6]) : Long.MAX_VALUE;  // for quick runs: minutes to replay
    loadTrack(trackF);
    Feed.Route route = loadRoute(routeF);
    for (int p = 0; p < warm; p++) {
      System.err.println("warm pass " + (p + 1));
      run(route, tileDirs, navStart, mode, limitMin, false);
    }
    run(route, tileDirs, navStart, mode, limitMin, true);
  }

  static void loadTrack(String f) throws Exception {
    List<String> lines = Files.readAllLines(new File(f).toPath());
    int n = lines.size() - 1;
    T = new double[n];
    LA = new double[n];
    LO = new double[n];
    BR = new double[n];
    SP = new double[n];
    double prev = 0;
    for (int i = 0; i < n; i++) {
      String[] c = lines.get(i + 1).split(",");
      T[i] = Double.parseDouble(c[1]);
      LA[i] = Double.parseDouble(c[2]);
      LO[i] = Double.parseDouble(c[3]);
      double b = Double.parseDouble(c[4]);
      if (i > 0) {  // unwrap so the interpolation takes the short way round
        while (b - prev > 180) b -= 360;
        while (b - prev < -180) b += 360;
      }
      prev = b;
      BR[i] = b;
      SP[i] = Double.parseDouble(c[5]);
    }
  }

  static Feed.Route loadRoute(String f) throws Exception {
    List<String> lines = Files.readAllLines(new File(f).toPath());
    int n = lines.size();
    double[] la = new double[n], lo = new double[n];
    byte[] lv = new byte[n];
    for (int i = 0; i < n; i++) {
      String[] c = lines.get(i).split(",");
      la[i] = Double.parseDouble(c[0]);
      lo[i] = Double.parseDouble(c[1]);
      lv[i] = (byte) Integer.parseInt(c[2].trim());
    }
    return new Feed.Route(1, la, lo, lv);
  }

  static Object get(Object o, Class<?> c, String name) throws Exception {
    Field f = c.getDeclaredField(name);
    f.setAccessible(true);
    return f.get(o);
  }

  static void set(Object o, Class<?> c, String name, Object v) throws Exception {
    Field f = c.getDeclaredField(name);
    f.setAccessible(true);
    f.set(o, v);
  }

  // MapView without its constructor (no View, no Context): its frame() and rebuildIfNeeded() only need these fields
  static MapView newView() throws Exception {
    Field uf = Class.forName("sun.misc.Unsafe").getDeclaredField("theUnsafe");
    uf.setAccessible(true);
    Object u = uf.get(null);
    Method alloc = u.getClass().getMethod("allocateInstance", Class.class);
    MapView v = (MapView) alloc.invoke(u, MapView.class);
    Class<?> c = MapView.class;
    set(v, c, "latest", new MapView.Frame());
    set(v, c, "tickFrame", new MapView.Frame());
    set(v, c, "drawFrame", new MapView.Frame());
    set(v, c, "frameW", new double[2]);
    set(v, c, "smW", new double[2]);
    set(v, c, "tickW", new double[2]);
    set(v, c, "frameEN", new float[2]);
    set(v, c, "smEN", new float[2]);
    set(v, c, "snap", new Snap());  // field initialisers do not run on an allocateInstance view
    set(v, c, "tickEN", new float[2]);
    set(v, c, "building", new java.util.concurrent.atomic.AtomicBoolean());
    set(v, c, "k", 600 / MapView.R0);
    set(v, c, "heldH", Double.NaN);
    set(v, c, "lastSpd", -1.0);
    return v;
  }

  // ---- tiles: the comma sends the 3 x 3 around the car, nearest first, when the car enters a tile

  static int missing;
  static final java.util.HashSet<String> missingKeys = new java.util.HashSet<String>();

  static void tiles(double lat, double lon, String[] dirs, int[] last) throws Exception {
    double[] w = Feed.world(lat, lon);
    int tx = (int) Math.floor(w[0]), ty = (int) Math.floor(w[1]);
    if (tx == last[0] && ty == last[1]) return;
    last[0] = tx;
    last[1] = ty;
    List<int[]> want = new ArrayList<int[]>();
    for (int dx = -1; dx <= 1; dx++) for (int dy = -1; dy <= 1; dy++) want.add(new int[] {tx + dx, ty + dy});
    Collections.sort(want, new java.util.Comparator<int[]>() {
      public int compare(int[] p, int[] q) {
        return Math.abs(p[0] - tx0) + Math.abs(p[1] - ty0) - Math.abs(q[0] - tx0) - Math.abs(q[1] - ty0);
      }
    });
    Method evict = Feed.class.getDeclaredMethod("evict", Feed.Tile.class);
    evict.setAccessible(true);
    for (int[] t : want) {
      String key = t[0] + "/" + t[1];
      if (Feed.tiles.containsKey(key)) continue;
      File f = null;
      for (String d : dirs) {
        File c = new File(d + "/14/" + t[0] + "/" + t[1] + ".rt");
        if (c.exists()) {
          f = c;
          break;
        }
      }
      if (f == null) {
        if (missingKeys.add(key)) missing++;
        continue;
      }
      Feed.Tile tile = Feed.decode(t[0], t[1], Files.readAllBytes(f.toPath()));
      Feed.tiles.put(key, tile);
      evict.invoke(null, tile);
      Feed.tilesVersion++;
    }
  }

  static int tx0, ty0;  // the car's tile for the comparator above (set before sorting)

  // ---- the draw calls and quads drawFrame makes for a cull result (mirrors GlRoads.drawFrame's pass list)

  // passes over group g: ground class 2 (kerb, surface), a bridge layer 3 (casing, kerb, surface; not at all once the
  // flat bridges fade out zoomed in), a route group 3 (kerb, surface, carpet)
  static int passes(int g, boolean bridges, boolean carpet) {
    if (g >= ROUTE) return carpet ? 3 : 2;
    int ly = (g % MapView.DIM_OFF) / MapView.BRIDGE;
    if (ly == 0) return 2;
    return bridges ? 3 : 0;
  }

  // {draw calls, quads, attribute-pointer sets, GL calls the old per-run scheme would make}: GlRoads.lines() draws
  // each run from an offset in the index buffer and sets the pointers once a page of MAX_QUADS; the old scheme
  // set both pointers and drew, three GL calls, for every run
  static long[] drawStats(GlCull g, boolean bridges, boolean carpet) {
    long[] st = new long[4];
    int[] page = {-1};
    int b = MapView.BRIDGE;
    int[] ord = MapView.DRAW_ORDER;
    for (int c : ord) pass(g, c, st, page);
    for (int l = 0; l < 2 * LEVELS; l++) pass(g, ROUTE + l, st, page);
    for (int l = 0; l < 2 * LEVELS; l++) pass(g, ROUTE + l, st, page);
    for (int c : ord) pass(g, c, st, page);
    for (int c : ord) pass(g, c + MapView.DIM_OFF, st, page);
    for (int c : ord) pass(g, c + MapView.DIM_OFF, st, page);
    if (carpet) for (int l = 0; l < LEVELS; l++) pass(g, ROUTE + LEVELS + l, st, page);
    if (bridges) {
      for (int ly = 1; ly < CloseUp.LAYERS; ly++) {
        for (int rep = 0; rep < 3; rep++) {
          for (int c : ord) pass(g, b * ly + c, st, page);
        }
        for (int rep = 0; rep < 3; rep++) {
          for (int c : ord) pass(g, b * ly + MapView.DIM_OFF + c, st, page);
        }
      }
    }
    page[0] = -1;
    if (carpet) for (int l = 0; l < LEVELS; l++) pass(g, ROUTE + l, st, page);
    return st;
  }

  static void pass(GlCull g, int gr, long[] st, int[] ptr) {
    for (int r = g.rFrom[gr]; r < g.rFrom[gr + 1]; r++) {
      st[3] += 3L * ((g.rLen[r] + MAX_QUADS - 1) / MAX_QUADS);
      for (int s = g.rStart[r], left = g.rLen[r]; left > 0; ) {
        int page = s / MAX_QUADS * MAX_QUADS;
        if (page != ptr[0]) {
          ptr[0] = page;
          st[2]++;
        }
        int q = Math.min(left, page + MAX_QUADS - s);
        st[0]++;
        st[1] += q;
        s += q;
        left -= q;
      }
    }
  }

  // ---- GlRoads.packLoop's construction of the cull (copied; keep in step with it)
  static GlCull makeCull(MapView.Scene s) {
    MapView.Chunks roads = s.route.length > 0 ? s.stubsChunks : s.roadsChunks;
    MapView.Chunks route = s.routeChunks;
    int[] rcls = new int[roads.cls.length];
    for (int i = 0; i < rcls.length; i++) {
      rcls[i] = roads.cls[i] + (i < roads.dim.length && roads.dim[i] != 0 ? MapView.DIM_OFF : 0);
    }
    int[] level = new int[route.cls.length];
    for (int i = 0; i < level.length; i++) {
      level[i] = route.cls[i] < 2 ? 0 : Math.min(LEVELS - 1, route.cls[i] - 1);
      if (route.under[i] != 0) level[i] += LEVELS;
    }
    return new GlCull(s, ROUTE, 2 * LEVELS, roads.pts, roads.box, rcls, roads.at, route.pts, route.box, level, route.at);
  }

  // ---- stats

  static final class Min {
    int n;
    double spd, ik, cullMs, maxMs, cand, runs, calls, quads, pieces, segs, ptr, glOld;
    int rebuilds, builds, nav;
    double buildMs, makeMs;
    final List<Float> ms = new ArrayList<Float>();
  }

  static final class Tot {
    long calls, quads, runs, frames, ptr, glOld;
    long[] groupRuns;
    long[] groupQuads;
    long[] classQuads = new long[6], denseClassQuads = new long[6];
    long dense;
  }

  static void run(Feed.Route route, String[] tileDirs, long navStart, String mode, long limitMin, boolean report) throws Exception {
    Feed.tiles.clear();
    Feed.tilesVersion = 0;
    Feed.route = null;
    Feed.turnMeters = -1;
    missing = 0;
    missingKeys.clear();
    MapView mv = newView();
    Class<?> mc = MapView.class;
    Method rebuild = mc.getDeclaredMethod("rebuildIfNeeded", long.class);
    rebuild.setAccessible(true);
    Method frameM = mc.getDeclaredMethod("frame", MapView.Scene.class, long.class);
    frameM.setAccessible(true);
    Field sceneF = mc.getDeclaredField("scene");
    sceneF.setAccessible(true);
    Field buildingF = mc.getDeclaredField("building");
    buildingF.setAccessible(true);

    TreeMap<Long, Min> mins = new TreeMap<Long, Min>();
    SimpleDateFormat fmt = new SimpleDateFormat("HH:mm");
    fmt.setTimeZone(TZ);
    int[] lastTile = {Integer.MIN_VALUE, Integer.MIN_VALUE};
    MapView.Scene cur = null;
    GlCull cull = null, ref = null;
    long violations = 0, checked = 0, tTicks = 0, tForced = 0, tFrames = 0;
    int builds = 0;
    double t0 = T[0], t1 = T[T.length - 1];
    long frames = (long) ((t1 - t0) / 1000.0 / DT);
    if (limitMin != Long.MAX_VALUE) frames = Math.min(frames, limitMin * 60 * 20);
    int seg = 0;
    Tot tot = new Tot();
    int prevRebuilds = 0;
    long cullStart = System.nanoTime();
    Tick tick = new Tick(mode);
    GlCull.MORTON = Boolean.parseBoolean(System.getProperty("morton", "true"));
    GlCull.GAP = Integer.getInteger("gap", 0);
    for (long fi = 0; fi < frames; fi++) {
      double wall = t0 + fi * DT * 1000.0;
      while (seg + 2 < T.length && T[seg + 1] <= wall) seg++;
      double f = Math.max(0, Math.min(1, (wall - T[seg]) / (T[seg + 1] - T[seg])));
      double lat = LA[seg] + (LA[seg + 1] - LA[seg]) * f, lon = LO[seg] + (LO[seg + 1] - LO[seg]) * f;
      double brg = BR[seg] + (BR[seg + 1] - BR[seg]) * f, spd = SP[seg] + (SP[seg + 1] - SP[seg]) * f;
      long now = (long) wall;
      Feed.lat = lat;
      Feed.lon = lon;
      Feed.hdg = ((brg % 360) + 360) % 360;
      Feed.spd = spd;
      Feed.poseAt = now;
      Feed.pose = new Feed.Pose(lat, lon, Feed.hdg, spd, now);
      boolean nav = wall >= navStart;
      Feed.route = nav ? route : null;
      double[] w0 = Feed.world(lat, lon);
      tx0 = (int) Math.floor(w0[0]);
      ty0 = (int) Math.floor(w0[1]);
      tiles(lat, lon, tileDirs, lastTile);

      long bt = System.nanoTime();
      MapView.Scene before = (MapView.Scene) sceneF.get(mv);
      rebuild.invoke(mv, now);
      while (((java.util.concurrent.atomic.AtomicBoolean) buildingF.get(mv)).get()) Thread.sleep(0, 200000);
      MapView.Scene s = (MapView.Scene) sceneF.get(mv);
      double buildMs = (System.nanoTime() - bt) / 1e6;
      if (s == null) continue;  // no tile yet
      Min m = mins.get((long) (wall / MIN));
      if (m == null) {
        m = new Min();
        mins.put((long) (wall / MIN), m);
      }
      double makeMs = 0;
      if (s != cur) {
        cur = s;
        builds++;
        m.builds++;
        m.buildMs += buildMs;
        long mt = System.nanoTime();
        if (cull != null) {
          tTicks += cull.ticks;
          tForced += cull.forced;
          tFrames += cull.frames;
        }
        cull = makeCull(s);
        ref = tick.needsRef() ? makeCull(s) : null;
        makeMs = (System.nanoTime() - mt) / 1e6;
        m.makeMs += makeMs;
        prevRebuilds = 0;
        tick.reset();
      }
      MapView.Frame fr = (MapView.Frame) frameM.invoke(mv, s, now);
      float ce = fr.ce, cn = fr.cn, ch = fr.ch, sh = fr.sh, ik = fr.ik;
      long ct = System.nanoTime();
      tick.cull(cull, now, ce, cn, ch, sh, ik);
      long dn = System.nanoTime() - ct;
      if (ref != null) {
        ref.cull(ce, cn, ch, sh, ik, MapView.BOX_X0, MapView.BOX_X1, MapView.BOX_Z0, MapView.BOX_Z1);
        checked++;
        if (!superset(cull, ref)) violations++;
      }
      boolean bridges = 1 - CloseUp.gate(MapView.R0 / ik, CloseUp.ROAD_SHOW, CloseUp.ROAD_HIDE) > 0;
      long[] ds = drawStats(cull, bridges, true);
      int runs = cull.rFrom[cull.groups];
      m.n++;
      m.spd += spd;
      m.ik += ik;
      m.cullMs += dn / 1e6;
      m.maxMs = Math.max(m.maxMs, dn / 1e6);
      m.ms.add((float) (dn / 1e6));
      m.cand += cull.candidateCount();
      m.runs += runs;
      m.calls += ds[0];
      m.quads += ds[1];
      m.ptr += ds[2];
      m.glOld += ds[3];
      m.pieces += cull.count;
      m.segs += cull.segs;
      m.rebuilds += cull.rebuilds - prevRebuilds;
      prevRebuilds = cull.rebuilds;
      if (nav) m.nav++;
      if (report) {
        tot.frames++;
        tot.calls += ds[0];
        tot.ptr += ds[2];
        tot.glOld += ds[3];
        tot.quads += ds[1];
        tot.runs += runs;
        // quads in the cull per road class (ground + dim + bridge layers) and for the route, summed over frames:
        // what dropping a class at a zoom (LOD) would remove from the draw; [0..4] class, [5] route; dense = ik < 0.15
        for (int gr = 0; gr < cull.groups; gr++) {
          long qn = 0;
          for (int r = cull.rFrom[gr]; r < cull.rFrom[gr + 1]; r++) qn += cull.rLen[r];
          int cl = gr >= ROUTE ? 5 : (gr % MapView.DIM_OFF) % MapView.BRIDGE;
          tot.classQuads[cl] += qn;
          if (ik < 0.15f) tot.denseClassQuads[cl] += qn;
        }
        if (ik < 0.15f) tot.dense++;
      }
    }
    if (!report) return;
    if (cull != null) {
      tTicks += cull.ticks;
      tForced += cull.forced;
      tFrames += cull.frames;
    }
    System.out.println("update(): calls=" + tFrames + " culls=" + tTicks + " early=" + tForced + " morton=" + GlCull.MORTON
        + " gap=" + GlCull.GAP);
    System.out.println("mode=" + mode + " frames=" + tot.frames + " scene builds=" + builds + " missing tiles=" + missing + " "
        + missingKeys + " replay wall s=" + (System.nanoTime() - cullStart) / 1e9);
    System.out.println("superset check: frames checked=" + checked + " violations=" + violations);
    System.out.println("min   mode  spd  ik    pieces segs   cand  runs  calls  quads   cull avg  p99   max ms  rebuilds builds(ms)  make ms  ptr  glOld");
    for (Map.Entry<Long, Min> e : mins.entrySet()) {
      Min m = e.getValue();
      if (m.n == 0) continue;
      Collections.sort(m.ms);
      float p99 = m.ms.get(Math.min(m.ms.size() - 1, (int) (m.ms.size() * 0.99)));
      System.out.println(String.format(Locale.US, "%s %-5s %4.1f %.3f %6.0f %7.0f %5.0f %5.0f %6.0f %7.0f  %7.3f %6.3f %7.2f %5d  %2d(%6.0f)  %6.0f  %4.0f %6.0f",
          fmt.format(new Date(e.getKey() * MIN)), m.nav * 2 > m.n ? "route" : "free", m.spd / m.n, m.ik / m.n, m.pieces / m.n,
          m.segs / m.n, m.cand / m.n, m.runs / m.n, m.calls / m.n, m.quads / m.n, m.cullMs / m.n, p99, m.maxMs, m.rebuilds,
          m.builds, m.buildMs, m.makeMs, m.ptr / m.n, m.glOld / m.n));
    }
    if (tot.frames > 0) {
      System.out.println(String.format(Locale.US, "drive mean per frame: draws=%.1f ptrsets=%.1f GLcalls(new)=%.1f GLcalls(old per-run scheme)=%.1f quads=%.0f runs=%.1f", (double) tot.calls / tot.frames,
          (double) tot.ptr / tot.frames, (double) (tot.calls + tot.ptr) / tot.frames, (double) tot.glOld / tot.frames,
          (double) tot.quads / tot.frames, (double) tot.runs / tot.frames));
    }
    int street = 0, lines = 0;
    for (String l : de.robv.android.xposed.XposedBridge.LINES) {
      lines++;
      if (l.contains("street \"")) street++;
    }
    if (tot.frames > 0) {
      StringBuilder cb = new StringBuilder("LOD view: quads per frame by class [DRAW_ORDER ids 0..4, route] whole drive:");
      for (int i = 0; i < 6; i++) cb.append(String.format(Locale.US, " %d=%.0f", i, (double) tot.classQuads[i] / tot.frames));
      cb.append(" | frames with ik<0.15: ").append(tot.dense);
      if (tot.dense > 0) for (int i = 0; i < 6; i++) cb.append(String.format(Locale.US, " %d=%.0f", i, (double) tot.denseClassQuads[i] / tot.dense));
      System.out.println(cb);
    }
    System.out.println("xposed log lines=" + lines + " street lines=" + street);
    if (violations != 0) System.exit(1);
  }

  // every run of the fresh cull lies inside one run of the stale one, group by group: pieces are culled in buffer
  // order and neighbours merged, so a fresh run whose pieces are all in the stale set lies inside a single stale run
  static boolean superset(GlCull stale, GlCull fresh) {
    for (int g = 0; g < fresh.groups; g++) {
      int j = stale.rFrom[g], je = stale.rFrom[g + 1];
      for (int i = fresh.rFrom[g]; i < fresh.rFrom[g + 1]; i++) {
        int s = fresh.rStart[i], e = s + fresh.rLen[i];
        while (j < je && stale.rStart[j] + stale.rLen[j] <= s) j++;
        if (j == je || stale.rStart[j] > s || stale.rStart[j] + stale.rLen[j] < e) return false;
      }
    }
    return true;
  }

  // how the bench culls: every frame (the app today), or the 10 Hz widened cull once GlCull has it
  static final class Tick {
    final String mode;

    Tick(String mode) {
      this.mode = mode;
    }

    int n;

    boolean needsRef() {
      return !mode.equals("frame");
    }

    void reset() {}

    void cull(GlCull g, long now, float ce, float cn, float ch, float sh, float ik) {
      if (mode.equals("control")) {  // negative control: a plain cull every other frame, no margin; must show violations
        if (n++ % 2 == 0) g.cull(ce, cn, ch, sh, ik, MapView.BOX_X0, MapView.BOX_X1, MapView.BOX_Z0, MapView.BOX_Z1);
      } else if (mode.equals("frame")) g.cull(ce, cn, ch, sh, ik, MapView.BOX_X0, MapView.BOX_X1, MapView.BOX_Z0, MapView.BOX_Z1);
      else g.update(now, ce, cn, ch, sh, ik, MapView.BOX_X0, MapView.BOX_X1, MapView.BOX_Z0, MapView.BOX_Z1);
    }
  }
}
