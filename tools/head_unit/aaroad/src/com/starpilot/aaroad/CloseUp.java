package com.starpilot.aaroad;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

// The zoomed-in look (Peter 2026-11-01, after Google Maps' immersive close-up, not extruded buildings): bridges raised
// over the roads they cross, with ramps at their ends, a dark side wall and a soft shadow on the ground; lane lines on
// motorways; thin sidewalks and dashed crosswalks; rail lines with cross ties (drawn under the roads, CloseView.rails).
// All of it fades in only while the map is zoomed in (gate below): cruising, the map draws exactly what it drew
// before, so its frame rate is untouched.
//
// This file is plain Java (no Android, no GL) so test/CloseUpTest.java can check it on the desktop. It turns a scene's
// lines (metres east, north of the scene's anchor) into segments in draw ranges. CloseView draws them in software,
// GlClose on the GPU, both the same way: a point h world metres up is drawn where the camera's ray through it meets
// the ground, so every shader and stroke stays a ground-plane one (lift()).
//
// Data: road_tiles format 5 (Feed.Tile layer, lanes, oneway, walk*), format 6 rails (Feed.Tile rail*). Mapbox has lane counts only in Japan, so in the
// US a motorway gets LANES_ONEWAY or LANES_TWOWAY.
final class CloseUp {
  // the camera, as MapView's CAM_H and CAM_ZC: height above the ground and the foot's z, in display metres
  static final float CAM_H = 150, CAM_ZC = -50;
  // world metres up per Mapbox layer: about 6 m a deck, times 4: twice for MapView.ROAD_SCALE, as the widths are, and
  // twice more because the camera sits about 90 m up at the closest zoom, where at 12 m a deck moved 5-7 px and did not
  // read as raised in the harness renders (2026-11-01); at 24 m it moves 12-15 px
  static final float LAYER_M = 6 * 4;
  // a bridge reaches its height this far from an end: along the road it meets there, or inside itself (at most a
  // third of it) when nothing meets it
  static final float RAMP_M = 60;
  // ends closer than this meet (tile edges cut lines; the two halves' ends land within a metre or two)
  static final float JOIN_M = 4;
  static final int LANES_ONEWAY = 3, LANES_TWOWAY = 4;
  static final float MOTORWAY_W = 56;  // MapView.ROAD_M[0]
  static final int WALK_CROSSING = 2;  // Feed.WALK_CROSSING, road_tiles.WALK_CROSSING
  // lane lines: a dash ON_M long every PERIOD_M (US 10 ft line, 30 ft gap is 3 m in 12 m; drawn a little longer so
  // a dash is more than a dot at the close-up zoom), and their half width in world metres
  static final float DASH_ON_M = 6, DASH_PERIOD_M = 18, CROSS_ON_M = 1.5f, CROSS_PERIOD_M = 3.5f;
  // what is kept: lines with a point this near the anchor (the scene is rebuilt every MapView.MOVE_R 600 m, and the
  // close-up view reaches about 700 m ahead), and at most this many segments, the nearest first
  static final float ROAD_R = 1400, WALK_R = 1000, RAIL_R = 1000;
  static final int ROAD_SEGS = 6144, WALK_SEGS = 4096, RAIL_SEGS = 3072;
  // a rail line's cross ties: one every TIE_M along it, TIE_HALF_M each side of it (a real tie is about 2.6 m long,
  // drawn twice that as the roads are, MapView.ROAD_SCALE; every 8 m is about one in thirteen, so they read as ties
  // at the close-up zoom instead of a solid band)
  static final float TIE_M = 8, TIE_HALF_M = 2.6f, RAIL_PIECE_M = 100;
  // the gate: shown fully at a displayed range up to SHOW m, gone from HIDE m, faded between (no flicker as the zoom
  // eases). Free drive's range is 12 s of road: roads and bridges show under about 40 mph, walks under about 24 mph;
  // with a route they show while the view closes in on a turn, an exit or a stop.
  static final float ROAD_SHOW = 220, ROAD_HIDE = 260, WALK_SHOW = 130, WALK_HIDE = 160;

  // per segment: a.e, a.n, b.e, b.n, height at a, at b (world m, before the gate), metres along its line at a
  static final int SEG = 7;
  static final int CORNER = 9;  // floats per corner of a segment's quad on the GPU (corners)
  // draw ranges: the walks, the crossings, then per layer group 0 (ground) to 3, the decks of each road class, and the
  // lane lines: dashed, solid white, solid yellow (a two-way road's centre, a one-way's left edge); last the rail
  // lines and their ties (after the rest so the ranges before them keep their numbers)
  static final int WALK = 0, CROSS = 1, LAYERS = 4, CLASSES = 5, DASH = CLASSES, EDGE = CLASSES + 1, YELLOW = CLASSES + 2;
  static final int PER_LAYER = CLASSES + 3, RAIL = 2 + LAYERS * PER_LAYER, TIE = RAIL + 1, RANGES = TIE + 1;

  static int range(int layer, int kind) {
    return 2 + layer * PER_LAYER + kind;
  }

  static final class Detail {
    float[] seg = new float[0];
    final int[] from = new int[RANGES + 1];  // range r is segments from[r] to from[r + 1]
    int roadSegs, walkSegs, railSegs, roadDropped, walkDropped, railDropped, bridges, ramps;
    float maxH;  // the highest point, world metres

    int count() {
      return from[RANGES];
    }

    boolean empty(int r) {
      return from[r] == from[r + 1];
    }
  }

  // The scene's lines, as MapView.build finds them in the tiles.
  static final class Lines {
    final List<float[]> pts = new ArrayList<float[]>();
    final List<int[]> attr = new ArrayList<int[]>();  // cls, bridge 0/1, layer, lanes, oneway 0/1
    final List<float[]> walks = new ArrayList<float[]>();
    final List<Integer> walkKind = new ArrayList<Integer>();
    final List<float[]> rails = new ArrayList<float[]>();

    void road(float[] p, int cls, boolean bridge, int layer, int lanes, boolean oneway) {
      if (p.length < 4) return;
      pts.add(p);
      attr.add(new int[] {cls, bridge ? 1 : 0, layer, lanes, oneway ? 1 : 0});
    }

    void walk(float[] p, int kind) {
      if (p.length < 4) return;
      walks.add(p);
      walkKind.add(kind);
    }

    void rail(float[] p) {
      if (p.length >= 4) rails.add(p);
    }
  }

  // 0 to 1: how much of the close-up look shows at a displayed range of rng m
  static float gate(double rng, float show, float hide) {
    if (rng <= show) return 1;
    if (rng >= hide) return 0;
    float t = (float) ((hide - rng) / (hide - show));
    return t * t * (3 - 2 * t);
  }

  // The camera's tilt (Peter's immersive-nav request): it looks down PITCH below the horizon cruising, and CLOSE_PITCH,
  // further toward the horizon, zoomed in. The tilt follows the displayed range, so it glides with the zoom's own
  // easing (MapView.frame): today's tilt from TILT_HIDE m out (the close-up look's ROAD_HIDE), all of it by TILT_SHOW.
  // The camera stays where it is (CAM_H, CAM_ZC, so lift() holds), and the focal length and centre row move with the
  // tilt to keep the car at CAR_Y and its px per metre across: only the road ahead looks more laid back. At 48 degrees
  // the top corners of the free-drive view show past FreeView's box above row 21, under the fog's 95% dark.
  static final double PITCH = Math.toRadians(55), CLOSE_PITCH = Math.toRadians(48);
  static final float TILT_SHOW = 180, TILT_HIDE = ROAD_HIDE;
  // MapView's lens at PITCH: focal length and the centre's column and row, px
  static final double FOCAL = 260, CX = 369, CY = 102.663;

  // the tilt at a displayed range of rng m
  static double pitch(double rng) {
    float g = gate(rng, TILT_SHOW, TILT_HIDE);
    return g <= 0 ? PITCH : PITCH + (CLOSE_PITCH - PITCH) * g;
  }

  // The camera at a tilt as a 3x3 from the car's frame (x, z, 1) to the screen, row-major as Matrix.setValues takes it,
  // into out. At PITCH exactly the values it always had.
  static float[] camera(double pitch, float[] out) {
    double c0p = CAM_H * Math.sin(PITCH) - CAM_ZC * Math.cos(PITCH);
    double cp = Math.cos(pitch), sp = Math.sin(pitch), c0 = CAM_H * sp - CAM_ZC * cp;
    double f = FOCAL * (c0 / c0p);  // the same px per metre across at the car
    double cy = CY + (FOCAL * (CAM_H * Math.cos(PITCH) + CAM_ZC * Math.sin(PITCH)) / c0p
                      - f * (CAM_H * cp + CAM_ZC * sp) / c0);  // the car on the same row
    double[] h = {f, CX * cp, CX * c0, 0, cy * cp - f * sp, cy * c0 + f * (CAM_H * cp + CAM_ZC * sp), 0, cp, c0};
    for (int i = 0; i < 9; i++) out[i] = (float) (h[i] / c0);
    return out;
  }

  // The ground point a point hd display metres up is drawn at: its car-frame (x, z) scaled about the camera's foot.
  // Also the factor widths scale by there.
  static float lift(float hd) {
    return CAM_H / Math.max(CAM_H - hd, 1f);
  }

  private static final class Seg {
    final float[] v = new float[SEG];
    final float d;  // from the anchor, for the cap
    final int range;

    Seg(float ae, float an, float be, float bn, float ha, float hb, float at, int range) {
      v[0] = ae;
      v[1] = an;
      v[2] = be;
      v[3] = bn;
      v[4] = ha;
      v[5] = hb;
      v[6] = at;
      this.range = range;
      d = (float) Math.min(Math.hypot(ae, an), Math.hypot(be, bn));
    }
  }

  static Detail build(Lines in) {
    Detail out = new Detail();
    // the roads near enough
    List<float[]> pts = new ArrayList<float[]>();
    List<int[]> attr = new ArrayList<int[]>();
    for (int i = 0; i < in.pts.size(); i++) {
      if (near(in.pts.get(i), ROAD_R)) {
        pts.add(in.pts.get(i));
        attr.add(in.attr.get(i));
      }
    }
    int n = pts.size();
    // every line's two ends in a grid, to find what meets each bridge end
    HashMap<Long, List<int[]>> ends = new HashMap<Long, List<int[]>>();
    for (int i = 0; i < n; i++) {
      float[] p = pts.get(i);
      for (int e = 0; e < 2; e++) {
        int j = e == 0 ? 0 : p.length - 2;
        long key = key(p[j], p[j + 1]);
        List<int[]> l = ends.get(key);
        if (l == null) ends.put(key, l = new ArrayList<int[]>());
        l.add(new int[] {i, e});
      }
    }
    // per line: the height it is raised to at each end (0: not), the layer group, and whether a bridge ramps inside
    float[][] endH = new float[n][2];
    boolean[][] self = new boolean[n][2];
    int[] group = new int[n];
    for (int i = 0; i < n; i++) {
      int[] a = attr.get(i);
      if (a[1] == 0) continue;
      int layer = Math.max(1, a[2]);
      float h = layer * LAYER_M;
      group[i] = Math.max(group[i], Math.min(LAYERS - 1, layer));
      out.bridges++;
      float[] p = pts.get(i);
      for (int e = 0; e < 2; e++) {
        int j = e == 0 ? 0 : p.length - 2;
        boolean cont = false, met = false;
        for (int[] o : meeting(ends, p[j], p[j + 1], pts, i)) {
          int[] oa = attr.get(o[0]);
          if (oa[1] != 0) {
            if (Math.max(1, oa[2]) == layer) cont = true;
            continue;
          }
          met = true;
          if (h > endH[o[0]][o[1]]) endH[o[0]][o[1]] = h;
          group[o[0]] = Math.max(group[o[0]], Math.min(LAYERS - 1, layer));
        }
        endH[i][e] = h;
        self[i][e] = !cont && !met;
      }
    }
    List<Seg> roads = new ArrayList<Seg>();
    for (int i = 0; i < n; i++) {
      int[] a = attr.get(i);
      boolean bridge = a[1] != 0, ramp = !bridge && (endH[i][0] > 0 || endH[i][1] > 0);
      if (!bridge && !ramp && a[0] != 0) continue;  // only motorways get lane lines on the ground
      if (ramp) out.ramps++;
      float[] p = pts.get(i);
      float[] h;
      if (bridge) {
        float len = length(p), rl = Math.min(RAMP_M, len / 3);
        p = cut(p, self[i][0] ? rl : -1, self[i][1] ? len - rl : -1);
        h = bridgeHeights(p, endH[i][0], self[i][0], rl, self[i][1]);
      } else if (ramp) {
        float len = length(p);
        p = cut(p, endH[i][0] > 0 ? Math.min(RAMP_M, len) : -1, endH[i][1] > 0 ? Math.max(0, len - RAMP_M) : -1);
        h = rampHeights(p, endH[i][0], endH[i][1]);
      } else {
        h = new float[p.length / 2];
      }
      // the raised stretch to its layer group, the rest on the ground: split where the height leaves 0
      for (int[] run : runs(h)) {
        boolean up = run[2] != 0;
        int g = up ? Math.max(1, group[i]) : 0;
        float[] rp = Arrays.copyOfRange(p, 2 * run[0], 2 * run[1] + 2);
        float[] rh = Arrays.copyOfRange(h, run[0], run[1] + 1);
        for (float v : rh) out.maxH = Math.max(out.maxH, v);
        if (up) for (int j = 0; j + 1 < rh.length; j++) {
          roads.add(new Seg(rp[2 * j], rp[2 * j + 1], rp[2 * j + 2], rp[2 * j + 3], rh[j], rh[j + 1], 0, range(g, a[0])));
        }
        if (a[0] == 0) lanes(rp, rh, a[3], a[4] != 0, g, roads);
      }
    }
    List<Seg> walks = new ArrayList<Seg>();
    for (int i = 0; i < in.walks.size(); i++) {
      float[] p = in.walks.get(i);
      if (!near(p, WALK_R)) continue;
      int r = in.walkKind.get(i) == WALK_CROSSING ? CROSS : WALK;
      float at = 0;
      for (int j = 0; j + 3 < p.length; j += 2) {
        walks.add(new Seg(p[j], p[j + 1], p[j + 2], p[j + 3], 0, 0, at, r));
        at += (float) Math.hypot(p[j + 2] - p[j], p[j + 3] - p[j + 1]);
      }
    }
    // the rail lines on the ground, every one (a rail bridge too: drawn under the roads, a road under it shows over
    // it), in pieces at most RAIL_PIECE_M long (a main line's points can be kilometres apart, so its ends say nothing
    // of how near it passes), the pieces within RAIL_R, and a tie across each every TIE_M
    List<Seg> rails = new ArrayList<Seg>();
    for (float[] p : in.rails) {
      float at = 0, next = TIE_M / 2;
      for (int j = 0; j + 3 < p.length; j += 2) {
        float dx = p[j + 2] - p[j], dy = p[j + 3] - p[j + 1], len = (float) Math.hypot(dx, dy);
        int pieces = Math.max(1, (int) Math.ceil(len / RAIL_PIECE_M));
        for (int k = 0; k < pieces; k++) {
          float u0 = (float) k / pieces, u1 = (float) (k + 1) / pieces;
          float ae = p[j] + dx * u0, an = p[j + 1] + dy * u0, be = p[j] + dx * u1, bn = p[j + 1] + dy * u1;
          if (Math.hypot((ae + be) / 2, (an + bn) / 2) > RAIL_R) continue;
          rails.add(new Seg(ae, an, be, bn, 0, 0, at + len * u0, RAIL));
        }
        if (len > 1e-3f) {
          float nx = -dy / len * TIE_HALF_M, ny = dx / len * TIE_HALF_M;
          for (; next <= at + len; next += TIE_M) {
            float u = (next - at) / len, te = p[j] + dx * u, tn = p[j + 1] + dy * u;
            if (te * te + tn * tn <= RAIL_R * RAIL_R) rails.add(new Seg(te - nx, tn - ny, te + nx, tn + ny, 0, 0, 0, TIE));
          }
        }
        at += len;
      }
    }
    out.roadDropped = cap(roads, ROAD_SEGS);
    out.walkDropped = cap(walks, WALK_SEGS);
    out.railDropped = cap(rails, RAIL_SEGS);
    out.roadSegs = roads.size();
    out.walkSegs = walks.size();
    out.railSegs = rails.size();
    // into ranges, in order
    int[] count = new int[RANGES];
    for (Seg s : roads) count[s.range]++;
    for (Seg s : walks) count[s.range]++;
    for (Seg s : rails) count[s.range]++;
    for (int r = 0; r < RANGES; r++) out.from[r + 1] = out.from[r] + count[r];
    int[] at = Arrays.copyOf(out.from, RANGES);
    out.seg = new float[out.count() * SEG];
    for (List<Seg> l : Arrays.asList(roads, walks, rails)) {
      for (Seg s : l) System.arraycopy(s.v, 0, out.seg, SEG * at[s.range]++, SEG);
    }
    return out;
  }

  // Lane lines of a motorway stretch rp with heights rh, into group g's lane ranges: the separators dashed, the edges
  // solid just inside the road, a two-way road's centre and a one-way road's left edge yellow. The road keeps its
  // drawn width (MapView.ROAD_M[0], three lanes of 18.7 m at ROAD_SCALE 2); the lanes divide it.
  static void lanes(float[] rp, float[] rh, int lanes, boolean oneway, int g, List<Seg> out) {
    int n = lanes > 0 ? lanes : oneway ? LANES_ONEWAY : LANES_TWOWAY;
    float w = MOTORWAY_W, edge = w / 2 - 1.5f;
    for (int i = 0; i <= n; i++) {
      float o = w * ((float) i / n - 0.5f);  // + to the left of the line's direction
      int kind;
      if (i == 0 || i == n) {
        o = i == 0 ? -edge : edge;
        kind = i == n && oneway ? YELLOW : EDGE;
      } else if (!oneway && 2 * i == n) {
        kind = YELLOW;
      } else {
        kind = DASH;
      }
      float[] q = offset(rp, o);
      float at = 0;
      for (int j = 0; j + 3 < q.length; j += 2) {
        float len = (float) Math.hypot(q[j + 2] - q[j], q[j + 3] - q[j + 1]);
        out.add(new Seg(q[j], q[j + 1], q[j + 2], q[j + 3], rh[j / 2], rh[j / 2 + 1], at, range(g, kind)));
        at += len;
      }
    }
  }

  // The polyline moved o metres to its left, mitred at the bends (at most twice o out, as a sharp bend would spike).
  static float[] offset(float[] p, float o) {
    int m = p.length / 2;
    float[] q = new float[p.length];
    for (int i = 0; i < m; i++) {
      float nx = 0, ny = 0, n1x = 0, n1y = 0;
      int segs = 0;
      for (int s = i - 1; s <= i; s++) {
        if (s < 0 || s + 1 >= m) continue;
        float dx = p[2 * s + 2] - p[2 * s], dy = p[2 * s + 3] - p[2 * s + 1];
        float l = (float) Math.hypot(dx, dy);
        if (l < 1e-4f) continue;
        nx += -dy / l;
        ny += dx / l;
        n1x = -dy / l;
        n1y = dx / l;
        segs++;
      }
      float nl = (float) Math.hypot(nx, ny), f = 1;
      if (nl < 1e-4f) {
        nx = n1x;
        ny = n1y;
      } else {
        nx /= nl;
        ny /= nl;
        float c = nx * n1x + ny * n1y;  // cos of half the bend
        f = segs > 1 ? Math.min(2, 1 / Math.max(c, 0.5f)) : 1;
      }
      q[2 * i] = p[2 * i] + nx * o * f;
      q[2 * i + 1] = p[2 * i + 1] + ny * o * f;
    }
    return q;
  }

  // Heights of a bridge's points: h, falling to 0 over rl at an end that ramps inside it (self).
  static float[] bridgeHeights(float[] p, float h, boolean self0, float rl, boolean self1) {
    float len = length(p);
    float[] out = new float[p.length / 2];
    float s = 0;
    for (int i = 0; i < out.length; i++) {
      if (i > 0) s += (float) Math.hypot(p[2 * i] - p[2 * i - 2], p[2 * i + 1] - p[2 * i - 1]);
      float f = 1;
      if (self0) f = Math.min(f, ease(s / rl));
      if (self1) f = Math.min(f, ease((len - s) / rl));
      out[i] = h * f;
    }
    return out;
  }

  // Heights of a road meeting a bridge end: h0 at its first point (h1 at its last), down to 0 RAMP_M along it.
  static float[] rampHeights(float[] p, float h0, float h1) {
    float len = length(p);
    float[] out = new float[p.length / 2];
    float s = 0;
    for (int i = 0; i < out.length; i++) {
      if (i > 0) s += (float) Math.hypot(p[2 * i] - p[2 * i - 2], p[2 * i + 1] - p[2 * i - 1]);
      out[i] = Math.max(h0 * ease(1 - s / RAMP_M), h1 * ease(1 - (len - s) / RAMP_M));
    }
    return out;
  }

  private static float ease(float t) {
    t = Math.max(0, Math.min(1, t));
    return t * t * (3 - 2 * t);
  }

  // The polyline with points added at distances c0 and c1 along it (ignored when negative or at a point already).
  static float[] cut(float[] p, float c0, float c1) {
    float lo = Math.min(c0, c1), hi = Math.max(c0, c1);
    float[] cs = lo >= 0 ? new float[] {lo, hi} : new float[] {hi};
    List<Float> out = new ArrayList<Float>();
    out.add(p[0]);
    out.add(p[1]);
    float s = 0;
    int next = 0;
    for (int i = 2; i + 1 < p.length; i += 2) {
      float d = (float) Math.hypot(p[i] - p[i - 2], p[i + 1] - p[i - 1]);
      while (next < cs.length && (cs[next] < 0 || cs[next] < s + d)) {
        float c = cs[next++];
        if (c < 0 || c - s <= 0.01f || s + d - c <= 0.01f) continue;
        float t = (c - s) / d;
        out.add(p[i - 2] + (p[i] - p[i - 2]) * t);
        out.add(p[i - 1] + (p[i + 1] - p[i - 1]) * t);
      }
      out.add(p[i]);
      out.add(p[i + 1]);
      s += d;
    }
    float[] q = new float[out.size()];
    for (int i = 0; i < q.length; i++) q[i] = out.get(i);
    return q;
  }

  // Runs of points [first, last, raised 0/1]: the raised stretches take in the point where they touch the ground.
  static List<int[]> runs(float[] h) {
    List<int[]> out = new ArrayList<int[]>();
    int m = h.length, i = 0;
    while (i + 1 < m) {
      boolean up = h[i] > 0 || h[i + 1] > 0;
      int j = i + 1;
      while (j + 1 < m && (h[j] > 0 || h[j + 1] > 0) == up) j++;
      out.add(new int[] {i, j, up ? 1 : 0});
      i = j;
    }
    return out;
  }

  // Drops the farthest segments past max; returns how many went.
  private static int cap(List<Seg> l, int max) {
    if (l.size() <= max) return 0;
    float[] d = new float[l.size()];
    for (int i = 0; i < d.length; i++) d[i] = l.get(i).d;
    Arrays.sort(d);
    float cut = d[max - 1];
    int below = 0;
    while (d[below] < cut) below++;
    int kept = 0, dropped = 0, ties = max - below;  // all nearer than cut, and as many at cut as fit
    for (int i = 0; i < l.size(); i++) {
      Seg s = l.get(i);
      if (s.d < cut || (s.d == cut && ties-- > 0)) {
        l.set(kept++, s);
      } else {
        dropped++;
      }
    }
    while (l.size() > kept) l.remove(l.size() - 1);
    return dropped;
  }

  // For GlClose: segments from..from+n of d as quads of 4 corners into out, CORNER floats each: a.e, a.n, b.e, b.n,
  // across (-1, 1), end (0, 1), metres along at a, the heights at a and b. Along is kept mod a multiple of both dash
  // periods so a float holds it to a millimetre however far the line runs.
  static void corners(Detail d, int from, int n, float[] out) {
    float[] v = d.seg;
    float period = 126;  // 7 x 18 m and 36 x 3.5 m: a multiple of both dash periods
    int o = 0;
    for (int i = from; i < from + n; i++) {
      int s = i * SEG;
      float at = v[s + 6] % period;
      for (int c = 0; c < 4; c++) {
        out[o] = v[s];
        out[o + 1] = v[s + 1];
        out[o + 2] = v[s + 2];
        out[o + 3] = v[s + 3];
        out[o + 4] = (c & 1) == 0 ? -1 : 1;
        out[o + 5] = c >> 1;
        out[o + 6] = at;
        out[o + 7] = v[s + 4];
        out[o + 8] = v[s + 5];
        o += CORNER;
      }
    }
  }

  static float length(float[] p) {
    float s = 0;
    for (int i = 2; i + 1 < p.length; i += 2) s += (float) Math.hypot(p[i] - p[i - 2], p[i + 1] - p[i - 1]);
    return s;
  }

  private static boolean near(float[] p, float r) {
    float r2 = r * r;
    for (int i = 0; i + 1 < p.length; i += 2) if (p[i] * p[i] + p[i + 1] * p[i + 1] < r2) return true;
    return false;
  }

  private static long key(float e, float n) {
    return ((long) Math.floor(e / JOIN_M) << 32) ^ ((long) Math.floor(n / JOIN_M) & 0xffffffffL);
  }

  // The other lines' ends within JOIN_M of (e, n), as [line, end].
  private static List<int[]> meeting(HashMap<Long, List<int[]>> ends, float e, float n, List<float[]> pts, int self) {
    List<int[]> out = new ArrayList<int[]>();
    for (int dx = -1; dx <= 1; dx++) {
      for (int dy = -1; dy <= 1; dy++) {
        List<int[]> l = ends.get(key(e + dx * JOIN_M, n + dy * JOIN_M));
        if (l == null) continue;
        for (int[] o : l) {
          if (o[0] == self) continue;
          float[] p = pts.get(o[0]);
          int j = o[1] == 0 ? 0 : p.length - 2;
          if (Math.hypot(p[j] - e, p[j + 1] - n) <= JOIN_M) out.add(o);
        }
      }
    }
    return out;
  }
}
