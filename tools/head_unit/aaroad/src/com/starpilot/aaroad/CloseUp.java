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
  // After the ties (2026-11-05), so every range before keeps its number: an overpass's deck per raised group (1 to 3)
  // and class, drawn as a structure (a fascia, open under it) where range(g, c) is drawn as a fill down to the ground
  // (ramps, a bridge's own ramped ends), then each group's piers.
  static final int PER_LAYER = CLASSES + 3, RAIL = 2 + LAYERS * PER_LAYER, TIE = RAIL + 1, DECK0 = TIE + 1;
  static final int PIER0 = DECK0 + (LAYERS - 1) * CLASSES, RANGES = PIER0 + LAYERS - 1;

  static int range(int layer, int kind) {
    return 2 + layer * PER_LAYER + kind;
  }

  // an overpass's deck in group g (1 to 3), class c
  static int deck(int g, int c) {
    return DECK0 + (g - 1) * CLASSES + c;
  }

  // group g's piers (1 to 3)
  static int pier(int g) {
    return PIER0 + g - 1;
  }

  // the road class a raised range draws (range(g, c) or deck(g, c))
  static int cls(int r) {
    return r >= DECK0 && r < PIER0 ? (r - DECK0) % CLASSES : (r - 2) % PER_LAYER;
  }

  static final class Detail {
    float[] seg = new float[0];
    final int[] from = new int[RANGES + 1];  // range r is segments from[r] to from[r + 1]
    int roadSegs, walkSegs, railSegs, roadDropped, walkDropped, railDropped, bridges, ramps;
    float maxH;  // the highest point, world metres
    // free drive, when an overpass was drawn flat as the car's road (overpasses()): that road (carRoad()), so
    // MapView rebuilds once the car leaves it (onRoad) and the overpass it then drives under is raised again
    float[][] heldOn;

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
    // which bridges are drawn raised (overpasses()); null: build() finds them, along path
    boolean[] over;
    float[][] path;
    // per road, where roads cross under it, UNDER floats each (e, n, that road's class, its direction de, dn), from
    // overpasses()
    float[][] under;
    // overpasses(): a bridge something crosses under was kept down as the road along path
    boolean held;

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

  // ---- which bridges are drawn raised (Peter 2026-11-05, FM 544: a creek bridge on the road he was on drew as a grey
  // slab floating over it, "like another route on top")
  //
  // A tile bridge is raised only when the car could drive under it: an overpass. That is, a road crosses it in plan
  // (a ground road, or a bridge of a lower layer; a rail line or a path does not count, as the car can't drive
  // there), at least CROSS_MIN_DEG across, more than JOIN_M from both lines' ends (a road meeting a bridge's end is
  // its approach, not under it). Bridges joined end to end on one layer (cut at tile edges, or split in the data)
  // count as one. And the road being driven is never raised: a bridge with ON_PATH_SHARE of its length within
  // ON_PATH_M and ON_PATH_DEG of the route (with no route, the car's own road ahead and behind, carRoad()) stays on
  // the ground. Every other bridge (a creek or ditch, a rail line or a path under it, the one the car is on) is drawn
  // as the road it is part of, with no deck, ramps, casing or caps, zoomed in or not.
  //
  // Wrong when: the road under it is missing from the tiles (a private drive, a road cut from the tile) - drawn flat;
  // a long water bridge with a shore road under one end - all of it raised; a viaduct over a parallel frontage road
  // that only crosses at a slant under CROSS_MIN_DEG - flat; free drive turned off the road before the scene is
  // rebuilt (FreeView.covered) - the new road's overpass stays raised until then (but leaving a road that held an
  // overpass down rebuilds at once: Detail.heldOn).
  static final float CROSS_MIN_DEG = 20, ON_PATH_M = 12, ON_PATH_DEG = 30, ON_PATH_SHARE = 0.5f;
  // carRoad: the car is on a line within FOLLOW_M running within FOLLOW_DEG of its heading; joins turning less than
  // FOLLOW_DEG continue it, ROAD_R each way
  static final float FOLLOW_M = 20, FOLLOW_DEG = 35;
  // an overpass's piers: a pair across the deck (PIER_ACROSS of its drawn half width out) about every PIER_M along
  // its full-height stretch, none within PIER_CLEAR_M of the drawn edge of a road crossing under it; a span shorter
  // than PIER_M rests on its ends. Drawn widths (MapView.ROAD_M, ROAD_SCALE 2) per class:
  static final float PIER_M = 30, PIER_ACROSS = 0.6f, PIER_CLEAR_M = 6;
  static final float[] ROAD_W = {56, 36, 20, 12, 10};
  static final int UNDER = 5;  // floats per crossing in Lines.under

  // Per road of in: true for a bridge drawn raised. Fills in.under. path: the route's (or carRoad's) polylines, or null.
  static boolean[] overpasses(Lines in, float[][] path) {
    int n = in.pts.size();
    boolean[] out = new boolean[n];
    in.under = new float[n][];
    List<Integer> br = new ArrayList<Integer>();
    for (int i = 0; i < n; i++) if (in.attr.get(i)[1] != 0) br.add(i);
    int nb = br.size();
    if (nb == 0) return out;
    float[][] box = new float[n][];
    for (int i = 0; i < n; i++) box[i] = bounds(in.pts.get(i));
    // bridges joined end to end on one layer: one group
    int[] parent = new int[nb];
    for (int k = 0; k < nb; k++) parent[k] = k;
    for (int a = 0; a < nb; a++) {
      for (int b = a + 1; b < nb; b++) {
        int ia = br.get(a), ib = br.get(b);
        if (deckLayer(in.attr.get(ia)) != deckLayer(in.attr.get(ib)) || !overlap(box[ia], box[ib], JOIN_M)) continue;
        if (endsMeet(in.pts.get(ia), in.pts.get(ib))) parent[find(parent, a)] = find(parent, b);
      }
    }
    boolean[] crossed = new boolean[nb];
    List<List<float[]>> under = new ArrayList<List<float[]>>();
    for (int k = 0; k < nb; k++) under.add(new ArrayList<float[]>());
    float minSin = (float) Math.sin(Math.toRadians(CROSS_MIN_DEG));
    for (int i = 0; i < n; i++) {
      int[] a = in.attr.get(i);
      float[] p = in.pts.get(i);
      for (int k = 0; k < nb; k++) {
        int b = br.get(k);
        if (b == i || !overlap(box[i], box[b], 0)) continue;
        if (a[1] != 0 && deckLayer(a) >= deckLayer(in.attr.get(b))) continue;  // level with it or over it
        float[] q = in.pts.get(b);
        for (int j = 0; j + 3 < p.length; j += 2) {
          for (int m = 0; m + 3 < q.length; m += 2) {
            float rx = p[j + 2] - p[j], ry = p[j + 3] - p[j + 1], sx = q[m + 2] - q[m], sy = q[m + 3] - q[m + 1];
            float den = rx * sy - ry * sx, lr = (float) Math.hypot(rx, ry), ls = (float) Math.hypot(sx, sy);
            if (lr <= 0 || ls <= 0 || Math.abs(den) < minSin * lr * ls) continue;
            float qx = q[m] - p[j], qy = q[m + 1] - p[j + 1];
            float t = (qx * sy - qy * sx) / den, u = (qx * ry - qy * rx) / den;
            if (t < 0 || t > 1 || u < 0 || u > 1) continue;
            float xe = p[j] + t * rx, xn = p[j + 1] + t * ry;
            if (nearEnd(p, xe, xn) || nearEnd(q, xe, xn)) continue;
            crossed[k] = true;
            under.get(k).add(new float[] {xe, xn, a[0], rx / lr, ry / lr});
          }
        }
      }
    }
    boolean[] group = new boolean[nb];
    for (int k = 0; k < nb; k++) if (crossed[k]) group[find(parent, k)] = true;
    // the road being driven: a group mostly along the path stays down
    if (path != null && path.length > 0) {
      float[] len = new float[nb], on = new float[nb];
      for (int k = 0; k < nb; k++) {
        int g = find(parent, k);
        if (!group[g]) continue;
        float[] q = in.pts.get(br.get(k));
        for (int m = 0; m + 3 < q.length; m += 2) {
          float sx = q[m + 2] - q[m], sy = q[m + 3] - q[m + 1], l = (float) Math.hypot(sx, sy);
          if (l <= 0) continue;
          int steps = Math.max(1, (int) Math.ceil(l / 10));
          for (int st = 0; st < steps; st++) {
            float f = (st + 0.5f) / steps;
            len[g] += l / steps;
            if (alongPath(path, q[m] + sx * f, q[m + 1] + sy * f, sx / l, sy / l)) on[g] += l / steps;
          }
        }
      }
      for (int g = 0; g < nb; g++) {
        if (group[g] && len[g] > 0 && on[g] >= ON_PATH_SHARE * len[g]) {
          group[g] = false;
          in.held = true;
        }
      }
    }
    for (int k = 0; k < nb; k++) {
      int b = br.get(k);
      out[b] = group[find(parent, k)];
      if (out[b] && !under.get(k).isEmpty()) in.under[b] = flatten(under.get(k));
    }
    return out;
  }

  // ---- which link roads are highway ramps (Peter 2026-11-05, Angel Pkwy: short olive strokes inside grey streets
  // looked like traffic)
  //
  // Tile class 3 (road_tiles._CLASS_OF LINK) is every Mapbox "_link": motorway and trunk ramps, but also the
  // primary/secondary/tertiary links, the turn and slip lanes at a street corner. The ramp gold (MapView.ROADS[3]) is
  // meant to say "highway ramp", so a link keeps it only when it touches a motorway or trunk (class 0) or joins one
  // through other links: an end of one within JOIN_M of the other's line. Every other link is drawn as a street
  // (STREET, class 2: its grey, width and kerb). Lines are in metres, in one frame for all tiles (tile edges cut
  // lines; the halves' ends land on each other, so a ramp cut at a tile edge stays one).
  //
  // Wrong when: the highway or the rest of the ramp is in a tile not yet loaded - grey until the scene is rebuilt with
  // it; a street link ending within JOIN_M of a highway it does not join (under a highway bridge) - gold; a slip lane
  // at an at-grade junction on a trunk road touches the trunk - gold; a U-turn lane under a highway joining two
  // frontage roads touches no highway - grey.
  static final int MOTORWAY = 0, STREET = 2, LINK = 3;

  // Per link: true for a highway ramp (keeps the ramp gold).
  static boolean[] ramps(List<float[]> links, List<float[]> highways) {
    int n = links.size();
    boolean[] out = new boolean[n];
    if (n == 0) return out;
    float[][] box = new float[n][];
    for (int i = 0; i < n; i++) box[i] = bounds(links.get(i));
    boolean[] touch = new boolean[n];
    for (float[] h : highways) {
      if (h.length < 2) continue;
      float[] hb = bounds(h);
      for (int i = 0; i < n; i++) {
        if (touch[i] || !overlap(box[i], hb, JOIN_M)) continue;
        touch[i] = touches(links.get(i), h);
      }
    }
    int[] parent = new int[n];
    for (int i = 0; i < n; i++) parent[i] = i;
    for (int a = 0; a < n; a++) {
      for (int b = a + 1; b < n; b++) {
        if (!overlap(box[a], box[b], JOIN_M) || find(parent, a) == find(parent, b)) continue;
        if (touches(links.get(a), links.get(b))) parent[find(parent, a)] = find(parent, b);
      }
    }
    boolean[] group = new boolean[n];
    for (int i = 0; i < n; i++) if (touch[i]) group[find(parent, i)] = true;
    for (int i = 0; i < n; i++) out[i] = group[find(parent, i)];
    return out;
  }

  // an end of p within JOIN_M of q's line, or an end of q within JOIN_M of p's
  private static boolean touches(float[] p, float[] q) {
    if (p.length < 2 || q.length < 2) return false;
    return within(q, p[0], p[1]) || within(q, p[p.length - 2], p[p.length - 1])
        || within(p, q[0], q[1]) || within(p, q[q.length - 2], q[q.length - 1]);
  }

  private static boolean within(float[] p, float e, float n) {
    float m2 = JOIN_M * JOIN_M;
    if (p.length < 4) return (p[0] - e) * (p[0] - e) + (p[1] - n) * (p[1] - n) <= m2;
    for (int j = 0; j + 3 < p.length; j += 2) {
      float dx = p[j + 2] - p[j], dy = p[j + 3] - p[j + 1], len2 = dx * dx + dy * dy;
      float t = len2 <= 0 ? 0 : ((e - p[j]) * dx + (n - p[j + 1]) * dy) / len2;
      t = t < 0 ? 0 : t > 1 ? 1 : t;
      float fe = p[j] + t * dx - e, fn = p[j + 1] + t * dy - n;
      if (fe * fe + fn * fn <= m2) return true;
    }
    return false;
  }

  // The car's road (with no route): the line within FOLLOW_M of the car at (0, 0) running its way (he, hn, a unit
  // vector east, north), and the lines that carry on from each of its ends turning least, up to ROAD_R each way.
  // Null when the car is on no line.
  static float[][] carRoad(Lines in, float he, float hn) {
    int n = in.pts.size(), best = -1;
    float bestD = FOLLOW_M, cos = (float) Math.cos(Math.toRadians(FOLLOW_DEG));
    for (int i = 0; i < n; i++) {
      float[] p = in.pts.get(i);
      for (int j = 0; j + 3 < p.length; j += 2) {
        float dx = p[j + 2] - p[j], dy = p[j + 3] - p[j + 1], l = (float) Math.hypot(dx, dy);
        if (l <= 0 || Math.abs((dx * he + dy * hn) / l) < cos) continue;
        float t = Math.max(0, Math.min(1, -(p[j] * dx + p[j + 1] * dy) / (l * l)));
        float d = (float) Math.hypot(p[j] + t * dx, p[j + 1] + t * dy);
        if (d < bestD) {
          bestD = d;
          best = i;
        }
      }
    }
    if (best < 0) return null;
    List<float[]> out = new ArrayList<float[]>();
    boolean[] used = new boolean[n];
    used[best] = true;
    out.add(in.pts.get(best));
    for (int e = 0; e < 2; e++) {
      int at = best, end = e;
      float run = length(in.pts.get(best)) / 2;
      while (run < ROAD_R) {
        float[] p = in.pts.get(at);
        int j = end == 0 ? 0 : p.length - 2, k = end == 0 ? 2 : p.length - 4;
        float ox = p[j] - p[k], oy = p[j + 1] - p[k + 1], ol = (float) Math.hypot(ox, oy);  // out of the line here
        if (ol <= 0) break;
        int next = -1, nextEnd = 0;
        float nextCos = cos;
        for (int i = 0; i < n; i++) {
          if (used[i]) continue;
          float[] q = in.pts.get(i);
          for (int qe = 0; qe < 2; qe++) {
            int a = qe == 0 ? 0 : q.length - 2, b = qe == 0 ? 2 : q.length - 4;
            if (Math.hypot(q[a] - p[j], q[a + 1] - p[j + 1]) > JOIN_M) continue;
            float ix = q[b] - q[a], iy = q[b + 1] - q[a + 1], il = (float) Math.hypot(ix, iy);  // into it from here
            if (il <= 0) continue;
            float c = (ox * ix + oy * iy) / (ol * il);
            if (c > nextCos) {
              nextCos = c;
              next = i;
              nextEnd = 1 - qe;
            }
          }
        }
        if (next < 0) break;
        used[next] = true;
        out.add(in.pts.get(next));
        run += length(in.pts.get(next));
        at = next;
        end = nextEnd;
      }
    }
    return out.toArray(new float[0][]);
  }

  // Whether a car at (e, n) heading (he, hn), a unit vector east, north, is still on road (carRoad's lines): within
  // FOLLOW_M of one running within FOLLOW_DEG of its heading.
  static boolean onRoad(float[][] road, float e, float n, float he, float hn) {
    float cos = (float) Math.cos(Math.toRadians(FOLLOW_DEG));
    for (float[] p : road) {
      for (int j = 0; j + 3 < p.length; j += 2) {
        float dx = p[j + 2] - p[j], dy = p[j + 3] - p[j + 1], l = (float) Math.hypot(dx, dy);
        if (l <= 0 || Math.abs((dx * he + dy * hn) / l) < cos) continue;
        float t = Math.max(0, Math.min(1, ((e - p[j]) * dx + (n - p[j + 1]) * dy) / (l * l)));
        if (Math.hypot(p[j] + t * dx - e, p[j + 1] + t * dy - n) <= FOLLOW_M) return true;
      }
    }
    return false;
  }

  private static boolean alongPath(float[][] path, float e, float n, float ue, float un) {
    float cos = (float) Math.cos(Math.toRadians(ON_PATH_DEG));
    for (float[] p : path) {
      for (int j = 0; j + 3 < p.length; j += 2) {
        float ax = p[j], ay = p[j + 1], dx = p[j + 2] - ax, dy = p[j + 3] - ay;
        if (Math.min(ax, ax + dx) > e + ON_PATH_M || Math.max(ax, ax + dx) < e - ON_PATH_M
            || Math.min(ay, ay + dy) > n + ON_PATH_M || Math.max(ay, ay + dy) < n - ON_PATH_M) continue;
        float l = (float) Math.hypot(dx, dy);
        if (l <= 0 || Math.abs((dx * ue + dy * un) / l) < cos) continue;
        float t = Math.max(0, Math.min(1, ((e - ax) * dx + (n - ay) * dy) / (l * l)));
        if (Math.hypot(ax + t * dx - e, ay + t * dy - n) <= ON_PATH_M) return true;
      }
    }
    return false;
  }

  private static int deckLayer(int[] a) {
    return Math.max(1, a[2]);
  }

  private static int find(int[] parent, int k) {
    while (parent[k] != k) k = parent[k] = parent[parent[k]];
    return k;
  }

  private static float[] bounds(float[] p) {
    float[] b = {Float.MAX_VALUE, Float.MAX_VALUE, -Float.MAX_VALUE, -Float.MAX_VALUE};
    for (int i = 0; i + 1 < p.length; i += 2) {
      b[0] = Math.min(b[0], p[i]);
      b[1] = Math.min(b[1], p[i + 1]);
      b[2] = Math.max(b[2], p[i]);
      b[3] = Math.max(b[3], p[i + 1]);
    }
    return b;
  }

  private static boolean overlap(float[] a, float[] b, float m) {
    return a[0] <= b[2] + m && b[0] <= a[2] + m && a[1] <= b[3] + m && b[1] <= a[3] + m;
  }

  private static boolean endsMeet(float[] p, float[] q) {
    for (int a = 0; a < 2; a++) {
      for (int b = 0; b < 2; b++) {
        int i = a == 0 ? 0 : p.length - 2, j = b == 0 ? 0 : q.length - 2;
        if (Math.hypot(p[i] - q[j], p[i + 1] - q[j + 1]) <= JOIN_M) return true;
      }
    }
    return false;
  }

  private static boolean nearEnd(float[] p, float e, float n) {
    int j = p.length - 2;
    return Math.hypot(p[0] - e, p[1] - n) <= JOIN_M || Math.hypot(p[j] - e, p[j + 1] - n) <= JOIN_M;
  }

  private static float[] flatten(List<float[]> l) {
    float[] out = new float[UNDER * l.size()];
    for (int i = 0; i < l.size(); i++) System.arraycopy(l.get(i), 0, out, UNDER * i, UNDER);
    return out;
  }

  static Detail build(Lines in) {
    Detail out = new Detail();
    boolean[] over = in.over != null ? in.over : overpasses(in, in.path);
    // the roads near enough; a bridge not raised (overpasses()) as the ground road it is drawn as
    List<float[]> pts = new ArrayList<float[]>(), unders = new ArrayList<float[]>();
    List<int[]> attr = new ArrayList<int[]>();
    for (int i = 0; i < in.pts.size(); i++) {
      if (near(in.pts.get(i), ROAD_R)) {
        int[] a = in.attr.get(i);
        if (a[1] != 0 && !over[i]) {
          a = a.clone();
          a[1] = 0;
        }
        pts.add(in.pts.get(i));
        attr.add(a);
        unders.add(in.under != null && i < in.under.length ? in.under[i] : null);
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
        // an overpass's full-height stretch is its deck (drawn open under it), the ramped rest a fill to the ground
        float full = Math.max(1, a[2]) * LAYER_M - 1e-3f;
        if (up) for (int j = 0; j + 1 < rh.length; j++) {
          int r = bridge && rh[j] >= full && rh[j + 1] >= full ? deck(g, a[0]) : range(g, a[0]);
          roads.add(new Seg(rp[2 * j], rp[2 * j + 1], rp[2 * j + 2], rp[2 * j + 3], rh[j], rh[j + 1], 0, r));
        }
        if (up && bridge) piers(rp, rh, full, a[0], g, unders.get(i), roads);
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

  // An overpass stretch rp's piers (heights rh, full from full up, class c, group g): each a vertical segment from the
  // ground to the deck, a pair across it every PIER_M or so along each full-height run, clear of the roads under it
  // (under: Lines.under's crossings; may be null).
  static void piers(float[] rp, float[] rh, float full, int c, int g, float[] under, List<Seg> out) {
    float s = 0, s0 = -1, last = 0;
    List<float[]> runs = new ArrayList<float[]>();
    for (int j = 0; j < rh.length; j++) {
      if (j > 0) s += (float) Math.hypot(rp[2 * j] - rp[2 * j - 2], rp[2 * j + 1] - rp[2 * j - 1]);
      if (rh[j] >= full) {
        if (s0 < 0) s0 = s;
        last = s;
      } else if (s0 >= 0) {
        runs.add(new float[] {s0, last});
        s0 = -1;
      }
    }
    if (s0 >= 0) runs.add(new float[] {s0, last});
    float half = ROAD_W[c] / 2 * PIER_ACROSS;
    for (float[] run : runs) {
      float len = run[1] - run[0];
      int count = (int) Math.floor(len / PIER_M);
      for (int k = 1; k <= count; k++) {
        float at = run[0] + len * k / (count + 1);
        float[] pt = pointAt(rp, at);
        if (pt == null) continue;
        float h = heightAt(rp, rh, at);
        for (int side = -1; side <= 1; side += 2) {
          float e = pt[0] - pt[3] * half * side, n = pt[1] + pt[2] * half * side;
          boolean clear = true;  // off every road under: its drawn half width and PIER_CLEAR_M from its line
          for (int u = 0; under != null && u + UNDER <= under.length; u += UNDER) {
            float dx = e - under[u], dy = n - under[u + 1];
            float off = Math.abs(dx * under[u + 4] - dy * under[u + 3]);
            if (off < ROAD_W[(int) under[u + 2]] / 2 + PIER_CLEAR_M && Math.hypot(dx, dy) < 2 * ROAD_W[0]) clear = false;
          }
          if (clear) out.add(new Seg(e, n, e, n, 0, h, 0, pier(g)));
        }
      }
    }
  }

  // the point at metres along p: e, n, the unit direction there (de, dn); null past its end
  private static float[] pointAt(float[] p, float at) {
    float s = 0;
    for (int j = 0; j + 3 < p.length; j += 2) {
      float dx = p[j + 2] - p[j], dy = p[j + 3] - p[j + 1], l = (float) Math.hypot(dx, dy);
      if (l > 0 && at <= s + l) {
        float t = Math.max(0, (at - s) / l);
        return new float[] {p[j] + dx * t, p[j + 1] + dy * t, dx / l, dy / l};
      }
      s += l;
    }
    return null;
  }

  private static float heightAt(float[] p, float[] h, float at) {
    float s = 0;
    for (int j = 0; j + 1 < h.length; j++) {
      float l = (float) Math.hypot(p[2 * j + 2] - p[2 * j], p[2 * j + 3] - p[2 * j + 1]);
      if (l > 0 && at <= s + l) return h[j] + (h[j + 1] - h[j]) * Math.max(0, (at - s) / l);
      s += l;
    }
    return h[h.length - 1];
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
