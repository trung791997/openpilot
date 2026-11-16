package com.starpilot.aaroad;

import android.graphics.Canvas;
import android.graphics.Paint;
import android.graphics.Path;

// The close-up look (CloseUp) for the view: the scene's detail found in the tiles, and drawn in software when the GPU
// layer is not up (and in the desktop harness). GlClose draws the same passes in the same order on the GPU.
final class CloseView {
  static final int WALK_C = 0xffa4a8ae, CROSS_C = 0xffc4c8ce, LANE_C = 0xffdedede, YELLOW_C = 0xffd8ae3a;
  static final int WALL_C = 0xff1e1d1a, SHADOW_C = 0xff000000;
  // on screen at the car: a sidewalk 1.2 px, a crossing's stripes 3 px, a lane line 1.2 px; the shadow reaches
  // SHADOW_PX past a deck's casing, in two steps
  static final float WALK_PX = 1.2f, CROSS_PX = 3, LANE_PX = 1.2f, SHADOW_PX = 5;
  static final float WALK_A = 0.55f, CROSS_A = 0.7f, LANE_A = 0.8f, SHADOW_A = 0.22f;
  // Peter 2026-11-08, Apple-style 3D: a third, wider and fainter shadow stroke so the two hard-edged steps above
  // read as a soft falloff instead of a double outline. Drawn widest/faintest first, same as the two steps already
  // there. *static*: picked by eye in the desktop harness, not measured on the head unit.
  static final float SHADOW_PX3 = 9, SHADOW_A3 = 0.10f;
  // the deck's side wall: the deck drawn dark at these fractions of its height under it (a quarter apart, so a thin
  // link ramp two layers up still shows one wall, not stripes)
  static final float[] WALL_AT = {0, 0.25f, 0.5f, 0.75f};
  // Peter 2026-11-08, Apple-style 3D: baked directional shading, one fixed light implied from above/the deck top -
  // WALL_AT is ordered bottom (0, nearest the ground, darkest) to top (0.75, nearest the deck, lightest), so a
  // per-band brightness scalar here reads as a lit gradient down the wall at zero extra fragment cost (same strokes,
  // same draw calls, just a baked uColor.rgb multiplier instead of per-pixel lighting). *static*: a visual choice,
  // not measured.
  static final float[] WALL_SHADE = {0.65f, 0.78f, 0.9f, 1.0f};
  // a rail line: a thin dark grey line with thin cross ties (CloseUp.TIE_M), as a printed map draws one, darker than
  // any road so it reads as under them
  static final int RAIL_C = 0xff707479, TIE_C = 0xff5c6065;
  static final float RAIL_PX = 1.3f, TIE_PX = 1.1f;
  // an overpass (CloseUp.overpasses) as a structure: its deck's fascia, a concrete band under the parapet drawn at
  // these fractions of its height (the lowest the dark soffit), a light parapet for the casing, and concrete piers
  // PIER_PX wide down to its shadow
  static final float[] FASCIA_AT = {0.8f, 0.87f, 0.94f};
  static final int UNDER_C = 0xff2b2a27, FASCIA_C = 0xff6e6b64, PARAPET_C = 0xff8c8981, PIER_C = 0xff5f5c56;
  static final float PIER_PX = 3;
  // Peter 2026-11-08, Apple-style 3D: FASCIA_AT is ordered soffit (darkest, unlit) to top band (lightest, nearest
  // the lit deck top); PIER_SHADE darkens the piers below even the darkest wall band, so the stack reads
  // top-lightest to pier-darkest. *static*: a visual choice, not measured.
  static final float[] FASCIA_SHADE = {0.85f, 0.92f, 1.05f};
  static final float PIER_SHADE = 0.55f;
  // Peter 2026-11-08, Apple-style 3D: the deck's own top surface (the road colour drawn on the lifted deck range)
  // lightened a little versus the same class's ground-level colour, so the deck top reads as the lit face next to
  // the darker wall/fascia bands below it. *static*: a visual choice, not measured.
  static final float DECK_TOP_SHADE = 1.12f;

  // Peter 2026-11-08, Apple-style 3D: scales color's rgb by f (clamping each channel to 255), alpha unchanged.
  // Called only while building the shaded colour tables below (class init, not per frame or per segment).
  private static int shade(int color, float f) {
    int r = Math.min(255, Math.round(((color >> 16) & 0xff) * f));
    int g = Math.min(255, Math.round(((color >> 8) & 0xff) * f));
    int b = Math.min(255, Math.round((color & 0xff) * f));
    return (color & 0xff000000) | (r << 16) | (g << 8) | b;
  }

  // Peter 2026-11-08, Apple-style 3D: baked once, not per frame - WALL_C/FASCIA_C/UNDER_C shaded per band above,
  // and the deck-top (lightened) road/kerb colours by class, so passes() below just looks the right one up.
  private static final int[] WALL_SHADED = new int[WALL_AT.length];
  private static final int[] FASCIA_SHADED = new int[FASCIA_AT.length];
  private static final int UNDER_SHADED = shade(UNDER_C, FASCIA_SHADE[0]);
  private static final int PIER_SHADED = shade(PIER_C, PIER_SHADE);
  private static final int[] DECK_ROADS = new int[MapView.ROADS.length];
  private static final int[] DECK_KERBS = new int[MapView.KERBS.length];
  static {
    for (int i = 0; i < WALL_AT.length; i++) WALL_SHADED[i] = shade(WALL_C, WALL_SHADE[i]);
    for (int i = 0; i < FASCIA_AT.length; i++) FASCIA_SHADED[i] = shade(FASCIA_C, FASCIA_SHADE[i]);
    for (int i = 0; i < MapView.ROADS.length; i++) DECK_ROADS[i] = shade(MapView.ROADS[i], DECK_TOP_SHADE);
    for (int i = 0; i < MapView.KERBS.length; i++) DECK_KERBS[i] = shade(MapView.KERBS[i], DECK_TOP_SHADE);
  }

  // One pass: range r of the detail at hs of its height, dashed (on, period world m) or solid (period 0).
  interface Pass {
    void lines(int r, float hs, float width, int color, float alpha, float on, float period);
  }

  // The passes, in order, for a frame with gate gr (roads, bridges, lanes) and gw (walks): the walks, the ground's
  // lane lines, the shadows of every raised road, the overpasses' piers, then each layer group from the lowest: its
  // ramps' walls, its overpass decks' fascia, then casing (parapet on a deck), kerb, surface and lanes of both.
  static void passes(CloseUp.Detail d, float ik, float gr, float gw, Pass p) {
    float lane = MapView.dm(LANE_PX), floor = MapView.dm(MapView.MIN_ROAD_PX), kerb = MapView.dm(2 * MapView.KERB_PX);
    float casing = MapView.dm(2 * MapView.KERB_PX + 2 * MapView.CASING_PX);
    if (gw > 0) {
      p.lines(CloseUp.WALK, 0, MapView.dm(WALK_PX), WALK_C, WALK_A * gw, 0, 0);
      p.lines(CloseUp.CROSS, 0, MapView.dm(CROSS_PX), CROSS_C, CROSS_A * gw, CloseUp.CROSS_ON_M, CloseUp.CROSS_PERIOD_M);
    }
    if (gr <= 0) return;
    lanes(0, 0, lane, gr, p);
    for (int g = 1; g < CloseUp.LAYERS; g++) {
      for (int c : MapView.DRAW_ORDER) {
        float w = Math.max(floor, MapView.ROAD_M[c] * ik) + casing;
        for (int pass = 0; pass < 2; pass++) {  // range, then deck: no array per call
          int r = pass == 0 ? CloseUp.range(g, c) : CloseUp.deck(g, c);
          // widest/faintest first (phase 2's soft-falloff step), then the two existing hard-edged steps
          p.lines(r, 0, w + MapView.dm(SHADOW_PX3), SHADOW_C, SHADOW_A3 * gr, 0, 0);
          p.lines(r, 0, w + 2 * MapView.dm(SHADOW_PX), SHADOW_C, SHADOW_A * gr, 0, 0);
          p.lines(r, 0, w + MapView.dm(SHADOW_PX), SHADOW_C, SHADOW_A * gr, 0, 0);
        }
      }
    }
    // piers stand on the ground, so under every deck: each a column from 0 to the deck (zero plan length)
    for (int g = 1; g < CloseUp.LAYERS; g++) p.lines(CloseUp.pier(g), gr, MapView.dm(PIER_PX), PIER_SHADED, gr, 0, 0);
    for (int g = 1; g < CloseUp.LAYERS; g++) {
      for (int a = 0; a < WALL_AT.length; a++) {
        for (int c : MapView.DRAW_ORDER) {
          p.lines(CloseUp.range(g, c), WALL_AT[a] * gr, Math.max(floor, MapView.ROAD_M[c] * ik) + casing,
                  WALL_SHADED[a], gr, 0, 0);
        }
      }
      for (int f = 0; f < FASCIA_AT.length; f++) {
        for (int c : MapView.DRAW_ORDER) {
          p.lines(CloseUp.deck(g, c), FASCIA_AT[f] * gr, Math.max(floor, MapView.ROAD_M[c] * ik) + casing,
                  f == 0 ? UNDER_SHADED : FASCIA_SHADED[f], gr, 0, 0);
        }
      }
      for (int c : MapView.DRAW_ORDER) {
        float w = Math.max(floor, MapView.ROAD_M[c] * ik) + casing;
        p.lines(CloseUp.range(g, c), gr, w, MapView.CASING_C, gr, 0, 0);
        p.lines(CloseUp.deck(g, c), gr, w, PARAPET_C, gr, 0, 0);
      }
      for (int c : MapView.DRAW_ORDER) {
        float w = Math.max(floor, MapView.ROAD_M[c] * ik) + kerb;
        p.lines(CloseUp.range(g, c), gr, w, MapView.KERBS[c], 1, 0, 0);
        p.lines(CloseUp.deck(g, c), gr, w, DECK_KERBS[c], 1, 0, 0);
      }
      for (int c : MapView.DRAW_ORDER) {
        float w = Math.max(floor, MapView.ROAD_M[c] * ik);
        p.lines(CloseUp.range(g, c), gr, w, MapView.ROADS[c], 1, 0, 0);
        p.lines(CloseUp.deck(g, c), gr, w, DECK_ROADS[c], 1, 0, 0);
      }
      lanes(g, gr, lane, gr, p);
    }
  }

  // The rail lines, under the roads (MapView and GlRoads draw this before the roads), with the roads' gate gr: the
  // ties, then the line over them.
  static void rails(CloseUp.Detail d, float gr, Pass p) {
    if (gr <= 0) return;
    p.lines(CloseUp.TIE, 0, MapView.dm(TIE_PX), TIE_C, gr, 0, 0);
    p.lines(CloseUp.RAIL, 0, MapView.dm(RAIL_PX), RAIL_C, gr, 0, 0);
  }

  private static void lanes(int g, float hs, float lane, float gr, Pass p) {
    p.lines(CloseUp.range(g, CloseUp.DASH), hs, lane, LANE_C, LANE_A * gr, CloseUp.DASH_ON_M, CloseUp.DASH_PERIOD_M);
    p.lines(CloseUp.range(g, CloseUp.EDGE), hs, lane, LANE_C, LANE_A * gr, 0, 0);
    p.lines(CloseUp.range(g, CloseUp.YELLOW), hs, lane, YELLOW_C, LANE_A * gr, 0, 0);
  }

  // The scene's detail: the roads and walks of the tiles in metres from the anchor (ax, ay tile units, mpt metres per
  // tile unit), as MapView.free() places them. route: the route's runs in the same metres (empty in free drive, when
  // the car's own road along h, its heading in radians from north, stands in for it; see CloseUp.overpasses). raised,
  // when not null, gets per tile, class and line whether a tile bridge in the detail is drawn raised (2) or flat (1),
  // for the base layer (MapView.build); 0, a tile bridge out of reach, keeps its old look. streets: links() of the
  // same tiles, the link roads drawn as streets.
  static CloseUp.Detail detail(Iterable<Feed.Tile> tiles, double ax, double ay, double mpt, float[][] route, double h,
                               java.util.Map<Feed.Tile, byte[][]> raised, java.util.Map<Feed.Tile, boolean[]> streets) {
    CloseUp.Lines in = new CloseUp.Lines();
    java.util.List<Object[]> from = new java.util.ArrayList<Object[]>();  // per road of in: tile, class, line
    double reach = Math.max(CloseUp.ROAD_R, CloseUp.WALK_R) / mpt;
    for (Feed.Tile tile : tiles) {
      double dx = Math.max(0, Math.max(tile.x - ax, ax - (tile.x + 1))), dy = Math.max(0, Math.max(tile.y - ay, ay - (tile.y + 1)));
      if (dx * dx + dy * dy > reach * reach) continue;
      for (int c = 0; c < 5; c++) {
        float[] pts = tile.pts[c];
        int[] starts = tile.starts[c];
        if (pts == null || starts == null) continue;
        boolean[] bridge = tile.bridge[c], oneway = tile.oneway[c];
        byte[] layer = tile.layer[c], lanes = tile.lanes[c];
        for (int l = 0; l + 1 < starts.length; l++) {
          boolean br = bridge != null && l < bridge.length && bridge[l];
          // every line: a bridge, a motorway (lanes), what may meet a bridge (its ramps) or pass under it, the car's road
          int before = in.pts.size();
          in.road(metres(tile, pts, starts[l], starts[l + 1], ax, ay, mpt), drawn(streets, tile, c, l), br,
                  layer != null && l < layer.length ? layer[l] : 0, lanes != null && l < lanes.length ? lanes[l] & 0xff : 0,
                  oneway != null && l < oneway.length && oneway[l]);
          if (in.pts.size() > before) from.add(new Object[] {tile, c, l});
        }
      }
      int[] ws = tile.walkStarts;
      for (int l = 0; l + 1 < ws.length; l++) {
        in.walk(metres(tile, tile.walkPts, ws[l], ws[l + 1], ax, ay, mpt), l < tile.walkKind.length ? tile.walkKind[l] : 0);
      }
      int[] rs = tile.railStarts;
      for (int l = 0; l + 1 < rs.length; l++) in.rail(metres(tile, tile.railPts, rs[l], rs[l + 1], ax, ay, mpt));
    }
    float[][] path = route != null && route.length > 0 ? route : CloseUp.carRoad(in, (float) Math.sin(h), (float) Math.cos(h));
    in.over = CloseUp.overpasses(in, path);
    if (raised != null) {
      for (int i = 0; i < from.size(); i++) {
        if (in.attr.get(i)[1] == 0) continue;
        Object[] t = from.get(i);
        Feed.Tile tile = (Feed.Tile) t[0];
        int c = (Integer) t[1], l = (Integer) t[2];
        byte[][] r = raised.get(tile);
        if (r == null) raised.put(tile, r = new byte[5][]);
        if (r[c] == null) r[c] = new byte[tile.starts[c].length];
        r[c][l] = (byte) (in.over[i] ? 2 : 1);
      }
    }
    CloseUp.Detail d = CloseUp.build(in);
    // free drive: an overpass held down as the car's road stays down only while the car is on that road
    if ((route == null || route.length == 0) && in.held) d.heldOn = path;
    return d;
  }

  // The link roads (tile class 3) that are not highway ramps (CloseUp.ramps), per tile: true for a line drawn as a
  // street. Once per scene (MapView.build), over every tile, so a ramp cut at a tile edge is judged whole; the base
  // map (MapView.build) and the close-up (detail) both draw a line's class through drawn(), so the GPU and software
  // paths, zoomed in or not, agree.
  static java.util.Map<Feed.Tile, boolean[]> links(Iterable<Feed.Tile> tiles, double ax, double ay, double mpt) {
    java.util.List<float[]> links = new java.util.ArrayList<float[]>(), highways = new java.util.ArrayList<float[]>();
    java.util.List<Feed.Tile> of = new java.util.ArrayList<Feed.Tile>();
    for (Feed.Tile tile : tiles) {
      int[] ls = tile.starts[CloseUp.LINK];
      if (tile.pts[CloseUp.LINK] == null || ls == null) continue;
      for (int l = 0; l + 1 < ls.length; l++) {
        links.add(metres(tile, tile.pts[CloseUp.LINK], ls[l], ls[l + 1], ax, ay, mpt));
        of.add(tile);
      }
    }
    java.util.IdentityHashMap<Feed.Tile, boolean[]> out = new java.util.IdentityHashMap<Feed.Tile, boolean[]>();
    if (links.isEmpty()) return out;
    for (Feed.Tile tile : tiles) {
      int[] hs = tile.starts[CloseUp.MOTORWAY];
      if (tile.pts[CloseUp.MOTORWAY] == null || hs == null) continue;
      for (int l = 0; l + 1 < hs.length; l++) highways.add(metres(tile, tile.pts[CloseUp.MOTORWAY], hs[l], hs[l + 1], ax, ay, mpt));
    }
    boolean[] ramp = CloseUp.ramps(links, highways);
    int i = 0;
    while (i < of.size()) {
      Feed.Tile tile = of.get(i);
      boolean[] street = new boolean[tile.starts[CloseUp.LINK].length - 1];
      for (int l = 0; l < street.length; l++, i++) street[l] = !ramp[i];
      out.put(tile, street);
    }
    return out;
  }

  // the class tile line (c, l) is drawn as: a link that is not a highway ramp as a street, else its tile class
  static int drawn(java.util.Map<Feed.Tile, boolean[]> streets, Feed.Tile tile, int c, int l) {
    if (c != CloseUp.LINK || streets == null) return c;
    boolean[] st = streets.get(tile);
    return st != null && l < st.length && st[l] ? CloseUp.STREET : c;
  }

  private static float[] metres(Feed.Tile tile, float[] pts, int from, int to, double ax, double ay, double mpt) {
    float[] out = new float[2 * Math.max(0, to - from)];
    for (int i = from; i < to; i++) {
      out[2 * (i - from)] = (float) (((tile.x + pts[2 * i] / Feed.EXTENT) - ax) * mpt);
      out[2 * (i - from) + 1] = (float) (-((tile.y + pts[2 * i + 1] / Feed.EXTENT) - ay) * mpt);
    }
    return out;
  }

  // ---- software drawing

  private final Path path = new Path();
  private final Paint paint = new Paint(Paint.ANTI_ALIAS_FLAG);
  private float t0, t1;
  int segsDrawn;  // the last frame's, for the harness

  CloseView() {
    paint.setStyle(Paint.Style.STROKE);
    paint.setStrokeJoin(Paint.Join.ROUND);
    paint.setStrokeCap(Paint.Cap.ROUND);
  }

  // Into the canvas, which has the camera on it: each pass a path in the car's frame.
  void draw(final Canvas canvas, final CloseUp.Detail d, final float ce, final float cn, final float ch, final float sh,
            final float ik, float gr, float gw) {
    segsDrawn = 0;
    if (d == null || d.count() == 0 || (gr <= 0 && gw <= 0)) return;
    passes(d, ik, gr, gw, pass(canvas, d, ce, cn, ch, sh, ik));
  }

  // The rail lines (rails()), before the roads.
  void rails(Canvas canvas, CloseUp.Detail d, float ce, float cn, float ch, float sh, float ik, float gr) {
    if (d == null || d.count() == 0 || gr <= 0) return;
    rails(d, gr, pass(canvas, d, ce, cn, ch, sh, ik));
  }

  private Pass pass(final Canvas canvas, final CloseUp.Detail d, final float ce, final float cn, final float ch,
                    final float sh, final float ik) {
    return new Pass() {
      @Override
      public void lines(int r, float hs, float width, int color, float alpha, float on, float period) {
        if (d.empty(r) || alpha <= 0) return;
        float s = build(d, r, hs, on, period, ce, cn, ch, sh, ik);
        if (path.isEmpty()) return;
        paint.setColor(color);
        paint.setAlpha(Math.round(255 * Math.min(1, alpha)));
        paint.setStrokeWidth(width * s);
        canvas.drawPath(path, paint);
      }
    };
  }

  // Range r into path, each point lifted by hs of its height; returns the width factor at the highest point.
  private float build(CloseUp.Detail d, int r, float hs, float on, float period, float ce, float cn, float ch, float sh,
                      float ik) {
    path.rewind();
    float[] v = d.seg;
    float top = 1, lx = Float.NaN, lz = Float.NaN;
    for (int i = d.from[r]; i < d.from[r + 1]; i++) {
      int o = i * CloseUp.SEG;
      float de = v[o] - ce, dn = v[o + 1] - cn;
      float ax = (de * ch - dn * sh) * ik, az = (de * sh + dn * ch) * ik;
      de = v[o + 2] - ce;
      dn = v[o + 3] - cn;
      float bx = (de * ch - dn * sh) * ik, bz = (de * sh + dn * ch) * ik;
      if (hs > 0) {
        float sa = CloseUp.lift(v[o + 4] * hs * ik), sb = CloseUp.lift(v[o + 5] * hs * ik);
        top = Math.max(top, Math.max(sa, sb));
        ax *= sa;
        az = CloseUp.CAM_ZC + (az - CloseUp.CAM_ZC) * sa;
        bx *= sb;
        bz = CloseUp.CAM_ZC + (bz - CloseUp.CAM_ZC) * sb;
      }
      if ((ax < MapView.BOX_X0 && bx < MapView.BOX_X0) || (ax > MapView.BOX_X1 && bx > MapView.BOX_X1)
          || (az < MapView.BOX_Z0 && bz < MapView.BOX_Z0) || (az > MapView.BOX_Z1 && bz > MapView.BOX_Z1)) {
        lx = Float.NaN;
        continue;
      }
      segsDrawn++;
      if (period <= 0) {
        boolean joined = ax == lx && az == lz;
        if (clip(ax, az, bx, bz)) {
          float dx = bx - ax, dz = bz - az;
          if (!joined || t0 > 0) path.moveTo(ax + t0 * dx, az + t0 * dz);
          path.lineTo(ax + t1 * dx, az + t1 * dz);
          lx = t1 >= 1 ? bx : Float.NaN;
          lz = bz;
        } else {
          lx = Float.NaN;
        }
        continue;
      }
      // the dashes: [k period, k period + on] along the line, over this segment's [at, at + len]
      float at = v[o + 6], len = (float) Math.hypot(v[o + 2] - v[o], v[o + 3] - v[o + 1]);
      if (len <= 0) continue;
      for (float k = (float) Math.floor(at / period) * period; k < at + len; k += period) {
        float u0 = Math.max(0, (k - at) / len), u1 = Math.min(1, (k + on - at) / len);
        if (u1 <= u0) continue;
        float x0 = ax + (bx - ax) * u0, z0 = az + (bz - az) * u0, x1 = ax + (bx - ax) * u1, z1 = az + (bz - az) * u1;
        if (!clip(x0, z0, x1, z1)) continue;
        path.moveTo(x0 + t0 * (x1 - x0), z0 + t0 * (z1 - z0));
        path.lineTo(x0 + t1 * (x1 - x0), z0 + t1 * (z1 - z0));
      }
    }
    return top;
  }

  // Liang-Barsky against the view's box, as MapView.segment: t0, t1 the part inside
  private boolean clip(float x0, float z0, float x1, float z1) {
    float dx = x1 - x0, dz = z1 - z0;
    t0 = 0;
    t1 = 1;
    return edge(-dx, x0 - MapView.BOX_X0) && edge(dx, MapView.BOX_X1 - x0) && edge(-dz, z0 - MapView.BOX_Z0)
        && edge(dz, MapView.BOX_Z1 - z0);
  }

  private boolean edge(float p, float q) {
    if (p == 0) return q >= 0;
    float r = q / p;
    if (p < 0) {
      if (r > t1) return false;
      if (r > t0) t0 = r;
    } else {
      if (r < t0) return false;
      if (r < t1) t1 = r;
    }
    return true;
  }
}
