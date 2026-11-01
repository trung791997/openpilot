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
  // the deck's side wall: the deck drawn dark at these fractions of its height under it (a quarter apart, so a thin
  // link ramp two layers up still shows one wall, not stripes)
  static final float[] WALL_AT = {0, 0.25f, 0.5f, 0.75f};

  // One pass: range r of the detail at hs of its height, dashed (on, period world m) or solid (period 0).
  interface Pass {
    void lines(int r, float hs, float width, int color, float alpha, float on, float period);
  }

  // The passes, in order, for a frame with gate gr (roads, bridges, lanes) and gw (walks): the walks, the ground's
  // lane lines, the decks' shadows, then each layer group from the lowest: its walls, casing, kerb, surface and lanes.
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
        p.lines(CloseUp.range(g, c), 0, w + 2 * MapView.dm(SHADOW_PX), SHADOW_C, SHADOW_A * gr, 0, 0);
        p.lines(CloseUp.range(g, c), 0, w + MapView.dm(SHADOW_PX), SHADOW_C, SHADOW_A * gr, 0, 0);
      }
    }
    for (int g = 1; g < CloseUp.LAYERS; g++) {
      for (float at : WALL_AT) {
        for (int c : MapView.DRAW_ORDER) {
          p.lines(CloseUp.range(g, c), at * gr, Math.max(floor, MapView.ROAD_M[c] * ik) + casing, WALL_C, gr, 0, 0);
        }
      }
      for (int c : MapView.DRAW_ORDER) {
        p.lines(CloseUp.range(g, c), gr, Math.max(floor, MapView.ROAD_M[c] * ik) + casing, MapView.CASING_C, gr, 0, 0);
      }
      for (int c : MapView.DRAW_ORDER) {
        p.lines(CloseUp.range(g, c), gr, Math.max(floor, MapView.ROAD_M[c] * ik) + kerb, MapView.KERBS[c], 1, 0, 0);
      }
      for (int c : MapView.DRAW_ORDER) {
        p.lines(CloseUp.range(g, c), gr, Math.max(floor, MapView.ROAD_M[c] * ik), MapView.ROADS[c], 1, 0, 0);
      }
      lanes(g, gr, lane, gr, p);
    }
  }

  private static void lanes(int g, float hs, float lane, float gr, Pass p) {
    p.lines(CloseUp.range(g, CloseUp.DASH), hs, lane, LANE_C, LANE_A * gr, CloseUp.DASH_ON_M, CloseUp.DASH_PERIOD_M);
    p.lines(CloseUp.range(g, CloseUp.EDGE), hs, lane, LANE_C, LANE_A * gr, 0, 0);
    p.lines(CloseUp.range(g, CloseUp.YELLOW), hs, lane, YELLOW_C, LANE_A * gr, 0, 0);
  }

  // The scene's detail: the roads and walks of the tiles in metres from the anchor (ax, ay tile units, mpt metres per
  // tile unit), as MapView.free() places them.
  static CloseUp.Detail detail(Iterable<Feed.Tile> tiles, double ax, double ay, double mpt) {
    CloseUp.Lines in = new CloseUp.Lines();
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
          // lines that can show: bridges, motorways (lanes) and what may meet a bridge (only ends are looked at, but
          // a ramp needs the line)
          in.road(metres(tile, pts, starts[l], starts[l + 1], ax, ay, mpt), c, br,
                  layer != null && l < layer.length ? layer[l] : 0, lanes != null && l < lanes.length ? lanes[l] & 0xff : 0,
                  oneway != null && l < oneway.length && oneway[l]);
        }
      }
      int[] ws = tile.walkStarts;
      for (int l = 0; l + 1 < ws.length; l++) {
        in.walk(metres(tile, tile.walkPts, ws[l], ws[l + 1], ax, ay, mpt), l < tile.walkKind.length ? tile.walkKind[l] : 0);
      }
    }
    return CloseUp.build(in);
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
    passes(d, ik, gr, gw, new Pass() {
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
    });
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
