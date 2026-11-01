package com.starpilot.aaroad;

// The car under an overpass (Peter 2026-11-01: "add like an outline of the car if I go beneath a bridge or overpass
// for some realism effect", as Google Maps' immersive view does it): while the close-up shows its raised decks
// (CloseUp), a deck drawn over the car's place hides the car there, and a light outline of the car shows through it.
//
// Plain Java (no Android) so test/UnderTest.java can check it on the desktop. MapView draws it, the same on the
// software and GPU paths: the GPU layer's map shows through the slow layer, so the deck under a clipped-away car is the
// one the map drew, lane lines and all.
//
// Which deck is the car's: the feed has no layer for the car's road (MapView.currentRoad matches a name only), so the
// car is on a deck when a raised segment running its way (within ON_DECK_DEG) holds it, and on the ground otherwise.
// A deck over it is one at least OVER_M above that, drawn where it reaches the car's body.
final class Under {
  // world m past a deck's half width the car still counts as on it: GPS beside the deck must not show the car under
  // its own bridge
  static final float ON_DECK_M = 6;
  static final float ON_DECK_DEG = 30;  // as MapView's ON_ROAD_DEG
  static final float OVER_M = CloseUp.LAYER_M / 2;  // half a deck: a ramp's low end beside the car is not over it
  static final float CAR_PX = 11;  // the car's body on screen, px from its point: a deck reaching this far covers it
  static final long FADE_MS = 150;  // the outline eases in and out over this, so a deck's edge does not flicker
  // a covering segment is cut to this many display metres each side of the car before it is drawn as a clip (the
  // rest is off the car, and a long segment's far end may sit behind the camera)
  static final float NEAR_DM = 40;

  private static final float COS_ON = (float) Math.cos(Math.toRadians(ON_DECK_DEG));

  // The car's height, world m: the highest raised segment running its way that holds it, 0 on the ground. (ce, cn)
  // the car in metres from the anchor, (sh, ch) the way it points (east, north), roadM each class's width in world m.
  static float carHeight(CloseUp.Detail d, float ce, float cn, float ch, float sh, float[] roadM) {
    float hc = 0;
    if (d == null) return 0;
    float[] v = d.seg;
    for (int g = 1; g < CloseUp.LAYERS; g++) {
      for (int c = 0; c < CloseUp.CLASSES; c++) {
        int r = CloseUp.range(g, c);
        float reach = roadM[c] / 2 + ON_DECK_M;
        for (int i = d.from[r]; i < d.from[r + 1]; i++) {
          int o = i * CloseUp.SEG;
          float dx = v[o + 2] - v[o], dy = v[o + 3] - v[o + 1];
          float len = (float) Math.hypot(dx, dy);
          if (len <= 0) continue;
          if (Math.abs((dx * sh + dy * ch) / len) < COS_ON) continue;  // across the car's way: a deck over it
          float t = Math.max(0, Math.min(1, ((ce - v[o]) * dx + (cn - v[o + 1]) * dy) / (len * len)));
          float ex = v[o] + t * dx - ce, ey = v[o + 1] + t * dy - cn;
          if (ex * ex + ey * ey > reach * reach) continue;
          hc = Math.max(hc, v[o + 4] + t * (v[o + 5] - v[o + 4]));
        }
      }
    }
    return hc;
  }

  // The raised segments drawn over the car's body this frame, into out (segment indices; at most out.length) and
  // their count. The detail is drawn with heights at hs (the close-up gate) and widths w(c) = max(floor, roadM[c] ik)
  // + casing display metres, lifted as CloseView does; carDm is the body's reach in display metres. None at hs 0.
  static int covering(CloseUp.Detail d, float ce, float cn, float ch, float sh, float ik, float hs, float[] roadM,
                      float floor, float casing, float carDm, int[] out) {
    if (d == null || hs <= 0 || d.maxH <= 0) return 0;
    float hc = carHeight(d, ce, cn, ch, sh, roadM);
    if (d.maxH < hc + OVER_M) return 0;
    float[] v = d.seg;
    int n = 0;
    for (int g = 1; g < CloseUp.LAYERS; g++) {
      for (int c = 0; c < CloseUp.CLASSES; c++) {
        int r = CloseUp.range(g, c);
        float half = (Math.max(floor, roadM[c] * ik) + casing) / 2;
        for (int i = d.from[r]; i < d.from[r + 1] && n < out.length; i++) {
          int o = i * CloseUp.SEG;
          if (Math.max(v[o + 4], v[o + 5]) < hc + OVER_M) continue;
          float[] q = lifted(v, o, ce, cn, ch, sh, ik, hs, LIFTED);
          float dx = q[2] - q[0], dz = q[3] - q[1], l2 = dx * dx + dz * dz;
          float t = l2 <= 0 ? 0 : Math.max(0, Math.min(1, -(q[0] * dx + q[1] * dz) / l2));
          float h = v[o + 4] + t * (v[o + 5] - v[o + 4]);
          if (h < hc + OVER_M) continue;
          float s = CloseUp.lift(h * hs * ik);
          float x = q[0] + t * dx, z = q[1] + t * dz, reach = half * s + carDm;
          if (x * x + z * z <= reach * reach) out[n++] = i;
        }
      }
    }
    return n;
  }

  private static final float[] LIFTED = new float[4];  // UI thread only

  // Segment o's two ends in the car's frame, lifted by hs of their heights: [ax, az, bx, bz]
  static float[] lifted(float[] v, int o, float ce, float cn, float ch, float sh, float ik, float hs, float[] q) {
    for (int e = 0; e < 2; e++) {
      float de = v[o + 2 * e] - ce, dn = v[o + 2 * e + 1] - cn;
      float x = (de * ch - dn * sh) * ik, z = (de * sh + dn * ch) * ik;
      float s = CloseUp.lift(v[o + 4 + e] * hs * ik);
      q[2 * e] = x * s;
      q[2 * e + 1] = CloseUp.CAM_ZC + (z - CloseUp.CAM_ZC) * s;
    }
    return q;
  }

  // The 0 to 1 ease of the outline: set() notes the wanted end, level() moves toward it at 1 / FADE_MS a millisecond
  // from where it was, so a deck edge crossed back part way turns back part way (as the brake glow).
  static final class Fade {
    private boolean on;
    private float from;
    private long at;

    void set(boolean want, long now) {
      if (want == on) return;
      from = level(now);
      on = want;
      at = now;
    }

    float level(long now) {
      float d = Math.max(0, now - at) / (float) FADE_MS;
      return on ? Math.min(1f, from + d) : Math.max(0f, from - d);
    }

    boolean on() {
      return on;
    }

    boolean moving(long now) {
      float l = level(now);
      return on ? l < 1 : l > 0;
    }
  }
}
