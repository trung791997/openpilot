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
  static final long FADE_MS = 150;  // the outline eases over this at or under FADE_REF_SPD, so a deck's edge does
  // not flicker
  // Fade scales down with speed (2026-11-07, Peter: a pass under an overpass at highway speed is itself under a
  // second, so a flat 150 ms ease either side ate most of it): FADE_MS still applies at and under FADE_REF_SPD,
  // falling straight to FADE_MIN_MS by about 28 m/s (100 km/h) and staying there above it. *static*: Peter's ask
  // (the two endpoints are his numbers), not measured against a real pass under a deck.
  static final long FADE_MIN_MS = 80;
  static final double FADE_REF_SPD = 15;  // m/s (54 km/h)
  // a covering segment is cut to this many display metres each side of the car before it is drawn as a clip (the
  // rest is off the car, and a long segment's far end may sit behind the camera)
  static final float NEAR_DM = 40;

  // The outline's ease time at speed spdMps (m/s, caller already clamps it to 0 or more):
  // clamp(FADE_MS * FADE_REF_SPD / spdMps, FADE_MIN_MS, FADE_MS).
  static long fadeMs(double spdMps) {
    if (spdMps <= FADE_REF_SPD) return FADE_MS;
    return Math.round(Math.max(FADE_MIN_MS, FADE_MS * FADE_REF_SPD / spdMps));
  }

  private static final float COS_ON = (float) Math.cos(Math.toRadians(ON_DECK_DEG));

  // The car's height, world m: among the roads running its way (within ON_DECK_DEG) that hold it (half the road's
  // width plus ON_DECK_M), the one whose centreline is nearest the car wins, ground (Detail.ground: plain roads and
  // a ramp's own ground tail, height 0) or raised (a deck's own range) alike; a tie keeps the higher one (the old
  // rule, when only raised candidates competed). No candidate at all is the ground, height 0. (ce, cn) the car in
  // metres from the anchor, (sh, ch) the way it points (east, north), roadM each class's width in world m.
  static float carHeight(CloseUp.Detail d, float ce, float cn, float ch, float sh, float[] roadM) {
    if (d == null) return 0;
    float hc = 0, bestD2 = Float.POSITIVE_INFINITY;
    float[] g = d.ground;
    for (int i = 0; i < d.groundSegs; i++) {
      int o = i * CloseUp.GROUND_SEG;
      int c = (int) g[o + 4];
      float reach = roadM[c] / 2 + ON_DECK_M;
      // cheap bbox reject before the dot products (2026-11-07, measured on a large synthetic scene: see UnderTest)
      if (ce < Math.min(g[o], g[o + 2]) - reach || ce > Math.max(g[o], g[o + 2]) + reach) continue;
      if (cn < Math.min(g[o + 1], g[o + 3]) - reach || cn > Math.max(g[o + 1], g[o + 3]) + reach) continue;
      float dx = g[o + 2] - g[o], dy = g[o + 3] - g[o + 1];
      float len = (float) Math.hypot(dx, dy);
      if (len <= 0) continue;
      if (Math.abs((dx * sh + dy * ch) / len) < COS_ON) continue;  // across the car's way: a deck over it
      float t = Math.max(0, Math.min(1, ((ce - g[o]) * dx + (cn - g[o + 1]) * dy) / (len * len)));
      float ex = g[o] + t * dx - ce, ey = g[o + 1] + t * dy - cn;
      float d2 = ex * ex + ey * ey;
      if (d2 > reach * reach) continue;
      if (d2 < bestD2) bestD2 = d2;  // ground is always height 0; hc stays unless a nearer raised candidate wins
    }
    float[] v = d.seg;
    for (int layer = 1; layer < CloseUp.LAYERS; layer++) {
      for (int k = 0; k < 2 * CloseUp.CLASSES; k++) {
        int c = k % CloseUp.CLASSES;
        int r = k < CloseUp.CLASSES ? CloseUp.range(layer, c) : CloseUp.deck(layer, c);
        float reach = roadM[c] / 2 + ON_DECK_M;
        for (int i = d.from[r]; i < d.from[r + 1]; i++) {
          int o = i * CloseUp.SEG;
          if (ce < Math.min(v[o], v[o + 2]) - reach || ce > Math.max(v[o], v[o + 2]) + reach) continue;
          if (cn < Math.min(v[o + 1], v[o + 3]) - reach || cn > Math.max(v[o + 1], v[o + 3]) + reach) continue;
          float dx = v[o + 2] - v[o], dy = v[o + 3] - v[o + 1];
          float len = (float) Math.hypot(dx, dy);
          if (len <= 0) continue;
          if (Math.abs((dx * sh + dy * ch) / len) < COS_ON) continue;  // across the car's way: a deck over it
          float t = Math.max(0, Math.min(1, ((ce - v[o]) * dx + (cn - v[o + 1]) * dy) / (len * len)));
          float ex = v[o] + t * dx - ce, ey = v[o + 1] + t * dy - cn;
          float d2 = ex * ex + ey * ey;
          if (d2 > reach * reach) continue;
          float h = v[o + 4] + t * (v[o + 5] - v[o + 4]);
          if (d2 < bestD2) {
            bestD2 = d2;
            hc = h;
          } else if (d2 == bestD2) {
            hc = Math.max(hc, h);
          }
        }
      }
    }
    return hc;
  }

  // a raised segment running within this of the route's way is never a deck "over" the route: it is the route's own
  // deck (or one alongside it), not one it passes beneath
  static final float ALONG_DEG = 45;
  private static final float COS_ALONG = (float) Math.cos(Math.toRadians(ALONG_DEG));

  // One point (pe, pn), heading (ch, sh) along a route segment: true if it sits under a deck higher than the
  // road it is itself on by more than OVER_M (own height rh from carHeight, which already keeps only the segment's
  // own way). Split out of underCuts below so a long route edge can be sampled at more than its one midpoint.
  //
  // A raised segment running along the route (within ALONG_DEG) is never over it (Peter 2026-11-08, split
  // harness: on the I-10/I-35 and McDermott connectors the route's own curving deck missed carHeight's ON_DECK_DEG
  // match, its height fell back to the ground, and the deck it rides then hid it). A hidden route line is worse
  // than one drawn over a parallel viaduct. No road found at all still reads as the ground: a minor street the
  // close-up detail leaves out must still go under the highway crossing it.
  private static boolean underPoint(CloseUp.Detail d, float pe, float pn, float ch, float sh) {
    float rh = carHeight(d, pe, pn, ch, sh, CloseUp.ROAD_W);
    float[] v = d.seg;
    for (int layer = 1; layer < CloseUp.LAYERS; layer++) {
      for (int k = 0; k < 2 * CloseUp.CLASSES; k++) {
        int c = k % CloseUp.CLASSES;
        int r = k < CloseUp.CLASSES ? CloseUp.range(layer, c) : CloseUp.deck(layer, c);
        float reach = CloseUp.ROAD_W[c] / 2;
        for (int i = d.from[r]; i < d.from[r + 1]; i++) {
          int o = i * CloseUp.SEG;
          if (Math.max(v[o + 4], v[o + 5]) <= rh + OVER_M) continue;
          if (pe < Math.min(v[o], v[o + 2]) - reach || pe > Math.max(v[o], v[o + 2]) + reach) continue;
          if (pn < Math.min(v[o + 1], v[o + 3]) - reach || pn > Math.max(v[o + 1], v[o + 3]) + reach) continue;
          float sx = v[o + 2] - v[o], sy = v[o + 3] - v[o + 1], len2 = sx * sx + sy * sy;
          if (len2 <= 0) continue;
          if (Math.abs(sx * sh + sy * ch) >= COS_ALONG * (float) Math.sqrt(len2)) continue;  // along the route
          float t = Math.max(0, Math.min(1, ((pe - v[o]) * sx + (pn - v[o + 1]) * sy) / len2));
          float ex = v[o] + t * sx - pe, ey = v[o + 1] + t * sy - pn;
          if (ex * ex + ey * ey <= reach * reach) return true;
        }
      }
    }
    return false;
  }

  // A route edge's own shape points can be far apart (a straight highway stretch may carry only its two ends), so
  // testing only the edge's midpoint can step clean over a deck that crosses a short way into it (Peter 2026-11-08,
  // found on the walk3d close-up harness, where a 1450 m synthetic edge's midpoint missed a crossing 475 m off it).
  // UNDER_STEP_M, a deck's narrowest expected width (two lanes plus shoulders) so no crossing is thin enough to
  // fall between samples: static choice, not road-measured. Capped at UNDER_SAMPLES so one long highway edge can't
  // make the once-per-build classification loop unbounded.
  private static final float UNDER_STEP_M = 8;
  private static final int UNDER_SAMPLES = 256;

  // Whether a route edge passes under a deck higher than the road it is itself on, and where that state changes
  // along it: first is the state at (ae, an); at holds, in ascending metres from (ae, an), each point where it
  // flips, as the midpoint between the last sample of one state and the first of the next -- collinear with the
  // edge, so a caller that splits the edge there doesn't bend the line or move its length.
  static final class Cuts {
    final boolean first;
    final float[] at;
    Cuts(boolean first, float[] at) { this.first = first; this.at = at; }
  }

  // A route edge's own shape points can be far apart (a straight highway stretch may carry only its two ends), so
  // testing only the edge's midpoint can step clean over a deck that crosses a short way into it, or -- the Part A
  // bug (Peter 2026-11-08) -- miss that the edge is only PARTLY under a higher deck: a long straight edge can ride
  // its own overpass through part of its length (not under anything higher than itself there) and duck under a
  // different, higher flyover through another part. underHigherDeck used to collapse that to one flag for the
  // whole edge, so the edge's own deck then painted over the route on the bridge the car is actually driving on.
  // Sampled every UNDER_STEP_M along the edge (its direction is one heading throughout, so carHeight's own-way
  // filter still applies at each sample) instead of once at the midpoint. Called once per route edge at scene
  // build time (MapView.build), not per frame.
  static Cuts underCuts(CloseUp.Detail d, float ae, float an, float be, float bn) {
    float dx = be - ae, dy = bn - an, len = (float) Math.hypot(dx, dy);
    if (d == null || len <= 0) return new Cuts(false, new float[0]);
    float sh = dx / len, ch = dy / len;
    int n = Math.min(UNDER_SAMPLES, Math.max(1, (int) Math.ceil(len / UNDER_STEP_M)));
    boolean first = underPoint(d, ae, an, ch, sh), prev = first;
    java.util.List<Float> cuts = new java.util.ArrayList<Float>();
    for (int i = 1; i <= n; i++) {
      float t = (float) i / n;
      boolean cur = underPoint(d, ae + dx * t, an + dy * t, ch, sh);
      if (cur != prev) cuts.add(((i - 0.5f) / n) * len);
      prev = cur;
    }
    float[] at = new float[cuts.size()];
    for (int i = 0; i < at.length; i++) at[i] = cuts.get(i);
    return new Cuts(first, at);
  }

  // The raised segments drawn over the car's body this frame, into out (segment indices; at most out.length) and
  // their count, at every zoom (2026-11-07: this used to run only in the close-up; now it also finds the flat decks
  // cruising draws). The detail is drawn with heights at hs (the close-up gate, 0 cruising) and widths w(c) =
  // max(floor, roadM[c] ik) + casing display metres, lifted as CloseView does; at hs 0, lifted() scales by 1 (flat),
  // matching the flat footprint cruising draws. The height test against the car (hc + OVER_M) is against the data
  // heights, not hs, so it does not depend on the gate either. carDm is the body's reach in display metres.
  static int covering(CloseUp.Detail d, float ce, float cn, float ch, float sh, float ik, float hs, float[] roadM,
                      float floor, float casing, float carDm, int[] out) {
    if (d == null || d.maxH <= 0) return 0;
    float hc = carHeight(d, ce, cn, ch, sh, roadM);
    if (d.maxH < hc + OVER_M) return 0;
    float[] v = d.seg;
    int n = 0;
    for (int g = 1; g < CloseUp.LAYERS; g++) {
      for (int k = 0; k < 2 * CloseUp.CLASSES; k++) {  // ramps and decks of each class
        int c = k % CloseUp.CLASSES, r = k < CloseUp.CLASSES ? CloseUp.range(g, c) : CloseUp.deck(g, c);
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

  // The 0 to 1 ease of the outline: set() notes the wanted end, level() moves toward it at 1 / ms a millisecond from
  // where it was, so a deck edge crossed back part way turns back part way (as the brake glow). ms is the pace of
  // the ease in flight: set(want, now) keeps whatever pace was last given (FADE_MS until told otherwise);
  // set(want, now, fadeMs) is for the speed-scaled pace (fadeMs(), Peter 2026-11-07) and only takes hold on a new
  // transition, not mid-ease, so an ease already running does not jump.
  static final class Fade {
    private boolean on;
    private float from;
    private long at;
    private long ms = FADE_MS;

    void set(boolean want, long now) {
      set(want, now, ms);
    }

    void set(boolean want, long now, long fadeMs) {
      if (want == on) return;
      from = level(now);
      on = want;
      at = now;
      ms = fadeMs;
    }

    float level(long now) {
      float d = Math.max(0, now - at) / (float) ms;
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
