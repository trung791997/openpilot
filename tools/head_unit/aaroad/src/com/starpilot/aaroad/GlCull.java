package com.starpilot.aaroad;

import java.nio.ByteBuffer;
import java.nio.FloatBuffer;

// The GPU layer's copy of a scene: where each piece's quads sit in the vertex buffer, the quads themselves, and the
// test of which pieces are in view. Plain Java with no GL or Android in it, so the desktop test (test/GlCullTest.java)
// can hold the cull against the old one and time both.
//
// Peter's drive 2026-10-31 logged cull at 46-49 ms a frame: the old test turned all four corners of every piece's box
// into the car's frame, 16 Math.min/max calls a piece (not inlined on Dalvik), for some tens of thousands of pieces.
// Now each piece has a centre and a radius, and the pieces whose circle comes near a circle round the view box are
// kept as candidates, from a circle SLACK times wider, so the list holds until the view has moved out of it. Each
// frame a candidate's centre is turned into the car's frame and dropped if its reach misses the view box; the old
// corner test, written without the calls, runs on the rest, in buffer order, so the runs drawn are the same as before.
final class GlCull {
  static final int FLOATS = 7, STRIDE = FLOATS * 4;  // per corner: a.e, a.n, b.e, b.n, across, end, along at a
  // display metres past the view box's circle; more than half the widest road plus the float error of the tests
  static final float MARGIN = 60;
  static final float SLACK = 1.3f;  // the candidates' circle against the view's: rebuilt once the view moves 0.3 of it
  static final float PAD = 1;  // display metres on the centre test, for its float error against the corner test
  private static final float ROOT2 = (float) Math.sqrt(2);

  final Object scene;  // the MapView.Scene this was made from
  final int groups, count, segs;
  final int[] gFrom;   // pieces of group g: gFrom[g] to gFrom[g + 1]
  final int[] cStart, cCount;  // each piece's first segment in the buffer and its segments
  private final float[] bx0, by0, bx1, by1, cx, cy, cr;  // box, centre, and the radius the old test can reach
  private final float[][] pts;
  private final float[] at;

  // the cull's result: runs of neighbouring pieces, group g's from rFrom[g] to rFrom[g + 1]
  final int[] rStart, rLen, rFrom;
  int rebuilds;  // candidate lists made, for the test

  private final int[] cand, candFrom;
  private boolean candOk;
  private float candE, candN, candR;

  // Pieces in the order drawn: groups 0 to roadGroups - 1 the roads of that class, then routeGroups of the route,
  // route piece i in group roadGroups + routeGroup[i] (all in the first if routeGroup is null).
  GlCull(Object scene, int roadGroups, int routeGroups, float[][] roadPts, float[][] roadBox, int[] roadCls,
         float[] roadAt, float[][] routePts, float[][] routeBox, int[] routeGroup, float[] routeAt) {
    this.scene = scene;
    this.groups = roadGroups + routeGroups;
    int route = roadGroups;
    int n = 0;
    for (int i = 0; i < roadPts.length; i++) if (roadCls[i] >= 0 && roadCls[i] < route) n++;
    n += routePts.length;
    count = n;
    gFrom = new int[groups + 1];
    cStart = new int[n];
    cCount = new int[n];
    bx0 = new float[n];
    by0 = new float[n];
    bx1 = new float[n];
    by1 = new float[n];
    cx = new float[n];
    cy = new float[n];
    cr = new float[n];
    pts = new float[n][];
    at = new float[n];
    int k = 0, sg = 0;
    for (int g = 0; g < groups; g++) {
      gFrom[g] = k;
      boolean isRoute = g >= route;
      float[][] p = isRoute ? routePts : roadPts, b = isRoute ? routeBox : roadBox;
      float[] a = isRoute ? routeAt : roadAt;
      for (int i = 0; i < p.length; i++) {
        if (!isRoute && roadCls[i] != g) continue;
        if (isRoute && (routeGroup == null ? 0 : routeGroup[i]) != g - route) continue;
        float[] bb = b[i];
        bx0[k] = bb[0];
        by0[k] = bb[1];
        bx1[k] = bb[2];
        by1[k] = bb[3];
        cx[k] = (bb[0] + bb[2]) / 2;
        cy[k] = (bb[1] + bb[3]) / 2;
        // the box's corners lie within half its diagonal of the centre; the old test takes the square round those
        // corners in the car's frame, which lies within that times root 2
        cr[k] = (float) Math.hypot(bb[2] - bb[0], bb[3] - bb[1]) / 2 * ROOT2;
        pts[k] = p[i];
        at[k] = a[i];
        cStart[k] = sg;
        cCount[k] = p[i].length / 2 - 1;
        sg += cCount[k++];
      }
    }
    gFrom[groups] = k;
    segs = sg;
    rStart = new int[n];
    rLen = new int[n];
    rFrom = new int[groups + 1];
    cand = new int[n];
    candFrom = new int[groups + 1];
  }

  int bytes() {
    return segs * 4 * STRIDE;
  }

  // Every segment as one quad of four corners into b from 0, a few thousand floats at a time.
  void pack(ByteBuffer b, float[] tmp) {
    FloatBuffer fb = b.asFloatBuffer();
    int per = 4 * FLOATS, room = tmp.length / per * per, o = 0;
    for (int i = 0; i < count; i++) {
      float[] q = pts[i];
      double along = at[i];
      for (int j = 0; j + 3 < q.length; j += 2) {
        if (o == room) {
          fb.put(tmp, 0, o);
          o = 0;
        }
        float ae = q[j], an = q[j + 1], be = q[j + 2], bn = q[j + 3], al = (float) along;
        for (int c = 0; c < 4; c++) {
          tmp[o] = ae;
          tmp[o + 1] = an;
          tmp[o + 2] = be;
          tmp[o + 3] = bn;
          tmp[o + 4] = (c & 1) == 0 ? -1 : 1;
          tmp[o + 5] = c >> 1;
          tmp[o + 6] = al;
          o += FLOATS;
        }
        double de = be - ae, dn = bn - an;
        along += Math.sqrt(de * de + dn * dn);
      }
    }
    fb.put(tmp, 0, o);
  }

  // The pieces in view of a car at (ce, cn) heading (ch, sh) at zoom ik, the view box x0..x1, z0..z1 in its frame.
  void cull(float ce, float cn, float ch, float sh, float ik, float x0, float x1, float z0, float z1) {
    float k = 1 / ik, mx = (x0 + x1) / 2, mz = (z0 + z1) / 2;
    float rv = (float) Math.hypot(x1 - x0, z1 - z0) / 2 + MARGIN;
    // the view circle's centre back in the world: de = (x ch + z sh) k, dn = (z ch - x sh) k
    float ve = ce + (mx * ch + mz * sh) * k, vn = cn + (mz * ch - mx * sh) * k, vr = rv * k;
    float me = ve - candE, mn = vn - candN;
    if (!candOk || (float) Math.sqrt(me * me + mn * mn) + vr > candR) candidates(ve, vn, vr * SLACK);
    int r = 0;
    for (int g = 0; g < groups; g++) {
      rFrom[g] = r;
      for (int c = candFrom[g]; c < candFrom[g + 1]; c++) {
        int i = cand[c];
        // the centre in the car's frame: a piece whose reach round it misses the box cannot pass the old test
        float de = cx[i] - ce, dn = cy[i] - cn, rr = cr[i] * ik + PAD;
        float x = (de * ch - dn * sh) * ik;
        if (x + rr < x0 || x - rr > x1) continue;
        float z = (de * sh + dn * ch) * ik;
        if (z + rr < z0 || z - rr > z1) continue;
        // the old test: the box's four corners in the car's frame, and the box round them against the view box
        float e0 = bx0[i] - ce, e1 = bx1[i] - ce, n0 = by0[i] - cn, n1 = by1[i] - cn;
        float xa = (e0 * ch - n0 * sh) * ik, xb = (e1 * ch - n0 * sh) * ik, xc = (e1 * ch - n1 * sh) * ik, xd = (e0 * ch - n1 * sh) * ik;
        float lo = xa, hi = xa;
        if (xb < lo) lo = xb; else if (xb > hi) hi = xb;
        if (xc < lo) lo = xc; else if (xc > hi) hi = xc;
        if (xd < lo) lo = xd; else if (xd > hi) hi = xd;
        if (hi < x0 || lo > x1) continue;
        float za = (e0 * sh + n0 * ch) * ik, zb = (e1 * sh + n0 * ch) * ik, zc = (e1 * sh + n1 * ch) * ik, zd = (e0 * sh + n1 * ch) * ik;
        lo = za;
        hi = za;
        if (zb < lo) lo = zb; else if (zb > hi) hi = zb;
        if (zc < lo) lo = zc; else if (zc > hi) hi = zc;
        if (zd < lo) lo = zd; else if (zd > hi) hi = zd;
        if (hi < z0 || lo > z1) continue;
        if (r > rFrom[g] && rStart[r - 1] + rLen[r - 1] == cStart[i]) {
          rLen[r - 1] += cCount[i];
        } else {
          rStart[r] = cStart[i];
          rLen[r++] = cCount[i];
        }
      }
    }
    rFrom[groups] = r;
  }

  // Every piece whose circle comes within R of (e, n), in buffer order.
  private void candidates(float e, float n, float rad) {
    int m = 0;
    for (int g = 0; g < groups; g++) {
      candFrom[g] = m;
      for (int i = gFrom[g]; i < gFrom[g + 1]; i++) {
        float de = cx[i] - e, dn = cy[i] - n, lim = rad + cr[i];
        if (de * de + dn * dn <= lim * lim) cand[m++] = i;
      }
    }
    candFrom[groups] = m;
    candE = e;
    candN = n;
    candR = rad;
    candOk = true;
    rebuilds++;
  }

  int candidateCount() {
    return candFrom[groups];
  }
}
