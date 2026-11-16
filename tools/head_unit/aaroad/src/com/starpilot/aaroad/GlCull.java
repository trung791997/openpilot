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

  // Pieces of a group are laid out in the buffer in Morton (Z-curve) order of their centres, so the pieces in view,
  // a patch of ground, fall into few runs of neighbours (2026-11-08, the I-35 drive: 300 to 500 draw calls a frame
  // in the dense stretches when the roads were laid out run by run). Off: the order the scene had them.
  static boolean MORTON = true;
  static final float MORTON_CELL = 64;  // metres per step of the Z-curve's grid
  // Runs of one group closer than this many quads are drawn as one, the quads between them included: a draw call
  // costs the Tegra more than a few hundred quads the GPU clips (0: off). The quads between lie outside the view
  // box (the cull rejected them), so they draw nothing that was not already off the screen.
  static int GAP = 0;
  // The cull runs at most every TICK_MS (update()), on the view box widened by how far the camera can move in that
  // time; update() runs it sooner if the camera has left that margin.
  static final long TICK_MS = 100;
  static final float MARGIN_MIN = 3, MARGIN_MAX = 90;  // display metres

  final Object scene;  // the MapView.Scene this was made from
  int band;  // the Douglas-Peucker band (Perf.DpBand) the road pieces were simplified at when packed; GlRoads sets it
  final int groups, count, segs;
  final int[] gFrom;   // pieces of group g: gFrom[g] to gFrom[g + 1]
  final int[] cStart, cCount;  // each piece's first segment in the buffer and its segments
  private final float[] bx0, by0, bx1, by1, cx, cy, cr;  // box, centre, and the radius the old test can reach
  private final float[][] pts;
  private final float[] at;

  // the cull's result: runs of neighbouring pieces, group g's from rFrom[g] to rFrom[g + 1]
  final int[] rStart, rLen, rFrom;
  int rebuilds;  // candidate lists made, for the test and the log
  int ticks, forced, frames;  // update(): culls made, of them early because the camera left the margin, calls
  final float maxDiag;  // the longest diagonal of a piece's box, metres

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
    int[] ids = new int[Math.max(roadPts.length, routePts.length)];
    float diag = 0;
    for (int g = 0; g < groups; g++) {
      gFrom[g] = k;
      boolean isRoute = g >= route;
      float[][] p = isRoute ? routePts : roadPts, b = isRoute ? routeBox : roadBox;
      float[] a = isRoute ? routeAt : roadAt;
      int m = 0;
      for (int i = 0; i < p.length; i++) {
        if (!isRoute && roadCls[i] != g) continue;
        if (isRoute && (routeGroup == null ? 0 : routeGroup[i]) != g - route) continue;
        ids[m++] = i;
      }
      if (MORTON && m > 1) morton(ids, m, b);
      for (int j = 0; j < m; j++) {
        int i = ids[j];
        float[] bb = b[i];
        bx0[k] = bb[0];
        by0[k] = bb[1];
        bx1[k] = bb[2];
        by1[k] = bb[3];
        cx[k] = (bb[0] + bb[2]) / 2;
        cy[k] = (bb[1] + bb[3]) / 2;
        // the box's corners lie within half its diagonal of the centre; the old test takes the square round those
        // corners in the car's frame, which lies within that times root 2
        float d = (float) Math.hypot(bb[2] - bb[0], bb[3] - bb[1]);
        if (d > diag) diag = d;
        cr[k] = d / 2 * ROOT2;
        pts[k] = p[i];
        at[k] = a[i];
        cStart[k] = sg;
        cCount[k] = p[i].length / 2 - 1;
        sg += cCount[k++];
      }
    }
    maxDiag = diag;
    gFrom[groups] = k;
    segs = sg;
    rStart = new int[n];
    rLen = new int[n];
    rFrom = new int[groups + 1];
    cand = new int[n];
    candFrom = new int[groups + 1];
  }

  // ids[0..m) sorted by the Z-curve key of their boxes' centres (ties in their given order).
  private static void morton(int[] ids, int m, float[][] box) {
    float minE = Float.MAX_VALUE, minN = Float.MAX_VALUE;
    for (int j = 0; j < m; j++) {
      float[] bb = box[ids[j]];
      minE = Math.min(minE, (bb[0] + bb[2]) / 2);
      minN = Math.min(minN, (bb[1] + bb[3]) / 2);
    }
    long[] key = new long[m];
    for (int j = 0; j < m; j++) {
      float[] bb = box[ids[j]];
      int qe = Math.min(65535, (int) (((bb[0] + bb[2]) / 2 - minE) / MORTON_CELL));
      int qn = Math.min(65535, (int) (((bb[1] + bb[3]) / 2 - minN) / MORTON_CELL));
      key[j] = ((long) (spread(qe) | spread(qn) << 1) << 32) | j;
    }
    java.util.Arrays.sort(key);
    int[] old = new int[m];
    System.arraycopy(ids, 0, old, 0, m);
    for (int j = 0; j < m; j++) ids[j] = old[(int) key[j]];
  }

  // The 16 bits of v with a zero between each.
  private static int spread(int v) {
    v = (v | v << 8) & 0x00ff00ff;
    v = (v | v << 4) & 0x0f0f0f0f;
    v = (v | v << 2) & 0x33333333;
    v = (v | v << 1) & 0x55555555;
    return v;
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
      if (GAP > 0) {  // runs a few quads apart become one
        int w = rFrom[g];
        for (int q = rFrom[g]; q < r; q++) {
          if (w > rFrom[g] && rStart[q] - (rStart[w - 1] + rLen[w - 1]) <= GAP) {
            rLen[w - 1] = rStart[q] + rLen[q] - rStart[w - 1];
          } else {
            rStart[w] = rStart[q];
            rLen[w++] = rLen[q];
          }
        }
        r = w;
      }
    }
    rFrom[groups] = r;
  }

  // The cull for this frame's camera, at most every TICK_MS (Peter's drive 2026-11-08: 15 to 21 ms a frame of it
  // on I-35 North). The result stays valid for the frames in between because the cull is made on the view box
  // widened by the margin the camera can move in a tick -- the speed and turn and zoom rates since the last cull --
  // and a frame whose camera is already outside that margin culls at once. The margin is exact, not a guess:
  // a point q in the car's frame now is at q0 in the frame at the last cull, |q0 - q| <= ik0 |dpos| +
  // |q| (|ik0 / ik - 1| + chord of the turn), and every corner that decides a piece lies within rho of the car (the
  // box's far corner plus a piece's own size). So whatever the cull for this camera would keep, the stale one
  // kept. Returns whether it culled.
  boolean update(long now, float ce, float cn, float ch, float sh, float ik, float x0, float x1, float z0, float z1) {
    frames++;
    float rho = (float) Math.hypot(Math.max(Math.abs(x0), Math.abs(x1)), Math.max(Math.abs(z0), Math.abs(z1)))
        + 1.5f * maxDiag * Math.max(ik, tIk);
    float de = 0, dn = 0, chord = 0;
    if (tOk) {
      de = ce - tE;
      dn = cn - tN;
      chord = (float) Math.sqrt(Math.max(0, 2 - 2 * (ch * tCh + sh * tSh)));
      if (now - tAt < TICK_MS) {
        float d = tIk * (float) Math.sqrt(de * de + dn * dn) + rho * (Math.abs(tIk / ik - 1) + chord);
        if (d <= tM) return false;
        forced++;
      }
    }
    float m = MARGIN_MIN;
    if (tOk && now > tAt) {
      // the rates over the time since the last cull, for the tick and a frame or two of lateness
      float dt = (now - tAt) / 1000f, t = (TICK_MS + 80) / 1000f;
      float v = (float) Math.sqrt(de * de + dn * dn) / dt, w = chord / dt, z = Math.abs(tIk / ik - 1) / dt;
      m += 1.5f * t * (ik * v + rho * (w + z));
      if (m > MARGIN_MAX) m = MARGIN_MAX;
    }
    cull(ce, cn, ch, sh, ik, x0 - m, x1 + m, z0 - m, z1 + m);
    tOk = true;
    tAt = now;
    tE = ce;
    tN = cn;
    tCh = ch;
    tSh = sh;
    tIk = ik;
    tM = m;
    ticks++;
    return true;
  }

  private boolean tOk;
  private long tAt;
  private float tE, tN, tCh, tSh, tIk, tM;

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
