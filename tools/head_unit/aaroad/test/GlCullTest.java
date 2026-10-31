package com.starpilot.aaroad;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

// Desktop check of GlCull against the cull and upload it replaced (GlRoads before 2026-10-31), on a synthetic scene
// of the size the drive logged: a street grid 8 km across cut into pieces of at most 150 m, and a route through it.
//   javac -d /tmp/glcull src/com/starpilot/aaroad/GlCull.java test/GlCullTest.java
//   java -cp /tmp/glcull com.starpilot.aaroad.GlCullTest
// It fails (exit 1) if any frame's runs or any packed byte differ. Times are this machine's, not the Tegra's.
public class GlCullTest {
  static final int GROUPS = 6, ROUTE = 5;
  static final float BOX_X0 = -380, BOX_X1 = 240, BOX_Z0 = -45, BOX_Z1 = 185;  // MapView's
  static final double R0 = 70.03;

  public static void main(String[] a) {
    Random rnd = new Random(7);
    List<float[]> pts = new ArrayList<float[]>();
    List<Integer> cls = new ArrayList<Integer>();
    // streets every 60 m both ways, each cut in 150 m pieces of 6 points, wobbling a little
    for (int dir = 0; dir < 2; dir++) {
      for (float u = -4000; u <= 4000; u += 60) {
        int c = rnd.nextInt(5);
        for (float v = -4000; v < 4000; v += 150) {
          float[] p = new float[12];
          for (int j = 0; j < 6; j++) {
            float w = v + 30 * j, o = u + (float) rnd.nextGaussian() * 2;
            p[2 * j] = dir == 0 ? w : o;
            p[2 * j + 1] = dir == 0 ? o : w;
          }
          pts.add(p);
          cls.add(c);
        }
      }
    }
    float[][] rp = pts.toArray(new float[0][]);
    int[] rc = new int[rp.length];
    for (int i = 0; i < rc.length; i++) rc[i] = cls.get(i);
    float[] rat = new float[rp.length];
    // the route: a curve across, in 150 m pieces
    List<float[]> route = new ArrayList<float[]>();
    for (int i = 0; i < 53; i++) {
      float[] p = new float[12];
      for (int j = 0; j < 6; j++) {
        double t = (i * 5 + j) * 30.0;
        p[2 * j] = (float) (-4000 + t);
        p[2 * j + 1] = (float) (800 * Math.sin(t / 1500));
      }
      route.add(p);
    }
    float[][] up = route.toArray(new float[0][]);
    float[] uat = new float[up.length];
    for (int i = 0; i < uat.length; i++) uat[i] = i * 150;
    float[][] rb = boxes(rp), ub = boxes(up);

    Old old = new Old(rp, rb, rc, rat, up, ub, uat);
    GlCull neu = new GlCull(null, ROUTE, 1, rp, rb, rc, rat, up, ub, null, uat);
    System.out.println("pieces " + neu.count + ", segments " + neu.segs + ", " + neu.bytes() / 1024 + " KB");
    int bad = 0;

    // packing: the same bytes as the old upload
    ByteBuffer ob = ByteBuffer.allocateDirect(neu.bytes()).order(ByteOrder.nativeOrder());
    ByteBuffer nb = ByteBuffer.allocateDirect(neu.bytes()).order(ByteOrder.nativeOrder());
    long t0 = System.nanoTime();
    old.pack(ob);
    long t1 = System.nanoTime();
    neu.pack(nb, new float[4088]);
    long t2 = System.nanoTime();
    int byteDiff = 0;
    for (int i = 0; i < neu.bytes(); i++) if (ob.get(i) != nb.get(i)) byteDiff++;
    System.out.println("pack old " + ms(t1 - t0) + " ms, new " + ms(t2 - t1) + " ms, bytes differing " + byteDiff);
    if (byteDiff != 0) bad++;

    // driving: a car along the route at 10 to 35 m/s for 120 s at 30 frames a second, the zoom moving with speed,
    // then random places, headings and zooms
    int frames = 0, frameDiff = 0;
    long oldNs = 0, newNs = 0, newMax = 0, oldMax = 0;
    double along = 0;
    for (int f = 0; f < 3600 + 4000; f++) {
      float ce, cn, ch, sh, ik;
      if (f < 3600) {
        double spd = 10 + 25 * (0.5 + 0.5 * Math.sin(f / 600.0));
        along += spd / 30;
        double t = along, e = -4000 + t, n = 800 * Math.sin(t / 1500), h = Math.atan2(1, 800 / 1500.0 * Math.cos(t / 1500));
        ce = (float) e;
        cn = (float) n;
        ch = (float) Math.cos(h);
        sh = (float) Math.sin(h);
        ik = (float) (R0 / Math.max(200, Math.min(600, spd * 20)));
      } else {
        ce = (rnd.nextFloat() - 0.5f) * 6000;
        cn = (rnd.nextFloat() - 0.5f) * 6000;
        double h = rnd.nextDouble() * 2 * Math.PI;
        ch = (float) Math.cos(h);
        sh = (float) Math.sin(h);
        ik = (float) (R0 / (120 + rnd.nextDouble() * 480));
      }
      long a0 = System.nanoTime();
      old.cull(ce, cn, ch, sh, ik);
      long a1 = System.nanoTime();
      neu.cull(ce, cn, ch, sh, ik, BOX_X0, BOX_X1, BOX_Z0, BOX_Z1);
      long a2 = System.nanoTime();
      if (f == 3600) {
        System.out.println("driving " + (frames) + " frames: old avg " + us(oldNs / frames) + " us max " + us(oldMax)
            + ", new avg " + us(newNs / frames) + " us max " + us(newMax) + "; candidate lists " + neu.rebuilds
            + ", candidates now " + neu.candidateCount());
      }
      if (f > 100) {  // after the JIT has warmed
        oldNs += a1 - a0;
        newNs += a2 - a1;
        oldMax = Math.max(oldMax, a1 - a0);
        newMax = Math.max(newMax, a2 - a1);
        frames++;
      }
      boolean same = true;
      for (int g = 0; g <= GROUPS && same; g++) same = old.rFrom[g] == neu.rFrom[g];
      for (int r = 0; r < old.rFrom[GROUPS] && same; r++) same = old.rStart[r] == neu.rStart[r] && old.rLen[r] == neu.rLen[r];
      if (!same) frameDiff++;
    }
    System.out.println("all, with 4000 random jumps, " + frames + " frames: old avg " + us(oldNs / frames) + " us max " + us(oldMax)
        + ", new avg " + us(newNs / frames) + " us max " + us(newMax) + "; candidate lists " + neu.rebuilds
        + "; frames whose runs differ " + frameDiff);
    if (frameDiff != 0) bad++;
    System.out.println(bad == 0 ? "PASS" : "FAIL");
    if (bad != 0) System.exit(1);
  }

  static String ms(long ns) {
    return String.valueOf(Math.round(ns / 1e5) / 10.0);
  }

  static String us(long ns) {
    return String.valueOf(Math.round(ns / 1e2) / 10.0);
  }

  static float[][] boxes(float[][] runs) {
    float[][] b = new float[runs.length][];
    for (int r = 0; r < runs.length; r++) {
      float x0 = Float.MAX_VALUE, y0 = Float.MAX_VALUE, x1 = -Float.MAX_VALUE, y1 = -Float.MAX_VALUE;
      float[] p = runs[r];
      for (int i = 0; i + 1 < p.length; i += 2) {
        x0 = Math.min(x0, p[i]);
        x1 = Math.max(x1, p[i]);
        y0 = Math.min(y0, p[i + 1]);
        y1 = Math.max(y1, p[i + 1]);
      }
      b[r] = new float[] {x0, y0, x1, y1};
    }
    return b;
  }

  // GlRoads' upload() and cull() as they were, on plain arrays.
  static final class Old {
    final float[][] cBox, cPts;
    final float[] cAt;
    final int[] cStart, cCount, gFrom = new int[GROUPS + 1], rStart, rLen, rFrom = new int[GROUPS + 1];

    Old(float[][] rp, float[][] rb, int[] rc, float[] rat, float[][] up, float[][] ub, float[] uat) {
      int n = 0;
      for (int c : rc) if (c >= 0 && c < ROUTE) n++;
      n += up.length;
      cBox = new float[n][];
      cPts = new float[n][];
      cAt = new float[n];
      cStart = new int[n];
      cCount = new int[n];
      int k = 0, segs = 0;
      for (int g = 0; g < GROUPS; g++) {
        gFrom[g] = k;
        float[][] p = g == ROUTE ? up : rp, b = g == ROUTE ? ub : rb;
        float[] at = g == ROUTE ? uat : rat;
        for (int i = 0; i < p.length; i++) {
          if (g != ROUTE && rc[i] != g) continue;
          cBox[k] = b[i];
          cPts[k] = p[i];
          cAt[k] = at[i];
          cStart[k] = segs;
          cCount[k] = p[i].length / 2 - 1;
          segs += cCount[k++];
        }
      }
      gFrom[GROUPS] = k;
      rStart = new int[k];
      rLen = new int[k];
    }

    void pack(ByteBuffer bytes) {
      FloatBuffer fb = bytes.asFloatBuffer();
      float[] tmp = new float[4 * 7];
      for (int i = 0; i < cPts.length; i++) {
        float[] q = cPts[i];
        double along = cAt[i];
        for (int j = 0; j + 3 < q.length; j += 2) {
          for (int c = 0; c < 4; c++) {
            int o = c * 7;
            tmp[o] = q[j];
            tmp[o + 1] = q[j + 1];
            tmp[o + 2] = q[j + 2];
            tmp[o + 3] = q[j + 3];
            tmp[o + 4] = (c & 1) == 0 ? -1 : 1;
            tmp[o + 5] = c >> 1;
            tmp[o + 6] = (float) along;
          }
          fb.put(tmp);
          along += Math.hypot(q[j + 2] - q[j], q[j + 3] - q[j + 1]);
        }
      }
    }

    void cull(float ce, float cn, float ch, float sh, float ik) {
      int r = 0;
      for (int g = 0; g < GROUPS; g++) {
        rFrom[g] = r;
        for (int i = gFrom[g]; i < gFrom[g + 1]; i++) {
          float[] b = cBox[i];
          float x0 = Float.MAX_VALUE, x1 = -Float.MAX_VALUE, z0 = Float.MAX_VALUE, z1 = -Float.MAX_VALUE;
          for (int c = 0; c < 4; c++) {
            float de = b[c == 0 || c == 3 ? 0 : 2] - ce, dn = b[c < 2 ? 1 : 3] - cn;
            float x = (de * ch - dn * sh) * ik, z = (de * sh + dn * ch) * ik;
            x0 = Math.min(x0, x);
            x1 = Math.max(x1, x);
            z0 = Math.min(z0, z);
            z1 = Math.max(z1, z);
          }
          if (x1 < BOX_X0 || x0 > BOX_X1 || z1 < BOX_Z0 || z0 > BOX_Z1) continue;
          if (r > rFrom[g] && rStart[r - 1] + rLen[r - 1] == cStart[i]) {
            rLen[r - 1] += cCount[i];
          } else {
            rStart[r] = cStart[i];
            rLen[r++] = cCount[i];
          }
        }
      }
      rFrom[GROUPS] = r;
    }
  }
}
