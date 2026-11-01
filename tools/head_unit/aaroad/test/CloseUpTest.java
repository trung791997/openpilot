package com.starpilot.aaroad;

import java.util.ArrayList;
import java.util.List;

// Desktop check of CloseUp: the gate, the lift, bridge heights and ramps, lane lines, walks and the caps.
//   javac -d /tmp/closeup src/com/starpilot/aaroad/CloseUp.java test/CloseUpTest.java
//   java -cp /tmp/closeup com.starpilot.aaroad.CloseUpTest
// Exits 1 on a failure.
public class CloseUpTest {
  static final float L = CloseUp.LAYER_M;  // one deck's height
  static int bad;

  static void check(boolean ok, String what) {
    System.out.println((ok ? "ok   " : "FAIL ") + what);
    if (!ok) bad++;
  }

  static boolean near(double a, double b, double eps) {
    return Math.abs(a - b) <= eps;
  }

  // the segments of range r as [ae, an, be, bn, ha, hb, at] each
  static List<float[]> segs(CloseUp.Detail d, int r) {
    List<float[]> out = new ArrayList<float[]>();
    for (int i = d.from[r]; i < d.from[r + 1]; i++) {
      float[] s = new float[CloseUp.SEG];
      System.arraycopy(d.seg, i * CloseUp.SEG, s, 0, CloseUp.SEG);
      out.add(s);
    }
    return out;
  }

  // height at the point (e, n) among the segments of range r, NaN if none ends there
  static float heightAt(CloseUp.Detail d, int r, float e, float n) {
    for (float[] s : segs(d, r)) {
      if (near(s[0], e, 0.05) && near(s[1], n, 0.05)) return s[4];
      if (near(s[2], e, 0.05) && near(s[3], n, 0.05)) return s[5];
    }
    return Float.NaN;
  }

  static float[] line(float... p) {
    return p;
  }

  public static void main(String[] a) {
    // the gate
    check(CloseUp.gate(100, 220, 260) == 1 && CloseUp.gate(220, 220, 260) == 1, "gate full at and under SHOW");
    check(CloseUp.gate(260, 220, 260) == 0 && CloseUp.gate(400, 220, 260) == 0, "gate shut at and past HIDE");
    boolean mono = true;
    float last = 1;
    for (double r = 220; r <= 260; r += 0.5) {
      float g = CloseUp.gate(r, 220, 260);
      mono &= g <= last + 1e-6f && g >= 0 && g <= 1;
      last = g;
    }
    check(mono, "gate falls smoothly between");
    check(near(CloseUp.gate(240, 220, 260), 0.5, 1e-6), "gate half way at the middle");

    // the lift: the camera's ray through a raised point meets the ground where lift() puts it
    boolean lifted = true;
    for (float hd : new float[] {0, 2, 5, 20}) {
      for (float z : new float[] {-40, 0, 80, 180}) {
        float x = 37;
        // camera at (0, H, ZC); ray through (x, hd, z) to y = 0
        double t = CloseUp.CAM_H / (CloseUp.CAM_H - hd);
        double gx = x * t, gz = CloseUp.CAM_ZC + (z - CloseUp.CAM_ZC) * t;
        float s = CloseUp.lift(hd);
        lifted &= near(x * s, gx, 1e-3) && near(CloseUp.CAM_ZC + (z - CloseUp.CAM_ZC) * s, gz, 1e-3);
      }
    }
    check(lifted && CloseUp.lift(0) == 1, "lift matches the camera ray");

    // a layer-1 street bridge from (0, 0) to (200, 0) between two streets: the streets ramp, the bridge stays up
    CloseUp.Lines in = new CloseUp.Lines();
    in.road(line(-300, 0, -100, 0, 0, 0), 2, false, 0, 0, false);
    in.road(line(0, 0, 100, 0, 200, 0), 2, true, 1, 0, false);
    in.road(line(200, 0, 500, 0), 2, false, 0, 0, false);
    CloseUp.Detail d = CloseUp.build(in);
    int deck1 = CloseUp.range(1, 2);
    check(near(heightAt(d, deck1, 100, 0), L, 1e-3), "bridge middle at one deck's height");
    check(near(heightAt(d, deck1, 0, 0), L, 1e-3) && near(heightAt(d, deck1, 200, 0), L, 1e-3),
          "bridge ends stay up where a road meets them");
    check(near(heightAt(d, deck1, -60, 0), 0, 1e-3) && near(heightAt(d, deck1, 260, 0), 0, 1e-3),
          "the meeting roads ramp down to the ground 60 m out (a point is cut in there)");
    float mid = heightAt(d, deck1, 230, 0);
    check(Float.isNaN(mid) || (mid > 0 && mid < L), "ramp in between");
    check(Float.isNaN(heightAt(d, deck1, 500, 0)) && Float.isNaN(heightAt(d, deck1, -300, 0)),
          "the ground beyond the ramps is not drawn again");
    check(d.bridges == 1 && d.ramps == 2, "one bridge, two ramps: " + d.bridges + ", " + d.ramps);
    check(d.empty(CloseUp.range(0, CloseUp.DASH)), "no lane lines on a street");

    // a lone bridge ramps inside itself over min(60 m, a third)
    in = new CloseUp.Lines();
    in.road(line(0, 0, 150, 0, 300, 0), 1, true, 1, 0, false);
    d = CloseUp.build(in);
    int deck = CloseUp.range(1, 1);
    check(near(heightAt(d, deck, 0, 0), 0, 1e-3) && near(heightAt(d, deck, 300, 0), 0, 1e-3), "lone bridge ends on the ground");
    check(near(heightAt(d, deck, 60, 0), L, 1e-3) && near(heightAt(d, deck, 240, 0), L, 1e-3) && near(heightAt(d, deck, 150, 0), L, 1e-3),
          "lone bridge up from 60 m in");
    in = new CloseUp.Lines();
    in.road(line(0, 0, 90, 0), 1, true, 1, 0, false);
    d = CloseUp.build(in);
    check(near(heightAt(d, CloseUp.range(1, 1), 30, 0), L, 1e-3), "a short bridge ramps over a third of itself");

    // a bridge cut at a tile edge, its halves' ends 1.5 m apart: no ramp at the cut
    in = new CloseUp.Lines();
    in.road(line(0, 0, 100, 0), 0, true, 1, 0, true);
    in.road(line(101.5f, 0, 300, 0), 0, true, 1, 0, true);
    d = CloseUp.build(in);
    deck = CloseUp.range(1, 0);
    check(near(heightAt(d, deck, 100, 0), L, 1e-3) && near(heightAt(d, deck, 101.5f, 0), L, 1e-3), "no ramp at a tile-edge cut");
    check(near(heightAt(d, deck, 0, 0), 0, 1e-3) && near(heightAt(d, deck, 300, 0), 0, 1e-3), "ramps at the true ends");

    // layers: 2 stands at 2 L in group 2; 5 is grouped with 3 but stands at 5 L; layer 0 bridges count as 1
    in = new CloseUp.Lines();
    in.road(line(0, 0, 100, 0, 200, 0), 2, true, 2, 0, false);
    in.road(line(0, 500, 100, 500, 200, 500), 2, true, 5, 0, false);
    in.road(line(0, 900, 100, 900, 200, 900), 2, true, 0, 0, false);
    d = CloseUp.build(in);
    check(near(heightAt(d, CloseUp.range(2, 2), 100, 0), 2 * L, 1e-3), "layer 2 at two decks");
    check(near(heightAt(d, CloseUp.range(3, 2), 100, 500), 5 * L, 1e-3), "layer 5 at five decks, drawn with group 3");
    check(near(heightAt(d, CloseUp.range(1, 2), 100, 900), L, 1e-3), "a bridge without a layer counts as layer 1");
    check(near(d.maxH, 5 * L, 1e-3), "max height");

    // lane lines on a one-way motorway east, no lane count (the US): 3 lanes, 2 dashed separators, a white right
    // edge, a yellow left edge, all inside the drawn width
    in = new CloseUp.Lines();
    in.road(line(0, 0, 100, 0, 200, 0), 0, false, 0, 0, true);
    d = CloseUp.build(in);
    List<float[]> dash = segs(d, CloseUp.range(0, CloseUp.DASH)), edge = segs(d, CloseUp.range(0, CloseUp.EDGE));
    List<float[]> yellow = segs(d, CloseUp.range(0, CloseUp.YELLOW));
    check(dash.size() == 4 && edge.size() == 2 && yellow.size() == 2, "one-way: 2 dashed, 1 white, 1 yellow line of 2 segments: "
          + dash.size() + " " + edge.size() + " " + yellow.size());
    float w = CloseUp.MOTORWAY_W;
    check(near(edge.get(0)[1], -(w / 2 - 1.5f), 1e-3) && near(yellow.get(0)[1], w / 2 - 1.5f, 1e-3), "edges just inside, yellow on the left");
    boolean sep = true;
    for (float[] s : dash) sep &= near(Math.abs(s[1]), w / 6, 1e-3) && s[4] == 0;
    check(sep, "separators a third of the width apart, on the ground");
    check(near(dash.get(1)[6], 100, 1e-3), "dash phase runs along the line");
    check(d.empty(CloseUp.range(0, 0)), "no deck for a ground motorway");
    // two-way, no lane count: 4 lanes, a yellow centre
    in = new CloseUp.Lines();
    in.road(line(0, 0, 200, 0), 0, false, 0, 0, false);
    d = CloseUp.build(in);
    yellow = segs(d, CloseUp.range(0, CloseUp.YELLOW));
    check(segs(d, CloseUp.range(0, CloseUp.DASH)).size() == 2 && segs(d, CloseUp.range(0, CloseUp.EDGE)).size() == 2
          && yellow.size() == 1 && near(yellow.get(0)[1], 0, 1e-3), "two-way: 2 dashed, 2 edges, yellow centre");
    // a lane count from the tile (Japan) wins
    in = new CloseUp.Lines();
    in.road(line(0, 0, 200, 0), 0, false, 0, 2, true);
    d = CloseUp.build(in);
    check(segs(d, CloseUp.range(0, CloseUp.DASH)).size() == 1, "2 lanes from the tile: 1 separator");
    // a motorway bridge's lanes are raised with it
    in = new CloseUp.Lines();
    in.road(line(0, 0, 150, 0, 300, 0), 0, true, 1, 0, true);
    d = CloseUp.build(in);
    boolean up = true;
    for (float[] s : segs(d, CloseUp.range(1, CloseUp.DASH))) up &= s[4] > 0 || s[5] > 0;
    check(!d.empty(CloseUp.range(1, CloseUp.DASH)) && up && d.empty(CloseUp.range(0, CloseUp.DASH)), "bridge lanes in the bridge's group");

    // offset at a right-angle bend: mitred to (o, o) out
    float[] q = CloseUp.offset(line(0, 0, 100, 0, 100, 100), 10);
    check(near(q[2], 90, 1e-3) && near(q[3], 10, 1e-3), "mitre at a bend: " + q[2] + ", " + q[3]);

    // walks: crossings dashed, the rest solid; kept nearest first under the cap
    in = new CloseUp.Lines();
    in.walk(line(0, 10, 50, 10), 1);
    in.walk(line(0, -10, 0, 10), 2);
    d = CloseUp.build(in);
    check(segs(d, CloseUp.WALK).size() == 1 && segs(d, CloseUp.CROSS).size() == 1, "sidewalk solid, crossing dashed");
    in = new CloseUp.Lines();
    for (int i = 0; i < 6000; i++) {
      float r = 900f * i / 6000, t = i * 2.4f;
      float e = (float) (r * Math.cos(t)), n = (float) (r * Math.sin(t));
      in.walk(line(e, n, e + 1, n), 1);
    }
    d = CloseUp.build(in);
    float far = 0;
    for (float[] s : segs(d, CloseUp.WALK)) far = Math.max(far, (float) Math.hypot(s[0], s[1]));
    check(d.walkSegs == CloseUp.WALK_SEGS && d.walkDropped > 0 && far < CloseUp.WALK_R, "walk cap " + d.walkSegs + " kept, " + d.walkDropped + " dropped, farthest " + far);
    float keptR = 900f * CloseUp.WALK_SEGS / 6000;
    check(far < keptR + 2, "the nearest kept: farthest " + far + " m, the cap reaches " + keptR);
    // roads past the radius are left out
    in = new CloseUp.Lines();
    in.road(line(5000, 0, 5200, 0), 0, false, 0, 0, true);
    check(CloseUp.build(in).count() == 0, "roads past ROAD_R left out");

    // rails: a straight main line 50 m north whose two points are 3 km apart is kept (in pieces), the pieces past
    // RAIL_R left out; a tie across it every TIE_M, TIE_HALF_M each side; the road ranges keep their numbers
    in = new CloseUp.Lines();
    in.rail(line(-3000, 50, 3000, 50));
    in.rail(line(5000, 0, 5200, 0));
    d = CloseUp.build(in);
    List<float[]> rail = segs(d, CloseUp.RAIL), ties = segs(d, CloseUp.TIE);
    float railFar = 0, railLen = 0;
    for (float[] s : rail) {
      railFar = Math.max(railFar, Math.min(Math.abs(s[0]), Math.abs(s[2])));
      railLen += (float) Math.hypot(s[2] - s[0], s[3] - s[1]);
      check(Math.hypot(s[2] - s[0], s[3] - s[1]) <= CloseUp.RAIL_PIECE_M + 1e-3, "rail piece at most RAIL_PIECE_M");
    }
    check(!rail.isEmpty() && railFar < CloseUp.RAIL_R && railLen > 1800 && railLen < 2 * CloseUp.RAIL_R + 2 * CloseUp.RAIL_PIECE_M,
          "a long rail is kept near the anchor: " + rail.size() + " pieces, " + railLen + " m");
    boolean across = !ties.isEmpty();
    for (float[] s : ties) {
      across &= near(s[0], s[2], 1e-3) && near(Math.abs(s[3] - s[1]), 2 * CloseUp.TIE_HALF_M, 1e-3) && near((s[1] + s[3]) / 2, 50, 1e-3);
    }
    check(across, "ties across the line, " + ties.size() + " of them");
    check(near(ties.size(), 2 * CloseUp.RAIL_R / CloseUp.TIE_M, 3), "a tie every TIE_M: " + ties.size());
    check(d.railSegs == rail.size() + ties.size() && d.empty(CloseUp.WALK) && d.empty(CloseUp.range(0, 0)), "rails only in their ranges");
    check(CloseUp.RAIL == 2 + CloseUp.LAYERS * CloseUp.PER_LAYER && CloseUp.range(1, 0) == 2 + CloseUp.PER_LAYER, "rail ranges after the road ranges");
    in = new CloseUp.Lines();
    for (int i = 0; i < 40; i++) in.rail(line(-900, i * 20 - 400, 900, i * 20 - 400));
    d = CloseUp.build(in);
    check(d.railSegs == CloseUp.RAIL_SEGS && d.railDropped > 0, "rail cap " + d.railSegs + " kept, " + d.railDropped + " dropped");

    // ranges are in order and cover the segments
    in = new CloseUp.Lines();
    in.road(line(0, 0, 150, 0, 300, 0), 0, true, 1, 0, true);
    in.road(line(-300, 0, 0, 0), 0, false, 0, 0, true);
    in.walk(line(0, 10, 50, 10), 1);
    in.rail(line(-200, -30, 200, -30));
    d = CloseUp.build(in);
    boolean order = d.from[0] == 0 && d.seg.length == d.count() * CloseUp.SEG;
    for (int r = 0; r < CloseUp.RANGES; r++) order &= d.from[r] <= d.from[r + 1];
    check(order, "ranges ordered, " + d.count() + " segments");

    // the size of a busy close-up: a 1.4 km motorway interchange grid and a street grid with sidewalks
    in = new CloseUp.Lines();
    for (int i = -7; i <= 7; i++) {
      float u = i * 200;
      in.road(line(-1400, u, -700, u, 0, u, 700, u, 1400, u), 0, false, 0, 0, true);
      in.road(line(u, -60, u, 60), 0, true, 1, 0, true);
    }
    for (float u = -1000; u <= 1000; u += 90) {
      for (float v = -1000; v < 1000; v += 90) {
        in.walk(line(u + 8, v, u + 8, v + 90), 1);
        in.walk(line(v, u + 8, v + 90, u + 8), 1);
        in.walk(line(u - 8, v, u + 8, v), 2);
      }
    }
    long t0 = System.nanoTime();
    d = CloseUp.build(in);
    long t1 = System.nanoTime();
    check(d.roadSegs <= CloseUp.ROAD_SEGS && d.walkSegs <= CloseUp.WALK_SEGS, "busy scene: " + d.roadSegs + " road and "
          + d.walkSegs + " walk segments (" + d.roadDropped + ", " + d.walkDropped + " dropped), " + d.count() * 4 * 36 / 1024
          + " KB on the GPU, built in " + (t1 - t0) / 1000000 + " ms here");

    // the GPU corners (GlClose): four per segment, across and end from the corner, along kept short, heights carried
    int at = d.count() - 3;
    float[] gq = new float[3 * 4 * CloseUp.CORNER];
    CloseUp.corners(d, at, 3, gq);
    boolean same = true;
    for (int i = 0; i < 3; i++) {
      int s = (at + i) * CloseUp.SEG;
      for (int c = 0; c < 4; c++) {
        int o = (i * 4 + c) * CloseUp.CORNER;
        for (int j = 0; j < 4; j++) same &= gq[o + j] == d.seg[s + j];
        same &= gq[o + 4] == ((c & 1) == 0 ? -1 : 1) && gq[o + 5] == (c >> 1) && gq[o + 7] == d.seg[s + 4]
                && gq[o + 8] == d.seg[s + 5];
        same &= gq[o + 6] >= 0 && gq[o + 6] < 126 && near(Math.IEEEremainder(d.seg[s + 6] - gq[o + 6], 18), 0, 1e-3)
                && near(Math.IEEEremainder(d.seg[s + 6] - gq[o + 6], 3.5), 0, 1e-3);
      }
    }
    check(same, "GPU corners carry the ends, heights, corner and along (mod both dash periods)");
    check(CloseUp.CORNER * 4 == 36, "36 bytes a corner");
    System.out.println(bad == 0 ? "all ok" : bad + " failed");
    System.exit(bad == 0 ? 0 : 1);
  }
}
