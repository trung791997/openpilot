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

  // height at the point (e, n) among the segments of range r, NaN if none ends there; for a layer group's road range
  // also among its overpass deck range (CloseUp.deck), where the full-height stretch goes
  static float heightAt(CloseUp.Detail d, int r, float e, float n) {
    float h = at(d, r, e, n);
    int g = (r - 2) / CloseUp.PER_LAYER, c = (r - 2) % CloseUp.PER_LAYER;
    if (Float.isNaN(h) && r >= 2 && r < CloseUp.RAIL && g >= 1 && c < CloseUp.CLASSES) h = at(d, CloseUp.deck(g, c), e, n);
    return h;
  }

  static float at(CloseUp.Detail d, int r, float e, float n) {
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
    in.road(line(130, -200, 130, 200), 2, false, 0, 0, false);  // the street under it, so it is an overpass
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
    in.road(line(130, -200, 130, 200), 2, false, 0, 0, false);
    d = CloseUp.build(in);
    int deck = CloseUp.range(1, 1);
    check(near(heightAt(d, deck, 0, 0), 0, 1e-3) && near(heightAt(d, deck, 300, 0), 0, 1e-3), "lone bridge ends on the ground");
    check(near(heightAt(d, deck, 60, 0), L, 1e-3) && near(heightAt(d, deck, 240, 0), L, 1e-3) && near(heightAt(d, deck, 150, 0), L, 1e-3),
          "lone bridge up from 60 m in");
    in = new CloseUp.Lines();
    in.road(line(0, 0, 90, 0), 1, true, 1, 0, false);
    in.road(line(45, -200, 45, 200), 2, false, 0, 0, false);
    d = CloseUp.build(in);
    check(near(heightAt(d, CloseUp.range(1, 1), 30, 0), L, 1e-3), "a short bridge ramps over a third of itself");

    // a bridge cut at a tile edge, its halves' ends 1.5 m apart: no ramp at the cut
    in = new CloseUp.Lines();
    in.road(line(0, 0, 100, 0), 0, true, 1, 0, true);
    in.road(line(101.5f, 0, 300, 0), 0, true, 1, 0, true);
    in.road(line(200, -200, 200, 200), 2, false, 0, 0, false);  // under the second half only: both are raised
    d = CloseUp.build(in);
    deck = CloseUp.range(1, 0);
    check(near(heightAt(d, deck, 100, 0), L, 1e-3) && near(heightAt(d, deck, 101.5f, 0), L, 1e-3), "no ramp at a tile-edge cut");
    check(near(heightAt(d, deck, 0, 0), 0, 1e-3) && near(heightAt(d, deck, 300, 0), 0, 1e-3), "ramps at the true ends");

    // layers: 2 stands at 2 L in group 2; 5 is grouped with 3 but stands at 5 L; layer 0 bridges count as 1
    in = new CloseUp.Lines();
    in.road(line(0, 0, 100, 0, 200, 0), 2, true, 2, 0, false);
    in.road(line(0, 500, 100, 500, 200, 500), 2, true, 5, 0, false);
    in.road(line(0, 900, 100, 900, 200, 900), 2, true, 0, 0, false);
    in.road(line(130, -200, 130, 1100), 2, false, 0, 0, false);
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
    in.road(line(130, -200, 130, 200), 2, false, 0, 0, false);
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

    // which bridges are raised (CloseUp.overpasses, Peter 2026-11-05 FM 544): only one the car could drive under
    in = new CloseUp.Lines();
    in.road(line(0, 0, 150, 0, 300, 0), 1, true, 1, 0, false);  // a creek bridge: nothing under it
    d = CloseUp.build(in);
    check(d.maxH == 0 && d.bridges == 0, "a bridge with no road under it is drawn as plain road: maxH " + d.maxH);
    in = new CloseUp.Lines();
    in.road(line(0, 0, 150, 0, 300, 0), 1, true, 1, 0, false);
    in.rail(line(150, -200, 150, 200));
    in.walk(line(100, -200, 100, 200), 0);
    d = CloseUp.build(in);
    check(d.maxH == 0, "a rail line or a path under it does not raise it");
    in = new CloseUp.Lines();
    in.road(line(0, 0, 150, 0, 300, 0), 1, true, 1, 0, false);
    in.road(line(300, 0, 300, 200), 2, false, 0, 0, false);  // meets its end: its approach
    in.road(line(-150, -200, 0, 0), 2, false, 0, 0, false);
    d = CloseUp.build(in);
    check(d.maxH == 0, "roads meeting its ends are not under it");
    in = new CloseUp.Lines();
    in.road(line(0, 0, 150, 0, 300, 0), 1, true, 1, 0, false);
    in.road(line(0, -20, 300, 40), 2, false, 0, 0, false);  // 11 degrees across
    d = CloseUp.build(in);
    check(d.maxH == 0, "a road crossing at a slant under " + CloseUp.CROSS_MIN_DEG + " degrees does not raise it");
    in = new CloseUp.Lines();
    in.road(line(0, 0, 150, 0, 300, 0), 1, true, 1, 0, false);
    in.road(line(150, -200, 150, 200), 2, true, 1, 0, false);  // another bridge on the same layer crossing it
    d = CloseUp.build(in);
    check(d.maxH == 0, "a bridge on the same layer crossing it is not under it");
    in = new CloseUp.Lines();
    in.road(line(0, 0, 150, 0, 300, 0), 1, true, 1, 0, false);
    in.road(line(150, -200, 150, 0, 150, 200), 0, true, 2, 0, false);  // a layer-2 flyover over it
    d = CloseUp.build(in);
    check(near(heightAt(d, CloseUp.range(2, 0), 150, 0), 2 * L, 1e-3) && Float.isNaN(heightAt(d, CloseUp.range(1, 1), 150, 0)),
          "a flyover over a lower bridge raises the flyover only");
    // the overpass the car drives under is raised, its deck in the deck range, piers clear of the road under it
    in = new CloseUp.Lines();
    in.road(line(0, 0, 150, 0, 300, 0), 1, true, 1, 0, false);
    in.road(line(150, -400, 150, 400), 0, false, 0, 0, true);
    in.path = new float[][] {line(150, -400, 150, 400)};
    d = CloseUp.build(in);
    check(near(at(d, CloseUp.deck(1, 1), 150, 0), L, 1e-3) && near(at(d, CloseUp.range(1, 1), 0, 0), 0, 1e-3),
          "a route under it keeps it raised: the full-height stretch a deck, the ends ramps");
    List<float[]> piers = segs(d, CloseUp.pier(1));
    boolean columns = !piers.isEmpty(), clear = true;
    for (float[] p : piers) {
      columns &= p[0] == p[2] && p[1] == p[3] && p[4] == 0 && near(p[5], L, 1e-3);
      clear &= Math.abs(p[0] - 150) >= CloseUp.ROAD_W[0] / 2 + CloseUp.PIER_CLEAR_M;
      clear &= p[0] >= 60 - 1e-3 && p[0] <= 240 + 1e-3;
    }
    check(columns && piers.size() % 2 == 0, piers.size() + " piers, columns from the ground to the deck, in pairs");
    check(clear, "no pier on the motorway under it or on the ramps");
    // the road being driven stays down: the route along the bridge, or with no route the car's own road
    in.path = new float[][] {line(-100, 0, 400, 0)};
    d = CloseUp.build(in);
    check(d.maxH == 0, "a route on the bridge: drawn as the road it is part of");
    in.path = null;
    in.over = null;
    float[][] own = CloseUp.carRoad(in, 1, 0);
    check(own != null && own.length == 1 && own[0][4] == 300, "the car on the bridge heading east: its road is the bridge");
    in.path = own;
    check(CloseUp.build(in).maxH == 0, "so the bridge the car is on is not raised");
    in.held = false;
    CloseUp.overpasses(in, own);
    check(in.held, "an overpass held down as the car's road is flagged (free drive rebuilds once the car leaves it)");
    check(CloseUp.onRoad(own, 100, 3, 1, 0) && CloseUp.onRoad(own, 100, 3, -1, 0), "on the bridge either way: still on its road");
    check(!CloseUp.onRoad(own, 150, -60, 0, 1) && !CloseUp.onRoad(own, 150, 0, 0, 1),
          "on the motorway under it heading north: off it, at the crossing too");
    in.road(line(-200, 0, 0, 0), 1, false, 0, 0, false);
    in.road(line(300, 0, 500, 30), 1, false, 0, 0, false);
    in.road(line(300, 0, 300, 300), 2, false, 0, 0, false);
    own = CloseUp.carRoad(in, 1, 0);
    check(own != null && own.length == 3, "carRoad follows the straightest join each way: " + (own == null ? 0 : own.length));
    in = new CloseUp.Lines();
    in.road(line(-150, 0, 0, 0, 150, 0), 1, true, 1, 0, false);
    in.road(line(0, -400, 0, 400), 0, false, 0, 0, true);
    in.path = CloseUp.carRoad(in, 0, 1);
    check(in.path != null && in.path[0][0] == 0 && in.path[0][1] == -400 && near(CloseUp.build(in).maxH, L, 1e-3),
          "the car on the motorway under it heading north: the bridge stays raised");
    in.held = false;
    CloseUp.overpasses(in, in.path);
    check(!in.held, "and nothing is held down, so no rebuild is asked for");

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

    // link roads (class 3): the ramp gold only for a highway ramp (CloseUp.ramps, Peter 2026-11-05 Angel Pkwy)
    List<float[]> lk = new ArrayList<float[]>(), hw = new ArrayList<float[]>();
    hw.add(line(-100, -1500, -100, 1500));                         // a motorway, one straight line
    lk.add(line(180, 0, 195, 5, 200, 20));                         // 0 primary_link: a slip lane at a street corner
    lk.add(line(-100, -650, -90, -560, -80, -500));                // 1 an off-ramp leaving the motorway mid-line
    lk.add(line(-80, -500, -60, -450));                            // 2 its rest, cut at a tile edge (ends meet)
    lk.add(line(-69.5f, -473.75f, -40, -480));                     // 3 a branch off 2's middle
    lk.add(line(-94, 300, -60, 360));                              // 4 an end 6 m off the motorway (past JOIN_M)
    lk.add(line(-200, 600, 0, 620));                               // 5 crosses the motorway in plan, ends far from it
    lk.add(line(200, 20, 230, 40));                                // 6 joins the street slip lane 0 only
    lk.add(line(500, -100, 500, 100));                             // 7 a motorway's end lands on its middle
    hw.add(line(700, 0, 501, 0));
    boolean[] rp = CloseUp.ramps(lk, hw);
    check(!rp[0], "link: a primary_link at a street corner is a street");
    check(rp[1], "link: an off-ramp leaving the motorway mid-line is a ramp");
    check(rp[2] && rp[3], "link: the ramp's rest past a tile edge and a branch off it are ramp too");
    check(!rp[4], "link: an end 6 m off the motorway does not touch it");
    check(!rp[5], "link: crossing the motorway in plan (over or under it) is not joining it");
    check(!rp[6], "link: joined only to a street link stays a street");
    check(rp[7], "link: a motorway ending on its middle makes it a ramp");
    check(CloseUp.ramps(new ArrayList<float[]>(), hw).length == 0, "link: no links, nothing to judge");
    // a busy area: 400 links (half off a motorway grid) and 200 highway lines, once per scene build
    lk.clear();
    hw.clear();
    for (int i = 0; i < 200; i++) hw.add(line(i * 300, -3000, i * 300, 3000));
    for (int i = 0; i < 400; i++) {
      float x = (i / 2) * 300, y = (i % 40) * 150 - 3000;
      lk.add(i % 2 == 0 ? line(x, y, x + 40, y + 60) : line(x + 100, y, x + 140, y + 60));
    }
    t0 = System.nanoTime();
    rp = CloseUp.ramps(lk, hw);
    t1 = System.nanoTime();
    boolean alt = true;
    for (int i = 0; i < rp.length; i++) alt &= rp[i] == (i % 2 == 0);
    check(alt, "link: busy area judged right, in " + (t1 - t0) / 1000000 + " ms here");
    System.out.println(bad == 0 ? "all ok" : bad + " failed");
    System.exit(bad == 0 ? 0 : 1);
  }
}
