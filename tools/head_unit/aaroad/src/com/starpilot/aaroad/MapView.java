package com.starpilot.aaroad;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.LinearGradient;
import android.graphics.Matrix;
import android.graphics.Paint;
import android.graphics.Path;
import android.graphics.Shader;
import android.graphics.Typeface;
import android.os.SystemClock;
import android.text.TextPaint;
import android.text.TextUtils;
import android.view.View;
import android.view.ViewGroup;
import android.widget.RelativeLayout;
import android.widget.TextView;
import de.robv.android.xposed.XposedBridge;
import java.io.InputStream;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.zip.ZipEntry;
import java.util.zip.ZipFile;

// The continuous map on Honda Hack's meter nav page (Layout B of the 2026-10-29 mockup, map2.py): a heading-up
// map tilted away like a road seen from above and behind the car, the route as a road with a carpet in openpilot's
// mode colour, the roads around it, narrower and darker, and Peter's Civic where the car is. Honda Hack's arrow moves
// to the left column; the distance, the 500 ft bar, the street, the speed limit and the arrival time are drawn here.
//
// It sits behind navi_layout in the meter's root and only shows while the nav page is up and the comma's feed
// (Feed) has a fresh position. Otherwise the feed stays quiet and Honda Hack's own page is left as it was.
//
// Geometry: positions are kept in metres east/north of an anchor near the car and turned into the car's frame
// each frame (x right, z ahead), divided by the zoom k so that roads keep their on-screen width at every zoom.
// The camera is the mockup's (k = 1): 150 m up, 50 m behind, 55 deg down, focal 260 px, the car at (369, 180).
final class MapView extends View {
  private static final double CAM_H = 150, CAM_ZC = -50, PITCH = Math.toRadians(55), FOCAL = 260, CX = 369, CY = 102.663;
  private static final double R0 = 70.03;          // display metres from the car to the range's far edge (y 86)
  private static final float PX_DM = 1.7156f;      // px per display metre across the road at the car
  private static final float BOX_X0 = -380, BOX_X1 = 240, BOX_Z0 = -45, BOX_Z1 = 185;  // a little past the screen
  private static final float BOX_R = 430;          // display metres: covers the box from the car
  private static final float CAR_X = 369, CAR_Y = 180, CAR_PX = 26;  // car point is (26, 26) in the sprite
  private static final float LOCAL_PX = 88, KERB_PX = 2, ROUTE_PX = 22, STUB_PX = 18, SIDE_PX = 40, GRID_DM = 50;
  private static final double NEAR = 152.4, TURN_LEN = 40;   // side roads full width inside 500 ft; road shown past the turn
  private static final double CLOSE = 60;                    // m ahead at the turn itself: the closest zoom
  private static final double TAU_IN = 0.8, TAU_OUT = 2.0;   // s, zoom smoothing: in briskly, back out gently
  // Stopping at an intersection with no turn there: close in on it like a turn. Slowing means braking harder than
  // STOP_DECEL with the stop (v^2 / 2a) landing near the junction, or already crawling below CRAWL within STOP_NEAR.
  private static final double STOP_LOOK = 150, STOP_DECEL = 0.7, CRAWL = 4, STOP_NEAR = 50;
  // stopped within ON_ROUTE m of the route, the car points along it: GPS has no heading at a stop, and one taken
  // there (a comma start at a light, 2026-10-29 photo) turned the whole map about 80 degrees
  private static final double HOLD_SPEED = 1, ON_ROUTE = 12;
  private static final float JCELL = 50, JUNCTION_GAP = 8, JUNCTION_MERGE = 20, JUNCTION_SIDE = 20, JUNCTION_PAST = 8;
  private static final double SCENE_R = 4000, MOVE_R = 600, ROADS_R = 700, CELL = 500, DENSE = 100;
  private static final long FRAME_MS = 66, IDLE_MS = 500, STALE_MS = 2000, TILE_REBUILD_MS = 1000;
  private static final String[] HIDE = {"background", "turn_layout", "route_remain", "cur_road", "compass_layout",
                                        "limited_speed", "limited_speed_text", "traffic_layout"};
  private static final int ICON = 80, ICON_PAD = 4, ICON_LEFT = 14, ICON_TOP = -14;  // arrow 72 px, centred x 100

  static volatile String modulePath;
  private static Bitmap car;
  private static boolean carTried;

  private final ViewGroup navi;
  private final String pkg;
  private final Map<String, View> views = new HashMap<String, View>();
  private final Matrix camera = new Matrix();
  private final Paint grid = new Paint(), kerb = stroke(0xff9a9a9a, LOCAL_PX + 2 * KERB_PX), road = stroke(0xff454545, LOCAL_PX),
      carpet = stroke(Color.WHITE, ROUTE_PX), stub = stroke(0xff454545, STUB_PX),
      sideKerb = stroke(0xff6a6a6a, SIDE_PX + 2 * KERB_PX), side = stroke(0xff383838, SIDE_PX),
      fill = new Paint(), mask = new Paint(), fog = new Paint();
  private final TextPaint big = text(28, true, Color.WHITE), unit = text(16, false, 0xffcccccc), street = text(24, true, Color.WHITE),
      eta = text(25, true, Color.WHITE), etaDim = text(17, false, 0xffb0b0b0), etaClock = text(23, true, 0xffe6e6e6), sign = text(8.5f, true, 0xff111111),
      signBig = text(23, true, 0xff111111);
  private final Path routePath = new Path(), carpetPath = new Path(), stubPath = new Path(), gridPath = new Path();
  private final SimpleDateFormat clock = new SimpleDateFormat("h:mm a", Locale.US);

  private volatile Scene scene;
  private volatile boolean building;
  private long builtAt, lastFrame, drawMs, drawSum, spdAt;
  private double lastSpd = -1, decel;  // m/s, m/s^2 (positive while slowing), from the fed speed
  private int drawCount;
  private boolean live;
  private double k = 600 / R0;
  private ViewGroup.LayoutParams iconSaved, iconOurs;
  private int[] iconPad;
  private float t0, t1;  // clip parameters of the segment being clipped

  // Positions are metres east (e) and north (n) of the anchor, in runs of e, n pairs.
  private static final class Scene {
    double ax, ay, mpt;  // anchor in zoom-14 tile units; metres per tile unit there
    long routeId = -1;
    int tilesVersion;
    float[][] route = new float[0][], stubs = new float[0][], roads = new float[0][];
    float[][] routeBox, stubsBox, roadsBox;  // min e, min n, max e, max n of each run
    float[] junctions = new float[0];  // e, n of every place a side road meets or crosses the route
  }

  MapView(Context context, ViewGroup navi, String pkg) {
    super(context);
    this.navi = navi;
    this.pkg = pkg;
    setLayerType(LAYER_TYPE_SOFTWARE, null);  // perspective paths and clipPath: software Skia only on 4.2
    setVisibility(INVISIBLE);
    double cp = Math.cos(PITCH), sp = Math.sin(PITCH), c0 = CAM_H * sp - CAM_ZC * cp;
    double[] h = {FOCAL, CX * cp, CX * c0, 0, CY * cp - FOCAL * sp, CY * c0 + FOCAL * (CAM_H * cp + CAM_ZC * sp), 0, cp, c0};
    float[] v = new float[9];
    for (int i = 0; i < 9; i++) v[i] = (float) (h[i] / c0);
    camera.setValues(v);
    grid.setColor(0xff2a2a2a);
    grid.setStyle(Paint.Style.STROKE);
    grid.setStrokeWidth(0);  // hairline in screen pixels
    fill.setAntiAlias(true);
    mask.setShader(new LinearGradient(0, 0, 184, 0, new int[] {0xff050505, 0xff050505, 0x00050505},
                                      new float[] {0, 0.87f, 1}, Shader.TileMode.CLAMP));
    fog.setShader(new LinearGradient(0, 0, 0, 78, new int[] {0xff050505, 0xe6050505, 0x00050505},
                                     new float[] {0, 0.6f, 1}, Shader.TileMode.CLAMP));
    big.setTextAlign(Paint.Align.RIGHT);
    street.setTextAlign(Paint.Align.CENTER);
    sign.setTextAlign(Paint.Align.CENTER);
    signBig.setTextAlign(Paint.Align.CENTER);
    loadCar();
  }

  private static float dm(float px) {
    return px / PX_DM;
  }

  private static Paint stroke(int color, float px) {
    Paint p = new Paint(Paint.ANTI_ALIAS_FLAG);
    p.setColor(color);
    p.setStyle(Paint.Style.STROKE);
    p.setStrokeWidth(dm(px));
    p.setStrokeJoin(Paint.Join.ROUND);
    p.setStrokeCap(Paint.Cap.ROUND);
    return p;
  }

  private static TextPaint text(float size, boolean bold, int color) {
    TextPaint p = new TextPaint(Paint.ANTI_ALIAS_FLAG);
    p.setTextSize(size);
    p.setColor(color);
    p.setTypeface(bold ? Typeface.DEFAULT_BOLD : Typeface.DEFAULT);
    return p;
  }

  // The Civic (Crystal Black Sport), rendered from Peter's reference photos; a white chevron if it will not load.
  private static synchronized void loadCar() {
    if (carTried || modulePath == null) return;
    carTried = true;
    ZipFile zip = null;
    try {
      zip = new ZipFile(modulePath);
      ZipEntry e = zip.getEntry("assets/civic_map.png");
      InputStream in = zip.getInputStream(e);
      car = BitmapFactory.decodeStream(in);
      in.close();
    } catch (Throwable t) {
      XposedBridge.log("aaroad: map car icon not loaded: " + t);
    } finally {
      try {
        if (zip != null) zip.close();
      } catch (Throwable ignored) {
      }
    }
  }

  // ---- liveness: the nav page is showing and the comma's position is fresh

  private final Runnable tick = new Runnable() {
    @Override
    public void run() {
      try {
        boolean shown = navi.isShown();
        Feed.wanted = shown;
        long now = SystemClock.uptimeMillis();
        setLive(shown && Feed.poseAt > 0 && now - Feed.poseAt < STALE_MS);
        if (live) {
          rebuildIfNeeded(now);
          invalidate();
        }
      } catch (Throwable t) {
        XposedBridge.log("aaroad: map tick failed: " + t);
      }
      // software drawing on the Tegra: if a frame costs more than half the frame time, draw less often
      postDelayed(this, live ? Math.max(FRAME_MS, 2 * drawMs) : IDLE_MS);
    }
  };

  @Override
  protected void onAttachedToWindow() {
    super.onAttachedToWindow();
    removeCallbacks(tick);
    post(tick);
  }

  @Override
  protected void onDetachedFromWindow() {
    removeCallbacks(tick);
    Feed.wanted = false;
    super.onDetachedFromWindow();
  }

  private View find(String name) {
    if (views.containsKey(name)) return views.get(name);
    int id = navi.getResources().getIdentifier(name, "id", pkg);
    View v = id == 0 ? null : navi.findViewById(id);
    views.put(name, v);
    return v;
  }

  // Layout B over Honda Hack's page while the map shows; its own page back as it was when not.
  private void setLive(boolean on) {
    View icon = find("turn_icon");
    if (on == live) {
      if (!on) return;
      if (icon != null && icon.getLayoutParams() != iconOurs) moveIcon(icon);  // Honda Hack laid it out again
      for (String n : HIDE) {
        View v = find(n);
        if (v != null && v.getAlpha() != 0f) v.setAlpha(0f);
      }
      return;
    }
    live = on;
    lastFrame = 0;
    setVisibility(on ? VISIBLE : INVISIBLE);
    for (String n : HIDE) {
      View v = find(n);
      if (v != null) v.setAlpha(on ? 0f : 1f);
    }
    if (icon == null) return;
    if (on) {
      moveIcon(icon);
    } else if (iconSaved != null) {
      icon.setLayoutParams(iconSaved);
      icon.setPadding(iconPad[0], iconPad[1], iconPad[2], iconPad[3]);
      iconSaved = iconOurs = null;
    }
    XposedBridge.log("aaroad: meter map " + (on ? "showing" : "hidden"));
  }

  private void moveIcon(View icon) {
    iconSaved = icon.getLayoutParams();
    iconPad = new int[] {icon.getPaddingLeft(), icon.getPaddingTop(), icon.getPaddingRight(), icon.getPaddingBottom()};
    RelativeLayout.LayoutParams p = new RelativeLayout.LayoutParams(ICON, ICON);
    p.addRule(RelativeLayout.ALIGN_PARENT_TOP);
    p.addRule(RelativeLayout.ALIGN_PARENT_LEFT);
    p.leftMargin = ICON_LEFT;
    p.topMargin = ICON_TOP;
    iconOurs = p;
    icon.setPadding(ICON_PAD, ICON_PAD, ICON_PAD, ICON_PAD);
    icon.setLayoutParams(p);
  }

  // ---- scene: the route and the roads near it, rebuilt off the UI thread when the car moves on or data arrives

  private void rebuildIfNeeded(long now) {
    if (building) return;
    Scene s = scene;
    Feed.Route r = Feed.route;
    final double lat = Feed.lat, lon = Feed.lon;
    boolean need = s == null || (r == null ? -1 : r.id) != s.routeId
        || (Feed.tilesVersion != s.tilesVersion && now - builtAt > TILE_REBUILD_MS);
    if (!need) {
      double[] w = Feed.world(lat, lon);
      need = Math.hypot(w[0] - s.ax, w[1] - s.ay) * s.mpt > MOVE_R;
    }
    if (!need) return;
    building = true;
    builtAt = now;
    final Feed.Route route = r;
    final int version = Feed.tilesVersion;
    Thread t = new Thread(new Runnable() {
      @Override
      public void run() {
        try {
          scene = build(lat, lon, route, version);
        } catch (Throwable e) {
          XposedBridge.log("aaroad: map scene failed: " + e);
        } finally {
          building = false;
        }
      }
    }, "aaroad-scene");
    t.setDaemon(true);
    t.start();
  }

  private static Scene build(double lat, double lon, Feed.Route r, int version) {
    Scene s = new Scene();
    double[] a = Feed.world(lat, lon);
    s.ax = a[0];
    s.ay = a[1];
    s.mpt = Feed.metersPerTile(lat);
    s.tilesVersion = version;
    List<float[]> route = new ArrayList<float[]>(), stubs = new ArrayList<float[]>(), roads = new ArrayList<float[]>();
    HashSet<Long> cells = new HashSet<Long>();
    if (r != null) {
      s.routeId = r.id;
      Run run = new Run();
      double pe = 0, pn = 0;
      boolean pin = false;
      for (int i = 0; i < r.lat.length; i++) {
        double[] w = Feed.world(r.lat[i], r.lon[i]);
        double e = (w[0] - s.ax) * s.mpt, n = -(w[1] - s.ay) * s.mpt;
        boolean in = Math.hypot(e, n) < SCENE_R;
        if (i > 0 && (in || pin)) {
          if (run.size == 0) run.add(pe, pn);
          run.add(e, n);
          // the route's samples every DENSE m mark the cells whose roads may show as stubs
          int steps = (int) Math.ceil(Math.hypot(e - pe, n - pn) / DENSE);
          for (int j = 0; j <= steps; j++) {
            double f = steps == 0 ? 0 : (double) j / steps;
            cells.add(cell(pe + (e - pe) * f, pn + (n - pn) * f));
          }
        } else if (run.size > 0) {
          route.add(run.take());
        }
        pe = e;
        pn = n;
        pin = in;
      }
      if (run.size > 0) route.add(run.take());
    }
    for (Feed.Tile tile : Feed.tiles.values()) {
      for (int c = 0; c < 4; c++) {  // motorway, major, street, link; not service roads
        float[] pts = tile.pts[c];
        int[] starts = tile.starts[c];
        if (pts == null || starts == null) continue;
        for (int l = 0; l + 1 < starts.length; l++) {
          if (r != null) near(s, tile, pts, starts[l], starts[l + 1], cells, stubs);
          else around(s, tile, pts, starts[l], starts[l + 1], roads);
        }
      }
    }
    s.route = route.toArray(new float[0][]);
    s.stubs = stubs.toArray(new float[0][]);
    s.roads = roads.toArray(new float[0][]);
    s.routeBox = boxes(s.route);
    s.stubsBox = boxes(s.stubs);
    s.roadsBox = boxes(s.roads);
    if (r != null) s.junctions = junctions(s.route, s.stubs);
    return s;
  }

  // Where a side road crosses the route or ends on it (a T), merged within JUNCTION_MERGE m. A road running
  // along the route (its own road in the tiles) is skipped by the angle test.
  private static float[] junctions(float[][] route, float[][] stubs) {
    HashMap<Long, List<float[]>> segs = new HashMap<Long, List<float[]>>();
    for (float[] p : route) {
      for (int i = 0; i + 3 < p.length; i += 2) {
        float[] g = {p[i], p[i + 1], p[i + 2], p[i + 3]};
        int e0 = (int) Math.floor(Math.min(g[0], g[2]) / JCELL), e1 = (int) Math.floor(Math.max(g[0], g[2]) / JCELL);
        int n0 = (int) Math.floor(Math.min(g[1], g[3]) / JCELL), n1 = (int) Math.floor(Math.max(g[1], g[3]) / JCELL);
        for (int a = e0; a <= e1; a++) {
          for (int b = n0; b <= n1; b++) {
            Long key = ((long) a << 32) ^ (b & 0xffffffffL);
            List<float[]> l = segs.get(key);
            if (l == null) segs.put(key, l = new ArrayList<float[]>());
            l.add(g);
          }
        }
      }
    }
    List<float[]> found = new ArrayList<float[]>();
    for (float[] p : stubs) {
      for (int i = 0; i + 3 < p.length; i += 2) {
        float ax = p[i], ay = p[i + 1], dx = p[i + 2] - ax, dy = p[i + 3] - ay;
        float len = (float) Math.hypot(dx, dy);
        if (len < 0.5f) continue;
        // the ends of the side road reach JUNCTION_GAP m further, so a T that stops short of the route centre counts
        float ext = JUNCTION_GAP / len, lo = i == 0 ? -ext : 0, hi = i + 4 >= p.length ? 1 + ext : 1;
        int ca = (int) Math.floor((Math.min(ax, ax + dx) - JUNCTION_GAP) / JCELL), cb = (int) Math.floor((Math.max(ax, ax + dx) + JUNCTION_GAP) / JCELL);
        int cc = (int) Math.floor((Math.min(ay, ay + dy) - JUNCTION_GAP) / JCELL), cd = (int) Math.floor((Math.max(ay, ay + dy) + JUNCTION_GAP) / JCELL);
        for (int a = ca; a <= cb; a++) {
          for (int b = cc; b <= cd; b++) {
            List<float[]> l = segs.get(((long) a << 32) ^ (b & 0xffffffffL));
            if (l == null) continue;
            for (float[] g : l) {
              float ex = g[2] - g[0], ey = g[3] - g[1], den = dx * ey - dy * ex;
              float glen = (float) Math.hypot(ex, ey);
              if (glen < 0.5f || Math.abs(den) < 0.5f * len * glen) continue;  // within 30 degrees of the route: alongside it
              float t = ((g[0] - ax) * ey - (g[1] - ay) * ex) / den, u = ((g[0] - ax) * dy - (g[1] - ay) * dx) / den;
              if (t < lo || t > hi || u < 0 || u > 1) continue;
              float je = ax + t * dx, jn = ay + t * dy;
              boolean dup = false;
              for (float[] f : found) dup |= Math.hypot(f[0] - je, f[1] - jn) < JUNCTION_MERGE;
              if (!dup) found.add(new float[] {je, jn});
            }
          }
        }
      }
    }
    float[] out = new float[2 * found.size()];
    for (int i = 0; i < found.size(); i++) {
      out[2 * i] = found.get(i)[0];
      out[2 * i + 1] = found.get(i)[1];
    }
    return out;
  }

  // Compass direction (radians) of the route segment nearest (e, n), or NaN when none is within ON_ROUTE m.
  private static double routeHeading(float[][] route, float e, float n) {
    double best = ON_ROUTE * ON_ROUTE, out = Double.NaN;
    for (float[] p : route) {
      for (int i = 0; i + 3 < p.length; i += 2) {
        double dx = p[i + 2] - p[i], dy = p[i + 3] - p[i + 1], l2 = dx * dx + dy * dy;
        if (l2 < 1) continue;
        double t = Math.max(0, Math.min(1, ((e - p[i]) * dx + (n - p[i + 1]) * dy) / l2));
        double ex = p[i] + t * dx - e, ny = p[i + 1] + t * dy - n, d2 = ex * ex + ny * ny;
        if (d2 < best) { best = d2; out = Math.atan2(dx, dy); }
      }
    }
    return out;
  }

  // Distance ahead to the nearest junction in front of the car and close to its line of travel, or -1.
  private static double junctionAhead(Scene s, float ce, float cn, float ch, float sh) {
    double best = -1;
    for (int i = 0; i + 1 < s.junctions.length; i += 2) {
      float de = s.junctions[i] - ce, dn = s.junctions[i + 1] - cn;
      double ahead = de * sh + dn * ch, side = Math.abs(de * ch - dn * sh);
      if (ahead > -JUNCTION_PAST && ahead < STOP_LOOK && side < JUNCTION_SIDE && (best < 0 || ahead < best)) best = Math.max(0, ahead);
    }
    return best;
  }

  private static long cell(double e, double n) {
    return ((long) Math.floor(e / CELL) << 32) ^ ((long) Math.floor(n / CELL) & 0xffffffffL);
  }

  private static boolean nearRoute(HashSet<Long> cells, double e, double n) {
    for (int i = -2; i <= 2; i++) {  // two cells either way, so a side road is cut past the edge of the screen
      for (int j = -2; j <= 2; j++) {
        if (cells.contains(cell(e + i * CELL, n + j * CELL))) return true;
      }
    }
    return false;
  }

  // Parts of a tile road within a cell of the route (point index from..to), densified so a long straight
  // road crossing the route keeps the points near the crossing; one point either side of each run.
  private static void near(Scene s, Feed.Tile tile, float[] pts, int from, int to, HashSet<Long> cells, List<float[]> out) {
    Run run = new Run();
    double pe = 0, pn = 0;
    boolean pnear = false;
    for (int i = from; i < to; i++) {
      double e = ((tile.x + pts[2 * i] / Feed.EXTENT) - s.ax) * s.mpt;
      double n = -((tile.y + pts[2 * i + 1] / Feed.EXTENT) - s.ay) * s.mpt;
      int steps = i == from ? 0 : (int) Math.ceil(Math.hypot(e - pe, n - pn) / DENSE);
      for (int j = i == from ? 0 : 1; j <= steps; j++) {
        double f = steps == 0 ? 1 : (double) j / steps;
        double qe = pe + (e - pe) * f, qn = pn + (n - pn) * f;
        boolean isNear = nearRoute(cells, qe, qn);
        if (isNear) {
          if (run.size == 0 && i > from) run.add(pe + (e - pe) * (j - 1) / Math.max(1, steps), pn + (n - pn) * (j - 1) / Math.max(1, steps));
          run.add(qe, qn);
        } else if (pnear) {
          run.add(qe, qn);
          out.add(run.take());
        }
        pnear = isNear;
      }
      pe = e;
      pn = n;
    }
    if (run.size > 2) out.add(run.take());
  }

  private static void around(Scene s, Feed.Tile tile, float[] pts, int from, int to, List<float[]> out) {
    Run run = new Run();
    for (int i = from; i < to; i++) {
      double e = ((tile.x + pts[2 * i] / Feed.EXTENT) - s.ax) * s.mpt;
      double n = -((tile.y + pts[2 * i + 1] / Feed.EXTENT) - s.ay) * s.mpt;
      run.add(e, n);
    }
    if (run.size < 2) return;
    float[] b = boxes(new float[][] {run.peek()})[0];
    if (b[2] < -ROADS_R || b[0] > ROADS_R || b[3] < -ROADS_R || b[1] > ROADS_R) return;
    out.add(run.take());
  }

  private static float[][] boxes(float[][] runs) {
    float[][] b = new float[runs.length][];
    for (int r = 0; r < runs.length; r++) {
      float[] p = runs[r];
      float x0 = Float.MAX_VALUE, y0 = Float.MAX_VALUE, x1 = -Float.MAX_VALUE, y1 = -Float.MAX_VALUE;
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

  private static final class Run {
    float[] p = new float[64];
    int size;

    void add(double e, double n) {
      if (size + 2 > p.length) p = java.util.Arrays.copyOf(p, p.length * 2);
      p[size++] = (float) e;
      p[size++] = (float) n;
    }

    float[] peek() {
      return java.util.Arrays.copyOf(p, size);
    }

    float[] take() {
      float[] out = peek();
      size = 0;
      return out;
    }
  }

  // ---- drawing

  @Override
  protected void onDraw(Canvas canvas) {
    Scene s = scene;
    if (!live || s == null) return;
    long now = SystemClock.uptimeMillis();
    canvas.drawColor(0xff050505);

    // where the car is now: the last fix carried forward at its speed, at most half a second
    double h = Math.toRadians(Feed.hdg), spd = Math.max(0, Feed.spd);
    double ahead = Math.min(0.5, Math.max(0, (now - Feed.poseAt) / 1000.0)) * spd;
    double[] w = Feed.world(Feed.lat, Feed.lon);
    float ce = (float) ((w[0] - s.ax) * s.mpt + Math.sin(h) * ahead);
    float cn = (float) (-(w[1] - s.ay) * s.mpt + Math.cos(h) * ahead);
    if (spd < HOLD_SPEED) {
      double r = routeHeading(s.route, ce, cn);
      if (!Double.isNaN(r)) h = r;
    }
    float ch = (float) Math.cos(h), sh = (float) Math.sin(h);

    // zoom: about 20 s of road ahead, 200 to 600 m; once the turn is in range it is held near the top, so the
    // view closes in on the turn as the car reaches it and pulls back out once the next maneuver is far
    double turn = Feed.turnMeters >= 0 ? Math.max(0, Feed.turnMeters - spd * (now - Feed.stateAt) / 1000.0) : -1;
    double rng = Math.max(200, Math.min(600, spd * 20));
    if (turn >= 0 && turn + TURN_LEN < rng) rng = Math.max(turn + TURN_LEN, CLOSE);
    if (Feed.poseAt != spdAt) {
      double gap = (Feed.poseAt - spdAt) / 1000.0;
      if (lastSpd >= 0 && gap > 0 && gap < 1) decel += ((lastSpd - spd) / gap - decel) * Math.min(1, gap / 0.5);
      else decel = 0;
      lastSpd = spd;
      spdAt = Feed.poseAt;
    }
    double jn = s.route.length > 0 ? junctionAhead(s, ce, cn, ch, sh) : -1;
    if (jn >= 0) {
      double stopAt = decel > STOP_DECEL ? spd * spd / (2 * decel) : -1;
      boolean stopping = (stopAt >= 0 && Math.abs(stopAt - jn) < Math.max(25, 0.4 * jn)) || (spd < CRAWL && jn < STOP_NEAR);
      if (stopping && jn + TURN_LEN < rng) rng = Math.max(jn + TURN_LEN, CLOSE);
    }
    double dt = lastFrame == 0 ? 10 : (now - lastFrame) / 1000.0;
    lastFrame = now;
    k += (rng / R0 - k) * (1 - Math.exp(-dt / (rng / R0 < k ? TAU_IN : TAU_OUT)));
    float ik = (float) (1 / k), reach = (float) (BOX_R * k);

    routePath.rewind();
    carpetPath.rewind();
    stubPath.rewind();
    gridPath.rewind();
    gridLines(s, ce, cn, ch, sh, ik);
    paths(s.route, s.routeBox, routePath, ce, cn, ch, sh, ik, reach);
    paths(s.stubs, s.stubsBox, stubPath, ce, cn, ch, sh, ik, reach);
    if (s.route.length > 0) carpet(s, ce, cn, ch, sh, ik, reach);
    else paths(s.roads, s.roadsBox, stubPath, ce, cn, ch, sh, ik, reach);

    canvas.save();
    canvas.concat(camera);
    canvas.drawPath(gridPath, grid);
    if (s.route.length > 0) {
      // the roads around the route as roads, so the intersections and the road going on past a turn show:
      // every kerb first and then every surface, so the side roads open into the route instead of being cut by its kerb
      // side roads are full width at the closest zoom and thin out as the view zooms out, instead of filling it
      float sw = Math.min(1, (float) (NEAR / R0) * ik);
      sideKerb.setStrokeWidth(dm(SIDE_PX + 2 * KERB_PX) * sw);
      side.setStrokeWidth(dm(SIDE_PX) * sw);
      canvas.drawPath(stubPath, sideKerb);
      canvas.drawPath(routePath, kerb);
      canvas.drawPath(stubPath, side);
      canvas.drawPath(routePath, road);
      carpet.setColor(modeColor(Feed.mode));
      carpet.setAlpha(242);
      canvas.drawPath(carpetPath, carpet);
    } else {
      stub.setAlpha(160);
      canvas.drawPath(stubPath, stub);
    }
    canvas.restore();

    if (car != null) {
      canvas.drawBitmap(car, CAR_X - CAR_PX, CAR_Y - CAR_PX, null);
    } else {
      fill.setColor(Color.WHITE);
      Path p = new Path();
      p.moveTo(CAR_X, CAR_Y - 14);
      p.lineTo(CAR_X + 10, CAR_Y + 10);
      p.lineTo(CAR_X, CAR_Y + 5);
      p.lineTo(CAR_X - 10, CAR_Y + 10);
      p.close();
      canvas.drawPath(p, fill);
    }
    overlay(canvas, now);

    long ms = SystemClock.uptimeMillis() - now;
    drawMs = (drawMs * 7 + ms) / 8;
    drawSum += ms;
    if (++drawCount % 900 == 150) XposedBridge.log("aaroad: map draw " + drawSum / drawCount + " ms average over " + drawCount + " frames");
  }

  private static int modeColor(String mode) {
    if ("exp".equals(mode)) return Color.rgb(255, 140, 30);
    if ("chill".equals(mode)) return Color.rgb(40, 200, 90);
    if ("aol".equals(mode)) return Color.rgb(40, 130, 255);
    return Color.rgb(235, 235, 235);
  }

  // World grid every 50 display metres, fixed to the ground so it slides by as the car moves.
  private void gridLines(Scene s, float ce, float cn, float ch, float sh, float ik) {
    double g = GRID_DM / ik, reach = BOX_R / ik;
    double absE = s.ax * s.mpt + ce, absN = -s.ay * s.mpt + cn;  // metres from the map origin, for the phase
    for (int axis = 0; axis < 2; axis++) {
      double c = axis == 0 ? absE : absN;
      double first = Math.ceil((c - reach) / g) * g - c;
      for (double d = first; d <= reach; d += g) {
        float e0 = (float) (axis == 0 ? d : -reach), n0 = (float) (axis == 0 ? -reach : d);
        float e1 = (float) (axis == 0 ? d : reach), n1 = (float) (axis == 0 ? reach : d);
        segment(gridPath, (e0 * ch - n0 * sh) * ik, (e0 * sh + n0 * ch) * ik,
                (e1 * ch - n1 * sh) * ik, (e1 * sh + n1 * ch) * ik, false);
      }
    }
  }

  private void paths(float[][] runs, float[][] boxes, Path path, float ce, float cn, float ch, float sh, float ik, float reach) {
    for (int r = 0; r < runs.length; r++) {
      float[] b = boxes[r];
      if (b[2] < ce - reach || b[0] > ce + reach || b[3] < cn - reach || b[1] > cn + reach) continue;
      line(runs[r], 0, path, ce, cn, ch, sh, ik, Float.NaN, 0);
    }
  }

  // The run from point index `from` on, into the car's frame and clipped; (fe, fn) replaces point `from` if given.
  private void line(float[] p, int from, Path path, float ce, float cn, float ch, float sh, float ik, float fe, float fn) {
    boolean pen = false;
    float px = 0, pz = 0;
    for (int i = 2 * from; i + 1 < p.length; i += 2) {
      float de = (i == 2 * from && !Float.isNaN(fe) ? fe : p[i]) - ce;
      float dn = (i == 2 * from && !Float.isNaN(fe) ? fn : p[i + 1]) - cn;
      float x = (de * ch - dn * sh) * ik, z = (de * sh + dn * ch) * ik;
      if (i > 2 * from) pen = segment(path, px, pz, x, z, pen);
      px = x;
      pz = z;
    }
  }

  // The carpet: the route from the car's place on it to the end of the scene, so the driven part is bare road.
  private void carpet(Scene s, float ce, float cn, float ch, float sh, float ik, float reach) {
    int bestRun = -1, bestSeg = 0;
    float bestD = Float.MAX_VALUE, be = 0, bn = 0;
    for (int r = 0; r < s.route.length; r++) {
      float[] b = s.routeBox[r];
      if (b[2] < ce - reach || b[0] > ce + reach || b[3] < cn - reach || b[1] > cn + reach) continue;
      float[] p = s.route[r];
      for (int i = 0; i + 3 < p.length; i += 2) {
        float ex = p[i + 2] - p[i], ny = p[i + 3] - p[i + 1], len2 = ex * ex + ny * ny;
        float t = len2 > 0 ? Math.max(0, Math.min(1, ((ce - p[i]) * ex + (cn - p[i + 1]) * ny) / len2)) : 0;
        float qe = p[i] + t * ex, qn = p[i + 1] + t * ny, d = (qe - ce) * (qe - ce) + (qn - cn) * (qn - cn);
        if (d < bestD) {
          bestD = d;
          bestRun = r;
          bestSeg = i / 2;
          be = qe;
          bn = qn;
        }
      }
    }
    if (bestRun < 0) return;
    line(s.route[bestRun], bestSeg, carpetPath, ce, cn, ch, sh, ik, be, bn);
    for (int r = bestRun + 1; r < s.route.length; r++) {
      float[] b = s.routeBox[r];
      if (b[2] < ce - reach || b[0] > ce + reach || b[3] < cn - reach || b[1] > cn + reach) continue;
      line(s.route[r], 0, carpetPath, ce, cn, ch, sh, ik, Float.NaN, 0);
    }
  }

  // Liang-Barsky: adds the part of (x0, z0)-(x1, z1) inside the box; returns whether the path ends at (x1, z1).
  private boolean segment(Path path, float x0, float z0, float x1, float z1, boolean pen) {
    float dx = x1 - x0, dz = z1 - z0;
    t0 = 0;
    t1 = 1;
    if (!clip(-dx, x0 - BOX_X0) || !clip(dx, BOX_X1 - x0) || !clip(-dz, z0 - BOX_Z0) || !clip(dz, BOX_Z1 - z0)) return false;
    if (!pen || t0 > 0) path.moveTo(x0 + t0 * dx, z0 + t0 * dz);
    path.lineTo(x0 + t1 * dx, z0 + t1 * dz);
    return t1 >= 1;
  }

  private boolean clip(float p, float q) {
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

  // ---- Layout B text: distance and bar in the left column, street on top, speed limit, arrival

  private void overlay(Canvas canvas, long now) {
    canvas.drawRect(0, 0, 184, 215, mask);
    canvas.drawRect(0, 0, 584, 78, fog);

    View lanes = find("drive_way_layout");
    float streetY = lanes != null && lanes.isShown() ? navi.getTop() + lanes.getBottom() + 22 : 48;
    boolean hasSign = Feed.limitMs > 0;
    TextView road = (TextView) find("next_road");
    CharSequence name = road != null && road.getVisibility() == VISIBLE ? road.getText() : null;
    if (!TextUtils.isEmpty(name)) {
      float maxw = hasSign ? 356 : 380, size = 24;
      street.setTextSize(size);
      while (street.measureText(name.toString()) > maxw && size > 17) street.setTextSize(--size);
      canvas.drawText(TextUtils.ellipsize(name, street, maxw, TextUtils.TruncateAt.END).toString(), 292, streetY, street);
    }

    TextView dist = (TextView) find("seg_dist");
    CharSequence d = dist != null && dist.getVisibility() == VISIBLE ? dist.getText() : null;
    if (!TextUtils.isEmpty(d)) {
      String t = d.toString().trim();
      int sp = t.lastIndexOf(' ');
      String num = sp > 0 ? t.substring(0, sp) : t, u = sp > 0 ? t.substring(sp + 1) : "";
      canvas.drawText(num, 120, 108, big);
      canvas.drawText(u, 124, 108, unit);
      double ft = Meter.feet(d);
      if (ft >= 0 && ft <= 500) bar(canvas, (int) Math.round(ft / 50), 92, 118, 24, 64);
    }

    if (hasSign) {
      double v = Feed.limitMs * (Feed.metric ? 3.6 : 2.23694);
      speedSign(canvas, String.valueOf(Math.round(v)), 474, 30, 40, 52);
    }

    if (Feed.remainingSec >= 0) {
      double left = Math.max(0, Feed.remainingSec - (now - Feed.stateAt) / 1000.0);
      long min = Math.round(left / 60);
      String n = min >= 60 ? (min / 60) + " h " + (min % 60) : String.valueOf(min);
      String rest = " · " + clock.format(new Date(System.currentTimeMillis() + (long) (left * 1000)));
      View vd = find("vehicle_data_layout");
      boolean low = vd != null && vd.isShown() && vd.getHeight() > 0;
      float x = low ? 128 : 64, y = low ? navi.getTop() + vd.getTop() - 4 : 205;
      canvas.drawText(n, x, y, eta);
      x += eta.measureText(n);
      canvas.drawText(" min", x, y, etaDim);
      x += etaDim.measureText(" min");
      canvas.drawText(rest, x, y, etaClock);
    }
  }

  // Stock look: white end caps, ten stacked segments lit white from the bottom, one per 50 ft left.
  private void bar(Canvas canvas, int lit, float x, float y, float w, float h) {
    fill.setColor(Color.WHITE);
    canvas.drawRect(x, y, x + w, y + 3, fill);
    canvas.drawRect(x, y + h - 3, x + w, y + h, fill);
    float seg = (h - 6 - 22) / 10f;
    for (int i = 0; i < 10; i++) {
      float b = y + h - 5 - i * (seg + 2);
      fill.setColor(i < lit ? Color.WHITE : 0xff5a5a5a);
      canvas.drawRect(x + 2, b - seg, x + w - 2, b, fill);
    }
  }

  // US speed limit sign: white plate, black inset border, the number large enough to read at a glance.
  private void speedSign(Canvas canvas, String n, float x, float y, float w, float h) {
    fill.setStyle(Paint.Style.FILL);
    fill.setColor(0xfff4f4f4);
    canvas.drawRoundRect(new android.graphics.RectF(x, y, x + w, y + h), 4, 4, fill);
    fill.setStyle(Paint.Style.STROKE);
    fill.setStrokeWidth(1.4f);
    fill.setColor(0xff111111);
    canvas.drawRoundRect(new android.graphics.RectF(x + 2.5f, y + 2.5f, x + w - 2.5f, y + h - 2.5f), 2.5f, 2.5f, fill);
    fill.setStyle(Paint.Style.FILL);
    canvas.drawText("SPEED", x + w / 2, y + 14, sign);
    canvas.drawText("LIMIT", x + w / 2, y + 23, sign);
    canvas.drawText(n, x + w / 2, y + 46, signBig);
  }
}
