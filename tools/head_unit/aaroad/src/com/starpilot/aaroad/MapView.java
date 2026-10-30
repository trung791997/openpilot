package com.starpilot.aaroad;

import android.content.Context;
import android.content.res.AssetManager;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.LinearGradient;
import android.graphics.Matrix;
import android.graphics.Paint;
import android.graphics.Path;
import android.graphics.Rect;
import android.graphics.RectF;
import android.graphics.Shader;
import android.graphics.Typeface;
import android.graphics.drawable.Drawable;
import android.os.SystemClock;
import android.text.TextPaint;
import android.text.TextUtils;
import android.view.View;
import android.view.ViewGroup;
import android.widget.ImageView;
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
// map tilted away like a road seen from above and behind the car, every road to the same scale, the route marked
// only by a line in openpilot's mode colour, and Peter's Civic where the car is. Honda Hack's arrow moves
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
  static final float PX_DM = 1.7156f;      // px per display metre across the road at the car
  static final float BOX_X0 = -380, BOX_X1 = 240, BOX_Z0 = -45, BOX_Z1 = 185;  // a little past the screen
  private static final float BOX_R = 430;          // display metres: covers the box from the car
  // Road shapes are kept between frames (Peter 2026-10-30: building them was 46 of 158 ms a frame on the Tegra): each
  // run is cut into pieces of at most CHUNK_M m, each a Path in world metres built with the scene, and a frame only
  // picks the pieces in view and draws them through a matrix. A piece reaching nearer than NEAR_Z display metres is
  // still turned and clipped point by point: the camera's horizon line behind the car is at z -264, and anything past
  // it would come out on the wrong side of the screen.
  private static final float CHUNK_M = 150, NEAR_Z = -200;
  static final float FREE_X = 292;  // the meter's centre
  static final float CAR_X = 369, CAR_Y = 180, CAR_PX = 26;  // car point is (26, 26) in the sprite
  static final float KERB_PX = 2, MIN_ROAD_PX = 2, GRID_DM = 50;
  // Every road at its real width, so an intersection looks like the one out of the windscreen (Peter 2026-10-29:
  // the route drawn wider than the roads around it looked wrong; the orange line alone marks the route). Metres
  // kerb to kerb, both directions, by tile class: motorway, major, street, link; the route itself as a street
  // under its own road, where the tile has it. The orange line is about two lanes wide, between LINE_MIN_PX and
  // LINE_MAX_PX on screen. All of them are drawn ROAD_SCALE times as wide, the same for every road so they keep
  // their real proportions: at real width a street is a hairline beside the car icon.
  static final float[] ROAD_M = {28 * 2, 18 * 2, 10 * 2, 6 * 2, 5 * 2};  // x ROAD_SCALE 2
  // minor (service roads, parking-lot aisles, Peter 2026-10-29: the phone map shows them) under the rest
  static final int[] DRAW_ORDER = {4, 0, 1, 2, 3};
  static final float ROUTE_M = 10 * 2, LINE_M = 6 * 2, LINE_MIN_PX = 4, LINE_MAX_PX = 14;
  private static final double TURN_LEN = 40;   // m of road shown past the turn
  private static final double CLOSE = 60;                    // m ahead at the turn itself: the closest zoom
  private static final double TAU_IN = 0.8, TAU_OUT = 2.0;   // s, zoom smoothing: in briskly, back out gently
  // Stopping at an intersection with no turn there: close in on it like a turn. Slowing means braking harder than
  // STOP_DECEL with the stop (v^2 / 2a) landing near the junction, or already crawling below CRAWL within STOP_NEAR.
  private static final double STOP_LOOK = 150, STOP_DECEL = 0.7, CRAWL = 4, STOP_NEAR = 50;
  // stopped within ON_ROUTE m of the route, the car points along it: GPS has no heading at a stop, and one taken
  // there (a comma start at a light, 2026-10-29 photo) turned the whole map about 80 degrees
  private static final double HOLD_SPEED = 1, ON_ROUTE = 12;
  // stopped off the route (a parking lot, 2026-10-29 video), the map swam: GPS heading and position wander a few
  // metres and many degrees a second at a standstill. Stopped, the car keeps the heading it last moved with and
  // stays put while the fix wanders within HOLD_DRIFT m.
  private static final double HOLD_DRIFT = 10;
  private static final float JCELL = 50, JUNCTION_GAP = 8, JUNCTION_MERGE = 20, JUNCTION_SIDE = 20, JUNCTION_PAST = 8;
  private static final double SCENE_R = 4000, MOVE_R = 600, ROADS_R = 700, CELL = 500, DENSE = 100;
  private static final long FRAME_MS = 66, IDLE_MS = 500, STALE_MS = 2000, TILE_REBUILD_MS = 1000;
  // once showing, the map rides out a position gap this long with the car held where it was (Peter 2026-10-30
  // photo: mid-route the cluster flipped back to Honda Hack's page for a moment, once the comma's feed paused)
  private static final long GAP_HOLD_MS = 10000;
  private static final String[] HIDE = {"background", "turn_layout", "route_remain", "cur_road", "compass_layout",
                                        "limited_speed", "limited_speed_text", "traffic_layout", "turn_icon"};
  // arrow 52 px in the turn card, right of its bar (Peter 2026-10-29: loose over the map it looked out of place).
  // Drawn here from the stock view's picture: moving the view itself put it 70 px right of this, over the road
  // name, on the car (2026-10-30), its parent being offset in a way the layout dump did not show
  private static final int ICON = 52, ICON_LEFT = 52, ICON_TOP = 36;
  // Street names from the road tiles (Peter 2026-10-30: the street the car is on even with no route, and the names of
  // the streets around it). The car is on the nearest named road within ON_ROAD m that runs within ON_ROAD_DEG of its
  // heading, so a cross street at a junction does not take over. Looked up every ROAD_EVERY_MS, not per frame.
  // A different road has to be ROAD_CLEARLY m closer for ROAD_SWITCH_MS before it replaces the shown name, so a
  // parallel frontage road or a GPS wobble does not flicker the tag; a road that stops qualifying (the car turned off
  // it) gives way at once. With no road in reach the name stays ROAD_LOST_MS, then goes.
  private static final double ON_ROAD = 25, ON_ROAD_DEG = 35, ROAD_CLEARLY = 4;
  private static final long ROAD_EVERY_MS = 300, ROAD_SWITCH_MS = 1500, ROAD_LOST_MS = 4000;
  // At most LABELS other streets labelled. Where they go is chosen every LABEL_EVERY_MS from samples along the named
  // roads, nearest the car first, each kept clear of the cards, the car, the speed sign, the street tag and the
  // others. Between choices only the LABELS anchors are moved with the map (a few multiplies each), and one that
  // has slid onto something is skipped for that frame: the per-frame cost is two drawText calls per label.
  private static final int LABELS = 4;
  private static final long LABEL_EVERY_MS = 500;
  private static final float LABEL_STEP = 30, LABEL_MAX_W = 130, LABEL_GAP = 6, LABEL_TOP = 64, MINOR_COST = 150;

  static volatile String modulePath;
  private static Bitmap car;
  private static boolean carTried;

  private final ViewGroup navi;
  private final String pkg;
  private final Map<String, View> views = new HashMap<String, View>();
  private final Matrix camera = new Matrix();
  private final Paint grid = new Paint(), kerb = stroke(0xff6a6a6a, 0), road = stroke(0xff383838, 0),
      carpet = stroke(Color.WHITE, 0), sideKerb = stroke(0xff6a6a6a, 0), side = stroke(0xff383838, 0),
      fill = new Paint(), mask = new Paint(), fog = new Paint(), panel = new Paint(), edge = new Paint(),
      rim = new Paint(), ring = new Paint(), turnGlow = new Paint(), greenGlow = new Paint();
  // Inter on the cards and the street tag, sized to Peter's mockup; the speed sign keeps the system bold
  private final TextPaint big = text(36, inter(true), Color.WHITE), unit = text(17, inter(false), 0xffc8d0dc),
      street = text(17, inter(true), Color.WHITE), cardDim = text(13, inter(false), 0xffaab4c8),
      cardRoad = text(19, inter(true), Color.WHITE), trip = text(18, inter(true), Color.WHITE),
      tripClock = text(15, inter(false), 0xffd8dee8),
      songFrom = text(11, inter(false), 0xffaab4c8), songTitle = text(15, inter(true), Color.WHITE),
      songArtist = text(12, inter(false), 0xffaab4c8),
      sign = text(8.5f, Typeface.DEFAULT_BOLD, 0xff111111),
      signBig = text(23, Typeface.DEFAULT_BOLD, 0xff111111);
  private final Path routePath = new Path(), carpetPath = new Path(), gridPath = new Path(), coverClip = new Path();
  // the cards' shapes and rims never move, so they are built once
  private final Path turnCard = card(CARD_L, TURN_T, CARD_R, TURN_B), tripCard = card(CARD_L, TRIP_T, TRIP_R, TRIP_B),
      songCard = card(CARD_L, SONG_T, SONG_R, SONG_B), turnRim = leftEnd(CARD_L, TURN_T, TURN_B),
      logoWaves = spotifyWaves(SONG_X + LOGO_R, SONG_T + 10, LOGO_R);
  private final Paint smooth = new Paint(Paint.FILTER_BITMAP_FLAG);  // the 56 px cover scaled to 40
  private final Paint waves = new Paint(Paint.ANTI_ALIAS_FLAG);  // black: the waves are cut out of the green disc
  private final Path[] stubPaths = {new Path(), new Path(), new Path(), new Path(), new Path()};
  // this frame's kept pieces by class, in world metres, drawn through `world`; the near ones go in stubPaths/routePath
  private final Path[] stubWorld = {new Path(), new Path(), new Path(), new Path(), new Path()};
  private final Path routeWorld = new Path();
  private final Path[] routeWorlds = {routeWorld}, routeFrames = {routePath};
  private final Matrix world = new Matrix();  // world metres to the car's frame, this frame
  private final float[] worldV = new float[9], drawEN = new float[2], tickEN = new float[2];
  // What the last frame showed, so the tick only asks for a frame when something on screen would change (Peter
  // 2026-10-30): the car moved a pixel or turned, the zoom is still easing, or anything else drawn changed (stamp()).
  private Scene drawnScene;
  private float drawnE, drawnN;
  private double drawnH, drawnK, drawnBase;
  private boolean drawnMoving, zoomSettled, logDue, loggedCanvas;
  private long drawnAt, drawnStamp;
  private int checks, skipped;
  private static final double MOVE_PX = 1, TURN_DEG = 0.4, ZOOM_EPS = 0.01;
  private static final long REDRAW_MS = 1000;  // the clock, the trip countdown and the song ring tick on their own
  private final SimpleDateFormat clock = new SimpleDateFormat("h:mm a", Locale.US);

  private volatile Scene scene;
  private volatile boolean building;
  private long builtAt, lastFrame, drawMs, drawSum, spdAt, setupSum, pathSum, strokeSum, cardSum;
  private double heldH = Double.NaN;  // radians, the GPS heading while last moving
  private double[] heldW;              // world position held at a standstill
  private double lastSpd = -1, decel;  // m/s, m/s^2 (positive while slowing), from the fed speed
  private int drawCount;
  private boolean live;
  private String hideWhy = "";
  private double k = 600 / R0;
  private float t0, t1;  // clip parameters of the segment being clipped
  private final float[] cam = new float[9];  // the camera's values, to place labels where the roads are drawn
  private String onRoad, switchTo;  // street shown as the one the car is on (null: none); the one waiting to replace it
  private long roadAt, switchSince, lostSince, labelAt;
  private int labelCount;
  private final String[] labelText = new String[LABELS];
  private final float[] labelE = new float[LABELS], labelN = new float[LABELS], labelW = new float[LABELS],
      labelX = new float[LABELS], labelY = new float[LABELS];
  private final boolean[] labelOn = new boolean[LABELS];
  private final float[] tagBox = new float[4];  // the street tag as last drawn, with its pointer; zero size when none
  private final TextPaint label = text(11.5f, inter(false), 0xffc8ccd2), halo = text(11.5f, inter(false), 0xff050505);

  // Positions are metres east (e) and north (n) of the anchor, in runs of e, n pairs.
  static final class Scene {
    double ax, ay, mpt;  // anchor in zoom-14 tile units; metres per tile unit there
    long routeId = -1;
    int tilesVersion;
    float[][] route = new float[0][], stubs = new float[0][], roads = new float[0][];
    int[] stubsCls = new int[0], roadsCls = new int[0];  // tile class of each run
    String[] stubsName = new String[0], roadsName = new String[0];  // street name of each run, null when none
    float[][] routeBox, stubsBox, roadsBox;  // min e, min n, max e, max n of each run
    float[] junctions = new float[0];  // e, n of every place a side road meets or crosses the route
    float[][] routeAlong = new float[0][];  // metres along the route at each point of each run, runs end to end
    Chunks routeChunks = new Chunks(), stubsChunks = new Chunks(), roadsChunks = new Chunks();
  }

  // The runs cut into pieces of at most CHUNK_M m (neighbours share their end point), each kept as a Path in world
  // metres (x east, y north), with its points, its box (min e, min n, max e, max n) and its class.
  static final class Chunks {
    Path[] path = new Path[0];
    float[][] pts = new float[0][], box = new float[0][];
    int[] cls = new int[0];
    float[] at = new float[0];  // metres along the runs, end to end, at each piece's first point
  }

  // The roads, the route, the grid and the carpet drawn on the GPU under this view (GlRoads, Peter 2026-10-30:
  // stroking them in software was most of a frame on the Tegra). Meter adds it when GPU is set; the desktop harness
  // has no GL and leaves it off. Until it has drawn a frame, and for good if it fails, the view draws them itself.
  interface Roads {
    boolean ready();
    void frame(Scene s, float ce, float cn, float ch, float sh, float ik, boolean free, int color, float cut, float cutE, float cutN);
    long takeMs();  // average ms per GL frame since the last call, -1 with none
    void setShown(boolean on);
  }

  static boolean GPU = true;
  Roads gpu;

  MapView(Context context, ViewGroup navi, String pkg) {
    super(context);
    this.navi = navi;
    this.pkg = pkg;
    setLayerType(LAYER_TYPE_SOFTWARE, null);  // perspective paths and clipPath: software Skia only on 4.2
    setVisibility(INVISIBLE);
    float[] v = cameraValues();
    camera.setValues(v);
    System.arraycopy(v, 0, cam, 0, 9);
    label.setTextAlign(Paint.Align.CENTER);
    halo.setTextAlign(Paint.Align.CENTER);
    halo.setStyle(Paint.Style.STROKE);  // a dark outline under each label, so it reads over a road
    halo.setStrokeWidth(3);
    halo.setStrokeJoin(Paint.Join.ROUND);
    grid.setColor(0xff2a2a2a);
    grid.setStyle(Paint.Style.STROKE);
    grid.setStrokeWidth(0);  // hairline in screen pixels
    fill.setAntiAlias(true);
    mask.setShader(new LinearGradient(0, 0, COL, 0, new int[] {0xff050505, 0xff050505, 0x00050505},
                                      new float[] {0, (CARD_R + 4) / COL, 1}, Shader.TileMode.CLAMP));
    fog.setShader(new LinearGradient(0, 0, 0, 78, new int[] {0xff050505, 0xe6050505, 0x00050505},
                                     new float[] {0, 0.6f, 1}, Shader.TileMode.CLAMP));
    panel.setAntiAlias(true);
    panel.setColor(0xff000000);  // black as the rest of the dash (Peter 2026-10-30)
    edge.setAntiAlias(true);
    edge.setStyle(Paint.Style.STROKE);
    edge.setStrokeWidth(1);
    edge.setColor(0x38ffffff);
    rim.setAntiAlias(true);
    rim.setStyle(Paint.Style.STROKE);
    rim.setStrokeWidth(2);
    waves.setColor(0xff000000);
    ring.setAntiAlias(true);
    ring.setStyle(Paint.Style.STROKE);
    ring.setStrokeWidth(2.5f);
    ring.setStrokeCap(Paint.Cap.ROUND);
    // gradients, not blur: BlurMaskFilter and shadow layers are too slow for the Tegra's software canvas. Opaque,
    // blended into the black card by hand: inside the card's clip the car drew see-through ends solid (2026-10-30)
    float w = CARD_R - CARD_L;
    turnGlow.setShader(new LinearGradient(CARD_L, 0, CARD_L + 0.4f * w, 0, new int[] {0xff2a2a2a, 0xff000000},
                                          new float[] {0, 1}, Shader.TileMode.CLAMP));
    greenGlow.setShader(new LinearGradient(SONG_R, 0, SONG_R - 0.35f * (SONG_R - CARD_L), 0, new int[] {0xff0a4820, 0xff000000}, new float[] {0, 1}, Shader.TileMode.CLAMP));
    street.setTextAlign(Paint.Align.CENTER);
    sign.setTextAlign(Paint.Align.CENTER);
    signBig.setTextAlign(Paint.Align.CENTER);
    loadCar();
  }

  // The camera as a 3x3 from the car's frame (x, z, 1) to the screen, row-major as Matrix.setValues takes it.
  static float[] cameraValues() {
    double cp = Math.cos(PITCH), sp = Math.sin(PITCH), c0 = CAM_H * sp - CAM_ZC * cp;
    double[] h = {FOCAL, CX * cp, CX * c0, 0, CY * cp - FOCAL * sp, CY * c0 + FOCAL * (CAM_H * cp + CAM_ZC * sp), 0, cp, c0};
    float[] v = new float[9];
    for (int i = 0; i < 9; i++) v[i] = (float) (h[i] / c0);
    return v;
  }

  static float dm(float px) {
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

  private static TextPaint text(float size, Typeface face, int color) {
    TextPaint p = new TextPaint(Paint.ANTI_ALIAS_FLAG);
    p.setTextSize(size);
    p.setColor(color);
    p.setTypeface(face);
    return p;
  }

  // Inter (SIL OFL, assets/fonts) read from the module apk on first use: Display SemiBold for the bold text, Medium
  // for the rest. The system faces if it will not load. The AssetManager stays referenced with the faces.
  private static Typeface semiBold, medium;
  private static AssetManager fontAssets;

  private static synchronized Typeface inter(boolean bold) {
    if (medium == null) {
      try {
        AssetManager am = AssetManager.class.newInstance();
        AssetManager.class.getMethod("addAssetPath", String.class).invoke(am, modulePath);  // hidden on 4.2
        semiBold = Typeface.createFromAsset(am, "fonts/InterDisplay-SemiBold.ttf");
        medium = Typeface.createFromAsset(am, "fonts/Inter-Medium.ttf");
        fontAssets = am;
      } catch (Throwable t) {
        XposedBridge.log("aaroad: Inter not loaded, system font instead: " + t);
        semiBold = Typeface.DEFAULT_BOLD;
        medium = Typeface.DEFAULT;
      }
    }
    return bold ? semiBold : medium;
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
        long age = Feed.poseAt > 0 ? now - Feed.poseAt : Long.MAX_VALUE;
        hideWhy = !shown ? "page not shown" : age == Long.MAX_VALUE ? "no position yet"
            : "no position for " + age / 1000 + " s";
        setLive(shown && age < (live ? GAP_HOLD_MS : STALE_MS));
        if (live) {
          rebuildIfNeeded(now);
          if (changed(now)) invalidate();
          else skipped++;
          if (++checks % 900 == 150) logDue = true;  // with the next frame's timing
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
    if (on == live) {
      if (!on) return;
      for (String n : HIDE) {
        View v = find(n);
        if (v != null && v.getAlpha() != 0f) v.setAlpha(0f);
      }
      return;
    }
    live = on;
    lastFrame = 0;
    drawnAt = 0;
    setVisibility(on ? VISIBLE : INVISIBLE);
    if (gpu != null) gpu.setShown(on);
    for (String n : HIDE) {
      View v = find(n);
      if (v != null) v.setAlpha(on ? 0f : 1f);
    }
    XposedBridge.log("aaroad: meter map " + (on ? "showing" : "hidden: " + hideWhy));
  }

  // Whether a frame now would look different from the last one drawn (Peter 2026-10-30: most frames at a light or on
  // a straight redrew the same picture). Reads the held pose, never sets it; at a standstill the car does not move
  // on screen unless the fix wanders past HOLD_DRIFT.
  private boolean changed(long now) {
    Scene s = scene;
    if (s == null) return false;
    if (s != drawnScene || now - drawnAt >= REDRAW_MS || !zoomSettled || stamp(now) != drawnStamp) return true;
    double spd = Math.max(0, Feed.spd);
    boolean moving = spd >= HOLD_SPEED;
    if (moving != drawnMoving || Math.abs(zoomTarget(s, spd, now) / drawnBase - 1) > ZOOM_EPS) return true;
    if (!moving) {
      double[] held = heldW, w = Feed.world(Feed.lat, Feed.lon);
      return held == null || Math.hypot(w[0] - held[0], w[1] - held[1]) * s.mpt > HOLD_DRIFT;
    }
    double h = pose(s, now, spd, false, tickEN);
    double px = Math.hypot(tickEN[0] - drawnE, tickEN[1] - drawnN) * PX_DM / drawnK;
    return px > MOVE_PX || Math.abs(Math.IEEEremainder(h - drawnH, 2 * Math.PI)) > Math.toRadians(TURN_DEG);
  }

  // Everything drawn besides the car's place and the zoom, folded into one number: the feed's state (mode, limit,
  // trip, street, song), the cover, Honda Hack's distance, next street and arrow, the lane bar, a song line's scroll.
  private long stamp(long now) {
    long h = Feed.stateAt;
    h = h * 31 + Double.doubleToLongBits(Feed.limitMs);
    h = h * 31 + hash(Feed.mode);
    h = h * 31 + hash(Feed.road);
    h = h * 31 + hash(Feed.mediaStatus);
    h = h * 31 + hash(Feed.mediaTitle);
    h = h * 31 + Feed.coverId;
    h = h * 31 + hash(onRoad);
    View v = find("seg_dist");
    if (v instanceof TextView) h = h * 31 + (v.getVisibility() == VISIBLE ? hash(((TextView) v).getText()) : 1);
    v = find("next_road");
    if (v instanceof TextView) h = h * 31 + (v.getVisibility() == VISIBLE ? hash(((TextView) v).getText()) : 1);
    v = find("turn_icon");
    if (v != null) {
      Drawable d = v.isShown() && v instanceof ImageView ? ((ImageView) v).getDrawable() : null;
      h = h * 31 + (v.isShown() ? 1 : 2);
      if (d != null) h = (h * 31 + System.identityHashCode(d)) * 31 + d.getLevel();
    }
    v = find("drive_way_layout");
    if (v != null) h = h * 31 + (v.isShown() ? 1 : 2);
    return (h * 31 + titleRoll.shift(now)) * 31 + artistRoll.shift(now);
  }

  private static int hash(CharSequence c) {
    if (c == null) return 0;
    if (c instanceof String) return c.hashCode();
    int h = 1;
    for (int i = 0; i < c.length(); i++) h = h * 31 + c.charAt(i);
    return h;
  }

  // Where the car is now: the last fix carried forward at speed spd, at most half a second; e, n into en, and the
  // heading returned. hold (onDraw only): at a standstill the car keeps the heading it last moved with, stays put
  // while the fix wanders within HOLD_DRIFT, and near the route points along it. The tick passes false: no change.
  private double pose(Scene s, long now, double spd, boolean hold, float[] en) {
    double h = Math.toRadians(Feed.hdg);
    double ahead = Math.min(0.5, Math.max(0, (now - Feed.poseAt) / 1000.0)) * spd;
    double[] w = Feed.world(Feed.lat, Feed.lon);
    if (hold && spd >= HOLD_SPEED) {
      heldH = h;
      heldW = null;
    } else if (hold) {
      if (heldW == null || Math.hypot(w[0] - heldW[0], w[1] - heldW[1]) * s.mpt > HOLD_DRIFT) heldW = w;
      w = heldW;
      if (Double.isNaN(heldH)) heldH = h;  // parked since the map came up: the first heading, held
      h = heldH;
    }
    en[0] = (float) ((w[0] - s.ax) * s.mpt + Math.sin(h) * ahead);
    en[1] = (float) (-(w[1] - s.ay) * s.mpt + Math.cos(h) * ahead);
    if (hold && spd < HOLD_SPEED) {
      double r = routeHeading(s.route, en[0], en[1]);
      if (!Double.isNaN(r)) h = r;
    }
    return h;
  }

  // zoom: about 20 s of road ahead, 200 to 600 m; once the turn is in range it is held near the top, so the
  // view closes in on the turn as the car reaches it and pulls back out once the next maneuver is far
  private static double zoomTarget(Scene s, double spd, long now) {
    double turn = Feed.turnMeters >= 0 ? Math.max(0, Feed.turnMeters - spd * (now - Feed.stateAt) / 1000.0) : -1;
    // with no route nothing lies far ahead to show, so the view stays close: about 12 s of road, 120 to 300 m
    double rng = s.route.length == 0 ? Math.max(120, Math.min(300, spd * 12)) : Math.max(200, Math.min(600, spd * 20));
    if (turn >= 0 && turn + TURN_LEN < rng) rng = Math.max(turn + TURN_LEN, CLOSE);
    return rng;
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
    List<Integer> stubsCls = new ArrayList<Integer>(), roadsCls = new ArrayList<Integer>();
    List<String> stubsName = new ArrayList<String>(), roadsName = new ArrayList<String>();
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
      for (int c = 0; c < 5; c++) {  // motorway, major, street, link, minor
        float[] pts = tile.pts[c];
        int[] starts = tile.starts[c];
        String[] names = tile.names[c];
        if (pts == null || starts == null) continue;
        for (int l = 0; l + 1 < starts.length; l++) {
          // with a route, every road around the car shows as in free drive, and farther out the roads along the route
          if (r == null) around(s, tile, pts, starts[l], starts[l + 1], roads);
          else if (!around(s, tile, pts, starts[l], starts[l + 1], stubs)) near(s, tile, pts, starts[l], starts[l + 1], cells, stubs);
          String name = names != null && l < names.length ? names[l] : null;
          while (roadsCls.size() < roads.size()) { roadsCls.add(c); roadsName.add(name); }
          while (stubsCls.size() < stubs.size()) { stubsCls.add(c); stubsName.add(name); }
        }
      }
    }
    s.route = route.toArray(new float[0][]);
    s.stubs = stubs.toArray(new float[0][]);
    s.roads = roads.toArray(new float[0][]);
    s.stubsCls = ints(stubsCls);
    s.roadsCls = ints(roadsCls);
    s.stubsName = stubsName.toArray(new String[0]);
    s.roadsName = roadsName.toArray(new String[0]);
    s.routeBox = boxes(s.route);
    s.stubsBox = boxes(s.stubs);
    s.roadsBox = boxes(s.roads);
    if (r != null) s.junctions = junctions(s.route, s.stubs);
    s.routeAlong = new float[s.route.length][];
    double along = 0;
    for (int i = 0; i < s.route.length; i++) {
      float[] p = s.route[i], al = new float[p.length / 2];
      for (int j = 0; j < al.length; j++) {
        if (j > 0) along += Math.hypot(p[2 * j] - p[2 * j - 2], p[2 * j + 1] - p[2 * j - 1]);
        al[j] = (float) along;
      }
      s.routeAlong[i] = al;
    }
    s.routeChunks = chunks(s.route, null);
    s.stubsChunks = chunks(s.stubs, s.stubsCls);
    s.roadsChunks = chunks(s.roads, s.roadsCls);
    return s;
  }

  // Cuts each run into pieces of at most CHUNK_M m, a long segment into equal parts first; see Chunks. The pieces
  // overlap only in the shared point, and the round caps there join them as the round join did.
  private static Chunks chunks(float[][] runs, int[] cls) {
    List<float[]> pts = new ArrayList<float[]>();
    List<Integer> pcls = new ArrayList<Integer>();
    List<Float> pat = new ArrayList<Float>();
    Run run = new Run();
    double along = 0, start;
    for (int r = 0; r < runs.length; r++) {
      float[] p = runs[r];
      if (p.length < 4) continue;
      run.add(p[0], p[1]);
      start = along;
      double len = 0;
      for (int i = 2; i + 1 < p.length; i += 2) {
        double pe = p[i - 2], pn = p[i - 1], de = p[i] - pe, dn = p[i + 1] - pn, d = Math.hypot(de, dn);
        int parts = Math.max(1, (int) Math.ceil(d / CHUNK_M));
        for (int j = 1; j <= parts; j++) {
          if (len > 0 && len + d / parts > CHUNK_M) {
            float[] c = run.take();
            pts.add(c);
            pcls.add(cls == null ? 0 : cls[r]);
            pat.add((float) start);
            start = along;
            run.add(c[c.length - 2], c[c.length - 1]);
            len = 0;
          }
          if (j == parts) run.add(p[i], p[i + 1]);
          else run.add(pe + de * j / parts, pn + dn * j / parts);
          len += d / parts;
          along += d / parts;
        }
      }
      if (run.size >= 4) {
        pts.add(run.take());
        pcls.add(cls == null ? 0 : cls[r]);
        pat.add((float) start);
      }
      run.size = 0;
    }
    Chunks out = new Chunks();
    out.pts = pts.toArray(new float[0][]);
    out.box = boxes(out.pts);
    out.cls = ints(pcls);
    out.at = new float[pat.size()];
    for (int i = 0; i < out.at.length; i++) out.at[i] = pat.get(i);
    out.path = new Path[out.pts.length];
    for (int c = 0; c < out.pts.length; c++) {
      float[] q = out.pts[c];
      Path path = new Path();
      path.moveTo(q[0], q[1]);
      for (int i = 2; i + 1 < q.length; i += 2) path.lineTo(q[i], q[i + 1]);
      out.path[c] = path;
    }
    return out;
  }

  // Where a side road crosses the route or ends on it (a T), merged within JUNCTION_MERGE m. A road running
  // along the route (its own road in the tiles) is skipped by the angle test.
  private static int[] ints(List<Integer> l) {
    int[] a = new int[l.size()];
    for (int i = 0; i < a.length; i++) a[i] = l.get(i);
    return a;
  }

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

  private static boolean around(Scene s, Feed.Tile tile, float[] pts, int from, int to, List<float[]> out) {
    Run run = new Run();
    for (int i = from; i < to; i++) {
      double e = ((tile.x + pts[2 * i] / Feed.EXTENT) - s.ax) * s.mpt;
      double n = -((tile.y + pts[2 * i + 1] / Feed.EXTENT) - s.ay) * s.mpt;
      run.add(e, n);
    }
    if (run.size < 2) return true;
    float[] b = boxes(new float[][] {run.peek()})[0];
    if (b[2] < -ROADS_R || b[0] > ROADS_R || b[3] < -ROADS_R || b[1] > ROADS_R) return false;
    out.add(run.take());
    return true;
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

  // ---- street names

  // The street the car is on, from the named tile roads near it; see ON_ROAD for the rules.
  private void currentRoad(Scene s, float ce, float cn, double h, long now) {
    if (now - roadAt < ROAD_EVERY_MS) return;
    roadAt = now;
    boolean free = s.route.length == 0;
    float[][] runs = free ? s.roads : s.stubs, boxes = free ? s.roadsBox : s.stubsBox;
    String[] names = free ? s.roadsName : s.stubsName;
    double hx = Math.sin(h), hy = Math.cos(h), aligned = Math.cos(Math.toRadians(ON_ROAD_DEG));
    String best = null;
    double bestD = ON_ROAD, curD = Double.MAX_VALUE;
    for (int r = 0; r < runs.length; r++) {
      String name = names[r];
      float[] b = boxes[r];
      if (name == null || b[2] < ce - ON_ROAD || b[0] > ce + ON_ROAD || b[3] < cn - ON_ROAD || b[1] > cn + ON_ROAD) continue;
      float[] p = runs[r];
      for (int i = 0; i + 3 < p.length; i += 2) {
        double dx = p[i + 2] - p[i], dy = p[i + 3] - p[i + 1], l2 = dx * dx + dy * dy;
        if (l2 < 1 || Math.abs(dx * hx + dy * hy) < aligned * Math.sqrt(l2)) continue;  // either direction of travel
        double t = Math.max(0, Math.min(1, ((ce - p[i]) * dx + (cn - p[i + 1]) * dy) / l2));
        double d = Math.hypot(p[i] + t * dx - ce, p[i + 1] + t * dy - cn);
        if (d >= ON_ROAD) continue;
        if (d < bestD) {
          bestD = d;
          best = name;
        }
        if (name.equals(onRoad)) curD = Math.min(curD, d);
      }
    }
    if (best == null) {
      switchTo = null;
      if (onRoad != null && lostSince == 0) lostSince = now;
      if (onRoad != null && now - lostSince > ROAD_LOST_MS) onRoad = null;
      return;
    }
    lostSince = 0;
    if (onRoad == null || curD == Double.MAX_VALUE) {
      onRoad = best;  // nothing shown yet, or the car has left the shown road
      switchTo = null;
    } else if (best.equals(onRoad) || bestD + ROAD_CLEARLY >= curD) {
      switchTo = null;
    } else if (!best.equals(switchTo)) {
      switchTo = best;
      switchSince = now;
    } else if (now - switchSince >= ROAD_SWITCH_MS) {
      onRoad = best;
      switchTo = null;
    }
  }

  // Screen position of the car-frame point (x, z) through the camera, or null when it is off the drawn box.
  private boolean project(float x, float z, float off, float[] out) {
    if (x < BOX_X0 || x > BOX_X1 || z < BOX_Z0 || z > BOX_Z1) return false;
    float w = cam[6] * x + cam[7] * z + cam[8];
    if (w <= 0.01f) return false;
    out[0] = (cam[0] * x + cam[1] * z + cam[2]) / w + off;
    out[1] = (cam[3] * x + cam[4] * z + cam[5]) / w;
    return true;
  }

  // Whether a label box centred on (x, y) would cover a card, the car, the speed sign, the street tag or the edges.
  // Off the map: left of the cards (or of the tach needle without a route), and the fog at the top.
  private boolean blocked(float x, float y, float w, boolean free) {
    float l = x - w / 2 - 3, r = x + w / 2 + 3, t = y - 9, b = y + 7;
    if (l < (free ? CARD_L + 6 : CARD_R + 10) || r > 578 || t < LABEL_TOP || b > 212) return true;
    float carX = free ? FREE_X : CAR_X;
    if (hits(l, t, r, b, carX - 30, CAR_Y - 32, carX + 30, 215)) return true;
    if (Feed.limitMs > 0 && hits(l, t, r, b, 466, 24, 522, 90)) return true;
    if (tagBox[2] > tagBox[0] && hits(l, t, r, b, tagBox[0] - 6, tagBox[1] - 6, tagBox[2] + 6, tagBox[3] + 6)) return true;
    return free && Feed.mediaStatus != null && !TextUtils.isEmpty(Feed.mediaTitle)
        && hits(l, t, r, b, CARD_L - 6, SONG_T - 6, SONG_R + SLANT + 6, 215);
  }

  private static boolean hits(float l, float t, float r, float b, float l2, float t2, float r2, float b2) {
    return l < r2 && l2 < r && t < b2 && t2 < b;
  }

  // Picks up to LABELS other named streets and where to write them, every LABEL_EVERY_MS. Samples every LABEL_STEP
  // display metres along each named road in view, nearest the car first (service roads last), and takes the first
  // clear spot for each name.
  private void placeLabels(Scene s, float ce, float cn, float ch, float sh, float ik, float reach, boolean free,
                           String current, long now) {
    if (now - labelAt < LABEL_EVERY_MS) return;
    labelAt = now;
    float[][] runs = free ? s.roads : s.stubs, boxes = free ? s.roadsBox : s.stubsBox;
    String[] names = free ? s.roadsName : s.stubsName;
    int[] cls = free ? s.roadsCls : s.stubsCls;
    float off = free ? FREE_X - CAR_X : 0, step = LABEL_STEP / ik;
    float[] q = new float[2];
    List<float[]> found = new ArrayList<float[]>();  // cost, run, e, n, x, y
    for (int r = 0; r < runs.length; r++) {
      String name = names[r];
      float[] b = boxes[r];
      if (name == null || name.equals(current)) continue;
      if (b[2] < ce - reach || b[0] > ce + reach || b[3] < cn - reach || b[1] > cn + reach) continue;
      float[] p = runs[r];
      for (int i = 0; i + 3 < p.length; i += 2) {
        float dx = p[i + 2] - p[i], dy = p[i + 3] - p[i + 1];
        int pieces = Math.max(1, (int) Math.ceil(Math.hypot(dx, dy) / step));
        for (int j = 0; j < pieces; j++) {
          float f = (j + 0.5f) / pieces, e = p[i] + f * dx, n = p[i + 1] + f * dy, de = e - ce, dn = n - cn;
          if (!project((de * ch - dn * sh) * ik, (de * sh + dn * ch) * ik, off, q)) continue;
          if (q[1] < LABEL_TOP + 9 || q[1] > 205) continue;
          float cost = (float) Math.hypot(de, dn) + (cls[r] == 4 ? MINOR_COST : 0);
          found.add(new float[] {cost, r, e, n, q[0], q[1]});
        }
      }
    }
    java.util.Collections.sort(found, new java.util.Comparator<float[]>() {
      @Override
      public int compare(float[] a, float[] b) {
        return Float.compare(a[0], b[0]);
      }
    });
    int count = 0;
    HashSet<String> done = new HashSet<String>();
    for (float[] c : found) {
      if (count == LABELS) break;
      String name = names[(int) c[1]];
      if (done.contains(name)) continue;
      String text = TextUtils.ellipsize(name, label, LABEL_MAX_W, TextUtils.TruncateAt.END).toString();
      float w = label.measureText(text);
      if (blocked(c[4], c[5], w, free)) continue;
      boolean clear = true;
      for (int i = 0; i < count && clear; i++) {
        clear = !hits(c[4] - w / 2 - LABEL_GAP, c[5] - 9 - LABEL_GAP, c[4] + w / 2 + LABEL_GAP, c[5] + 7 + LABEL_GAP,
                      labelX[i] - labelW[i] / 2, labelY[i] - 9, labelX[i] + labelW[i] / 2, labelY[i] + 7);
      }
      if (!clear || crossed(runs, boxes, (int) c[1], c[2], c[3], c[4], w, ce, cn, ch, sh, ik, off, q)) continue;
      done.add(name);
      labelText[count] = text;
      labelE[count] = c[2];
      labelN[count] = c[3];
      labelW[count] = w;
      labelX[count] = c[4];
      labelY[count] = c[5];
      count++;
    }
    labelCount = count;
  }

  // Whether another road runs under a label written at (e, n) on run r: a name centred on a junction could be read
  // as either street. The label's half width is turned into display metres with the screen scale at that point.
  private boolean crossed(float[][] runs, float[][] boxes, int r, float e, float n, float x, float w, float ce,
                          float cn, float ch, float sh, float ik, float off, float[] q) {
    float de = e - ce, dn = n - cn;
    if (!project((de * ch - dn * sh) * ik + 5, (de * sh + dn * ch) * ik, off, q)) return true;
    float px = Math.abs(q[0] - x);  // screen px across 5 camera units beside the label
    double clear = (w / 2 + 4) / Math.max(px, 0.5f) * 5 / ik;
    for (int o = 0; o < runs.length; o++) {
      float[] b = boxes[o], t = runs[o];
      if (o == r || b[2] < e - clear || b[0] > e + clear || b[3] < n - clear || b[1] > n + clear) continue;
      for (int j = 0; j + 3 < t.length; j += 2) {
        double sx = t[j + 2] - t[j], sy = t[j + 3] - t[j + 1], l2 = sx * sx + sy * sy;
        double u = l2 < 1e-6 ? 0 : Math.max(0, Math.min(1, ((e - t[j]) * sx + (n - t[j + 1]) * sy) / l2));
        if (Math.hypot(t[j] + u * sx - e, t[j + 1] + u * sy - n) < clear) return true;
      }
    }
    return false;
  }

  // The chosen labels follow the map between choices; one that has moved onto something is left out this frame.
  private void moveLabels(float ce, float cn, float ch, float sh, float ik, boolean free) {
    float off = free ? FREE_X - CAR_X : 0;
    float[] q = new float[2];
    for (int i = 0; i < labelCount; i++) {
      float de = labelE[i] - ce, dn = labelN[i] - cn;
      labelOn[i] = project((de * ch - dn * sh) * ik, (de * sh + dn * ch) * ik, off, q) && !blocked(q[0], q[1], labelW[i], free);
      labelX[i] = q[0];
      labelY[i] = q[1];
    }
  }

  // ---- drawing

  @Override
  protected void onDraw(Canvas canvas) {
    Scene s = scene;
    if (!live || s == null) return;
    long now = SystemClock.uptimeMillis();
    // with the GPU layer drawing, the road area is left clear so it shows through from under the window
    boolean onGpu = gpu != null && gpu.ready();
    if (!onGpu) canvas.drawColor(0xff050505);
    drawnStamp = stamp(now);  // before anything is read, so a change during the frame still asks for the next one

    double spd = Math.max(0, Feed.spd);
    double h = pose(s, now, spd, true, drawEN);
    float ce = drawEN[0], cn = drawEN[1];
    float ch = (float) Math.cos(h), sh = (float) Math.sin(h);
    long t0 = SystemClock.uptimeMillis();

    double rng = zoomTarget(s, spd, now);
    drawnBase = rng;
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
    zoomSettled = Math.abs(rng / R0 / k - 1) <= ZOOM_EPS;
    drawnScene = s;
    drawnAt = now;
    drawnE = ce;
    drawnN = cn;
    drawnH = h;
    drawnK = k;
    drawnMoving = spd >= HOLD_SPEED;

    // world metres (e, n) to the car's frame: x = ((e - ce) ch - (n - cn) sh) / k, z = ((e - ce) sh + (n - cn) ch) / k
    worldV[0] = ch * ik;
    worldV[1] = -sh * ik;
    worldV[2] = (-ce * ch + cn * sh) * ik;
    worldV[3] = sh * ik;
    worldV[4] = ch * ik;
    worldV[5] = (-ce * sh - cn * ch) * ik;
    worldV[6] = 0;
    worldV[7] = 0;
    worldV[8] = 1;
    world.setValues(worldV);
    // with no route there is no turn text, so the map takes the whole meter: the car at its centre
    boolean free = s.route.length == 0;
    // the GPU layer is handed every frame, even while the view still draws the roads, so it is warm when it takes over
    if (gpu != null) {
      float cut = Float.NaN;
      if (!free && carpetStart(s, ce, cn, reach)) {
        float[] p = s.route[cRun];
        cut = s.routeAlong[cRun][cSeg] + (float) Math.hypot(cE - p[2 * cSeg], cN - p[2 * cSeg + 1]);
      }
      gpu.frame(s, ce, cn, ch, sh, ik, free, modeColor(Feed.mode), cut, cE, cN);
    }
    if (!onGpu) {
      routePath.rewind();
      routeWorld.rewind();
      carpetPath.rewind();
      for (Path p : stubPaths) p.rewind();
      for (Path p : stubWorld) p.rewind();
      gridPath.rewind();
      gridLines(s, ce, cn, ch, sh, ik);
      pieces(s.routeChunks, routeWorlds, routeFrames, ce, cn, ch, sh, ik);
      if (s.route.length > 0) pieces(s.stubsChunks, stubWorld, stubPaths, ce, cn, ch, sh, ik);
      if (s.route.length > 0) carpet(s, ce, cn, ch, sh, ik, reach);
      else pieces(s.roadsChunks, stubWorld, stubPaths, ce, cn, ch, sh, ik);
    }

    currentRoad(s, ce, cn, h, now);
    String shown = !TextUtils.isEmpty(Feed.road) ? Feed.road : onRoad;
    placeLabels(s, ce, cn, ch, sh, ik, reach, free, shown, now);
    moveLabels(ce, cn, ch, sh, ik, free);
    long t1 = SystemClock.uptimeMillis();
    canvas.save();
    if (free) canvas.translate(FREE_X - CAR_X, 0f);
    if (!onGpu) {
      canvas.save();
      canvas.concat(camera);
      canvas.drawPath(gridPath, grid);
      // a road W m wide is W / k display metres; never thinner than MIN_ROAD_PX so a far road still shows
      float floor = dm(MIN_ROAD_PX);
      // every kerb first and then every surface, so crossing roads open into each other instead of being cut by a kerb
      for (int c : DRAW_ORDER) stroke(canvas, stubWorld[c], stubPaths[c], sideKerb, Math.max(floor, ROAD_M[c] * ik) + dm(2 * KERB_PX));
      if (s.route.length > 0) stroke(canvas, routeWorld, routePath, kerb, Math.max(floor, ROUTE_M * ik) + dm(2 * KERB_PX));
      for (int c : DRAW_ORDER) stroke(canvas, stubWorld[c], stubPaths[c], side, Math.max(floor, ROAD_M[c] * ik));
      if (s.route.length > 0) {
        stroke(canvas, routeWorld, routePath, road, Math.max(floor, ROUTE_M * ik));
        carpet.setStrokeWidth(Math.max(dm(LINE_MIN_PX), Math.min(dm(LINE_MAX_PX), LINE_M * ik)));
        carpet.setColor(modeColor(Feed.mode));
        carpet.setAlpha(242);
        canvas.drawPath(carpetPath, carpet);
      }
      canvas.restore();
    }

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
    canvas.restore();
    long t2 = SystemClock.uptimeMillis();
    overlay(canvas, now, free);

    long ms = SystemClock.uptimeMillis() - now;
    drawMs = (drawMs * 7 + ms) / 8;
    drawSum += ms;
    // where a frame's time goes: the camera and zoom, building the road shapes, stroking them, the cards
    setupSum += t0 - now;
    pathSum += t1 - t0;
    strokeSum += t2 - t1;
    cardSum += ms - (t2 - now);
    drawCount++;
    // every 900 ticks, with how many of them found nothing new to draw
    if (logDue) {
      logDue = false;
      XposedBridge.log("aaroad: map draw " + drawSum / drawCount + " ms average over " + drawCount + " frames ("
          + setupSum / drawCount + " setup, " + pathSum / drawCount + " shapes, " + strokeSum / drawCount + " roads, "
          + cardSum / drawCount + " cards), " + (checks > 0 ? 100L * skipped / checks : 0) + "% frames skipped"
          + (gpu != null ? ", gl " + gpu.takeMs() + " ms" : "")
          + (loggedCanvas ? "" : ", canvas " + (canvas.isHardwareAccelerated() ? "GPU" : "software") + ", view "
             + (isHardwareAccelerated() ? "GPU" : "software")));
      loggedCanvas = true;
    }
  }

  // One road layer: this frame's kept pieces through the world matrix, where widths are k times larger since the
  // matrix divides by k, then the near pieces built in the car's frame at the display width.
  private void stroke(Canvas canvas, Path kept, Path near, Paint p, float width) {
    if (!kept.isEmpty()) {
      canvas.save();
      canvas.concat(world);
      p.setStrokeWidth((float) (width * k));
      canvas.drawPath(kept, p);
      canvas.restore();
    }
    if (!near.isEmpty()) {
      p.setStrokeWidth(width);
      canvas.drawPath(near, p);
    }
  }

  // The kept pieces in view this frame into kept[class], the ones reaching too near the camera turned and clipped
  // into near[class]. A piece is placed by the corners of its box in the car's frame.
  private void pieces(Chunks pc, Path[] kept, Path[] near, float ce, float cn, float ch, float sh, float ik) {
    for (int i = 0; i < pc.path.length; i++) {
      float[] b = pc.box[i];
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
      int c = pc.cls[i];
      if (z0 >= NEAR_Z) kept[c].addPath(pc.path[i]);
      else line(pc.pts[i], 0, near[c], ce, cn, ch, sh, ik, Float.NaN, 0);
    }
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
    if (!carpetStart(s, ce, cn, reach)) return;
    line(s.route[cRun], cSeg, carpetPath, ce, cn, ch, sh, ik, cE, cN);
    for (int r = cRun + 1; r < s.route.length; r++) {
      float[] b = s.routeBox[r];
      if (b[2] < ce - reach || b[0] > ce + reach || b[3] < cn - reach || b[1] > cn + reach) continue;
      line(s.route[r], 0, carpetPath, ce, cn, ch, sh, ik, Float.NaN, 0);
    }
  }

  // Where the carpet starts: the nearest point of the route to the car, as run cRun, segment cSeg, point (cE, cN).
  private int cRun, cSeg;
  private float cE, cN;

  private boolean carpetStart(Scene s, float ce, float cn, float reach) {
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
    cRun = bestRun;
    cSeg = bestSeg;
    cE = be;
    cN = bn;
    return bestRun >= 0;
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

  // ---- Peter's mockup (2026-10-30, second version): a turn card and a trip card down the left, the next street as
  // a tag over the map, the speed limit at the right, all on the dash's black. No close button: nothing on the
  // cluster takes a touch.

  private static final float COL = 300;  // the dark band behind the cards, solid as far as they reach: roads showed between them
  // left of x 36 the tach needle rests on the cards at idle (Peter's photo 2026-10-29)
  // the turn card starts 28 down: at 12 its top left corner almost met the tach ring, and at 18 it still looked
  // cut off by it (Peter's photos 2026-10-30)
  private static final float CARD_L = 36, CARD_R = 270, TURN_T = 28, TURN_B = 96, TRIP_R = 250, TRIP_T = 104, TRIP_B = 152;
  private static final float STRIP_W = 10, TEXT_X = 116;
  // every card ends in a lean, its top right corner this much further right than its bottom right (Peter's mockup)
  private static final float SLANT = 14, RADIUS = 10, CORNER = 5;
  private static final int ACCENT = 0xfff2f2f2, ACCENT_DIM = 0xff3a3a3a;
  // the music card under the trip card, with or without a route: the cover, where the sound comes from, the song,
  // and whether it plays. Only a glyph for that, no skip buttons, since the dash takes no touch.
  // as narrow as the trip card: at 270 it touched the car without a route (Peter 2026-10-30)
  private static final float SONG_T = 160, SONG_B = 208, SONG_R = 250, COVER_L = 46, COVER_T = 164, COVER_PX = 40, SONG_X = 96;
  private static final float GLYPH_X = 218, GLYPH_R = 14;
  private static final int SPOTIFY = 0xff1ed760;
  private static final float LOGO_R = 7;

  // the three sound waves of Spotify's logo (simple-icons' 24-unit outline, disc centred at 12,12), cut out of its
  // green disc: 0 move x y, 1 curve x1 y1 x2 y2 x y, 2 close
  private static final float[] SPOTIFY_WAVES = {
      0f, 17.521f, 17.34f, 1f, 17.281f, 17.699f, 16.861f, 17.82f, 16.5f, 17.58f, 1f, 13.68f, 15.84f, 10.14f,
      15.479f, 5.939f, 16.439f, 1f, 5.521f, 16.561f, 5.16f, 16.26f, 5.04f, 15.9f, 1f, 4.92f, 15.479f, 5.22f,
      15.12f, 5.58f, 15f, 1f, 10.14f, 13.979f, 14.1f, 14.4f, 17.22f, 16.32f, 1f, 17.64f, 16.5f, 17.699f,
      16.979f, 17.521f, 17.34f, 2f, 0, 18.961f, 14.04f, 1f, 18.66f, 14.46f, 18.12f, 14.64f, 17.699f, 14.34f,
      1f, 14.46f, 12.36f, 9.54f, 11.76f, 5.76f, 12.96f, 1f, 5.281f, 13.08f, 4.74f, 12.84f, 4.62f, 12.36f, 1f,
      4.5f, 11.88f, 4.74f, 11.339f, 5.22f, 11.219f, 1f, 9.6f, 9.9f, 15f, 10.561f, 18.72f, 12.84f, 1f,
      19.081f, 13.021f, 19.26f, 13.62f, 18.961f, 14.04f, 2f, 0, 19.081f, 10.68f, 1f, 15.24f, 8.4f, 8.82f,
      8.16f, 5.16f, 9.301f, 1f, 4.56f, 9.48f, 3.96f, 9.12f, 3.78f, 8.58f, 1f, 3.6f, 7.979f, 3.96f, 7.38f,
      4.5f, 7.199f, 1f, 8.76f, 5.939f, 15.78f, 6.179f, 20.221f, 8.82f, 1f, 20.76f, 9.12f, 20.94f, 9.84f,
      20.64f, 10.38f, 1f, 20.341f, 10.801f, 19.62f, 10.979f, 19.081f, 10.68f, 2f};

  private static Path spotifyWaves(float cx, float cy, float r) {
    Path out = new Path();
    float k = r / 12;
    for (int i = 0; i < SPOTIFY_WAVES.length; ) {
      float op = SPOTIFY_WAVES[i++];
      if (op == 0) {
        out.moveTo(cx + (SPOTIFY_WAVES[i] - 12) * k, cy + (SPOTIFY_WAVES[i + 1] - 12) * k);
        i += 2;
      } else if (op == 1) {
        float[] c = new float[6];
        for (int j = 0; j < 6; j += 2) {
          c[j] = cx + (SPOTIFY_WAVES[i + j] - 12) * k;
          c[j + 1] = cy + (SPOTIFY_WAVES[i + j + 1] - 12) * k;
        }
        out.cubicTo(c[0], c[1], c[2], c[3], c[4], c[5]);
        i += 6;
      } else {
        out.close();
      }
    }
    return out;
  }

  private void overlay(Canvas canvas, long now, boolean free) {
    if (!free) canvas.drawRect(0, 0, COL, 215, mask);
    canvas.drawRect(0, 0, 584, 78, fog);
    for (int i = 0; i < labelCount; i++) {
      if (!labelOn[i]) continue;
      canvas.drawText(labelText[i], labelX[i], labelY[i] + 4, halo);
      canvas.drawText(labelText[i], labelX[i], labelY[i] + 4, label);
    }

    View lanes = find("drive_way_layout");
    float streetY = lanes != null && lanes.isShown() ? navi.getTop() + lanes.getBottom() + 22 : 48;
    boolean hasSign = Feed.limitMs > 0;
    // the road the car is on, from the comma's route. Android Auto's next_road is the next turn's street, already on the
    // turn card; over the map it read as the current road (Peter's photo 2026-10-30, East Bethany Drive was the next turn)
    // With no route, or a route step without a road name, the street found under the car in the road tiles
    // (Peter 2026-10-30).
    CharSequence name = !TextUtils.isEmpty(Feed.road) ? Feed.road : onRoad;
    tagBox[2] = tagBox[0];
    if (!TextUtils.isEmpty(name)) {
      // a tag centred over the map between the cards and the sign, pointing down at the road
      float left = free ? 0 : CARD_R, right = hasSign ? 466 : 584;
      float maxw = Math.min(right - left - 40, 300), size = 18;
      float cx = free && hasSign ? 292 : (left + right) / 2;
      street.setTextSize(size);
      while (street.measureText(name.toString()) > maxw && size > 14) street.setTextSize(--size);
      String n = TextUtils.ellipsize(name, street, maxw, TextUtils.TruncateAt.END).toString();
      float w = street.measureText(n) + 24, t = streetY - 19, b = streetY + 7;
      RectF r = new RectF(cx - w / 2, t, cx + w / 2, b);
      tagBox[0] = cx - w / 2;
      tagBox[1] = t;
      tagBox[2] = cx + w / 2;
      tagBox[3] = b + 7;
      Path tip = new Path();
      tip.moveTo(cx - 7, b - 1);
      tip.lineTo(cx, b + 7);
      tip.lineTo(cx + 7, b - 1);
      canvas.drawRoundRect(r, 7, 7, panel);
      canvas.drawRoundRect(r, 7, 7, edge);
      canvas.drawPath(tip, panel);  // over the edge where the pointer leaves the tag
      canvas.drawPath(tip, edge);
      canvas.drawText(n, cx, streetY, street);
    }

    if (hasSign) {
      double v = Feed.limitMs * (Feed.metric ? 3.6 : 2.23694);
      speedSign(canvas, String.valueOf(Math.round(v)), 474, 30, 40, 52);
    }

    // the turn card: a blue strip down its left edge that empties over the last 500 ft as the stock bar did,
    // the stock arrow, the distance, and the road
    TextView dist = (TextView) find("seg_dist");
    CharSequence d = dist != null && dist.getVisibility() == VISIBLE ? dist.getText() : null;
    View icon = find("turn_icon");
    // the next turn's street, Android Auto's next_road. It read the street tag's name (the road the car is on) from
    // bdc227553 on, so the card named the current road instead of the turn (Peter's photo 2026-10-30)
    TextView next = (TextView) find("next_road");
    CharSequence turnRoad = next != null && next.getVisibility() == VISIBLE ? next.getText() : null;
    if (!free && (!TextUtils.isEmpty(d) || (icon != null && icon.isShown()))) {
      canvas.drawPath(turnCard, panel);
      double ft = TextUtils.isEmpty(d) ? -1 : Meter.feet(d);
      float lit = ft < 0 || ft > 500 ? TURN_B : TURN_B - (float) (ft / 500) * (TURN_B - TURN_T);
      canvas.save();
      canvas.clipPath(turnCard);
      canvas.drawRect(CARD_L, TURN_T, CARD_L + 0.4f * (CARD_R - CARD_L), TURN_B, turnGlow);  // a soft light from the left
      fill.setColor(ACCENT_DIM);
      canvas.drawRect(CARD_L, TURN_T, CARD_L + STRIP_W, lit, fill);
      fill.setColor(ACCENT);
      canvas.drawRect(CARD_L, lit, CARD_L + STRIP_W, TURN_B, fill);
      canvas.restore();
      canvas.drawPath(turnCard, edge);
      Drawable arrow = icon instanceof ImageView && icon.isShown() ? ((ImageView) icon).getDrawable() : null;
      if (arrow != null) {
        Rect was = arrow.copyBounds();
        arrow.setBounds(ICON_LEFT, ICON_TOP, ICON_LEFT + ICON, ICON_TOP + ICON);
        arrow.draw(canvas);
        arrow.setBounds(was);  // the stock view draws it again when the map goes
      }
      // a brighter rim on the strip's curve, lit as far as the strip is
      canvas.save();
      canvas.clipPath(rect(CARD_L - 2, TURN_T - 2, CARD_L + RADIUS + 2, lit));
      rim.setColor(0xff4a4a4a);
      canvas.drawPath(turnRim, rim);
      canvas.restore();
      canvas.save();
      canvas.clipPath(rect(CARD_L - 2, lit, CARD_L + RADIUS + 2, TURN_B + 2));
      rim.setColor(0xffffffff);
      canvas.drawPath(turnRim, rim);
      canvas.restore();

      float textW = CARD_R - SLANT - TEXT_X - 6;  // clear of the lean at the road name's baseline
      if (!TextUtils.isEmpty(d)) {
        String t = d.toString().trim();
        int sp = t.lastIndexOf(' ');
        String num = sp > 0 ? t.substring(0, sp) : t, u = sp > 0 ? t.substring(sp + 1) : "";
        float size = 36;  // a long distance shrinks to fit
        big.setTextSize(size);
        while (big.measureText(num) + 5 + unit.measureText(u) > textW && size > 24) big.setTextSize(--size);
        canvas.drawText(num, TEXT_X, 62, big);
        canvas.drawText(u, TEXT_X + big.measureText(num) + 5, 62, unit);
      }
      if (!TextUtils.isEmpty(turnRoad)) canvas.drawText(fit(turnRoad.toString(), cardRoad, textW), TEXT_X, TextUtils.isEmpty(d) ? 68 : 86, cardRoad);
    }

    // the trip card: time and distance left, a divider, the arrival time
    if (!free && Feed.remainingSec >= 0) {
      canvas.drawPath(tripCard, panel);
      canvas.drawPath(tripCard, edge);
      double left = Math.max(0, Feed.remainingSec - (now - Feed.stateAt) / 1000.0);
      long min = Math.round(left / 60);
      String n = min >= 60 ? (min / 60) + " h " + (min % 60) + " min" : min + " min";
      String at = clock.format(new Date(System.currentTimeMillis() + (long) (left * 1000)));
      float div = CARD_L + 112, mid = (TRIP_T + TRIP_B) / 2;
      boolean far = Feed.remainingMeters >= 0;
      canvas.drawText(fit(n, trip, div - CARD_L - 22), CARD_L + 14, far ? TRIP_T + 22 : mid + 7, trip);
      if (far) canvas.drawText(away(Feed.remainingMeters, Feed.metric), CARD_L + 14, TRIP_T + 41, cardDim);
      fill.setColor(0x40ffffff);
      canvas.drawRect(div, TRIP_T + 10, div + 1, TRIP_B - 10, fill);
      canvas.drawText(at, (div + TRIP_R - SLANT / 2) / 2 - tripClock.measureText(at) / 2, mid + 6, tripClock);  // centred at mid height
    }

    String status = Feed.mediaStatus, title = Feed.mediaTitle;
    if (status != null && !TextUtils.isEmpty(title)) song(canvas, status, title);
    else titleRoll.over = artistRoll.over = 0;
  }

  private void song(Canvas canvas, String status, String title) {
    boolean spotify = "spotify".equals(Feed.mediaSource);
    canvas.drawPath(songCard, panel);
    if (spotify) {  // Spotify's green comes in from the lean
      canvas.save();
      canvas.clipPath(songCard);
      canvas.drawRect(SONG_R - 0.35f * (SONG_R - CARD_L), SONG_T, SONG_R, SONG_B, greenGlow);
      canvas.restore();
    }
    canvas.drawPath(songCard, edge);

    RectF art = new RectF(COVER_L, COVER_T, COVER_L + COVER_PX, COVER_T + COVER_PX);
    Bitmap cover = Feed.cover;
    if (cover != null && Feed.mediaArt != 0 && Feed.coverId == Feed.mediaArt) {
      coverClip.rewind();
      coverClip.addRoundRect(art, 4, 4, Path.Direction.CW);
      canvas.save();
      canvas.clipPath(coverClip);
      canvas.drawBitmap(cover, null, art, smooth);
      canvas.restore();
    } else {
      fill.setColor(0xff2a2a2a);  // no cover (phone audio) or not here yet
      canvas.drawRoundRect(art, 4, 4, fill);
    }

    float w = GLYPH_X - GLYPH_R - 6 - SONG_X;
    String source = Feed.mediaSource, from = "spotify".equals(source) ? "Spotify" : "phone".equals(source) ? "Phone" : "";
    float x = SONG_X;
    if (spotify) {
      fill.setColor(SPOTIFY);
      canvas.drawCircle(x + LOGO_R, SONG_T + 10, LOGO_R, fill);
      canvas.drawPath(logoWaves, waves);
      x += 2 * LOGO_R + 4;
    }
    if (!from.isEmpty()) canvas.drawText(fit(from, songFrom, w - (x - SONG_X)), x, SONG_T + 14, songFrom);
    titleRoll.draw(canvas, title, songTitle, SONG_X, SONG_T + 30, w);
    String artist = Feed.mediaArtist;
    if (!TextUtils.isEmpty(artist)) artistRoll.draw(canvas, artist.toUpperCase(Locale.US), songArtist, SONG_X, SONG_T + 44, w);
    else artistRoll.over = 0;

    // pause bars while it plays, a play triangle while paused
    float cy = (SONG_T + SONG_B) / 2;
    fill.setColor(0xff2e2e2e);
    canvas.drawCircle(GLYPH_X, cy, GLYPH_R, fill);
    // a green ring round the button fills clockwise from the top as the song plays (Peter 2026-10-30)
    long dur = Feed.mediaDur, pos = Feed.mediaPos;
    if (dur > 0 && pos >= 0) {
      if ("playing".equals(status)) pos += SystemClock.uptimeMillis() - Feed.mediaAt;
      float done = Math.min(1f, pos / (float) dur);
      RectF ringBox = new RectF(GLYPH_X - GLYPH_R, cy - GLYPH_R, GLYPH_X + GLYPH_R, cy + GLYPH_R);
      ring.setColor(0x33ffffff);
      canvas.drawArc(ringBox, 0, 360, false, ring);
      ring.setColor(SPOTIFY);
      canvas.drawArc(ringBox, -90, 360 * done, false, ring);
    }
    fill.setColor(Color.WHITE);
    if ("playing".equals(status)) {
      canvas.drawRect(GLYPH_X - 5, cy - 6, GLYPH_X - 2, cy + 6, fill);
      canvas.drawRect(GLYPH_X + 2, cy - 6, GLYPH_X + 5, cy + 6, fill);
    } else {
      Path play = new Path();
      play.moveTo(GLYPH_X - 4, cy - 7);
      play.lineTo(GLYPH_X + 7, cy);
      play.lineTo(GLYPH_X - 4, cy + 7);
      play.close();
      canvas.drawPath(play, fill);
    }
  }

  // a card: the left end rounded, the right end leaning in SLANT from top to bottom with small rounded corners
  private static Path card(float l, float t, float r, float b) {
    float dx = SLANT / (b - t) * CORNER;  // how far the lean moves in over a corner
    Path p = new Path();
    p.moveTo(l + RADIUS, t);
    p.lineTo(r - CORNER, t);
    p.quadTo(r, t, r - dx, t + CORNER);
    p.lineTo(r - SLANT + dx, b - CORNER);
    p.quadTo(r - SLANT, b, r - SLANT - CORNER, b);
    p.lineTo(l + RADIUS, b);
    p.quadTo(l, b, l, b - RADIUS);
    p.lineTo(l, t + RADIUS);
    p.quadTo(l, t, l + RADIUS, t);
    p.close();
    return p;
  }

  // the rounded left end of a card, for its rim
  private static Path leftEnd(float l, float t, float b) {
    Path p = new Path();
    p.moveTo(l + RADIUS, t);
    p.quadTo(l, t, l, t + RADIUS);
    p.lineTo(l, b - RADIUS);
    p.quadTo(l, b, l + RADIUS, b);
    return p;
  }

  // the leaning right end of a card, for its rim
  private static Path rightEnd(float r, float t, float b) {
    float dx = SLANT / (b - t) * CORNER;
    Path p = new Path();
    p.moveTo(r - CORNER, t);
    p.quadTo(r, t, r - dx, t + CORNER);
    p.lineTo(r - SLANT + dx, b - CORNER);
    p.quadTo(r - SLANT, b, r - SLANT - CORNER, b);
    return p;
  }

  private static Path rect(float l, float t, float r, float b) {
    Path p = new Path();
    p.moveTo(l, t);
    p.lineTo(r, t);
    p.lineTo(r, b);
    p.lineTo(l, b);
    p.close();
    return p;
  }

  // A song line too long for the card scrolls (Peter 2026-10-30): it holds at the start, slides left slowly until
  // its end shows, holds there, then jumps back and starts over. Lines that fit stay still. Each line keeps its
  // own clock and restarts when its text changes.
  private static final class Roll {
    private static final long HOLD_MS = 2000;
    private static final float PX_PER_S = 25;
    private String text;
    private long since;
    float over;  // px the line is longer than its room as last drawn; 0 when it fits or is not shown

    void draw(Canvas canvas, String s, TextPaint p, float x, float y, float w) {
      long now = SystemClock.uptimeMillis();
      if (!s.equals(text)) { text = s; since = now; }
      over = p.measureText(s) - w;
      if (over <= 0) { canvas.drawText(s, x, y, p); return; }
      float dx = dx(now);
      canvas.save();
      canvas.clipRect(x, SONG_T, x + w, SONG_B);  // sideways only: capital accents reach up past the line
      canvas.drawText(s, x - dx, y, p);
      canvas.restore();
    }

    private float dx(long now) {
      long slide = (long) (over / PX_PER_S * 1000), t = (now - since) % (2 * HOLD_MS + slide);
      return t < HOLD_MS ? 0 : Math.min(over, (t - HOLD_MS) * PX_PER_S / 1000);
    }

    // how far the line is scrolled at `now`, in whole px, so the tick asks for a frame while it slides
    int shift(long now) {
      return over > 0 ? (int) dx(now) : 0;
    }
  }

  private final Roll titleRoll = new Roll(), artistRoll = new Roll();

  private static String fit(String s, TextPaint p, float w) {
    return TextUtils.ellipsize(s, p, w, TextUtils.TruncateAt.END).toString();
  }

  // Distance left to the destination, as the phone shows it.
  static String away(double m, boolean metric) {
    if (metric) return m < 1000 ? Math.round(m / 10) * 10 + " m" : String.format(Locale.US, "%.1f km", m / 1000);
    double mi = m / 1609.344;
    return mi < 0.1 ? Math.round(m * 3.28084 / 50) * 50 + " ft" : String.format(Locale.US, "%.1f mi", mi);
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
