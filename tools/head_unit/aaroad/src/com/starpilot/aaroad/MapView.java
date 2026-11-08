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
import android.graphics.Region;
import android.graphics.Shader;
import android.graphics.Typeface;
import android.os.Handler;
import android.os.Looper;
import android.os.SystemClock;
import android.text.TextPaint;
import android.text.TextUtils;
import android.view.View;
import android.view.ViewGroup;
import java.io.File;
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
  // the camera's place, tilt and lens: CloseUp's, which tilts it further toward the horizon zoomed in (CloseUp.pitch)
  private static final double CAM_H = CloseUp.CAM_H, CAM_ZC = CloseUp.CAM_ZC, PITCH = CloseUp.PITCH;
  static final double R0 = 70.03;          // display metres from the car to the range's far edge (y 86)
  static final float PX_DM = 1.7156f;      // px per display metre across the road at the car
  // a little past the screen; see FreeView (2026-10-31: the right edge 240 left the free-drive map's top right bare)
  static final float BOX_X0 = FreeView.X0, BOX_X1 = FreeView.X1, BOX_Z0 = FreeView.Z0, BOX_Z1 = FreeView.Z1;
  private static final float BOX_R = 430;          // display metres: covers the box from the car
  // Road shapes are kept between frames (Peter 2026-10-30: building them was 46 of 158 ms a frame on the Tegra): each
  // run is cut into pieces of at most CHUNK_M m, each a Path in world metres built with the scene, and a frame only
  // picks the pieces in view and draws them through a matrix. A piece reaching nearer than NEAR_Z display metres is
  // still turned and clipped point by point: the camera's horizon line behind the car is at z -264, and anything past
  // it would come out on the wrong side of the screen.
  private static final float CHUNK_M = 150, NEAR_Z = -200;
  static final float FREE_X = 292;  // the meter's centre
  static final float CAR_X = 369, CAR_Y = 180, CAR_PX = 26;  // car point is (26, 26) in the sprite
  // Brake lights (Peter 2026-10-31): the sprite's tail lights, the C shapes at rows 30 to 38, glow red while the pose
  // says the car is braking, fading in and out over GLOW_MS. The view redraws only while the glow changes; the tick
  // runs every GLOW_TICK_MS during a fade on the GPU path, so the fade shows as a few steps rather than one.
  static final float LIGHT_DX = 16, LIGHT_DY = 8;  // each light's centre from the car point
  private static final long GLOW_MS = 150, GLOW_TICK_MS = 33;
  private final Paint glowPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
  private boolean glowOn;
  private float glowFrom, drawnGlow;
  private long glowAt;
  // Under an overpass (Under, Peter 2026-11-01): where a raised deck is drawn over the car, the car is cut away so the
  // deck shows, and a light outline of it (OUTLINE_C, a faint OUTLINE_FILL inside) shows through, easing over
  // Under.FADE_MS. The sprite's body is its alpha over OUTLINE_ALPHA (under it is the soft shadow). cover holds the
  // decks over the car, kept from the last frame that had one while the outline fades out; deckClip their outline on
  // screen. The tick runs every GLOW_TICK_MS while under a deck on the GPU path, as the deck slides by the car.
  private static final int OUTLINE_C = 0xfff4f4f4, OUTLINE_FILL = 0x30ffffff, OUTLINE_ALPHA = 170;
  // Back on (Peter 2026-11-08, after turning it off the same day): he wants the outline under overpasses. The
  // under-deck check could outline the car while it drove on the bridge itself; that is part of the deck audit.
  private static final boolean UNDER_OUTLINE = true;
  private static Bitmap outline;
  private final Under.Fade under = new Under.Fade();
  private final int[] cover = new int[48], tickCover = new int[1];
  private int coverN;
  private CloseUp.Detail coverOf;
  private final float[] coverQ = new float[4];
  private final Path deckClip = new Path();
  private final Paint carPaint = new Paint(Paint.FILTER_BITMAP_FLAG), outlinePaint = new Paint(Paint.FILTER_BITMAP_FLAG);
  private final Frame slowFrame = new Frame();
  private float drawnUnder;
  private float drawnUE = Float.NaN, drawnUN, drawnUH;
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
  // Road colours by tile class, kerb and surface, the same on the GPU layer: grey, but motorways and trunks in a muted
  // gold and their ramps a dimmer gold, so a highway reads as one at a glance (Peter 2026-10-31). The gold is dark
  // enough to keep the orange route line and the amber traffic colour (#ffb300) the brightest things on the map.
  // The route's own road is grey (KERB_C/ROAD_C), drawn under the tile roads' surfaces, so on a highway it is the gold.
  static final int KERB_C = 0xff6a6a6a, ROAD_C = 0xff383838;
  static final int[] KERBS = {0xff54451a, KERB_C, KERB_C, 0xff433816, KERB_C};
  static final int[] ROADS = {0xff7a6420, ROAD_C, ROAD_C, 0xff5a4b1e, ROAD_C};
  // Bridges (format 3 tiles, road_tiles.BRIDGE) go over the roads under them, Peter 2026-10-31: after every other
  // road's kerb and surface, in their own colours, with a dark casing CASING_PX wide each side so the road below
  // reads as passing under. A bridge's class in the scene is its tile class + BRIDGE; the carpet stays on top of
  // the deck it rides on, but not of a higher deck it passes under (Peter 2026-11-08, overriding the above: Apple
  // Maps shows the route diving under a bridge it doesn't use). The under-a-higher-deck piece of carpet is drawn
  // here, before the bridges, so the bridge draws over it -- see Scene.routeUnder, Under.underCuts, and the
  // carpetUnderPaths split of carpet()/carpetLine().
  static final int BRIDGE = 5, CASING_C = 0xff101010;
  static final float CASING_PX = 2;
  // Part B (Peter 2026-11-08 approved): the branch not taken at a split ahead -- the other ramp, or the mainline
  // continuing ahead when the route exits -- draws dimmed, the way Apple Maps does it. The dimmed half of a stub
  // road's class space, same doubling trick as the route's under-a-deck half above (GlRoads.LEVELS): a dimmed
  // piece's class is its plain class (+ BRIDGE * layer if it's on a deck) plus DIM_OFF, so it shares every other
  // class's drawing code and only needs its own kerb/surface colour pass. See stubsDim() and Scene.stubsDim.
  static final int DIM_OFF = BRIDGE * CloseUp.LAYERS;
  // the night canvas clears to this (see the drawColor call below); blended 50% with a road's own colour for the
  // untaken branch, as the background it's meant to recede toward
  private static final int BG = 0xff050505;
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
  // Part B (Peter 2026-11-08 approved, "grey out the ramp not taken"): a stub run whose end lands within
  // FORK_MERGE_M of the route (loose, OSM nodes rarely coincide exactly, unlike the junction dots above which only
  // need to find a crossing) and which leaves there at between FORK_MIN_DEG and FORK_MAX_DEG off the route's own
  // heading is the other branch of a split -- too small an angle and it is the route's own road re-matched (or a
  // frontage road running alongside, Under's ON_DECK-style near-parallel case); too large and it is a road merely
  // crossing the route, which stubsDim() must leave alone. Design values, not yet checked against a real fork's
  // OSM geometry (no close-up fork case in the harnesses at the time this was written) -- revisit against a real
  // ramp if the dimming is ever seen in the wrong place, or missing, on road. DIM_M is the spec's own number
  // (first ~150-200 m of the untaken branch), not measured either.
  private static final float FORK_MERGE_M = 15, FORK_MIN_DEG = 5, FORK_MAX_DEG = 70, DIM_M = 180;
  private static final double SCENE_R = 4000, MOVE_R = 600, ROADS_R = 700, CELL = 500, DENSE = 100;
  // SLOW_MS: with the GPU layer drawing the map, this view only holds the car, tag, sign and cards, so the tick looks
  // for a change in them ten times a second
  private static final long SLOW_MS = 100;
  // the turn card goes once the comma's nav state is this old (it sends it once a second)
  private static final long NAV_STALE_MS = 3000;
  private static final long FRAME_MS = 66, IDLE_MS = 500, STALE_MS = 2000, TILE_REBUILD_MS = 1000;
  // once showing, the map rides out a position gap with the car held where it was (Peter 2026-10-30 photo: mid-route
  // the cluster flipped back to Honda Hack's page for a moment, once the comma's feed paused); see Liveness
  private static final String[] HIDE = {"background", "turn_layout", "route_remain", "cur_road", "compass_layout",
                                        "limited_speed", "limited_speed_text", "traffic_layout", "turn_icon"};
  // arrow in the turn card, right of its bar (Peter 2026-10-29: loose over the map it looked out of place).
  // Drawn here from the stock view's picture: moving the view itself put it 70 px right of this, over the road
  // name, on the car (2026-10-30), its parent being offset in a way the layout dump did not show.
  // 34 px beside the distance, down from 52, so the card fits the narrow column (Peter 2026-10-31)
  private static final int ICON = 34, ICON_LEFT = 52, ICON_TOP = 35;
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
  private final Paint grid = new Paint(), kerb = stroke(KERB_C, 0), road = stroke(ROAD_C, 0),
      carpet = stroke(Color.WHITE, 0), sideKerb = stroke(KERB_C, 0), side = stroke(ROAD_C, 0),
      fill = new Paint(), mask = new Paint(), fog = new Paint(), panel = new Paint(), edge = new Paint(),
      rim = new Paint(), ring = new Paint(), turnGlow = new Paint(), greenGlow = new Paint(),
      badgeBox = new Paint(Paint.ANTI_ALIAS_FLAG);
  // Inter on the cards and the street tag, sized to Peter's mockup; the speed sign keeps the system bold
  private final TextPaint big = text(30, inter(true), Color.WHITE), unit = text(15, inter(false), 0xffc8d0dc),
      street = text(17, inter(true), Color.WHITE), cardDim = text(13, inter(false), 0xffaab4c8),
      cardRoad = text(18, inter(true), Color.WHITE), badgeInk = text(15, inter(true), BADGE_INK),
      songFrom = text(11, inter(false), 0xffaab4c8), songTitle = text(15, inter(true), Color.WHITE),
      songArtist = text(12, inter(false), 0xffaab4c8),
      sign = text(8.5f, Typeface.DEFAULT_BOLD, 0xff111111),
      signBig = text(23, Typeface.DEFAULT_BOLD, 0xff111111);
  private final Path routePath = new Path(), gridPath = new Path(), coverClip = new Path();
  // the music card's shape and logo, built again only when the card moves (songAt, its top); the turn card's
  // shape follows its rows
  private Path songCard, logoWaves;
  private float songAt = -1;
  private final Paint smooth = new Paint(Paint.FILTER_BITMAP_FLAG);  // the 56 px cover scaled to 40
  private final Paint waves = new Paint(Paint.ANTI_ALIAS_FLAG);  // black: the waves are cut out of the green disc
  // by scene class: the tile classes, then their bridges, one bucket of BRIDGE classes per deck layer (1 to
  // CloseUp.LAYERS - 1) so draw() can paint a higher layer's casing and fill over a lower one's (Peter 2026-11-08,
  // "the highway lines simply overlay the street below, with no joint seams"). Doubled past DIM_OFF (Part B,
  // 2026-11-08) for the same classes dimmed, the untaken branch of a split (Scene.stubsDim) -- the free-drive
  // roadsChunks path through pieces() never sets a dim flag, so that half of these arrays stays empty there.
  private final Path[] stubPaths = paths(2 * BRIDGE * CloseUp.LAYERS);
  private final CloseView closeView = new CloseView();
  // this frame's kept pieces by class, in world metres, drawn through `world`; the near ones go in stubPaths/routePath
  private final Path[] stubWorld = paths(2 * BRIDGE * CloseUp.LAYERS);
  private final Path routeWorld = new Path();
  // the carpet in the car's frame by traffic: unknown or low in the mode's colour, then moderate, heavy, severe.
  // carpetUnderPaths (Peter 2026-11-08) holds the same four, split off where Scene.routeUnder says the route runs
  // under a higher deck -- drawn before the bridges/decks instead of after, so it goes under them (Apple Maps style).
  private final Path[] carpetPaths = {new Path(), new Path(), new Path(), new Path()};
  private final Path[] carpetUnderPaths = {new Path(), new Path(), new Path(), new Path()};
  static final int[] TRAFFIC = {0xffffb300, 0xffff5722, 0xffc62828};  // moderate, heavy, severe; GlRoads draws the same
  private final Path[] routeWorlds = {routeWorld}, routeFrames = {routePath};
  private final Matrix world = new Matrix();  // world metres to the car's frame, this frame
  private final float[] worldV = new float[9], tickEN = new float[2];
  // What the last frame showed, so the tick only asks for a frame when something on screen would change (Peter
  // 2026-10-30): the car moved a pixel or turned, the zoom is still easing, or anything else drawn changed (stamp()).
  private Scene drawnScene;
  private float drawnE, drawnN;
  private double drawnH, drawnK, drawnBase;
  private boolean drawnMoving, zoomSettled, logDue, loggedCanvas;
  private long drawnAt, drawnStamp;
  private int drawnRoll;
  private final Rect rollBox = new Rect();
  private int checks, skipped, drawsAtLog;
  private long rollsAtLog;
  private long logFrom;
  private String drawnRoad;
  private Feed.Pose pokedPose;
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
  private String hideWhy = "not started";  // why the map is hidden, for the log
  private boolean holding;                // up only because the comma's feed still talks through a GPS gap
  private long poseGapFrom;               // uptime the position went stale while up (0: it is fresh)
  private static String whereLogged;  // Events.status() as last logged
  private double k = 600 / R0;
  private float t0, t1;  // clip parameters of the segment being clipped
  private final float[] cam = new float[9];  // the camera's values, to place labels where the roads are drawn
  private double camPitch = CloseUp.PITCH;  // the tilt cam and camera are at (useCamera), UI thread
  private String onRoad, switchTo;  // street shown as the one the car is on (null: none); the one waiting to replace it
  private long roadAt, switchSince, lostSince, labelAt;
  private int labelCount;
  private final String[] labelText = new String[LABELS];
  private final boolean[] labelRef = new boolean[LABELS];  // the label is a route number, drawn as a badge
  private final float[] labelE = new float[LABELS], labelN = new float[LABELS], labelW = new float[LABELS],
      labelX = new float[LABELS], labelY = new float[LABELS];
  private final boolean[] labelOn = new boolean[LABELS];
  private final float[] tagBox = new float[4];  // the street tag as last drawn, with its pointer; zero size when none
  volatile float[] tagShown = new float[4];     // the same, handed to the GPU layer whenever it changes
  volatile Labels labels;                       // the chosen labels, handed to the GPU layer at each choice

  // The labels as chosen, for the GPU layer: it moves them with the map on its own clock (Peter 2026-10-30).
  static final class Labels {
    final Scene s;  // whose anchor e, n are from: a rebuild moves it
    final String[] text;
    final float[] e, n, w;
    final int count;

    Labels(Scene s, String[] text, float[] e, float[] n, float[] w, int count) {
      this.s = s;
      this.text = text.clone();
      this.e = e.clone();
      this.n = n.clone();
      this.w = w.clone();
      this.count = count;
    }
  }

  // The labels' halo and text paints, fresh for the GPU layer's own thread, left aligned; then a route number's
  // badge text, box and dark rim (drawLabel).
  static TextPaint[] labelPaints() {
    TextPaint halo = text(11.5f, inter(false), 0xff050505), fill = text(11.5f, inter(false), 0xffc8ccd2);
    halo.setStyle(Paint.Style.STROKE);
    halo.setStrokeWidth(3);
    halo.setStrokeJoin(Paint.Join.ROUND);
    TextPaint box = text(1, null, Color.WHITE), rim = text(1, null, 0xff050505);
    return new TextPaint[] {halo, fill, text(LABEL_BADGE, inter(true), BADGE_INK), box, rim};
  }
  private final TextPaint[] labelBadge = labelPaints();  // the overlay's route-number labels, drawn as the GPU's are

  // A street label at left, base (the halo then the text), or a route number ("US 75", "I-635") as a white badge
  // with a dark rim, the way Google Maps writes them over the map (Peter 2026-10-31). p is labelPaints(). The GPU
  // layer draws its atlas with this too, so the two paths match; a badge is placeLabels' labelW wide, no wider.
  static void drawLabel(Canvas c, String t, boolean ref, float left, float base, TextPaint[] p) {
    if (!ref) {
      c.drawText(t, left, base, p[0]);
      c.drawText(t, left, base, p[1]);
      return;
    }
    // centred in the box blocked() keeps for a label (y - 9 to y + 7, y = base - 4), rim included
    badge(c, Shield.REF, t, left + 1, base - 5, LABEL_BADGE, p[2], p[3], p[4]);
  }

  // Badges: an exit number in Google's green with white text, a route number in white with black text. bs is the
  // badge's text size; the box is centred on mid, rounded, and 0.76 bs wider than its text.
  static final int EXIT_GREEN = 0xff1e7d32, BADGE_INK = 0xff111111;
  static final float LABEL_BADGE = 10.5f;  // a map label's badge text: its box with the rim is 16 px, the label's

  static String badgeText(Shield.Piece p) {
    return p.kind == Shield.EXIT ? "Exit " + p.text : p.text;
  }

  static float badgeW(String t, float bs, TextPaint ink) {
    ink.setTextSize(bs);
    return ink.measureText(t) + 0.76f * bs;
  }

  // Draws the badge with its left at x and returns its width; rim, when given, is a 1 px dark edge round it.
  static float badge(Canvas c, int kind, String t, float x, float mid, float bs, TextPaint ink, Paint box, Paint rim) {
    float w = badgeW(t, bs, ink), h = 0.66f * bs, r = 0.25f * bs;
    if (rim != null) c.drawRoundRect(new RectF(x - 1, mid - h - 1, x + w + 1, mid + h + 1), r + 1, r + 1, rim);
    box.setColor(kind == Shield.EXIT ? EXIT_GREEN : Color.WHITE);
    c.drawRoundRect(new RectF(x, mid - h, x + w, mid + h), r, r, box);
    ink.setColor(kind == Shield.EXIT ? Color.WHITE : BADGE_INK);
    c.drawText(t, x + 0.38f * bs, mid + 0.3635f * bs, ink);  // Inter's caps are 0.727 em: centred on mid
    return w;
  }
  private final TextPaint label = text(11.5f, inter(false), 0xffc8ccd2), halo = text(11.5f, inter(false), 0xff050505);

  // Positions are metres east (e) and north (n) of the anchor, in runs of e, n pairs.
  static final class Scene {
    double ax, ay, mpt;  // anchor in zoom-14 tile units; metres per tile unit there
    long routeId = -1;
    int tilesVersion;
    float[][] route = new float[0][], stubs = new float[0][], roads = new float[0][];
    int[] stubsCls = new int[0], roadsCls = new int[0];  // tile class of each run, + BRIDGE on a bridge
    String[] stubsName = new String[0], roadsName = new String[0];  // street name of each run, null when none
    float[][] routeBox, stubsBox, roadsBox;  // min e, min n, max e, max n of each run
    float[] junctions = new float[0];  // e, n of every place a side road meets or crosses the route
    byte[][] routeLevel = new byte[0][];  // traffic per segment of each route run, 0 unknown to 4 severe (Feed.Route.lvl)
    // whether each segment of each route run passes under a deck higher than the road it is on (Peter 2026-11-08,
    // Under.underCuts against s.close), found once here at build; carpet()/carpetLine() read it, no new work
    // per frame. Same run/segment shape as routeLevel, built right after s.close so the heights it reads are ready.
    byte[][] routeUnder = new byte[0][];
    // Part B (Peter 2026-11-08): 1 on a stubs segment that is the first DIM_M of a branch not taken at a split,
    // found once here at build by stubsDim() against the finished route and stubs; same run/segment shape as
    // stubsCls. draw() reads it through stubsChunks' dim field below, no work per frame.
    byte[][] stubsDim = new byte[0][];
    float[][] routeAlong = new float[0][];  // metres along the route at each point of each run, runs end to end
    float[] signals = new float[0];  // the traffic signal and stop icons, Signals.cluster: e, n, kind, along in fours
    Chunks routeChunks = new Chunks(), stubsChunks = new Chunks(), roadsChunks = new Chunks();
    // with no route: the rectangle of roads kept, in the frame of heading bh (FreeView.area), and whether service
    // roads are in it; area null with a route
    float[] area;
    double bh;
    boolean minor = true;
    CloseUp.Detail close;  // the close-up view's raised bridges, lane lines and sidewalks (CloseView)
  }

  // The runs cut into pieces of at most CHUNK_M m (neighbours share their end point), each kept as a Path in world
  // metres (x east, y north), with its points, its box (min e, min n, max e, max n) and its class.
  static final class Chunks {
    Path[] path = new Path[0];
    float[][] pts = new float[0][], box = new float[0][];
    int[] cls = new int[0];
    float[] at = new float[0];  // metres along the runs, end to end, at each piece's first point
    // route chunks only (Peter 2026-11-08): 1 where the whole piece passes under a higher deck (Scene.routeUnder),
    // 0 otherwise; a piece is cut at a change the same way a traffic-level change cuts one (chunks()'s un param),
    // so GlRoads' route groups can split on it without a piece straddling both.
    int[] under = new int[0];
    // stubs chunks only (Peter 2026-11-08, Part B): 1 where the whole piece is the dimmed, untaken branch of a
    // split (Scene.stubsDim); cuts a piece the same way under does, so the dimmed and plain halves of a stub
    // never share one piece.
    int[] dim = new int[0];
  }

  // The roads, the route, the grid, the carpet and the fades over them drawn on the GPU under this view (GlRoads,
  // Peter 2026-10-30: stroking them in software was most of a frame on the Tegra). It runs its own clock, taking each
  // frame's camera from frame(). Meter adds it when GPU is set; the desktop harness has no GL and leaves it off. Until
  // it has drawn a frame, and for good if it fails, the view draws them itself.
  interface Roads {
    boolean ready();
    void poke();  // something changed while the car stands: draw now rather than at the idle rate
    String takeStats();  // GL ms a frame and the frame rate since the last call, for the log
    void setShown(boolean on);
  }

  static boolean GPU = true;
  Roads gpu;

  MapView(Context context, ViewGroup navi, String pkg) {
    super(context);
    this.navi = navi;
    this.pkg = pkg;
    // the SD card first; the meter process's own files dir when it may not write there, then its cache dir. Each
    // asked on its own: one that throws (or gives null) must not take the others with it
    Context app = null;
    try {
      app = context.getApplicationContext();
    } catch (Throwable ignored) {
    }
    Events.use(filesDir(app), filesDir(context), cacheDir(app), cacheDir(context));
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
  // At today's (cruising) tilt; a frame's own is Frame.cam.
  static float[] cameraValues() {
    return CloseUp.camera(PITCH, new float[9]);
  }

  // The UI thread's camera (labels, the car's outline under a deck, the software map) to f's tilt.
  private void useCamera(Frame f) {
    if (f.pitch == camPitch) return;
    System.arraycopy(f.cam, 0, cam, 0, 9);
    camera.setValues(cam);
    camPitch = f.pitch;
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
        Xlog.log("aaroad: Inter not loaded, system font instead: " + t);
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
      if (car != null) outline = outlineOf(car);
    } catch (Throwable t) {
      Xlog.log("aaroad: map car icon not loaded: " + t);
    } finally {
      try {
        if (zip != null) zip.close();
      } catch (Throwable ignored) {
      }
    }
  }

  // The car's outline for under a deck: a 2 px line on the edge of the sprite's body (one pixel each side of it), a
  // faint fill inside. Made once, with the sprite.
  static Bitmap outlineOf(Bitmap src) {
    int w = src.getWidth(), h = src.getHeight();
    int[] px = new int[w * h], out = new int[w * h];
    src.getPixels(px, 0, w, 0, 0, w, h);
    for (int y = 0; y < h; y++) {
      for (int x = 0; x < w; x++) {
        boolean in = body(px, w, h, x, y), edge = false;
        for (int dy = -1; dy <= 1 && !edge; dy++) {
          for (int dx = -1; dx <= 1 && !edge; dx++) edge = body(px, w, h, x + dx, y + dy) != in;
        }
        out[y * w + x] = edge ? OUTLINE_C : in ? OUTLINE_FILL : 0;
      }
    }
    return Bitmap.createBitmap(out, w, h, Bitmap.Config.ARGB_8888);
  }

  private static boolean body(int[] px, int w, int h, int x, int y) {
    return x >= 0 && y >= 0 && x < w && y < h && px[y * w + x] >>> 24 >= OUTLINE_ALPHA;
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
        long feedAge = Feed.stateAt > 0 ? now - Feed.stateAt : Long.MAX_VALUE;
        String why = Liveness.hideReason(shown, live, age, feedAge);
        if (why != null && !shown) why += pageHidden();
        hideWhy = why;
        noteGap(now, age, feedAge);
        setLive(why == null);
        String where = Events.status();
        if (where != null && !where.equals(whereLogged)) {  // once, and again only if it changes
          whereLogged = where;
          Xlog.log(where);
        }
        if (live) {
          rebuildIfNeeded(now);
          if (gpu != null && gpu.ready()) {
            slowTick(now);
          } else if (changed(now)) {
            invalidate();
            if (gpu != null) gpu.poke();
          } else {
            skipped++;
          }
          if (++checks % 900 == 150) logDue = true;  // with the next frame's timing
        }
      } catch (Throwable t) {
        say("aaroad: map tick failed: " + t);
      }
      // software drawing on the Tegra: if a frame costs more than half the frame time, draw less often
      // and while the car glides up or down for the lane row (lift), so it keeps with the map under it
      long delay = SLOW_MS;
      try {
        long t = SystemClock.uptimeMillis();
        long slow = glowing(t) || lift != drawnLift || drawnUnder > 0 || under.moving(t)
            ? Math.max(GLOW_TICK_MS, 2 * drawMs) : SLOW_MS;
        delay = !live ? IDLE_MS : gpu != null && gpu.ready() ? slow : Math.max(FRAME_MS, 2 * drawMs);
      } catch (Throwable t) {
        say("aaroad: map tick delay failed: " + t);
      } finally {
        tickAt = SystemClock.uptimeMillis();
        postDelayed(this, Math.min(delay, IDLE_MS));
      }
    }
  };

  // Peter's drive 2026-11-06: from about 21:00 this tick stopped running while the GPU layer went on at 30 a second,
  // so the scene was never rebuilt and the map drove off its roads into a blank page for over an hour (no error in
  // the log). The GPU thread calls this each frame; a tick quiet for TICK_LOST_MS is posted again, and said once.
  // Not static: Hook.initZygote loads this class in the zygote, which has no main looper yet. A static Handler threw
  // there at every head unit boot after the 11-06 install, and a class whose static init fails cannot be used again
  // until the next boot, so the map never appeared (Peter 2026-11-07).
  private final Handler main = new Handler(Looper.getMainLooper());
  private volatile long tickAt;
  private long kickedAt;
  static final long TICK_LOST_MS = 2000;

  long tickAge(long now) {
    long at = tickAt;
    return at == 0 ? -1 : now - at;
  }

  void kickTick(long now) {
    long at = tickAt;
    // a detached view's tick is stopped on purpose (onDetachedFromWindow)
    if (at == 0 || now - at < TICK_LOST_MS || now - kickedAt < TICK_LOST_MS || getWindowToken() == null) return;
    kickedAt = now;
    say("aaroad: map tick quiet for " + (now - at) + " ms, posted again");
    removeCallbacks(tick);
    main.removeCallbacks(tick);
    main.post(tick);
  }

  // With the GPU layer drawing everything that moves at 30 a second (Peter 2026-10-30), this view is the slow layer:
  // the street under the car and the labels are chosen here from the GPU layer's last frame, and the view redraws
  // only when the car, tag, sign or cards would look different, about once a second (the clock and countdown are in
  // the cards' key). The GPU layer is poked when the feed's state changes, so a new mode colour shows while it idles.
  private void slowTick(long now) {
    Scene s = scene;
    if (s == null) return;
    Frame f = null;
    synchronized (this) {
      if (latest.s != null) {
        latest.copyTo(tickFrame);
        f = tickFrame;
      }
    }
    if (f != null && f.s == s) {  // not one from before a rebuild, placed from the old anchor
      currentRoad(s, f.ce, f.cn, f.h, now);
      String shown = !TextUtils.isEmpty(Feed.road) ? Feed.road : onRoad;
      useCamera(f);
      placeLabels(s, f.ce, f.cn, f.ch, f.sh, f.ik, f.reach, f.free, shown, now);
      if (labelAt == now) {
        labels = new Labels(s, labelText, labelE, labelN, labelW, labelCount);
        gpu.poke();
      }
    }
    // the GPU layer idles at a standstill; a fix that would move the car on screen wakes it at once
    Feed.Pose p = pose();
    if (p != pokedPose) {
      pokedPose = p;
      double[] held = heldW, w = Feed.world(p.lat, p.lon, tickW);
      if (p.spd >= HOLD_SPEED || held == null || Math.hypot(w[0] - held[0], w[1] - held[1]) * s.mpt > HOLD_DRIFT) {
        gpu.poke();
      }
    }
    long st = stamp(now);
    String road = !TextUtils.isEmpty(Feed.road) ? Feed.road : onRoad;
    if (st != drawnStamp) gpu.poke();
    if (s != drawnScene || st != drawnStamp || !TextUtils.equals(road, drawnRoad)
        || !cardKey(now, s.route.length == 0).equals(cardsKey) || glow(now) != drawnGlow || Math.abs(lift - drawnLift) > 0.5f
        || (f != null && f.s == s && underWanted(f) != under.on()) || underMoving(now)) {
      invalidate();
    } else if (rolled(now) != drawnRoll) {
      titleRoll.box(rollBox);
      artistRoll.box(rollBox);
      if (rollBox.isEmpty()) invalidate();
      else invalidate(rollBox);
      rollBox.setEmpty();
      rollDraws++;
    } else {
      skipped++;
    }
  }

  private long rollDraws;

  @Override
  protected void onAttachedToWindow() {
    super.onAttachedToWindow();
    say("aaroad: map view attached " + id());
    removeCallbacks(tick);
    post(tick);
  }

  @Override
  protected void onDetachedFromWindow() {
    removeCallbacks(tick);
    main.removeCallbacks(tick);
    Feed.wanted = false;
    // Honda Hack re-creating its meter page takes this view with it; a new one comes with the new page (Meter)
    say("aaroad: " + (live ? "hidden" : "still hidden") + " reason=view detached " + id());
    live = false;
    super.onDetachedFromWindow();
  }

  private static File filesDir(Context c) {
    try {
      return c == null ? null : c.getFilesDir();
    } catch (Throwable t) {
      return null;
    }
  }

  private static File cacheDir(Context c) {
    try {
      return c == null ? null : c.getCacheDir();
    } catch (Throwable t) {
      return null;
    }
  }

  // One line to logcat and to the rolling file (Events).
  static void say(String line) {
    Xlog.log(line);
    Events.record(line);
  }

  private String id() {
    return "map@" + Integer.toHexString(System.identityHashCode(this));
  }

  // Which of Honda Hack's views is keeping its nav page off screen, so a page switch can be told from the window
  // or the whole display going.
  private String pageHidden() {
    if (navi.getWindowToken() == null) return " (nav page detached)";
    if (navi.getWindowVisibility() != VISIBLE) return " (meter window " + vis(navi.getWindowVisibility()) + ")";
    for (View v = navi; v != null; v = v.getParent() instanceof View ? (View) v.getParent() : null) {
      if (v.getVisibility() != VISIBLE) {
        String n = null;
        try {
          if (v.getId() != NO_ID) n = v.getResources().getResourceEntryName(v.getId());
        } catch (Throwable ignored) {
        }
        return " (" + (n != null ? n : v.getClass().getSimpleName()) + " " + vis(v.getVisibility()) + ")";
      }
    }
    return "";
  }

  private static String vis(int v) {
    return v == GONE ? "gone" : v == INVISIBLE ? "invisible" : String.valueOf(v);
  }

  // The GPS gaps the map rides out, one line when it starts holding past the old limit and one when the position
  // comes back, so the events file shows each gap and how long it was.
  private void noteGap(long now, long age, long feedAge) {
    boolean stale = live && age != Long.MAX_VALUE && age >= STALE_MS;
    if (stale && poseGapFrom == 0) poseGapFrom = now - age;
    boolean h = Liveness.holding(live, age, feedAge);
    if (h && !holding) say("aaroad: map held reason=no position for " + age / 1000 + " s, comma feed alive");
    holding = h;
    if (!stale && poseGapFrom != 0) {
      if (live) say(String.format(Locale.US, "aaroad: position back after %.1f s", (now - poseGapFrom) / 1000.0));
      poseGapFrom = 0;
    }
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
    if (!on) {
      holding = false;
      poseGapFrom = 0;
    }
    say("aaroad: " + (on ? "shown" : "hidden") + " reason=" + (on ? "page shown, position fresh" : hideWhy));
  }

  // Whether a frame now would look different from the last one drawn (Peter 2026-10-30: most frames at a light or on
  // a straight redrew the same picture). Reads the held pose, never sets it; at a standstill the car does not move
  // on screen unless the fix wanders past HOLD_DRIFT.
  private boolean changed(long now) {
    Scene s = scene;
    if (s == null) return false;
    if (s != drawnScene || now - drawnAt >= REDRAW_MS || !zoomSettled || stamp(now) != drawnStamp) return true;
    if (rolled(now) != drawnRoll) return true;
    if (glow(now) != drawnGlow || under.moving(now)) return true;
    Feed.Pose p = pose();
    double spd = Math.max(0, p.spd);
    boolean moving = spd >= HOLD_SPEED;
    if (moving != drawnMoving || Math.abs(zoomTarget(s, spd, now) / drawnBase - 1) > ZOOM_EPS) return true;
    if (!moving) {
      double[] held = heldW, w = Feed.world(p.lat, p.lon, tickW);
      return held == null || Math.hypot(w[0] - held[0], w[1] - held[1]) * s.mpt > HOLD_DRIFT;
    }
    double h = pose(s, p, now, false, tickEN, tickW);
    double px = Math.hypot(tickEN[0] - drawnE, tickEN[1] - drawnN) * PX_DM / drawnK;
    return px > MOVE_PX || Math.abs(Math.IEEEremainder(h - drawnH, 2 * Math.PI)) > Math.toRadians(TURN_DEG);
  }

  // Everything drawn besides the car's place and the zoom, folded into one number: the feed's state (mode, limit,
  // trip, street, song), the cover, Honda Hack's distance, next street and arrow, the lane bar, a song line's scroll.
  // Not the state's arrival time: the comma sends it once a second even when nothing in it changed, and each one
  // redrew the whole view for nothing (Peter 2026-11-08, parked: 4-6 redraws a second at 31-37 ms each). Not the
  // song lines' scroll either: that is rolled(), redrawn on its own small strip.
  private long stamp(long now) {
    long h = Feed.metric ? 1 : 2;
    h = h * 31 + Double.doubleToLongBits(Feed.limitMs);
    h = h * 31 + hash(Feed.mode);
    h = h * 31 + hash(Feed.road);
    h = h * 31 + hash(Feed.roadRef);
    h = h * 31 + hash(Feed.mediaStatus);
    h = h * 31 + hash(Feed.mediaTitle);
    h = h * 31 + Feed.coverId;
    h = h * 31 + hash(onRoad);
    Feed.Nav nav = turnNav(now);  // the turn card: its maneuver, text and the distance as shown, and gone when stale
    if (nav != null) {
      h = h * 31 + hash(nav.type);
      h = h * 31 + hash(nav.modifier);
      h = h * 31 + hash(nav.text);
      h = h * 31 + hash(nav.road);
      double m = turnLeft(nav, now);
      h = h * 31 + (m >= 0 ? hash(Turn.distance(m, Feed.metric)) : 1);
      h = h * 31 + java.util.Arrays.hashCode(nav.lanes);  // the lane row, and whether it shows yet
      h = h * 31 + java.util.Arrays.hashCode(nav.take);
      h = h * 31 + hash(nav.toward);
      h = h * 31 + (Lanes.shown(nav.lanes, m) ? 1 : 2);
    } else {
      h = h * 31 + 2;
    }
    View v = find("drive_way_layout");
    if (v != null) h = h * 31 + (v.isShown() ? 1 : 2);
    return h;
  }

  // How far the song lines are scrolled. A change redraws only their strip (rollBox): the lines are drawn live over
  // the cards' bitmap, not into it, so a scroll step no longer redraws the cards and the whole view (Peter 2026-11-08:
  // a long title sliding at 25 px a second redrew everything up to 10 times a second, ~35 ms each on the Tegra).
  private int rolled(long now) {
    return titleRoll.shift(now) * 31 + artistRoll.shift(now);
  }

  private static int hash(CharSequence c) {
    if (c == null) return 0;
    if (c instanceof String) return c.hashCode();
    int h = 1;
    for (int i = 0; i < c.length(); i++) h = h * 31 + c.charAt(i);
    return h;
  }

  // The last fix as one piece; from the separate fields when none has come as one (the harness sets only those).
  private static Feed.Pose pose() {
    Feed.Pose p = Feed.pose;
    return p != null ? p : new Feed.Pose(Feed.lat, Feed.lon, Feed.hdg, Feed.spd, Feed.poseAt);
  }

  // Where the car is now: fix p carried forward at its speed, at most half a second; e, n into en, and the
  // heading returned. hold (frame() only): at a standstill the car keeps the heading it last moved with, stays put
  // while the fix wanders within HOLD_DRIFT, and near the route points along it. The tick passes false: no change.
  // w: the caller's own scratch for the world position, so a frame allocates nothing (see Frame).
  private double pose(Scene s, Feed.Pose p, long now, boolean hold, float[] en, double[] w) {
    double spd = Math.max(0, p.spd), h = Math.toRadians(p.hdg);
    double ahead = Math.min(0.5, Math.max(0, (now - p.at) / 1000.0)) * spd;
    Feed.world(p.lat, p.lon, w);
    if (hold && spd >= HOLD_SPEED) {
      heldH = h;
      heldW = null;
    } else if (hold) {
      // a new array when the hold moves, rarely; w is scratch, and the UI thread reads heldW without the lock
      if (heldW == null || Math.hypot(w[0] - heldW[0], w[1] - heldW[1]) * s.mpt > HOLD_DRIFT) heldW = new double[] {w[0], w[1]};
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
    double rng = s.route.length == 0 ? FreeView.range(spd) : Math.max(200, Math.min(600, spd * 20));
    if (turn >= 0 && turn + TURN_LEN < rng) rng = Math.max(turn + TURN_LEN, CLOSE);
    // the lane row under the car takes the bottom of the map: a little closer in, so the turn still fits above it
    if (Lanes.at == Lanes.AT_CAR && s.route.length > 0 && lanesShown(now)) rng *= LANE_ZOOM;
    return rng;
  }

  // Whether the lane row shows now: the comma sent lanes for the next maneuver, and it is within Lanes.SHOW_M.
  private static boolean lanesShown(long now) {
    Feed.Nav nav = turnNav(now);
    return nav != null && Lanes.shown(nav.lanes, turnLeft(nav, now));
  }

  // The lane row under the car (Lanes.AT_CAR, Peter 2026-11-01): the map and car drawn LIFT px higher while it
  // shows, easing over LIFT_TAU (frame()), and the zoom LANE_ZOOM closer. lift is the last frame's, for the slow
  // layer and project(); drawnLift what the slow layer last drew the car with.
  static final float LIFT = 38;
  private static final double LIFT_TAU = 0.35, LANE_ZOOM = 0.85;
  private volatile float lift;
  private float drawnLift;

  // One frame's camera: where the car is drawn, which way it points, the zoom and where the carpet starts. The view's
  // onDraw and the GPU layer's own clock both take it from here, so the two agree; synchronized, as both may ask
  // while the GPU layer warms up.
  //
  // Each caller owns its Frame and frame() fills every field, so the 30 frames a second allocate nothing (Peter's drive
  // 2026-10-31: camera 160-225 ms in some frames; on Dalvik a thread that allocates while a collection runs waits for
  // it). latest is the view's own copy of the last one, made under the lock and copied out under it.
  static final class Frame {
    Scene s;  // the scene the frame was made for: ce, cn are metres from its anchor
    float ce, cn, ch, sh, ik, reach, cutE, cutN, cut = Float.NaN;
    double h, k, base;
    int cRun = -1, cSeg, color;
    boolean free, moving, settled;
    float lift;  // px the map and car are drawn higher, for the lane row under the car (Lanes.AT_CAR)
    double pitch = Double.NaN;  // the camera's tilt (CloseUp.pitch) and its values at it
    final float[] cam = new float[9];

    void copyTo(Frame o) {
      o.s = s;
      o.ce = ce;
      o.cn = cn;
      o.ch = ch;
      o.sh = sh;
      o.ik = ik;
      o.reach = reach;
      o.cutE = cutE;
      o.cutN = cutN;
      o.cut = cut;
      o.h = h;
      o.k = k;
      o.base = base;
      o.cRun = cRun;
      o.cSeg = cSeg;
      o.color = color;
      o.free = free;
      o.moving = moving;
      o.settled = settled;
      o.lift = lift;
      if (o.pitch != pitch) System.arraycopy(cam, 0, o.cam, 0, 9);
      o.pitch = pitch;
    }
  }

  private final Frame latest = new Frame(), tickFrame = new Frame(), drawFrame = new Frame();
  private final double[] frameW = new double[2], smW = new double[2], tickW = new double[2];

  Scene scene() {
    return scene;
  }

  // onDraw's frame, on the UI thread
  Frame frame(Scene s, long now) {
    return frame(s, now, drawFrame);
  }

  synchronized Frame frame(Scene s, long now, Frame f) {
    f.s = s;
    f.cRun = -1;
    f.cSeg = 0;
    f.cut = Float.NaN;
    f.cutE = f.cutN = 0;
    Feed.Pose p = pose();
    double spd = Math.max(0, p.spd);
    f.moving = spd >= HOLD_SPEED;
    double h = smooth(s, p, now, f.moving, frameEN, pose(s, p, now, true, frameEN, frameW));
    float ce = frameEN[0], cn = frameEN[1], ch = (float) Math.cos(h), sh = (float) Math.sin(h);

    double rng = zoomTarget(s, spd, now);
    f.base = rng;
    if (p.at != spdAt) {
      double gap = (p.at - spdAt) / 1000.0;
      if (lastSpd >= 0 && gap > 0 && gap < 1) decel += ((lastSpd - spd) / gap - decel) * Math.min(1, gap / 0.5);
      else decel = 0;
      lastSpd = spd;
      spdAt = p.at;
    }
    double jn = s.route.length > 0 ? junctionAhead(s, ce, cn, ch, sh) : -1;
    if (jn >= 0) {
      double stopAt = decel > STOP_DECEL ? spd * spd / (2 * decel) : -1;
      boolean stopping = (stopAt >= 0 && Math.abs(stopAt - jn) < Math.max(25, 0.4 * jn)) || (spd < CRAWL && jn < STOP_NEAR);
      if (stopping && jn + TURN_LEN < rng) rng = Math.max(jn + TURN_LEN, CLOSE);
    }
    double dt = lastFrame == 0 ? 10 : Math.max(0, now - lastFrame) / 1000.0;  // two threads ask: now may run back
    lastFrame = now;
    k += (rng / R0 - k) * (1 - Math.exp(-dt / (rng / R0 < k ? TAU_IN : TAU_OUT)));
    f.k = k;
    double pitch = CloseUp.pitch(k * R0);  // the tilt glides with the zoom
    if (pitch != f.pitch) CloseUp.camera(pitch, f.cam);
    f.pitch = pitch;
    // the lane row under the car: the map and car glide up LIFT px to make room while it shows, and back down after
    float want = Lanes.at == Lanes.AT_CAR && s.route.length > 0 && lanesShown(now) ? LIFT : 0;
    float l = lift + (want - lift) * (float) (1 - Math.exp(-dt / LIFT_TAU));
    if (Math.abs(want - l) < 0.3f) l = want;
    lift = f.lift = l;
    f.settled = Math.abs(rng / R0 / k - 1) <= ZOOM_EPS && l == want;
    f.ce = ce;
    f.cn = cn;
    f.h = h;
    f.ch = ch;
    f.sh = sh;
    f.ik = (float) (1 / k);
    f.reach = (float) (BOX_R * k);
    // with no route there is no turn text, so the map takes the whole meter: the car at its centre
    f.free = s.route.length == 0;
    f.color = modeColor(Feed.mode);
    if (!f.free) carpetStart(s, f);
    f.copyTo(latest);
    return f;
  }

  // At 30 frames a second the 20 Hz fixes would show as small steps wherever the last fix carried forward disagrees
  // with the next. On a new fix the step is kept as an offset and faded out over SMOOTH_TAU, so the car glides onto
  // it; a step over SNAP_M or SNAP_DEG (a GPS jump) or after a gap is taken at once, and at a standstill the holds in
  // pose() rule.
  private static final double SMOOTH_TAU = 0.06, SNAP_M = 15, SNAP_DEG = 20;
  private static final long SMOOTH_GAP_MS = 500;  // after a longer gap in the fixes the last one says nothing about now
  private final float[] frameEN = new float[2], smEN = new float[2];
  private Feed.Pose smPose;
  private long smAt;
  private double offE, offN, offH;

  private double smooth(Scene s, Feed.Pose p, long now, boolean moving, float[] en, double h) {
    double fade = Math.exp(-(smAt == 0 ? 1 : Math.max(0, now - smAt) / 1000.0) / SMOOTH_TAU);
    smAt = now;
    offE *= fade;
    offN *= fade;
    offH *= fade;
    if (p != smPose) {
      if (moving && smPose != null && p.at - smPose.at < SMOOTH_GAP_MS) {
        double was = pose(s, smPose, now, false, smEN, smW);  // where the last fix put the car now
        offE += smEN[0] - en[0];
        offN += smEN[1] - en[1];
        offH = Math.IEEEremainder(offH + was - h, 2 * Math.PI);
      }
      smPose = p;
    }
    if (!moving || Math.hypot(offE, offN) > SNAP_M || Math.abs(offH) > Math.toRadians(SNAP_DEG)) offE = offN = offH = 0;
    en[0] += (float) offE;
    en[1] += (float) offN;
    return h + offH;
  }

  // ---- scene: the route and the roads near it, rebuilt off the UI thread when the car moves on or data arrives

  private void rebuildIfNeeded(long now) {
    if (building) return;
    Scene s = scene;
    Feed.Route r = Feed.route;
    final double lat = Feed.lat, lon = Feed.lon;
    boolean need = s == null || (r == null ? -1 : r.id) != s.routeId
        || (Feed.tilesVersion != s.tilesVersion && now - builtAt > TILE_REBUILD_MS);
    Feed.Pose p = pose();
    final double spd = Math.max(0, p.spd), h = Math.toRadians(p.hdg), kView = Math.max(k, FreeView.range(spd) / R0);
    if (!need) {
      double[] w = Feed.world(lat, lon, tickW);
      double e = (w[0] - s.ax) * s.mpt, n = -(w[1] - s.ay) * s.mpt;
      need = Math.hypot(e, n) > MOVE_R;
      // with no route, moving: once the view now or LEAD_S s on leaves the roads kept, or service roads should come
      // or go; stopped, the heading is noise and the old 600 m rule holds (FreeView)
      if (!need && s.area != null && spd >= HOLD_SPEED && now - builtAt > TILE_REBUILD_MS) {
        need = !FreeView.covered(s.area, s.bh, e, n, h, spd, kView)
            || FreeView.minor(FreeView.range(spd), s.minor) != s.minor;
      }
      // an overpass drawn flat as the car's road: rebuilt once the car is off that road (CloseUp.overpasses)
      if (!need && r == null && s.close != null && s.close.heldOn != null && spd >= HOLD_SPEED
          && now - builtAt > TILE_REBUILD_MS) {
        need = !CloseUp.onRoad(s.close.heldOn, (float) e, (float) n, (float) Math.sin(h), (float) Math.cos(h));
      }
    }
    if (!need) return;
    building = true;
    builtAt = now;
    final Feed.Route route = r;
    final int version = Feed.tilesVersion;
    final boolean minor = s == null || s.minor;
    Thread t = new Thread(new Runnable() {
      @Override
      public void run() {
        try {
          scene = build(lat, lon, route, version, h, spd, kView, minor);
          Roads g = gpu;
          if (g != null) g.poke();
        } catch (Throwable e) {
          Xlog.log("aaroad: map scene failed: " + e);
        } finally {
          building = false;
        }
      }
    }, "aaroad-scene");
    t.setDaemon(true);
    t.start();
  }

  // h, spd: the car's heading (radians) and speed; kView the zoom the view has or is heading to; minor whether the
  // last scene had service roads (all for FreeView, with no route)
  private static Scene build(double lat, double lon, Feed.Route r, int version, double h, double spd, double kView,
                             boolean minor) {
    Scene s = new Scene();
    double[] a = Feed.world(lat, lon);
    s.ax = a[0];
    s.ay = a[1];
    s.mpt = Feed.metersPerTile(lat);
    s.tilesVersion = version;
    if (r == null) {
      s.area = FreeView.area(spd, kView);
      s.bh = h;
      s.minor = FreeView.minor(FreeView.range(spd), minor);
    }
    List<byte[]> levels = new ArrayList<byte[]>();
    List<float[]> route = new ArrayList<float[]>(), stubs = new ArrayList<float[]>(), roads = new ArrayList<float[]>();
    List<Integer> stubsCls = new ArrayList<Integer>(), roadsCls = new ArrayList<Integer>();
    List<String> stubsName = new ArrayList<String>(), roadsName = new ArrayList<String>();
    HashSet<Long> cells = new HashSet<Long>();
    if (r != null) {
      s.routeId = r.id;
      Run run = new Run();
      java.io.ByteArrayOutputStream lv = new java.io.ByteArrayOutputStream();  // the run's traffic, per segment
      double pe = 0, pn = 0;
      boolean pin = false;
      for (int i = 0; i < r.lat.length; i++) {
        double[] w = Feed.world(r.lat[i], r.lon[i]);
        double e = (w[0] - s.ax) * s.mpt, n = -(w[1] - s.ay) * s.mpt;
        boolean in = Math.hypot(e, n) < SCENE_R;
        if (i > 0 && (in || pin)) {
          if (run.size == 0) run.add(pe, pn);
          run.add(e, n);
          lv.write(r.lvl != null && i - 1 < r.lvl.length ? r.lvl[i - 1] : 0);
          // the route's samples every DENSE m mark the cells whose roads may show as stubs
          int steps = (int) Math.ceil(Math.hypot(e - pe, n - pn) / DENSE);
          for (int j = 0; j <= steps; j++) {
            double f = steps == 0 ? 0 : (double) j / steps;
            cells.add(cell(pe + (e - pe) * f, pn + (n - pn) * f));
          }
        } else if (run.size > 0) {
          route.add(run.take());
          levels.add(lv.toByteArray());
          lv.reset();
        }
        pe = e;
        pn = n;
        pin = in;
      }
      if (run.size > 0) {
        route.add(run.take());
        levels.add(lv.toByteArray());
      }
    }
    // the close-up detail first: it finds which bridges are overpasses (CloseUp.overpasses), and the rest are drawn
    // as plain road here too. One copy of the tile set for both, so they judge the same tiles; and the link roads that
    // are not highway ramps, drawn as streets in both (CloseView.links, Peter 2026-11-05)
    List<Feed.Tile> tiles = new ArrayList<Feed.Tile>(Feed.tiles.values());
    java.util.Map<Feed.Tile, boolean[]> streets = CloseView.links(tiles, s.ax, s.ay, s.mpt);
    java.util.IdentityHashMap<Feed.Tile, byte[][]> raised = new java.util.IdentityHashMap<Feed.Tile, byte[][]>();
    s.close = CloseView.detail(tiles, s.ax, s.ay, s.mpt, route.toArray(new float[0][]), h, raised, streets);
    // the route under a higher deck (Peter 2026-11-08, Part A fix): classified per *sample* along an edge, not
    // once for the whole edge between two route vertices -- a long, straight highway edge can ride its own deck
    // through part of its length and duck under a different, higher one through another part (its own bridge
    // deck would then paint over the route on the bridge it is actually driving on, Under.underCuts' comment).
    // Where Under.underCuts finds a transition, the edge is split there: one extra, collinear point, so the
    // route's shape and routeAlong's distance don't move; the traffic level (levels/routeLevel) is copied onto
    // both halves of the split. route, levels and the new under flags are kept the same length throughout, so
    // s.route / s.routeLevel / s.routeUnder stay one byte or point per segment in step for every later use
    // (chunks, carpet/carpetLine, routeAlong, junctions, Signals.cluster). Read once here against the heights
    // s.close just found, not per frame.
    List<byte[]> unders = new ArrayList<byte[]>();
    for (int ri = 0; ri < route.size(); ri++) {
      float[] p = route.get(ri);
      byte[] lv = levels.get(ri);
      Run split = new Run();
      java.io.ByteArrayOutputStream rlv = new java.io.ByteArrayOutputStream(), ru = new java.io.ByteArrayOutputStream();
      split.add(p[0], p[1]);
      for (int i = 0; i + 3 < p.length; i += 2) {
        float ae = p[i], an = p[i + 1], be = p[i + 2], bn = p[i + 3];
        int level = i / 2 < lv.length ? lv[i / 2] : 0;
        Under.Cuts c = Under.underCuts(s.close, ae, an, be, bn);
        boolean state = c.first;
        float dx = be - ae, dy = bn - an, len = (float) Math.hypot(dx, dy);
        for (float at : c.at) {
          float t = len <= 0 ? 0 : at / len;
          split.add(ae + dx * t, an + dy * t);
          rlv.write(level);
          ru.write(state ? 1 : 0);
          state = !state;
        }
        split.add(be, bn);
        rlv.write(level);
        ru.write(state ? 1 : 0);
      }
      route.set(ri, split.take());
      levels.set(ri, rlv.toByteArray());
      unders.add(ru.toByteArray());
    }
    s.routeUnder = unders.toArray(new byte[0][]);
    for (Feed.Tile tile : tiles) {
      byte[][] up = raised.get(tile);
      for (int c = 0; c < 5; c++) {  // motorway, major, street, link, minor
        float[] pts = tile.pts[c];
        int[] starts = tile.starts[c];
        String[] names = tile.names[c];
        boolean[] bridge = tile.bridge[c];
        byte[] layer = tile.layer[c];
        if (pts == null || starts == null) continue;
        for (int l = 0; l + 1 < starts.length; l++) {
          // with a route, every road around the car shows as in free drive, and farther out the roads along the route
          if (r == null) {
            if (c != 4 || s.minor) free(s, tile, pts, starts[l], starts[l + 1], roads);
          }
          else if (!around(s, tile, pts, starts[l], starts[l + 1], stubs)) near(s, tile, pts, starts[l], starts[l + 1], cells, stubs);
          String name = names != null && l < names.length ? names[l] : null;
          boolean flat = up != null && up[c] != null && l < up[c].length && up[c][l] == 1;  // a bridge not over a road
          int dc = CloseView.drawn(streets, tile, c, l);  // a link that is not a highway ramp: a street
          boolean isBridge = bridge != null && l < bridge.length && bridge[l] && !flat;
          // the OSM layer (CloseUp.LAYER_M stacking order), kept instead of thrown away so a flyover's deck draws
          // over a ramp's below it the same way CloseUp already lifts it: no layer tag (0) is a bridge with nothing
          // under it, CloseUp.layerOf's convention (Math.max(1, ...)); clipped to CloseUp.LAYERS - 1 the same way.
          int rawLayer = layer != null && l < layer.length ? layer[l] : 0;
          int ly = isBridge ? Math.max(1, Math.min(CloseUp.LAYERS - 1, rawLayer)) : 0;
          int sc = isBridge ? dc + BRIDGE * ly : dc;
          while (roadsCls.size() < roads.size()) { roadsCls.add(sc); roadsName.add(name); }
          while (stubsCls.size() < stubs.size()) { stubsCls.add(sc); stubsName.add(name); }
        }
      }
    }
    s.route = route.toArray(new float[0][]);
    s.routeLevel = levels.toArray(new byte[0][]);
    s.stubs = stubs.toArray(new float[0][]);
    s.roads = roads.toArray(new float[0][]);
    // Part B (Peter 2026-11-08): against the finished route and stubs, so stubsDim() matches real route-to-road
    // geometry (Under.carHeight's own ON_DECK-style matching, not a guess); found once here, not per frame.
    s.stubsCls = ints(stubsCls);
    if (r != null) s.stubsDim = stubsDim(s.route, s.routeUnder, s.stubs, s.stubsCls);
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
    int sigCount = 0;
    for (Feed.Tile tile : Feed.tiles.values()) sigCount += tile.signals.length / 3;
    double[] sig = new double[sigCount * 3];
    int k = 0;
    for (Feed.Tile tile : Feed.tiles.values()) {
      float[] t = tile.signals;
      for (int i = 0; i + 2 < t.length && k < sig.length; i += 3, k += 3) {
        sig[k] = tile.x + t[i] / Feed.EXTENT;
        sig[k + 1] = tile.y + t[i + 1] / Feed.EXTENT;
        sig[k + 2] = t[i + 2];
      }
    }
    s.signals = Signals.cluster(sig, k / 3, s.ax, s.ay, s.mpt, s.route, s.routeAlong);
    s.routeChunks = chunks(s.route, null, s.routeLevel, s.routeUnder);
    s.stubsChunks = chunks(s.stubs, s.stubsCls, null, null, s.stubsDim);
    s.roadsChunks = chunks(s.roads, s.roadsCls, null);
    return s;
  }

  // Cuts each run into pieces of at most CHUNK_M m, a long segment into equal parts first; see Chunks. The pieces
  // overlap only in the shared point, and the round caps there join them as the round join did. With lv, the
  // route's traffic per segment of each run, a piece also ends where the traffic changes and its cls is the level.
  private static Chunks chunks(float[][] runs, int[] cls, byte[][] lv) {
    return chunks(runs, cls, lv, null, null);
  }

  // un (route only, Peter 2026-11-08): per segment, 1 where it passes under a higher deck; a change in it cuts a
  // piece the same way a change in lv (traffic level) does, so GlRoads can group route pieces by it too.
  private static Chunks chunks(float[][] runs, int[] cls, byte[][] lv, byte[][] un) {
    return chunks(runs, cls, lv, un, null);
  }

  // dim (stubs only, Peter 2026-11-08, Part B): per segment, 1 where it is the dimmed, untaken branch of a split;
  // cuts a piece the same way un does, so the dimmed and plain halves of a stub are never drawn as one piece.
  private static Chunks chunks(float[][] runs, int[] cls, byte[][] lv, byte[][] un, byte[][] dim) {
    List<float[]> pts = new ArrayList<float[]>();
    List<Integer> pcls = new ArrayList<Integer>(), punder = new ArrayList<Integer>(), pdim = new ArrayList<Integer>();
    List<Float> pat = new ArrayList<Float>();
    Run run = new Run();
    double along = 0, start;
    for (int r = 0; r < runs.length; r++) {
      float[] p = runs[r];
      if (p.length < 4) continue;
      run.add(p[0], p[1]);
      start = along;
      double len = 0;
      byte[] rl = lv != null && r < lv.length ? lv[r] : null;
      byte[] ru = un != null && r < un.length ? un[r] : null;
      byte[] rd = dim != null && r < dim.length ? dim[r] : null;
      int cur = 0, curU = 0, curD = 0;  // the level, under-deck flag and dim flag of the piece being built
      for (int i = 2; i + 1 < p.length; i += 2) {
        double pe = p[i - 2], pn = p[i - 1], de = p[i] - pe, dn = p[i + 1] - pn, d = Math.hypot(de, dn);
        int parts = Math.max(1, (int) Math.ceil(d / CHUNK_M));
        int level = rl != null && i / 2 - 1 < rl.length ? rl[i / 2 - 1] : 0;
        int u = ru != null && i / 2 - 1 < ru.length ? ru[i / 2 - 1] : 0;
        int dm = rd != null && i / 2 - 1 < rd.length ? rd[i / 2 - 1] : 0;
        for (int j = 1; j <= parts; j++) {
          if (len > 0 && (len + d / parts > CHUNK_M || level != cur || u != curU || dm != curD)) {
            float[] c = run.take();
            pts.add(c);
            pcls.add(cls == null ? cur : cls[r]);
            punder.add(curU);
            pdim.add(curD);
            pat.add((float) start);
            start = along;
            run.add(c[c.length - 2], c[c.length - 1]);
            len = 0;
          }
          if (j == parts) run.add(p[i], p[i + 1]);
          else run.add(pe + de * j / parts, pn + dn * j / parts);
          cur = level;
          curU = u;
          curD = dm;
          len += d / parts;
          along += d / parts;
        }
      }
      if (run.size >= 4) {
        pts.add(run.take());
        pcls.add(cls == null ? cur : cls[r]);
        punder.add(curU);
        pdim.add(curD);
        pat.add((float) start);
      }
      run.size = 0;
    }
    Chunks out = new Chunks();
    out.pts = pts.toArray(new float[0][]);
    out.box = boxes(out.pts);
    out.cls = ints(pcls);
    out.under = ints(punder);
    out.dim = ints(pdim);
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
  private static Path[] paths(int n) {
    Path[] p = new Path[n];
    for (int i = 0; i < n; i++) p[i] = new Path();
    return p;
  }

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

  // Part B (Peter 2026-11-08): 1 on a stubs segment that is the first DIM_M of a branch leaving the route at a
  // split not taken (FORK_MERGE_M, FORK_MIN_DEG..FORK_MAX_DEG above); 0 elsewhere. Checked at both ends of each
  // run, since either end may be the one that meets the route (near() and around() keep a stub's points in the
  // tile's own direction, not the route's direction of travel).
  // Only motorway (0) and link (3) runs, the ramp and mainline a split is made of: a street meeting the route at
  // an angle in that window is a junction, not a split, and stays at full colour. A bridge run ending near the
  // route where the route runs under a deck (under, Scene.routeUnder) is the overpass crossing above it, not a
  // fork, and is skipped too (seen on the St Mary's crossing harness case, 2026-11-08).
  private static byte[][] stubsDim(float[][] route, byte[][] under, float[][] stubs, int[] cls) {
    byte[][] out = new byte[stubs.length][];
    for (int si = 0; si < stubs.length; si++) {
      float[] p = stubs[si];
      out[si] = new byte[Math.max(0, p.length / 2 - 1)];
      int tc = si < cls.length ? cls[si] % BRIDGE : -1;
      if (p.length < 4 || (tc != 0 && tc != 3)) continue;
      for (int end = 0; end < 2; end++) {
        int ei = end == 0 ? 0 : p.length - 2, di = end == 0 ? 2 : p.length - 4;
        double[] m = nearestOnRoute(route, p[ei], p[ei + 1]);
        if (m == null) continue;
        int mr = (int) m[3], ms = (int) m[4];
        if (cls[si] >= BRIDGE && mr < under.length && ms < under[mr].length && under[mr][ms] != 0) continue;
        float ox = p[di] - p[ei], on = p[di + 1] - p[ei + 1];  // the branch's own heading, away from the match
        double rl = Math.hypot(m[1], m[2]), ol = Math.hypot(ox, on);
        if (rl < 1e-3 || ol < 1e-3) continue;
        double deg = Math.toDegrees(Math.acos(Math.max(-1, Math.min(1, (m[1] * ox + m[2] * on) / (rl * ol)))));
        deg = Math.min(deg, 180 - deg);  // the route's own direction here is arbitrary (forward or back): the acute angle either way
        if (deg < FORK_MIN_DEG || deg > FORK_MAX_DEG) continue;  // too parallel: its own road; too square: a crossing road, leave it
        double along = 0;
        if (end == 0) {
          for (int i = 0; i + 3 < p.length && along < DIM_M; i += 2) {
            out[si][i / 2] = 1;
            along += Math.hypot(p[i + 2] - p[i], p[i + 3] - p[i + 1]);
          }
        } else {
          for (int i = p.length - 2; i - 1 >= 2 && along < DIM_M; i -= 2) {
            out[si][i / 2 - 1] = 1;
            along += Math.hypot(p[i] - p[i - 2], p[i + 1] - p[i - 1]);
          }
        }
      }
    }
    return out;
  }

  // The closest point on any route run to (e, n), as its distance, the (unnormalised) direction of the route
  // segment it falls on, and that segment's run and index, or null when none is within FORK_MERGE_M. Same projection as routeHeading, kept separate
  // since that one answers a different question (the car's own heading) at a different, tighter range (ON_ROUTE).
  private static double[] nearestOnRoute(float[][] route, float e, float n) {
    double best = FORK_MERGE_M * FORK_MERGE_M;
    double[] out = null;
    for (int r = 0; r < route.length; r++) {
      float[] p = route[r];
      for (int i = 0; i + 3 < p.length; i += 2) {
        double dx = p[i + 2] - p[i], dy = p[i + 3] - p[i + 1], l2 = dx * dx + dy * dy;
        if (l2 < 1) continue;
        double t = Math.max(0, Math.min(1, ((e - p[i]) * dx + (n - p[i + 1]) * dy) / l2));
        double px = p[i] + t * dx - e, py = p[i + 1] + t * dy - n, d2 = px * px + py * py;
        if (d2 < best) { best = d2; out = new double[] {Math.sqrt(d2), dx, dy, r, i / 2}; }
      }
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

  // A road for the free-drive scene: kept when it reaches into the rectangle ahead (FreeView) or within ROADS_R of
  // the anchor, its points thinned to FreeView.SIMPLIFY m
  private static void free(Scene s, Feed.Tile tile, float[] pts, int from, int to, List<float[]> out) {
    Run run = new Run();
    for (int i = from; i < to; i++) {
      double e = ((tile.x + pts[2 * i] / Feed.EXTENT) - s.ax) * s.mpt;
      double n = -((tile.y + pts[2 * i + 1] / Feed.EXTENT) - s.ay) * s.mpt;
      run.add(e, n);
    }
    if (run.size < 4) return;
    float[] b = boxes(new float[][] {run.peek()})[0];
    boolean near = !(b[2] < -ROADS_R || b[0] > ROADS_R || b[3] < -ROADS_R || b[1] > ROADS_R);
    if (!near && !FreeView.touches(b, s.area, Math.cos(s.bh), Math.sin(s.bh))) return;
    out.add(FreeView.simplify(run.p, run.size / 2, FreeView.SIMPLIFY));
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
  boolean project(float x, float z, float off, float[] out) {
    return project(cam, x, z, off, out);
  }

  // the same through camera values c (the GPU layer's frame's, Frame.cam)
  boolean project(float[] c, float x, float z, float off, float[] out) {
    float[] cam = c;
    if (x < BOX_X0 || x > BOX_X1 || z < BOX_Z0 || z > BOX_Z1) return false;
    float w = cam[6] * x + cam[7] * z + cam[8];
    if (w <= 0.01f) return false;
    out[0] = (cam[0] * x + cam[1] * z + cam[2]) / w + off;
    out[1] = (cam[3] * x + cam[4] * z + cam[5]) / w - lift;
    return true;
  }

  // Whether a label box centred on (x, y) would cover a card, the car, the speed sign, the street tag or the edges.
  // Off the map: left of the cards (or of the tach needle without a route), and the fog at the top.
  boolean blocked(float x, float y, float w, boolean free, float[] tagBox) {
    float l = x - w / 2 - 3, r = x + w / 2 + 3, t = y - 9, b = y + 7;
    if (l < (free ? CARD_L + 6 : CARD_R + 10) || r > 578 || t < LABEL_TOP || b > 212) return true;
    float carX = free ? FREE_X : CAR_X;
    float up = lift;
    if (hits(l, t, r, b, carX - 30, CAR_Y - 32 - up, carX + 30, 215 - up)) return true;
    if (up > 1 && hits(l, t, r, b, CAR_X - LANE_HALF - 6, LANE_CAR_T - 6, CAR_X + LANE_HALF + 6, 215)) return true;  // the lane row
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
          float cost = (float) Math.hypot(de, dn) + (cls[r] % BRIDGE == 4 ? MINOR_COST : 0);
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
      String ref = Shield.ref(name);  // "I 635" from the tiles is written "I-635" in a white badge
      String text = ref != null ? ref : TextUtils.ellipsize(name, label, LABEL_MAX_W, TextUtils.TruncateAt.END).toString();
      float w = ref != null ? badgeW(ref, LABEL_BADGE, labelBadge[2]) + 2 : label.measureText(text);
      if (blocked(c[4], c[5], w, free, tagBox)) continue;
      boolean clear = true;
      for (int i = 0; i < count && clear; i++) {
        clear = !hits(c[4] - w / 2 - LABEL_GAP, c[5] - 9 - LABEL_GAP, c[4] + w / 2 + LABEL_GAP, c[5] + 7 + LABEL_GAP,
                      labelX[i] - labelW[i] / 2, labelY[i] - 9, labelX[i] + labelW[i] / 2, labelY[i] + 7);
      }
      if (!clear || crossed(runs, boxes, (int) c[1], c[2], c[3], c[4], w, ce, cn, ch, sh, ik, off, q)) continue;
      done.add(name);
      labelText[count] = text;
      labelRef[count] = ref != null;
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

  // The traffic signal and stop icons this frame (Signals.choose), each where the map puts its intersection and left
  // out while it would cover a card, the car, the sign, the tag or an edge, as a label is: into xy (x, y pairs, whole
  // pixels, the icon's centre) and sprite (SignalIcon.sprite), farthest first. Returns how many. The view and the GPU
  // layer both place them with this, each with its own c, q (scratch), xy and sprite.
  int placeSignals(Scene s, float ce, float cn, float ch, float sh, float ik, boolean free, float[] tag,
                   Signals.Choice c, float[] q, float[] xy, int[] sprite) {
    Signals.choose(s.signals, ce, cn, ch, sh, c);
    float off = free ? FREE_X - CAR_X : 0;
    int n = 0;
    for (int i = 0; i < c.count; i++) {
      int g = 4 * c.idx[i];
      float de = s.signals[g] - ce, dn = s.signals[g + 1] - cn;
      if (!project((de * ch - dn * sh) * ik, (de * sh + dn * ch) * ik, off, q)) continue;
      // the icon's box as a label's, a little taller: blocked() takes a label's centre, 1 px above its box's middle
      if (blocked(q[0], q[1] + 1, 12, free, tag)) continue;
      xy[2 * n] = Math.round(q[0]);
      xy[2 * n + 1] = Math.round(q[1]);
      sprite[n] = SignalIcon.sprite((int) s.signals[g + 2], c.big(c.idx[i]));
      n++;
    }
    return n;
  }
  private final float[] sigQ = new float[2];
  private final Signals.Choice sigChoice = new Signals.Choice();
  private final float[] sigXY = new float[2 * Signals.MAX];
  private final int[] sigSprite = new int[Signals.MAX];
  private int sigCount;
  private final Paint sigPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
  private final Path sigPath = new Path();
  private final RectF sigRect = new RectF(0, 0, 0, 0);

  // The chosen labels follow the map between choices; one that has moved onto something is left out this frame.
  private void moveLabels(float ce, float cn, float ch, float sh, float ik, boolean free) {
    float off = free ? FREE_X - CAR_X : 0;
    float[] q = new float[2];
    for (int i = 0; i < labelCount; i++) {
      float de = labelE[i] - ce, dn = labelN[i] - cn;
      labelOn[i] = project((de * ch - dn * sh) * ik, (de * sh + dn * ch) * ik, off, q) && !blocked(q[0], q[1], labelW[i], free, tagBox);
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
    if (onGpu) {
      // the slow layer: the car, tag, sign and cards over the GPU layer's map (slowTick)
      boolean free = s.route.length == 0;
      drawnScene = s;
      drawnAt = now;
      drawnRoad = !TextUtils.isEmpty(Feed.road) ? Feed.road : onRoad;
      canvas.save();
      float up = lift;
      drawnLift = up;
      canvas.translate(free ? FREE_X - CAR_X : 0f, -up);
      Frame uf = null;
      synchronized (this) {
        if (latest.s == s) {
          latest.copyTo(slowFrame);
          uf = slowFrame;
        }
      }
      drawCar(canvas, uf);
      canvas.restore();
      overlay(canvas, now, free, true);
      drawn(canvas, now, now, now, now);
      return;
    }

    Frame f = frame(s, now);
    useCamera(f);
    float ce = f.ce, cn = f.cn, ch = f.ch, sh = f.sh, ik = f.ik, reach = f.reach;
    double h = f.h;
    long t0 = SystemClock.uptimeMillis();
    drawnBase = f.base;
    zoomSettled = f.settled;
    drawnScene = s;
    drawnAt = now;
    drawnE = ce;
    drawnN = cn;
    drawnH = h;
    drawnK = f.k;
    drawnMoving = f.moving;

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
    boolean free = f.free;
    if (!onGpu) {
      routePath.rewind();
      routeWorld.rewind();
      for (Path p : carpetPaths) p.rewind();
      for (Path p : carpetUnderPaths) p.rewind();
      for (Path p : stubPaths) p.rewind();
      for (Path p : stubWorld) p.rewind();
      gridPath.rewind();
      gridLines(s, ce, cn, ch, sh, ik);
      pieces(s.routeChunks, routeWorlds, routeFrames, ce, cn, ch, sh, ik);
      if (s.route.length > 0) pieces(s.stubsChunks, stubWorld, stubPaths, ce, cn, ch, sh, ik);
      if (s.route.length > 0) carpet(s, f, ce, cn, ch, sh, ik, reach);
      else pieces(s.roadsChunks, stubWorld, stubPaths, ce, cn, ch, sh, ik);
    }

    currentRoad(s, ce, cn, h, now);
    String shown = !TextUtils.isEmpty(Feed.road) ? Feed.road : onRoad;
    placeLabels(s, ce, cn, ch, sh, ik, reach, free, shown, now);
    if (labelAt == now && gpu != null) {
      labels = new Labels(s, labelText, labelE, labelN, labelW, labelCount);
      gpu.poke();
    }
    if (!onGpu) moveLabels(ce, cn, ch, sh, ik, free);  // else the GPU layer moves and draws them
    sigCount = onGpu ? 0 : placeSignals(s, ce, cn, ch, sh, ik, free, tagBox, sigChoice, sigQ, sigXY, sigSprite);  // likewise
    long t1 = SystemClock.uptimeMillis();
    canvas.save();
    drawnLift = f.lift;
    canvas.translate(free ? FREE_X - CAR_X : 0f, -f.lift);
    if (!onGpu) {
      canvas.save();
      canvas.concat(camera);
      canvas.drawPath(gridPath, grid);
      // zoomed in: the rail lines, under the roads (CloseView.rails)
      closeView.rails(canvas, s.close, ce, cn, ch, sh, ik, CloseUp.gate(f.k * R0, CloseUp.ROAD_SHOW, CloseUp.ROAD_HIDE));
      // a road W m wide is W / k display metres; never thinner than MIN_ROAD_PX so a far road still shows
      float floor = dm(MIN_ROAD_PX);
      // every kerb first and then every surface, so crossing roads open into each other instead of being cut by a kerb
      for (int c : DRAW_ORDER) {
        sideKerb.setColor(KERBS[c]);
        stroke(canvas, stubWorld[c], stubPaths[c], sideKerb, Math.max(floor, ROAD_M[c] * ik) + dm(2 * KERB_PX));
      }
      if (s.route.length > 0) stroke(canvas, routeWorld, routePath, kerb, Math.max(floor, ROUTE_M * ik) + dm(2 * KERB_PX));
      if (s.route.length > 0) stroke(canvas, routeWorld, routePath, road, Math.max(floor, ROUTE_M * ik));
      for (int c : DRAW_ORDER) {
        side.setColor(ROADS[c]);
        stroke(canvas, stubWorld[c], stubPaths[c], side, Math.max(floor, ROAD_M[c] * ik));
      }
      // Part B (Peter 2026-11-08): the untaken branch of a split, at ground level, drawn dimmed over the plain
      // ground pass above -- same kerb-then-surface order, same widths, just the DIM_OFF half of the classes and
      // dim()'s colours instead of KERBS/ROADS.
      for (int c : DRAW_ORDER) {
        sideKerb.setColor(dim(KERBS[c]));
        stroke(canvas, stubWorld[c + DIM_OFF], stubPaths[c + DIM_OFF], sideKerb, Math.max(floor, ROAD_M[c] * ik) + dm(2 * KERB_PX));
      }
      for (int c : DRAW_ORDER) {
        side.setColor(dim(ROADS[c]));
        stroke(canvas, stubWorld[c + DIM_OFF], stubPaths[c + DIM_OFF], side, Math.max(floor, ROAD_M[c] * ik));
      }
      // the carpet where the route passes under a higher deck (Scene.routeUnder, Peter 2026-11-08): drawn here,
      // before the bridges below, so their casing/kerb/surface paint over it and the line reads as diving under
      // the deck it doesn't use -- the rest of the carpet (carpetPaths) still draws last, on top, below.
      if (s.route.length > 0) {
        carpet.setStrokeWidth(Math.max(dm(LINE_MIN_PX), Math.min(dm(LINE_MAX_PX), LINE_M * ik)));
        for (int l = 0; l < carpetUnderPaths.length; l++) {
          if (carpetUnderPaths[l].isEmpty()) continue;
          carpet.setColor(l == 0 ? modeColor(Feed.mode) : TRAFFIC[l - 1]);
          carpet.setAlpha(242);
          canvas.drawPath(carpetUnderPaths[l], carpet);
        }
      }
      // the bridges over all of that: casing, kerb, surface, lowest layer first. Only overpasses are here
      // (CloseUp.overpasses); zoomed in they fade out as CloseView draws them raised, with deck, piers and shadow,
      // so the flat copy doesn't show under the deck as a second road.
      //
      // Peter 2026-11-08 ("the highway lines simply overlay the street below, with no joint seams"): each layer's
      // casing, then kerb, then fill are drawn fully before the next layer up starts, so a layer-2 flyover's fill
      // always paints over a layer-1 ramp's deck where they cross, and a ramp joining the mainline at the same
      // layer has its casing hidden by whichever piece's fill is drawn last there -- no round-cap seam crossing the
      // joint, as Google draws it. Tried first: only merging touching pieces of the same class (same width), which
      // had no visible effect since the seam is between classes (motorway vs motorway_link), not within one; see
      // /tmp/hu/audit/merge-same-class.patch.
      float flatA = s.close != null ? 1 - CloseUp.gate(f.k * R0, CloseUp.ROAD_SHOW, CloseUp.ROAD_HIDE) : 1;
      if (flatA > 0) {
        int alpha = Math.round(255 * flatA);
        for (int ly = 1; ly < CloseUp.LAYERS; ly++) {
          sideKerb.setColor(CASING_C);
          sideKerb.setAlpha(alpha);
          for (int c : DRAW_ORDER) {
            stroke(canvas, stubWorld[c + BRIDGE * ly], stubPaths[c + BRIDGE * ly], sideKerb,
                   Math.max(floor, ROAD_M[c] * ik) + dm(2 * KERB_PX + 2 * CASING_PX));
          }
          for (int c : DRAW_ORDER) {
            sideKerb.setColor(KERBS[c]);
            sideKerb.setAlpha(alpha);
            stroke(canvas, stubWorld[c + BRIDGE * ly], stubPaths[c + BRIDGE * ly], sideKerb,
                   Math.max(floor, ROAD_M[c] * ik) + dm(2 * KERB_PX));
          }
          for (int c : DRAW_ORDER) {
            side.setColor(ROADS[c]);
            side.setAlpha(alpha);
            stroke(canvas, stubWorld[c + BRIDGE * ly], stubPaths[c + BRIDGE * ly], side, Math.max(floor, ROAD_M[c] * ik));
          }
          // Part B (Peter 2026-11-08): the untaken branch on this deck layer, same dim() treatment as the ground
          // pass above, same casing-then-kerb-then-surface order and the layer's own fade alpha.
          for (int c : DRAW_ORDER) {
            sideKerb.setColor(CASING_C);
            sideKerb.setAlpha(alpha);
            stroke(canvas, stubWorld[c + BRIDGE * ly + DIM_OFF], stubPaths[c + BRIDGE * ly + DIM_OFF], sideKerb,
                   Math.max(floor, ROAD_M[c] * ik) + dm(2 * KERB_PX + 2 * CASING_PX));
          }
          for (int c : DRAW_ORDER) {
            sideKerb.setColor(dim(KERBS[c]));
            sideKerb.setAlpha(alpha);
            stroke(canvas, stubWorld[c + BRIDGE * ly + DIM_OFF], stubPaths[c + BRIDGE * ly + DIM_OFF], sideKerb,
                   Math.max(floor, ROAD_M[c] * ik) + dm(2 * KERB_PX));
          }
          for (int c : DRAW_ORDER) {
            side.setColor(dim(ROADS[c]));
            side.setAlpha(alpha);
            stroke(canvas, stubWorld[c + BRIDGE * ly + DIM_OFF], stubPaths[c + BRIDGE * ly + DIM_OFF], side, Math.max(floor, ROAD_M[c] * ik));
          }
        }
        sideKerb.setAlpha(255);
        side.setAlpha(255);
      }
      // zoomed in: the raised bridges, lane lines and sidewalks (CloseView), under the carpet
      closeView.draw(canvas, s.close, ce, cn, ch, sh, ik, CloseUp.gate(f.k * R0, CloseUp.ROAD_SHOW, CloseUp.ROAD_HIDE),
                     CloseUp.gate(f.k * R0, CloseUp.WALK_SHOW, CloseUp.WALK_HIDE));
      if (s.route.length > 0) {
        carpet.setStrokeWidth(Math.max(dm(LINE_MIN_PX), Math.min(dm(LINE_MAX_PX), LINE_M * ik)));
        for (int l = 0; l < carpetPaths.length; l++) {
          if (carpetPaths[l].isEmpty()) continue;
          carpet.setColor(l == 0 ? modeColor(Feed.mode) : TRAFFIC[l - 1]);
          carpet.setAlpha(242);
          canvas.drawPath(carpetPaths[l], carpet);
        }
      }
      canvas.restore();
    }

    drawCar(canvas, f);
    canvas.restore();
    long t2 = SystemClock.uptimeMillis();
    overlay(canvas, now, free, onGpu);
    drawn(canvas, now, t0, t1, t2);
  }

  private void drawCar(Canvas canvas, Frame f) {
    if (f != null) useCamera(f);
    long now = SystemClock.uptimeMillis();
    float g = glow(now);
    drawnGlow = g;
    float u = underDeck(f, now);
    if (u <= 0) {
      body(canvas, g, 255);
      return;
    }
    // under a deck: the car outside the deck as ever; under it, the car fades out and its outline in
    canvas.save();
    canvas.clipPath(deckClip, Region.Op.DIFFERENCE);
    body(canvas, g, 255);
    canvas.restore();
    canvas.save();
    canvas.clipPath(deckClip);
    int a = Math.round(255 * (1 - u));
    if (a > 0) body(canvas, g * (1 - u), a);
    if (outline != null) {
      outlinePaint.setAlpha(Math.round(255 * u));
      canvas.drawBitmap(outline, CAR_X - CAR_PX, CAR_Y - CAR_PX, outlinePaint);
    } else {
      fill.setColor(OUTLINE_C);
      fill.setAlpha(Math.round(255 * u));
      fill.setStyle(Paint.Style.STROKE);
      fill.setStrokeWidth(2);
      canvas.drawPath(arrow(), fill);
      fill.setStyle(Paint.Style.FILL);
    }
    canvas.restore();
  }

  // The car at alpha a (0 to 255) with its brake glow g.
  private void body(Canvas canvas, float g, int a) {
    if (g > 0) lights(canvas, g, true);
    if (car != null) {
      carPaint.setAlpha(a);
      canvas.drawBitmap(car, CAR_X - CAR_PX, CAR_Y - CAR_PX, a == 255 ? null : carPaint);
      if (g > 0) lights(canvas, g, false);
    } else {
      fill.setColor(Color.WHITE);
      fill.setAlpha(a);
      canvas.drawPath(arrow(), fill);
    }
  }

  private static Path arrow() {
    Path p = new Path();
    p.moveTo(CAR_X, CAR_Y - 14);
    p.lineTo(CAR_X + 10, CAR_Y + 10);
    p.lineTo(CAR_X, CAR_Y + 5);
    p.lineTo(CAR_X - 10, CAR_Y + 10);
    p.close();
    return p;
  }

  // How much the car shows as an outline under a deck this frame, 0 to 1 (just the ease now; every zoom finds a
  // deck over the car, not only the close-up), with deckClip the decks over it on screen. 0 with no frame or no
  // deck over the car.
  private float underDeck(Frame f, long now) {
    if (!UNDER_OUTLINE) {
      drawnUnder = 0;
      return 0;
    }
    CloseUp.Detail d = f == null || f.s == null ? null : f.s.close;
    // the close-up gate, passed to Under.covering as hs: in the close-up the clip follows the lifted decks as drawn
    // there; at 0 (cruising) it follows the flat decks cruising draws instead (Under.covering handles hs 0).
    float gr = f == null ? 0 : CloseUp.gate(f.k * R0, CloseUp.ROAD_SHOW, CloseUp.ROAD_HIDE);
    if (d != coverOf) {
      coverOf = d;
      coverN = 0;
    }
    int n = f == null ? 0 : Under.covering(d, f.ce, f.cn, f.ch, f.sh, f.ik, gr, ROAD_M, dm(MIN_ROAD_PX),
                                           dm(2 * KERB_PX + 2 * CASING_PX), dm(Under.CAR_PX), cover);
    if (n > 0) n = aboveCarLayer(d, f, cover, n);
    under.set(n > 0, now, Under.fadeMs(Math.max(0, Feed.spd)));
    if (n > 0) coverN = n;  // else the last ones, while the outline fades out
    float u = under.level(now);
    drawnUnder = u;
    if (f != null) {
      drawnUE = f.ce;
      drawnUN = f.cn;
      drawnUH = (float) f.h;
    }
    if (u <= 0 || coverN == 0) return 0;
    deckShape(d, f, gr);
    return deckClip.isEmpty() ? 0 : u;
  }

  // The decks over the car into deckClip, on screen: each one's part within Under.NEAR_DM of the car, as wide as its
  // casing, lifted as drawn, with a round-ish end so pieces meet without a notch. Each piece is wound the same way, so
  // where two overlap the clip holds both.
  private void deckShape(CloseUp.Detail d, Frame f, float gr) {
    deckClip.rewind();
    float[] v = d.seg;
    float casing = dm(2 * KERB_PX + 2 * CASING_PX), floor = dm(MIN_ROAD_PX);
    for (int k = 0; k < coverN; k++) {
      int i = cover[k], o = i * CloseUp.SEG;
      int c = CloseUp.cls(closeRange(d, i));
      float half = (Math.max(floor, ROAD_M[c] * f.ik) + casing) / 2;
      float[] q = Under.lifted(v, o, f.ce, f.cn, f.ch, f.sh, f.ik, gr, coverQ);
      float dx = q[2] - q[0], dz = q[3] - q[1], len = (float) Math.hypot(dx, dz);
      if (len <= 0) continue;
      float t = Math.max(0, Math.min(1, -(q[0] * dx + q[1] * dz) / (len * len)));
      float t0 = Math.max(0, t - Under.NEAR_DM / len), t1 = Math.min(1, t + Under.NEAR_DM / len);
      float ha = half * CloseUp.lift((v[o + 4] + t0 * (v[o + 5] - v[o + 4])) * gr * f.ik);
      float hb = half * CloseUp.lift((v[o + 4] + t1 * (v[o + 5] - v[o + 4])) * gr * f.ik);
      float ax = q[0] + t0 * dx, az = q[1] + t0 * dz, bx = q[0] + t1 * dx, bz = q[1] + t1 * dz;
      float nx = -dz / len, nz = dx / len;
      polygon(new float[] {ax + nx * ha, az + nz * ha, bx + nx * hb, bz + nz * hb, bx - nx * hb, bz - nz * hb,
                           ax - nx * ha, az - nz * ha});
      if (t0 == 0) polygon(disc(ax, az, ha));
      if (t1 == 1) polygon(disc(bx, bz, hb));
    }
  }

  // which range segment i is in
  private static int closeRange(CloseUp.Detail d, int i) {
    int r = 0;
    while (r + 1 < CloseUp.RANGES && d.from[r + 1] <= i) r++;
    return r;
  }

  // Keeps only out[0..n) whose own deck layer is strictly above the layer of the road the car is on (Peter
  // 2026-11-08: Under.covering's height margin (OVER_M) alone can still call the car's own bridge "covering" it
  // right at a layer boundary -- this is why UNDER_OUTLINE was once turned off, d852f9f41). Under.carHeight and
  // CloseUp.layerOf are both package-visible helpers already used elsewhere; nothing here is per-frame allocation,
  // just a compaction of the existing out array. Returns the new count.
  private static int aboveCarLayer(CloseUp.Detail d, Frame f, int[] out, int n) {
    float hc = Under.carHeight(d, f.ce, f.cn, f.ch, f.sh, ROAD_M);
    int carLayer = Math.max(0, Math.min(CloseUp.LAYERS - 1, Math.round(hc / CloseUp.LAYER_M)));
    int kept = 0;
    for (int k = 0; k < n; k++) {
      if (CloseUp.layerOf(closeRange(d, out[k])) > carLayer) out[kept++] = out[k];
    }
    return kept;
  }

  private static float[] disc(float x, float z, float r) {
    float[] p = new float[16];
    for (int j = 0; j < 8; j++) {
      double a = j * Math.PI / 4;
      p[2 * j] = x + r * (float) Math.cos(a);
      p[2 * j + 1] = z + r * (float) Math.sin(a);
    }
    return p;
  }

  // Car-frame points (x, z pairs) to the screen through the camera, into deckClip, wound clockwise on screen.
  private void polygon(float[] p) {
    int n = p.length / 2;
    float area = 0;
    for (int j = 0; j < n; j++) {
      float x = p[2 * j], z = p[2 * j + 1], w = cam[6] * x + cam[7] * z + cam[8];
      if (w <= 0) return;
      p[2 * j] = (cam[0] * x + cam[1] * z + cam[2]) / w;
      p[2 * j + 1] = (cam[3] * x + cam[4] * z + cam[5]) / w;
    }
    for (int j = 0; j < n; j++) {
      int k = (j + 1) % n;
      area += p[2 * j] * p[2 * k + 1] - p[2 * k] * p[2 * j + 1];
    }
    for (int j = 0; j < n; j++) {
      int m = area >= 0 ? j : n - 1 - j;
      if (j == 0) deckClip.moveTo(p[2 * m], p[2 * m + 1]);
      else deckClip.lineTo(p[2 * m], p[2 * m + 1]);
    }
    deckClip.close();
  }

  // Whether frame f puts the car under a deck (the slow layer's check between its draws), at every zoom. Filtered
  // by layer the same way underDeck is, so this does not ask for a redraw over a case underDeck would then decide
  // is really the car's own bridge.
  private boolean underWanted(Frame f) {
    if (!UNDER_OUTLINE) return false;
    float gr = CloseUp.gate(f.k * R0, CloseUp.ROAD_SHOW, CloseUp.ROAD_HIDE);
    int n = Under.covering(f.s.close, f.ce, f.cn, f.ch, f.sh, f.ik, gr, ROAD_M, dm(MIN_ROAD_PX),
                           dm(2 * KERB_PX + 2 * CASING_PX), dm(Under.CAR_PX), tickCover);
    return n > 0 && aboveCarLayer(f.s.close, f, tickCover, n) > 0;
  }

  // Under a deck the outline is still moving (easing, or the deck sliding by the car): the slow layer redraws.
  private boolean underMoving(long now) {
    if (under.moving(now)) return true;
    if (drawnUnder <= 0) return false;
    Frame f = tickFrame;
    return f.s != null && (f.ce != drawnUE || f.cn != drawnUN || (float) f.h != drawnUH);
  }

  // The brake glow, 0 to 1: it notices Feed.brake changing and moves from where it was toward the new end at 1/GLOW_MS
  // a millisecond, so a tap shorter than the fade turns back from part way. Off while the pose is stale.
  private float glow(long now) {
    boolean on = Feed.brake && now - Feed.poseAt < STALE_MS;
    if (on != glowOn) {
      glowFrom = glowLevel(now);
      glowOn = on;
      glowAt = now;
    }
    return glowLevel(now);
  }

  private float glowLevel(long now) {
    float d = (now - glowAt) / (float) GLOW_MS;
    return glowOn ? Math.min(1f, glowFrom + d) : Math.max(0f, glowFrom - d);
  }

  private boolean glowing(long now) {
    float g = glow(now);
    return g != (glowOn ? 1f : 0f) || g != drawnGlow;
  }

  // Under the sprite, a wide red halo that shows past the body's sides; over it, a bright core on each lamp.
  private void lights(Canvas canvas, float g, boolean under) {
    for (int side = -1; side <= 1; side += 2) {
      float x = CAR_X + side * LIGHT_DX, y = CAR_Y + LIGHT_DY;
      if (under) {
        glowPaint.setColor(0xffff1a1a);
        glowPaint.setAlpha(Math.round(60 * g));
        canvas.drawCircle(x, y, 13, glowPaint);
        glowPaint.setAlpha(Math.round(90 * g));
        canvas.drawCircle(x, y, 9, glowPaint);
      } else {
        glowPaint.setColor(0xffff2a20);
        glowPaint.setAlpha(Math.round(150 * g));
        canvas.drawCircle(x, y, 6, glowPaint);
        glowPaint.setColor(0xffff9a80);
        glowPaint.setAlpha(Math.round(200 * g));
        canvas.drawCircle(x, y, 2.5f, glowPaint);
      }
    }
  }

  // A frame's timing into the log's sums, and every 900 ticks the log line.
  private void drawn(Canvas canvas, long now, long t0, long t1, long t2) {
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
      String redraws = logFrom > 0 && now > logFrom
          ? String.format(Locale.US, ", view %.1f redraws/s", (drawCount - drawsAtLog) * 1000.0 / (now - logFrom)) : "";
      drawsAtLog = drawCount;
      rollsAtLog = rollDraws;
      logFrom = now;
      say("aaroad: map draw " + drawSum / drawCount + " ms average over " + drawCount + " frames ("
          + setupSum / drawCount + " setup, " + pathSum / drawCount + " shapes, " + strokeSum / drawCount + " roads, "
          + cardSum / drawCount + " cards), " + (checks > 0 ? 100L * skipped / checks : 0) + "% frames skipped"
          + redraws + String.format(Locale.US, ", %d song strips", rollDraws - rollsAtLog) + (gpu != null ? ", " + gpu.takeStats() : "")
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
      // the route's surface is one path whatever the traffic; a stub piece goes to its own dimmed half (Part B,
      // Scene.stubsDim through Chunks.dim) when it is the untaken branch of a split, same offset GlRoads' route
      // groups use for the under-a-deck half (DIM_OFF mirrors that doubling, just on the stub/road class space)
      int c = kept.length == 1 ? 0 : pc.cls[i] + (i < pc.dim.length && pc.dim[i] != 0 ? DIM_OFF : 0);
      if (z0 >= NEAR_Z) kept[c].addPath(pc.path[i]);
      else line(pc.pts[i], 0, near[c], ce, cn, ch, sh, ik, Float.NaN, 0);
    }
  }

  // Part B (Peter 2026-11-08): a road colour blended halfway to the background (BG), for the untaken branch of a
  // split -- Apple Maps' own look for it. Alpha untouched: the blend already reads as dimmed against BG, and the
  // roads it is mixed with are opaque already (KERBS/ROADS have none).
  static int dim(int color) {
    int br = (BG >> 16) & 0xff, bg = (BG >> 8) & 0xff, bb = BG & 0xff;
    int r = (((color >> 16) & 0xff) + br) / 2, g = (((color >> 8) & 0xff) + bg) / 2, b = ((color & 0xff) + bb) / 2;
    return 0xff000000 | (r << 16) | (g << 8) | b;
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

  // line() into carpetPaths (or carpetUnderPaths, Scene.routeUnder, Peter 2026-11-08), each segment into the path
  // for its traffic level (lv per segment, may be short) and under/over group.
  private void carpetLine(float[] p, byte[] lv, byte[] un, int from, float ce, float cn, float ch, float sh, float ik,
                          float fe, float fn) {
    boolean pen = false;
    int last = -1;
    float px = 0, pz = 0;
    for (int i = 2 * from; i + 1 < p.length; i += 2) {
      float de = (i == 2 * from && !Float.isNaN(fe) ? fe : p[i]) - ce;
      float dn = (i == 2 * from && !Float.isNaN(fe) ? fn : p[i + 1]) - cn;
      float x = (de * ch - dn * sh) * ik, z = (de * sh + dn * ch) * ik;
      if (i > 2 * from) {
        int seg = i / 2 - 1, l = lv != null && seg < lv.length ? lv[seg] : 0, k = l < 2 ? 0 : l - 1;
        boolean u = un != null && seg < un.length && un[seg] != 0;
        int key = u ? k + carpetPaths.length : k;  // either changing starts a new stroke, as a level change did
        if (key != last) pen = false;  // a new path starts its own stroke
        last = key;
        pen = segment(u ? carpetUnderPaths[k] : carpetPaths[k], px, pz, x, z, pen);
      }
      px = x;
      pz = z;
    }
  }

  // The carpet: the route from the car's place on it to the end of the scene, so the driven part is bare road.
  private void carpet(Scene s, Frame f, float ce, float cn, float ch, float sh, float ik, float reach) {
    if (f.cRun < 0) return;
    carpetLine(s.route[f.cRun], s.routeLevel[f.cRun], s.routeUnder[f.cRun], f.cSeg, ce, cn, ch, sh, ik, f.cutE, f.cutN);
    for (int r = f.cRun + 1; r < s.route.length; r++) {
      float[] b = s.routeBox[r];
      if (b[2] < ce - reach || b[0] > ce + reach || b[3] < cn - reach || b[1] > cn + reach) continue;
      carpetLine(s.route[r], s.routeLevel[r], s.routeUnder[r], 0, ce, cn, ch, sh, ik, Float.NaN, 0);
    }
  }

  // Where the carpet starts: the nearest point of the route to the car, as run cRun, segment cSeg, point (cutE, cutN)
  // of f, and how far along the route that is.
  private static void carpetStart(Scene s, Frame f) {
    float ce = f.ce, cn = f.cn, reach = f.reach;
    int bestRun = -1, bestSeg = 0;
    float bestD = Float.MAX_VALUE, be = 0, bn = 0;
    for (int r = 0; r < s.route.length; r++) {
      float[] b = s.routeBox[r];
      if (b[2] < ce - reach || b[0] > ce + reach || b[3] < cn - reach || b[1] > cn + reach) continue;
      float[] p = s.route[r];
      for (int i = 0; i + 3 < p.length; i += 2) {
        float ex = p[i + 2] - p[i], ny = p[i + 3] - p[i + 1], len2 = ex * ex + ny * ny;
        // clamped to 0..1 by hand: Math.min and max are calls on Dalvik, and this runs for every route segment a frame
        float t = len2 > 0 ? ((ce - p[i]) * ex + (cn - p[i + 1]) * ny) / len2 : 0;
        t = t < 0 ? 0 : t > 1 ? 1 : t;
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
    f.cRun = bestRun;
    f.cSeg = bestSeg;
    f.cutE = be;
    f.cutN = bn;
    if (bestRun >= 0) {
      float[] p = s.route[bestRun];
      f.cut = s.routeAlong[bestRun][bestSeg] + (float) Math.hypot(be - p[2 * bestSeg], bn - p[2 * bestSeg + 1]);
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

  // ---- Peter's mockup (2026-10-30, second version): a turn card down the left, the next street as a tag over the
  // map, the speed limit at the right, all on the dash's black. No close button: nothing on the cluster takes a touch.
  // The trip card is folded into the turn card as its last line and the column is 210 wide, not 270: the cards took
  // about half the meter (Peter 2026-10-31).

  static final float COL = 236;  // the dark band behind the cards, solid as far as they reach: roads showed between them
  // left of x 36 the tach needle rests on the cards at idle (Peter's photo 2026-10-29)
  // the turn card starts 28 down: at 12 its top left corner almost met the tach ring, and at 18 it still looked
  // cut off by it (Peter's photos 2026-10-30)
  static final float CARD_L = 36, CARD_R = 210, TURN_T = 28;
  // the turn card's rows, each there only when it has something: the arrow and the distance, the road across the
  // card, then time left, distance left and arrival. Its height follows them.
  private static final float STRIP_W = 10, ROW_X = 50, TEXT_X = 92, TOP_H = 42, ROAD_H = 24, ETA_H = 20, PAD = 6;
  // every card ends in a lean, its top right corner this much further right than its bottom right (Peter's mockup)
  private static final float SLANT = 14, RADIUS = 10, CORNER = 5;
  private static final int ACCENT = 0xfff2f2f2, ACCENT_DIM = 0xff3a3a3a;
  // the music card, with or without a route: the cover, where the sound comes from, the song, and whether it
  // plays. Only a glyph for that, no skip buttons, since the dash takes no touch.
  // at 270 wide it touched the car without a route (Peter 2026-10-30); now as wide as the turn card, and while
  // routing drawn SONG_GAP under it rather than here at the bottom (Peter 2026-10-31)
  private static final float SONG_T = 160, SONG_B = 208, SONG_R = CARD_R, COVER_L = 46, COVER_T = 164, COVER_PX = 40, SONG_X = 96;
  private static final float GLYPH_X = SONG_R - 32, GLYPH_R = 14, SONG_GAP = 8;
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

  private void overlay(Canvas canvas, long now, boolean free, boolean onGpu) {
    if (!onGpu) {  // else the GPU layer fades the roads itself
      if (!free) canvas.drawRect(0, 0, COL, 215, mask);
      canvas.drawRect(0, 0, 584, 78, fog);
    }
    // the signals under the names, as Google Maps has them
    for (int i = 0; i < sigCount && !onGpu; i++) SignalIcon.draw(canvas, sigSprite[i], sigXY[2 * i], sigXY[2 * i + 1], sigPaint, sigPath, sigRect);
    for (int i = 0; i < labelCount && !onGpu; i++) {
      if (!labelOn[i]) continue;
      if (labelRef[i]) {
        drawLabel(canvas, labelText[i], true, labelX[i] - labelW[i] / 2, labelY[i] + 4, labelBadge);
        continue;
      }
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
      // A route number in it, or the comma's ref for the road ("US 75" for North Central Expressway), is a white
      // badge before the name, as Google Maps writes the road (Peter 2026-10-31); the badge alone if the name will
      // not fit beside it even at 14 px.
      List<Shield.Piece> pieces = tagPieces(name.toString());
      boolean badges = pieces.size() > 1 || pieces.get(0).kind != Shield.TEXT;
      street.setTextSize(size);
      while ((badges ? piecesW(pieces, street, TAG_BADGE * size) : street.measureText(name.toString())) > maxw && size > 14)
        street.setTextSize(--size);
      if (badges && piecesW(pieces, street, TAG_BADGE * size) > maxw) {
        List<Shield.Piece> only = new ArrayList<Shield.Piece>();
        for (Shield.Piece p : pieces) if (p.kind != Shield.TEXT) only.add(p);
        pieces = only;
      }
      String n = badges ? null : TextUtils.ellipsize(name, street, maxw, TextUtils.TruncateAt.END).toString();
      float w = (badges ? Math.min(piecesW(pieces, street, TAG_BADGE * size), maxw) : street.measureText(n)) + 24;
      float t = streetY - 19, b = streetY + 7;
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
      if (n != null) canvas.drawText(n, cx, streetY, street);
      else {
        street.setTextAlign(Paint.Align.LEFT);
        drawPieces(canvas, pieces, cx - w / 2 + 12, cx + w / 2 - 12, streetY, street, size, TAG_BADGE);
        street.setTextAlign(Paint.Align.CENTER);
      }
    }
    if (!java.util.Arrays.equals(tagBox, tagShown)) tagShown = tagBox.clone();

    if (hasSign) {
      double v = Feed.limitMs * (Feed.metric ? 3.6 : 2.23694);
      speedSign(canvas, String.valueOf(Math.round(v)), 474, 30, 40, 52);
    }

    // The cards are drawn into a bitmap and only redrawn when something on them changes: drawing them every frame was
    // most of a frame on the Tegra (30-48 ms of 45-59 ms with a route, Peter 2026-10-30), though the map moved and they
    // did not. Their panels are opaque, so the bitmap over the map looks as drawing them straight on did.
    String key = cardKey(now, free);
    if (cardBmp == null) {
      cardBmp = Bitmap.createBitmap(CARDS_W, 215, Bitmap.Config.ARGB_8888);
      cardCanvas = new Canvas(cardBmp);
    }
    if (!key.equals(cardsKey)) {
      cardBmp.eraseColor(Color.TRANSPARENT);
      cards(cardCanvas, now, free);
      cardsKey = key;
    }
    canvas.drawBitmap(cardBmp, 0, 0, null);
    titleRoll.live(canvas, now);
    artistRoll.live(canvas, now);
    drawnRoll = rolled(now);

    // the lane row under the car (Lanes.AT_CAR), in the room the lifted map leaves; drawn once the lift is most of
    // the way, so it never sits over the car
    Feed.Nav nav = free ? null : turnNav(now);
    if (nav != null && Lanes.at == Lanes.AT_CAR && lift > LIFT - 4 && Lanes.shown(nav.lanes, turnLeft(nav, now))) {
      float half = Math.min(LANE_HALF, nav.lanes.length * LANE_CELL / 2f + 8);
      RectF box = new RectF(CAR_X - half, LANE_CAR_T, CAR_X + half, LANE_CAR_T + LANE_H - 2);
      canvas.drawRoundRect(box, 8, 8, panel);
      canvas.drawRoundRect(box, 8, 8, edge);
      laneRow(canvas, nav, box.left + 6, box.top, box.right - 6, box.bottom);
    }
  }

  private static final int CARDS_W = (int) CARD_R + 2;  // clear of the cards' right ends
  private Bitmap cardBmp;
  private Canvas cardCanvas;
  private String cardsKey;

  // Everything the cards show, as they would draw it now.
  private String cardKey(long now, boolean free) {
    StringBuilder k = new StringBuilder(160).append(free);
    Feed.Nav nav = free ? null : turnNav(now);
    if (nav != null) {
      double m = turnLeft(nav, now);
      k.append('|').append(nav.type).append('|').append(nav.modifier).append('|').append(Turn.text(nav.text));
      k.append('|').append(m >= 0 ? Turn.distance(m, Feed.metric) : null).append('|').append(nav.toward);
      if (Lanes.shown(nav.lanes, m)) {
        k.append('|').append(Lanes.at).append(java.util.Arrays.toString(nav.lanes)).append(java.util.Arrays.toString(nav.take));
      }
    }
    if (Feed.remainingSec >= 0) {
      double left = Math.max(0, Feed.remainingSec - (now - Feed.stateAt) / 1000.0);
      k.append('|').append(Math.round(left / 60)).append('|').append(clock.format(new Date(System.currentTimeMillis() + (long) (left * 1000))));
      if (Feed.remainingMeters >= 0) k.append('|').append(away(Feed.remainingMeters, Feed.metric));
    }
    String status = Feed.mediaStatus, title = Feed.mediaTitle;
    if (status != null && !TextUtils.isEmpty(title)) {
      k.append('|').append(status).append('|').append(title).append('|').append(Feed.mediaArtist).append('|').append(Feed.mediaSource);
      Bitmap cover = Feed.cover;
      k.append('|').append(cover != null && Feed.mediaArt != 0 && Feed.coverId == Feed.mediaArt ? System.identityHashCode(cover) : 0);
      long dur = Feed.mediaDur, pos = Feed.mediaPos;
      if (dur > 0 && pos >= 0) {
        if ("playing".equals(status)) pos += SystemClock.uptimeMillis() - Feed.mediaAt;
        k.append('|').append(Math.round(120 * Math.min(1f, pos / (float) dur)));  // the ring, to 3 degrees
      }
    }
    return k.toString();
  }

  private void cards(Canvas canvas, long now, boolean free) {
    // the turn card: a blue strip down its left edge that empties over the last 500 ft as the stock bar did, the
    // arrow and the distance, the maneuver's text, and the trip. The turn is the comma's own (Feed.nav, the step the
    // StarPilot Auto car screen draws), not Honda Hack's panel: that came through Android Auto's throttled next-turn
    // path and lagged or froze (Peter 2026-10-31: the dash said "0.2 mi US 75 South" while the car screen said
    // "Exit 6 Walnut Hill Lane 1.9 mi"). Gone when the comma has not said for NAV_STALE_MS.
    Feed.Nav nav = free ? null : turnNav(now);
    double meters = nav != null ? turnLeft(nav, now) : -1;
    String d = meters >= 0 ? Turn.distance(meters, Feed.metric) : null;
    Bitmap arrow = nav != null ? NavArrow.get(nav.type, nav.modifier) : null;
    List<List<Shield.Piece>> lines = nav != null ? turnLines(Turn.text(nav.text)) : NO_LINES;
    boolean top = nav != null && (d != null || arrow != null), road = !lines.isEmpty();
    boolean trip = !free && Feed.remainingSec >= 0;  // a card for the trip alone when there is no turn
    // where the maneuver leads, under its text as Google Maps writes it: "toward Garland"
    String toward = nav != null && road && !TextUtils.isEmpty(nav.toward) ? "toward " + nav.toward : null;
    float b = -1;
    if (top || road || trip) {
      b = TURN_T + (top ? 0 : PAD) + (top ? TOP_H : 0) + lines.size() * ROAD_H + (toward != null ? TOWARD_H : 0)
          + (trip ? ETA_H : 0) + PAD;
      Path turnCard = card(CARD_L, TURN_T, CARD_R, b);
      canvas.drawPath(turnCard, panel);
      double ft = meters < 0 ? -1 : meters * 3.28084;
      float lit = ft < 0 || ft > 500 ? b : b - (float) (ft / 500) * (b - TURN_T);
      canvas.save();
      canvas.clipPath(turnCard);
      canvas.drawRect(CARD_L, TURN_T, CARD_L + 0.4f * (CARD_R - CARD_L), b, turnGlow);  // a soft light from the left
      fill.setColor(ACCENT_DIM);
      canvas.drawRect(CARD_L, TURN_T, CARD_L + STRIP_W, lit, fill);
      fill.setColor(ACCENT);
      canvas.drawRect(CARD_L, lit, CARD_L + STRIP_W, b, fill);
      canvas.restore();
      canvas.drawPath(turnCard, edge);
      // a brighter rim on the strip's curve, lit as far as the strip is
      Path turnRim = leftEnd(CARD_L, TURN_T, b);
      canvas.save();
      canvas.clipPath(rect(CARD_L - 2, TURN_T - 2, CARD_L + RADIUS + 2, lit));
      rim.setColor(0xff4a4a4a);
      canvas.drawPath(turnRim, rim);
      canvas.restore();
      canvas.save();
      canvas.clipPath(rect(CARD_L - 2, lit, CARD_L + RADIUS + 2, b + 2));
      rim.setColor(0xffffffff);
      canvas.drawPath(turnRim, rim);
      canvas.restore();

      float y = TURN_T + (top ? 0 : PAD);  // the top of the next row
      if (top) {
        if (arrow != null) canvas.drawBitmap(arrow, null, new RectF(ICON_LEFT, ICON_TOP, ICON_LEFT + ICON, ICON_TOP + ICON), smooth);
        if (!TextUtils.isEmpty(d)) {
          String t = d.toString().trim();
          int sp = t.lastIndexOf(' ');
          String num = sp > 0 ? t.substring(0, sp) : t, u = sp > 0 ? t.substring(sp + 1) : "";
          float x = arrow != null ? TEXT_X : ROW_X, textW = right(y + 34, b) - x;
          float size = 30;  // a long distance shrinks to fit
          big.setTextSize(size);
          while (big.measureText(num) + 5 + unit.measureText(u) > textW && size > 22) big.setTextSize(--size);
          canvas.drawText(num, x, y + 34, big);
          canvas.drawText(u, x + big.measureText(num) + 5, y + 34, unit);
        }
        y += TOP_H;
      }
      // across the whole card, under the arrow: beside it only ~100 px were left. turnLines set the size.
      for (List<Shield.Piece> line : lines) {
        drawPieces(canvas, line, ROW_X, right(y + 18, b), y + 18, cardRoad, cardSize, CARD_BADGE);
        y += ROAD_H;
      }
      if (toward != null) {
        canvas.drawText(TextUtils.ellipsize(toward, cardDim, right(y + 14, b) - ROW_X, TextUtils.TruncateAt.END).toString(),
                        ROW_X, y + 14, cardDim);
        y += TOWARD_H;
      }
      if (trip) canvas.drawText(eta(now, right(y + 14, b) - ROW_X), ROW_X, y + 14, cardDim);
    }
    // the lane row under the turn card (Lanes.AT_CARD), a card of its own; the song goes under it, or waits for
    // the maneuver when it would run off the bottom
    float under = b;
    if (b > 0 && nav != null && Lanes.at == Lanes.AT_CARD && Lanes.shown(nav.lanes, meters)) {
      float t = b + LANE_GAP;
      Path laneCard = card(CARD_L, t, CARD_R, t + LANE_H);
      canvas.drawPath(laneCard, panel);
      canvas.drawPath(laneCard, edge);
      laneRow(canvas, nav, CARD_L + 8, t, CARD_R - SLANT - 4, t + LANE_H);
      under = t + LANE_H;
    }

    String status = Feed.mediaStatus, title = Feed.mediaTitle;
    float songT = under > 0 ? under + SONG_GAP : SONG_T;  // up under the turn card
    if (status != null && !TextUtils.isEmpty(title) && songT + SONG_B - SONG_T <= SONG_LAST) song(canvas, status, title, songT);
    else titleRoll.over = artistRoll.over = 0;
  }

  // The lane row (Lanes, Peter 2026-11-01, after Google Maps): each lane's arrows in a cell of its own, left to right,
  // the one to take in white over the rest in grey, thin lines between the lanes. Under the turn card or under the
  // car (Lanes.at).
  static final float LANE_H = 34, LANE_GAP = 4, LANE_CELL = 34, LANE_PX = 26, TOWARD_H = 20, SONG_LAST = 213;
  // under the car: centred on it, its half width at most clear of the cards, LANE_CAR_T down to 211
  static final float LANE_CAR_T = 179, LANE_HALF = CAR_X - CARD_R - 12;
  private static final int LANE_ON = 0xffffffff, LANE_OFF = 0xff5a5a5a, LANE_SEP = 0x30ffffff;
  private final Paint laneLine = new Paint(Paint.ANTI_ALIAS_FLAG), laneTip = new Paint(Paint.ANTI_ALIAS_FLAG);
  private final Path lanePath = new Path();
  private final float[] lanePts = new float[2 * Lanes.POINTS], laneHead = new float[6];
  private final int[] laneOrder = new int[10];

  private void laneRow(Canvas canvas, Feed.Nav nav, float l, float t, float r, float b) {
    int n = nav.lanes.length;
    float cw = Lanes.cell(n, r - l, LANE_CELL), x0 = (l + r - n * cw) / 2, cy = (t + b) / 2;
    float s = Math.min(cw * 0.9f, Math.min(b - t - 8, LANE_PX));
    laneLine.setStyle(Paint.Style.STROKE);
    laneLine.setStrokeCap(Paint.Cap.ROUND);
    laneLine.setStrokeJoin(Paint.Join.ROUND);
    laneLine.setStrokeWidth(Math.max(2, 0.13f * s));
    laneTip.setStyle(Paint.Style.FILL);
    for (int i = 0; i < n; i++) {
      float cx = x0 + (i + 0.5f) * cw;
      if (i > 0) {
        laneLine.setColor(LANE_SEP);
        laneLine.setStrokeWidth(1);
        canvas.drawLine(x0 + i * cw, t + 7, x0 + i * cw, b - 7, laneLine);
        laneLine.setStrokeWidth(Math.max(2, 0.13f * s));
      }
      int m = Lanes.order(nav.lanes[i], nav.take[i], laneOrder);
      for (int j = 0; j < m; j++) {
        int bit = laneOrder[j], c = bit == nav.take[i] ? LANE_ON : LANE_OFF;
        int pts = Lanes.arrow(bit, nav.left, cx, cy, s, lanePts, laneHead);
        lanePath.rewind();
        lanePath.moveTo(lanePts[0], lanePts[1]);
        for (int q = 1; q < pts; q++) lanePath.lineTo(lanePts[2 * q], lanePts[2 * q + 1]);
        laneLine.setColor(c);
        canvas.drawPath(lanePath, laneLine);
        lanePath.rewind();
        lanePath.moveTo(laneHead[0], laneHead[1]);
        lanePath.lineTo(laneHead[2], laneHead[3]);
        lanePath.lineTo(laneHead[4], laneHead[5]);
        lanePath.close();
        laneTip.setColor(c);
        canvas.drawPath(lanePath, laneTip);
      }
    }
  }

  // Where text on a row with its baseline at y has to end, clear of the turn card's lean (bottom at b).
  private static float right(float y, float b) {
    return CARD_R - SLANT * (y - TURN_T) / (b - TURN_T) - 6;
  }

  // The next maneuver, or null when the comma's last word on it is over NAV_STALE_MS old: a card that froze on an
  // old turn was the fault this replaced.
  private static Feed.Nav turnNav(long now) {
    Feed.Nav n = Feed.nav;
    return n != null && now - n.at <= NAV_STALE_MS ? n : null;
  }

  // Metres to the maneuver now: the comma's distance less what the car has driven since, at its speed; -1 if unknown.
  // The comma says once a second, so this keeps the number moving between its datagrams.
  static double turnLeft(Feed.Nav n, long now) {
    if (n.distance < 0) return -1;
    return Math.max(0, n.distance - Math.max(0, Feed.spd) * Math.max(0, now - n.at) / 1000.0);
  }

  private static final List<List<Shield.Piece>> NO_LINES = new ArrayList<List<Shield.Piece>>();
  private static final float LINE_W = CARD_R - 6 - 0.6f * SLANT - ROW_X;  // about where the lean cuts a text row
  // a badge's text against the line's: the card's 18-15 px text gets 15-13 px badges (a 20-17 px box in the 24 px
  // row); the tag's 18-14 px a little smaller, to stay inside its 26 px
  private static final float CARD_BADGE = 0.85f, TAG_BADGE = 0.8f;
  private float cardSize = 18;  // cardRoad's size as turnLines left it

  // The text in one line at 18 px, shrunk to 15 px if that fits; else two lines at 16 px broken between words, the
  // second ending in an ellipsis if it is still too long, as the car screen does with its two lines. Sets cardRoad's
  // size. An exit number ("Exit 34") and route numbers ("US 75") are badges (Shield) that a line never breaks inside;
  // with none the lines are the plain text as before.
  private List<List<Shield.Piece>> turnLines(String t) {
    if (t.isEmpty()) return NO_LINES;
    List<Shield.Piece> all = Shield.split(t);
    for (int size = 18; size >= 15; size--) {
      cardRoad.setTextSize(cardSize = size);
      if (piecesW(all, cardRoad, CARD_BADGE * size) <= LINE_W) return lines(all);
    }
    cardRoad.setTextSize(cardSize = 16);
    List<Shield.Piece> atoms = new ArrayList<Shield.Piece>();  // words and badges
    for (Shield.Piece p : all) {
      if (p.kind != Shield.TEXT) atoms.add(p);
      else for (String w : p.text.split(" ")) atoms.add(new Shield.Piece(Shield.TEXT, w));
    }
    int i = 1;
    while (i < atoms.size() && piecesW(join(atoms.subList(0, i + 1)), cardRoad, CARD_BADGE * 16) <= LINE_W) i++;
    List<Shield.Piece> first = join(atoms.subList(0, i));
    if (i >= atoms.size()) return lines(first);  // one word too long for a line: drawPieces cuts it
    return lines(first, join(atoms.subList(i, atoms.size())));
  }

  @SuppressWarnings("unchecked")
  private static List<List<Shield.Piece>> lines(List<Shield.Piece>... l) {
    return java.util.Arrays.asList(l);
  }

  // words run together into one piece of text between badges, so plain text measures and draws as one string
  private static List<Shield.Piece> join(List<Shield.Piece> atoms) {
    List<Shield.Piece> out = new ArrayList<Shield.Piece>();
    StringBuilder words = new StringBuilder();
    for (Shield.Piece p : atoms) {
      if (p.kind == Shield.TEXT) {
        if (words.length() > 0) words.append(' ');
        words.append(p.text);
        continue;
      }
      if (words.length() > 0) out.add(new Shield.Piece(Shield.TEXT, words.toString()));
      words.setLength(0);
      out.add(p);
    }
    if (words.length() > 0) out.add(new Shield.Piece(Shield.TEXT, words.toString()));
    return out;
  }

  // The width of pieces in a row: text in p, badges with bs px text, a space between pieces.
  private float piecesW(List<Shield.Piece> pieces, TextPaint p, float bs) {
    float w = 0, gap = p.measureText(" ");
    for (int i = 0; i < pieces.size(); i++) {
      Shield.Piece q = pieces.get(i);
      w += (i > 0 ? gap : 0)
          + (q.kind == Shield.TEXT ? p.measureText(q.text) : badgeW(badgeText(q), bs, badgeInk));
    }
    return w;
  }

  // Draws pieces from x on the baseline base, the text in p (left aligned: it is size px), the badges centred on the
  // text's caps. Text that runs past end ends in an ellipsis; a badge that does not fit is left off, with what
  // follows it.
  private void drawPieces(Canvas c, List<Shield.Piece> pieces, float x, float end, float base, TextPaint p, float size,
                          float scale) {
    float gap = p.measureText(" "), mid = base - 0.3635f * size;
    for (int i = 0; i < pieces.size() && x < end; i++) {
      Shield.Piece q = pieces.get(i);
      if (q.kind == Shield.TEXT) {
        String t = fit(q.text, p, end - x);
        c.drawText(t, x, base, p);
        x += p.measureText(t) + gap;
        if (!t.equals(q.text)) break;
        continue;
      }
      String t = badgeText(q);
      if (x + badgeW(t, scale * size, badgeInk) > end + 0.5f) break;
      x += badge(c, q.kind, t, x, mid, scale * size, badgeInk, badgeBox, null) + gap;
    }
  }

  // The street tag's pieces: the name with any route number in it as a badge, else the comma's ref for the road
  // ("US 75") as a badge before the name.
  private static List<Shield.Piece> tagPieces(String name) {
    List<Shield.Piece> pieces = Shield.split(name);
    if (pieces.isEmpty()) pieces.add(new Shield.Piece(Shield.TEXT, name));
    for (Shield.Piece p : pieces) if (p.kind == Shield.REF) return pieces;
    String ref = name.equals(Feed.road) ? Shield.firstRef(Feed.roadRef) : null;
    if (ref != null) pieces.add(0, new Shield.Piece(Shield.REF, ref));
    return pieces;
  }

  // The trip as one line: "11 min · 5.2 mi · 2:10 AM", the values the trip card showed (Peter 2026-10-31). A piece
  // that is missing is left out; the distance goes first if the line is too long for w.
  private String eta(long now, float w) {
    double left = Math.max(0, Feed.remainingSec - (now - Feed.stateAt) / 1000.0);
    long min = Math.round(left / 60);
    String n = min >= 60 ? (min / 60) + " h " + (min % 60) + " min" : min + " min";
    String at = clock.format(new Date(System.currentTimeMillis() + (long) (left * 1000)));
    String dot = "\u2009\u00b7\u2009";  // thin spaces: with full ones "11 min · 5.2 mi · 2:10 AM" ran into the lean
    String line = n + dot + at;
    if (Feed.remainingMeters >= 0) {
      String all = n + dot + away(Feed.remainingMeters, Feed.metric) + dot + at;
      if (cardDim.measureText(all) <= w) line = all;
    }
    return fit(line, cardDim, w);
  }

  // the music card with its top at t: SONG_T, or under the turn card while routing
  private void song(Canvas canvas, String status, String title, float t) {
    float o = t - SONG_T, b = SONG_B + o;
    if (t != songAt) {
      songCard = card(CARD_L, t, SONG_R, b);
      logoWaves = spotifyWaves(SONG_X + LOGO_R, t + 10, LOGO_R);
      songAt = t;
    }
    boolean spotify = "spotify".equals(Feed.mediaSource);
    canvas.drawPath(songCard, panel);
    if (spotify) {  // Spotify's green comes in from the lean
      canvas.save();
      canvas.clipPath(songCard);
      canvas.drawRect(SONG_R - 0.35f * (SONG_R - CARD_L), t, SONG_R, b, greenGlow);
      canvas.restore();
    }
    canvas.drawPath(songCard, edge);

    RectF art = new RectF(COVER_L, COVER_T + o, COVER_L + COVER_PX, COVER_T + o + COVER_PX);
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
      canvas.drawCircle(x + LOGO_R, t + 10, LOGO_R, fill);
      canvas.drawPath(logoWaves, waves);
      x += 2 * LOGO_R + 4;
    }
    if (!from.isEmpty()) canvas.drawText(fit(from, songFrom, w - (x - SONG_X)), x, t + 14, songFrom);
    titleRoll.draw(canvas, title, songTitle, SONG_X, t + 30, w, t, b);
    String artist = Feed.mediaArtist;
    if (!TextUtils.isEmpty(artist)) artistRoll.draw(canvas, artist.toUpperCase(Locale.US), songArtist, SONG_X, t + 44, w, t, b);
    else artistRoll.over = 0;

    // pause bars while it plays, a play triangle while paused
    float cy = (t + b) / 2;
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

    private TextPaint paint;
    private float x, y, w, top, bottom;

    // Into the cards' bitmap: a line that fits is drawn there; one that scrolls is only measured and placed, and
    // live() draws it over the bitmap each frame.
    void draw(Canvas canvas, String s, TextPaint p, float x, float y, float w, float top, float bottom) {
      long now = SystemClock.uptimeMillis();
      if (!s.equals(text)) { text = s; since = now; }
      over = p.measureText(s) - w;
      paint = p;
      this.x = x; this.y = y; this.w = w; this.top = top; this.bottom = bottom;
      if (over <= 0) canvas.drawText(s, x, y, p);
    }

    void live(Canvas canvas, long now) {
      if (over <= 0 || text == null || paint == null) return;
      canvas.save();
      canvas.clipRect(x, top, x + w, bottom);  // sideways only: capital accents reach up past the line
      canvas.drawText(text, x - dx(now), y, paint);
      canvas.restore();
    }

    // the strip a scroll step redraws, added to r
    void box(Rect r) {
      if (over > 0) r.union((int) Math.floor(x) - 1, (int) Math.floor(top), (int) Math.ceil(x + w) + 1, (int) Math.ceil(bottom));
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
