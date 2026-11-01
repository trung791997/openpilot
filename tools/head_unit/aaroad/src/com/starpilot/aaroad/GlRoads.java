package com.starpilot.aaroad;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.PixelFormat;
import android.opengl.EGL14;
import android.opengl.EGLConfig;
import android.opengl.EGLContext;
import android.opengl.EGLDisplay;
import android.opengl.EGLSurface;
import android.opengl.GLES20;
import android.opengl.GLUtils;
import android.os.SystemClock;
import android.text.TextPaint;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import de.robv.android.xposed.XposedBridge;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;

// The map's grid, roads, route and carpet drawn with GLES2 (Peter 2026-10-30: the software canvas spent most of a
// frame stroking them on the Tegra), and the fades over them: the dark band behind the cards and the fog at the top.
// and the street labels over those. MapView keeps the cards, car, tag and sign, and leaves the road area clear when
// this layer has drawn. The view still chooses the labels; each name is written once, by the same text code and font
// as the view's, into a row of a texture, and every frame the row is laid on the map at whole pixels, so it reads as
// sharp as before.
//
// When it draws: on its own clock, every FRAME_MS (30 a second, Peter 2026-10-30; the view drew at most 15), taking
// the camera from MapView.frame() each time, so the map moves without the view redrawing. With the car standing and
// the zoom settled it draws every IDLE_MS, or at once when MapView pokes it (a new scene, anything else redrawn).
//
// Where it goes: a plain SurfaceView, under MapView in the meter's root and the same size. Its surface sits behind the
// cluster window; the view punches a clear hole in the window for it, so the map's masks and fog still blend over the
// roads. The meter window is drawn in software and has no TextureView; the only placement seen working on this unit
// so far is a SurfaceView over the window (GlProbe), and that would cover the cards.
//
// How it draws: every piece of every road is uploaded once per scene as one quad per segment carrying both end
// points. The quads are packed off the GL thread (aaroad-pack) and sent a slice a frame into a second buffer while the
// last scene goes on drawing; the new scene is drawn from the frame after its last slice (Peter's drive 2026-10-31:
// the upload in one frame stalled it 330-360 ms). The vertex shader turns them with the car's pose and the zoom and
// widens the quad by the half width; the fragment shader keeps what lies within the half width of the segment, so joins and caps come
// out round as Skia strokes them, smoothed over one screen pixel. A frame changes only a few uniforms: the camera is
// MapView's, as a projection whose near plane drops what lies behind the horizon. The carpet is the route's own
// quads, with each point's distance along the route: what lies before the car's place on it is dropped, with a round
// cap there. The grid is one quad over the box the view clips its grid to, with the lines found per pixel.
//
// If EGL or GL fails anywhere, the layer logs why, hides, and the view draws everything itself as before.
final class GlRoads extends SurfaceView implements SurfaceHolder.Callback, MapView.Roads, Runnable {
  private static final int MAX_QUADS = 16384;  // 4 corners each, under the 65536 a short index reaches
  private static final int STRIDE = GlCull.STRIDE;  // per corner: a.e, a.n, b.e, b.n, across, end, along at a
  private static final int SLICE = 256 * 1024;  // bytes of a new scene sent a frame, while the last one draws
  private static final float NEAR_W = 0.05f;  // the near plane, in the camera's w: 1 at the car, 0 at the horizon
  private static final int BG = 0xff050505, KERB = MapView.KERB_C, ROAD = MapView.ROAD_C, GRID = 0xff2a2a2a;
  // groups 0-4 the road classes, 5-9 their bridges (MapView.BRIDGE), 10-13 the route by traffic: unknown or low,
  // moderate, heavy, severe. A group with nothing in view costs no draw call.
  private static final int ROUTE = 2 * MapView.BRIDGE, LEVELS = 4;
  private static final long FRAME_MS = 33, IDLE_MS = 500;
  private static final int ATLAS_W = 256, ATLAS_H = 128, ROW = 32, PAD = 4, BASE = 22;  // a label per row, its baseline at BASE
  private static boolean loggedOn;

  private static final String LINE_VS =
      "uniform mat4 uP; uniform vec4 uW; uniform vec4 uV; uniform vec4 uCam; uniform float uR; uniform vec4 uCut;\n"
      + "attribute vec4 aSeg; attribute vec3 aC;\n"
      + "varying vec2 vPA; varying vec2 vU; varying float vL; varying float vW; varying float vNum;\n"
      + "varying float vRel; varying vec2 vAC;\n"
      + "vec2 car(vec2 p) { return vec2(uW.x * p.x - uW.y * p.y + uW.z, uW.y * p.x + uW.x * p.y + uW.w); }\n"
      + "void main() {\n"
      + "  vec2 a = car(aSeg.xy), b = car(aSeg.zw), d = b - a;\n"
      + "  float l = length(d);\n"
      + "  vec2 u = l > 1e-5 ? d / l : vec2(1.0, 0.0);\n"
      + "  vec2 e = aC.y < 0.5 ? a : b;\n"
      + "  float w = abs(uCam.z * e.y + 1.0);\n"
      // room for the smoothing: about two screen pixels in ground units there, each way
      + "  float m = uR + 2.0 * (w / uCam.x + w * w * abs(uCam.w) * 2.0);\n"
      + "  vec2 p = e + u * (aC.y < 0.5 ? -m : m) + vec2(-u.y, u.x) * (aC.x * m);\n"
      + "  vPA = p - a; vU = u; vL = l;\n"
      + "  vRel = (aC.z - uCut.x) * uCut.y; vAC = a - uCut.zw;\n"
      + "  vec4 c = uP * vec4(p, 0.0, 1.0);\n"
      + "  vW = c.w; vNum = uV.x * p.x + uV.y * p.y + uV.z;\n"
      + "  gl_Position = c;\n"
      + "}\n";

  // ground units per screen pixel at the fragment: c1 across the screen, c2 down it (a step down also slides sideways)
  private static final String FOOT =
      "vec2 c2(float w, float num) { float dz = w * w * uCam.w; return vec2((num / w * uCam.z - uCam.y) / uCam.x * dz, dz); }\n";

  private static final String LINE_FS =
      "precision mediump float;\n"
      + "uniform vec4 uColor; uniform float uR; uniform vec4 uCam; uniform float uCarpet;\n"
      + "varying vec2 vPA; varying vec2 vU; varying float vL; varying float vW; varying float vNum;\n"
      + "varying float vRel; varying vec2 vAC;\n"
      + FOOT
      + "void main() {\n"
      + "  float t = dot(vPA, vU), tc = clamp(t, 0.0, vL);\n"
      + "  vec2 g;\n"
      + "  if (uCarpet > 0.5 && vRel + tc < 0.0) {\n"
      // behind the car's place on the route: only the segment it lies on draws, the round cap at that place
      + "    if (vRel > 0.0 || vRel + vL < 0.0) discard;\n"
      + "    g = vPA + vAC;\n"
      + "  } else {\n"
      + "    g = vPA - vU * tc;\n"
      + "  }\n"
      + "  float d = length(g);\n"
      + "  vec2 n = g / max(d, 1e-3);\n"
      + "  float fp = length(vec2(n.x * vW / uCam.x, dot(n, c2(vW, vNum))));\n"
      + "  float a = clamp((uR - d) / max(fp, 1e-3) + 0.5, 0.0, 1.0);\n"
      + "  if (a <= 0.0) discard;\n"
      + "  gl_FragColor = vec4(uColor.rgb, uColor.a * a);\n"
      + "}\n";

  private static final String GRID_VS =
      "uniform mat4 uP; uniform vec4 uV; uniform vec4 uRot;\n"
      + "attribute vec2 aP;\n"
      + "varying vec2 vG; varying float vW; varying float vNum;\n"
      + "void main() {\n"
      + "  vG = vec2(aP.x * uRot.x + aP.y * uRot.y + uRot.z, -aP.x * uRot.y + aP.y * uRot.x + uRot.w);\n"
      + "  vec4 c = uP * vec4(aP, 0.0, 1.0);\n"
      + "  vW = c.w; vNum = uV.x * aP.x + uV.y * aP.y + uV.z;\n"
      + "  gl_Position = c;\n"
      + "}\n";

  // a one-pixel line where the view's unsmoothed hairline grid falls: within half a pixel along the steeper axis
  private static final String GRID_FS =
      "precision mediump float;\n"
      + "uniform vec4 uColor; uniform vec4 uCam; uniform vec4 uRot; uniform float uStep;\n"
      + "varying vec2 vG; varying float vW; varying float vNum;\n"
      + FOOT
      + "float px(vec2 n, vec2 a, vec2 b) { return max(abs(dot(n, a)), abs(dot(n, b))); }\n"
      + "void main() {\n"
      + "  vec2 a = vec2(vW / uCam.x, 0.0), b = c2(vW, vNum);\n"
      + "  float h = 0.5 * uStep;\n"
      + "  float de = abs(mod(vG.x + h, uStep) - h), dn = abs(mod(vG.y + h, uStep) - h);\n"
      + "  if (de >= 0.5 * px(vec2(uRot.x, uRot.y), a, b) && dn >= 0.5 * px(vec2(-uRot.y, uRot.x), a, b)) discard;\n"
      + "  gl_FragColor = uColor;\n"
      + "}\n";

  // a fade in screen pixels: the rectangle uRect, its alpha from uA.x to uA.y at uA.w and on to uA.z, across (uAxis 0)
  // or down (1), as the view's LinearGradient with three stops; the colour the same at all three
  private static final String FADE_VS =
      "uniform vec4 uRect; uniform vec2 uSize; uniform float uAxis;\n"
      + "attribute vec2 aP;\n"
      + "varying float vT;\n"
      + "void main() {\n"
      + "  vec2 p = mix(uRect.xy, uRect.zw, aP);\n"
      + "  vT = uAxis < 0.5 ? aP.x : aP.y;\n"
      + "  gl_Position = vec4(p.x / uSize.x * 2.0 - 1.0, 1.0 - p.y / uSize.y * 2.0, 0.0, 1.0);\n"
      + "}\n";

  private static final String FADE_FS =
      "precision mediump float;\n"
      + "uniform vec4 uA; uniform vec3 uRgb;\n"
      + "varying float vT;\n"
      + "void main() {\n"
      + "  float a = vT < uA.w ? mix(uA.x, uA.y, vT / uA.w) : mix(uA.y, uA.z, (vT - uA.w) / (1.0 - uA.w));\n"
      + "  gl_FragColor = vec4(uRgb, a);\n"
      + "}\n";

  private static final String TEXT_VS =
      "uniform vec4 uRect; uniform vec2 uSize; uniform vec4 uUV;\n"
      + "attribute vec2 aP;\n"
      + "varying vec2 vUV;\n"
      + "void main() {\n"
      + "  vec2 p = mix(uRect.xy, uRect.zw, aP);\n"
      + "  vUV = mix(uUV.xy, uUV.zw, aP);\n"
      + "  gl_Position = vec4(p.x / uSize.x * 2.0 - 1.0, 1.0 - p.y / uSize.y * 2.0, 0.0, 1.0);\n"
      + "}\n";

  private static final String TEXT_FS =
      "precision mediump float;\n"
      + "uniform sampler2D uTex;\n"
      + "varying vec2 vUV;\n"
      + "void main() {\n"
      + "  gl_FragColor = texture2D(uTex, vUV);\n"
      + "}\n";

  private final MapView map;
  private final float[] v = MapView.cameraValues();
  private final Object lock = new Object();

  // handed over under lock: the surface and pokes from the UI thread
  private SurfaceHolder holder, glHolder;
  private boolean poked, threadStarted;
  private long glSum, glCount;
  // the time from one moving frame to the next, for the log: 1 ms buckets, the last one everything longer
  private final int[] gaps = new int[200];
  private long gapSum, gapCount, gapMax;

  private volatile boolean ready, failed;

  // GL thread only
  private EGLDisplay dpy = EGL14.EGL_NO_DISPLAY;
  private EGLConfig cfg;
  private EGLContext ctx = EGL14.EGL_NO_CONTEXT;
  private EGLSurface surf = EGL14.EGL_NO_SURFACE;
  private int line, grid, fade, text, vbo, ibo, quad, unit, tex;
  private int lP, lW, lV, lCam, lR, lCut, lColor, lCarpet, lSeg, lC;
  private int gP, gV, gRot, gCam, gColor, gStep, gA;
  private int fRect, fSize, fAxis, fA, fRgb, fP;
  private int tRect, tSize, tUV, tTex, tP;
  private Bitmap atlas;
  private Canvas atlasCanvas;
  private TextPaint[] paints;
  private final String[] atlasText = new String[ATLAS_H / ROW];
  private final float[] lq = new float[2];
  // the traffic signal and stop icons: their sprites' texture, and this frame's choice and places (MapView.placeSignals)
  private int sigTex;
  private final Signals.Choice sigChoice = new Signals.Choice();
  private final float[] sigQ = new float[2], sigXY = new float[2 * Signals.MAX];
  private final int[] sigSprite = new int[Signals.MAX];
  private GlClose close;  // null when its program did not build
  private MapView.Scene glScene;  // the scene vbo holds, and the one drawn
  private GlCull cur;              // its pieces and their cull
  private GlCull pend;             // packed, being sent into back from byte pendOff
  private int back, pendOff;
  private MapView.Scene asked;     // the last scene the packer was asked for
  private final MapView.Frame glFrame = new MapView.Frame();
  private final int[] surfSize = new int[2];
  // handed between the GL thread and the packer under packLock: the scene to pack, the result, and bytes, which is
  // the packer's from an ask until the GL thread has sent the last slice
  private final Object packLock = new Object();
  private MapView.Scene packAsk;
  private GlCull packDone;
  private boolean packBusy, packStarted;
  private ByteBuffer bytes;
  private boolean checkNext;
  private final float[] proj = new float[16];

  // Where a moving frame's time goes (Peter 2026-10-31: 32-43 ms a frame and 233-378 ms worst while driving, 10-12 ms
  // standing), logged every PARTS_MS. GL calls only queue work, so the GPU's own time shows up where the driver
  // blocks: mostly the swap. Each part() closes the stage since the last mark; primitives only, nothing allocated.
  private static final int P_LOCK = 0, P_CAMERA = 1, P_UPLOAD = 2, P_CLEAR = 3, P_GRID = 4, P_CULL = 5, P_ROADS = 6,
      P_CARPET = 7, P_FADE = 8, P_LABELS = 9, P_ATLAS = 10, P_SWAP = 11, P_SLEEP = 12, PARTS = 13;
  private static final String[] PART_NAMES = {"lock", "camera", "upload", "clear", "grid", "cull", "roads", "carpet",
                                              "fade", "labels", "atlas", "swap", "sleep"};
  private static final long PARTS_MS = 10000, SLOW_NS = 100000000L;
  private final long[] partNow = new long[PARTS], partSum = new long[PARTS], partMax = new long[PARTS];
  private final int[] slowTop = new int[PARTS];
  private long partMark, partsFrom;
  private int partFrames, slowFrames, atlasUploads, sceneUploads;

  GlRoads(Context context, MapView map) {
    super(context);
    this.map = map;
    getHolder().setFormat(PixelFormat.RGBA_8888);
    getHolder().addCallback(this);
    setVisibility(INVISIBLE);  // shown with the map; INVISIBLE leaves no hole in the window
  }

  // ---- MapView.Roads, on the UI thread

  @Override
  public boolean ready() {
    return ready && !failed;
  }

  @Override
  public void poke() {
    synchronized (lock) {
      poked = true;
      lock.notifyAll();
    }
  }

  // GL ms a frame, and while moving the frame rate, the longest gap between frames and the 95th percentile one.
  @Override
  public String takeStats() {
    synchronized (lock) {
      String out = "gl " + (glCount > 0 ? Math.round(glSum / 1e6 / glCount) : -1) + " ms";
      if (gapCount > 0) {
        long p95 = 0;
        for (long n = 0; p95 < gaps.length - 1 && (n += gaps[(int) p95]) < 0.95 * gapCount; ) p95++;
        out += String.format(java.util.Locale.US, ", %.1f fps, worst %d ms, p95 %d ms", 1000.0 * gapCount / gapSum,
            gapMax, p95);
      } else {
        out += ", no moving frames";
      }
      glSum = 0;
      glCount = 0;
      java.util.Arrays.fill(gaps, 0);
      gapSum = 0;
      gapCount = 0;
      gapMax = 0;
      return out;
    }
  }

  @Override
  public void setShown(boolean on) {
    setVisibility(on && !failed ? VISIBLE : INVISIBLE);
  }

  // ---- the surface, on the UI thread

  @Override
  public void surfaceCreated(SurfaceHolder h) {
    synchronized (lock) {
      holder = h;
      if (!threadStarted && !failed) {
        threadStarted = true;
        Thread t = new Thread(this, "aaroad-gl");
        t.setDaemon(true);
        t.start();
      }
      lock.notifyAll();
    }
  }

  @Override
  public void surfaceChanged(SurfaceHolder h, int format, int w, int hgt) {
    poke();
  }

  // The surface may not outlive this call: wait for the GL thread to let go of it, a second at most.
  @Override
  public void surfaceDestroyed(SurfaceHolder h) {
    synchronized (lock) {
      holder = null;
      lock.notifyAll();
      long end = SystemClock.uptimeMillis() + 1000;
      try {
        for (long left; glHolder != null && !failed && (left = end - SystemClock.uptimeMillis()) > 0; ) lock.wait(left);
      } catch (InterruptedException ignored) {
      }
    }
  }

  // ---- the GL thread

  @Override
  public void run() {
    try {
      String why = initEgl();
      if (why != null) {
        fail(why);
        return;
      }
      long next = 0, last = 0;
      boolean idle = false;
      while (!failed) {
        SurfaceHolder h;
        java.util.Arrays.fill(partNow, 0);
        partMark = System.nanoTime();
        synchronized (lock) {
          part(P_LOCK);  // takeStats and surfaceDestroyed hold it on the UI thread
          // until the next frame is due, the surface changes, or, while idle, a poke
          while (holder == glHolder) {
            if (glHolder == null) {
              lock.wait();
              continue;
            }
            long left = next - SystemClock.uptimeMillis();
            if (left <= 0 || (idle && poked)) break;
            lock.wait(left);
          }
          h = holder;
          poked = false;
        }
        part(P_SLEEP);
        if (h != glHolder) {
          why = setSurface(h);
          if (why != null) {
            fail(why);
            return;
          }
          next = 0;
          last = 0;
          continue;
        }
        long start = SystemClock.uptimeMillis();
        MapView.Scene newest = map.scene();
        if (newest == null) {
          idle = true;
          next = start + IDLE_MS;
          continue;
        }
        part(P_CAMERA);
        why = stepUpload(newest);
        if (why != null) {
          fail(why);
          return;
        }
        part(P_UPLOAD);
        MapView.Scene s = glScene;  // the scene on the GPU, until the newest is all there
        if (s == null) {
          // packing the first scene: the view draws the roads meanwhile; the packer pokes when done
          idle = true;
          next = start + IDLE_MS;
          continue;
        }
        MapView.Frame f = glFrame;
        // frame() is synchronized on the view, which onDraw also calls; taken here first so the wait is seen apart
        synchronized (map) {
          part(P_LOCK);
          map.frame(s, start, f);
        }
        part(P_CAMERA);
        why = drawFrame(s, f);
        if (why != null) {
          fail(why);
          return;
        }
        boolean counted = last > 0 && !idle;  // the frames the gaps count: after a frame's sleep, not an idle one
        if (counted) {
          long gap = start - last;
          synchronized (lock) {
            part(P_LOCK);
            gaps[(int) Math.min(gap, gaps.length - 1)]++;
            gapSum += gap;
            gapCount++;
            gapMax = Math.max(gapMax, gap);
          }
        }
        part(P_LOCK);
        if (counted) addParts();
        last = start;
        // a frame that ran late is followed at once, not skipped
        idle = !f.moving && f.settled && pend == null;
        next = idle ? start + IDLE_MS : Math.max(start + FRAME_MS, SystemClock.uptimeMillis());
        logParts(start);  // after next is set, so its own time comes out of the coming sleep
      }
    } catch (Throwable t) {
      fail(String.valueOf(t));
    }
  }

  private String initEgl() {
    dpy = EGL14.eglGetDisplay(EGL14.EGL_DEFAULT_DISPLAY);
    int[] ver = new int[2];
    if (dpy == null || dpy == EGL14.EGL_NO_DISPLAY || !EGL14.eglInitialize(dpy, ver, 0, ver, 1)) return "no EGL display";
    int[] attrs = {EGL14.EGL_RENDERABLE_TYPE, EGL14.EGL_OPENGL_ES2_BIT, EGL14.EGL_SURFACE_TYPE, EGL14.EGL_WINDOW_BIT,
                   EGL14.EGL_RED_SIZE, 8, EGL14.EGL_GREEN_SIZE, 8, EGL14.EGL_BLUE_SIZE, 8, EGL14.EGL_ALPHA_SIZE, 8,
                   EGL14.EGL_NONE};
    EGLConfig[] c = new EGLConfig[1];
    int[] n = new int[1];
    if (!EGL14.eglChooseConfig(dpy, attrs, 0, c, 0, 1, n, 0) || n[0] < 1) return "no GLES2 RGBA8888 window config";
    cfg = c[0];
    ctx = EGL14.eglCreateContext(dpy, cfg, EGL14.EGL_NO_CONTEXT, new int[] {EGL14.EGL_CONTEXT_CLIENT_VERSION, 2, EGL14.EGL_NONE}, 0);
    if (ctx == null || ctx == EGL14.EGL_NO_CONTEXT) return "no GLES2 context, egl error 0x" + Integer.toHexString(EGL14.eglGetError());
    return null;
  }

  // Lets go of the old surface (the context keeps the buffers and programs) and takes the new one, if any.
  private String setSurface(SurfaceHolder h) {
    if (ready) {
      ready = false;
      invalidateMap();  // the view draws the roads again until this layer has drawn on the new surface
    }
    if (surf != EGL14.EGL_NO_SURFACE) {
      EGL14.eglMakeCurrent(dpy, EGL14.EGL_NO_SURFACE, EGL14.EGL_NO_SURFACE, EGL14.EGL_NO_CONTEXT);
      EGL14.eglDestroySurface(dpy, surf);
      surf = EGL14.EGL_NO_SURFACE;
    }
    String why = null;
    if (h != null) {
      surf = EGL14.eglCreateWindowSurface(dpy, cfg, h, new int[] {EGL14.EGL_NONE}, 0);
      if (surf == null || surf == EGL14.EGL_NO_SURFACE) {
        surf = EGL14.EGL_NO_SURFACE;
        why = "no window surface, egl error 0x" + Integer.toHexString(EGL14.eglGetError());
      } else if (!EGL14.eglMakeCurrent(dpy, surf, surf, ctx)) {
        why = "makeCurrent failed, egl error 0x" + Integer.toHexString(EGL14.eglGetError());
      } else if (line == 0) {
        why = setupGl();
      }
      checkNext = true;
    }
    synchronized (lock) {
      glHolder = why == null ? h : null;
      lock.notifyAll();
    }
    return why;
  }

  private String setupGl() {
    line = GlProbe.program(LINE_VS, LINE_FS);
    grid = GlProbe.program(GRID_VS, GRID_FS);
    fade = GlProbe.program(FADE_VS, FADE_FS);
    text = GlProbe.program(TEXT_VS, TEXT_FS);
    if (line == 0 || grid == 0 || fade == 0 || text == 0) return "shader compile failed";
    lP = GLES20.glGetUniformLocation(line, "uP");
    lW = GLES20.glGetUniformLocation(line, "uW");
    lV = GLES20.glGetUniformLocation(line, "uV");
    lCam = GLES20.glGetUniformLocation(line, "uCam");
    lR = GLES20.glGetUniformLocation(line, "uR");
    lCut = GLES20.glGetUniformLocation(line, "uCut");
    lColor = GLES20.glGetUniformLocation(line, "uColor");
    lCarpet = GLES20.glGetUniformLocation(line, "uCarpet");
    lSeg = GLES20.glGetAttribLocation(line, "aSeg");
    lC = GLES20.glGetAttribLocation(line, "aC");
    gP = GLES20.glGetUniformLocation(grid, "uP");
    gV = GLES20.glGetUniformLocation(grid, "uV");
    gRot = GLES20.glGetUniformLocation(grid, "uRot");
    gCam = GLES20.glGetUniformLocation(grid, "uCam");
    gColor = GLES20.glGetUniformLocation(grid, "uColor");
    gStep = GLES20.glGetUniformLocation(grid, "uStep");
    gA = GLES20.glGetAttribLocation(grid, "aP");
    fRect = GLES20.glGetUniformLocation(fade, "uRect");
    fSize = GLES20.glGetUniformLocation(fade, "uSize");
    fAxis = GLES20.glGetUniformLocation(fade, "uAxis");
    fA = GLES20.glGetUniformLocation(fade, "uA");
    fRgb = GLES20.glGetUniformLocation(fade, "uRgb");
    fP = GLES20.glGetAttribLocation(fade, "aP");
    tRect = GLES20.glGetUniformLocation(text, "uRect");
    tSize = GLES20.glGetUniformLocation(text, "uSize");
    tUV = GLES20.glGetUniformLocation(text, "uUV");
    tTex = GLES20.glGetUniformLocation(text, "uTex");
    tP = GLES20.glGetAttribLocation(text, "aP");
    if (lSeg < 0 || lC < 0 || gA < 0 || fP < 0 || tP < 0) return "shader attributes missing";
    // the labels' texture: texels laid one to one on screen pixels, so nearest
    int[] t = new int[1];
    GLES20.glGenTextures(1, t, 0);
    tex = t[0];
    GLES20.glBindTexture(GLES20.GL_TEXTURE_2D, tex);
    GLES20.glTexParameteri(GLES20.GL_TEXTURE_2D, GLES20.GL_TEXTURE_MIN_FILTER, GLES20.GL_NEAREST);
    GLES20.glTexParameteri(GLES20.GL_TEXTURE_2D, GLES20.GL_TEXTURE_MAG_FILTER, GLES20.GL_NEAREST);
    GLES20.glTexParameteri(GLES20.GL_TEXTURE_2D, GLES20.GL_TEXTURE_WRAP_S, GLES20.GL_CLAMP_TO_EDGE);
    GLES20.glTexParameteri(GLES20.GL_TEXTURE_2D, GLES20.GL_TEXTURE_WRAP_T, GLES20.GL_CLAMP_TO_EDGE);
    // the signal icons' sprites, drawn once by the view's own routine (SignalIcon.draw) and laid one texel to a pixel
    GLES20.glGenTextures(1, t, 0);
    sigTex = t[0];
    GLES20.glBindTexture(GLES20.GL_TEXTURE_2D, sigTex);
    GLES20.glTexParameteri(GLES20.GL_TEXTURE_2D, GLES20.GL_TEXTURE_MIN_FILTER, GLES20.GL_NEAREST);
    GLES20.glTexParameteri(GLES20.GL_TEXTURE_2D, GLES20.GL_TEXTURE_MAG_FILTER, GLES20.GL_NEAREST);
    GLES20.glTexParameteri(GLES20.GL_TEXTURE_2D, GLES20.GL_TEXTURE_WRAP_S, GLES20.GL_CLAMP_TO_EDGE);
    GLES20.glTexParameteri(GLES20.GL_TEXTURE_2D, GLES20.GL_TEXTURE_WRAP_T, GLES20.GL_CLAMP_TO_EDGE);
    Bitmap sprites = signalSprites();
    GLUtils.texImage2D(GLES20.GL_TEXTURE_2D, 0, sprites, 0);
    sprites.recycle();
    int[] b = new int[5];
    GLES20.glGenBuffers(5, b, 0);
    ibo = b[0];
    quad = b[1];
    vbo = b[2];
    unit = b[3];
    back = b[4];
    // every quad's two triangles, the same for all: corners 0 1 2 and 2 1 3
    ByteBuffer ib = ByteBuffer.allocateDirect(MAX_QUADS * 12).order(ByteOrder.nativeOrder());
    for (int q = 0; q < MAX_QUADS; q++) {
      int i = 4 * q;
      ib.putShort((short) i).putShort((short) (i + 1)).putShort((short) (i + 2));
      ib.putShort((short) (i + 2)).putShort((short) (i + 1)).putShort((short) (i + 3));
    }
    ib.position(0);
    GLES20.glBindBuffer(GLES20.GL_ELEMENT_ARRAY_BUFFER, ibo);
    GLES20.glBufferData(GLES20.GL_ELEMENT_ARRAY_BUFFER, MAX_QUADS * 12, ib, GLES20.GL_STATIC_DRAW);
    // the grid's quad: the box the view clips its grid to, in the car's frame
    float[] box = {MapView.BOX_X0, MapView.BOX_Z0, MapView.BOX_X1, MapView.BOX_Z0, MapView.BOX_X0, MapView.BOX_Z1,
                   MapView.BOX_X1, MapView.BOX_Z1};
    FloatBuffer fb = ByteBuffer.allocateDirect(box.length * 4).order(ByteOrder.nativeOrder()).asFloatBuffer();
    fb.put(box).position(0);
    GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, quad);
    GLES20.glBufferData(GLES20.GL_ARRAY_BUFFER, box.length * 4, fb, GLES20.GL_STATIC_DRAW);
    // the fades' quad: the unit square, stretched over each rectangle
    fb = ByteBuffer.allocateDirect(32).order(ByteOrder.nativeOrder()).asFloatBuffer();
    fb.put(new float[] {0, 0, 1, 0, 0, 1, 1, 1}).position(0);
    GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, unit);
    GLES20.glBufferData(GLES20.GL_ARRAY_BUFFER, 32, fb, GLES20.GL_STATIC_DRAW);
    int err = GLES20.glGetError();
    if (err != GLES20.GL_NO_ERROR) return "gl error 0x" + Integer.toHexString(err) + " in setup";
    // the close-up look (GlClose): if its program does not build, the map draws without it rather than failing
    close = new GlClose();
    if (!close.setup()) {
      XposedBridge.log("aaroad: close-up program failed; drawing without it");
      close = null;
    }
    GLES20.glGetError();
    return null;
  }

  // A new scene's quads: asks the packer for them, then sends them into back a slice a frame (all at once while there
  // is nothing on screen to keep moving), then draws from back. The scene drawn until then stays glScene, so the frame
  // is made for it. s is the view's newest scene.
  private String stepUpload(MapView.Scene s) {
    if (pend == null) {
      synchronized (packLock) {
        if (packDone != null) {
          pend = packDone;
          packDone = null;
          pendOff = 0;
        } else if (s != null && s != glScene && s != asked && !packBusy) {
          asked = s;
          packAsk = s;
          packBusy = true;
          if (!packStarted) {
            packStarted = true;
            Thread t = new Thread(new Runnable() {
              @Override
              public void run() {
                packLoop();
              }
            }, "aaroad-pack");
            t.setDaemon(true);
            t.start();
          }
          packLock.notifyAll();
        }
      }
      if (pend == null) return null;
      GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, back);
      GLES20.glBufferData(GLES20.GL_ARRAY_BUFFER, Math.max(pend.bytes(), 4), null, GLES20.GL_STATIC_DRAW);
    }
    int size = pend.bytes();
    GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, back);
    do {
      int n = Math.min(SLICE, size - pendOff);
      if (n > 0) {
        bytes.position(pendOff);
        GLES20.glBufferSubData(GLES20.GL_ARRAY_BUFFER, pendOff, n, bytes);
      }
      pendOff += n;
    } while (pendOff < size && cur == null);
    if (pendOff < size) return null;
    bytes.position(0);
    int err = GLES20.glGetError();
    if (err != GLES20.GL_NO_ERROR) return "gl error 0x" + Integer.toHexString(err) + " uploading " + pend.segs + " segments";
    int was = vbo;
    vbo = back;
    back = was;
    // the old scene's storage let go now, not at the next upload; the driver keeps it while a frame still reads it
    GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, back);
    GLES20.glBufferData(GLES20.GL_ARRAY_BUFFER, 4, null, GLES20.GL_STATIC_DRAW);
    cur = pend;
    glScene = (MapView.Scene) pend.scene;
    pend = null;
    synchronized (packLock) {
      packBusy = false;  // bytes is the packer's again
    }
    sceneUploads++;
    checkNext = true;
    return null;
  }

  // The packer: each asked scene's pieces, in the order the view draws them, into bytes. With a route the view draws
  // the roads along it (stubs), without one the roads around the car.
  private void packLoop() {
    float[] tmp = new float[4096 / (4 * GlCull.FLOATS) * 4 * GlCull.FLOATS];
    while (!failed) {
      MapView.Scene s;
      ByteBuffer b;
      synchronized (packLock) {
        while (packAsk == null) {
          try {
            packLock.wait();
          } catch (InterruptedException e) {
            return;
          }
        }
        s = packAsk;
        packAsk = null;
        b = bytes;
      }
      GlCull g = null;
      try {
        MapView.Chunks roads = s.route.length > 0 ? s.stubsChunks : s.roadsChunks;
        MapView.Chunks route = s.routeChunks;
        int[] level = new int[route.cls.length];  // a route piece's cls is its traffic, 0 to 4 (MapView.chunks)
        for (int i = 0; i < level.length; i++) level[i] = route.cls[i] < 2 ? 0 : Math.min(LEVELS - 1, route.cls[i] - 1);
        g = new GlCull(s, ROUTE, LEVELS, roads.pts, roads.box, roads.cls, roads.at, route.pts, route.box, level, route.at);
        int size = g.bytes();
        if (b == null || b.capacity() < size) b = ByteBuffer.allocateDirect(Math.max(size, 1 << 16)).order(ByteOrder.nativeOrder());
        b.position(0);
        g.pack(b, tmp);
        b.position(0);
      } catch (Throwable t) {
        XposedBridge.log("aaroad: scene pack failed: " + t);
        g = null;
      }
      synchronized (packLock) {
        bytes = b;
        if (g != null) packDone = g;
        else packBusy = false;  // the GL thread asks again only for a newer scene
      }
      if (g != null) poke();
    }
  }

  private void lines(int g, float width, int color, float alpha) {
    int[] rFrom = cur.rFrom, rStart = cur.rStart, rLen = cur.rLen;
    if (rFrom[g] == rFrom[g + 1]) return;
    GLES20.glUniform1f(lR, width / 2);
    GLES20.glUniform4f(lColor, (color >> 16 & 0xff) / 255f, (color >> 8 & 0xff) / 255f, (color & 0xff) / 255f, alpha);
    for (int r = rFrom[g]; r < rFrom[g + 1]; r++) {
      for (int st = rStart[r], left = rLen[r]; left > 0; ) {
        int q = Math.min(left, MAX_QUADS);
        GLES20.glVertexAttribPointer(lSeg, 4, GLES20.GL_FLOAT, false, STRIDE, st * 4 * STRIDE);
        GLES20.glVertexAttribPointer(lC, 3, GLES20.GL_FLOAT, false, STRIDE, st * 4 * STRIDE + 16);
        GLES20.glDrawElements(GLES20.GL_TRIANGLES, q * 6, GLES20.GL_UNSIGNED_SHORT, 0);
        st += q;
        left -= q;
      }
    }
  }

  private String drawFrame(MapView.Scene s, MapView.Frame f) {
    long t0 = System.nanoTime();
    float ce = f.ce, cn = f.cn, ch = f.ch, sh = f.sh, ik = f.ik, cut = f.cut, cutE = f.cutE, cutN = f.cutN;
    boolean free = f.free;
    int color = f.color;
    int[] q = surfSize;
    EGL14.eglQuerySurface(dpy, surf, EGL14.EGL_WIDTH, q, 0);
    EGL14.eglQuerySurface(dpy, surf, EGL14.EGL_HEIGHT, q, 1);
    int w = q[0], h = q[1];
    if (w <= 0 || h <= 0) return null;
    GLES20.glViewport(0, 0, w, h);
    GLES20.glClearColor((BG >> 16 & 0xff) / 255f, (BG >> 8 & 0xff) / 255f, (BG & 0xff) / 255f, 1);
    GLES20.glClear(GLES20.GL_COLOR_BUFFER_BIT);
    GLES20.glEnable(GLES20.GL_BLEND);
    // the colours blend, the alpha stays 1: the window composites the masks over an opaque layer
    GLES20.glBlendFuncSeparate(GLES20.GL_SRC_ALPHA, GLES20.GL_ONE_MINUS_SRC_ALPHA, GLES20.GL_ZERO, GLES20.GL_ONE);
    projection(w, h, free ? MapView.FREE_X - MapView.CAR_X : 0, f.lift);
    float qa = 1 / (v[4] - v[5] * v[7]);
    part(P_CLEAR);

    // the grid, every GRID_DM display metres, fixed to the ground
    double k = 1.0 / ik, step = MapView.GRID_DM * k;
    double absE = s.ax * s.mpt + ce, absN = -s.ay * s.mpt + cn;
    float phE = (float) ((absE % step + step) % step / k), phN = (float) ((absN % step + step) % step / k);
    GLES20.glUseProgram(grid);
    GLES20.glUniformMatrix4fv(gP, 1, false, proj, 0);
    GLES20.glUniform4f(gV, v[0], v[1], v[2], 0);
    GLES20.glUniform4f(gRot, ch, sh, phE, phN);
    GLES20.glUniform4f(gCam, v[0], v[1], v[7], qa);
    GLES20.glUniform4f(gColor, (GRID >> 16 & 0xff) / 255f, (GRID >> 8 & 0xff) / 255f, (GRID & 0xff) / 255f, 1);
    GLES20.glUniform1f(gStep, MapView.GRID_DM);
    GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, quad);
    GLES20.glEnableVertexAttribArray(gA);
    GLES20.glVertexAttribPointer(gA, 2, GLES20.GL_FLOAT, false, 8, 0);
    GLES20.glDrawArrays(GLES20.GL_TRIANGLE_STRIP, 0, 4);
    GLES20.glDisableVertexAttribArray(gA);
    part(P_GRID);
    // zoomed in: the rail lines, under the roads (CloseView.rails)
    if (close != null) {
      GLES20.glBindBuffer(GLES20.GL_ELEMENT_ARRAY_BUFFER, ibo);
      close.rails(s, f, proj, v[0], v[1], v[2], v[7], qa, CloseUp.gate(MapView.R0 / ik, CloseUp.ROAD_SHOW, CloseUp.ROAD_HIDE));
    }

    // the roads: every kerb, then every surface, then the carpet, as the view strokes them
    cur.cull(ce, cn, ch, sh, ik, MapView.BOX_X0, MapView.BOX_X1, MapView.BOX_Z0, MapView.BOX_Z1);
    part(P_CULL);
    GLES20.glUseProgram(line);
    GLES20.glUniformMatrix4fv(lP, 1, false, proj, 0);
    GLES20.glUniform4f(lW, ch * ik, sh * ik, (-ce * ch + cn * sh) * ik, (-ce * sh - cn * ch) * ik);
    GLES20.glUniform4f(lV, v[0], v[1], v[2], 0);
    GLES20.glUniform4f(lCam, v[0], v[1], v[7], qa);
    float cx = ((cutE - ce) * ch - (cutN - cn) * sh) * ik, cz = ((cutE - ce) * sh + (cutN - cn) * ch) * ik;
    GLES20.glUniform4f(lCut, Float.isNaN(cut) ? 0 : cut, ik, cx, cz);
    GLES20.glUniform1f(lCarpet, 0);
    GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, vbo);
    GLES20.glBindBuffer(GLES20.GL_ELEMENT_ARRAY_BUFFER, ibo);
    GLES20.glEnableVertexAttribArray(lSeg);
    GLES20.glEnableVertexAttribArray(lC);
    float floor = MapView.dm(MapView.MIN_ROAD_PX), kerb = MapView.dm(2 * MapView.KERB_PX);
    float route = Math.max(floor, MapView.ROUTE_M * ik);
    // by class colour (MapView.KERBS/ROADS); the route's own grey road under the tile roads' surfaces
    for (int c : MapView.DRAW_ORDER) lines(c, Math.max(floor, MapView.ROAD_M[c] * ik) + kerb, MapView.KERBS[c], 1);
    for (int l = 0; l < LEVELS; l++) lines(ROUTE + l, route + kerb, KERB, 1);
    for (int l = 0; l < LEVELS; l++) lines(ROUTE + l, route, ROAD, 1);
    for (int c : MapView.DRAW_ORDER) lines(c, Math.max(floor, MapView.ROAD_M[c] * ik), MapView.ROADS[c], 1);
    // the bridges over all of that, as the view: casing, kerb, surface
    float casing = MapView.dm(2 * MapView.KERB_PX + 2 * MapView.CASING_PX);
    int b = MapView.BRIDGE;
    for (int c : MapView.DRAW_ORDER) lines(b + c, Math.max(floor, MapView.ROAD_M[c] * ik) + casing, MapView.CASING_C, 1);
    for (int c : MapView.DRAW_ORDER) lines(b + c, Math.max(floor, MapView.ROAD_M[c] * ik) + kerb, MapView.KERBS[c], 1);
    for (int c : MapView.DRAW_ORDER) lines(b + c, Math.max(floor, MapView.ROAD_M[c] * ik), MapView.ROADS[c], 1);
    // zoomed in: the raised decks, lane lines, sidewalks and crosswalks, as the view's CloseView, then the carpet over
    if (close != null) {
      double rng = MapView.R0 / ik;
      float gr = CloseUp.gate(rng, CloseUp.ROAD_SHOW, CloseUp.ROAD_HIDE), gw = CloseUp.gate(rng, CloseUp.WALK_SHOW, CloseUp.WALK_HIDE);
      close.step(s);
      if (gr > 0 || gw > 0) {
        GLES20.glDisableVertexAttribArray(lSeg);
        GLES20.glDisableVertexAttribArray(lC);
        close.draw(s, f, proj, v[0], v[1], v[2], v[7], qa, gr, gw);
        GLES20.glUseProgram(line);
        GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, vbo);
        GLES20.glEnableVertexAttribArray(lSeg);
        GLES20.glEnableVertexAttribArray(lC);
      }
    }
    part(P_ROADS);
    if (!Float.isNaN(cut)) {
      GLES20.glUniform1f(lCarpet, 1);
      float wide = Math.max(MapView.dm(MapView.LINE_MIN_PX), Math.min(MapView.dm(MapView.LINE_MAX_PX), MapView.LINE_M * ik));
      for (int l = 0; l < LEVELS; l++) lines(ROUTE + l, wide, l == 0 ? color : MapView.TRAFFIC[l - 1], 242 / 255f);
    }
    GLES20.glDisableVertexAttribArray(lSeg);
    GLES20.glDisableVertexAttribArray(lC);
    part(P_CARPET);

    // the fades over the roads, as the view's mask and fog: the band behind the cards with a route, the fog at the top
    GLES20.glUseProgram(fade);
    GLES20.glUniform2f(fSize, w, h);
    GLES20.glUniform3f(fRgb, (BG >> 16 & 0xff) / 255f, (BG >> 8 & 0xff) / 255f, (BG & 0xff) / 255f);
    GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, unit);
    GLES20.glEnableVertexAttribArray(fP);
    GLES20.glVertexAttribPointer(fP, 2, GLES20.GL_FLOAT, false, 8, 0);
    if (!free) fade(0, 0, MapView.COL, 215, 0, 1, 1, 0, (MapView.CARD_R + 4) / MapView.COL);
    fade(0, 0, 584, 78, 1, 1, 0xe6 / 255f, 0, 0.6f);
    GLES20.glDisableVertexAttribArray(fP);
    part(P_FADE);
    signals(f, w, h);  // under the names, as the view draws them
    labels(f, w, h);
    part(P_LABELS);

    if (checkNext) {
      // the first frame on a surface or with a scene is checked; later ones are trusted
      int err = GLES20.glGetError();
      if (err != GLES20.GL_NO_ERROR) return "gl error 0x" + Integer.toHexString(err) + " drawing";
    }
    if (!EGL14.eglSwapBuffers(dpy, surf)) {
      int err = EGL14.eglGetError();
      synchronized (lock) {
        if (holder != glHolder) return null;  // the surface is going away; the next turn of the loop lets it go
      }
      return "swap failed, egl error 0x" + Integer.toHexString(err);
    }
    long ns = System.nanoTime() - t0;
    part(P_SWAP);
    synchronized (lock) {
      part(P_LOCK);
      glSum += ns;
      glCount++;
    }
    checkNext = false;
    if (!ready) {
      ready = true;
      if (!loggedOn) {
        loggedOn = true;
        MapView.say("aaroad: gpu roads on, " + GLES20.glGetString(GLES20.GL_RENDERER) + ", " + w + "x" + h
            + ", first frame " + Math.round(ns / 1e5) / 10.0 + " ms");
      }
      invalidateMap();  // the view stops drawing the roads from its next frame
    }
    return null;
  }

  // The view's chosen labels where this frame puts them: each left out while it would cover a card, the car, the sign,
  // the tag or an edge, as the view decides. The texture holds premultiplied colour, blended as such.
  private void labels(MapView.Frame f, int w, int h) {
    MapView.Labels set = map.labels;
    int n = set == null || set.s != f.s ? 0 : Math.min(set.count, atlasText.length);  // none placed from an old anchor
    if (n == 0) return;
    // every row, so a name left in a row the last redraw erased is not taken as still there
    boolean same = true;
    for (int i = 0; i < atlasText.length && same; i++) same = i < n ? set.text[i].equals(atlasText[i]) : atlasText[i] == null;
    GLES20.glBindTexture(GLES20.GL_TEXTURE_2D, tex);
    if (!same) {
      part(P_LABELS);
      if (atlas == null) {
        atlas = Bitmap.createBitmap(ATLAS_W, ATLAS_H, Bitmap.Config.ARGB_8888);
        atlasCanvas = new Canvas(atlas);
        paints = MapView.labelPaints();
      }
      atlas.eraseColor(0);
      for (int i = 0; i < n; i++) {
        // a route number ("US 75") as the overlay draws it, a white badge (MapView.drawLabel)
        MapView.drawLabel(atlasCanvas, set.text[i], Shield.ref(set.text[i]) != null, PAD, i * ROW + BASE, paints);
        atlasText[i] = set.text[i];
      }
      for (int i = n; i < atlasText.length; i++) atlasText[i] = null;
      GLUtils.texImage2D(GLES20.GL_TEXTURE_2D, 0, atlas, 0);
      atlasUploads++;
      part(P_ATLAS);  // the names drawn into the bitmap and the bitmap sent to GL
    }
    float off = f.free ? MapView.FREE_X - MapView.CAR_X : 0;
    float[] tag = map.tagShown;
    GLES20.glUseProgram(text);
    GLES20.glUniform2f(tSize, w, h);
    GLES20.glUniform1i(tTex, 0);
    GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, unit);
    GLES20.glEnableVertexAttribArray(tP);
    GLES20.glVertexAttribPointer(tP, 2, GLES20.GL_FLOAT, false, 8, 0);
    GLES20.glBlendFuncSeparate(GLES20.GL_ONE, GLES20.GL_ONE_MINUS_SRC_ALPHA, GLES20.GL_ZERO, GLES20.GL_ONE);
    for (int i = 0; i < n; i++) {
      float de = set.e[i] - f.ce, dn = set.n[i] - f.cn;
      float x = (de * f.ch - dn * f.sh) * f.ik, z = (de * f.sh + dn * f.ch) * f.ik;
      if (!map.project(x, z, off, lq) || map.blocked(lq[0], lq[1], set.w[i], f.free, tag)) continue;
      // where the view's centred text would start and sit, at whole pixels
      float x0 = Math.round(lq[0] - set.w[i] / 2) - PAD, y0 = Math.round(lq[1] + 4) - BASE;
      GLES20.glUniform4f(tRect, x0, y0, x0 + ATLAS_W, y0 + ROW);
      GLES20.glUniform4f(tUV, 0, i * ROW / (float) ATLAS_H, 1, (i + 1) * ROW / (float) ATLAS_H);
      GLES20.glDrawArrays(GLES20.GL_TRIANGLE_STRIP, 0, 4);
    }
    GLES20.glDisableVertexAttribArray(tP);
    GLES20.glBlendFuncSeparate(GLES20.GL_SRC_ALPHA, GLES20.GL_ONE_MINUS_SRC_ALPHA, GLES20.GL_ZERO, GLES20.GL_ONE);
  }

  // The four sprites side by side, each centred in its CELL square as the view draws an icon at its whole pixel.
  static Bitmap signalSprites() {
    int c = SignalIcon.CELL;
    Bitmap b = Bitmap.createBitmap(c * SignalIcon.SPRITES, c, Bitmap.Config.ARGB_8888);
    b.eraseColor(0);
    Canvas k = new Canvas(b);
    android.graphics.Paint p = new android.graphics.Paint(android.graphics.Paint.ANTI_ALIAS_FLAG);
    android.graphics.Path path = new android.graphics.Path();
    android.graphics.RectF r = new android.graphics.RectF(0, 0, 0, 0);
    for (int i = 0; i < SignalIcon.SPRITES; i++) SignalIcon.draw(k, i, i * c + c / 2, c / 2, p, path, r);
    return b;
  }

  // The signal icons the view would draw this frame, placed as it places them (MapView.placeSignals), as quads of
  // their sprites with the labels' program and premultiplied blend.
  private void signals(MapView.Frame f, int w, int h) {
    MapView.Scene s = f.s;
    if (s == null || s.signals.length == 0) return;
    int n = map.placeSignals(s, f.ce, f.cn, f.ch, f.sh, f.ik, f.free, map.tagShown, sigChoice, sigQ, sigXY, sigSprite);
    if (n == 0) return;
    int c = SignalIcon.CELL;
    GLES20.glUseProgram(text);
    GLES20.glUniform2f(tSize, w, h);
    GLES20.glUniform1i(tTex, 0);
    GLES20.glBindTexture(GLES20.GL_TEXTURE_2D, sigTex);
    GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, unit);
    GLES20.glEnableVertexAttribArray(tP);
    GLES20.glVertexAttribPointer(tP, 2, GLES20.GL_FLOAT, false, 8, 0);
    GLES20.glBlendFuncSeparate(GLES20.GL_ONE, GLES20.GL_ONE_MINUS_SRC_ALPHA, GLES20.GL_ZERO, GLES20.GL_ONE);
    for (int i = 0; i < n; i++) {
      float x0 = sigXY[2 * i] - c / 2, y0 = sigXY[2 * i + 1] - c / 2;
      GLES20.glUniform4f(tRect, x0, y0, x0 + c, y0 + c);
      GLES20.glUniform4f(tUV, sigSprite[i] / (float) SignalIcon.SPRITES, 0, (sigSprite[i] + 1) / (float) SignalIcon.SPRITES, 1);
      GLES20.glDrawArrays(GLES20.GL_TRIANGLE_STRIP, 0, 4);
    }
    GLES20.glDisableVertexAttribArray(tP);
    GLES20.glBlendFuncSeparate(GLES20.GL_SRC_ALPHA, GLES20.GL_ONE_MINUS_SRC_ALPHA, GLES20.GL_ZERO, GLES20.GL_ONE);
  }

  // The time since the last mark goes to stage i.
  private void part(int i) {
    long t = System.nanoTime();
    partNow[i] += t - partMark;
    partMark = t;
  }

  // One counted frame into the sums; one over SLOW_NS, sleep included as the gaps include it, notes its largest stage.
  private void addParts() {
    long total = 0;
    int top = 0;
    for (int i = 0; i < PARTS; i++) {
      long t = partNow[i];
      partSum[i] += t;
      if (t > partMax[i]) partMax[i] = t;
      if (t > partNow[top]) top = i;
      total += t;
    }
    partFrames++;
    if (total > SLOW_NS) {
      slowFrames++;
      slowTop[top]++;
    }
  }

  // Every PARTS_MS, one line if any moving frames were counted; the window starts over either way.
  private void logParts(long now) {
    if (partsFrom == 0) partsFrom = now;
    if (now - partsFrom < PARTS_MS) return;
    if (partFrames > 0) {
      StringBuilder b = new StringBuilder("aaroad: gl parts avg/max ms: ");
      for (int i = 0; i < PARTS; i++) {
        if (i > 0) b.append(", ");
        b.append(PART_NAMES[i]).append(' ').append(ms(partSum[i] / partFrames)).append('/').append(ms(partMax[i]));
      }
      b.append("; ").append(partFrames).append(" frames; label uploads ").append(atlasUploads)
          .append("; scene uploads ").append(sceneUploads).append("; slow frames ").append(slowFrames);
      if (slowFrames > 0) {
        b.append(" (top stage");
        for (int i = 0; i < PARTS; i++) if (slowTop[i] > 0) b.append(' ').append(PART_NAMES[i]).append(' ').append(slowTop[i]);
        b.append(')');
      }
      MapView.say(b.toString());
    }
    java.util.Arrays.fill(partSum, 0);
    java.util.Arrays.fill(partMax, 0);
    java.util.Arrays.fill(slowTop, 0);
    partFrames = slowFrames = atlasUploads = sceneUploads = 0;
    partsFrom = now;
  }

  private static String ms(long ns) {
    return String.valueOf(Math.round(ns / 1e5) / 10.0);
  }

  private void fade(float x0, float y0, float x1, float y1, int axis, float a0, float a1, float a2, float stop) {
    GLES20.glUniform4f(fRect, x0, y0, x1, y1);
    GLES20.glUniform1f(fAxis, axis);
    GLES20.glUniform4f(fA, a0, a1, a2, stop);
    GLES20.glDrawArrays(GLES20.GL_TRIANGLE_STRIP, 0, 4);
  }

  // MapView's camera as a projection: car frame (x, z, 0, 1) to clip space for a w x h surface, the screen shifted
  // right by tx and up by lift (the lane row under the car, MapView.LIFT). Clip w is the camera's w, and clip z is
  // w - 2 NEAR_W, so the near plane drops w < NEAR_W.
  private void projection(int w, int h, float tx, float lift) {
    java.util.Arrays.fill(proj, 0);
    proj[0] = 2 * v[0] / w;
    proj[4] = 2 * v[1] / w + 2 * tx * v[7] / w - v[7];
    proj[5] = v[7] - 2 * v[4] / h;
    proj[6] = v[7];
    proj[7] = v[7];
    proj[12] = 2 * v[2] / w + 2 * tx / w - 1;
    proj[13] = 1 - 2 * v[5] / h;
    // up by lift px: clip y gains 2 lift / h times clip w (v[7] z + 1)
    proj[5] += 2 * lift * v[7] / h;
    proj[13] += 2 * lift / h;
    proj[14] = 1 - 2 * NEAR_W;
    proj[15] = 1;
  }

  private void invalidateMap() {
    map.postInvalidate();
  }

  private void fail(String why) {
    if (failed) return;
    failed = true;
    ready = false;
    MapView.say("aaroad: gpu roads off: " + why);
    try {
      if (dpy != EGL14.EGL_NO_DISPLAY) {
        EGL14.eglMakeCurrent(dpy, EGL14.EGL_NO_SURFACE, EGL14.EGL_NO_SURFACE, EGL14.EGL_NO_CONTEXT);
        if (surf != EGL14.EGL_NO_SURFACE) EGL14.eglDestroySurface(dpy, surf);
        if (ctx != EGL14.EGL_NO_CONTEXT) EGL14.eglDestroyContext(dpy, ctx);
        surf = EGL14.EGL_NO_SURFACE;
        ctx = EGL14.EGL_NO_CONTEXT;
      }
    } catch (Throwable ignored) {
    }
    synchronized (lock) {
      glHolder = null;
      lock.notifyAll();
    }
    post(new Runnable() {
      @Override
      public void run() {
        setVisibility(GONE);
        if (map.gpu == GlRoads.this) map.gpu = null;  // the view draws everything itself from now on
        map.invalidate();
      }
    });
  }
}
