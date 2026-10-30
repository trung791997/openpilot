package com.starpilot.aaroad;

import android.graphics.PixelFormat;
import android.opengl.EGL14;
import android.opengl.EGLConfig;
import android.opengl.EGLContext;
import android.opengl.EGLDisplay;
import android.opengl.EGLSurface;
import android.opengl.GLES20;
import android.opengl.Matrix;
import android.os.SystemClock;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.ViewGroup;
import android.widget.FrameLayout;

import java.io.File;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;

import de.robv.android.xposed.XposedBridge;

// A visible test of the 3D turn close-up (Peter 2026-10-30: "Yeah let's try that"). The GPU test drew to the dash
// through its own layer in about 22 ms a frame, but drew nothing visible, so it did not show the layer actually
// appears. This draws a made-up 3D road with a right turn in the map's spot, x 300..540 (inside the tach ring),
// with the camera driving through it, for 20 s and then takes it away. It waits 20 s after the map comes up, runs
// only while the car stands still, and runs once: the marker is written first, so a crash does not repeat it.
final class Gl3d {
  private static final String MARKER = "aaroad_gl3d_test_1";
  private static final int X = 300, W = 240, H = 215;
  private static final long DELAY_MS = 20000, WAIT_MS = 300000, RUN_MS = 20000, FRAME_MS = 33;
  private static boolean started;

  static synchronized void testOnce(final ViewGroup root) {
    if (started) return;
    started = true;
    new Thread(new Runnable() {
      @Override
      public void run() {
        try {
          Thread.sleep(DELAY_MS);
          String proc = GlProbe.processName();
          if (proc == null) return;
          long until = SystemClock.uptimeMillis() + WAIT_MS;
          while (Feed.spd >= 0.3) {
            if (SystemClock.uptimeMillis() > until) {
              XposedBridge.log("aaroad: gl 3d test skipped, the car kept moving");
              return;
            }
            Thread.sleep(1000);
          }
          File marker = new File("/data/data/" + proc + "/cache", MARKER);
          if (!marker.createNewFile()) return;
          show(root);
        } catch (Throwable t) {
          XposedBridge.log("aaroad: gl 3d test failed: " + t);
        }
      }
    }, "aaroad-gl3d").start();
  }

  private static void show(final ViewGroup root) {
    root.post(new Runnable() {
      @Override
      public void run() {
        final FrameLayout box = new FrameLayout(root.getContext());
        final SurfaceView sv = new SurfaceView(root.getContext());
        sv.setZOrderOnTop(true);
        sv.getHolder().setFormat(PixelFormat.TRANSLUCENT);
        sv.getHolder().addCallback(new SurfaceHolder.Callback() {
          boolean ran;
          @Override
          public void surfaceCreated(final SurfaceHolder holder) {
            if (ran) return;
            ran = true;
            new Thread(new Runnable() {
              @Override
              public void run() {
                String r;
                try {
                  r = render(holder);
                } catch (Throwable t) {
                  r = "failed: " + t;
                }
                XposedBridge.log("aaroad: gl 3d test " + r);
                root.post(new Runnable() {
                  @Override
                  public void run() {
                    root.removeView(box);
                  }
                });
              }
            }, "aaroad-gl3d-draw").start();
          }
          @Override
          public void surfaceChanged(SurfaceHolder holder, int format, int w, int h) {}
          @Override
          public void surfaceDestroyed(SurfaceHolder holder) {}
        });
        FrameLayout.LayoutParams lp = new FrameLayout.LayoutParams(W, H);
        lp.leftMargin = X;
        box.addView(sv, lp);
        root.addView(box, new ViewGroup.LayoutParams(ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.MATCH_PARENT));
        XposedBridge.log("aaroad: gl 3d test showing");
      }
    });
  }

  private static String render(SurfaceHolder window) throws InterruptedException {
    EGLDisplay dpy = EGL14.eglGetDisplay(EGL14.EGL_DEFAULT_DISPLAY);
    int[] ver = new int[2];
    if (!EGL14.eglInitialize(dpy, ver, 0, ver, 1)) return "no EGL display";
    EGLSurface surf = EGL14.EGL_NO_SURFACE;
    EGLContext ctx = EGL14.EGL_NO_CONTEXT;
    try {
      int[] attrs = {EGL14.EGL_RENDERABLE_TYPE, EGL14.EGL_OPENGL_ES2_BIT, EGL14.EGL_SURFACE_TYPE, EGL14.EGL_WINDOW_BIT,
                     EGL14.EGL_RED_SIZE, 8, EGL14.EGL_GREEN_SIZE, 8, EGL14.EGL_BLUE_SIZE, 8, EGL14.EGL_ALPHA_SIZE, 8,
                     EGL14.EGL_DEPTH_SIZE, 16, EGL14.EGL_NONE};
      EGLConfig[] cfg = new EGLConfig[1];
      int[] n = new int[1];
      if (!EGL14.eglChooseConfig(dpy, attrs, 0, cfg, 0, 1, n, 0) || n[0] < 1) return "no window config";
      ctx = EGL14.eglCreateContext(dpy, cfg[0], EGL14.EGL_NO_CONTEXT, new int[]{EGL14.EGL_CONTEXT_CLIENT_VERSION, 2, EGL14.EGL_NONE}, 0);
      if (ctx == null || ctx == EGL14.EGL_NO_CONTEXT) return "no context";
      surf = EGL14.eglCreateWindowSurface(dpy, cfg[0], window, new int[]{EGL14.EGL_NONE}, 0);
      if (surf == null || surf == EGL14.EGL_NO_SURFACE) return "no window surface";
      if (!EGL14.eglMakeCurrent(dpy, surf, surf, ctx)) return "makeCurrent failed";
      return drive(dpy, surf);
    } finally {
      EGL14.eglMakeCurrent(dpy, EGL14.EGL_NO_SURFACE, EGL14.EGL_NO_SURFACE, EGL14.EGL_NO_CONTEXT);
      if (surf != EGL14.EGL_NO_SURFACE) EGL14.eglDestroySurface(dpy, surf);
      if (ctx != EGL14.EGL_NO_CONTEXT) EGL14.eglDestroyContext(dpy, ctx);
      EGL14.eglTerminate(dpy);
    }
  }

  // ---- the made-up road: 60 m straight ahead, a right turn of 15 m radius, 60 m straight to the right.
  // World metres: x right, y up, z ahead.

  private static final float STEP = 1, LANE_HALF = 3.5f;

  private static float[] centreline() {
    int straight = 60, arc = 24;
    float[] c = new float[(straight + arc + straight + 1) * 2];
    int k = 0;
    for (int i = 0; i < straight; i++) { c[k++] = 0; c[k++] = i * STEP; }
    for (int i = 0; i < arc; i++) {
      double a = Math.PI - i * (Math.PI / 2) / arc;
      c[k++] = (float) (15 + 15 * Math.cos(a));
      c[k++] = (float) (60 + 15 * Math.sin(a));
    }
    for (int i = 0; i <= straight; i++) { c[k++] = 15 + i * STEP; c[k++] = 75; }
    // lookAt is right-handed: looking along +z puts +x on the screen's left, so store x mirrored to keep the turn a right turn
    for (int i = 0; i < c.length; i += 2) c[i] = -c[i];
    return c;
  }

  // a flat strip from offset a to b (metres right of the centreline) at height y, as a triangle strip
  private static FloatBuffer strip(float[] c, float a, float b, float y) {
    int n = c.length / 2;
    float[] v = new float[n * 6];
    for (int i = 0; i < n; i++) {
      int j = Math.min(i + 1, n - 1), h = Math.max(i - 1, 0);
      float tx = c[j * 2] - c[h * 2], tz = c[j * 2 + 1] - c[h * 2 + 1], len = (float) Math.hypot(tx, tz);
      float rx = tz / len, rz = -tx / len;
      v[i * 6] = c[i * 2] + rx * a; v[i * 6 + 1] = y; v[i * 6 + 2] = c[i * 2 + 1] + rz * a;
      v[i * 6 + 3] = c[i * 2] + rx * b; v[i * 6 + 4] = y; v[i * 6 + 5] = c[i * 2 + 1] + rz * b;
    }
    return buf(v);
  }

  private static FloatBuffer grid() {
    int lines = 2 * 21;
    float[] v = new float[lines * 6];
    int k = 0;
    for (int i = -10; i <= 10; i++) {
      float p = i * 15;
      float[] a = {p - 20, 0, -60, p - 20, 0, 240, 130, 0, p + 90, -170, 0, p + 90};
      System.arraycopy(a, 0, v, k, 12);
      k += 12;
    }
    return buf(v);
  }

  private static FloatBuffer buf(float[] v) {
    FloatBuffer fb = ByteBuffer.allocateDirect(v.length * 4).order(ByteOrder.nativeOrder()).asFloatBuffer();
    fb.put(v).position(0);
    return fb;
  }

  private static String drive(EGLDisplay dpy, EGLSurface surf) throws InterruptedException {
    int prog = GlProbe.program(
        "uniform mat4 m; attribute vec3 p; varying float d;\n"
        + "void main() { vec4 q = m * vec4(p, 1.0); d = q.w; gl_Position = q; }",
        "precision mediump float; uniform vec3 c; varying float d;\n"
        + "void main() { float f = clamp((d - 50.0) / 110.0, 0.0, 1.0); gl_FragColor = vec4(mix(c, vec3(0.02), f), 1.0); }");
    if (prog == 0) return "shader compile failed";
    float[] c = centreline();
    FloatBuffer road = strip(c, -LANE_HALF, LANE_HALF, 0), left = strip(c, -LANE_HALF, -LANE_HALF + 0.3f, 0.02f),
        right = strip(c, LANE_HALF - 0.3f, LANE_HALF, 0.02f), route = strip(c, -1.1f, 1.1f, 0.04f), grid = grid();
    int n = c.length / 2, gridN = grid.capacity() / 3;
    FloatBuffer car = ByteBuffer.allocateDirect(9 * 4).order(ByteOrder.nativeOrder()).asFloatBuffer();

    GLES20.glUseProgram(prog);
    int p = GLES20.glGetAttribLocation(prog, "p"), mu = GLES20.glGetUniformLocation(prog, "m"), cu = GLES20.glGetUniformLocation(prog, "c");
    GLES20.glEnableVertexAttribArray(p);
    GLES20.glViewport(0, 0, W, H);
    GLES20.glLineWidth(1.5f);
    float[] proj = new float[16], view = new float[16], mvp = new float[16];
    Matrix.perspectiveM(proj, 0, 42, (float) W / H, 1, 300);

    long start = SystemClock.uptimeMillis(), sum = 0, max = 0;
    int frames = 0;
    float speed = 9, span = n - 25;  // m/s, about 20 mph; loops before the end of the road
    while (SystemClock.uptimeMillis() - start < RUN_MS) {
      long a = SystemClock.uptimeMillis();
      float s = ((a - start) / 1000f * speed) % span;
      int i = (int) s, j = Math.min(i + 3, n - 1);
      float f = s - i, px = c[i * 2] + f * (c[(i + 1) * 2] - c[i * 2]), pz = c[i * 2 + 1] + f * (c[(i + 1) * 2 + 1] - c[i * 2 + 1]);
      float tx = c[j * 2] - c[i * 2], tz = c[j * 2 + 1] - c[i * 2 + 1], len = (float) Math.hypot(tx, tz);
      tx /= len; tz /= len;
      Matrix.setLookAtM(view, 0, px - tx * 16, 8, pz - tz * 16, px + tx * 14, 0, pz + tz * 14, 0, 1, 0);
      Matrix.multiplyMM(mvp, 0, proj, 0, view, 0);
      GLES20.glUniformMatrix4fv(mu, 1, false, mvp, 0);

      GLES20.glClearColor(0.02f, 0.02f, 0.02f, 1);
      GLES20.glClear(GLES20.GL_COLOR_BUFFER_BIT);
      draw(p, cu, grid, GLES20.GL_LINES, gridN, 0.16f, 0.16f, 0.17f);
      draw(p, cu, road, GLES20.GL_TRIANGLE_STRIP, n * 2, 0.30f, 0.31f, 0.33f);
      draw(p, cu, left, GLES20.GL_TRIANGLE_STRIP, n * 2, 0.85f, 0.85f, 0.85f);
      draw(p, cu, right, GLES20.GL_TRIANGLE_STRIP, n * 2, 0.85f, 0.85f, 0.85f);
      draw(p, cu, route, GLES20.GL_TRIANGLE_STRIP, n * 2, 1.0f, 0.55f, 0.1f);
      float rx = tz, rz = -tx;
      car.position(0);
      car.put(new float[]{px + tx * 2.2f, 0.06f, pz + tz * 2.2f, px - tx * 1.6f + rx * 1.3f, 0.06f, pz - tz * 1.6f + rz * 1.3f,
                          px - tx * 1.6f - rx * 1.3f, 0.06f, pz - tz * 1.6f - rz * 1.3f}).position(0);
      draw(p, cu, car, GLES20.GL_TRIANGLES, 3, 0.25f, 0.55f, 1.0f);
      EGL14.eglSwapBuffers(dpy, surf);

      long ms = SystemClock.uptimeMillis() - a;
      sum += ms;
      max = Math.max(max, ms);
      frames++;
      if (ms < FRAME_MS) Thread.sleep(FRAME_MS - ms);
    }
    int err = GLES20.glGetError();
    GLES20.glDeleteProgram(prog);
    return frames + " frames in " + RUN_MS / 1000 + " s, " + (frames > 0 ? sum / frames : 0) + " ms avg " + max + " max, glError " + err;
  }

  private static void draw(int p, int cu, FloatBuffer v, int mode, int count, float r, float g, float b) {
    GLES20.glUniform3f(cu, r, g, b);
    GLES20.glVertexAttribPointer(p, 3, GLES20.GL_FLOAT, false, 12, v);
    GLES20.glDrawArrays(mode, 0, count);
  }

  private Gl3d() {}
}
