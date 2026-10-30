package com.starpilot.aaroad;

import android.app.AndroidAppHelper;
import android.app.Application;
import android.content.Context;
import android.opengl.EGL14;
import android.opengl.EGLConfig;
import android.opengl.EGLContext;
import android.opengl.EGLDisplay;
import android.opengl.EGLSurface;
import android.opengl.GLES20;
import android.os.SystemClock;

import java.io.File;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;

import de.robv.android.xposed.XposedBridge;

// One-time check whether the head unit can draw the 3D close-up (Peter 2026-10-29: 3D for the zoomed-in view
// near a turn only, 2D while cruising). Off screen, on its own thread, nothing shown: a GLES2 context on a
// 584x215 pbuffer draws a 15k-triangle mesh, about the size of the mockup close-ups, and logs the frame and
// read-back times. It runs once per marker version: the marker is written before the first GL call, so if the
// Tegra driver takes the cluster process down, the probe does not run again when it restarts.
final class GlProbe {
  private static final String MARKER = "aaroad_glprobe_2";
  private static final int W = 584, H = 215, GRID = 87, FRAMES = 20;  // 2 * 86 * 86 = 14792 triangles
  private static final long DELAY_MS = 30000;

  private static boolean started;

  static synchronized void startOnce(final Context ctx) {
    if (started) return;
    started = true;
    Thread t = new Thread(new Runnable() {
      @Override
      public void run() {
        try {
          Thread.sleep(DELAY_MS);
          // the meter view carries Honda Hack's context, whose folders this process cannot write (EACCES on the
          // first run, 2026-10-29), and the cluster app's cache folder failed the same way on the second; the
          // marker goes in the first of these this process can write, and the probe stays off if none
          Application app = AndroidAppHelper.currentApplication();
          Context own = app != null ? app : ctx;
          // the process is Mitsubishi's external-display app (uid 10056, 2026-10-30) with no Application to ask, so
          // its own folder comes from the process name
          String proc = processName();
          File[] dirs = {proc != null ? new File("/data/data/" + proc + "/cache") : null,
                         proc != null ? new File("/data/data/" + proc) : null,
                         own.getCacheDir(), own.getFilesDir(), ctx.getCacheDir(), new File("/data/local/tmp")};
          String tried = "uid " + android.os.Process.myUid() + " app " + (app != null ? app.getPackageName() : "none");
          File marker = null;
          for (File d : dirs) {
            if (d == null) continue;
            File f = new File(d, MARKER);
            if (f.exists()) return;
            try {
              if (f.createNewFile()) { marker = f; break; }
            } catch (java.io.IOException e) { /* next folder */ }
            tried += ", " + d;
          }
          if (marker == null) {
            XposedBridge.log("aaroad: gl probe skipped, nowhere to keep its marker (" + tried + ")");
            return;
          }
          XposedBridge.log("aaroad: gl probe marker " + marker);
          XposedBridge.log("aaroad: gl probe " + probe());
        } catch (Throwable t) {
          XposedBridge.log("aaroad: gl probe failed: " + t);
        }
      }
    }, "aaroad-glprobe");
    t.setPriority(Thread.MIN_PRIORITY);
    t.start();
  }

  private static String probe() {
    EGLDisplay dpy = EGL14.eglGetDisplay(EGL14.EGL_DEFAULT_DISPLAY);
    int[] ver = new int[2];
    if (!EGL14.eglInitialize(dpy, ver, 0, ver, 1)) return "no EGL display";
    EGLSurface surf = EGL14.EGL_NO_SURFACE;
    EGLContext ctx = EGL14.EGL_NO_CONTEXT;
    try {
      int[] attrs = {EGL14.EGL_RENDERABLE_TYPE, EGL14.EGL_OPENGL_ES2_BIT, EGL14.EGL_SURFACE_TYPE, EGL14.EGL_PBUFFER_BIT,
                     EGL14.EGL_RED_SIZE, 8, EGL14.EGL_GREEN_SIZE, 8, EGL14.EGL_BLUE_SIZE, 8, EGL14.EGL_DEPTH_SIZE, 16,
                     EGL14.EGL_NONE};
      EGLConfig[] cfg = new EGLConfig[1];
      int[] n = new int[1];
      if (!EGL14.eglChooseConfig(dpy, attrs, 0, cfg, 0, 1, n, 0) || n[0] < 1) return "no GLES2 pbuffer config";
      ctx = EGL14.eglCreateContext(dpy, cfg[0], EGL14.EGL_NO_CONTEXT, new int[]{EGL14.EGL_CONTEXT_CLIENT_VERSION, 2, EGL14.EGL_NONE}, 0);
      if (ctx == null || ctx == EGL14.EGL_NO_CONTEXT) return "no GLES2 context";
      surf = EGL14.eglCreatePbufferSurface(dpy, cfg[0], new int[]{EGL14.EGL_WIDTH, W, EGL14.EGL_HEIGHT, H, EGL14.EGL_NONE}, 0);
      if (surf == null || surf == EGL14.EGL_NO_SURFACE) return "no pbuffer";
      if (!EGL14.eglMakeCurrent(dpy, surf, surf, ctx)) return "makeCurrent failed";
      String gpu = GLES20.glGetString(GLES20.GL_RENDERER) + " / " + GLES20.glGetString(GLES20.GL_VERSION);
      return gpu + ": " + draw();
    } finally {
      EGL14.eglMakeCurrent(dpy, EGL14.EGL_NO_SURFACE, EGL14.EGL_NO_SURFACE, EGL14.EGL_NO_CONTEXT);
      if (surf != EGL14.EGL_NO_SURFACE) EGL14.eglDestroySurface(dpy, surf);
      if (ctx != EGL14.EGL_NO_CONTEXT) EGL14.eglDestroyContext(dpy, ctx);
      EGL14.eglTerminate(dpy);
    }
  }

  private static String draw() {
    int prog = program(
        "uniform float t; attribute vec3 p; varying float s;\n"
        + "void main() { float z = p.y * 0.9 + 0.1; s = p.z;\n"
        + "  gl_Position = vec4((p.x - 0.5 + 0.02 * sin(t)) * 2.0 / z * 0.3, 1.0 - 1.6 * p.y, z, 1.0); }",
        "precision mediump float; varying float s;\n"
        + "void main() { gl_FragColor = vec4(s * 0.3, s * 0.3, 0.35 + s * 0.3, 1.0); }");
    if (prog == 0) return "shader compile failed";

    // a grid mesh as indexed triangles, the shape of a lane-level road scene
    float[] v = new float[GRID * GRID * 3];
    for (int j = 0, k = 0; j < GRID; j++)
      for (int i = 0; i < GRID; i++) {
        v[k++] = i / (GRID - 1f);
        v[k++] = j / (GRID - 1f);
        v[k++] = ((i / 8 + j / 8) & 1);
      }
    short[] idx = new short[(GRID - 1) * (GRID - 1) * 6];
    for (int j = 0, k = 0; j < GRID - 1; j++)
      for (int i = 0; i < GRID - 1; i++) {
        short a = (short) (j * GRID + i), b = (short) (a + 1), c = (short) (a + GRID), d = (short) (c + 1);
        idx[k++] = a; idx[k++] = b; idx[k++] = c; idx[k++] = b; idx[k++] = d; idx[k++] = c;
      }
    int[] buf = new int[2];
    GLES20.glGenBuffers(2, buf, 0);
    FloatBuffer fb = ByteBuffer.allocateDirect(v.length * 4).order(ByteOrder.nativeOrder()).asFloatBuffer();
    fb.put(v).position(0);
    GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, buf[0]);
    GLES20.glBufferData(GLES20.GL_ARRAY_BUFFER, v.length * 4, fb, GLES20.GL_STATIC_DRAW);
    ByteBuffer ib = ByteBuffer.allocateDirect(idx.length * 2).order(ByteOrder.nativeOrder());
    ib.asShortBuffer().put(idx);
    GLES20.glBindBuffer(GLES20.GL_ELEMENT_ARRAY_BUFFER, buf[1]);
    GLES20.glBufferData(GLES20.GL_ELEMENT_ARRAY_BUFFER, idx.length * 2, ib, GLES20.GL_STATIC_DRAW);

    GLES20.glUseProgram(prog);
    int p = GLES20.glGetAttribLocation(prog, "p"), t = GLES20.glGetUniformLocation(prog, "t");
    GLES20.glEnableVertexAttribArray(p);
    GLES20.glVertexAttribPointer(p, 3, GLES20.GL_FLOAT, false, 12, 0);
    GLES20.glViewport(0, 0, W, H);
    GLES20.glEnable(GLES20.GL_DEPTH_TEST);

    ByteBuffer px = ByteBuffer.allocateDirect(W * H * 4).order(ByteOrder.nativeOrder());
    long drawSum = 0, drawMax = 0, readSum = 0;
    for (int f = 0; f <= FRAMES; f++) {
      long a = SystemClock.elapsedRealtimeNanos();
      GLES20.glClearColor(0, 0, 0, 1);
      GLES20.glClear(GLES20.GL_COLOR_BUFFER_BIT | GLES20.GL_DEPTH_BUFFER_BIT);
      GLES20.glUniform1f(t, f * 0.1f);
      GLES20.glDrawElements(GLES20.GL_TRIANGLES, idx.length, GLES20.GL_UNSIGNED_SHORT, 0);
      GLES20.glFinish();
      long b = SystemClock.elapsedRealtimeNanos();
      px.position(0);
      GLES20.glReadPixels(0, 0, W, H, GLES20.GL_RGBA, GLES20.GL_UNSIGNED_BYTE, px);
      long c = SystemClock.elapsedRealtimeNanos();
      if (f == 0) continue;  // the first frame pays for uploads and shader setup
      drawSum += b - a; drawMax = Math.max(drawMax, b - a); readSum += c - b;
      Thread.yield();
    }
    int err = GLES20.glGetError();
    GLES20.glDeleteBuffers(2, buf, 0);
    GLES20.glDeleteProgram(prog);
    return idx.length / 3 + " triangles, draw " + ms(drawSum / FRAMES) + " ms avg " + ms(drawMax) + " max, read-back "
        + ms(readSum / FRAMES) + " ms, centre px " + (px.get((H / 2 * W + W / 2) * 4) & 0xff) + ", glError " + err;
  }

  private static String processName() {
    try {
      java.io.FileInputStream in = new java.io.FileInputStream("/proc/self/cmdline");
      byte[] b = new byte[256];
      int n = in.read(b);
      in.close();
      int end = 0;
      while (end < n && b[end] != 0) end++;
      return end > 0 ? new String(b, 0, end, "UTF-8") : null;
    } catch (java.io.IOException e) {
      return null;
    }
  }

  private static String ms(long ns) {
    return String.valueOf(Math.round(ns / 1e5) / 10.0);
  }

  private static int program(String vs, String fs) {
    int v = shader(GLES20.GL_VERTEX_SHADER, vs), f = shader(GLES20.GL_FRAGMENT_SHADER, fs);
    if (v == 0 || f == 0) return 0;
    int p = GLES20.glCreateProgram();
    GLES20.glAttachShader(p, v);
    GLES20.glAttachShader(p, f);
    GLES20.glLinkProgram(p);
    int[] ok = new int[1];
    GLES20.glGetProgramiv(p, GLES20.GL_LINK_STATUS, ok, 0);
    return ok[0] == 0 ? 0 : p;
  }

  private static int shader(int type, String src) {
    int s = GLES20.glCreateShader(type);
    GLES20.glShaderSource(s, src);
    GLES20.glCompileShader(s);
    int[] ok = new int[1];
    GLES20.glGetShaderiv(s, GLES20.GL_COMPILE_STATUS, ok, 0);
    return ok[0] == 0 ? 0 : s;
  }
}
