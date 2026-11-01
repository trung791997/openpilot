package com.starpilot.aaroad;

import android.opengl.GLES20;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;

// The close-up look on the GPU layer (GlRoads): the scene's CloseUp detail in its own two buffers, drawn in
// CloseView.passes' order between the bridges and the carpet, as the view draws it in software.
//
// Each segment is a quad of 4 corners, 9 floats a corner: a.e, a.n, b.e, b.n; across (-1, 1), end (0, 1), metres
// along at a (mod the dash period); the heights at a and b. The vertex shader turns the ends into the car's frame as
// GlRoads' does, then lifts each by its height: a point hd display metres up is drawn where the camera's ray through
// it meets the ground (CloseUp.lift), and the half width grows by the same factor. The fragment shader is GlRoads'
// without the carpet, plus dashes: a fragment more than ON past each PERIOD along is dropped.
//
// A new scene's detail is packed and sent on the GL thread PACK_SEGS segments a frame (147 KB) into the back buffer
// while the last one draws, its camera moved to the old anchor; then the buffers swap. Nothing allocates per frame.
final class GlClose implements CloseView.Pass {
  static final int FLOATS = CloseUp.CORNER, STRIDE = FLOATS * 4;
  static final int PACK_SEGS = 1024;
  private static final int MAX_QUADS = 16384;  // GlRoads': the index buffer's quads

  private static final String VS =
      "uniform mat4 uP; uniform vec4 uW; uniform vec4 uV; uniform vec4 uCam; uniform float uR; uniform vec4 uH;\n"
      + "attribute vec4 aSeg; attribute vec3 aC; attribute vec2 aH;\n"
      + "varying vec2 vPA; varying vec2 vU; varying float vL; varying float vW; varying float vNum;\n"
      + "varying float vR; varying float vAl; varying float vK;\n"
      + "vec2 car(vec2 p) { return vec2(uW.x * p.x - uW.y * p.y + uW.z, uW.y * p.x + uW.x * p.y + uW.w); }\n"
      // uH: x display metres up per world metre of height (0 on the ground), y the camera's height, z its foot's z
      + "float up(float h) { return uH.y / max(uH.y - h * uH.x, 1.0); }\n"
      + "vec2 lift(vec2 p, float s) { return vec2(p.x * s, uH.z + (p.y - uH.z) * s); }\n"
      + "void main() {\n"
      + "  float sa = up(aH.x), sb = up(aH.y);\n"
      + "  vec2 a = lift(car(aSeg.xy), sa), b = lift(car(aSeg.zw), sb), d = b - a;\n"
      + "  float l = length(d);\n"
      + "  vec2 u = l > 1e-5 ? d / l : vec2(1.0, 0.0);\n"
      + "  vec2 e = aC.y < 0.5 ? a : b;\n"
      + "  float w = abs(uCam.z * e.y + 1.0);\n"
      + "  vR = uR * max(sa, sb);\n"
      + "  float m = vR + 2.0 * (w / uCam.x + w * w * abs(uCam.w) * 2.0);\n"
      + "  vec2 p = e + u * (aC.y < 0.5 ? -m : m) + vec2(-u.y, u.x) * (aC.x * m);\n"
      + "  vPA = p - a; vU = u; vL = l;\n"
      + "  vAl = aC.z; vK = length(uW.xy) * 0.5 * (sa + sb);  // display metres per world metre here\n"
      + "  vec4 c = uP * vec4(p, 0.0, 1.0);\n"
      + "  vW = c.w; vNum = uV.x * p.x + uV.y * p.y + uV.z;\n"
      + "  gl_Position = c;\n"
      + "}\n";

  private static final String FS =
      "precision mediump float;\n"
      + "uniform vec4 uColor; uniform vec4 uCam; uniform vec2 uDash;\n"
      + "varying vec2 vPA; varying vec2 vU; varying float vL; varying float vW; varying float vNum;\n"
      + "varying float vR; varying float vAl; varying float vK;\n"
      + "vec2 c2(float w, float num) { float dz = w * w * uCam.w; return vec2((num / w * uCam.z - uCam.y) / uCam.x * dz, dz); }\n"
      + "void main() {\n"
      + "  float t = dot(vPA, vU), tc = clamp(t, 0.0, vL);\n"
      // uDash: x the period, y the dash, in world metres; x 0 solid
      + "  if (uDash.x > 0.0 && (t < 0.0 || t > vL || mod(vAl + tc / vK, uDash.x) > uDash.y)) discard;\n"
      + "  vec2 g = vPA - vU * tc;\n"
      + "  float d = length(g);\n"
      + "  vec2 n = g / max(d, 1e-3);\n"
      + "  float fp = length(vec2(n.x * vW / uCam.x, dot(n, c2(vW, vNum))));\n"
      + "  float a = clamp((vR - d) / max(fp, 1e-3) + 0.5, 0.0, 1.0);\n"
      + "  if (a <= 0.0) discard;\n"
      + "  gl_FragColor = vec4(uColor.rgb, uColor.a * a);\n"
      + "}\n";

  private int prog, pP, pW, pV, pCam, pR, pH, pColor, pDash, aSeg, aC, aH;
  private int vbo, back;
  private ByteBuffer buf;
  private FloatBuffer fb;
  private final float[] corner = new float[PACK_SEGS * 4 * FLOATS];

  // the detail on the GPU (shown, of scene shownScene) and the one being sent (into back, packAt segments so far)
  private CloseUp.Detail shown, packing;
  private MapView.Scene shownScene, packScene;
  private int packAt;
  private float ik;
  int uploads;  // for the log

  // On the GL thread with the context current: false if the program did not build (the close-up look stays off).
  boolean setup() {
    prog = GlProbe.program(VS, FS);
    if (prog == 0) return false;
    pP = GLES20.glGetUniformLocation(prog, "uP");
    pW = GLES20.glGetUniformLocation(prog, "uW");
    pV = GLES20.glGetUniformLocation(prog, "uV");
    pCam = GLES20.glGetUniformLocation(prog, "uCam");
    pR = GLES20.glGetUniformLocation(prog, "uR");
    pH = GLES20.glGetUniformLocation(prog, "uH");
    pColor = GLES20.glGetUniformLocation(prog, "uColor");
    pDash = GLES20.glGetUniformLocation(prog, "uDash");
    aSeg = GLES20.glGetAttribLocation(prog, "aSeg");
    aC = GLES20.glGetAttribLocation(prog, "aC");
    aH = GLES20.glGetAttribLocation(prog, "aH");
    if (aSeg < 0 || aC < 0 || aH < 0) return false;
    int[] b = new int[2];
    GLES20.glGenBuffers(2, b, 0);
    vbo = b[0];
    back = b[1];
    buf = ByteBuffer.allocateDirect(PACK_SEGS * 4 * STRIDE).order(ByteOrder.nativeOrder());
    fb = buf.asFloatBuffer();
    return GLES20.glGetError() == GLES20.GL_NO_ERROR;
  }

  // Once a frame, before draw: sends the next slice of s's detail if it is not up yet.
  void step(MapView.Scene s) {
    if (s == null || s == shownScene) {
      packScene = null;
      return;
    }
    CloseUp.Detail d = s.close;
    if (d == null || d.count() == 0) {
      shown = null;
      shownScene = s;
      packScene = null;
      return;
    }
    if (packScene != s) {
      packScene = s;
      packing = d;
      packAt = 0;
      GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, back);
      GLES20.glBufferData(GLES20.GL_ARRAY_BUFFER, d.count() * 4 * STRIDE, null, GLES20.GL_STATIC_DRAW);
    }
    int n = Math.min(PACK_SEGS, packing.count() - packAt);
    if (n > 0) {
      CloseUp.corners(packing, packAt, n, corner);
      fb.position(0);
      fb.put(corner, 0, n * 4 * FLOATS);
      buf.position(0);
      GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, back);
      GLES20.glBufferSubData(GLES20.GL_ARRAY_BUFFER, packAt * 4 * STRIDE, n * 4 * STRIDE, buf);
      packAt += n;
    }
    if (packAt < packing.count()) return;
    int was = vbo;
    vbo = back;
    back = was;
    GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, back);
    GLES20.glBufferData(GLES20.GL_ARRAY_BUFFER, 4, null, GLES20.GL_STATIC_DRAW);
    shown = packing;
    shownScene = packScene;
    packing = null;
    packScene = null;
    uploads++;
  }

  // Draws the passes for frame f of scene s, gates gr (roads) and gw (walks). proj, v, qa as GlRoads' line program
  // has them. The index buffer is GlRoads', bound. Leaves its own program and buffer bound and its arrays disabled.
  void draw(MapView.Scene s, MapView.Frame f, float[] proj, float v0, float v1, float v2, float v7, float qa, float gr,
            float gw) {
    CloseUp.Detail d = shown;
    if (d == null || (gr <= 0 && gw <= 0)) return;
    float ce = f.ce, cn = f.cn, ch = f.ch, sh = f.sh;
    MapView.Scene u = shownScene;
    if (u != s) {  // the detail on the GPU is the last scene's while the new one is sent: its anchor
      double x = s.ax + ce / s.mpt, y = s.ay - cn / s.mpt;
      ce = (float) ((x - u.ax) * u.mpt);
      cn = (float) (-(y - u.ay) * u.mpt);
    }
    ik = f.ik;
    GLES20.glUseProgram(prog);
    GLES20.glUniformMatrix4fv(pP, 1, false, proj, 0);
    GLES20.glUniform4f(pW, ch * ik, sh * ik, (-ce * ch + cn * sh) * ik, (-ce * sh - cn * ch) * ik);
    GLES20.glUniform4f(pV, v0, v1, v2, 0);
    GLES20.glUniform4f(pCam, v0, v1, v7, qa);
    GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, vbo);
    GLES20.glEnableVertexAttribArray(aSeg);
    GLES20.glEnableVertexAttribArray(aC);
    GLES20.glEnableVertexAttribArray(aH);
    CloseView.passes(d, ik, gr, gw, this);
    GLES20.glDisableVertexAttribArray(aSeg);
    GLES20.glDisableVertexAttribArray(aC);
    GLES20.glDisableVertexAttribArray(aH);
  }

  @Override
  public void lines(int r, float hs, float width, int color, float alpha, float on, float period) {
    CloseUp.Detail d = shown;
    int from = d.from[r], left = d.from[r + 1] - from;
    if (left <= 0 || alpha <= 0) return;
    GLES20.glUniform1f(pR, width / 2);
    GLES20.glUniform4f(pH, hs * ik, CloseUp.CAM_H, CloseUp.CAM_ZC, 0);
    GLES20.glUniform2f(pDash, period, on);
    GLES20.glUniform4f(pColor, (color >> 16 & 0xff) / 255f, (color >> 8 & 0xff) / 255f, (color & 0xff) / 255f,
                       Math.min(1, alpha));
    for (int st = from; left > 0; ) {
      int q = Math.min(left, MAX_QUADS);
      GLES20.glVertexAttribPointer(aSeg, 4, GLES20.GL_FLOAT, false, STRIDE, st * 4 * STRIDE);
      GLES20.glVertexAttribPointer(aC, 3, GLES20.GL_FLOAT, false, STRIDE, st * 4 * STRIDE + 16);
      GLES20.glVertexAttribPointer(aH, 2, GLES20.GL_FLOAT, false, STRIDE, st * 4 * STRIDE + 28);
      GLES20.glDrawElements(GLES20.GL_TRIANGLES, q * 6, GLES20.GL_UNSIGNED_SHORT, 0);
      st += q;
      left -= q;
    }
  }
}
