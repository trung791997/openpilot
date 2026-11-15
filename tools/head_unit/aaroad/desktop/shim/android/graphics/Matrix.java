package android.graphics;

// Android's 3x3 matrix, row-major [MSCALE_X MSKEW_X MTRANS_X; MSKEW_Y MSCALE_Y MTRANS_Y; MPERSP_0 MPERSP_1 MPERSP_2].
// Pure float math, so perspective (the dash map's camera) behaves as Skia's does.
public class Matrix {
  public static final int MSCALE_X = 0, MSKEW_X = 1, MTRANS_X = 2, MSKEW_Y = 3, MSCALE_Y = 4, MTRANS_Y = 5, MPERSP_0 = 6, MPERSP_1 = 7, MPERSP_2 = 8;
  public final float[] m = {1, 0, 0, 0, 1, 0, 0, 0, 1};

  public Matrix() {}
  public Matrix(Matrix o) { set(o); }
  public void reset() { setValues(new float[] {1, 0, 0, 0, 1, 0, 0, 0, 1}); }
  public void set(Matrix o) { System.arraycopy(o.m, 0, m, 0, 9); }
  public void setValues(float[] v) { System.arraycopy(v, 0, m, 0, 9); }
  public void getValues(float[] v) { System.arraycopy(m, 0, v, 0, 9); }
  public boolean isIdentity() { return m[0] == 1 && m[1] == 0 && m[2] == 0 && m[3] == 0 && m[4] == 1 && m[5] == 0 && m[6] == 0 && m[7] == 0 && m[8] == 1; }
  public boolean hasPerspective() { return m[6] != 0 || m[7] != 0 || m[8] != 1; }
  public void setTranslate(float dx, float dy) { setValues(new float[] {1, 0, dx, 0, 1, dy, 0, 0, 1}); }
  public void setScale(float sx, float sy) { setValues(new float[] {sx, 0, 0, 0, sy, 0, 0, 0, 1}); }
  public void setScale(float sx, float sy, float px, float py) { setValues(new float[] {sx, 0, px - sx * px, 0, sy, py - sy * py, 0, 0, 1}); }
  public void setRotate(float deg) {
    double r = Math.toRadians(deg);
    float c = (float) Math.cos(r), s = (float) Math.sin(r);
    setValues(new float[] {c, -s, 0, s, c, 0, 0, 0, 1});
  }
  public void setRotate(float deg, float px, float py) {
    double r = Math.toRadians(deg);
    float c = (float) Math.cos(r), s = (float) Math.sin(r);
    setValues(new float[] {c, -s, px - c * px + s * py, s, c, py - s * px - c * py, 0, 0, 1});
  }
  private boolean mul(Matrix a, Matrix b) {  // this = a * b
    float[] x = a.m, y = b.m, r = new float[9];
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) r[3 * i + j] = x[3 * i] * y[j] + x[3 * i + 1] * y[3 + j] + x[3 * i + 2] * y[6 + j];
    }
    setValues(r);
    return true;
  }
  public boolean setConcat(Matrix a, Matrix b) { return mul(a, b); }
  public boolean preConcat(Matrix o) { return mul(new Matrix(this), o); }
  public boolean postConcat(Matrix o) { return mul(o, new Matrix(this)); }
  public boolean preTranslate(float dx, float dy) { Matrix t = new Matrix(); t.setTranslate(dx, dy); return preConcat(t); }
  public boolean postTranslate(float dx, float dy) { Matrix t = new Matrix(); t.setTranslate(dx, dy); return postConcat(t); }
  public boolean preScale(float sx, float sy) { Matrix t = new Matrix(); t.setScale(sx, sy); return preConcat(t); }
  public boolean postScale(float sx, float sy) { Matrix t = new Matrix(); t.setScale(sx, sy); return postConcat(t); }
  public boolean preScale(float sx, float sy, float px, float py) { Matrix t = new Matrix(); t.setScale(sx, sy, px, py); return preConcat(t); }
  public boolean preRotate(float d) { Matrix t = new Matrix(); t.setRotate(d); return preConcat(t); }
  public boolean postRotate(float d) { Matrix t = new Matrix(); t.setRotate(d); return postConcat(t); }
  public boolean preRotate(float d, float px, float py) { Matrix t = new Matrix(); t.setRotate(d, px, py); return preConcat(t); }
  public boolean postRotate(float d, float px, float py) { Matrix t = new Matrix(); t.setRotate(d, px, py); return postConcat(t); }

  public boolean invert(Matrix inv) {
    float[] a = m;
    double c00 = a[4] * a[8] - a[5] * a[7], c01 = a[5] * a[6] - a[3] * a[8], c02 = a[3] * a[7] - a[4] * a[6];
    double det = a[0] * c00 + a[1] * c01 + a[2] * c02;
    if (Math.abs(det) < 1e-30) return false;
    double d = 1 / det;
    float[] r = {
        (float) (c00 * d), (float) ((a[2] * a[7] - a[1] * a[8]) * d), (float) ((a[1] * a[5] - a[2] * a[4]) * d),
        (float) (c01 * d), (float) ((a[0] * a[8] - a[2] * a[6]) * d), (float) ((a[2] * a[3] - a[0] * a[5]) * d),
        (float) (c02 * d), (float) ((a[1] * a[6] - a[0] * a[7]) * d), (float) ((a[0] * a[4] - a[1] * a[3]) * d)};
    inv.setValues(r);
    return true;
  }

  public void mapPoints(float[] dst, int dstIndex, float[] src, int srcIndex, int count) {
    for (int i = 0; i < count; i++) {
      float x = src[srcIndex + 2 * i], y = src[srcIndex + 2 * i + 1];
      float w = m[6] * x + m[7] * y + m[8];
      float nx = m[0] * x + m[1] * y + m[2], ny = m[3] * x + m[4] * y + m[5];
      if (w != 1) {
        float iw = w == 0 ? 0 : 1 / w;
        nx *= iw;
        ny *= iw;
      }
      dst[dstIndex + 2 * i] = nx;
      dst[dstIndex + 2 * i + 1] = ny;
    }
  }
  public void mapPoints(float[] dst, float[] src) { mapPoints(dst, 0, src, 0, src.length / 2); }
  public void mapPoints(float[] pts) { mapPoints(pts, 0, pts, 0, pts.length / 2); }
  public float mapRadius(float r) {
    float a = (float) Math.hypot(m[0], m[3]), b = (float) Math.hypot(m[1], m[4]);
    return r * (float) Math.sqrt(a * b);
  }
  public boolean mapRect(RectF dst, RectF src) {
    float[] p = {src.left, src.top, src.right, src.top, src.right, src.bottom, src.left, src.bottom};
    mapPoints(p);
    float l = p[0], t = p[1], r = p[0], b = p[1];
    for (int i = 1; i < 4; i++) {
      l = Math.min(l, p[2 * i]); r = Math.max(r, p[2 * i]);
      t = Math.min(t, p[2 * i + 1]); b = Math.max(b, p[2 * i + 1]);
    }
    dst.set(l, t, r, b);
    return true;
  }
  public boolean mapRect(RectF r) { return mapRect(r, r); }
  public boolean setPolyToPoly(float[] src, int si, float[] dst, int di, int n) {
    if (n != 4) return false;  // a 4 point projective fit is all the app could ask for
    float[] a = squareTo(src, si), b = squareTo(dst, di);
    Matrix ma = new Matrix(), mb = new Matrix(), inv = new Matrix();
    ma.setValues(a);
    mb.setValues(b);
    if (!ma.invert(inv)) return false;
    return mul(mb, inv);
  }
  private static float[] squareTo(float[] p, int o) {  // unit square -> quad
    double x0 = p[o], y0 = p[o + 1], x1 = p[o + 2], y1 = p[o + 3], x2 = p[o + 4], y2 = p[o + 5], x3 = p[o + 6], y3 = p[o + 7];
    double dx1 = x1 - x2, dx2 = x3 - x2, dx3 = x0 - x1 + x2 - x3, dy1 = y1 - y2, dy2 = y3 - y2, dy3 = y0 - y1 + y2 - y3;
    double g, h;
    double den = dx1 * dy2 - dx2 * dy1;
    if (Math.abs(dx3) < 1e-12 && Math.abs(dy3) < 1e-12) { g = 0; h = 0; }
    else { g = (dx3 * dy2 - dx2 * dy3) / den; h = (dx1 * dy3 - dx3 * dy1) / den; }
    return new float[] {(float) (x1 - x0 + g * x1), (float) (x3 - x0 + h * x3), (float) x0,
        (float) (y1 - y0 + g * y1), (float) (y3 - y0 + h * y3), (float) y0, (float) g, (float) h, 1};
  }
  public String toString() { return java.util.Arrays.toString(m); }
}
