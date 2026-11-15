package android.graphics;

import java.awt.AlphaComposite;
import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.RenderingHints;
import java.awt.Shape;
import java.awt.font.FontRenderContext;
import java.awt.geom.AffineTransform;
import java.awt.geom.Area;
import java.awt.geom.Arc2D;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Line2D;
import java.awt.geom.Path2D;
import java.awt.geom.PathIterator;
import java.awt.geom.Rectangle2D;
import java.awt.geom.RoundRectangle2D;
import java.awt.image.BufferedImage;
import java.util.ArrayDeque;

// Java2D behind Android's Canvas API, for desktop renders of the dash map.
//
// Everything is drawn in device space: a shape in the canvas's local space goes through the current Matrix (a full 3x3,
// perspective included: the dash map's camera is one) and the result is filled. A stroke is built in local space (the
// width is in local units, display metres for the roads) and then mapped, which is what Skia does, so a road comes out
// wider near the car and narrower toward the horizon. A zero width stroke is a 1 px hairline in device space. Lines
// keep straight under perspective (Skia maps the points, it does not subdivide). Clips are not anti-aliased, as on the
// head unit's Android 4.2 software canvas.
public class Canvas {
  private final BufferedImage img;
  private final Graphics2D g;
  private Matrix m = new Matrix();
  private Area clip;  // device space; null = everything
  private final ArrayDeque<Object[]> stack = new ArrayDeque<Object[]>();

  public Canvas(Bitmap b) {
    img = b.img;
    g = img.createGraphics();
    g.setRenderingHint(RenderingHints.KEY_STROKE_CONTROL, RenderingHints.VALUE_STROKE_PURE);
  }

  public int getWidth() { return img.getWidth(); }
  public int getHeight() { return img.getHeight(); }
  public boolean isHardwareAccelerated() { return false; }

  public int save() {
    stack.push(new Object[] {new Matrix(m), clip == null ? null : new Area(clip)});
    return stack.size();
  }
  public void restore() {
    if (stack.isEmpty()) return;
    Object[] s = stack.pop();
    m = (Matrix) s[0];
    clip = (Area) s[1];
  }
  public void translate(float dx, float dy) { m.preTranslate(dx, dy); }
  public void scale(float sx, float sy) { m.preScale(sx, sy); }
  public void scale(float sx, float sy, float px, float py) { m.preScale(sx, sy, px, py); }
  public void rotate(float d) { m.preRotate(d); }
  public void concat(Matrix o) { m.preConcat(o); }
  public void setMatrix(Matrix o) { if (o == null) m.reset(); else m.set(o); }
  public void getMatrix(Matrix o) { o.set(m); }

  // ---- clip

  private void clipDevice(Shape dev, Region.Op op) {
    Area a = new Area(dev);
    if (op == Region.Op.DIFFERENCE) {
      if (clip == null) {
        clip = new Area(new Rectangle2D.Float(-1e6f, -1e6f, 2e6f, 2e6f));
      }
      clip.subtract(a);
    } else if (op == Region.Op.REPLACE) {
      clip = a;
    } else if (op == Region.Op.UNION) {
      if (clip != null) clip.add(a);
    } else if (clip == null) {
      clip = a;
    } else {
      clip.intersect(a);
    }
  }
  public boolean clipRect(float l, float t, float r, float b) { clipDevice(device(new Rectangle2D.Float(l, t, r - l, b - t)), Region.Op.INTERSECT); return true; }
  public boolean clipRect(RectF r) { return clipRect(r.left, r.top, r.right, r.bottom); }
  public boolean clipRect(float l, float t, float r, float b, Region.Op op) { clipDevice(device(new Rectangle2D.Float(l, t, r - l, b - t)), op); return true; }
  public boolean clipPath(Path p) { clipDevice(device(p.p), Region.Op.INTERSECT); return true; }
  public boolean clipPath(Path p, Region.Op op) { clipDevice(device(p.p), op); return true; }

  // ---- drawing

  public void drawColor(int color) {
    g.setClip(clip);
    g.setComposite(AlphaComposite.SrcOver);
    g.setColor(new Color(color, true));
    g.fillRect(-1000000, -1000000, 2000000, 2000000);
  }
  public void drawRect(float l, float t, float r, float b, Paint p) { draw(new Rectangle2D.Float(Math.min(l, r), Math.min(t, b), Math.abs(r - l), Math.abs(b - t)), p); }
  public void drawRect(RectF r, Paint p) { drawRect(r.left, r.top, r.right, r.bottom, p); }
  public void drawRoundRect(RectF r, float rx, float ry, Paint p) { draw(new RoundRectangle2D.Float(r.left, r.top, r.width(), r.height(), 2 * rx, 2 * ry), p); }
  public void drawCircle(float cx, float cy, float rad, Paint p) { draw(new Ellipse2D.Float(cx - rad, cy - rad, 2 * rad, 2 * rad), p); }
  public void drawOval(RectF r, Paint p) { draw(new Ellipse2D.Float(r.left, r.top, r.width(), r.height()), p); }
  public void drawLine(float x0, float y0, float x1, float y1, Paint p) {
    Path2D.Float s = new Path2D.Float();
    s.moveTo(x0, y0);
    s.lineTo(x1, y1);
    draw(s, p, true);
  }
  public void drawArc(RectF r, float start, float sweep, boolean center, Paint p) {
    // Android's angles run clockwise from 3 o'clock; Arc2D's counter-clockwise
    draw(new Arc2D.Float(r.left, r.top, r.width(), r.height(), -start, -sweep, center ? Arc2D.PIE : Arc2D.OPEN), p);
  }
  public void drawPath(Path path, Paint p) { draw(path.p, p); }

  private void draw(Shape local, Paint p) { draw(local, p, false); }

  private void draw(Shape local, Paint p, boolean lineOnly) {
    if (local == null) return;
    boolean fill = p.style != Paint.Style.STROKE && !lineOnly, stroke = p.style != Paint.Style.FILL || lineOnly;
    if (fill) paint(device(local), p);
    if (stroke) {
      if (p.strokeWidth <= 0) {
        paint(new BasicStroke(1f).createStrokedShape(device(local)), p);  // hairline: 1 device px
      } else {
        float s = scaleAt(local);
        BasicStroke bs = new BasicStroke(p.strokeWidth, p.cap == Paint.Cap.ROUND ? BasicStroke.CAP_ROUND : p.cap == Paint.Cap.SQUARE ? BasicStroke.CAP_SQUARE : BasicStroke.CAP_BUTT,
            p.join == Paint.Join.ROUND ? BasicStroke.JOIN_ROUND : p.join == Paint.Join.BEVEL ? BasicStroke.JOIN_BEVEL : BasicStroke.JOIN_MITER, Math.max(1f, p.miter));
        Shape outline = bs.createStrokedShape(local);
        if (m.hasPerspective()) outline = flatten(outline, Math.max(0.02f, 0.2f / Math.max(s, 1e-6f)));
        paint(device(outline), p);
      }
    }
  }

  // device pixels per local unit near the shape (for flatness before a perspective map)
  private float scaleAt(Shape local) {
    Rectangle2D b = local.getBounds2D();
    float[] a = {(float) b.getCenterX(), (float) b.getCenterY(), (float) b.getCenterX() + 1, (float) b.getCenterY()};
    m.mapPoints(a);
    return (float) Math.hypot(a[2] - a[0], a[3] - a[1]);
  }

  private static Shape flatten(Shape s, float flat) {
    Path2D.Float out = new Path2D.Float(Path2D.WIND_NON_ZERO);
    out.append(s.getPathIterator(null, flat), false);
    return out;
  }

  // local shape to device shape
  private Shape device(Shape local) {
    if (!m.hasPerspective()) {
      float[] v = m.m;
      return new AffineTransform(v[0], v[3], v[1], v[4], v[2], v[5]).createTransformedShape(local);
    }
    return Path.project(flatten(local, Math.max(0.02f, 0.2f / Math.max(scaleAt(local), 1e-6f))), m);
  }

  private void paint(Shape dev, Paint p) {
    g.setClip(clip);
    g.setComposite(AlphaComposite.SrcOver);
    g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, p.isAntiAlias() ? RenderingHints.VALUE_ANTIALIAS_ON : RenderingHints.VALUE_ANTIALIAS_OFF);
    g.setPaint(awtPaint(p));
    g.fill(dev);
  }

  private java.awt.Paint awtPaint(Paint p) {
    if (p.shader instanceof LinearGradient) {
      LinearGradient lg = (LinearGradient) p.shader;
      float[] pts = {lg.x0, lg.y0, lg.x1, lg.y1};
      m.mapPoints(pts);
      int n = lg.colors.length;
      float[] fr = new float[n];
      Color[] cs = new Color[n];
      for (int i = 0; i < n; i++) {
        fr[i] = lg.pos == null ? i / (float) (n - 1) : lg.pos[i];
        if (i > 0 && fr[i] <= fr[i - 1]) fr[i] = Math.min(1f, fr[i - 1] + 1e-4f);
        cs[i] = new Color(lg.colors[i], true);
      }
      for (int i = n - 2; i >= 0; i--) if (fr[i] >= fr[i + 1]) fr[i] = Math.max(0f, fr[i + 1] - 1e-4f);
      if (Math.hypot(pts[2] - pts[0], pts[3] - pts[1]) < 1e-3 || fr[0] != 0 && fr[0] < 0) return new Color(p.color, true);
      try {
        return new java.awt.LinearGradientPaint(pts[0], pts[1], pts[2], pts[3], fr, cs, java.awt.MultipleGradientPaint.CycleMethod.NO_CYCLE);
      } catch (IllegalArgumentException e) {
        return new Color(lg.colors[0], true);
      }
    }
    return new Color(p.color, true);
  }

  // ---- text: the matrix's affine part, perspective ignored (the app draws text in screen space)

  public void drawText(String text, float x, float y, Paint p) {
    if (text == null || text.isEmpty()) return;
    float w = p.measureText(text);
    float dx = p.align == Paint.Align.CENTER ? -w / 2 : p.align == Paint.Align.RIGHT ? -w : 0;
    float[] v = m.m;
    AffineTransform at = new AffineTransform(v[0], v[3], v[1], v[4], v[2], v[5]);
    g.setClip(clip);
    g.setComposite(AlphaComposite.SrcOver);
    g.setRenderingHint(RenderingHints.KEY_TEXT_ANTIALIASING, p.isAntiAlias() ? RenderingHints.VALUE_TEXT_ANTIALIAS_ON : RenderingHints.VALUE_TEXT_ANTIALIAS_OFF);
    g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
    g.setRenderingHint(RenderingHints.KEY_FRACTIONALMETRICS, RenderingHints.VALUE_FRACTIONALMETRICS_ON);
    g.setPaint(awtPaint(p));
    AffineTransform old = g.getTransform();
    g.transform(at);
    if (p.style == Paint.Style.STROKE && p.strokeWidth > 0) {
      Shape glyphs = p.font().createGlyphVector(new FontRenderContext(null, true, true), text).getOutline(x + dx, y);
      g.setStroke(new BasicStroke(p.strokeWidth, p.cap == Paint.Cap.ROUND ? BasicStroke.CAP_ROUND : BasicStroke.CAP_BUTT,
          p.join == Paint.Join.ROUND ? BasicStroke.JOIN_ROUND : BasicStroke.JOIN_MITER));
      g.draw(glyphs);
    } else {
      g.setFont(p.font());
      g.drawString(text, x + dx, y);
    }
    g.setTransform(old);
  }
  public void drawText(CharSequence text, int start, int end, float x, float y, Paint p) { drawText(text.subSequence(start, end).toString(), x, y, p); }

  // ---- bitmaps (the matrix's affine part)

  public void drawBitmap(Bitmap b, float left, float top, Paint p) {
    drawBitmap(b, null, new RectF(left, top, left + b.getWidth(), top + b.getHeight()), p);
  }
  public void drawBitmap(Bitmap b, Rect src, RectF dst, Paint p) {
    int sx = src == null ? 0 : src.left, sy = src == null ? 0 : src.top;
    int sw = src == null ? b.getWidth() : src.width(), sh = src == null ? b.getHeight() : src.height();
    float[] v = m.m;
    AffineTransform at = new AffineTransform(v[0], v[3], v[1], v[4], v[2], v[5]);
    g.setClip(clip);
    boolean smooth = p != null && (p.flags & Paint.FILTER_BITMAP_FLAG) != 0;
    g.setRenderingHint(RenderingHints.KEY_INTERPOLATION, smooth ? RenderingHints.VALUE_INTERPOLATION_BILINEAR : RenderingHints.VALUE_INTERPOLATION_NEAREST_NEIGHBOR);
    int a = p == null ? 255 : p.color >>> 24;
    g.setComposite(a == 255 ? AlphaComposite.SrcOver : AlphaComposite.getInstance(AlphaComposite.SRC_OVER, a / 255f));
    AffineTransform old = g.getTransform();
    g.transform(at);
    g.drawImage(b.img, Math.round(dst.left), Math.round(dst.top), Math.round(dst.right), Math.round(dst.bottom), sx, sy, sx + sw, sy + sh, null);
    g.setTransform(old);
    g.setComposite(AlphaComposite.SrcOver);
  }
  public void drawBitmap(Bitmap b, Rect src, Rect dst, Paint p) { drawBitmap(b, src, new RectF(dst), p); }
}
