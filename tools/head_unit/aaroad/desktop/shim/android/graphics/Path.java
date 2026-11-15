package android.graphics;

import java.awt.geom.AffineTransform;
import java.awt.geom.Path2D;
import java.awt.geom.PathIterator;
import java.awt.geom.Rectangle2D;

// Backed by Path2D.Float. transform() with a perspective matrix maps every control point through the matrix, as Skia does.
public class Path {
  public enum Direction { CW, CCW }
  public enum FillType { WINDING, EVEN_ODD, INVERSE_WINDING, INVERSE_EVEN_ODD }

  public Path2D.Float p = new Path2D.Float(Path2D.WIND_NON_ZERO);
  private boolean open;  // a moveTo with nothing after it

  public Path() {}
  public Path(Path o) { set(o); }
  public void set(Path o) { p = new Path2D.Float(o.p); open = o.open; }
  public void reset() { p.reset(); open = false; }
  public void rewind() { p.reset(); open = false; }
  public boolean isEmpty() { return p.getCurrentPoint() == null; }
  public void setFillType(FillType t) { p.setWindingRule(t == FillType.EVEN_ODD || t == FillType.INVERSE_EVEN_ODD ? Path2D.WIND_EVEN_ODD : Path2D.WIND_NON_ZERO); }
  public void moveTo(float x, float y) { p.moveTo(x, y); }
  public void lineTo(float x, float y) { if (p.getCurrentPoint() == null) p.moveTo(x, y); else p.lineTo(x, y); }
  public void quadTo(float x1, float y1, float x2, float y2) { if (p.getCurrentPoint() == null) p.moveTo(x1, y1); p.quadTo(x1, y1, x2, y2); }
  public void cubicTo(float x1, float y1, float x2, float y2, float x3, float y3) { if (p.getCurrentPoint() == null) p.moveTo(x1, y1); p.curveTo(x1, y1, x2, y2, x3, y3); }
  public void close() { if (p.getCurrentPoint() != null) p.closePath(); }
  public void addPath(Path o) { p.append(o.p, false); }
  public void addPath(Path o, float dx, float dy) { p.append(o.p.getPathIterator(AffineTransform.getTranslateInstance(dx, dy)), false); }
  public void addRect(float l, float t, float r, float b, Direction d) {
    moveTo(l, t); lineTo(r, t); lineTo(r, b); lineTo(l, b); close();
  }
  public void addRect(RectF r, Direction d) { addRect(r.left, r.top, r.right, r.bottom, d); }
  public void addRoundRect(RectF r, float rx, float ry, Direction d) {
    p.append(new java.awt.geom.RoundRectangle2D.Float(r.left, r.top, r.width(), r.height(), 2 * rx, 2 * ry), false);
  }
  public void addCircle(float x, float y, float r, Direction d) { p.append(new java.awt.geom.Ellipse2D.Float(x - r, y - r, 2 * r, 2 * r), false); }
  public void offset(float dx, float dy) { p.transform(AffineTransform.getTranslateInstance(dx, dy)); }
  public void computeBounds(RectF b, boolean exact) {
    Rectangle2D r = p.getBounds2D();
    b.set((float) r.getMinX(), (float) r.getMinY(), (float) r.getMaxX(), (float) r.getMaxY());
  }
  public void transform(Matrix m, Path dst) {
    Path2D.Float o = project(p, m);
    if (dst == null) p = o; else dst.p = o;
  }
  public void transform(Matrix m) { transform(m, null); }

  // every point of s through m (control points too), as a new Path2D
  public static Path2D.Float project(java.awt.Shape s, Matrix m) {
    Path2D.Float out = new Path2D.Float(s instanceof Path2D ? ((Path2D) s).getWindingRule() : Path2D.WIND_NON_ZERO);
    float[] c = new float[6], q = new float[6];
    for (PathIterator it = s.getPathIterator(null); !it.isDone(); it.next()) {
      int t = it.currentSegment(c);
      int n = t == PathIterator.SEG_CUBICTO ? 3 : t == PathIterator.SEG_QUADTO ? 2 : t == PathIterator.SEG_CLOSE ? 0 : 1;
      m.mapPoints(q, 0, c, 0, n);
      switch (t) {
        case PathIterator.SEG_MOVETO: out.moveTo(q[0], q[1]); break;
        case PathIterator.SEG_LINETO: out.lineTo(q[0], q[1]); break;
        case PathIterator.SEG_QUADTO: out.quadTo(q[0], q[1], q[2], q[3]); break;
        case PathIterator.SEG_CUBICTO: out.curveTo(q[0], q[1], q[2], q[3], q[4], q[5]); break;
        default: out.closePath();
      }
    }
    return out;
  }
}
