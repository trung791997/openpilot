package android.graphics;

public class RectF {
  public float left, top, right, bottom;
  public RectF() {}
  public RectF(float l, float t, float r, float b) { left = l; top = t; right = r; bottom = b; }
  public RectF(RectF o) { set(o); }
  public RectF(Rect o) { left = o.left; top = o.top; right = o.right; bottom = o.bottom; }
  public final float width() { return right - left; }
  public final float height() { return bottom - top; }
  public final float centerX() { return (left + right) * 0.5f; }
  public final float centerY() { return (top + bottom) * 0.5f; }
  public final boolean isEmpty() { return left >= right || top >= bottom; }
  public void set(float l, float t, float r, float b) { left = l; top = t; right = r; bottom = b; }
  public void set(RectF o) { left = o.left; top = o.top; right = o.right; bottom = o.bottom; }
  public void setEmpty() { left = right = top = bottom = 0; }
  public void offset(float dx, float dy) { left += dx; right += dx; top += dy; bottom += dy; }
  public void inset(float dx, float dy) { left += dx; right -= dx; top += dy; bottom -= dy; }
  public boolean contains(float x, float y) { return left < right && top < bottom && x >= left && x < right && y >= top && y < bottom; }
  public boolean intersects(float l, float t, float r, float b) { return left < r && l < right && top < b && t < bottom; }
  public boolean intersect(float l, float t, float r, float b) {
    if (left < r && l < right && top < b && t < bottom) {
      if (left < l) left = l;
      if (top < t) top = t;
      if (right > r) right = r;
      if (bottom > b) bottom = b;
      return true;
    }
    return false;
  }
  public boolean intersect(RectF o) { return intersect(o.left, o.top, o.right, o.bottom); }
  public void union(float l, float t, float r, float b) {
    if (l < r && t < b) {
      if (left < right && top < bottom) {
        if (left > l) left = l;
        if (top > t) top = t;
        if (right < r) right = r;
        if (bottom < b) bottom = b;
      } else {
        left = l; top = t; right = r; bottom = b;
      }
    }
  }
  public void union(RectF o) { union(o.left, o.top, o.right, o.bottom); }
  public void roundOut(Rect r) { r.set((int) Math.floor(left), (int) Math.floor(top), (int) Math.ceil(right), (int) Math.ceil(bottom)); }
  public String toString() { return "RectF(" + left + ", " + top + ", " + right + ", " + bottom + ")"; }
}
