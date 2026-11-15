package android.graphics;

public class Rect {
  public int left, top, right, bottom;
  public Rect() {}
  public Rect(int l, int t, int r, int b) { left = l; top = t; right = r; bottom = b; }
  public Rect(Rect o) { set(o); }
  public final int width() { return right - left; }
  public final int height() { return bottom - top; }
  public final int centerX() { return (left + right) >> 1; }
  public final int centerY() { return (top + bottom) >> 1; }
  public final boolean isEmpty() { return left >= right || top >= bottom; }
  public void set(int l, int t, int r, int b) { left = l; top = t; right = r; bottom = b; }
  public void set(Rect o) { left = o.left; top = o.top; right = o.right; bottom = o.bottom; }
  public void setEmpty() { left = right = top = bottom = 0; }
  public void offset(int dx, int dy) { left += dx; right += dx; top += dy; bottom += dy; }
  public void inset(int dx, int dy) { left += dx; right -= dx; top += dy; bottom -= dy; }
  public boolean contains(int x, int y) { return left < right && top < bottom && x >= left && x < right && y >= top && y < bottom; }
  public boolean intersects(int l, int t, int r, int b) { return left < r && l < right && top < b && t < bottom; }
  public boolean intersect(int l, int t, int r, int b) {
    if (left < r && l < right && top < b && t < bottom) {
      if (left < l) left = l;
      if (top < t) top = t;
      if (right > r) right = r;
      if (bottom > b) bottom = b;
      return true;
    }
    return false;
  }
  public boolean intersect(Rect o) { return intersect(o.left, o.top, o.right, o.bottom); }
  public void union(int l, int t, int r, int b) {
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
  public void union(Rect o) { union(o.left, o.top, o.right, o.bottom); }
  public String toString() { return "Rect(" + left + ", " + top + ", " + right + ", " + bottom + ")"; }
}
