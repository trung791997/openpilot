package android.view;

import android.content.Context;
import android.content.res.Resources;
import android.graphics.Canvas;
import android.graphics.Rect;

// Just enough View for MapView: a size the harness sets, and no looper (posts are dropped; the harness runs the tick).
public class View {
  public static final int VISIBLE = 0, INVISIBLE = 4, GONE = 8, NO_ID = -1, LAYER_TYPE_SOFTWARE = 1, LAYER_TYPE_NONE = 0;
  private final Context context;
  private int w, h, visibility;

  public View(Context c) { context = c; }
  public Context getContext() { return context; }
  public Resources getResources() { return context.getResources(); }
  public ViewParent getParent() { return null; }
  public int getWindowVisibility() { return VISIBLE; }
  private float alpha = 1f;
  public float getAlpha() { return alpha; }
  public void setAlpha(float a) { alpha = a; }
  public boolean isHardwareAccelerated() { return false; }
  public int getTop() { return 0; }
  public int getBottom() { return h; }
  public int getId() { return NO_ID; }
  public int getWidth() { return w; }
  public int getHeight() { return h; }
  public void shimSize(int width, int height) { w = width; h = height; }
  public void setLayerType(int t, android.graphics.Paint p) {}
  public void setVisibility(int v) { visibility = v; }
  public int getVisibility() { return visibility; }
  public boolean isShown() { return true; }
  public Object getWindowToken() { return this; }
  public void invalidate() {}
  public void invalidate(Rect r) {}
  public void postInvalidate() {}
  public boolean post(Runnable r) { return true; }
  public boolean postDelayed(Runnable r, long ms) { return true; }
  public boolean removeCallbacks(Runnable r) { return true; }
  protected void onDraw(Canvas c) {}
  protected void onAttachedToWindow() {}
  protected void onDetachedFromWindow() {}
  public void draw(Canvas c) { onDraw(c); }
}
