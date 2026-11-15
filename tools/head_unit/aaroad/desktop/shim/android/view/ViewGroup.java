package android.view;

import android.content.Context;

public class ViewGroup extends View {
  public static class LayoutParams {
    public static final int MATCH_PARENT = -1, WRAP_CONTENT = -2;
    public LayoutParams(int w, int h) {}
  }
  public ViewGroup(Context c) { super(c); }
  public void addView(View v, int index, LayoutParams p) {}
  public void addView(View v, LayoutParams p) {}
  public void addView(View v) {}
  public void removeView(View v) {}
  public View findViewById(int id) { return null; }
}
