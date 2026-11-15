package android.graphics;

public class LinearGradient extends Shader {
  public final float x0, y0, x1, y1;
  public final int[] colors;
  public final float[] pos;
  public final TileMode mode;
  public LinearGradient(float x0, float y0, float x1, float y1, int[] colors, float[] pos, TileMode mode) {
    this.x0 = x0; this.y0 = y0; this.x1 = x1; this.y1 = y1; this.colors = colors; this.pos = pos; this.mode = mode;
  }
  public LinearGradient(float x0, float y0, float x1, float y1, int c0, int c1, TileMode mode) {
    this(x0, y0, x1, y1, new int[] {c0, c1}, null, mode);
  }
}
