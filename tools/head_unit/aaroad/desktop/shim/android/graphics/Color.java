package android.graphics;

public class Color {
  public static final int BLACK = 0xff000000, WHITE = 0xffffffff, TRANSPARENT = 0, RED = 0xffff0000, GREEN = 0xff00ff00, BLUE = 0xff0000ff,
      GRAY = 0xff888888, DKGRAY = 0xff444444, LTGRAY = 0xffcccccc, YELLOW = 0xffffff00, CYAN = 0xff00ffff, MAGENTA = 0xffff00ff;
  public static int rgb(int r, int g, int b) { return 0xff000000 | (r << 16) | (g << 8) | b; }
  public static int argb(int a, int r, int g, int b) { return (a << 24) | (r << 16) | (g << 8) | b; }
  public static int alpha(int c) { return c >>> 24; }
  public static int red(int c) { return (c >> 16) & 255; }
  public static int green(int c) { return (c >> 8) & 255; }
  public static int blue(int c) { return c & 255; }
}
