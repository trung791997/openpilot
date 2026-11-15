package android.graphics;

import java.awt.image.BufferedImage;

public class Bitmap {
  public enum Config { ALPHA_8, RGB_565, ARGB_4444, ARGB_8888 }
  public final BufferedImage img;
  private boolean recycled;
  public Bitmap(BufferedImage i) { img = i; }
  public static Bitmap createBitmap(int w, int h, Config c) { return new Bitmap(new BufferedImage(w, h, BufferedImage.TYPE_INT_ARGB)); }
  public static Bitmap createBitmap(int[] colors, int w, int h, Config c) {
    Bitmap b = createBitmap(w, h, c);
    b.img.setRGB(0, 0, w, h, colors, 0, w);
    return b;
  }
  public int getWidth() { return img.getWidth(); }
  public int getHeight() { return img.getHeight(); }
  public void getPixels(int[] px, int offset, int stride, int x, int y, int w, int h) { img.getRGB(x, y, w, h, px, offset, stride); }
  public void setPixels(int[] px, int offset, int stride, int x, int y, int w, int h) { img.setRGB(x, y, w, h, px, offset, stride); }
  public int getPixel(int x, int y) { return img.getRGB(x, y); }
  public void setPixel(int x, int y, int c) { img.setRGB(x, y, c); }
  public void eraseColor(int c) {
    java.awt.Graphics2D g = img.createGraphics();
    g.setComposite(java.awt.AlphaComposite.Src);
    g.setColor(new java.awt.Color(c, true));
    g.fillRect(0, 0, img.getWidth(), img.getHeight());
    g.dispose();
  }
  public void recycle() { recycled = true; }
  public boolean isRecycled() { return recycled; }
}
