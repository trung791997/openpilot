package android.graphics;

import java.awt.image.BufferedImage;
import java.io.ByteArrayInputStream;
import java.io.InputStream;
import javax.imageio.ImageIO;

public class BitmapFactory {
  public static Bitmap decodeStream(InputStream in) {
    try {
      return of(ImageIO.read(in));
    } catch (Exception e) {
      return null;
    }
  }
  public static Bitmap decodeByteArray(byte[] b, int off, int len) {
    return decodeStream(new ByteArrayInputStream(b, off, len));
  }
  private static Bitmap of(BufferedImage src) {
    if (src == null) return null;
    BufferedImage out = new BufferedImage(src.getWidth(), src.getHeight(), BufferedImage.TYPE_INT_ARGB);
    java.awt.Graphics2D g = out.createGraphics();
    g.drawImage(src, 0, 0, null);
    g.dispose();
    return new Bitmap(out);
  }
}
