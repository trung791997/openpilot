package android.graphics;

import android.content.res.AssetManager;
import java.awt.Font;
import java.io.InputStream;

public class Typeface {
  public static final Typeface DEFAULT = new Typeface(new Font(Font.SANS_SERIF, Font.PLAIN, 1)),
      DEFAULT_BOLD = new Typeface(new Font(Font.SANS_SERIF, Font.BOLD, 1));
  public final Font font;
  public Typeface(Font f) { font = f; }
  // Real Inter files from assets/fonts when the shim's AssetManager can open them; else the system face.
  public static Typeface createFromAsset(AssetManager am, String path) {
    try (InputStream in = am.open(path)) {
      return new Typeface(Font.createFont(Font.TRUETYPE_FONT, in));
    } catch (Exception e) {
      System.err.println("shim: font " + path + " not loaded: " + e);
      return DEFAULT;
    }
  }
}
