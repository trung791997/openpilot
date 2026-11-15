package android.graphics;

import java.awt.Font;
import java.awt.font.FontRenderContext;

public class Paint {
  public static final int ANTI_ALIAS_FLAG = 1, FILTER_BITMAP_FLAG = 2, DITHER_FLAG = 4, SUBPIXEL_TEXT_FLAG = 128;
  public enum Style { FILL, STROKE, FILL_AND_STROKE }
  public enum Cap { BUTT, ROUND, SQUARE }
  public enum Join { MITER, ROUND, BEVEL }
  public enum Align { LEFT, CENTER, RIGHT }
  public static class FontMetrics { public float top, ascent, descent, bottom, leading; }

  public int flags, color = 0xff000000;
  public Style style = Style.FILL;
  public Cap cap = Cap.BUTT;
  public Join join = Join.MITER;
  public Align align = Align.LEFT;
  public float strokeWidth, miter = 4, textSize = 12;
  public Shader shader;
  public Typeface face = Typeface.DEFAULT;

  public Paint() {}
  public Paint(int flags) { this.flags = flags; }
  public Paint(Paint o) { set(o); }
  public void set(Paint o) {
    flags = o.flags; color = o.color; style = o.style; cap = o.cap; join = o.join; align = o.align;
    strokeWidth = o.strokeWidth; miter = o.miter; textSize = o.textSize; shader = o.shader; face = o.face;
  }
  public void setAntiAlias(boolean a) { flags = a ? flags | ANTI_ALIAS_FLAG : flags & ~ANTI_ALIAS_FLAG; }
  public boolean isAntiAlias() { return (flags & ANTI_ALIAS_FLAG) != 0; }
  public void setFilterBitmap(boolean a) { flags = a ? flags | FILTER_BITMAP_FLAG : flags & ~FILTER_BITMAP_FLAG; }
  public boolean fakeBold;
  public void setFakeBoldText(boolean b) { fakeBold = b; }
  public void setDither(boolean d) {}
  public void setColor(int c) { color = c; }
  public int getColor() { return color; }
  public void setAlpha(int a) { color = (a << 24) | (color & 0xffffff); }
  public int getAlpha() { return color >>> 24; }
  public void setStyle(Style s) { style = s; }
  public Style getStyle() { return style; }
  public void setStrokeWidth(float w) { strokeWidth = w; }
  public float getStrokeWidth() { return strokeWidth; }
  public void setStrokeCap(Cap c) { cap = c; }
  public void setStrokeJoin(Join j) { join = j; }
  public void setStrokeMiter(float m) { miter = m; }
  public Shader setShader(Shader s) { shader = s; return s; }
  public void setTextSize(float s) { textSize = s; }
  public float getTextSize() { return textSize; }
  public void setTextAlign(Align a) { align = a; }
  public Typeface setTypeface(Typeface t) { face = t == null ? Typeface.DEFAULT : t; return t; }
  public Typeface getTypeface() { return face; }

  public Font font() { return face.font.deriveFont(textSize); }
  private static final FontRenderContext FRC = new FontRenderContext(null, true, true);
  public float measureText(String s) { return s.isEmpty() ? 0 : (float) font().getStringBounds(s, FRC).getWidth(); }
  public float measureText(CharSequence s, int a, int b) { return measureText(s.subSequence(a, b).toString()); }
  public float measureText(String s, int a, int b) { return measureText(s.substring(a, b)); }
  public float ascent() { return -font().getLineMetrics("Hg", FRC).getAscent(); }
  public float descent() { return font().getLineMetrics("Hg", FRC).getDescent(); }
  public float getFontSpacing() { return font().getLineMetrics("Hg", FRC).getHeight(); }
  public float getFontMetrics(FontMetrics m) {
    java.awt.font.LineMetrics l = font().getLineMetrics("Hg", FRC);
    m.ascent = -l.getAscent(); m.descent = l.getDescent(); m.top = m.ascent; m.bottom = m.descent; m.leading = l.getLeading();
    return l.getHeight();
  }
  public FontMetrics getFontMetrics() { FontMetrics m = new FontMetrics(); getFontMetrics(m); return m; }
  public void getTextBounds(String s, int a, int b, Rect r) {
    java.awt.geom.Rectangle2D bb = font().createGlyphVector(FRC, s.substring(a, b)).getPixelBounds(FRC, 0, 0);
    r.set((int) bb.getMinX(), (int) bb.getMinY(), (int) bb.getMaxX(), (int) bb.getMaxY());
  }
  public int breakText(String s, boolean forward, float max, float[] measured) {
    int n = 0;
    while (n < s.length() && measureText(s.substring(0, n + 1)) <= max) n++;
    if (measured != null) measured[0] = measureText(s.substring(0, n));
    return n;
  }
}
