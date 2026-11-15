package android.text;

public class TextUtils {
  public enum TruncateAt { START, MIDDLE, END, MARQUEE }
  public static boolean isEmpty(CharSequence s) { return s == null || s.length() == 0; }
  public static boolean equals(CharSequence a, CharSequence b) {
    if (a == b) return true;
    return a != null && b != null && a.toString().equals(b.toString());
  }
  // END only (all the app uses): the text cut with an ellipsis so it measures at most avail
  public static CharSequence ellipsize(CharSequence text, TextPaint p, float avail, TruncateAt where) {
    String s = text.toString();
    if (p.measureText(s) <= avail) return text;
    String e = "…";
    int n = s.length();
    while (n > 0 && p.measureText(s.substring(0, n) + e) > avail) n--;
    return n == 0 ? "" : s.substring(0, n) + e;
  }
}
