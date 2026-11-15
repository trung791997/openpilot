package de.robv.android.xposed;

// The Xposed log sink, to stderr; the only non-android class the harness shims (Xlog calls it on every log line).
public final class XposedBridge {
  public static void log(String text) { System.err.println("[xposed] " + text); }
  public static void log(Throwable t) { t.printStackTrace(); }
}
