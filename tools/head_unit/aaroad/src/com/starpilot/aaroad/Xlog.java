package com.starpilot.aaroad;

import de.robv.android.xposed.XposedBridge;

// aaroad's lines to Honda Hack's error.log (and logcat) through XposedBridge.log, with the wall-clock time in front
// (Events.stamp), so a line there can be matched to the drive (2026-11-01: they had none).
final class Xlog {
  static void log(String line) {
    XposedBridge.log(Events.stamp(line));
  }

  // A failure with its stack: a stamped line, then the trace as XposedBridge writes it.
  static void log(Throwable t) {
    XposedBridge.log(Events.stamp("aaroad: " + t));
    XposedBridge.log(t);
  }

  private Xlog() {}
}
