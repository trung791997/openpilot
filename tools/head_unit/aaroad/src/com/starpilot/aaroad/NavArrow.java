package com.starpilot.aaroad;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import de.robv.android.xposed.XposedBridge;
import java.io.InputStream;
import java.util.Enumeration;
import java.util.HashSet;
import java.util.Locale;
import java.util.Set;
import java.util.zip.ZipEntry;
import java.util.zip.ZipFile;

// The turn card's arrow: the same picture the StarPilot Auto car screen draws for the same maneuver.
// build.sh copies the car screen's arrows (openpilot/starpilot/ui/assets/navigation, the files its
// navigation-assets.json lists) into the apk under assets/nav/, and Turn.icon is navigation_icons.icon_name in
// Java, so both screens pick the same file.
final class NavArrow {
  static final String DIR = "assets/nav/";

  private static Set<String> have;
  private static String shownName;
  private static Bitmap shown;

  // The arrow for a maneuver, read from the module apk the first time it is asked for; the last one is kept, as the
  // next maneuver changes a few times a minute at most. null if the apk has no arrows (an older build) or on any error.
  static synchronized Bitmap get(String type, String modifier) {
    String path = MapView.modulePath;
    if (path == null) return null;
    ZipFile zip = null;
    String n = null;
    try {
      if (have == null) {
        have = new HashSet<String>();
        zip = new ZipFile(path);
        for (Enumeration<? extends ZipEntry> e = zip.entries(); e.hasMoreElements(); ) {
          String entry = e.nextElement().getName();
          if (entry.startsWith(DIR) && entry.toLowerCase(Locale.US).endsWith(".png")) have.add(entry.substring(DIR.length()));
        }
        if (have.isEmpty()) XposedBridge.log("aaroad: no turn arrows in the module apk");
      }
      if (have.isEmpty()) return null;
      n = Turn.icon(type, modifier, have);
      if (n.equals(shownName)) return shown;
      if (zip == null) zip = new ZipFile(path);
      InputStream in = zip.getInputStream(zip.getEntry(DIR + n));
      Bitmap b = BitmapFactory.decodeStream(in);
      in.close();
      shownName = n;
      shown = b;
      return b;
    } catch (Throwable t) {
      XposedBridge.log("aaroad: turn arrow not loaded: " + t);  // once: the failed name is kept as shown, with no picture
      if (have == null) have = new HashSet<String>();
      shownName = n;
      shown = null;
      return null;
    } finally {
      try {
        if (zip != null) zip.close();
      } catch (Throwable ignored) {
      }
    }
  }

  private NavArrow() {}
}
