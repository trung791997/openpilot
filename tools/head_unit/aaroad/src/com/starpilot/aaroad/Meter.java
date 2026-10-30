package com.starpilot.aaroad;

import android.view.View;
import android.view.ViewGroup;
import android.view.ViewParent;
import android.widget.TextView;
import de.robv.android.xposed.XC_MethodHook;
import de.robv.android.xposed.XposedBridge;
import de.robv.android.xposed.XposedHelpers;
import java.util.WeakHashMap;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

// Honda Hack's Civic meter (meter_civic.xml) puts the nav distance and street in a 180 px column at
// the top left, where the street's first letter ran under the speedometer ring (2026-10-28 photo).
// The 2026-10-28 rearrangement never ran: seg_dist and next_road sit in a LinearLayout (turn_layout),
// not the RelativeLayout it waited for. Replaced by the continuous map (MapView, Layout B of the
// 2026-10-29 mockup), which draws the distance, bar and street itself from Honda Hack's text views.
//
// The map is added behind navi_layout once per meter view, on the first nav text Honda Hack sets;
// Honda Hack re-creates the meter when its settings change. Any failure leaves its layout untouched.
final class Meter {
  private static final String[] NAV_TEXT = {"seg_dist", "next_road", "cur_road", "route_remain"};
  private static final double FEET_PER_METER = 3.28084;
  private static final Pattern DIST = Pattern.compile("^\\s*([0-9]+(?:\\.[0-9]+)?)\\s*(ft|mi|m|km)\\b");

  private static boolean installed;
  private static final WeakHashMap<View, String> names = new WeakHashMap<View, String>();
  private static final WeakHashMap<View, Boolean> mapped = new WeakHashMap<View, Boolean>();

  static synchronized void install(String process) {
    if (installed || process == null || !process.contains("externaldisplay")) return;
    installed = true;
    XposedHelpers.findAndHookMethod(TextView.class, "setText", CharSequence.class, TextView.BufferType.class,
        new XC_MethodHook() {
      @Override
      protected void afterHookedMethod(MethodHookParam param) {
        TextView v = (TextView) param.thisObject;
        try {
          if (!navText(name(v))) return;
          ViewGroup navi = navi(v);
          if (navi == null || mapped.containsKey(navi)) return;
          mapped.put(navi, Boolean.TRUE);
          if (!(navi.getParent() instanceof ViewGroup)) return;
          ViewGroup root = (ViewGroup) navi.getParent();
          String pkg = v.getResources().getResourcePackageName(v.getId());
          root.addView(new MapView(root.getContext(), navi, pkg), 0,
              new ViewGroup.LayoutParams(ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.MATCH_PARENT));
          Feed.start();
          GlProbe.startOnce(root.getContext(), root);
          XposedBridge.log("aaroad: meter map added");
        } catch (Throwable t) {
          XposedBridge.log("aaroad: meter map failed: " + t);
        }
      }
    });
  }

  private static boolean navText(String name) {
    if (name == null) return false;
    for (String n : NAV_TEXT) {
      if (n.equals(name)) return true;
    }
    return false;
  }

  private static ViewGroup navi(View v) {
    for (ViewParent p = v.getParent(); p instanceof View; p = p.getParent()) {
      if ("navi_layout".equals(name((View) p))) return (ViewGroup) p;
    }
    return null;
  }

  // Feet in Honda Hack's distance text ("350 ft", "0.3 mi"), or -1.
  static double feet(CharSequence text) {
    if (text == null) return -1;
    Matcher m = DIST.matcher(text.toString());
    if (!m.find()) return -1;
    double n = Double.parseDouble(m.group(1));
    String u = m.group(2);
    return u.equals("ft") ? n : u.equals("mi") ? n * 5280 : u.equals("km") ? n * 1000 * FEET_PER_METER : n * FEET_PER_METER;
  }

  private static String name(View v) {
    if (names.containsKey(v)) return names.get(v);
    String n = null;
    if (v.getId() != View.NO_ID) {
      try {
        n = v.getResources().getResourceEntryName(v.getId());
      } catch (Throwable ignored) {
      }
    }
    names.put(v, n);
    return n;
  }

  private Meter() {}
}
