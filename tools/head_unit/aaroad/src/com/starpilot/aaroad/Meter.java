package com.starpilot.aaroad;

import android.view.Gravity;
import android.view.View;
import android.view.ViewGroup;
import android.widget.RelativeLayout;
import android.widget.TextView;
import de.robv.android.xposed.XC_MethodHook;
import de.robv.android.xposed.XposedBridge;
import de.robv.android.xposed.XposedHelpers;
import java.util.WeakHashMap;

// Honda Hack's Civic meter (meter_civic.xml) puts the nav distance and street in a narrow column
// at the top left and the arrow in the middle. Stack them the way the stock nav direction page
// does instead: arrow on top, distance centered under it, street centered under that. During
// navigation Honda Hack moves the compass to the bottom right corner, so the center is free.
//
// Rearranged once per meter view, on the first text set into seg_dist or next_road; Honda Hack
// re-creates the meter when its settings change. Any failure leaves its layout untouched.
final class Meter {
  // px, of the 484 px between navi_layout's paddings. Honda Hack's 180 px column at the left padding
  // ran the street's first letter into the speedometer ring (2026-10-28 photo); keep text off both rings.
  private static final int TEXT_W = 300;

  private static boolean installed;
  private static final WeakHashMap<View, Boolean> seen = new WeakHashMap<View, Boolean>();

  static synchronized void install(String process) {
    if (installed || process == null || !process.contains("externaldisplay")) return;
    installed = true;
    XposedHelpers.findAndHookMethod(TextView.class, "setText", CharSequence.class, TextView.BufferType.class,
        new XC_MethodHook() {
      @Override
      protected void afterHookedMethod(MethodHookParam param) {
        View v = (View) param.thisObject;
        if (seen.containsKey(v)) return;
        seen.put(v, Boolean.TRUE);
        try {
          String name = name(v);
          if (("seg_dist".equals(name) || "next_road".equals(name)) && v.getParent() instanceof RelativeLayout) {
            arrange((RelativeLayout) v.getParent(), v);
          }
        } catch (Throwable t) {
          XposedBridge.log("aaroad: meter layout failed: " + t);
        }
      }
    });
  }

  private static void arrange(RelativeLayout parent, View any) {
    if (seen.containsKey(parent)) return;
    seen.put(parent, Boolean.TRUE);
    String pkg = any.getResources().getResourcePackageName(any.getId());
    View icon = find(parent, pkg, "turn_icon"), dist = find(parent, pkg, "seg_dist"), road = find(parent, pkg, "next_road");
    if (!(dist instanceof TextView) || !(road instanceof TextView) || icon == null) return;

    RelativeLayout.LayoutParams r = params(road);
    r.addRule(RelativeLayout.BELOW, 0);
    r.addRule(RelativeLayout.ALIGN_PARENT_LEFT, 0);
    r.addRule(RelativeLayout.ALIGN_PARENT_BOTTOM);
    r.addRule(RelativeLayout.CENTER_HORIZONTAL);
    r.width = TEXT_W;
    r.setMargins(0, 0, 0, 4);
    center((TextView) road);
    road.setLayoutParams(r);

    RelativeLayout.LayoutParams d = params(dist);
    d.addRule(RelativeLayout.ABOVE, road.getId());
    d.addRule(RelativeLayout.CENTER_HORIZONTAL);
    d.width = TEXT_W;
    d.setMargins(0, 0, 0, -6);
    center((TextView) dist);
    dist.setLayoutParams(d);

    RelativeLayout.LayoutParams i = params(icon);
    i.addRule(RelativeLayout.ABOVE, dist.getId());
    i.topMargin = 0;
    icon.setPadding(4, 4, 4, 4);
    icon.setLayoutParams(i);
    XposedBridge.log("aaroad: meter nav laid out like the stock page");
  }

  private static void center(TextView t) {
    t.setGravity(Gravity.CENTER_HORIZONTAL);
    t.setTextAlignment(View.TEXT_ALIGNMENT_CENTER);
  }

  private static RelativeLayout.LayoutParams params(View v) {
    ViewGroup.LayoutParams p = v.getLayoutParams();
    return p instanceof RelativeLayout.LayoutParams ? (RelativeLayout.LayoutParams) p
        : new RelativeLayout.LayoutParams(p);
  }

  private static View find(View parent, String pkg, String name) {
    int id = parent.getResources().getIdentifier(name, "id", pkg);
    return id == 0 ? null : parent.findViewById(id);
  }

  private static String name(View v) {
    if (v.getId() == View.NO_ID) return null;
    try {
      return v.getResources().getResourceEntryName(v.getId());
    } catch (Throwable t) {
      return null;
    }
  }

  private Meter() {}
}
