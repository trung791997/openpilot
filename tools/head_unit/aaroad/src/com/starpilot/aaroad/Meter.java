package com.starpilot.aaroad;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.view.Gravity;
import android.view.View;
import android.view.ViewGroup;
import android.widget.RelativeLayout;
import android.widget.TextView;
import de.robv.android.xposed.XC_MethodHook;
import de.robv.android.xposed.XposedBridge;
import de.robv.android.xposed.XposedHelpers;
import java.util.WeakHashMap;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

// Honda Hack's Civic meter (meter_civic.xml) puts the nav distance and street in a 180 px column at
// the top left, where the street's first letter ran under the speedometer ring (2026-10-28 photo).
// Lay it out like the stock nav direction page (2026-10-29 photo) instead: Honda Hack's arrow stays
// in the middle, a segmented distance bar sits left of it with the distance above the bar, and the
// street (which stock does not show) goes centered under the arrow. During navigation Honda Hack
// moves the compass to the bottom right corner.
//
// The bar shows from 500 ft: one of ten segments per 50 ft left, empty at 0 ft, hidden above
// 500 ft and whenever Honda Hack hides the distance (no distance yet for a new turn). It is read
// back from the distance text, so it steps once per Android Auto distance update (about 1 s).
//
// Rearranged once per meter view, on the first text set into seg_dist or next_road; Honda Hack
// re-creates the meter when its settings change. Any failure leaves its layout untouched.
final class Meter {
  private static final int STREET_W = 300;  // px, of the 484 px between navi_layout's paddings
  private static final int BAR_W = 28, BAR_H = 64, BAR_LEFT = 132;  // bar right edge 20 px left of the arrow column
  private static final int SEGMENTS = 10;
  private static final double FT_PER_SEGMENT = 50, FEET_PER_METER = 3.28084;
  private static final Pattern DIST = Pattern.compile("^\\s*([0-9]+(?:\\.[0-9]+)?)\\s*(ft|mi|m|km)\\b");

  private static boolean installed;
  private static final WeakHashMap<View, String> names = new WeakHashMap<View, String>();
  private static final WeakHashMap<View, Boolean> arranged = new WeakHashMap<View, Boolean>();
  private static final WeakHashMap<View, Bar> bars = new WeakHashMap<View, Bar>();

  static synchronized void install(String process) {
    if (installed || process == null || !process.contains("externaldisplay")) return;
    installed = true;
    XposedHelpers.findAndHookMethod(TextView.class, "setText", CharSequence.class, TextView.BufferType.class,
        new XC_MethodHook() {
      @Override
      protected void afterHookedMethod(MethodHookParam param) {
        TextView v = (TextView) param.thisObject;
        try {
          String name = name(v);
          if (!"seg_dist".equals(name) && !"next_road".equals(name)) return;
          if (v.getParent() instanceof RelativeLayout) arrange((RelativeLayout) v.getParent(), v);
          if ("seg_dist".equals(name)) update(v);
        } catch (Throwable t) {
          XposedBridge.log("aaroad: meter layout failed: " + t);
        }
      }
    });
    XposedHelpers.findAndHookMethod(View.class, "setVisibility", int.class, new XC_MethodHook() {
      @Override
      protected void afterHookedMethod(MethodHookParam param) {
        if (bars.isEmpty() || !bars.containsKey(param.thisObject)) return;
        try {
          update((TextView) param.thisObject);
        } catch (Throwable t) {
          XposedBridge.log("aaroad: meter bar failed: " + t);
        }
      }
    });
  }

  private static void arrange(RelativeLayout parent, View any) {
    if (arranged.containsKey(parent)) return;
    arranged.put(parent, Boolean.TRUE);
    String pkg = any.getResources().getResourcePackageName(any.getId());
    View icon = find(parent, pkg, "turn_icon"), dist = find(parent, pkg, "seg_dist"), road = find(parent, pkg, "next_road");
    if (!(dist instanceof TextView) || !(road instanceof TextView) || icon == null) return;

    Bar bar = new Bar(parent.getContext());
    bar.setId(View.generateViewId());
    bar.setVisibility(View.INVISIBLE);
    RelativeLayout.LayoutParams b = new RelativeLayout.LayoutParams(BAR_W, BAR_H);
    b.addRule(RelativeLayout.ALIGN_PARENT_LEFT);
    b.addRule(RelativeLayout.ALIGN_BOTTOM, icon.getId());
    b.leftMargin = BAR_LEFT;
    b.bottomMargin = 16;  // turn_icon's padding
    parent.addView(bar, b);
    bars.put(dist, bar);

    RelativeLayout.LayoutParams d = params(dist);
    d.width = ViewGroup.LayoutParams.WRAP_CONTENT;
    d.addRule(RelativeLayout.ABOVE, bar.getId());
    d.addRule(RelativeLayout.ALIGN_RIGHT, bar.getId());
    d.setMargins(0, 0, 0, 2);
    ((TextView) dist).setTextSize(android.util.TypedValue.COMPLEX_UNIT_PX, 36);
    dist.setLayoutParams(d);

    RelativeLayout.LayoutParams r = params(road);
    r.addRule(RelativeLayout.BELOW, 0);
    r.addRule(RelativeLayout.ALIGN_PARENT_LEFT, 0);
    r.addRule(RelativeLayout.ALIGN_PARENT_BOTTOM);
    r.addRule(RelativeLayout.CENTER_HORIZONTAL);
    r.width = STREET_W;
    r.setMargins(0, 0, 0, 0);
    ((TextView) road).setGravity(Gravity.CENTER_HORIZONTAL);
    road.setTextAlignment(View.TEXT_ALIGNMENT_CENTER);
    road.setLayoutParams(r);
    XposedBridge.log("aaroad: meter nav laid out like the stock page");
  }

  private static void update(TextView dist) {
    Bar bar = bars.get(dist);
    if (bar == null) return;
    double ft = dist.getVisibility() == View.VISIBLE ? feet(dist.getText()) : -1;
    if (ft < 0 || ft > SEGMENTS * FT_PER_SEGMENT) {
      bar.setVisibility(View.INVISIBLE);
      return;
    }
    bar.lit = (int) Math.min(SEGMENTS, Math.round(ft / FT_PER_SEGMENT));
    bar.setVisibility(View.VISIBLE);
    bar.invalidate();
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

  // Stock look: white end caps, ten stacked segments, lit white from the bottom, the rest gray.
  static final class Bar extends View {
    int lit;
    private final Paint paint = new Paint();

    Bar(Context c) {
      super(c);
    }

    @Override
    protected void onDraw(Canvas canvas) {
      int w = getWidth(), h = getHeight(), cap = 3, gap = 2;
      paint.setColor(Color.WHITE);
      canvas.drawRect(0, 0, w, cap, paint);
      canvas.drawRect(0, h - cap, w, h, paint);
      float seg = (h - 2 * cap - (SEGMENTS + 1) * gap) / (float) SEGMENTS;
      for (int i = 0; i < SEGMENTS; i++) {
        float bottom = h - cap - gap - i * (seg + gap);
        paint.setColor(i < lit ? Color.WHITE : Color.rgb(90, 90, 90));
        canvas.drawRect(2, bottom - seg, w - 2, bottom, paint);
      }
    }
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
