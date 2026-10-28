package com.starpilot.aaroad;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Rect;
import android.graphics.RectF;
import android.view.View;
import android.widget.ImageView;
import de.robv.android.xposed.XC_MethodHook;
import de.robv.android.xposed.XposedBridge;
import de.robv.android.xposed.XposedHelpers;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.List;
import java.util.WeakHashMap;

// Honda Hack's screen cast copies the whole 800x480 head-unit screen as a 400x240 frame and
// draws it fitXY into the Civic cluster's 584x191 cast area (meter_civic.xml), so it is shown
// ~1.8x too wide. Swap the frame for one cut to the cast area's aspect: the chosen screen
// regions side by side, each kept at its own aspect, centered on black.
//
// Regions come from /data/local/tmp/aacast.txt, re-read every 2 s so they can be tuned live:
// "card", "map", "both", "full", or "x,y,w,h;x,y,w,h" in 800x480 screen pixels. Missing file
// means "card". Any failure leaves Honda Hack's frame untouched.
final class Cast {
  private static final String CONFIG = "/data/local/tmp/aacast.txt";
  private static final int[] CARD = {532, 122, 244, 88};   // StarPilot nav card, 2026-10-28 screenshot
  private static final int[] MAP = {520, 212, 272, 168};   // map under the card
  private static final int[] FULL = {0, 0, 800, 480};
  private static final int FRAME_W = 400, FRAME_H = 240;   // Honda Hack frame, half of the screen

  private static boolean installed;
  private static final WeakHashMap<View, Boolean> isCast = new WeakHashMap<View, Boolean>();
  private static List<int[]> regions = parse("card");
  private static long checkedAt, configTime = -1;
  private static Bitmap out;
  private static Canvas canvas;
  private static final Paint paint = new Paint(Paint.FILTER_BITMAP_FLAG);
  private static boolean logged;

  static synchronized void install(String pkg) {
    if (installed) return;
    installed = true;
    XposedHelpers.findAndHookMethod(ImageView.class, "setImageBitmap", Bitmap.class, new XC_MethodHook() {
      @Override
      protected void beforeHookedMethod(MethodHookParam param) {
        Bitmap frame = (Bitmap) param.args[0];
        if (frame == null || frame.getWidth() != FRAME_W || frame.getHeight() != FRAME_H) return;
        try {
          ImageView view = (ImageView) param.thisObject;
          if (!castView(view)) return;
          param.args[0] = crop(frame, view);
          if (!logged) {
            logged = true;
            XposedBridge.log("aaroad: cluster cast cropped to " + describe() + " in " + pkg);
          }
        } catch (Throwable t) {
          XposedBridge.log("aaroad: cast crop failed: " + t);
        }
      }
    });
  }

  private static boolean castView(View view) {
    Boolean known = isCast.get(view);
    if (known == null) {
      known = false;
      try {
        known = view.getId() != View.NO_ID && "screen_cast".equals(view.getResources().getResourceEntryName(view.getId()));
      } catch (Throwable ignored) {
      }
      isCast.put(view, known);
    }
    return known;
  }

  private static Bitmap crop(Bitmap frame, View view) {
    reload();
    int w = view.getWidth() > 0 ? view.getWidth() : 584, h = view.getHeight() > 0 ? view.getHeight() : 191;
    if (out == null || out.getWidth() != w || out.getHeight() != h) {
      out = Bitmap.createBitmap(w, h, Bitmap.Config.ARGB_8888);
      canvas = new Canvas(out);
    }
    canvas.drawColor(Color.BLACK);
    float total = 0;
    for (int[] r : regions) total += (float) r[2] * h / r[3];
    float scale = Math.min(1f, w / total), x = (w - total * scale) / 2f, dh = h * scale, y = (h - dh) / 2f;
    for (int[] r : regions) {
      float dw = (float) r[2] * h / r[3] * scale;
      Rect src = new Rect(r[0] / 2, r[1] / 2, (r[0] + r[2]) / 2, (r[1] + r[3]) / 2);
      canvas.drawBitmap(frame, src, new RectF(x, y, x + dw, y + dh), paint);
      x += dw;
    }
    return out;
  }

  private static void reload() {
    long now = System.currentTimeMillis();
    if (now - checkedAt < 2000) return;
    checkedAt = now;
    File f = new File(CONFIG);
    long t = f.exists() ? f.lastModified() : 0;
    if (t == configTime) return;
    configTime = t;
    String text = "card";
    if (t != 0) {
      try {
        BufferedReader in = new BufferedReader(new FileReader(f));
        try {
          String line = in.readLine();
          if (line != null && line.trim().length() > 0) text = line.trim();
        } finally {
          in.close();
        }
      } catch (Throwable ignored) {
      }
    }
    List<int[]> next = parse(text);
    if (next.isEmpty()) next = parse("card");
    regions = next;
    XposedBridge.log("aaroad: cast regions " + describe());
  }

  static List<int[]> parse(String text) {
    List<int[]> list = new ArrayList<int[]>();
    if (text.equals("card")) list.add(CARD);
    else if (text.equals("map")) list.add(MAP);
    else if (text.equals("both")) { list.add(CARD); list.add(MAP); }
    else if (text.equals("full")) list.add(FULL);
    else {
      for (String part : text.split(";")) {
        String[] n = part.trim().split("\\s*,\\s*");
        if (n.length != 4) return new ArrayList<int[]>();
        try {
          int[] r = {Integer.parseInt(n[0]), Integer.parseInt(n[1]), Integer.parseInt(n[2]), Integer.parseInt(n[3])};
          if (r[0] < 0 || r[1] < 0 || r[2] < 2 || r[3] < 2 || r[0] + r[2] > 800 || r[1] + r[3] > 480) return new ArrayList<int[]>();
          list.add(r);
        } catch (NumberFormatException e) {
          return new ArrayList<int[]>();
        }
      }
    }
    return list;
  }

  private static String describe() {
    StringBuilder sb = new StringBuilder();
    for (int[] r : regions) sb.append(sb.length() > 0 ? ";" : "").append(r[0]).append(',').append(r[1]).append(',').append(r[2]).append(',').append(r[3]);
    return sb.toString();
  }

  private Cast() {}
}
