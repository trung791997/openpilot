package com.starpilot.aaroad;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.os.SystemClock;
import android.view.ViewGroup;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.image.BufferedImage;
import java.io.ByteArrayOutputStream;
import java.io.File;
import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.nio.file.Files;
import java.util.Base64;
import javax.imageio.ImageIO;
import org.json.JSONArray;
import org.json.JSONObject;

// Desktop render of the dash map: the app's own MapView.onDraw, software path, fed through Feed's own datagram handler
// (Feed.handle, the one the UDP pump calls) with the synthetic scene desktop/make_scene.py wrote.
//
//   Render <roundabout|uturn|free> <music 0|1> <nav 0|1> <out.png> <scene.json> <module.zip> [metres ahead of the maneuver]
//
// In the package of the app so the package-private classes are reachable; nothing under src/ is changed. The desktop
// shims (android.* on Java2D) are in desktop/shim. Not the Tegra GPU: MapView.GPU is off, as the app's own fallback.
public final class Render {
  static final int W = 584, H = 215;  // the 584x215 canvas the view lays out in (MapView hard-codes 215; Cast later crops it for the meter)
  static final double MPS = 12.0;     // the car's speed along the route, 27 mph
  static final long STEP_MS = 100;

  public static void main(String[] a) throws Exception {
    String scenario = a[0];
    boolean music = a[1].equals("1"), nav = a[2].equals("1");
    File out = new File(a[3]);
    double ahead = a.length > 6 ? Double.parseDouble(a[6]) : 200;
    MapView.modulePath = a[5];
    MapView.GPU = false;  // no GL here: the view draws the roads itself, as it does when the GPU layer fails

    JSONObject scene = new JSONObject(new String(Files.readAllBytes(new File(a[4]).toPath()), "UTF-8"));
    JSONObject sc = scene.getJSONObject("scenarios").getJSONObject(scenario);
    Method handle = Feed.class.getDeclaredMethod("handle", JSONObject.class);
    handle.setAccessible(true);

    File tmp = Files.createTempDirectory("aaroad-render").toFile();
    Context ctx = new Context(tmp);
    MapView view = new MapView(ctx, new ViewGroup(ctx), "com.example.meter");
    view.shimSize(W, H);

    JSONArray tiles = scene.getJSONArray("tiles");
    for (int i = 0; i < tiles.length(); i++) handle.invoke(null, new JSONObject(String.valueOf(tiles.opt(i))));
    
    // the way the car goes: the route's points, or a free drive's track
    JSONArray pts = sc.has("route") ? sc.getJSONArray("route") : sc.getJSONArray("track");
    int n = pts.length();
    double[] la = new double[n], lo = new double[n], cum = new double[n];
    for (int i = 0; i < n; i++) {
      la[i] = pts.getJSONArray(i).getDouble(0);
      lo[i] = pts.getJSONArray(i).getDouble(1);
      if (i > 0) cum[i] = cum[i - 1] + metres(la[i - 1], lo[i - 1], la[i], lo[i]);
    }
    int steps = 45;  // 4.5 s of driving: the camera, zoom and card state settle as they do on the road
    double manS = sc.has("maneuver_s") ? sc.getDouble("maneuver_s") : -1;
    double s0 = manS >= 0 ? Math.max(0, manS - ahead - steps * MPS * STEP_MS / 1000.0) : 40;
    boolean routeSent = false;
    Field tickF = MapView.class.getDeclaredField("tick");
    tickF.setAccessible(true);
    Runnable tick = (Runnable) tickF.get(view);
    Field sceneF = MapView.class.getDeclaredField("scene");
    sceneF.setAccessible(true);

    Bitmap frame = Bitmap.createBitmap(W, H, Bitmap.Config.ARGB_8888);
    Canvas canvas = new Canvas(frame);
    double s = s0;
    int waited = 0;
    for (int i = 0; i <= steps; ) {
      SystemClock.advance(STEP_MS);
      double[] p = at(s, cum, la, lo);
      send(handle, new JSONObject().put("v", 1).put("t", "p").put("lat", p[0]).put("lon", p[1]).put("hdg", p[2]).put("spd", MPS).put("age", 0.0));
      if (nav && !routeSent) {
        JSONArray rp = new JSONArray(), rc = new JSONArray();
        JSONArray lv = sc.getJSONArray("levels");
        for (int k = 0; k < n; k++) {
          rp.put(new JSONArray().put(la[k]).put(lo[k]));
          rc.put(lv.optInt(k, 0));
        }
        send(handle, new JSONObject().put("v", 1).put("t", "r").put("id", 77).put("i", 0).put("n", 1).put("pts", rp).put("c", rc));
        routeSent = true;
      }
      // the music card's title scrolls after a 2 s hold from when it first shows: it starts 1 s before the capture, to catch it at rest
      if (music && i == steps - 10) cover(handle);
      send(handle, state(sc, nav, music && i >= steps - 10, manS - s, cum[n - 1] - s));
      tick.run();
      boolean ready = sceneF.get(view) != null;
      if (!ready) {  // the scene builds on its own thread; let it, without driving on
        if (++waited > 600) throw new IllegalStateException("scene never built");
        Thread.sleep(50);
        continue;
      }
      frame.eraseColor(0xff000000);
      canvas.save();
      view.onDraw(canvas);
      canvas.restore();
      s += MPS * STEP_MS / 1000.0;
      i++;
    }
    System.err.println("render: " + scenario + " music=" + music + " nav=" + nav + " car at " + Math.round(s - MPS * STEP_MS / 1000.0)
        + " m, maneuver in " + (manS < 0 ? "-" : Math.round(manS - (s - MPS * STEP_MS / 1000.0)) + " m"));
    out.getParentFile().mkdirs();
    ImageIO.write(frame.img, "png", out);
    System.exit(0);
  }

  static JSONObject state(JSONObject sc, boolean nav, boolean music, double toManeuver, double remaining) throws Exception {
    JSONObject m = new JSONObject().put("v", 1).put("t", "s").put("mode", nav ? "chill" : "exp").put("metric", false);
    double limit = sc.has("limit") ? sc.getDouble("limit") : 17.9;
    m.put("limit", limit);
    if (nav) {
      JSONObject mv = sc.getJSONObject("maneuver");
      JSONObject n = new JSONObject();
      for (String k : new String[] {"type", "modifier", "text", "road", "ref", "toward"}) n.put(k, mv.optString(k, ""));
      if (Boolean.getBoolean("short")) n.put("text", "Take the 2nd exit").put("toward", "");  // one line, so the music card fits under it
      n.put("exit", mv.optInt("exit", 0)).put("degrees", mv.optInt("degrees", 0)).put("left", mv.optBoolean("left", false));
      n.put("distance", Math.max(0, Math.round(toManeuver * 10) / 10.0)).put("remaining", Math.round(remaining + 5000))
          .put("duration", Math.round((remaining + 5000) / 11.0)).put("limit", limit);
      m.put("nav", n);
    }
    if (music) {
      m.put("media", new JSONObject().put("status", "playing").put("title", "Demo Track").put("artist", "Demo Artist")
          .put("source", "Demo Player").put("art", 4242).put("dur", 215000));
      m.put("pos", 74000);
    }
    return m;
  }

  static void send(Method handle, JSONObject m) throws Exception {
    handle.invoke(null, new JSONObject(m.toString()));  // through the text, as the pump parses a datagram
  }

  // A plain colour square as the cover, through Feed's own 'a' datagrams (JPEG, chunked, base64).
  static void cover(Method handle) throws Exception {
    BufferedImage img = new BufferedImage(56, 56, BufferedImage.TYPE_INT_RGB);
    Graphics2D g = img.createGraphics();
    g.setColor(new Color(0x2f7f9f));
    g.fillRect(0, 0, 56, 56);
    g.dispose();
    ByteArrayOutputStream bo = new ByteArrayOutputStream();
    ImageIO.write(img, "jpg", bo);
    byte[] j = bo.toByteArray();
    int parts = (j.length + 999) / 1000;
    for (int i = 0; i < parts; i++) {
      byte[] c = java.util.Arrays.copyOfRange(j, i * 1000, Math.min(j.length, (i + 1) * 1000));
      send(handle, new JSONObject().put("v", 1).put("t", "a").put("id", 4242).put("i", i).put("n", parts).put("b", Base64.getEncoder().encodeToString(c)));
    }
  }

  static double metres(double la0, double lo0, double la1, double lo1) {
    double[] a = Feed.world(la0, lo0), b = Feed.world(la1, lo1);
    double mpt = Feed.metersPerTile(la0);
    return Math.hypot((b[0] - a[0]) * mpt, (b[1] - a[1]) * mpt);
  }

  // lat, lon and compass heading at distance s along the polyline
  static double[] at(double s, double[] cum, double[] la, double[] lo) {
    int n = cum.length, i = 0;
    while (i < n - 2 && cum[i + 1] < s) i++;
    double seg = cum[i + 1] - cum[i], t = seg <= 0 ? 0 : Math.max(0, Math.min(1, (s - cum[i]) / seg));
    double lat = la[i] + (la[i + 1] - la[i]) * t, lon = lo[i] + (lo[i + 1] - lo[i]) * t;
    double[] a = Feed.world(la[i], lo[i]), b = Feed.world(la[i + 1], lo[i + 1]);
    double hdg = Math.toDegrees(Math.atan2(b[0] - a[0], -(b[1] - a[1])));
    return new double[] {lat, lon, (hdg + 360) % 360};
  }
}
