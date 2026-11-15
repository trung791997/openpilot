package com.starpilot.aaroad;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.os.SystemClock;
import android.view.ViewGroup;
import java.io.File;
import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.Base64;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import javax.imageio.ImageIO;
import org.json.JSONObject;

// Replays a real GPS pass over REAL road tiles through the app's own MapView and records, per frame, whether the
// under-bridge outline is wanted. Nothing is reimplemented: the decision is MapView.underDeck's own (Under.covering,
// aboveCarLayer, Under.Fade), read back from MapView's `under` Fade after each onDraw. Works against whichever app
// build is first on the class path (new HEAD or the old 052c86a86a), which is how the positive control runs.
//
//   Replay <pass.csv> <tiles dir> <module.zip> <frames.csv> [png-out]
//   pass.csv: header, then t_ms,lat,lon,bearing_deg,speed_mps (GPS fixes, ~1 Hz). Interpolated to 10 Hz.
//   -Dpng.t=<seconds from pass start> saves that frame to png-out. -Dx.lat / -Dx.lon: the crossing point (distances).
// Not the Tegra GPU: MapView.GPU is off, the software canvas (as desktop/render.sh).
public final class Replay {
  static final int W = 584, H = 215;
  static final long STEP_MS = 100;

  public static void main(String[] a) throws Exception {
    File passF = new File(a[0]), tilesDir = new File(a[1]), out = new File(a[3]);
    MapView.modulePath = a[2];
    MapView.GPU = false;
    double xla = Double.parseDouble(System.getProperty("x.lat", "33.1168")), xlo = Double.parseDouble(System.getProperty("x.lon", "-96.6703"));
    double pngT = Double.parseDouble(System.getProperty("png.t", "-1"));
    Method handle = Feed.class.getDeclaredMethod("handle", JSONObject.class);
    handle.setAccessible(true);

    // the real tiles, as the comma sends them: 'm' datagrams, key "x/y", base64 chunks of 1000 bytes
    List<File> rt = new ArrayList<File>();
    collect(tilesDir, rt);
    // the first pose before any tile: Feed.evict keeps the MAX_TILES (25) nearest the car, and with no pose yet that is the
    // nearest to (0, 0). The 5x5 tiles around the crossing are what the comma would send for a car there.
    double[] cw = Feed.world(xla, xlo);
    int cx = (int) Math.floor(cw[0]), cy = (int) Math.floor(cw[1]);
    double[] f0 = at(readFixes(passF), readFixes(passF).get(0)[0]);
    send(handle, new JSONObject().put("v", 1).put("t", "p").put("lat", f0[0]).put("lon", f0[1]).put("hdg", f0[2]).put("spd", f0[3]).put("age", 0.0));
    for (File f : rt) {
      String y = f.getName().replace(".rt", ""), x = f.getParentFile().getName();
      if (Math.abs(Integer.parseInt(x) - cx) > 2 || Math.abs(Integer.parseInt(y) - cy) > 2) continue;
      byte[] raw = Files.readAllBytes(f.toPath());
      int parts = (raw.length + 999) / 1000;
      for (int i = 0; i < parts; i++) {
        byte[] c = java.util.Arrays.copyOfRange(raw, i * 1000, Math.min(raw.length, (i + 1) * 1000));
        send(handle, new JSONObject().put("v", 1).put("t", "m").put("k", x + "/" + y).put("i", i).put("n", parts)
            .put("b", Base64.getEncoder().encodeToString(c)));
      }
    }
    List<double[]> fx = readFixes(passF);
    double t0 = fx.get(0)[0], t1 = fx.get(fx.size() - 1)[0];

    File tmp = Files.createTempDirectory("aaroad-replay").toFile();
    Context ctx = new Context(tmp);
    MapView view = new MapView(ctx, new ViewGroup(ctx), "com.example.meter");
    view.shimSize(W, H);
    Field tickF = MapView.class.getDeclaredField("tick");
    tickF.setAccessible(true);
    Runnable tick = (Runnable) tickF.get(view);
    Field sceneF = MapView.class.getDeclaredField("scene");
    sceneF.setAccessible(true);
    Field buildingF = MapView.class.getDeclaredField("building");
    buildingF.setAccessible(true);
    AtomicBoolean building = (AtomicBoolean) buildingF.get(view);
    Field underF = MapView.class.getDeclaredField("under");
    underF.setAccessible(true);
    Field drawnF = MapView.class.getDeclaredField("drawnUnder");
    drawnF.setAccessible(true);
    Object under = underF.get(view);
    Method onM = under.getClass().getDeclaredMethod("on");
    onM.setAccessible(true);

    Field dfF = MapView.class.getDeclaredField("drawFrame");
    dfF.setAccessible(true);
    Field rmF = MapView.class.getDeclaredField("ROAD_M");
    rmF.setAccessible(true);
    float[] roadM = (float[]) rmF.get(null);
    boolean debug = Boolean.getBoolean("debug");
    Bitmap frame = Bitmap.createBitmap(W, H, Bitmap.Config.ARGB_8888);
    Canvas canvas = new Canvas(frame);
    double[] xw = Feed.world(xla, xlo);
    double mpt = Feed.metersPerTile(xla);
    StringBuilder sb = new StringBuilder("frame,t_s,lat,lon,hdg,spd,east_m,north_m,dist_m,under_on,under_level,close_maxH,car_h,raw_cover_n\n");
    int n = 0, shown = 0;
    boolean pngDone = false;
    Object lastScene = null;
    int lastNc = -1;
    int total = (int) Math.floor((t1 - t0) * 1000 / STEP_MS);
    for (int i = 0; i <= total; i++) {
      double t = t0 + i * STEP_MS / 1000.0;
      double[] p = at(fx, t);
      SystemClock.advance(STEP_MS);
      send(handle, new JSONObject().put("v", 1).put("t", "p").put("lat", p[0]).put("lon", p[1]).put("hdg", p[2]).put("spd", p[3]).put("age", 0.0));
      send(handle, new JSONObject().put("v", 1).put("t", "s").put("mode", "exp").put("metric", false).put("limit", 17.9));
      tick.run();
      int guard = 0;
      while (building.get() || sceneF.get(view) == null) {  // the scene builds on its own thread; let it finish, as the car would wait for it
        Thread.sleep(10);
        if (++guard > 3000) throw new IllegalStateException("scene never built");
      }
      if (guard > 0) tick.run();
      frame.eraseColor(0xff000000);
      canvas.save();
      view.onDraw(canvas);
      canvas.restore();
      boolean on = (Boolean) onM.invoke(under);
      float lvl = drawnF.getFloat(view);
      double[] w = Feed.world(p[0], p[1]);
      double e = (w[0] - xw[0]) * mpt, nn = -(w[1] - xw[1]) * mpt;
      sb.append(i).append(',').append(String.format("%.1f", t - t0)).append(',').append(String.format("%.6f,%.6f,%.1f,%.1f", p[0], p[1], p[2], p[3]))
          .append(',').append(Math.round(e)).append(',').append(Math.round(nn)).append(',').append(Math.round(Math.hypot(e, nn)))
          .append(',').append(on ? 1 : 0).append(',').append(String.format("%.2f", lvl));
      // what the decision saw, from the app's own classes: the detail's top height, the car's height (Under.carHeight)
      // and how many raised segments Under.covering finds over the car BEFORE MapView's layer filter
      MapView.Frame df = (MapView.Frame) dfF.get(view);
      if (debug && df != null && df.s != lastScene) {
        lastScene = df.s;
        CloseUp.Detail dd = df.s.close;
        System.err.println("scene@frame " + i + ": tiles=" + Feed.tiles.size() + (dd == null ? " close=null" : " segs=" + dd.roadSegs + " bridges=" + dd.bridges
            + " ramps=" + dd.ramps + " maxH=" + dd.maxH + " ground=" + dd.groundSegs + " heldOn=" + (dd.heldOn != null)));
      }
      CloseUp.Detail d = df == null || df.s == null ? null : df.s.close;
      if (d != null) {
        float gr = CloseUp.gate(df.k * MapView.R0, CloseUp.ROAD_SHOW, CloseUp.ROAD_HIDE);
        float hc = Under.carHeight(d, df.ce, df.cn, df.ch, df.sh, roadM);
        int[] cov = new int[512];
        int nc = Under.covering(d, df.ce, df.cn, df.ch, df.sh, df.ik, gr, roadM, MapView.dm(MapView.MIN_ROAD_PX),
            MapView.dm(2 * MapView.KERB_PX + 2 * MapView.CASING_PX), MapView.dm(Under.CAR_PX), cov);
        sb.append(',').append(String.format("%.1f,%.1f,%d", d.maxH, hc, nc));
        if (debug && on && (i % 10 == 0 || nc != lastNc)) {  // which raised segments the decision found over the car (car frame: east, north metres, heights)
          StringBuilder wb = new StringBuilder("  frame " + i + " E=" + Math.round(e) + " N=" + Math.round(nn) + " hdg=" + Math.round(p[2]) + " hc=" + hc + " covering:");
          for (int k = 0; k < nc; k++) {
            int o = cov[k] * CloseUp.SEG;
            wb.append(String.format(" [seg%d range%d (%.0f,%.0f)->(%.0f,%.0f) h %.1f->%.1f]", cov[k], rangeOf(d, cov[k]),
                d.seg[o], d.seg[o + 1], d.seg[o + 2], d.seg[o + 3], d.seg[o + 4], d.seg[o + 5]));
          }
          System.err.println(wb);
        }
        lastNc = nc;
      } else {
        sb.append(",-1,-1,-1");
      }
      sb.append('\n');
      n++;
      if (on) shown++;
      if (pngT >= 0 && !pngDone && t - t0 >= pngT && a.length > 4) {
        File png = new File(a[4]);
        png.getParentFile().mkdirs();
        ImageIO.write(frame.img, "png", png);
        pngDone = true;
      }
    }
    out.getParentFile().mkdirs();
    Files.write(out.toPath(), sb.toString().getBytes("UTF-8"));
    System.err.println("replay: " + passF.getName() + " frames=" + n + " outline_on=" + shown);
    System.exit(0);
  }

  // t_s, lat, lon, bearing, speed per fix
  static List<double[]> readFixes(File f) throws Exception {
    List<double[]> fx = new ArrayList<double[]>();
    List<String> lines = Files.readAllLines(f.toPath());
    for (int i = 1; i < lines.size(); i++) {
      String[] p = lines.get(i).trim().split(",");
      if (p.length < 5) continue;
      fx.add(new double[] {Double.parseDouble(p[0]) / 1000.0, Double.parseDouble(p[1]), Double.parseDouble(p[2]), Double.parseDouble(p[3]), Double.parseDouble(p[4])});
    }
    return fx;
  }

  static int rangeOf(CloseUp.Detail d, int i) {
    int r = 0;
    while (r + 1 < CloseUp.RANGES && d.from[r + 1] <= i) r++;
    return r;
  }

  static void collect(File d, List<File> out) {
    File[] fs = d.listFiles();
    if (fs == null) return;
    java.util.Arrays.sort(fs);
    for (File f : fs) {
      if (f.isDirectory()) collect(f, out);
      else if (f.getName().endsWith(".rt")) out.add(f);
    }
  }

  static void send(Method handle, JSONObject m) throws Exception {
    handle.invoke(null, new JSONObject(m.toString()));
  }

  // lat, lon, bearing (circular), speed at time t, linear between the two fixes around it
  static double[] at(List<double[]> fx, double t) {
    int i = 0;
    while (i < fx.size() - 2 && fx.get(i + 1)[0] < t) i++;
    double[] a = fx.get(i), b = fx.get(i + 1);
    double f = b[0] == a[0] ? 0 : Math.max(0, Math.min(1, (t - a[0]) / (b[0] - a[0])));
    double db = ((b[3] - a[3] + 540) % 360) - 180;
    return new double[] {a[1] + (b[1] - a[1]) * f, a[2] + (b[2] - a[2]) * f, (a[3] + db * f + 360) % 360, a[4] + (b[4] - a[4]) * f};
  }
}
