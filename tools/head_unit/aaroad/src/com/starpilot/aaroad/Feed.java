package com.starpilot.aaroad;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.net.LocalSocket;
import android.net.LocalSocketAddress;
import android.os.SystemClock;
import android.util.Base64;
import java.io.ByteArrayOutputStream;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.SocketTimeoutException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.zip.Inflater;
import org.json.JSONArray;
import org.json.JSONObject;

// The comma's cluster map feed (openpilot/starpilot/system/android_auto/hu_feed.py): it beacons once a second
// on the phone hotspot; while the meter's nav page is up this answers with a hello naming the road tiles it
// holds, and the comma sends openpilot's mode, the route, the car's position at 20 Hz and the missing tiles
// to the hello's address, and the song playing with its cover for the music card. No hellos, no feed: the comma reads nothing and sends nothing when not navigating.
//
// Runs in the process that draws the meter. If that process may not open network sockets, the datagrams go
// through Relay in Honda Hack's process instead, over a local socket.
final class Feed {
  static final int PORT = 47312;
  static final double TILE_ZOOM = 14, EXTENT = 4096;
  private static final int MAX_TILES = 25, HAVE_MAX = 64;
  private static final long HELLO_MS = 1000;

  static final class Tile {
    final int x, y;
    final float[][] pts = new float[5][];  // per road class: x, y pairs in tile units (0..4096 over the tile)
    final int[][] starts = new int[5][];   // per road class: index of each line's first point, then the end
    // per road class: each line's street name, null when it has none (or the comma sent a format 1 tile).
    final String[][] names = new String[5][];
    // per road class: whether each line is a bridge (road_tiles.BRIDGE), null when the tile is older than format 3
    final boolean[][] bridge = new boolean[5][];
    // traffic signal / stop / level crossing points: x, y in tile units and kind (road_tiles.SIGNAL 1, STOP 2,
    // LEVEL_CROSSING 3 from format 6) in threes;
    // empty when the tile is older than format 4
    float[] signals = new float[0];
    // per road class: each line's stacking layer (Mapbox ``layer``, bridges above 0), its lane count (0 unknown;
    // Mapbox has it only in Japan) and whether it is one-way; null when the tile is older than format 5
    final byte[][] layer = new byte[5][];
    final byte[][] lanes = new byte[5][];
    final boolean[][] oneway = new boolean[5][];
    // sidewalks, crossings, footways and pedestrian streets (format 5): x, y pairs in tile units, each line's
    // first point then the end, and each line's kind (WALK_SIDEWALK..WALK_PEDESTRIAN); empty before format 5
    float[] walkPts = new float[0];
    int[] walkStarts = {0};
    byte[] walkKind = new byte[0];
    // rail lines (format 6) in the same layout, kind RAIL_MAJOR..RAIL_SERVICE, | RAIL_BRIDGE on a bridge; empty
    // before format 6. Level crossings are signals points of kind 3 (Signals.CROSSING).
    float[] railPts = new float[0];
    int[] railStarts = {0};
    byte[] railKind = new byte[0];

    Tile(int x, int y) {
      this.x = x;
      this.y = y;
    }
  }

  static final int WALK_SIDEWALK = 1, WALK_CROSSING = 2, WALK_FOOTWAY = 3, WALK_PEDESTRIAN = 4;
  static final int RAIL_MAJOR = 1, RAIL_MINOR = 2, RAIL_SERVICE = 3, RAIL_BRIDGE = 0x80;
  static final int VERSION = 6;

  static final class Route {
    final long id;
    final double[] lat, lon;
    // traffic on the segment from point i to i + 1: 0 unknown, 1 low, 2 moderate, 3 heavy, 4 severe
    final byte[] lvl;

    Route(long id, double[] lat, double[] lon, byte[] lvl) {
      this.id = id;
      this.lat = lat;
      this.lon = lon;
      this.lvl = lvl;
    }
  }

  static final class Pose {
    final double lat, lon, hdg, spd;
    final long at;

    Pose(double lat, double lon, double hdg, double spd, long at) {
      this.lat = lat;
      this.lon = lon;
      this.hdg = hdg;
      this.spd = spd;
      this.at = at;
    }
  }

  // The next maneuver as the comma's 's' datagram had it (hu_feed.instruction: starpilotNavigation.instruction, the
  // same step the StarPilot Auto car screen draws). One piece, so the turn card never mixes two datagrams.
  // at: uptime of the datagram; distance (m) is the distance to the maneuver then.
  static final class Nav {
    static final int[] NO_LANES = new int[0];
    final String type, modifier, text, road;
    final int exit, degrees;
    final boolean left;
    final double distance;
    final long at;
    // the lanes before the maneuver, left to right (Lanes): the arrows each carries, and the one to take in it
    // (0 = not a lane to take); empty when the comma sends none. toward: where the maneuver leads ("Garland"), or "".
    final int[] lanes, take;
    final String toward;

    Nav(String type, String modifier, String text, String road, int exit, int degrees, boolean left, double distance, long at) {
      this(type, modifier, text, road, exit, degrees, left, distance, at, NO_LANES, NO_LANES, "");
    }

    Nav(String type, String modifier, String text, String road, int exit, int degrees, boolean left, double distance, long at,
        int[] lanes, int[] take, String toward) {
      this.type = type;
      this.modifier = modifier;
      this.text = text;
      this.road = road;
      this.exit = exit;
      this.degrees = degrees;
      this.left = left;
      this.distance = distance;
      this.at = at;
      this.lanes = lanes;
      this.take = take;
      this.toward = toward;
    }

    static Nav of(JSONObject nav, long at) {
      if (nav == null) return null;
      String modifier = nav.optString("modifier", "");
      // "lanes": [[arrows, take 0/1, arrow to take], ...]; a malformed list drops them all, as the comma does
      JSONArray rows = nav.optJSONArray("lanes");
      int n = rows == null || rows.length() > Lanes.MAX ? 0 : rows.length();
      int[] lanes = n == 0 ? NO_LANES : new int[n], take = n == 0 ? NO_LANES : new int[n];
      for (int i = 0; i < n; i++) {
        JSONArray row = rows.optJSONArray(i);
        int arrows = row == null ? 0 : row.optInt(0, 0) & Lanes.ALL;
        if (arrows == 0) {
          lanes = take = NO_LANES;
          break;
        }
        lanes[i] = arrows;
        take[i] = Lanes.lit(arrows, row.optInt(1, 0) != 0, row.optInt(2, 0), modifier);
      }
      return new Nav(nav.optString("type", ""), modifier, nav.optString("text", ""),
                     nav.optString("road", ""), nav.optInt("exit", 0), nav.optInt("degrees", 0), nav.optBoolean("left"),
                     nav.optDouble("distance", -1), at, lanes, take, nav.optString("toward", ""));
    }
  }

  // Everything MapView reads; written only by the feed thread.
  static volatile boolean wanted;  // the nav page is showing
  static volatile double lat, lon, hdg, spd;
  static volatile long poseAt;     // SystemClock.uptimeMillis() of the fix the pose stands for
  static volatile Pose pose;       // the same fix as one piece, so a reader on another thread never mixes two
  static volatile boolean brake;   // the pose's "b": the driver on the pedal or openpilot braking firmly; the rear lights
  static volatile long stateAt;    // when turnMeters was current
  static volatile String mode = "off";
  static volatile boolean metric;
  static volatile double turnMeters = -1, limitMs = -1, remainingSec = -1, remainingMeters = -1;
  static volatile Route route;
  static volatile Nav nav;           // null while the comma is not guiding
  static volatile String road = "";  // the road the car is on, from the comma's route step; "" = not known
  // its route number(s) as Mapbox gives them ("US 75", "US 75; TX 289"), for the street tag's badge; "" = none, or a
  // comma from before 2026-10-31 that does not send it
  static volatile String roadRef = "";
  static final Map<String, Tile> tiles = new ConcurrentHashMap<String, Tile>();
  static volatile int tilesVersion;
  // the song for the music card; mediaStatus null when nothing plays. mediaArt is the id of its cover (0: none),
  // which matches coverId once the comma's 'a' datagrams have all come in.
  static volatile String mediaStatus, mediaTitle, mediaArtist, mediaSource;
  static volatile long mediaArt;
  // song progress for the ring round the play button: mediaPos ms in at mediaAt (uptime), mediaDur ms; 0: unknown
  static volatile long mediaPos, mediaAt, mediaDur;
  static volatile Bitmap cover;
  static volatile long coverId;

  private static boolean started;
  private static final Map<String, String[]> tileParts = new HashMap<String, String[]>();
  private static final Map<Long, JSONObject[]> routeParts = new HashMap<Long, JSONObject[]>();
  private static long coverPartsId;
  private static String[] coverParts;
  private static volatile InetAddress comma;
  private static volatile long commaAt;
  private static boolean noNetworkLogged;

  private interface Link {
    int receive(byte[] buf) throws Exception;  // blocks; 0 on timeout
    void send(byte[] data) throws Exception;   // to the comma that beaconed last
  }

  static synchronized void start() {
    if (started) return;
    started = true;
    Thread t = new Thread(new Runnable() {
      @Override
      public void run() {
        boolean logged = false;
        while (true) {
          try {
            Link link = direct();
            if (link == null) link = relay();
            Xlog.log("aaroad: map feed listening" + (link instanceof Relayed ? " through Honda Hack" : ""));
            logged = false;
            pump(link);
          } catch (Throwable e) {
            if (!logged) Xlog.log("aaroad: map feed down, retrying: " + e);
            logged = true;
          }
          SystemClock.sleep(2000);
        }
      }
    }, "aaroad-feed");
    t.setDaemon(true);
    t.start();
  }

  private static Link direct() {
    final DatagramSocket sock;
    try {
      sock = new DatagramSocket(null);
      sock.setReuseAddress(true);
      sock.setBroadcast(true);
      sock.bind(new InetSocketAddress(PORT));
      sock.setSoTimeout(250);
    } catch (Exception e) {
      if (!noNetworkLogged) Xlog.log("aaroad: map feed has no network here (" + e.getMessage() + "), asking Honda Hack");
      noNetworkLogged = true;
      return null;
    }
    return new Link() {
      @Override
      public int receive(byte[] buf) throws Exception {
        DatagramPacket p = new DatagramPacket(buf, buf.length);
        try {
          sock.receive(p);
        } catch (SocketTimeoutException e) {
          return 0;
        }
        if (p.getLength() > 0 && isBeacon(buf, p.getLength())) comma = p.getAddress();
        return p.getLength();
      }

      @Override
      public void send(byte[] data) throws Exception {
        InetAddress to = comma;
        if (to != null) sock.send(new DatagramPacket(data, data.length, to, PORT));
      }
    };
  }

  private static final class Relayed implements Link {
    final LocalSocket sock = new LocalSocket();
    DataInputStream in;
    DataOutputStream out;

    @Override
    public int receive(byte[] buf) throws Exception {
      int n = in.readUnsignedShort();
      in.readFully(buf, 0, n);
      return n;
    }

    @Override
    public synchronized void send(byte[] data) throws Exception {
      out.writeShort(data.length);
      out.write(data);
      out.flush();
    }
  }

  private static Link relay() throws Exception {
    Relayed r = new Relayed();
    r.sock.connect(new LocalSocketAddress(Relay.NAME));
    r.in = new DataInputStream(r.sock.getInputStream());
    r.out = new DataOutputStream(r.sock.getOutputStream());
    return r;
  }

  private static void pump(final Link link) throws Exception {
    final boolean[] dead = {false};
    Thread hello = new Thread(new Runnable() {
      @Override
      public void run() {
        boolean heard = true;
        while (!dead[0]) {
          try {
            boolean h = SystemClock.uptimeMillis() - commaAt < 5000;
            if (h != heard && wanted) MapView.say("aaroad: comma feed " + (h ? "back" : "lost: nothing from the comma for 5 s"));
            heard = h;
            if (wanted && h) link.send(hello());
          } catch (Throwable ignored) {
            // the receive side sees a broken link and starts over
          }
          SystemClock.sleep(HELLO_MS);
        }
      }
    }, "aaroad-hello");
    hello.setDaemon(true);
    hello.start();
    byte[] buf = new byte[65536];
    try {
      while (true) {
        int n = link.receive(buf);
        if (n <= 0) continue;
        try {
          handle(new JSONObject(new String(buf, 0, n, "UTF-8")));
        } catch (Throwable e) {
          Xlog.log("aaroad: map feed dropped a datagram: " + e);
        }
      }
    } finally {
      dead[0] = true;
    }
  }

  static boolean isBeacon(byte[] buf, int n) {
    String s = new String(buf, 0, Math.min(n, 64));
    return s.contains("\"t\":\"b\"");
  }

  private static byte[] hello() throws Exception {
    JSONArray have = new JSONArray();
    for (String k : tiles.keySet()) {
      if (have.length() >= HAVE_MAX) break;
      have.put(k);
    }
    // rt: the newest road tile format this app reads, 6 with railroad crossings and rail lines (2026-11-01); the
    // comma sends an older app's tiles as format 5 (layers, lanes, sidewalks), 4 (signals), 3 (bridges), 2 (names,
    // 2026-10-30) or 1 (no "rt")
    return new JSONObject().put("v", 1).put("t", "h").put("have", have).put("rt", VERSION).toString().getBytes("UTF-8");
  }

  private static void handle(JSONObject m) throws Exception {
    if (m.optInt("v") != 1) return;
    String t = m.optString("t");
    long now = SystemClock.uptimeMillis();
    // Any datagram says the comma is there, not only its beacon: the beacon is a broadcast, and a head unit's Wi-Fi
    // drops broadcasts first, so a few lost beacons used to pause the hellos and with them the comma's whole feed
    // (hu_feed.py drops a peer after 5 s without hellos), taking the map down though poses still came (2026-11-01).
    commaAt = now;
    if (t.equals("b")) {
      // the comma's beacon; nothing more in it
    } else if (t.equals("p")) {
      lat = m.getDouble("lat");
      lon = m.getDouble("lon");
      hdg = m.getDouble("hdg");
      spd = m.optDouble("spd", 0);
      poseAt = now - (long) (m.optDouble("age", 0) * 1000);
      pose = new Pose(lat, lon, hdg, spd, poseAt);
      brake = m.optInt("b", 0) == 1;  // left out unless braking, and by a comma from before 2026-10-31
    } else if (t.equals("s")) {
      stateAt = now;
      mode = m.optString("mode", "off");
      metric = m.optBoolean("metric");
      JSONObject nav = m.optJSONObject("nav");
      turnMeters = nav != null ? nav.optDouble("distance", -1) : -1;
      limitMs = nav != null && !nav.isNull("limit") ? nav.optDouble("limit", -1) : m.isNull("limit") ? -1 : m.optDouble("limit", -1);
      remainingSec = nav != null ? nav.optDouble("duration", -1) : -1;
      remainingMeters = nav != null ? nav.optDouble("remaining", -1) : -1;
      road = nav != null ? nav.optString("road", "") : "";
      roadRef = nav != null ? nav.optString("ref", "") : "";
      Feed.nav = Nav.of(nav, now);
      if (nav == null) route = null;
      JSONObject media = m.optJSONObject("media");
      mediaTitle = media != null ? media.optString("title") : null;
      mediaArtist = media != null ? media.optString("artist") : null;
      mediaSource = media != null ? media.optString("source") : null;
      mediaArt = media != null ? media.optLong("art") : 0;
      mediaDur = media != null ? media.optLong("dur") : 0;
      mediaPos = m.isNull("pos") ? -1 : m.optLong("pos");
      mediaAt = SystemClock.uptimeMillis();
      mediaStatus = media != null ? media.optString("status", "playing") : null;
    } else if (t.equals("r")) {
      onRoute(m);
    } else if (t.equals("m")) {
      onTile(m);
    } else if (t.equals("a")) {
      onCover(m);
    }
  }

  private static void onRoute(JSONObject m) throws Exception {
    long id = m.getLong("id");
    int i = m.getInt("i"), n = m.getInt("n");
    if (n <= 0 || n > 400 || i < 0 || i >= n) return;
    Route current = route;
    if (current != null && current.id == id) return;  // the comma repeats the route every 5 s
    JSONObject[] parts = routeParts.get(id);
    if (parts == null || parts.length != n) {
      routeParts.clear();
      parts = new JSONObject[n];
      routeParts.put(id, parts);
    }
    parts[i] = m;
    int count = 0;
    for (JSONObject p : parts) {
      if (p == null) return;
      count += p.getJSONArray("pts").length();
    }
    double[] la = new double[count], lo = new double[count];
    byte[] lv = new byte[count];
    int j = 0;
    for (JSONObject part : parts) {
      JSONArray p = part.getJSONArray("pts");
      // "c", a traffic level per point, is left out by a comma without traffic; one of the wrong length is not used
      JSONArray c = part.optJSONArray("c");
      if (c != null && c.length() != p.length()) c = null;
      for (int k = 0; k < p.length(); k++, j++) {
        la[j] = p.getJSONArray(k).getDouble(0);
        lo[j] = p.getJSONArray(k).getDouble(1);
        int v = c == null ? 0 : c.optInt(k, 0);
        lv[j] = (byte) (v < 0 || v > 4 ? 0 : v);
      }
    }
    routeParts.remove(id);
    route = new Route(id, la, lo, lv);
  }

  private static void onTile(JSONObject m) throws Exception {
    String key = m.getString("k");
    int i = m.getInt("i"), n = m.getInt("n");
    if (n <= 0 || n > 2100 || i < 0 || i >= n || tiles.containsKey(key)) return;
    String[] parts = tileParts.get(key);
    if (parts == null || parts.length != n) {
      if (tileParts.size() > 4) tileParts.clear();
      parts = new String[n];
      tileParts.put(key, parts);
    }
    parts[i] = m.getString("b");
    ByteArrayOutputStream raw = new ByteArrayOutputStream();
    for (String p : parts) {
      if (p == null) return;
      raw.write(Base64.decode(p, Base64.DEFAULT));
    }
    tileParts.remove(key);
    String[] xy = key.split("/");
    Tile tile = decode(Integer.parseInt(xy[0]), Integer.parseInt(xy[1]), raw.toByteArray());
    tiles.put(key, tile);
    evict(tile);
    tilesVersion++;
  }

  // The album cover, a small JPEG in a few datagrams; only the latest one is kept.
  private static void onCover(JSONObject m) throws Exception {
    long id = m.getLong("id");
    int i = m.getInt("i"), n = m.getInt("n");
    if (n <= 0 || n > 64 || i < 0 || i >= n || (id == coverId && cover != null)) return;  // repeats every 30 s
    if (coverParts == null || coverPartsId != id || coverParts.length != n) {
      coverParts = new String[n];
      coverPartsId = id;
    }
    coverParts[i] = m.getString("b");
    ByteArrayOutputStream raw = new ByteArrayOutputStream();
    for (String p : coverParts) {
      if (p == null) return;
      raw.write(Base64.decode(p, Base64.DEFAULT));
    }
    coverParts = null;
    byte[] b = raw.toByteArray();
    Bitmap bmp = BitmapFactory.decodeByteArray(b, 0, b.length);
    if (bmp == null) return;
    cover = bmp;
    coverId = id;
  }

  // Keep the tiles nearest the car; the comma sends the 3 x 3 around it.
  private static void evict(Tile fresh) {
    if (tiles.size() <= MAX_TILES) return;
    double[] car = world(lat, lon);
    while (tiles.size() > MAX_TILES) {
      String far = null;
      double worst = -1;
      for (Iterator<Map.Entry<String, Tile>> it = tiles.entrySet().iterator(); it.hasNext(); ) {
        Map.Entry<String, Tile> e = it.next();
        double dx = e.getValue().x + 0.5 - car[0], dy = e.getValue().y + 0.5 - car[1], d = dx * dx + dy * dy;
        if (e.getValue() != fresh && d > worst) {
          worst = d;
          far = e.getKey();
        }
      }
      if (far == null) return;
      tiles.remove(far);
    }
  }

  // road_tiles.py encode_road_tile: "SPRT", version, class count, then zlib of per class
  // <BII (class, lines, points), the line lengths as <u2, and the points as <i4 x, y deltas.
  // Version 2 puts the tile's street names first (<H count, then <B length + UTF-8 each) and an <u2 name
  // index per line (0 = none) after the line lengths. Version 3 adds a flags byte per line after that, bit 0 a bridge.
  // Version 4 ends with the tile's signal points: <H count, then per point <h x, <h y, <B kind.
  // Version 5 adds per line after the flags an <b layer and a <B lane count (flags bit 2 one-way), and ends
  // with the walk lines: <I count, <I points, the <u2 lengths, a <B kind each, and the <i4 x, y deltas.
  static Tile decode(int x, int y, byte[] raw) throws Exception {
    if (raw.length < 6 || raw[0] != 'S' || raw[1] != 'P' || raw[2] != 'R' || raw[3] != 'T' || raw[4] < 1 || raw[4] > VERSION) {
      throw new IllegalArgumentException("not a road tile");
    }
    int version = raw[4];
    int classes = raw[5] & 0xff;
    Inflater inflater = new Inflater();
    ByteArrayOutputStream body = new ByteArrayOutputStream(raw.length * 4);
    try {
      inflater.setInput(raw, 6, raw.length - 6);
      byte[] buf = new byte[16384];
      while (!inflater.finished()) {
        int n = inflater.inflate(buf);
        if (n == 0 && (inflater.needsInput() || inflater.needsDictionary())) throw new IllegalArgumentException("truncated road tile");
        body.write(buf, 0, n);
        if (body.size() > 64 * 1024 * 1024) throw new IllegalArgumentException("oversized road tile");
      }
    } finally {
      inflater.end();
    }
    ByteBuffer b = ByteBuffer.wrap(body.toByteArray()).order(ByteOrder.LITTLE_ENDIAN);
    Tile tile = new Tile(x, y);
    String[] table = new String[0];
    if (version >= 2) {
      table = new String[b.getShort() & 0xffff];
      for (int i = 0; i < table.length; i++) {
        int len = b.get() & 0xff;
        table[i] = new String(b.array(), b.position(), len, "UTF-8");
        b.position(b.position() + len);
      }
    }
    for (int c = 0; c < classes; c++) {
      int cls = b.get() & 0xff, lines = b.getInt(), total = b.getInt();
      if (cls >= 5 || lines < 0 || total < 0 || (long) lines * (version >= 5 ? 7 : version >= 3 ? 5 : version >= 2 ? 4 : 2) + (long) total * 8 > b.remaining()) {
        throw new IllegalArgumentException("bad road tile class");
      }
      int[] starts = new int[lines + 1];
      for (int l = 0; l < lines; l++) starts[l + 1] = starts[l] + (b.getShort() & 0xffff);
      if (starts[lines] != total) throw new IllegalArgumentException("bad road tile lengths");
      if (version >= 2) {
        String[] names = new String[lines];
        for (int l = 0; l < lines; l++) {
          int k = b.getShort() & 0xffff;
          if (k > table.length) throw new IllegalArgumentException("bad road tile name");
          names[l] = k == 0 ? null : table[k - 1];
        }
        tile.names[cls] = names;
      }
      if (version >= 3) {
        boolean[] bridge = new boolean[lines];
        boolean[] oneway = new boolean[lines];
        for (int l = 0; l < lines; l++) {
          int f = b.get() & 0xff;
          bridge[l] = (f & 1) != 0;
          oneway[l] = (f & 4) != 0;
        }
        tile.bridge[cls] = bridge;
        if (version >= 5) {
          byte[] layer = new byte[lines], lanes = new byte[lines];
          b.get(layer);
          b.get(lanes);
          tile.layer[cls] = layer;
          tile.lanes[cls] = lanes;
          tile.oneway[cls] = oneway;
        }
      }
      float[] pts = new float[total * 2];
      int ax = 0, ay = 0;
      for (int p = 0; p < total; p++) {
        ax += b.getInt();
        ay += b.getInt();
        pts[2 * p] = ax;
        pts[2 * p + 1] = ay;
      }
      tile.pts[cls] = pts;
      tile.starts[cls] = starts;
    }
    if (version >= 4) {
      int n = b.getShort() & 0xffff;
      if ((long) n * 5 > b.remaining()) throw new IllegalArgumentException("bad road tile points");
      float[] sig = new float[n * 3];
      for (int i = 0; i < n; i++) {
        sig[3 * i] = b.getShort();
        sig[3 * i + 1] = b.getShort();
        sig[3 * i + 2] = b.get() & 0xff;
      }
      tile.signals = sig;
    }
    if (version >= 5) {
      KindLines w = kindLines(b, "walk");
      tile.walkPts = w.pts;
      tile.walkStarts = w.starts;
      tile.walkKind = w.kind;
    }
    if (version >= 6) {
      KindLines r = kindLines(b, "rail");
      tile.railPts = r.pts;
      tile.railStarts = r.starts;
      tile.railKind = r.kind;
    }
    return tile;
  }

  private static final class KindLines {
    float[] pts;
    int[] starts;
    byte[] kind;
  }

  // A table of lines with a kind byte each (the walks, and from format 6 the rails), as road_tiles._kind_lines.
  private static KindLines kindLines(ByteBuffer b, String what) {
    int n = b.getInt(), total = b.getInt();
    if (n < 0 || total < 0 || (long) n * 3 + (long) total * 8 > b.remaining()) throw new IllegalArgumentException("bad road tile " + what + "s");
    KindLines out = new KindLines();
    int[] starts = new int[n + 1];
    for (int l = 0; l < n; l++) starts[l + 1] = starts[l] + (b.getShort() & 0xffff);
    if (starts[n] != total) throw new IllegalArgumentException("bad road tile " + what + " lengths");
    byte[] kind = new byte[n];
    b.get(kind);
    float[] pts = new float[total * 2];
    int ax = 0, ay = 0;
    for (int p = 0; p < total; p++) {
      ax += b.getInt();
      ay += b.getInt();
      pts[2 * p] = ax;
      pts[2 * p + 1] = ay;
    }
    out.pts = pts;
    out.starts = starts;
    out.kind = kind;
    return out;
  }

  // Web Mercator position in zoom-14 tile units, as road_tiles.world_xy.
  static double[] world(double latitude, double longitude) {
    return world(latitude, longitude, new double[2]);
  }

  // The same into out, for the map's frames, which allocate nothing.
  static double[] world(double latitude, double longitude, double[] out) {
    double n = 1 << (int) TILE_ZOOM;
    latitude = Math.max(-85.05112878, Math.min(85.05112878, latitude));
    double s = Math.sin(Math.toRadians(latitude));
    out[0] = (longitude + 180) / 360 * n;
    out[1] = (0.5 - Math.log((1 + s) / (1 - s)) / (4 * Math.PI)) * n;
    return out;
  }

  static double metersPerTile(double latitude) {
    return 40075016.686 * Math.cos(Math.toRadians(latitude)) / (1 << (int) TILE_ZOOM);
  }

  private Feed() {}
}
