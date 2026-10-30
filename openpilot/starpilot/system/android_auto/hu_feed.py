"""Turn guidance, openpilot's mode and the car's position for the head unit's cluster map (tools/head_unit/aaroad).

The cluster map colours the route by what openpilot is doing (exp orange, chill green, lateral-only blue,
white when openpilot is not driving) and draws the car on the road map around it. Android Auto's own nav
channel carries only the next turn as text and a picture, about once a second, and only while the comma is
the head unit's Android Auto source, so this rides on the LAN the two share instead.

The comma reaches the head unit's Android Auto over the AAWireless dongle, which gives no IP route to the
head unit; both are on the phone hotspot, so this uses that network. A beacon goes out once a second on
each broadcast-capable interface and says only "a comma is here". A head unit that wants the feed answers
with a hello; the comma then sends to that address alone, and stops when the hellos stop. With no head
unit asking, nothing about the drive leaves the comma and the sensors are never read.

Every datagram is one JSON object with "v" (VERSION) and "t" (kind):
  b  beacon, broadcast, 1 Hz: {"port"}
  h  hello, head unit -> comma, about 1 Hz: {"have": ["x/y", ...], "rt"} road tiles it holds; no "have", no tiles.
     rt is the newest road tile format it reads (2: street names per line); without rt it reads only format 1,
     and gets its tiles turned back into format 1 (an older app drops anything else, and the comma would resend
     it every TILE_RETRY s)
  s  state, 1 Hz and on change: {"mode", "metric", "nav", "limit", "media", "pos"}; nav is null unless a route is guiding;
     limit (m/s) is the speed limit openpilot goes by, or null; media is the song playing or paused, else null:
     {"status": "playing"|"paused", "title", "artist", "source": "spotify"|"phone", "art", "dur"}, art the id of its
     cover (crc32 of the cover URL) or 0 without one, dur the song length in ms or 0. pos is how far into the song it
     is (ms) when sent, or null; it is left out of the change check, so a playing song does not resend every tick.
  r  route, on change and every ROUTE_EVERY s: {"id", "i", "n", "pts": [[lat, lon], ...]}, n datagrams
  p  pose, 20 Hz: {"lat", "lon", "hdg", "spd", "age"}, the last GPS fix carried forward to now
  m  road tile, one at a time: {"k": "x/y", "i", "n", "b": base64}; the stored zoom-14 file (road_tiles.py),
     in the hello's rt format or older
  a  album cover, once it is downloaded and every COVER_EVERY s while a song is on: {"id", "i", "n", "b": base64};
     a COVER_PX square JPEG, id as media's art
"""
from __future__ import annotations

import base64
import fcntl
import json
import math
import socket
import struct
import threading
import time
import unicodedata
import urllib.request
import zlib
from collections.abc import Callable

PORT = 47312
VERSION = 1
POSE_HZ = 20.
STATE_EVERY = 1.
BEACON_EVERY = 1.
ROUTE_EVERY = 5.
PEER_TIMEOUT = 5.
ROUTE_CHUNK = 50           # points per datagram, under 1.3 kB
TILE_CHUNK = 1000          # tile bytes per datagram, about 1.4 kB once base64'd
TILE_CHUNKS_PER_TICK = 20  # 400 datagrams a second at most
TILE_SCAN_EVERY = 1.       # s between looks for a tile the head unit lacks
TILE_RETRY = 3.            # s after sending a tile before it goes again, if the hellos still lack it
# navigationd publishes once a second but can pause for a few seconds while it reroutes. A missing nav makes the
# cluster drop the route and fall back to every nearby street in grey, so a pause shorter than this keeps the last turn.
NAV_HOLD_NS = 10_000_000_000
MAX_CARRY = 1.             # s: past this a fix is too old to carry forward
METERS_PER_DEG = 111_320.
MODES = ('exp', 'chill', 'aol', 'off')
MEDIA_EVERY = 1.           # s between reads of the song (a small file the soloist supervisor rewrites)
COVER_EVERY = 30.          # s between repeats of the cover, so a head unit that missed a datagram gets it whole
COVER_PX = 56              # the card draws it about 40 px square
COVER_MAX_BYTES = 2 * 1024 * 1024
COVER_TIMEOUT = 5.

_IFF_UP, _IFF_BROADCAST = 0x1, 0x2
_SIOCGIFFLAGS, _SIOCGIFADDR, _SIOCGIFBRDADDR = 0x8913, 0x8915, 0x8919
# aa0 is the AAWireless dongle's Wi-Fi: the head unit is behind its USB side, never on that network.
_SKIP = ('lo', 'aa')


def mode_of(enabled: bool, experimental: bool, lat_active: bool, desired_long: bool | None) -> str:
  """What openpilot is doing, as the driver sees it. desired_long: the AOL axis's longitudinal intent while
  AOL runs (None without a fresh axis), since AOL keeps selfdrive enabled for lateral alone."""
  if enabled and desired_long is not False:
    return 'exp' if experimental else 'chill'
  return 'aol' if lat_active else 'off'


# The heading turns the whole heading-up map, so its noise shows as the road swinging under the car (2026-10-29 video,
# a few degrees frame to frame at 48 mph). Below walking speed GPS has no real heading (the comma's still read about
# 40 deg accuracy and wandered 5 deg at a stop) and the car cannot turn, so a stop keeps the heading it arrived with.
# Moving, the gyro carries the heading between fixes and each GPS bearing pulls it only part of the way.
STOPPED_SPEED = 1.0
BEARING_GAIN = 0.3
MAX_GYRO_GAP = 1.  # s: past this the held heading is too old to carry


class HeadingFilter:
  def __init__(self):
    self.heading: float | None = None
    self.stamp = 0

  def __call__(self, bearing: float | None, speed: float, yaw_rate: float, now_ns: int) -> float | None:
    """Heading (deg) to send with the fix, None when there is none. yaw_rate in deg/s, + right."""
    if self.heading is not None and speed < STOPPED_SPEED:
      self.stamp = now_ns
      return self.heading
    dt = (now_ns - self.stamp) * 1e-9
    if bearing is None or self.heading is None or not 0 <= dt <= MAX_GYRO_GAP:
      self.heading = bearing  # nothing to carry: start again from the GPS, or have no heading
    else:
      predicted = self.heading + yaw_rate * dt
      self.heading = (predicted + BEARING_GAIN * ((bearing - predicted + 180) % 360 - 180)) % 360
    self.stamp = now_ns
    return self.heading


def carry(fix: tuple, speed: float, yaw_rate: float, now_ns: int) -> dict | None:
  """The fix (stamp_ns, lat, lon, bearing_deg) moved on to now at speed (m/s) and yaw_rate (deg/s, + right)."""
  stamp, lat, lon, bearing = fix
  age = (now_ns - stamp) * 1e-9
  if not 0 <= age <= MAX_CARRY:
    return None
  speed = speed if math.isfinite(speed) and speed > 0 else 0.
  yaw_rate = yaw_rate if math.isfinite(yaw_rate) else 0.
  heading = bearing + yaw_rate * age
  if speed:
    mean = math.radians(bearing + yaw_rate * age / 2)
    lat += speed * age * math.cos(mean) / METERS_PER_DEG
    lon += speed * age * math.sin(mean) / (METERS_PER_DEG * max(0.01, math.cos(math.radians(lat))))
  return {'lat': round(lat, 7), 'lon': round(lon, 7), 'hdg': round(heading % 360, 1), 'spd': round(speed, 2),
          'age': round(age, 3)}


def private(address: str) -> bool:
  try:
    a, b = (int(part) for part in address.split('.')[:2])
  except ValueError:
    return False
  return a == 10 or (a == 172 and 16 <= b <= 31) or (a == 192 and b == 168)


def broadcast_addresses() -> list[str]:
  """Directed broadcast address of each up, broadcast-capable interface (wlan0 on the phone hotspot)."""
  result = []
  probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  try:
    for _, name in socket.if_nameindex():
      if name.startswith(_SKIP):
        continue
      packed = struct.pack('256s', name.encode()[:15])
      try:
        flags = struct.unpack('H', fcntl.ioctl(probe.fileno(), _SIOCGIFFLAGS, packed)[16:18])[0]
        if flags & (_IFF_UP | _IFF_BROADCAST) != _IFF_UP | _IFF_BROADCAST:
          continue
        fcntl.ioctl(probe.fileno(), _SIOCGIFADDR, packed)  # raises with no IPv4 address
        result.append(socket.inet_ntoa(fcntl.ioctl(probe.fileno(), _SIOCGIFBRDADDR, packed)[20:24]))
      except OSError:
        continue
  finally:
    probe.close()
  return result


def tile_reader() -> Callable[[str], bytes | None]:
  from openpilot.starpilot.navigation.offline_roads import TileStore
  from openpilot.starpilot.navigation.road_tiles import DATA_ZOOM, TileKey
  store = TileStore()

  def read(name: str) -> bytes | None:
    x, y = (int(part) for part in name.split('/'))
    return store.read(TileKey(DATA_ZOOM, x, y))
  return read


def tile_for(raw: bytes, version: int) -> bytes:
  """A stored tile in a format the head unit reads: head unit apps from before 2026-10-30 take only format 1."""
  from openpilot.starpilot.navigation.road_tiles import downgrade_road_tile, road_tile_version
  have = road_tile_version(raw)
  if have is None or have <= version:
    return raw
  return downgrade_road_tile(raw, version)


def tiles_around(lat: float, lon: float) -> list[str]:
  """The tile under the car and its eight neighbours, nearest first."""
  from openpilot.starpilot.navigation.road_tiles import world_xy
  x, y = world_xy(lat, lon)
  cx, cy = int(x), int(y)
  ring = sorted(((dx, dy) for dx in (-1, 0, 1) for dy in (-1, 0, 1)),
                key=lambda d: (cx + d[0] + 0.5 - x) ** 2 + (cy + d[1] + 0.5 - y) ** 2)
  return [f'{cx + dx}/{cy + dy}' for dx, dy in ring]


def media_of(song: dict | None) -> dict | None:
  """The song as the cluster's music card shows it: what is playing, from where, and which cover goes with it."""
  if not isinstance(song, dict) or song.get('status') not in ('playing', 'paused') or not song.get('title'):
    return None
  url = song.get('artwork') or ''
  # precomposed letters: a base letter plus loose accent marks (as some Vietnamese tags come) stacks badly on the
  # head unit's old text renderer, while every precomposed Vietnamese letter is in the card's font
  return {'status': song['status'], 'title': nfc(song['title']), 'artist': nfc(song.get('artist') or ''),
          'source': str(song.get('source') or ''), 'art': cover_id(url) if isinstance(url, str) else 0,
          'dur': song['durationMs'] if isinstance(song.get('durationMs'), int) and song['durationMs'] > 0 else 0}


def elapsed_of(song: dict | None, now_ms: float) -> int | None:
  """How far into the song it is, in ms, for the ring round the play button; None when the player gives no position."""
  if not isinstance(song, dict) or not isinstance(song.get('position'), dict):
    return None
  p, dur = song['position'], song.get('durationMs')
  try:
    value = p['ms'] + max(0.0, now_ms - p['at']) * p.get('speed', 1.0) if song.get('status') == 'playing' else p['ms']
  except (KeyError, TypeError):
    return None
  return int(max(0, min(dur, value) if isinstance(dur, int) and dur > 0 else value))


def nfc(text) -> str:
  return unicodedata.normalize('NFC', str(text))


def cover_id(url: str) -> int:
  return zlib.crc32(url.encode()) & 0xffffffff if url else 0


def fetch_cover(url: str) -> bytes | None:
  """The cover at url (now_playing has already held it to Spotify's image hosts), as a small JPEG; None if it fails."""
  import io
  from PIL import Image
  try:
    with urllib.request.urlopen(url, timeout=COVER_TIMEOUT) as response:
      raw = response.read(COVER_MAX_BYTES + 1)
    if len(raw) > COVER_MAX_BYTES:
      return None
    image = Image.open(io.BytesIO(raw)).convert('RGB').resize((COVER_PX, COVER_PX), Image.LANCZOS)
    out = io.BytesIO()
    image.save(out, 'JPEG', quality=80)
    return out.getvalue()
  except Exception:
    return None


class CerealReader:
  """Everything the feed sends, from cereal; opened on the first hello so an idle feed costs no sockets."""
  SERVICES = ('selfdriveState', 'carControl', 'carState', 'aolAxisState', 'deviceMotion', 'starpilotNavigation',
              'gpsLocationExternal', 'gpsLocation', 'starpilotCarState', 'slcState')

  def __init__(self):
    from openpilot.cereal import messaging
    from openpilot.common.params import Params
    self.sm = messaging.SubMaster(list(self.SERVICES))
    self.params = Params()
    self._metric = (-math.inf, False)
    self._media = (-math.inf, None)
    self._layout = None
    self._heading = HeadingFilter()

  def metric(self, now: float) -> bool:
    if now - self._metric[0] >= 5.:
      self._metric = (now, self.params.get_bool('IsMetric'))
    return self._metric[1]

  def media(self, now: float) -> dict | None:
    """The soloist's now-playing snapshot, read once every MEDIA_EVERY s; None without a song or on any error."""
    if now - self._media[0] >= MEDIA_EVERY:
      song = None
      try:
        from openpilot.starpilot.soloist import now_playing
        if self._layout is None:
          from openpilot.starpilot.soloist.paths import Layout
          self._layout = Layout.default()
        song = now_playing.read(self._layout.status)
      except Exception:
        song = None  # no music card rather than no map
      self._media = (now, song)
    return self._media[1]

  def __call__(self) -> dict:
    from openpilot.starpilot.aol.runtime import current_axis
    from openpilot.starpilot.gps.source import bearing, select_location
    from openpilot.starpilot.navigation.wire import navigation_state
    sm = self.sm
    sm.update(0)
    now_ns = time.monotonic_ns()
    selfdrive, control = sm['selfdriveState'], sm['carControl']
    axis = current_axis(sm, now_ns=now_ns) if sm.seen['aolAxisState'] else None
    mode = mode_of(bool(sm.seen['selfdriveState'] and selfdrive.enabled), bool(selfdrive.experimentalMode),
                   bool(sm.seen['carControl'] and control.latActive),
                   bool(axis.nativeAcknowledged and axis.desiredLongitudinal) if axis is not None else None)
    value = {'mode': mode, 'metric': self.metric(time.monotonic()), 'nav': None, 'route': None, 'fix': None,
             'speed': float(sm['carState'].vEgo) if sm.seen['carState'] else 0., 'yaw_rate': 0.,
             'media': self.media(time.monotonic())}
    motion = sm['deviceMotion'].angularVelocityDevice
    if sm.seen['deviceMotion'] and motion.valid and abs(motion.z) < 1.:
      value['yaw_rate'] = math.degrees(motion.z)  # device z points down: + is a right turn, as compass heading runs
    location = select_location(sm, now_ns)
    if location is not None:
      stamp, gps = location
      heading = self._heading(bearing(gps), value['speed'], value['yaw_rate'], now_ns)
      if heading is not None:
        value['fix'] = (stamp, float(gps.latitude), float(gps.longitude), heading)
    value['limit'] = posted_limit(sm['slcState']) if sm.seen['slcState'] else None
    stamp = sm.logMonoTime['starpilotNavigation']
    state = navigation_state(sm['starpilotNavigation']) if sm.seen['starpilotNavigation'] else None
    if state is not None and state.status == 'guiding' and 0 < stamp <= now_ns <= stamp + NAV_HOLD_NS:
      value['nav'] = {**instruction(state.instruction),
                      'then': instruction(state.nextManeuver) if state.nextManeuver.maneuverType else None,
                      'remaining': round(float(state.instruction.remainingDistanceMeters)),
                      'duration': round(float(state.instruction.remainingDurationSeconds)),
                      'limit': round(float(state.speedLimit), 2) if state.speedLimit > 0 else None}
      points = [(round(point.latitude, 6), round(point.longitude, 6)) for point in state.route]
      if points:
        value['route'] = (f'{state.sessionId}:{state.revision}', points)
    return value


def posted_limit(slc) -> float | None:
  """The speed limit openpilot is going by (m/s), for the cluster's sign when there is no route: the one it accepted,
  else the one it sees now."""
  if slc.hasAccepted and slc.acceptedSpeedLimit > 0:
    return round(float(slc.acceptedSpeedLimit), 2)
  if slc.observationKind == 'valid' and slc.speedLimit > 0:
    return round(float(slc.speedLimit), 2)
  return None


def instruction(step) -> dict:
  return {'type': step.maneuverType, 'modifier': step.maneuverModifier, 'distance': round(float(step.distanceMeters), 1),
          'exit': int(step.exitNumber), 'degrees': int(step.exitDegrees), 'left': bool(step.drivingLeft),
          'text': step.text, 'road': step.roadName}


class HeadUnitFeed:
  def __init__(self, read: Callable[[], dict] | None = None, sock: socket.socket | None = None,
               broadcasts: Callable[[], list[str]] = broadcast_addresses, tiles: Callable[[str], bytes | None] | None = None,
               around: Callable[[float, float], list[str]] = tiles_around, clock: Callable[[], float] = time.monotonic,
               clock_ns: Callable[[], int] = time.monotonic_ns, log: Callable[..., None] = lambda *_, **__: None,
               cover: Callable[[str], bytes | None] | None = None):
    self._read, self.broadcasts, self._tiles, self.around = read, broadcasts, tiles, around
    self._cover = cover or fetch_cover
    self.clock, self.clock_ns, self.log = clock, clock_ns, log
    if sock is None:
      sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
      sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
      sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
      sock.bind(('0.0.0.0', PORT))
    sock.setblocking(False)
    self.sock = sock
    self.peer: tuple[str, int] | None = None
    self.peer_seen = -math.inf
    self.have: set[str] | None = None  # None: this head unit takes no road tiles
    self.tile_version = 1  # newest road tile format the head unit reads
    self.last_beacon = self.last_state = self.last_route = -math.inf
    self.sent_state: dict | None = None
    self.sent_route: str | None = None
    self.sending: list[bytes] = []  # datagrams of the tile on its way
    self.tile_sent: dict[str, float] = {}
    self.last_tile_scan = -math.inf
    self.cover_url = ''                          # the cover asked for last
    self.cover: tuple[int, bytes] | None = None  # (art id, JPEG), set by the download thread
    self.sent_cover: int | None = None
    self.last_cover = -math.inf
    self._cover_thread: threading.Thread | None = None
    self._stop = threading.Event()
    self._thread: threading.Thread | None = None

  def start(self) -> HeadUnitFeed:
    self._thread = threading.Thread(target=self._run, name='aa-hu-feed', daemon=True)
    self._thread.start()
    return self

  def close(self) -> None:
    self._stop.set()
    if self._thread is not None:
      self._thread.join(timeout=2)
    self.sock.close()

  def _run(self) -> None:
    while not self._stop.is_set():
      try:
        idle = self.tick()
      except Exception as error:  # never take Android Auto down with it
        self.log('hu_feed_error', error=f'{type(error).__name__}: {error}')
        idle = True
      self._stop.wait(BEACON_EVERY / 4 if idle else 1 / POSE_HZ)

  def _send(self, kind: str, body: dict, to: tuple[str, int] | None = None) -> None:
    data = json.dumps({'v': VERSION, 't': kind, **body}, separators=(',', ':')).encode()
    try:
      self.sock.sendto(data, to or self.peer)
    except OSError:
      pass

  def receive(self, now: float) -> None:
    while True:
      try:
        data, address = self.sock.recvfrom(4096)
      except (BlockingIOError, InterruptedError):
        return
      except OSError:
        return
      try:
        message = json.loads(data)
      except ValueError:
        continue
      if not (isinstance(message, dict) and message.get('v') == VERSION and message.get('t') == 'h' and private(address[0])):
        continue
      if self.peer != address:
        self.log('hu_feed_peer', port=address[1])
        self.peer, self.sent_state, self.sent_route, self.last_route = address, None, None, -math.inf
        self.sending, self.tile_sent = [], {}
        self.sent_cover, self.last_cover = None, -math.inf
      self.peer_seen = now
      have = message.get('have')
      self.have = {key for key in have[:64] if isinstance(key, str)} if isinstance(have, list) else None
      rt = message.get('rt')
      self.tile_version = rt if isinstance(rt, int) and not isinstance(rt, bool) and rt >= 1 else 1

  def tick(self) -> bool:
    """One pass; True when there is no head unit to feed."""
    now = self.clock()
    self.receive(now)
    if now - self.last_beacon >= BEACON_EVERY:
      self.last_beacon = now
      for address in self.broadcasts():
        self._send('b', {'port': PORT}, (address, PORT))
    if self.peer is not None and now - self.peer_seen > PEER_TIMEOUT:
      self.log('hu_feed_peer_lost')
      self.peer = None
    if self.peer is None:
      return True
    if self._read is None:
      self._read = CerealReader()
    value = self._read()
    song = value.get('media')
    media = media_of(song)
    state = {'mode': value['mode'], 'metric': value['metric'], 'nav': value['nav'], 'limit': value.get('limit'),
             'media': media}
    # pos moves every tick, so it rides along with the state but does not count as a change
    if state != self.sent_state or now - self.last_state >= STATE_EVERY:
      self._send('s', {**state, 'pos': elapsed_of(song, time.time() * 1000) if media is not None else None})  # noqa: TID251 epoch-ms anchors
      self.sent_state, self.last_state = state, now
    if media is not None and media['art']:
      self.send_cover(song['artwork'], media['art'], now)
    route = value.get('route')
    if route is None:
      self.sent_route = None  # the head unit drops the route with the nav, so it goes again the moment nav is back
    elif (route[0] != self.sent_route or now - self.last_route >= ROUTE_EVERY):
      key, points = route
      route_id = zlib.crc32(key.encode()) & 0xffffffff
      chunks = [points[i:i + ROUTE_CHUNK] for i in range(0, len(points), ROUTE_CHUNK)]
      for i, chunk in enumerate(chunks):
        self._send('r', {'id': route_id, 'i': i, 'n': len(chunks), 'pts': chunk})
      self.sent_route, self.last_route = key, now
    pose = carry(value['fix'], value['speed'], value['yaw_rate'], self.clock_ns()) if value['fix'] else None
    if pose is not None:
      self._send('p', pose)
      if self.have is not None:
        self.send_tiles(pose['lat'], pose['lon'], now)
    return False

  def send_tiles(self, lat: float, lon: float, now: float) -> None:
    if not self.sending and now - self.last_tile_scan >= TILE_SCAN_EVERY:
      self.last_tile_scan = now
      if self._tiles is None:
        self._tiles = tile_reader()
      for key in self.around(lat, lon):
        # A lost datagram leaves the tile out of the next hellos, and it goes again after TILE_RETRY.
        if key in self.have or now - self.tile_sent.get(key, -math.inf) < TILE_RETRY:
          continue
        raw = self._tiles(key)
        if raw:
          try:
            raw = tile_for(raw, self.tile_version)
          except ValueError:
            self.tile_sent[key] = now  # a broken file: try again after TILE_RETRY, maybe re-downloaded by then
            continue
          parts = [base64.b64encode(raw[i:i + TILE_CHUNK]).decode() for i in range(0, len(raw), TILE_CHUNK)]
          self.sending = [json.dumps({'v': VERSION, 't': 'm', 'k': key, 'i': i, 'n': len(parts), 'b': part},
                                     separators=(',', ':')).encode() for i, part in enumerate(parts)]
          self.tile_sent[key] = now
          break
    for data in self.sending[:TILE_CHUNKS_PER_TICK]:
      try:
        self.sock.sendto(data, self.peer)
      except OSError:
        pass
    del self.sending[:TILE_CHUNKS_PER_TICK]

  def send_cover(self, url: str, art: int, now: float) -> None:
    """Downloads the cover once per URL, off this thread (up to COVER_TIMEOUT s), then sends it when it is ready
    and again every COVER_EVERY s."""
    if url != self.cover_url:
      self.cover_url = url
      self._cover_thread = threading.Thread(target=self._fetch_cover, args=(url, art), name='aa-hu-cover', daemon=True)
      self._cover_thread.start()
    cover = self.cover
    if cover is None or cover[0] != art or (cover[0] == self.sent_cover and now - self.last_cover < COVER_EVERY):
      return
    raw = cover[1]
    parts = [base64.b64encode(raw[i:i + TILE_CHUNK]).decode() for i in range(0, len(raw), TILE_CHUNK)]
    for i, part in enumerate(parts):
      self._send('a', {'id': art, 'i': i, 'n': len(parts), 'b': part})
    self.sent_cover, self.last_cover = art, now

  def _fetch_cover(self, url: str, art: int) -> None:
    try:
      raw = self._cover(url)
    except Exception as error:
      self.log('hu_feed_cover_error', error=f'{type(error).__name__}: {error}')
      return
    if raw and url == self.cover_url:  # still the song on
      self.cover = (art, raw)
