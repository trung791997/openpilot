"""The head unit's cluster map gets openpilot's mode, the turn and the car's position, and only when it asks."""

import base64
import json
import math

from openpilot.starpilot.system.android_auto import hu_feed as hf

HU = ('10.122.0.20', 40000)
FIX = (1_000_000_000, 33.0, -117.0, 90.0)  # heading east


class Sock:
  def __init__(self):
    self.inbox, self.sent = [], []
  def setblocking(self, flag): pass
  def recvfrom(self, size):
    if not self.inbox:
      raise BlockingIOError
    return self.inbox.pop(0)
  def sendto(self, data, address): self.sent.append((json.loads(data), address))
  def close(self): pass
  def hello(self, address=HU, **body):
    self.inbox.append((json.dumps({'v': hf.VERSION, 't': 'h', **body}).encode(), address))
  def kinds(self, to=HU): return [message['t'] for message, address in self.sent if address == to]


def snapshot(**changes):
  value = {'mode': 'exp', 'metric': False, 'nav': None, 'route': None, 'fix': FIX, 'speed': 20., 'yaw_rate': 0.}
  value.update(changes)
  return value


def make(value=None, tiles=None, cover=None):
  sock, now, reads = Sock(), [100.], []
  def read():
    reads.append(1)
    return value if value is not None else snapshot()
  feed = hf.HeadUnitFeed(read=read, sock=sock, broadcasts=lambda: ['10.122.0.255'], tiles=tiles or (lambda key: None),
                         around=lambda lat, lon: ['1/1', '1/2'], clock=lambda: now[0], clock_ns=lambda: FIX[0] + 100_000_000,
                         cover=cover or (lambda url: None))
  return feed, sock, now, reads


def test_mode_follows_what_openpilot_drives():
  assert hf.mode_of(True, True, True, None) == 'exp'
  assert hf.mode_of(True, False, True, None) == 'chill'
  assert hf.mode_of(False, True, True, None) == 'aol'           # lateral alone, selfdrive off
  assert hf.mode_of(True, True, True, False) == 'aol'           # AOL keeps selfdrive on for lateral alone
  assert hf.mode_of(True, False, True, True) == 'chill'
  assert hf.mode_of(False, True, False, None) == 'off'
  assert hf.mode_of(True, True, False, False) == 'off'


def test_no_head_unit_means_only_a_beacon_and_no_reads():
  feed, sock, now, reads = make()
  assert feed.tick() is True
  assert sock.sent == [({'v': 1, 't': 'b', 'port': hf.PORT}, ('10.122.0.255', hf.PORT))]
  now[0] += 0.5
  feed.tick()
  assert len(sock.sent) == 1 and not reads   # one beacon a second, and the drive is never read


def test_hello_gets_state_route_and_pose_sent_to_that_head_unit_only():
  route = ('s:1', [(33.0 + i * 1e-4, -117.0) for i in range(130)])
  nav = {'road': 'Bolsa Ave', 'type': 'turn', 'modifier': 'left', 'distance': 150.0}
  feed, sock, now, _ = make(snapshot(nav=nav, route=route))
  sock.hello()
  assert feed.tick() is False
  assert sock.kinds() == ['s', 'r', 'r', 'r', 'p']  # 130 points in three
  state = sock.sent[1][0]
  assert state['mode'] == 'exp' and state['nav']['road'] == 'Bolsa Ave'
  parts = [message for message, _ in sock.sent if message['t'] == 'r']
  assert [p['i'] for p in parts] == [0, 1, 2] and {p['n'] for p in parts} == {3} and len({p['id'] for p in parts}) == 1
  assert sum(len(p['pts']) for p in parts) == 130


def test_state_on_change_and_once_a_second_route_every_five():
  value = snapshot(route=('s:1', [(33.0, -117.0), (33.1, -117.0)]))
  feed, sock, now, _ = make(value)
  sock.hello()
  feed.tick()
  sock.sent.clear()
  now[0] += 0.05
  feed.tick()
  assert sock.kinds() == ['p']
  value['mode'] = 'chill'
  now[0] += 0.05
  feed.tick()
  assert sock.kinds() == ['p', 's', 'p']
  sock.sent.clear()
  now[0] += hf.ROUTE_EVERY
  sock.hello()
  feed.tick()
  assert 'r' in sock.kinds()


def test_head_unit_gone_after_its_hellos_stop():
  feed, sock, now, _ = make()
  sock.hello()
  feed.tick()
  now[0] += hf.PEER_TIMEOUT + 0.1
  sock.sent.clear()
  assert feed.tick() is True
  assert sock.kinds() == [] and feed.peer is None


def test_hello_from_outside_the_lan_is_ignored():
  feed, sock, _, reads = make()
  sock.hello(address=('8.8.8.8', 40000))
  assert feed.tick() is True and not reads and feed.peer is None


def test_pose_carries_the_fix_forward_at_speed():
  pose = hf.carry(FIX, 20., 0., FIX[0] + 500_000_000)
  assert pose['hdg'] == 90.0 and pose['age'] == 0.5
  east = (pose['lon'] - FIX[2]) * hf.METERS_PER_DEG * math.cos(math.radians(FIX[1]))
  assert abs(east - 10.) < 0.05 and abs(pose['lat'] - FIX[1]) < 1e-7
  assert hf.carry(FIX, 20., 10., FIX[0] + 500_000_000)['hdg'] == 95.0   # right turn: compass heading grows
  assert hf.carry(FIX, 20., 0., FIX[0] + int((hf.MAX_CARRY + 0.1) * 1e9)) is None


def test_road_tiles_go_only_to_head_units_that_take_them_and_lack_them():
  raw = bytes(range(256)) * 9   # 2304 bytes: three datagrams
  feed, sock, now, _ = make(tiles=lambda key: raw if key == '1/2' else None)
  sock.hello()
  feed.tick()
  assert 'm' not in sock.kinds()          # no "have": this head unit takes no tiles
  sock.hello(have=['1/1'])
  now[0] += 0.05
  feed.tick()
  parts = [message for message, _ in sock.sent if message['t'] == 'm']
  assert [p['i'] for p in parts] == [0, 1, 2] and {p['k'] for p in parts} == {'1/2'}
  assert b''.join(base64.b64decode(p['b']) for p in parts) == raw
  sock.sent.clear()
  now[0] += hf.TILE_SCAN_EVERY
  sock.hello(have=['1/1'])
  feed.tick()
  assert 'm' not in sock.kinds()          # not again straight away
  now[0] += hf.TILE_RETRY
  sock.hello(have=['1/1'])
  feed.tick()
  assert 'm' in sock.kinds()              # still missing: a datagram was lost, so it goes again
  sock.sent.clear()
  now[0] += hf.TILE_RETRY
  sock.hello(have=['1/1', '1/2'])
  feed.tick()
  assert 'm' not in sock.kinds()


def test_named_tiles_go_as_is_to_new_head_units_and_without_names_to_old_ones():
  import numpy as np
  from openpilot.starpilot.navigation.road_tiles import MAJOR, decode_road_tile, encode_road_tile, TileKey
  roads, names = {MAJOR: [np.array([[0, 0], [400, 0]])]}, {MAJOR: ['Bolsa Ave']}
  raw = encode_road_tile(roads, names)

  def received(sock):
    return b''.join(base64.b64decode(m['b']) for m, _ in sock.sent if m['t'] == 'm')

  feed, sock, now, _ = make(tiles=lambda key: raw if key == '1/2' else None)
  sock.hello(have=['1/1'])                 # an app from before street names: no "rt"
  feed.tick()
  old = received(sock)
  assert old[4] == 1 and old == encode_road_tile(roads, version=1), "an old app would drop format 2 and get it resent forever"
  feed, sock, now, _ = make(tiles=lambda key: raw if key == '1/2' else None)
  sock.hello(have=['1/1'], rt=2)
  feed.tick()
  assert received(sock) == raw and decode_road_tile(TileKey(14, 1, 2), received(sock)).name_of(MAJOR, 0) == 'Bolsa Ave'
  feed, sock, now, _ = make(tiles=lambda key: b'SPRT\x02\x01broken' if key == '1/2' else None)
  sock.hello(have=['1/1'])
  feed.tick()
  assert 'm' not in sock.kinds()           # a broken file is skipped, not sent to be dropped


def test_datagrams_fit_one_frame():
  feed, sock, now, _ = make(snapshot(route=('s:1', [(-33.123456, -117.123456)] * 300)),
                            tiles=lambda key: bytes(5000))
  sock.hello(have=[])
  feed.tick()
  assert max(len(json.dumps(message, separators=(',', ':'))) for message, _ in sock.sent) < 1472


def test_private_addresses():
  assert hf.private('10.122.1.2') and hf.private('192.168.43.5') and hf.private('172.20.0.1')
  assert not hf.private('8.8.8.8') and not hf.private('172.32.0.1') and not hf.private('bogus')


def test_a_stop_keeps_the_heading_it_arrived_with():
  heading, second = hf.HeadingFilter(), 1_000_000_000
  assert heading(92.0, 8.0, 0., second) == 92.0
  assert heading(None, 0.0, 0., 2 * second) == 92.0     # stopped: GPS has no bearing
  assert heading(120.0, 0.0, 0., 3 * second) == 92.0    # nor a real one when it reports one
  assert hf.HeadingFilter()(None, 0.0, 0., second) is None
  assert heading(None, 5.0, 0., 4 * second) is None     # moving without a bearing: the old one is stale


def test_heading_noise_is_damped_and_the_gyro_carries_turns():
  heading, tenth = hf.HeadingFilter(), 100_000_000
  heading(90.0, 20.0, 0., tenth)
  assert heading(95.0, 20.0, 0., 2 * tenth) == 91.5       # a noisy bearing moves it 30 percent
  assert round(heading(92.5, 20.0, 10., 3 * tenth), 6) == 92.5  # a 10 deg/s right turn over 0.1 s
  wrap = hf.HeadingFilter()
  wrap(359.0, 20.0, 0., tenth)
  assert round(wrap(9.0, 20.0, 0., 2 * tenth), 6) == 2.0  # across north the short way


def test_the_route_goes_again_as_soon_as_nav_comes_back():
  nav = {'icon': 'turn_left', 'distance': 400}
  value = snapshot(nav=nav, route=('s:1', [(33.0, -117.0), (33.0, -116.99)]))
  feed, sock, now, reads = make(value)
  sock.hello()
  feed.tick()
  assert sock.kinds().count('r') == 1
  value.update(nav=None, route=None)  # the head unit drops its route
  now[0] += 1
  feed.tick()
  value.update(nav=nav, route=('s:1', [(33.0, -117.0), (33.0, -116.99)]))
  now[0] += 1
  feed.tick()
  assert sock.kinds().count('r') == 2  # not five seconds later


def test_sign_shows_the_accepted_limit_then_the_one_seen_and_nothing_unknown():
  from types import SimpleNamespace as N
  assert hf.posted_limit(N(hasAccepted=True, acceptedSpeedLimit=13.4112, observationKind='unknown', speedLimit=0)) == 13.41
  assert hf.posted_limit(N(hasAccepted=False, acceptedSpeedLimit=0, observationKind='valid', speedLimit=15.6464)) == 15.65
  assert hf.posted_limit(N(hasAccepted=False, acceptedSpeedLimit=0, observationKind='unknown', speedLimit=15.6)) is None


ART = 'https://i.scdn.co/image/ab67616d0000b273abc'
SONG = {'source': 'spotify', 'status': 'playing', 'title': 'Another Life', 'artist': 'RÜFÜS DU SOL', 'album': 'Surrender',
        'durationMs': 200000, 'position': {'ms': 1000, 'at': 1, 'speed': 1.0}, 'artwork': ART}


def tick_and_wait(feed):
  feed.tick()
  if feed._cover_thread is not None:
    feed._cover_thread.join(2)


def test_the_song_rides_in_the_state_without_its_position():
  feed, sock, now, _ = make(snapshot(media=dict(SONG)))
  sock.hello()
  feed.tick()
  state = next(message for message, _ in sock.sent if message['t'] == 's')
  assert state['media'] == {'status': 'playing', 'title': 'Another Life', 'artist': 'RÜFÜS DU SOL', 'source': 'spotify',
                            'art': hf.cover_id(ART), 'dur': 200000}
  assert state['pos'] == 200000  # anchored long ago: held at the end
  assert hf.elapsed_of({**SONG, 'position': {'ms': 1000, 'at': 0, 'speed': 1.0}}, 30000) == 31000
  assert hf.elapsed_of({**SONG, 'status': 'paused'}, 30000) == 1000
  assert hf.elapsed_of({**SONG, 'position': None}, 30000) is None
  assert hf.cover_id(ART) != 0 and hf.cover_id('') == 0


def test_no_song_no_card():
  feed, sock, now, _ = make()
  sock.hello()
  feed.tick()
  assert next(message for message, _ in sock.sent if message['t'] == 's')['media'] is None
  assert hf.media_of({**SONG, 'status': 'stopped'}) is None and hf.media_of({**SONG, 'title': ''}) is None
  assert hf.media_of({**SONG, 'artwork': ''})['art'] == 0
  decomposed = 'Tha\u0301c Ni\u0303ng'  # a + combining acute, i + combining tilde
  assert hf.media_of({**SONG, 'title': decomposed})['title'] == 'Th\u00e1c N\u0129ng'


def test_the_song_moving_on_does_not_resend_the_state():
  value = snapshot(media=dict(SONG))
  feed, sock, now, _ = make(value)
  sock.hello()
  feed.tick()
  sock.sent.clear()
  value['media'] = {**SONG, 'position': {'ms': 5000, 'at': 2, 'speed': 1.0}}
  now[0] += 0.05
  feed.tick()
  assert 's' not in sock.kinds()
  value['media'] = {**SONG, 'status': 'paused'}
  now[0] += 0.05
  feed.tick()
  assert 's' in sock.kinds()


def test_cover_is_fetched_once_then_sent_in_chunks_and_repeated():
  raw, fetched = bytes(range(256)) * 9, []   # 2304 bytes: three datagrams
  def cover(url):
    fetched.append(url)
    return raw
  feed, sock, now, _ = make(snapshot(media=dict(SONG)), cover=cover)
  sock.hello()
  tick_and_wait(feed)
  now[0] += 0.05
  feed.tick()
  parts = [message for message, _ in sock.sent if message['t'] == 'a']
  assert [p['i'] for p in parts] == [0, 1, 2] and {p['n'] for p in parts} == {3} and {p['id'] for p in parts} == {hf.cover_id(ART)}
  assert b''.join(base64.b64decode(p['b']) for p in parts) == raw
  assert max(len(json.dumps(message, separators=(',', ':'))) for message, _ in sock.sent) < 1472
  sock.sent.clear()
  now[0] += 1
  sock.hello()
  feed.tick()
  assert 'a' not in sock.kinds()            # not again straight away
  now[0] += hf.COVER_EVERY
  sock.hello()
  feed.tick()
  assert sock.kinds().count('a') == 3       # every COVER_EVERY s while the song is on
  assert fetched == [ART]                   # downloaded once
  sock.sent.clear()
  sock.hello(address=('10.122.0.21', 40000))
  now[0] += 1
  feed.tick()
  assert sock.kinds(('10.122.0.21', 40000)).count('a') == 3   # a new head unit gets it at once


def test_no_cover_sent_without_a_song_or_when_the_download_fails():
  value = snapshot(media={**SONG, 'artwork': ''})
  fetched = []
  feed, sock, now, _ = make(value, cover=lambda url: fetched.append(url))
  sock.hello()
  tick_and_wait(feed)
  assert not fetched and 'a' not in sock.kinds()
  value['media'] = dict(SONG)
  now[0] += 0.05
  tick_and_wait(feed)
  now[0] += 0.05
  feed.tick()
  assert fetched == [ART] and 'a' not in sock.kinds()   # the download came back empty


def test_a_new_song_drops_the_old_cover():
  value = snapshot(media=dict(SONG))
  feed, sock, now, _ = make(value, cover=lambda url: url.encode())
  sock.hello()
  tick_and_wait(feed)
  other = ART + 'def'
  value['media'] = {**SONG, 'title': 'Innerbloom', 'artwork': other}
  sock.sent.clear()
  now[0] += 0.05
  tick_and_wait(feed)
  now[0] += 0.05
  feed.tick()
  parts = [message for message, _ in sock.sent if message['t'] == 'a']
  assert {p['id'] for p in parts} == {hf.cover_id(other)} and base64.b64decode(parts[0]['b']) == other.encode()


def test_fetch_cover_shrinks_to_a_small_jpeg(monkeypatch):
  import io
  from PIL import Image
  big = io.BytesIO()
  Image.new('RGB', (640, 640), (200, 80, 40)).save(big, 'JPEG')
  class Response(io.BytesIO):
    def __enter__(self): return self
    def __exit__(self, *_): pass
  monkeypatch.setattr(hf.urllib.request, 'urlopen', lambda url, timeout: Response(big.getvalue()))
  out = hf.fetch_cover(ART)
  assert out[:2] == b'\xff\xd8' and Image.open(io.BytesIO(out)).size == (hf.COVER_PX, hf.COVER_PX) and len(out) < 4000
  monkeypatch.setattr(hf.urllib.request, 'urlopen', lambda url, timeout: Response(bytes(hf.COVER_MAX_BYTES + 1)))
  assert hf.fetch_cover(ART) is None
  def down(url, timeout):
    raise OSError('no network')
  monkeypatch.setattr(hf.urllib.request, 'urlopen', down)
  assert hf.fetch_cover(ART) is None
