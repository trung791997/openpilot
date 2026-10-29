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


def make(value=None, tiles=None):
  sock, now, reads = Sock(), [100.], []
  def read():
    reads.append(1)
    return value if value is not None else snapshot()
  feed = hf.HeadUnitFeed(read=read, sock=sock, broadcasts=lambda: ['10.122.0.255'], tiles=tiles or (lambda key: None),
                         around=lambda lat, lon: ['1/1', '1/2'], clock=lambda: now[0], clock_ns=lambda: FIX[0] + 100_000_000)
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
  assert hf.hold_heading(92.0, 8.0, None) == (92.0, 92.0)
  assert hf.hold_heading(None, 0.0, 92.0) == (92.0, 92.0)  # stopped: GPS has no bearing
  assert hf.hold_heading(None, 0.0, None) == (None, None)
  assert hf.hold_heading(None, 5.0, 92.0) == (None, None)  # moving without a bearing: the old one is stale
