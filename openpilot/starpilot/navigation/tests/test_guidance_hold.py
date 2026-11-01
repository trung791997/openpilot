"""A short GPS gap keeps the last turn on the car's screens; arrival, End navigation and reroutes still clear it."""

import collections
from types import SimpleNamespace

from openpilot.starpilot.navigation.guidance_hold import HOLD_NS, GuidanceHold
from openpilot.starpilot.system.android_auto import hu_feed as hf

S = 1_000_000_000
KEY = ('session', 'rev-1')

# navigationd's messages in drive 00000075--4ad120a8a2 (2026-10-31, rlog seg 8-9, seconds from the drive start): the
# car's GPS stopped for ~7 s at 26 mph, then one unusable fix at 46 mph. Status per message, as logged.
GAP_RUN = ([(500.05, 'guiding'), (501.03, 'guiding'), (501.22, 'guiding'), (502.44, 'guiding'), (503.44, 'guiding'),
            (504.22, 'waitingForLocation'), (505.43, 'waitingForLocation'), (506.42, 'waitingForLocation'),
            (507.62, 'waitingForLocation'), (508.03, 'guiding'), (509.03, 'guiding'), (510.02, 'guiding')])
BLIP_RUN = [(584.02, 'guiding'), (585.02, 'guiding'), (585.82, 'guiding'), (586.82, 'waitingForLocation'),
            (587.02, 'guiding'), (588.02, 'guiding')]


def step(distance=400., remaining=5000, duration=600):
  return {'type': 'turn', 'modifier': 'left', 'text': 'Country Club Road / FM 1378', 'distance': distance,
          'remaining': remaining, 'duration': duration}


def test_a_gap_keeps_the_step_and_counts_it_down_at_the_cars_speed():
  hold = GuidanceHold()
  assert hold('guiding', KEY, step(400.), 0, 10.) == (step(400.), False)
  shown, held = hold('waitingForLocation', KEY, None, S // 2, 10.)
  assert held and shown['text'] == 'Country Club Road / FM 1378'
  for tenth in range(6, 31):
    shown, held = hold('waitingForLocation', KEY, None, tenth * S // 10, 10.)
  assert held and shown['distance'] == 375. and shown['remaining'] == 4975 and shown['duration'] == 598
  assert isinstance(shown['remaining'], int) and isinstance(shown['duration'], int)
  shown, _ = hold('waitingForLocation', KEY, None, 3 * S + S // 2, 10.)
  assert shown['distance'] == 365.
  shown, _ = hold('waitingForLocation', KEY, None, 3 * S + 3 * S // 4, 10.)
  assert shown['distance'] == 365.  # moves once a second, so the feed does not resend every read
  for second in range(4, 20):
    shown, _ = hold('waitingForLocation', KEY, None, second * S, 30.)  # well past the turn
  assert shown['distance'] == 0.
  assert hold('guiding', KEY, step(150.), 15 * S, 10.) == (step(150.), False)


def test_a_long_pause_between_reads_adds_no_distance():
  hold = GuidanceHold()
  hold('guiding', KEY, step(400.), 0, 10.)
  shown, _ = hold('waitingForLocation', KEY, None, 10 * S, 10.)
  assert shown['distance'] == 390.


def test_route_ends_and_reroutes_clear_the_step_at_once():
  for status in ('noDestination', 'arrived', 'routing', 'routeUnavailable', 'disabled', 'needsKey'):
    hold = GuidanceHold()
    hold('guiding', KEY, step(), 0, 10.)
    assert hold(status, KEY, None, S // 5, 10.) == (None, False), status
    assert hold('waitingForLocation', KEY, None, S // 2, 10.) == (None, False), status  # and stays gone


def test_a_new_destination_during_a_gap_is_not_held():
  hold = GuidanceHold()
  hold('guiding', KEY, step(), 0, 10.)
  assert hold('waitingForLocation', ('session', 'rev-2'), None, S, 10.) == (None, False)


def test_a_gap_longer_than_the_hold_drops_the_step():
  hold = GuidanceHold()
  hold('guiding', KEY, step(), 0, 10.)
  assert hold('waitingForLocation', KEY, None, HOLD_NS, 10.)[1]
  assert hold('waitingForLocation', KEY, None, HOLD_NS + 1, 10.) == (None, False)


def test_nothing_held_without_a_step_first():
  assert GuidanceHold()('waitingForLocation', KEY, None, S, 10.) == (None, False)


# ---------------------------------------------------------------- the dash feed, on the drive's message pattern

def instruction(distance):
  return SimpleNamespace(maneuverType='turn', maneuverModifier='left', distanceMeters=distance, exitNumber=0,
                         exitDegrees=0, drivingLeft=False, text='Country Club Road / FM 1378', roadName='Country Club Road',
                         roadRef='FM 1378', remainingDistanceMeters=distance + 4000., remainingDurationSeconds=600.)


def navigation(status, distance, revision='rev-1'):
  guiding = status == 'guiding'
  return SimpleNamespace(navigation=SimpleNamespace(
    version=2, status=status, sessionId='session', revision=revision, speedLimit=0.,
    instruction=instruction(distance) if guiding else instruction(0.),
    nextManeuver=SimpleNamespace(maneuverType=''), routeCongestion=[],
    route=[SimpleNamespace(latitude=33.0, longitude=-96.0), SimpleNamespace(latitude=33.01, longitude=-96.0)]
    if guiding or status == 'waitingForLocation' else []))


class Sub:
  def __init__(self, speed):
    self.messages = {'carState': SimpleNamespace(vEgo=speed, brakePressed=False), 'carControl': SimpleNamespace(latActive=False),
                     'selfdriveState': SimpleNamespace(enabled=False, experimentalMode=False),
                     'deviceMotion': SimpleNamespace(angularVelocityDevice=SimpleNamespace(valid=False, z=0.))}
    self.seen = collections.defaultdict(bool, carState=True)
    self.logMonoTime = {'starpilotNavigation': 0}
    self.valid = {}
  def update(self, timeout): pass
  def __getitem__(self, name): return self.messages.get(name, SimpleNamespace())
  def publish(self, nav, at_ns):
    self.messages['starpilotNavigation'] = nav
    self.seen['starpilotNavigation'], self.logMonoTime['starpilotNavigation'] = True, at_ns


def reader(sub, monkeypatch, clock):
  read = hf.CerealReader.__new__(hf.CerealReader)
  read.sm, read._metric, read._media, read._heading = sub, (float('inf'), False), (float('inf'), None), hf.HeadingFilter()
  read._guidance, read._route = GuidanceHold(), None
  monkeypatch.setattr(hf.time, 'monotonic_ns', lambda: clock[0])
  return read


def replay(run, speed, monkeypatch, start_distance=900.):
  """Read the feed at 20 Hz through navigationd's logged messages; the true distance falls at the car's speed."""
  sub, clock = Sub(speed), [0]
  read = reader(sub, monkeypatch, clock)
  t0, messages, out = run[0][0], list(run), []
  t = t0
  while t <= run[-1][0] + 0.5:
    while messages and messages[0][0] <= t:
      at, status = messages.pop(0)
      sub.publish(navigation(status, start_distance - speed * (at - t0)), round(at * S))
    clock[0] = round(t * S)
    out.append((t, read()))
    t = round(t + 0.05, 2)
  return out


def test_drive_75_gap_keeps_the_dash_turn_card_and_route(monkeypatch):
  speed = 26 / 2.237
  out = replay(GAP_RUN, speed, monkeypatch)
  assert all(value['nav'] is not None and value['route'] is not None for _, value in out)
  held = [(t, value['nav']) for t, value in out if value['nav'].get('held')]
  assert held and 504.2 <= held[0][0] < 504.3 and 507.9 < held[-1][0] < 508.05  # the whole 3.8 s gap
  distances = [nav['distance'] for _, nav in held]
  assert distances == sorted(distances, reverse=True) and distances[0] > distances[-1]
  for t, nav in held:  # within one second's travel of where the car really is
    assert abs(nav['distance'] - (900. - speed * (t - 500.05))) < speed * 1.2
  assert all('held' not in value['nav'] for t, value in out if t >= 508.05)


def test_drive_75_single_bad_fix_does_not_blink_the_card(monkeypatch):
  out = replay(BLIP_RUN, 46 / 2.237, monkeypatch, start_distance=2000.)
  assert all(value['nav'] is not None for _, value in out)
  assert sum(1 for _, value in out if value['nav'].get('held')) == 4  # 586.82-587.02


def test_dash_clears_at_once_on_end_navigation_and_after_the_hold(monkeypatch):
  out = replay([(1., 'guiding'), (2., 'guiding'), (3., 'noDestination'), (4., 'noDestination')], 10., monkeypatch)
  assert all(value['nav'] is not None for t, value in out if t < 3.)
  assert all(value['nav'] is None and value['route'] is None for t, value in out if t >= 3.)
  long_gap = [(1., 'guiding')] + [(float(t), 'waitingForLocation') for t in range(2, 27)]
  out = replay(long_gap, 10., monkeypatch)
  assert all(value['nav'] is not None for t, value in out if t <= 21.)
  assert all(value['nav'] is None and value['route'] is None for t, value in out if t > 21.05)


# ---------------------------------------------------------------- the car's own cluster (Android Auto nav status)

def test_drive_75_gap_keeps_the_cluster_turn_and_end_navigation_still_clears_it(monkeypatch):
  from openpilot.common import params
  from openpilot.starpilot.system.android_auto import nav_status as ns
  monkeypatch.setattr(params, 'Params', lambda: SimpleNamespace(get_bool=lambda name: False))
  clock, sub = [0], Sub(26 / 2.237)
  sub.valid = collections.defaultdict(lambda: True)
  monkeypatch.setattr(ns.time, 'monotonic_ns', lambda: clock[0])
  feed = ns.NavFeed.__new__(ns.NavFeed)
  feed._sm, feed._guidance = sub, GuidanceHold()
  shown = []
  for at, status in GAP_RUN + [(511.0, 'noDestination')]:
    sub.publish(navigation(status, 900. - 11.6 * (at - 500.05)), round(at * S))
    clock[0] = round(at * S) + S // 10
    shown.append((status, feed._read_cereal()))
  assert all(snap is not None and snap['state'] == 'guiding' for status, snap in shown[:-1])
  held = [snap['distance'] for status, snap in shown if status == 'waitingForLocation']
  assert held == sorted(held, reverse=True) and len(held) == 4
  assert shown[-1][1] is None
