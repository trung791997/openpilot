"""Mapbox turn-by-turn for the car's own displays (instrument cluster, HUD), as a phone sends it.

Only head units that advertise the navigation status service receive it, and only while a route is
guiding. The route is polled on its own thread; the session loop just compares and sends.

Wire facts: mrmees/open-android-auto ``oaa/navigation`` and ``oaa/control/ChannelDescriptorData`` (reference only).
The legacy turn/distance pair and the cluster type follow opencardev/aasdk ``aap_protobuf/service/navigationstatus``
(NavigationNextTurnEvent, NavigationStatusService), which matches the captured phone distance packet.
"""
from __future__ import annotations

import math
import threading
import time
from collections.abc import Callable
from functools import lru_cache

from openpilot.starpilot.system.android_auto.wire import field, one, parse_fields, signed

MSG_CHANNEL_OPEN_REQUEST = 0x07
MSG_CHANNEL_OPEN_RESPONSE = 0x08
NAV_FOCUS_REQUEST = 0x0d
NAV_FOCUS_NOTIFICATION = 0x0e
FOCUS_NATIVE, FOCUS_PROJECTED = 1, 2
CLUSTER_START = 0x8001
CLUSTER_STOP = 0x8002
NAV_STATUS = 0x8003
NAV_TURN = 0x8004
NAV_DISTANCE = 0x8005
NAV_NOTIFICATION = 0x8006
NAV_CURRENT_POSITION = 0x8007

NAVIGATION_SERVICE = 8  # ChannelDescriptor.navigation_channel
TYPE_IMAGE, TYPE_ENUM = 1, 2  # NavigationStatusService.InstrumentClusterType
STATUS_ACTIVE, STATUS_INACTIVE, STATUS_REROUTING = 1, 2, 3
SIDE_LEFT, SIDE_RIGHT, SIDE_UNSPECIFIED = 1, 2, 3
(EVENT_DEPART, EVENT_NAME_CHANGE, EVENT_SLIGHT_TURN, EVENT_TURN, EVENT_SHARP_TURN, EVENT_U_TURN, EVENT_ON_RAMP,
 EVENT_OFF_RAMP, EVENT_FORK, EVENT_MERGE, EVENT_ROUNDABOUT_ENTER, EVENT_ROUNDABOUT_EXIT,
 EVENT_ROUNDABOUT_ENTER_AND_EXIT, EVENT_STRAIGHT) = range(1, 15)
EVENT_DESTINATION = 19
UNIT_METERS, UNIT_KILOMETERS, UNIT_KILOMETERS_P1, UNIT_MILES, UNIT_MILES_P1, UNIT_FEET = 1, 2, 3, 4, 5, 6

# Head units below protocol 2 predate NavigationState (0x8006/0x8007); a phone sends them the legacy pair only.
# Peter's 2019 Civic (1.3, image cluster 480x480) showed the arrow and distance but never the road name while
# 0x8006 followed every legacy turn (session-000044, 2026-10-27; road text was set, e.g. 'East Plano Parkway').
MODERN_MIN_PROTOCOL = (2, 0)

# The Civic cluster reads the legacy turn/distance pair. Session-000036 (2026-10-26) dropped video ~6 s
# after it 3 of 3 times, but that turn had the side and event in each other's fields and a modern maneuver number
# (36 = straight) where the legacy event belongs; this encoder fixes that. If projection still ends within
# LEGACY_PROBATION of the first legacy turn, the pair is switched off until the daemon restarts.
SEND_LEGACY = True
LEGACY_PROBATION = 20.0
_legacy_blocked = False
STEP_TIMEOUT = 3.0
START_GRACE = 2.0  # some head units never send the start request
CHECK_EVERY = 0.25
MIN_INTERVAL = 1.0
STALE_AFTER = 3.0
FALLBACK_SPEED = 13.4  # m/s, for the time to the turn when the route gives no pace


def navigation_channel(channels: list[dict]) -> dict | None:
  return next((channel for channel in channels if NAVIGATION_SERVICE in channel.get('services', [])), None)


def parse_navigation(raw: bytes) -> dict:
  """NavigationChannel: minimum interval, whether the car wants an image or an enum, and the image size."""
  fields = parse_fields(raw)
  options = parse_fields(one(fields, 3)) if isinstance(one(fields, 3), bytes) else {}
  return {'min_interval_ms': int(one(fields, 1, 0) or 0), 'type': one(fields, 2), 'image_width': one(options, 2),
          'image_height': one(options, 1), 'image_depth': one(options, 3)}


def _normal(value: str) -> str:
  return (value or '').replace(' ', '').replace('_', '').lower()


ROUNDABOUTS = ('roundabout', 'rotary')
# Exit angle from the modifier when Mapbox gives no degrees, right-hand traffic; mirrored for left-hand.
EXIT_ANGLES = {'sharpright': 45, 'right': 90, 'slightright': 135, 'straight': 180, 'slightleft': 225, 'left': 270,
               'sharpleft': 315, 'uturn': 360}


def maneuver_event(maneuver_type: str, modifier: str, driving_left: bool = False) -> tuple[int, int]:
  """Mapbox maneuver -> (event, turn side).

  The Civic cluster (head unit NavigationFocusManager, jadx, static) draws on/off ramps, merges and forks as a
  slight arrow whatever the modifier, and gives them the same point class as a plain turn, so ramps and merges go
  out as the turn their modifier says. Forks keep their own class. Mapbox's 'roundabout turn' is a small
  roundabout it says to show as an ordinary turn. For roundabouts the side is the traffic side, not the exit side.
  """
  kind, mod = _normal(maneuver_type), _normal(modifier)
  side = SIDE_LEFT if 'left' in mod else SIDE_RIGHT if 'right' in mod else SIDE_UNSPECIFIED
  if kind in ROUNDABOUTS or kind in ('exitroundabout', 'exitrotary'):
    return (EVENT_ROUNDABOUT_ENTER_AND_EXIT if kind in ROUNDABOUTS else EVENT_ROUNDABOUT_EXIT,
            SIDE_LEFT if driving_left else SIDE_RIGHT)
  fixed = {'depart': EVENT_DEPART, 'arrive': EVENT_DESTINATION, 'fork': EVENT_FORK}
  if kind in fixed:
    return fixed[kind], side
  if mod == 'uturn':
    return EVENT_U_TURN, side
  if 'slight' in mod:
    return EVENT_SLIGHT_TURN, side
  if 'sharp' in mod:
    return EVENT_SHARP_TURN, side
  if 'left' in mod or 'right' in mod:
    return EVENT_TURN, side
  return (EVENT_NAME_CHANGE if kind == 'newname' else EVENT_STRAIGHT), SIDE_UNSPECIFIED


def roundabout_exit(maneuver_type: str, modifier: str, degrees: int = 0, driving_left: bool = False) -> int | None:
  """Exit angle for a roundabout turn event, or None for anything else.

  Mapbox and Android Auto both measure it in the direction of travel with 180 = straight through. The head unit
  turns it into the arrow (right-hand traffic: 90 = right, 270 = left); with no angle it draws the arrow pointing
  back, which is what every roundabout showed before this was sent.
  """
  if _normal(maneuver_type) not in ROUNDABOUTS:
    return None
  if 0 < degrees <= 360:
    return degrees
  angle = EXIT_ANGLES.get(_normal(modifier), 180)
  return 360 - angle if driving_left and angle != 360 else angle


def display_distance(meters: float, metric: bool) -> tuple[int, int]:
  """(value x 1000, unit), rounded the way the comma's own navigation card shows it."""
  if metric:
    if meters < 1000:
      step = 25 if meters < 500 else 50
      return round(meters / step) * step * 1000, UNIT_METERS
    kilometers = meters / 1000
    return (round(kilometers) * 1000, UNIT_KILOMETERS) if kilometers >= 10 else (round(kilometers * 10) * 100, UNIT_KILOMETERS_P1)
  feet = meters * 3.28084
  if feet < 1000:
    step = 10 if feet <= 100 else 50
    return round(feet / step) * step * 1000, UNIT_FEET
  miles = feet / 5280
  return (round(miles) * 1000, UNIT_MILES) if miles >= 10 else (round(miles * 10) * 100, UNIT_MILES_P1)


def _java_round(x: float) -> int:
  return math.floor(x + 0.5)  # Math.round, not Python's round-half-even


def cluster_distance(meters: float) -> tuple[tuple[int, int], tuple[int, int]]:
  """What a Honda head unit draws on the cluster, in feet/miles and in metres/km.

  It ignores our rounded display value and redoes the rounding from distance_meters
  (NavigationFocusManager.getRemainingDistance, static read of the Mitsubishi firmware):
  100 ft steps under 1000 ft then 0.1 mi, or 10 m steps under 1000 m then 0.1 km. The
  progress-bar buckets (getSegmentPercentage) change only where these numbers do. The
  unit comes from the car's own setting, which we cannot see, so both are returned.
  """
  sent = max(0, round(meters))  # field 1 of the distance message
  feet = _java_round(int(sent / 0.3048) / 100) * 100
  imperial = (feet, 3) if feet < 1000 else (_java_round(feet / 5280 * 10), 4)
  metric = (_java_round(sent / 10) * 10, 0) if sent < 1000 else (_java_round(sent / 100), 1)
  return imperial, metric


@lru_cache(maxsize=64)
def turn_image(maneuver_type: str, modifier: str, width: int, height: int) -> bytes | None:
  """The comma's own turn arrow as a PNG of the size the car asks for."""
  try:
    import io
    from PIL import Image
    from openpilot.starpilot.ui.navigation_icons import ASSETS, icon_name
    with Image.open(ASSETS / icon_name(maneuver_type, modifier)) as icon:
      icon = icon.convert('RGBA')
      icon.thumbnail((width, height))
      canvas = Image.new('RGBA', (width, height), (0, 0, 0, 0))
      canvas.paste(icon, ((width - icon.width) // 2, (height - icon.height) // 2), icon)
      out = io.BytesIO()
      canvas.save(out, 'PNG')
      return out.getvalue()
  except Exception:
    return None


def status_message(status: int) -> bytes:
  return field(1, status)


def turn_message(road: str, event: int, side: int, image: bytes | None = None, angle: int | None = None,
                 exit_number: int = 0) -> bytes:
  # NavigationNextTurnEvent: road = 1, turn_side = 2, event = 3 (legacy NextTurnEnum), image = 4,
  # turn_angle = 5, turn_number = 6 (roundabouts only)
  return (field(1, road[:128]) + field(2, side) + field(3, event) + (field(4, image) if image else b'') +
          (field(5, angle) if angle is not None else b'') + (field(6, exit_number) if angle is not None and exit_number else b''))


def distance_message(meters: float, seconds: float, metric: bool) -> bytes:
  value, unit = display_distance(meters, metric)
  return field(1, max(0, round(meters))) + field(2, max(0, round(seconds))) + field(3, value) + field(4, unit)


def modern_maneuver(maneuver_type: str, modifier: str) -> int:
  """Android Maneuver.TYPE_* wire values, which include direction in the enum."""
  kind, mod = _normal(maneuver_type), _normal(modifier)
  right = 'right' in mod
  directional = right or 'left' in mod
  if kind == 'depart':
    return 1
  if kind == 'arrive':
    return (42 if right else 41) if directional else 39
  if kind == 'merge':
    return (28 if right else 27) if directional else 29
  if kind == 'fork':
    return (26 if right else 25) if directional else 36
  if kind in ('roundabout', 'rotary', 'roundaboutturn', 'exitroundabout', 'exitrotary'):
    # Mapbox's modifier describes the exit direction, not traffic circulation.
    # Do not invent a clockwise/counterclockwise arrow without that information.
    return 0
  if kind == 'onramp' and directional:
    return (13 if 'slight' in mod else 17 if 'sharp' in mod else 15) + int(right)
  if kind == 'offramp' and directional:
    return (21 if 'slight' in mod else 23) + int(right)
  if mod == 'uturn':
    return 0  # Mapbox does not specify the U-turn side.
  if directional:
    return (5 if 'slight' in mod else 9 if 'sharp' in mod else 7) + int(right)
  return 2 if kind == 'newname' else 36


def notification_message(road: str, maneuver: int) -> bytes:
  # NavigationState.steps[].{maneuver.type, road.name, cue.alternate_text}. Head units that draw their
  # own cluster text (Honda) read the cue, not road.name: with road.name alone the arrow showed blank.
  road = road[:128]
  return field(1, field(1, field(1, maneuver)) + field(2, field(1, road)) + (field(4, field(1, road)) if road else b''))


def current_position_message(meters: float, seconds: float, metric: bool) -> bytes:
  value, unit = display_distance(meters, metric)
  text = f'{value / 1000:.1f}' if unit in (UNIT_KILOMETERS_P1, UNIT_MILES_P1) else str(value // 1000)
  distance = field(1, max(0, round(meters))) + field(2, text) + field(3, unit)
  # NavigationNextTurnDistanceEvent.step_distance.{distance, time_to_step_seconds}
  return field(1, field(1, distance) + field(2, max(0, round(seconds))))


class NavFeed:
  """Guidance from the Mapbox route owner, read off the Android Auto loop."""
  def __init__(self, read: Callable[[], dict | None] | None = None):
    self.read = read or self._read_cereal
    self._lock = threading.Lock()
    self._stop = threading.Event()
    self._value: dict | None = None
    self._sm = None
    self._thread = threading.Thread(target=self._run, name='aa-nav-status', daemon=True)
    self._thread.start()

  def _read_cereal(self) -> dict | None:
    from openpilot.starpilot.navigation.wire import navigation_state
    if self._sm is None:
      from openpilot.cereal import messaging
      self._sm = messaging.SubMaster(['starpilotNavigation'])
    self._sm.update(0)
    stamp = self._sm.logMonoTime['starpilotNavigation']
    if not self._sm.valid['starpilotNavigation'] or not 0 < stamp <= time.monotonic_ns() <= stamp + int(STALE_AFTER * 1e9):
      return None
    state = navigation_state(self._sm['starpilotNavigation'])
    if state is None or state.status not in ('guiding', 'routing'):
      return None
    from openpilot.common.params import Params
    snapshot = {'state': state.status, 'metric': Params().get_bool('IsMetric')}
    if state.status == 'guiding':
      instruction = state.instruction
      snapshot.update(road=instruction.text, type=instruction.maneuverType, modifier=instruction.maneuverModifier,
                      distance=float(instruction.distanceMeters), remaining=float(instruction.remainingDistanceMeters),
                      duration=float(instruction.remainingDurationSeconds), degrees=int(instruction.exitDegrees),
                      exit=int(instruction.exitNumber), driving_left=bool(instruction.drivingLeft))
    return snapshot

  def _run(self) -> None:
    while not self._stop.is_set():
      try:
        value = self.read()
      except Exception:
        value = None
      with self._lock:
        self._value = value
      self._stop.wait(0.25)

  def snapshot(self) -> dict | None:
    with self._lock:
      return self._value

  def close(self) -> None:
    self._stop.set()
    self._thread.join(timeout=2)


class NavStatus:
  def __init__(self, session, channel: dict, feed, log: Callable[..., None], clock: Callable[[], float] = time.monotonic):
    self.session, self.channel_id, self.feed, self.log, self.clock = session, int(channel['id']), feed, log, clock
    self.options = channel.get('navigation') or {}
    self.image_size = None
    if self.options.get('type') == TYPE_IMAGE and self.options.get('image_width') and self.options.get('image_height'):
      self.image_size = (int(self.options['image_width']), int(self.options['image_height']))
    self.interval = max(MIN_INTERVAL, (self.options.get('min_interval_ms') or 0) / 1000)
    self.state = 'closed'  # closed, opening, open, unavailable
    self.deadline = 0.0
    self.started = False
    self.heard = False  # the car has said start or stop at least once
    self.next_check = 0.0
    self.next_distance = 0.0
    self.focus_requested = False
    self.focus = None
    self.legacy_since = None  # first legacy turn this session
    protocol = getattr(session, 'protocol', None)
    self.modern_supported = protocol is None or tuple(protocol) >= MODERN_MIN_PROTOCOL
    self._reset()

  def _reset(self) -> None:
    self.sent_status: int | None = None
    self.shown_turn: tuple | None = None
    self.shown_distance: tuple | None = None

  @classmethod
  def create(cls, session, log, *, feed=None, **kwargs):
    channel = navigation_channel(session.channels)
    if channel is None:
      return None
    return cls(session, channel, feed if feed is not None else NavFeed(), log, **kwargs)

  def _set(self, state: str) -> None:
    if state != self.state:
      self.log('nav_status_state', channel=self.channel_id, state=state, previous=self.state)
    self.state = state

  def service(self) -> None:
    now = self.clock()
    if self.state == 'closed':
      self.session.send(self.channel_id, MSG_CHANNEL_OPEN_REQUEST, field(1, 0) + field(2, self.channel_id), control=True)
      self.deadline = now + STEP_TIMEOUT
      self._set('opening')
    elif self.state == 'opening' and now > self.deadline:
      self._set('unavailable')
    elif self.state == 'open' and now >= self.next_check:
      self.next_check = now + CHECK_EVERY
      snap = self.feed.snapshot()
      active = snap is not None
      if active and not self.focus_requested:
        self.session.send(0, NAV_FOCUS_REQUEST, field(1, FOCUS_PROJECTED))
        self.focus_requested = True
        self.focus = None
        self.log('nav_focus_requested', focus=FOCUS_PROJECTED)
      if not self.started and not self.heard and now >= self.deadline:
        self.started = True
        self.log('nav_status_started', requested=False)
      if self.started and self.focus != FOCUS_NATIVE:
        self._update(snap, now)
      if not active and self.focus_requested:
        self.session.send(0, NAV_FOCUS_REQUEST, field(1, FOCUS_NATIVE))
        self.focus_requested = False
        self.log('nav_focus_requested', focus=FOCUS_NATIVE)

  def _legacy(self) -> bool:
    return SEND_LEGACY and not _legacy_blocked

  def _modern(self) -> bool:
    return self.modern_supported or not self._legacy()  # the newer pair is the fallback when legacy is off

  def _send(self, kind: int, body: bytes) -> None:
    if kind in (NAV_TURN, NAV_DISTANCE):
      if not self._legacy():
        return
      if self.legacy_since is None:
        self.legacy_since = self.clock()
    elif kind in (NAV_NOTIFICATION, NAV_CURRENT_POSITION) and not self._modern():
      return
    self.session.send(self.channel_id, kind, body)

  def _update(self, snap: dict | None, now: float) -> None:
    if snap is None:
      if self.sent_status != STATUS_INACTIVE:
        self._send(NAV_STATUS, status_message(STATUS_INACTIVE))
        self.sent_status, self.shown_turn, self.shown_distance = STATUS_INACTIVE, None, None
      return
    if snap['state'] != 'guiding':
      if self.sent_status != STATUS_REROUTING:
        self._send(NAV_STATUS, status_message(STATUS_REROUTING))
        self.sent_status = STATUS_REROUTING
      return
    if self.sent_status != STATUS_ACTIVE:
      self.shown_turn, self.shown_distance = None, None
      self._send(NAV_STATUS, status_message(STATUS_ACTIVE))
      self.sent_status = STATUS_ACTIVE
    degrees, exit_number, driving_left = snap.get('degrees', 0), snap.get('exit', 0), snap.get('driving_left', False)
    turn = (snap['road'], snap['type'], snap['modifier'], degrees, exit_number, driving_left)
    if turn != self.shown_turn:
      legacy_event, side = maneuver_event(snap['type'], snap['modifier'], driving_left)
      angle = roundabout_exit(snap['type'], snap['modifier'], degrees, driving_left)
      maneuver = modern_maneuver(snap['type'], snap['modifier'])
      image = turn_image(snap['type'], snap['modifier'], *self.image_size) if self.image_size else None
      # Older receivers use the flat messages; newer ones consume 0x8006/0x8007.
      self._send(NAV_TURN, turn_message(snap['road'], legacy_event, side, image, angle, exit_number))
      self._send(NAV_NOTIFICATION, notification_message(snap['road'], maneuver))
      legacy = self._legacy()
      # Not event=: that is the log record's own name, and it hid every one of these lines on the drive.
      self.log('nav_guidance_sent', maneuver=maneuver, legacy_event=legacy_event, side=side, angle=angle, legacy=legacy,
               modern=self._modern(), image=legacy and image is not None, road_chars=len(snap['road']))
      self.shown_turn, self.shown_distance = turn, None
    metric = bool(snap.get('metric'))
    # Send whenever either the cluster's number or our own rounded value would change, at most once per interval.
    shown = (display_distance(snap['distance'], metric), cluster_distance(snap['distance']))
    if shown != self.shown_distance and (self.shown_distance is None or now >= self.next_distance):
      pace = snap['remaining'] / snap['duration'] if snap.get('duration', 0) > 0 and snap.get('remaining', 0) > 0 else FALLBACK_SPEED
      seconds = snap['distance'] / max(pace, 1.0)
      self._send(NAV_DISTANCE, distance_message(snap['distance'], seconds, metric))
      self._send(NAV_CURRENT_POSITION, current_position_message(snap['distance'], seconds, metric))
      self.shown_distance, self.next_distance = shown, now + self.interval

  def handle(self, channel: int, kind: int, fields: dict) -> bool:
    if channel == 0 and kind == NAV_FOCUS_NOTIFICATION:
      self.focus = one(fields, 1)
      self.log('nav_focus', focus=self.focus)
      if self.focus == FOCUS_PROJECTED:
        self._reset()
        self.next_check = 0.0
      return True
    if channel != self.channel_id:
      return False
    if kind == MSG_CHANNEL_OPEN_RESPONSE and self.state == 'opening':
      if signed(one(fields, 1)) != 0:
        self.log('nav_status_rejected', channel=self.channel_id, status=signed(one(fields, 1)))
        self._set('unavailable')
      else:
        self._set('open')
        self.deadline = self.clock() + START_GRACE
    elif kind == CLUSTER_START:
      self.started, self.heard = True, True
      self._reset()
      self.next_check = 0.0
      self.log('nav_status_started', requested=True)
    elif kind == CLUSTER_STOP:
      self.started, self.heard = False, True
      self._reset()
    return True

  def close(self, error: BaseException | None = None) -> None:
    global _legacy_blocked
    if error is not None and self.legacy_since is not None and self.clock() - self.legacy_since < LEGACY_PROBATION:
      _legacy_blocked = True
      self.log('nav_legacy_blocked', after_s=round(self.clock() - self.legacy_since, 1), error=f'{type(error).__name__}: {error}')
    if hasattr(self.feed, 'close'):
      self.feed.close()
