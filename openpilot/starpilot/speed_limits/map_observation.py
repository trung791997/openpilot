"""Current-road Maps projection; source ordering/freshness stays in MapTracker."""
import hashlib
import json
import math
from dataclasses import dataclass

from openpilot.starpilot.speed_limits import acceptance as acc
from openpilot.starpilot.speed_limits import map_source as ms
from openpilot.starpilot.speed_limits.map_source import copy_frame as copy_frame


@dataclass(frozen=True)
class Upcoming:
  observation: acc.Observation
  distance_m: float
  boundary_id: str
  way_id: int


class MapSource:
  """Plannerd's map observation owner; never grants longitudinal authority."""

  def __init__(self):
    self.tracker = ms.MapTracker()
    self.upcoming: Upcoming | None = None
    self.way_id = 0

  def reset(self):
    self.tracker.reset()
    self.upcoming = None
    self.way_id = 0

  def step(self, frame: ms.MapFrame | None, *, received: bool, now_boot_ns: int,
           transport_alive: bool, drive_session: str, car_fingerprint: str) -> acc.Observation:
    self.upcoming = None
    self.way_id = 0
    evidence = self.tracker.step(frame, received=received, now_boot_ns=now_boot_ns,
                                 transport_alive=transport_alive)
    if evidence.observation.kind is acc.ObservationKind.STALE:
      return evidence.observation
    accepted = self.tracker.diagnostic(now_boot_ns=now_boot_ns).frame
    if (evidence.kind != 'matched_limit_unqualified' or accepted is None or
        accepted.way_selection != 'current' or
        any(type(value) is not str or not 0 < len(value) <= 256 or not value.strip() or
            any(ord(c) < 32 for c in value) for value in (drive_session, car_fingerprint))):
      # Missing speed metadata, coverage/GPS loss and ambiguous/lookahead ways
      # are incomplete evidence, not a new limit or regulatory absence.
      return acc.Observation(acc.ObservationKind.UNKNOWN)
    identity = hashlib.sha256(json.dumps((drive_session, car_fingerprint, accepted.producer_session,
      accepted.generation, accepted.source, accepted.way_id), separators=(',', ':')).encode()).hexdigest()
    self.way_id = accepted.way_id
    boundary = accepted.preview_boundary_id
    if (type(boundary) is str and len(boundary) == 64 and all(c in '0123456789abcdef' for c in boundary) and
        type(accepted.preview_speed_mps) is float and math.isfinite(accepted.preview_speed_mps) and
        accepted.preview_speed_mps >= 1.0 and abs(accepted.preview_speed_mps - accepted.speed_mps) > .02 and
        type(accepted.preview_distance_m) is float and math.isfinite(accepted.preview_distance_m) and
        0 < accepted.preview_distance_m <= 500 and type(accepted.preview_way_id) is int and accepted.preview_way_id > 0):
      preview_identity = hashlib.sha256(f'{identity}:{boundary}'.encode()).hexdigest()
      observation = acc.Observation(acc.ObservationKind.VALID, acc.Candidate('map',
        acc.ObservationIdentity(acc.IdentityKind.GEOGRAPHIC, value=preview_identity), accepted.preview_speed_mps))
      self.upcoming = Upcoming(observation, accepted.preview_distance_m, boundary, accepted.preview_way_id)
    return acc.Observation(acc.ObservationKind.VALID, acc.Candidate('map',
      acc.ObservationIdentity(acc.IdentityKind.GEOGRAPHIC, value=identity), accepted.speed_mps))


SAME_LIMIT_MPS = 0.5  # map limits are whole mph/km/h; this only absorbs float rounding


class SignFirst:
  """Vision before Maps: a seen sign holds until the map's posted limit changes.

  Vision is VALID only for SIGN_HOLD_NS after the car passes a sign. Plain ORDERED
  selection would then hand the limit back to the map on the same road, undoing a
  sign that disagrees with stale map data. After a sign, the map stays UNKNOWN
  (acceptance keeps the sign's limit) until the map reports a different limit than
  it did around the sign, which marks a new posted-limit stretch. Before any sign
  this drive, the map fills in directly.
  """

  def __init__(self):
    self.reset()

  def reset(self):
    self.sign_seen = False
    self.baseline_mps: float | None = None  # map limit when the sign was last seen

  def step(self, vision: acc.Observation, map_observation: acc.Observation) -> acc.Observation:
    map_mps = map_observation.candidate.speed_mps if map_observation.kind is acc.ObservationKind.VALID else None
    if vision.kind is acc.ObservationKind.VALID:
      self.sign_seen = True
      self.baseline_mps = map_mps
      return map_observation
    if not self.sign_seen or map_mps is None:
      return map_observation
    if self.baseline_mps is None:
      self.baseline_mps = map_mps  # the map had no limit at the sign; its first one is the same road
    if abs(map_mps - self.baseline_mps) <= SAME_LIMIT_MPS:
      return acc.Observation(acc.ObservationKind.UNKNOWN)
    self.reset()
    return map_observation


TURN_DEG = 60.        # navigation/road_limit.py MAX_TURN: heading change that means another road
TURN_MIN_SPEED = 4.   # m/s, road_limit MIN_SPEED; slower, GPS bearing is noise


def heading_change(a: float, b: float) -> float:
  return abs((a - b + 180) % 360 - 180)


class TurnOverride:
  """After a turn onto another road, a lower Mapbox limit replaces the held sign.

  Vision republishes the last sign it read as VALID for as long as it drives, and
  ONLINE is only a fallback, so a sign from the previous road outlived the turn:
  route 00000099--c504cef288 turned off a 50 mph road into a 25 mph neighborhood,
  Mapbox said 25 at 23:09 and the suggestion stayed 50 until the camera read a 20
  sign at 23:28. On the same road Mapbox often disagrees with the signs (45 or 55
  under a 50 sign), so this never acts without a turn, and never raises the limit.

  Turn: heading more than TURN_DEG from the heading when the sign was read (or
  when the override last armed), measured only while moving. Mapbox's value must
  be first seen after the turn: road_limit drops its answer on the same kind of
  turn, so the new road's value arrives as a new identity or after a non-VALID
  gap; the old road's value still VALID across the turn never qualifies. Ends on a
  new sign (new vision identity: the producer bumps episode only when the read
  value changes, so a repeat of the same sign does not end it), when Mapbox is
  no longer lower, or when ONLINE is not VALID.
  """

  def __init__(self):
    self.reset()

  def reset(self):
    self.sign = None                  # identity of the held sign
    self.reference_deg: float | None = None
    self.turned = False
    self.online = None                # identity of the current ONLINE value; None while not VALID
    self.online_after_turn = False    # that value (or its absence) was first seen after the turn
    self.active = False

  def step(self, vision: acc.Observation, online: acc.Observation, heading_deg: float | None) -> bool:
    """True while selection should see Vision as UNKNOWN so ONLINE fills in."""
    vision_valid = vision.kind is acc.ObservationKind.VALID
    online_valid = online.kind is acc.ObservationKind.VALID
    if vision_valid and vision.candidate.observation_identity != self.sign:
      self.reset()
      self.sign = vision.candidate.observation_identity
    if self.sign is not None and heading_deg is not None:
      if self.reference_deg is None:
        self.reference_deg = heading_deg
      elif heading_change(heading_deg, self.reference_deg) > TURN_DEG:
        self.turned = True
    identity = online.candidate.observation_identity if online_valid else None
    if identity != self.online:  # after the turn check: a value arriving with the turn counts
      self.online = identity
      self.online_after_turn = self.turned
    if self.sign is None:
      return False
    lower = vision_valid and online_valid and online.candidate.speed_mps < vision.candidate.speed_mps - SAME_LIMIT_MPS
    if self.active:
      self.active = online_valid and (lower or not vision_valid)
    elif lower and self.turned and self.online_after_turn:
      # Re-arming takes another turn from here and a Mapbox value seen after it.
      self.active, self.turned, self.online_after_turn, self.reference_deg = True, False, False, heading_deg
    return self.active and vision_valid
