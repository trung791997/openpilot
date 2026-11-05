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
