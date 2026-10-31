from __future__ import annotations

from bisect import bisect_right
from dataclasses import dataclass
import math

from openpilot.starpilot.navigation.owner import ValidationError, response_json

EARTH_RADIUS = 6371007.2
MAX_POINTS = 20000
SPEED_UNITS = {'km/h': 1 / 3.6, 'mph': 0.44704}
MAX_SPEED_LIMIT_MPS = 160 / 3.6
# Mapbox congestion per geometry segment -> the level published in Navigation.routeCongestion
CONGESTION = {'unknown': 0, 'low': 1, 'moderate': 2, 'heavy': 3, 'severe': 4}
REFRESH_URL = 'https://api.mapbox.com/directions-refresh/v1/mapbox/driving-traffic/'


def point(value) -> tuple[float, float]:
  if (not isinstance(value, (list, tuple)) or len(value) < 2 or
      any(type(v) not in (int, float) or not math.isfinite(v) for v in value[:2]) or
      not -180 <= value[0] <= 180 or not -90 <= value[1] <= 90):
    raise ValidationError('The route contains invalid coordinates')
  return float(value[0]), float(value[1])


def distance(a, b) -> float:
  lat1, lat2 = math.radians(a[1]), math.radians(b[1])
  h = math.sin((lat2 - lat1) / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(math.radians(b[0] - a[0]) / 2) ** 2
  return 2 * EARTH_RADIUS * math.asin(math.sqrt(min(1., max(0., h))))


def project(a, b, p) -> tuple[float, float]:
  latitude_scale = EARTH_RADIUS * math.pi / 180
  longitude_scale = latitude_scale * math.cos(math.radians((a[1] + b[1] + p[1]) / 3))
  x, y = (b[0] - a[0]) * longitude_scale, (b[1] - a[1]) * latitude_scale
  px, py = (p[0] - a[0]) * longitude_scale, (p[1] - a[1]) * latitude_scale
  fraction = min(1., max(0., (px * x + py * y) / (x * x + y * y))) if x * x + y * y > 1e-6 else 0.
  return fraction, math.hypot(px - fraction * x, py - fraction * y)


def number(value, maximum=1e8) -> float:
  if type(value) not in (int, float) or not math.isfinite(value) or not 0 <= value <= maximum:
    raise ValidationError('The route contains invalid distances')
  return float(value)


def modifier(value) -> str:
  return str(value).replace('slight left', 'slightLeft').replace('slight right', 'slightRight').replace(
    'sharp left', 'sharpLeft').replace('sharp right', 'sharpRight')


def max_speeds(data: dict, segments: int) -> list[float | None]:
  """Posted limit per route segment from Mapbox's maxspeed annotation, in m/s.

  Mapbox gives one entry per geometry segment: {'speed', 'unit'}, {'unknown': true}
  or {'none': true} (no posted limit). Only a numeric posted limit counts; a
  missing, short or malformed annotation leaves every segment unknown.
  """
  try:
    rows = data['legs'][0]['annotation']['maxspeed']
  except (KeyError, TypeError, IndexError):
    return [None] * segments
  if not isinstance(rows, list) or len(rows) != segments:
    return [None] * segments
  limits = []
  for row in rows:
    speed = row.get('speed') if isinstance(row, dict) else None
    factor = SPEED_UNITS.get(row.get('unit')) if isinstance(row, dict) else None
    mps = speed * factor if type(speed) in (int, float) and factor is not None and math.isfinite(speed) else None
    limits.append(mps if mps is not None and 0 < mps <= MAX_SPEED_LIMIT_MPS else None)
  return limits


def congestion_rows(rows, segments: int) -> list[int] | None:
  """Levels from a Mapbox congestion list of `segments` entries; None when it is missing, short or not a list."""
  if not isinstance(rows, list) or len(rows) != segments:
    return None
  return [CONGESTION.get(row, 0) if isinstance(row, str) else 0 for row in rows]


def congestion_levels(data: dict, segments: int) -> list[int]:
  """Traffic per route segment from Mapbox's congestion annotation: 0 unknown, 1 low, 2 moderate, 3 heavy,
  4 severe. A missing or mismatched annotation leaves every segment unknown."""
  try:
    rows = data['legs'][0]['annotation']['congestion']
  except (KeyError, TypeError, IndexError):
    return [0] * segments
  levels = congestion_rows(rows, segments)
  return levels if levels is not None else [0] * segments


def roundabout(primary: dict, maneuver: dict, step: dict) -> tuple[int, int, bool]:
  """(exit angle, exit number, drives on the left) for a roundabout maneuver; zeros when Mapbox gives none."""
  degrees, exit_number = primary.get('degrees'), maneuver.get('exit')
  degrees = round(degrees) if type(degrees) in (int, float) and math.isfinite(degrees) and 0 < degrees <= 360 else 0
  exit_number = exit_number if type(exit_number) is int and 0 < exit_number < 256 else 0
  return degrees, exit_number, step.get('driving_side') == 'left'


@dataclass(frozen=True)
class Step:
  start: float
  end: float
  duration: float
  text: str
  maneuver_type: str
  modifier: str
  exit_degrees: int = 0  # roundabout exit angle, 180 = straight through; 0 = not given
  exit_number: int = 0
  driving_left: bool = False
  road: str = ''  # the road this step drives along (Mapbox step name, else its ref); '' = not given


@dataclass(frozen=True)
class Progress:
  instruction: dict
  next_maneuver: dict
  off_route: bool
  arrived: bool
  speed_limit_mps: float | None = None  # Mapbox posted limit on the matched segment
  segment: int = 0  # the matched geometry segment


class NavigationRoute:
  def __init__(self, data: dict):
    try:
      coordinates = data['geometry']['coordinates']
      if not 2 <= len(coordinates) <= MAX_POINTS:
        raise ValidationError('The route is too long')
      self.points = [point(row) for row in coordinates]
      self.cumulative = [0.]
      for a, b in zip(self.points, self.points[1:], strict=False):
        self.cumulative.append(self.cumulative[-1] + distance(a, b))
      steps = data['legs'][0]['steps']
      if not 1 <= len(steps) <= 2048:
        raise ValidationError('The route has too many instructions')
      self.steps = []
      offset = 0.
      for index, step in enumerate(steps):
        end = offset + number(step['distance'])
        upcoming = steps[min(index + 1, len(steps) - 1)]
        maneuver = upcoming['maneuver']
        banners = step.get('bannerInstructions', [])
        primary = banners[0].get('primary', {}) if banners else {}
        text = str(primary.get('text') or maneuver.get('instruction', ''))[:256]
        self.steps.append(Step(offset, end, number(step['duration']), text,
                               str(primary.get('type') or maneuver['type'])[:40],
                               modifier(primary.get('modifier') or maneuver.get('modifier', 'none'))[:40],
                               *roundabout(primary, maneuver, upcoming),
                               str(step.get('name') or step.get('ref') or '')[:80]))
        offset = end
      self.total_distance = number(data['distance'])
      self.total_duration = number(data['duration'])
      if self.total_distance < 1 or offset < 1:
        raise ValidationError('The route is empty')
      self.starts = [step.start for step in self.steps]
      self.speed_limits = max_speeds(data, len(self.points) - 1)
      # replaced whole (never edited in place) by MapboxRouteEngine.refresh on the route thread
      self.congestion = congestion_levels(data, len(self.points) - 1)
      self.uuid = None      # the Directions response's uuid, for refreshes; None without enable_refresh
      self.route_index = 0  # this route's place in that response
    except (KeyError, TypeError, IndexError, AttributeError):
      raise ValidationError('The map service returned an incomplete route') from None

  def preview_indices(self) -> list[int]:
    stride = max(1, math.ceil((len(self.points) - 1) / 511))
    indices = list(range(0, len(self.points), stride))
    if indices[-1] != len(self.points) - 1:
      indices.append(len(self.points) - 1)
    return indices

  def preview(self) -> list[dict]:
    return [{'longitude': self.points[i][0], 'latitude': self.points[i][1]} for i in self.preview_indices()]

  def preview_congestion(self) -> list[int]:
    """One level per preview() point: the worst on the segments from that point to the next (the last point
    repeats the one before it)."""
    levels, indices = self.congestion, self.preview_indices()
    result = [max(levels[a:b], default=0) for a, b in zip(indices, indices[1:], strict=False)]
    return result + result[-1:] if result else [0] * len(indices)

  def progress(self, position, speed: float | None, bearing: float | None = None) -> Progress:
    best = (float('inf'), 0., 0)
    for index, (a, b) in enumerate(zip(self.points, self.points[1:], strict=False)):
      fraction, separation = project(a, b, position)
      along = self.cumulative[index] + fraction * (self.cumulative[index + 1] - self.cumulative[index])
      if separation < best[0]:
        best = separation, along, index
    separation, along, segment = best
    # Directions step lengths and geometry lengths can differ slightly.
    along *= self.steps[-1].end / max(self.cumulative[-1], 1.)
    index = min(len(self.steps) - 1, max(0, bisect_right(self.starts, along + .001) - 1))
    step = self.steps[index]
    remaining = max(0., step.end - along)
    duration = step.duration * min(1., remaining / max(step.end - step.start, 1.))
    duration += sum(row.duration for row in self.steps[index + 1:])
    instruction = {'text': step.text, 'maneuverType': step.maneuver_type, 'maneuverModifier': step.modifier,
                       'distanceMeters': remaining, 'remainingDistanceMeters': max(0., self.steps[-1].end - along),
                       'remainingDurationSeconds': duration, 'exitDegrees': step.exit_degrees,
                       'exitNumber': step.exit_number, 'drivingLeft': step.driving_left, 'roadName': step.road}
    upcoming = self.steps[min(index + 1, len(self.steps) - 1)]
    next_maneuver = {'maneuverType': upcoming.maneuver_type, 'maneuverModifier': upcoming.modifier,
                         'distanceMeters': max(0., upcoming.end - along)}
    known_speed = speed is not None and math.isfinite(speed)
    threshold_speed = speed if known_speed else 0.
    speeds, distances = (0., 5., 10., 20., 40.), (40., 50., 60., 80., 100.)
    bucket = min(3, max(0, bisect_right(speeds, threshold_speed) - 1))
    fraction = min(1., max(0., (threshold_speed - speeds[bucket]) / (speeds[bucket + 1] - speeds[bucket])))
    threshold = distances[bucket] + fraction * (distances[bucket + 1] - distances[bucket])
    misaligned = False
    if bearing is not None and known_speed and speed >= 2.5:
      a, b = self.points[segment], self.points[segment + 1]
      lat1, lat2, delta = math.radians(a[1]), math.radians(b[1]), math.radians(b[0] - a[0])
      direction = math.degrees(math.atan2(math.sin(delta) * math.cos(lat2),
                              math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(delta)))
      misaligned = abs((direction - bearing + 180) % 360 - 180) > 75
    arrived = known_speed and speed < 2 and distance(position, self.points[-1]) < 40 and instruction['remainingDistanceMeters'] < 40
    off_route = separation > threshold or misaligned
    return Progress(instruction, next_maneuver, off_route, arrived,
                    None if off_route else self.speed_limits[segment], segment)


class MapboxRouteEngine:
  def __init__(self, session, budget=None):
    self.session = session
    self.budget = budget  # MonthlyBudget for directions; None in tests

  def fetch(self, token: str, position, destination: dict, bearing: float | None = None,
            exclusions: tuple[str, ...] = ()) -> NavigationRoute:
    if self.budget is not None:
      self.budget.spend(enforce=False)  # upstream routing: counted for Setup, never refused
    url = ('https://api.mapbox.com/directions/v5/mapbox/driving-traffic/' +
           f'{position[0]},{position[1]};{destination["longitude"]},{destination["latitude"]}')
    params = {'access_token': token, 'geometries': 'geojson', 'steps': 'true', 'overview': 'full', 'banner_instructions': 'true', 'alternatives': 'true',
              'annotations': 'maxspeed,congestion', 'enable_refresh': 'true'}
    if exclusions:
      params['exclude'] = ','.join(exclusions)
    if bearing is not None:
      params['bearings'] = f'{int(bearing % 360)},90;'
    data = response_json(self.session, url, params)
    if data.get('code') != 'Ok' or not data.get('routes'):
      raise ValidationError('No driving route was found')
    routes = [NavigationRoute(row) for row in data['routes'][:3]]
    response_uuid = data.get('uuid')
    # it goes into the refresh URL's path
    usable = isinstance(response_uuid, str) and 0 < len(response_uuid) <= 128 and response_uuid.isascii() and \
      all(c.isalnum() or c in '-_' for c in response_uuid)
    for index, route in enumerate(routes):
      route.uuid, route.route_index = (response_uuid if usable else None), index
    routes[0].alternatives = routes
    return routes[0]

  def refresh(self, token: str, route: NavigationRoute, geometry_index: int) -> bool:
    """Fresh traffic for the rest of the route from Directions Refresh. Only route.congestion changes; on any
    error it keeps the old levels and returns False. Geometry, steps and guidance are never touched."""
    segments = len(route.points) - 1
    if route.uuid is None or not 0 <= geometry_index < segments:
      return False
    if self.budget is not None:
      self.budget.spend(enforce=False)  # counted with directions, never refused
    url = f'{REFRESH_URL}{route.uuid}/{route.route_index}/0'
    try:
      data = response_json(self.session, url, {'access_token': token, 'current_route_geometry_index': geometry_index})
      if data.get('code') != 'Ok':
        return False
      rows = data['route']['legs'][0]['annotation']['congestion']
    except (ValidationError, KeyError, TypeError, IndexError, AttributeError):
      return False
    # The refreshed annotation starts at geometry_index; a full-length one is taken whole.
    tail = congestion_rows(rows, segments - geometry_index)
    if tail is not None:
      route.congestion = route.congestion[:geometry_index] + tail
      return True
    full = congestion_rows(rows, segments)
    if full is not None:
      route.congestion = full
      return True
    return False
