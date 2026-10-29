"""Location from the car's head unit, over the Android Auto car-data channel.

The comma stands in for the phone, so the head unit offers it the car's own GPS the way it would a phone (sensor
type 1). It is the car's receiver, so it ranks and shows as car GPS. navigationd publishes it inside
starpilotNavigation; nothing in the control loop reads it. Coordinates go only into the handoff file, never a log.
"""
import json
import math
import threading
import time

from openpilot.starpilot.system.android_auto.wire import one, signed

# The Android Auto LocationData accuracy field is optional. A built-in car receiver is good to a few metres in the
# open; without a figure, claim one just inside the 25 m usable limit so any real figure above it is still refused.
NOMINAL_ACCURACY_M = 20.0
# Car receivers keep the last heading at a stop; trust a heading only while moving.
MIN_BEARING_SPEED = 1.0
NOMINAL_BEARING_ACCURACY_DEG = 15.0


def _optional(value, scale, low, high):
  if value is None:
    return math.nan
  value = signed(value) / scale
  return value if low <= value < high else math.nan


def head_unit_gps(fields, now_ns):
  """One Android Auto LocationData message (parsed fields) as a Gps dict.

  Fields: 1 timestamp, 2 latitude e7, 3 longitude e7, 4 accuracy e3 (m), 5 altitude e2 (m), 6 speed e3 (m/s),
  7 bearing e6 (degrees). The timestamp's unit differs between head units, so it only tells repeats apart.
  """
  latitude, longitude = one(fields, 2), one(fields, 3)
  if not isinstance(latitude, int) or not isinstance(longitude, int):
    raise ValueError("Missing position")
  latitude, longitude = signed(latitude) / 1e7, signed(longitude) / 1e7
  if not (-90 <= latitude <= 90 and -180 <= longitude <= 180) or (latitude == 0 and longitude == 0):
    raise ValueError("Invalid position")
  accuracy = one(fields, 4)
  accuracy = accuracy / 1e3 if isinstance(accuracy, int) and accuracy > 0 else NOMINAL_ACCURACY_M
  if not accuracy <= 10_000:
    raise ValueError("Invalid accuracy")
  speed = _optional(one(fields, 6), 1e3, 0, 100)
  bearing = _optional(one(fields, 7), 1e6, 0, 360) if speed >= MIN_BEARING_SPEED else math.nan
  timestamp = one(fields, 1, 0)
  return {
    "sourceMonoTime": now_ns,
    "latitude": latitude,
    "longitude": longitude,
    "altitude": _optional(one(fields, 5), 1e2, -1000, 10_000),
    "speed": speed,
    "bearingDeg": bearing,
    "horizontalAccuracy": accuracy,
    "verticalAccuracy": math.nan,
    "bearingAccuracyDeg": NOMINAL_BEARING_ACCURACY_DEG if math.isfinite(bearing) else math.nan,
    "speedAccuracy": math.nan,
    "unixTimestampMillis": 0,
    "hasFix": True,
  }, timestamp if isinstance(timestamp, int) else 0


def handoff_path():
  import tempfile
  from pathlib import Path
  shm = Path("/dev/shm")
  return (shm if shm.is_dir() else Path(tempfile.gettempdir())) / "starpilot_head_unit_gps.json"


class HeadUnitGps:
  """Hands the newest head-unit fix to navigationd. Counts stay coordinate-free for the session stats."""

  def __init__(self, path=None, mono_clock=time.monotonic_ns):
    self._path, self._mono_clock = path, mono_clock
    self._lock = threading.Lock()
    self._last = None
    self.received = self.accepted = self.repeated = self.rejected = 0
    self.last_error = ""

  def publish(self, fields):
    """True when the fix was handed off; repeats and unusable fixes are counted, not raised."""
    with self._lock:
      self.received += 1
      try:
        gps, timestamp = head_unit_gps(fields, self._mono_clock())
      except (ValueError, TypeError, OverflowError) as error:
        self.rejected += 1
        self.last_error = str(error)
        return False
      # A head unit that has lost its own fix may keep sending the last one. A repeated timestamp, or (without
      # one) an identical fix while moving, is not new, so it must not keep the stamp fresh.
      key = (gps["latitude"], gps["longitude"], gps["speed"], gps["bearingDeg"])
      if self._last is not None and ((timestamp and timestamp == self._last[0]) or
                                     (not timestamp and key == self._last[1] and gps["speed"] >= MIN_BEARING_SPEED)):
        self.repeated += 1
        return False
      self._last = (timestamp, key)
      path = self._path or handoff_path()
      temporary = path.with_name(path.name + ".tmp")
      temporary.write_text(json.dumps(gps))
      temporary.replace(path)
      self.accepted += 1
      return True

  def stats(self):
    return {"received": self.received, "accepted": self.accepted, "repeated": self.repeated, "rejected": self.rejected,
            "last_error": self.last_error}


def read_head_unit_gps(now_ns, path=None):
  """The handed-off fix as a Gps-like object, or None when absent, malformed, inaccurate or stale."""
  from types import SimpleNamespace
  from openpilot.starpilot.gps.source import HEAD_UNIT_MAX_AGE_NS, valid_fix
  try:
    with (path or handoff_path()).open("rb") as source:
      gps = json.loads(source.read(1 << 16))
    stamp = gps["sourceMonoTime"]
    if type(stamp) is not int:
      return None
    return valid_fix(SimpleNamespace(**gps), stamp, stamp, now_ns, HEAD_UNIT_MAX_AGE_NS)
  except (OSError, ValueError, KeyError, TypeError, AttributeError, OverflowError):
    return None
