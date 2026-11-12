"""Owner toggles for the Bosch-A longitudinal features, read once when plannerd starts (either planner).

- BoschAStockBrakeFeel: the Stock Brake Feel law in honda_bosch_a_long (D-086 and its follow-ups).
- BoschAHumanFollowing: HumanFollowing (human_following.py) and HumanAcceleration (human_acceleration.py).
- BoschANeedFloorMinGap: NeedFloor only acts at a gap of at least 1.5 s of headway (honda_bosch_a_long); default off, replay only.
- BoschACameraCapHold: camera-cap mode in radard (radar_camera_cap.py, C-012): off = VA (default), on = D+E. Read once by radard, not plannerd.

Like UseStarPilotLongitudinalPlanner, the values are latched for the life of the plannerd process, so a change
applies on the next drive. Only an exact saved "1" turns a feature on; a missing, unreadable or malformed value is
Off. A toggle only arms its feature: each one still applies its own car gate (honda_bosch_a_long.active_for,
human_following.enabled_for), so a car that is not a verified Bosch-A Honda with openpilot longitudinal is unaffected.
"""
from dataclasses import dataclass

from openpilot.starpilot.saved_source import read_saved

STOCK_BRAKE_FEEL_KEY = 'BoschAStockBrakeFeel'
HUMAN_FOLLOWING_KEY = 'BoschAHumanFollowing'
CAMERA_CAP_HOLD_KEY = 'BoschACameraCapHold'
NEED_FLOOR_MIN_GAP_KEY = 'BoschANeedFloorMinGap'
KEYS = (STOCK_BRAKE_FEEL_KEY, HUMAN_FOLLOWING_KEY, CAMERA_CAP_HOLD_KEY, NEED_FLOOR_MIN_GAP_KEY)
# (key, label, reason) for the settings rows on the Long Planner page
ROWS = (
  (STOCK_BRAKE_FEEL_KEY, "Stock Brake Feel (Bosch-A)",
   ("Shapes how braking builds and lets off using the stock Honda ACC timing, so braking feels closer to stock. " +
    "It never brakes harder than the planner asks. Works with either planner. Changes apply on the next drive. " +
    "Acts only on verified Bosch-A Hondas with openpilot longitudinal.")),
  (HUMAN_FOLLOWING_KEY, "Human Following (Bosch-A)",
   ("Follows the driving model's predicted path for the car ahead instead of a straight-line guess, and eases " +
    "acceleration at low set speeds and near the set speed. Works with either planner. Changes apply on the next drive. " +
    "Acts only on verified Bosch-A Hondas with openpilot longitudinal.")),
  (CAMERA_CAP_HOLD_KEY, "Camera cap hold: VA (default) / D+E (1 s tapered hold)",
   ("Off is VA. On adds a 1 s tapered hold to the far-lead camera cap: it never brakes harder than VA, softens real " +
    "far slowdowns by up to about 0.4 m/s\u00b2 for up to about 2 s (replay evidence only), and cuts more residual " +
    "phantoms. Changes apply on the next drive. Acts only on verified Bosch-A Hondas with a radar.")),
  (NEED_FLOOR_MIN_GAP_KEY, "Need floor: only at 1.5 s gap or more (Bosch-A)",
   ("Off keeps today's need floor. On stops the braking floor from acting when the lead is closer than 1.5 s of headway, " +
    "so it does not soften a close lead (replay evidence only). Needs Stock Brake Feel. Changes apply on the next drive. " +
    "Acts only on verified Bosch-A Hondas with openpilot longitudinal.")),
)


@dataclass(frozen=True)
class BoschAToggles:
  stock_brake_feel: bool = False
  human_following: bool = False
  camera_cap_hold: bool = False  # read by radard (read_camera_cap_hold), carried here so one parser owns every Bosch-A toggle
  need_floor_min_gap: bool = False  # NeedFloor 1.5 s headway gate (honda_bosch_a_long)


OFF = BoschAToggles()


def _on(params, key: str) -> bool:
  try:
    raw, readable = read_saved(params, key, 8)
  except Exception:
    return False
  return bool(readable and raw == b'1')


def read_toggles(params) -> BoschAToggles:
  """The saved toggles; Off for anything but an exact b'1'."""
  return BoschAToggles(stock_brake_feel=_on(params, STOCK_BRAKE_FEEL_KEY), human_following=_on(params, HUMAN_FOLLOWING_KEY),
                        camera_cap_hold=_on(params, CAMERA_CAP_HOLD_KEY),
                        need_floor_min_gap=_on(params, NEED_FLOOR_MIN_GAP_KEY))


def read_camera_cap_hold(params) -> bool:
  """radard's latch: True only for an exact saved "1"; missing, unreadable or malformed is VA."""
  return _on(params, CAMERA_CAP_HOLD_KEY)


def vehicle_supported(CP) -> bool:
  """Settings-row gate: a Honda with a verified Bosch-A radar and openpilot longitudinal. The planners still apply their
  own gates; this only decides whether the rows can be turned on."""
  if CP is None:
    return False
  try:
    from opendbc.car.honda.bosch_a_radar import VERIFIED_BOSCH_A_CARS
    return bool(CP.brand == "honda" and CP.carFingerprint in VERIFIED_BOSCH_A_CARS and CP.openpilotLongitudinalControl)
  except Exception:
    return False
