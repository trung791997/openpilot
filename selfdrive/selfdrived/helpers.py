import math
import time
from enum import StrEnum, auto

from cereal import car, log, messaging
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.locationd.helpers import Pose
from opendbc.car import ACCELERATION_DUE_TO_GRAVITY
from opendbc.car.lateral import ISO_LATERAL_ACCEL
from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX
from openpilot.starpilot.jetlink_adapter import AOL_GRACE_S, AOL_SERVICE, aol_in_control

MIN_EXCESSIVE_ACTUATION_COUNT = int(0.25 / DT_CTRL)
MIN_LATERAL_ENGAGE_BUFFER = int(1 / DT_CTRL)


class ExcessiveActuationType(StrEnum):
  LONGITUDINAL = auto()
  LATERAL = auto()


class ExcessiveActuationCheck:
  def __init__(self):
    self._excessive_counter = 0
    self._engaged_counter = 0

  def update(self, sm: messaging.SubMaster, CS: car.CarState, calibrated_pose: Pose,
             allow_impossible_acceleration: bool = False) -> ExcessiveActuationType | None:
    # CS.aEgo can be noisy to bumps in the road, transitioning from standstill, losing traction, etc.
    # longitudinal
    accel_calibrated = calibrated_pose.acceleration.x
    excessive_long_actuation = (
      not allow_impossible_acceleration and sm['carControl'].longActive and
      (accel_calibrated > ACCEL_MAX * 2 or accel_calibrated < ACCEL_MIN * 2)
    )

    # lateral
    yaw_rate = calibrated_pose.angular_velocity.yaw
    roll = sm['liveParameters'].roll
    roll_compensated_lateral_accel = (CS.vEgo * yaw_rate) - (math.sin(roll) * ACCELERATION_DUE_TO_GRAVITY)

    # Prevent false positives after overriding
    excessive_lat_actuation = False
    self._engaged_counter = self._engaged_counter + 1 if sm['carControl'].latActive and not CS.steeringPressed else 0
    if self._engaged_counter > MIN_LATERAL_ENGAGE_BUFFER:
      if abs(roll_compensated_lateral_accel) > ISO_LATERAL_ACCEL * 2:
        excessive_lat_actuation = True

    # livePose acceleration can be noisy due to bad mounting or aliased livePose measurements
    livepose_valid = abs(CS.aEgo - accel_calibrated) < 2
    self._excessive_counter = self._excessive_counter + 1 if livepose_valid and (excessive_long_actuation or excessive_lat_actuation) else 0

    excessive_type = None
    if self._excessive_counter > MIN_EXCESSIVE_ACTUATION_COUNT:
      if excessive_long_actuation:
        excessive_type = ExcessiveActuationType.LONGITUDINAL
      else:
        excessive_type = ExcessiveActuationType.LATERAL

    return excessive_type


# -- Jetlink's swap events (mirrors firestar5683/StarPilot#231) ------------------------------------------------------
# JetlinkBigActive (modeld's, on each change of Joined.big) stands in for the PR's modelV2.big, which this fork's
# cereal does not have; read a few times a second, as jetlink reads modeld's runtime record
JETLINK_BIG_POLL = max(round(0.2 / DT_CTRL), 1)


def jetlink_in_control(owner) -> bool:
  """The rule the swap itself waits on, over selfdrived's own state: openpilot enabled or active, or the adapter's
  AOL rule over starpilotCarState (AOL armed, paused included, or engaged)."""
  if owner.enabled or owner.active:
    return True
  sm = owner.sm
  heard = sm.seen[AOL_SERVICE]
  return aol_in_control(sm[AOL_SERVICE] if heard else None, heard and sm.alive[AOL_SERVICE],
                        waiting=sm.frame * DT_CTRL < AOL_GRACE_S)


def update_jetlink_swaps(owner, in_control, state=None):
  """One tick of Jetlink's swap events (a big model that joins mid-drive and can hand back), onto selfdrived's
  events. `state` is the joining model's; without it jetlink reads modeld's runtime record, and a gap in it holds
  the last state rather than reading as a switch."""
  model_ok = owner.sm.seen['modelV2'] and owner.sm.alive['modelV2'] and owner.sm.valid['modelV2']
  if owner.sm.frame % JETLINK_BIG_POLL == 0:
    try:
      owner.jetlink_big = bool(owner.params.get_bool("JetlinkBigActive"))
    except Exception:
      owner.jetlink_big = False
  swap = owner.jetlink_swaps.update(model_ok, model_ok and getattr(owner, 'jetlink_big', False), in_control, state=state)
  if swap.no_entry:
    owner.events.add(log.OnroadEvent.EventName.bigModelLoading)  # no engaging for the second after a swap
  if swap.settling and not owner.jetlink_settling:
    owner.big_model_ready_t = time.monotonic()  # a swap drops a frame: hold its comm and localizer errors
  owner.jetlink_settling, owner.jetlink_link_lost = swap.settling, swap.link_lost
  return swap


def jetlink_lost_alert():
  """The Jetlink big model left while in control: the small model drives on and nothing disengages."""
  from openpilot.selfdrive.selfdrived.events import ET, Alert, Priority
  alert = Alert("Big Model Lost", "Using small model", log.SelfdriveState.AlertStatus.userPrompt,
                log.SelfdriveState.AlertSize.mid, Priority.MID, car.CarControl.HUDControl.VisualAlert.steerRequired,
                log.SelfdriveState.AudibleAlert.warningSoft, .2)
  alert.alert_type, alert.event_type = 'jetlinkLinkLost/warning', ET.WARNING
  return alert
