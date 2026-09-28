import math
import os
import sys
import time

import numpy as np

from collections import namedtuple

# Panda3D's Cocoa graphics pipe pumps AppKit events while creating its window.
# If this command-line Python process has a previous crash record, AppKit may
# show its "restore windows" modal alert and block that pump indefinitely.
# Register this only in the simulator process; unlike `defaults write`, this
# does not change the user's persistent macOS preferences.
if sys.platform == "darwin":
  from Foundation import NSArgumentDomain, NSUserDefaults
  NSUserDefaults.standardUserDefaults().setVolatileDomain_forName_({"ApplePersistence": False}, NSArgumentDomain)

from panda3d.core import Vec3
from multiprocessing.connection import Connection

from metadrive.engine.core.engine_core import EngineCore
from metadrive.engine.core.image_buffer import ImageBuffer
from metadrive.envs.metadrive_env import MetaDriveEnv
from metadrive.obs.image_obs import ImageObservation

from openpilot.common.realtime import Ratekeeper

from openpilot.tools.sim.lib.common import vec3
from openpilot.tools.sim.lib.camerad import W, H

# MetaDrive's terrain.frag.glsl samples the shadow atlas with texture2D, which the macOS
# GL 4.1 core profile rejects. The shader then fails silently and the ground draws flat
# grey with no road surface or lane lines, so the driving model sees no road and stops.
if sys.platform == "darwin":
  from panda3d.core import NodePath, Shader
  from metadrive.engine.asset_loader import AssetLoader
  from metadrive.engine.core.terrain import Terrain

  def _core_profile_render_state(engine, vert, frag):
    def read(name):
      with open(AssetLoader.file_path("../shaders", name)) as f:
        src = f.read().replace("texture2D(", "texture(")
      if name == "terrain.frag.glsl":  # 50 % coverage thresholds for the anti-aliased lane lines (see get_semantic_map below)
        for a, b in (("attri.r > 0.01", "attri.r > 0.09"), ("value < 0.11", "value < 0.15"), ("value < 0.21", "value < 0.25")):
          assert a in src, a
          src = src.replace(a, b)
      return src
    dummy_np = NodePath("Dummy")
    dummy_np.setShader(Shader.make(Shader.SL_GLSL, vertex=read(vert), fragment=read(frag)))
    return dummy_np.getState()

  Terrain.make_render_state = staticmethod(_core_profile_render_state)

  # The skybox has the same problem: on macOS MetaDrive picks #version 120 shaders, which the core profile
  # rejects, so the sky draws flat grey with the skybox texture on a stray panel. Its #version 150 shaders work.
  import metadrive.engine.core.sky_box as sky_box
  sky_box.is_mac = lambda: False

# sim-lat-training: MetaDrive paints lane lines into the terrain's semantic texture (22 px/m, one float category per
# texel: 0 ground, 0.1 yellow, 0.2 road, 0.3 white) with cv2.polylines at 1 px (4.5 cm; yellow 2 px) from integer-truncated
# points, and terrain.frag thresholds the bilinearly filtered value (yellow < 0.11 < road < 0.21 < white). Every line edge
# snaps to the texel grid: straights along an axis look fine, but on curves the dashes render as wobbly blobs and the
# centre line as a zigzag, and TSFDO loses the lanes (gentle-map lane probs ~0.2-0.3 vs ~0.55 on the straight map).
# Draw the lines anti-aliased at a real 0.15 m width from sub-pixel points, encoding coverage c as road + c * (line - road),
# and move the shader's yellow/road/white thresholds to the 50 % coverage points (0.15, 0.25) so the edge is sub-texel.
# Same dash pattern and colours; lines are only blended over road/ground texels, never crosswalks.
import cv2
from metadrive.component.map.base_map import BaseMap
from metadrive.constants import MapTerrainSemanticColor, PGDrivableAreaProperty
from metadrive.type import MetaDriveType

_LANE_LINE_WIDTH_M = 0.15
_ROAD = MapTerrainSemanticColor.get_color(MetaDriveType.LANE_SURFACE_STREET)
_get_semantic_map_orig = BaseMap.get_semantic_map

def _get_semantic_map_smooth_lines(self, center_point, size=512, pixels_per_meter=8, color_setting=MapTerrainSemanticColor,
                                   line_sample_interval=2, polyline_thickness=1, layer=("lane_line", "lane")):
  mask = _get_semantic_map_orig(self, center_point, size, pixels_per_meter, color_setting, line_sample_interval,
                                polyline_thickness, tuple(l for l in layer if l != "lane_line"))
  if "lane_line" not in layer:
    return mask
  px = size * pixels_per_meter
  shift = 4
  thickness = max(1, round(_LANE_LINE_WIDTH_M * pixels_per_meter))
  skip = math.floor(PGDrivableAreaProperty.STRIPE_LENGTH * 2 / line_sample_interval)
  for obj in self.get_map_features(interval=line_sample_interval).values():
    if not (MetaDriveType.is_road_line(obj["type"]) or MetaDriveType.is_road_boundary_line(obj["type"])):
      continue
    line = np.asarray(obj["polyline"])[:, :2]
    pts = np.round(((line - np.asarray(center_point[:2])) * pixels_per_meter + px / 2) * (1 << shift)).astype(np.int32)
    if MetaDriveType.is_broken_line(obj["type"]):
      segs = [pts[i:i + skip + 1] for i in range(0, len(pts) - 1, skip * 2) if i + skip < len(pts)]
    else:
      segs = [pts]
    # rasterise this line's coverage in its own bounding box only (the full texture is ~11k x 11k)
    x0, y0 = np.maximum((pts.min(0) >> shift) - thickness - 2, 0)
    x1, y1 = np.minimum((pts.max(0) >> shift) + thickness + 3, px)
    if x1 <= x0 or y1 <= y0:
      continue
    cov = np.zeros((y1 - y0, x1 - x0), np.uint8)
    off = np.array([x0, y0], np.int32) << shift
    cv2.polylines(cov, [s - off for s in segs], False, 255, thickness, cv2.LINE_AA, shift)
    c = cov.astype(np.float32) / 255.0
    sub = mask[y0:y1, x0:x1, 0]
    color = MapTerrainSemanticColor.get_color(obj["type"])
    blended = _ROAD + c * (color - _ROAD)
    ground = sub < 0.001  # boundary lines straddle the road edge
    lane = (sub > 0.09) & (sub < 0.31)  # road or another line (keep the stronger one where two overlap); never crosswalk
    on = (c > 0) & (ground | (lane & (np.abs(blended - _ROAD) >= np.abs(sub - _ROAD))))
    sub[on] = blended[on]
  return mask

if sys.platform == "darwin":  # the matching shader thresholds are patched in _core_profile_render_state above
  BaseMap.get_semantic_map = _get_semantic_map_smooth_lines

# sim-lat-training: MetaDrive paints road and lane-line texture only inside a map_region_size square centred on the origin
# (1024 m by default) and clips sidewalks and lane-line bodies to it. The gentle preset runs ~1.5 km along x, so every
# gentle episode reached painted grass at x ~540 m and left the road there (ts/lx/lz1/lz2 departures at x 539-590) while
# TSFDO's lane probs were still 0.9. Centre the painted region on the built map instead (Terrain.reset runs after the map
# is built; it positions the terrain mesh at the same centre), drop the origin clips (sidewalks are 3D meshes and draw
# fine anywhere). map_region_size 2048 (11 px/m) drew the whole ground white on this Mac, so maps must fit in 1024 m.
from metadrive.constants import TerrainProperty
from metadrive.engine.core.terrain import Terrain as _Terrain

_terrain_reset_orig = _Terrain.reset

def _terrain_reset_map_centred(self, center_point):
  if self.engine.current_map is not None:
    center_point = list(self.engine.current_map.get_center_point())
  return _terrain_reset_orig(self, center_point)

_Terrain.reset = _terrain_reset_map_centred
TerrainProperty.point_in_map = classmethod(lambda cls, point: True)
TerrainProperty.clip_polygon = classmethod(lambda cls, polygon: [list(polygon)])

# Camera mount: height above the road (m) and the driver's liveCalibration rpyCalib (rad). rpyCalib is device_from_calib
# (common/transformations/camera.py get_view_frame_from_calib_frame): rot_from_euler(rpy) applied to road-forward gives it
# in device coordinates (x forward, y right, z down). Checked numerically 2026-09-27 against Panda3D LRotationf.setHpr
# (camera +y forward, +x right, +z up; H counter-clockwise, P nose up, R clockwise): a positive rpy pitch puts road-forward
# ABOVE the device nose (device pitched down) and a positive yaw puts it LEFT of the nose (device yawed right), so
# H = +yaw, P = -pitch, R = +roll. Until 2026-09-27 the sign of H and P was inverted: with the owner's rpyCalib
# (pitch 0.02045, yaw -0.01075) the render looked 1.17 deg up and 0.62 deg left while calibrationd was seeded with the
# opposite, a 2.3 deg pitch / 1.2 deg yaw mismatch between the image and the calibration the model was given.
CAM_HEIGHT = float(os.getenv("SIM_CAM_HEIGHT", "1.22"))
CAM_RPY = [float(x) for x in os.getenv("SIM_CAM_RPY", "0,0,0").split(",")]
C3_POSITION = Vec3(0.0, 0, CAM_HEIGHT)
C3_HPR = Vec3(math.degrees(CAM_RPY[2]), -math.degrees(CAM_RPY[1]), math.degrees(CAM_RPY[0]))


metadrive_simulation_state = namedtuple("metadrive_simulation_state", ["running", "done", "done_info"])
metadrive_vehicle_state = namedtuple("metadrive_vehicle_state", ["velocity", "position", "bearing", "steering_angle", "yaw_rate", "accel",
                                                                    "blinker"], defaults=[0])

# SIM_BLINKER=auto (sim-lat-training): hold the turn signal toward the next corner from SIM_BLINKER_LEAD_M metres before it
# until the car leaves it, like a driver signalling an intersection turn (controlsd's turn hold / turn-lead shaping only
# acts with a blinker on). +1 left, -1 right, 0 off. Unset: always 0, so earlier episodes stay comparable.
BLINKER_AUTO = os.getenv("SIM_BLINKER") == "auto"
BLINKER_LEAD_M = float(os.getenv("SIM_BLINKER_LEAD_M", "40"))
# SIM_STOP_BEFORE_TURN=S (seconds): brake the car to a standstill SIM_STOP_M metres before each corner and hold it there S
# seconds, then let openpilot pull away into the turn. The brake is applied here, not as a driver pedal, so openpilot stays
# engaged and steering through the stop (the standstill turn-hold case). 0/unset: never.
STOP_BEFORE_TURN_S = float(os.getenv("SIM_STOP_BEFORE_TURN", "0"))
STOP_M = float(os.getenv("SIM_STOP_M", "12"))
STOP_DECEL = 2.0  # m/s^2

def _lane_turn(lane) -> int:
  # MetaDrive headings are counter-clockwise (a positive steer, openpilot's left, raises the heading), so a heading gain
  # along the lane is a left turn. Before 2026-09-27 this sign was inverted and SIM_BLINKER=auto signalled the wrong way.
  d = (lane.heading_theta_at(lane.length) - lane.heading_theta_at(0.0) + math.pi) % (2 * math.pi) - math.pi
  return 0 if abs(d) < math.radians(20) else (1 if d > 0 else -1)

def dist_to_turn(vehicle) -> float:
  # metres left on a straight lane before the next corner starts; inf when not approaching one
  nav = vehicle.navigation
  cur = nav.current_ref_lanes[0] if nav.current_ref_lanes else None
  nxt = nav.next_ref_lanes[0] if nav.next_ref_lanes else None
  if cur is None or nxt is None or _lane_turn(cur) or not _lane_turn(nxt):
    return math.inf
  return cur.length - cur.local_coordinates(vehicle.position)[0]

def auto_blinker(vehicle) -> int:
  nav = vehicle.navigation
  cur = nav.current_ref_lanes[0] if nav.current_ref_lanes else None
  if cur is None:
    return 0
  turn = _lane_turn(cur)
  if turn:
    return turn
  nxt = nav.next_ref_lanes[0] if nav.next_ref_lanes else None
  if nxt is not None and cur.length - cur.local_coordinates(vehicle.position)[0] < BLINKER_LEAD_M:
    return _lane_turn(nxt)
  return 0

# SIM_PLANT=civic: the car's own dynamic bicycle model (opendbc VehicleModel from the car's CarParams: mass, inertia,
# wheelbase, centre of gravity, tyre stiffness, steer ratio) moves the car, in real time at 100 Hz, from the steering-wheel
# angle and accel request the bridge sends. MetaDrive only places the car and renders; its Bullet chassis (1.1 t, 2.47 m
# wheelbase, a 5 deg steering dead band, ~60 % of kinematic curvature, 20 ms of physics per 50 ms of wall time) no
# longer decides how the car moves. Longitudinal: a first-order lag on the accel request (not a Civic powertrain model).
CIVIC_PLANT = os.getenv("SIM_PLANT", "metadrive") == "civic"
PLANT_ACCEL_TAU = 0.3  # s
PLANT_DYN_MIN_SPEED = 3.0  # m/s; below it the lateral state sits at its steady state (the dynamics settle in < 20 ms)
# The car as it really is rather than as its CarParams say: paramsd's learned steer ratio, tyre-stiffness factor and
# angle offset from the driver's own route (liveParameters). The measured wheel angle is the true angle plus the offset.
# Unset = CarParams values, no offset.
PLANT_SR = float(os.getenv("SIM_PLANT_SR", "0")) or None
PLANT_STIFFNESS = float(os.getenv("SIM_PLANT_STIFFNESS", "1"))
PLANT_OFFSET_DEG = float(os.getenv("SIM_PLANT_OFFSET_DEG", "0"))
# SIM_PLANT_VGR=c020: the Civic's EPS publishes its wheel angle through the C020 variable ratio (opendbc steer_ratio.py).
# The bicycle model takes the linear angle (centre ratio), so the published angle goes through the inverse map first:
# the effective ratio falls from 14.85 at centre to ~14.2 at 90 deg and ~13.4 at 180 deg. Routes 286/287/289 measured
# the same ~13% drop to 300 deg (James, 2026-09-27, limited road evidence).
PLANT_VGR = os.getenv("SIM_PLANT_VGR", "")


class CivicPlant:
  def __init__(self, position, heading, speed):
    self.pos = np.array(position[:2], dtype=float)
    self.heading = float(heading)  # MetaDrive sense: counter-clockwise
    self.u, self.a, self.x = float(speed), 0.0, np.zeros(2)  # x = [lateral velocity (left), yaw rate (left)]
    self.vm = None

  def _model(self, p):
    from types import SimpleNamespace
    from opendbc.car.vehicle_model import VehicleModel
    if self.vm is None or self.vm_params != p:
      m, j, l, aF, cF, cR, sR = p
      cp = SimpleNamespace(mass=m, rotationalInertia=j, wheelbase=l, centerToFront=aF, steerRatioRear=0.0,
                           tireStiffnessFront=cF, tireStiffnessRear=cR, steerRatio=sR)
      self.vm, self.vm_params = VehicleModel(cp), p
      self.vm.update_params(PLANT_STIFFNESS, PLANT_SR or sR)
    return self.vm

  def update(self, wheel_deg, accel_cmd, params, dt):
    from opendbc.car.vehicle_model import create_dyn_state_matrices, dyn_ss_sol
    vm = self._model(params)
    self.a += (accel_cmd - self.a) * min(dt / PLANT_ACCEL_TAU, 1.0)
    if self.u <= 0.0 and self.a < 0.0:
      self.a = 0.0
    self.u = max(self.u + self.a * dt, 0.0)
    published = wheel_deg - PLANT_OFFSET_DEG
    if PLANT_VGR == "c020":
      from opendbc.car.honda.steer_ratio import get_honda_vgr_inverse, vgr_physical_to_linear
      from opendbc.car.honda.values import HondaFlags
      published = vgr_physical_to_linear(published, get_honda_vgr_inverse(HondaFlags.VGR_CIVIC_TBA_C020))
    sa = math.radians(published)
    if self.u > PLANT_DYN_MIN_SPEED:
      A, B = create_dyn_state_matrices(self.u, vm)
      self.x = self.x + (A @ self.x + B[:, 0] * sa) * dt
    else:
      us = max(self.u, 0.1)
      self.x = dyn_ss_sol(sa, us, 0.0, vm)[:, 0] * (self.u / us)
    self.heading += self.x[1] * dt
    self.pos += self.velocity() * dt

  def velocity(self):
    c, s = math.cos(self.heading), math.sin(self.heading)
    return np.array([self.u * c - self.x[0] * s, self.u * s + self.x[0] * c])

  def place(self, vehicle):
    vehicle.set_position(self.pos)
    vehicle.set_heading_theta(self.heading)
    vehicle.set_velocity([1.0, 0.0], 0.0)
    vehicle.set_angular_velocity(0.0)


def apply_metadrive_patches(arrive_dest_done=True, out_of_road_done=True):
  # By default, metadrive won't try to use cuda images unless it's used as a sensor for vehicles, so patch that in
  def add_image_sensor_patched(self, name: str, cls, args):
    if self.global_config["image_on_cuda"]:# and name == self.global_config["vehicle_config"]["image_source"]:
        sensor = cls(*args, self, cuda=True)
    else:
        sensor = cls(*args, self, cuda=False)
    assert isinstance(sensor, ImageBuffer), "This API is for adding image sensor"
    self.sensors[name] = sensor

  EngineCore.add_image_sensor = add_image_sensor_patched

  # we aren't going to use the built-in observation stack, so disable it to save time
  def observe_patched(self, *args, **kwargs):
    return self.state

  ImageObservation.observe = observe_patched

  # disable destination, we want to loop forever
  def arrive_destination_patch(self, *args, **kwargs):
    return False

  if not arrive_dest_done:
    MetaDriveEnv._is_arrive_destination = arrive_destination_patch

  # sim-lat-training: MetaDriveEnv.done_function ends the episode on out_of_road unconditionally (there is no config key
  # for it), which froze the world at the first departure. Report "never out of road" to done_function instead; road
  # departures are still detected and logged from the vehicle's lane state in the step loop below.
  def not_out_of_road_patch(self, *args, **kwargs):
    return False

  if not out_of_road_done:
    MetaDriveEnv._is_out_of_road = not_out_of_road_patch

def metadrive_process(dual_camera: bool, config: dict, camera_array, wide_camera_array, image_lock,
                      controls_recv: Connection, simulation_state_send: Connection, vehicle_state_send: Connection,
                      exit_event, op_engaged, test_duration, test_run):
  arrive_dest_done = config.pop("arrive_dest_done", True)
  out_of_road_done = config.pop("out_of_road_done", True)
  apply_metadrive_patches(arrive_dest_done, out_of_road_done)

  road_image = np.frombuffer(camera_array.get_obj(), dtype=np.uint8).reshape((H, W, 3))
  if dual_camera:
    assert wide_camera_array is not None
    wide_road_image = np.frombuffer(wide_camera_array.get_obj(), dtype=np.uint8).reshape((H, W, 3))

  env = MetaDriveEnv(config)

  def get_current_lane_info(vehicle):
    _, lane_info, on_lane = vehicle.navigation._get_current_lane(vehicle)
    lane_idx = lane_info[2] if lane_info is not None else None
    return lane_idx, on_lane

  def reset():
    env.reset()
    env.vehicle.config["max_speed_km_h"] = 1000
    lane_idx_prev, _ = get_current_lane_info(env.vehicle)

    simulation_state = metadrive_simulation_state(
      running=True,
      done=False,
      done_info=None,
    )
    simulation_state_send.send(simulation_state)

    return lane_idx_prev

  lane_idx_prev = reset()
  plant = CivicPlant(env.vehicle.position, env.vehicle.heading_theta, 0.0) if CIVIC_PLANT else None
  plant_cmd = None
  on_lane_prev = True
  map_end_seen = False
  start_time = None

  def get_cam_as_rgb(cam):
    cam = env.engine.sensors[cam]
    cam.get_cam().reparentTo(env.vehicle.origin)
    cam.get_cam().setPos(C3_POSITION)
    cam.get_cam().setHpr(C3_HPR)
    img = cam.perceive(to_float=False)
    if not isinstance(img, np.ndarray):
      img = img.get() # convert cupy array to numpy
    return img

  rk = Ratekeeper(100, None)

  steer_ratio = 8
  vc = [0,0]
  heading_prev, yaw_rate = env.vehicle.heading_theta, 0.0
  speed_prev, accel = 0.0, 0.0
  stop_hold, stopped_lane, stop_brake = None, None, 0.2

  record_dir = os.getenv("SIM_RECORD_DIR")
  gt_file = None
  if record_dir:
    import cv2
    os.makedirs(record_dir, exist_ok=True)
    # Ground-truth pose against the lane the car is on, per physics step (20 Hz): lateral offset from lane centre (m, + = right,
    # MetaDrive lane frame) and heading error (rad, car minus lane, + = left). Line error cannot be read from the model's lane lines. time.monotonic()
    # and speed are logged so the rows can be matched to the recorder by the speed trace.
    gt_file = open(os.path.join(record_dir, "lane_gt.csv"), "w")
    gt_file.write("t_mono,frame,speed,lane_idx,long_m,lat_m,heading_err,yaw_rate,lane_curv\n")  # yaw_rate rad/s (plant, + left), lane_curv 1/m (+ left)

  def cur_speed():
    return plant.u if plant is not None else float(np.linalg.norm(env.vehicle.velocity))

  while not exit_event.is_set():
    vel = plant.velocity() if plant is not None else env.vehicle.velocity
    if plant is not None:
      # 100 Hz, from the plant state just integrated. Until 2026-09-27 these were refreshed only on the 20 Hz physics step
      # below, so the gyro and the forward accel reached locationd 0-50 ms (25 ms mean) behind the steering angle: an
      # extra 25 ms in lagd's lateralDelay estimate and in every lag measured from carState vs the plant.
      yaw_rate, accel = float(plant.x[1]), plant.a
    vehicle_state = metadrive_vehicle_state(
      velocity=vec3(x=float(vel[0]), y=float(vel[1]), z=0),
      position=env.vehicle.position,
      bearing=float(math.degrees(env.vehicle.heading_theta)),
      steering_angle=env.vehicle.steering * env.vehicle.MAX_STEERING,
      yaw_rate=yaw_rate,
      accel=accel,
      blinker=auto_blinker(env.vehicle) if BLINKER_AUTO else 0,
    )
    vehicle_state_send.send(vehicle_state)

    if controls_recv.poll(0):
      while controls_recv.poll(0):
        steer_angle, gas, should_reset, plant_cmd = controls_recv.recv()

      steer_metadrive = steer_angle * 1 / (env.vehicle.MAX_STEERING * steer_ratio)
      steer_metadrive = np.clip(steer_metadrive, -1, 1)

      vc = [steer_metadrive, gas]

      if should_reset:
        lane_idx_prev = reset()
        start_time = None
        if plant is not None:
          plant = CivicPlant(env.vehicle.position, env.vehicle.heading_theta, 0.0)

    is_engaged = op_engaged.is_set()
    if is_engaged and start_time is None:
      start_time = time.monotonic()

    if plant is not None and plant_cmd is not None:
      accel_cmd = -STOP_DECEL if stop_hold is not None else plant_cmd[1]
      plant.update(plant_cmd[0], accel_cmd, plant_cmd[2], 1 / 100)
    if plant is not None and rk.frame % 1000 == 0:
      print(f"metadrive: plant frame {rk.frame} u {plant.u:.2f} a {plant.a:.2f} cmd {None if plant_cmd is None else plant_cmd[:2]}", flush=True)

    if rk.frame % 5 == 0:
      step_vc = vc
      if STOP_BEFORE_TURN_S > 0:
        nav_lane = id(env.vehicle.navigation.current_ref_lanes[0])
        if stop_hold is None and nav_lane != stopped_lane and dist_to_turn(env.vehicle) < STOP_M:
          stop_hold, stopped_lane = math.inf, nav_lane
          stop_brake = 0.2
          print(f"metadrive: stop_before_turn brake at frame {rk.frame}", flush=True)
        if stop_hold is not None:
          # Closed-loop brake to STOP_DECEL: a full MetaDrive brake passes selfdrived's excessive-actuation limit
          # (2 x ACCEL_MIN) and disengages openpilot, which the standstill case needs engaged.
          stop_brake = float(np.clip(stop_brake + 0.05 * (accel + STOP_DECEL), 0.05, 1.0))
          step_vc = [vc[0], -stop_brake]
          if stop_hold == math.inf and cur_speed() < 0.1:
            stop_hold = time.monotonic() + STOP_BEFORE_TURN_S
          elif time.monotonic() > stop_hold:
            stop_hold = None
            print(f"metadrive: stop_before_turn release at frame {rk.frame}", flush=True)
      if plant is not None:
        plant.place(env.vehicle)  # the plant decides the pose; Bullet holds the car still (full brake, zero velocity)
        step_vc = [0.0, -1.0]
      _, _, terminated, _, _ = env.step(step_vc)
      # Yaw rate over the 50 ms physics step, in MetaDrive's heading sense (metadrive_world maps it to the device gyro).
      # Without it the sim's gyro read zero, paramsd learned a runaway steering angle offset (36 deg in 17 s of
      # torque-mode driving) and the lateral controller believed a wound wheel was straight.
      heading = env.vehicle.heading_theta
      yaw_rate = (heading - heading_prev + math.pi) % (2 * math.pi) - math.pi
      yaw_rate /= 5 / 100
      heading_prev = heading
      speed = cur_speed()
      accel = (speed - speed_prev) / (5 / 100)
      speed_prev = speed
      if plant is not None:
        yaw_rate, accel = float(plant.x[1]), plant.a
      timeout = True if start_time is not None and time.monotonic() - start_time >= test_duration else False
      lane_idx_curr, on_lane = get_current_lane_info(env.vehicle)
      if gt_file is not None:
        try:
          lane = env.vehicle.lane
          lon, lat = lane.local_coordinates(env.vehicle.position)
          herr = (heading - lane.heading_theta_at(lon) + math.pi) % (2 * math.pi) - math.pi
          # lane curvature from the lane's own heading 1 m either side; car curvature is yaw_rate / speed
          lcurv = ((lane.heading_theta_at(lon + 0.5) - lane.heading_theta_at(lon - 0.5) + math.pi) % (2 * math.pi) - math.pi) / 1.0
          yr = float(plant.x[1]) if plant is not None else float("nan")
          gt_file.write(f"{time.monotonic():.4f},{rk.frame},{speed:.3f},{lane_idx_curr},{lon:.2f},{lat:.3f},{herr:.5f},{yr:.5f},{lcurv:.6f}\n")
          if rk.frame % 100 == 0:
            gt_file.flush()
        except Exception:
          pass
      out_of_lane = lane_idx_curr != lane_idx_prev or not on_lane
      lane_idx_prev = lane_idx_curr
      # sim-lat-training: log road departures and returns (out_of_road_done is off, so the world keeps stepping); the
      # episode recorder (tools/sim/sim_lat_record.py check_offroad) reads these lines from the bridge log
      # map_end: printed once when the car is within 40 m of the route's end; sim_lat_record.py stops recording there
      # (the drive maps' +12 s margin ran pieces off the end of the map, 2026-09-28)
      if not map_end_seen:
        nav = getattr(env.vehicle, "navigation", None)
        try:
          if nav is not None and nav.total_length > 0 and nav.total_length - nav.travelled_length < 40.0:
            print(f"metadrive: map_end at frame {rk.frame}", flush=True)
            map_end_seen = True
        except Exception:
          pass
      if on_lane != on_lane_prev:
        pos = tuple(round(float(x), 1) for x in env.vehicle.position)
        print(f"metadrive: {'back_on_road' if on_lane else 'out_of_road'} at frame {rk.frame} pos {pos}", flush=True)
        on_lane_prev = on_lane

      if terminated or ((out_of_lane or timeout) and test_run):
        if terminated:
          done_result = env.done_function("default_agent")
        elif out_of_lane:
          done_result = (True, {"out_of_lane" : True})
        elif timeout:
          done_result = (True, {"timeout" : True})

        simulation_state = metadrive_simulation_state(
          running=False,
          done=done_result[0],
          done_info=done_result[1],
        )
        simulation_state_send.send(simulation_state)

      if dual_camera:
        wide_road_image[...] = get_cam_as_rgb("rgb_wide")
      road_image[...] = get_cam_as_rgb("rgb_road")
      image_lock.release()

      # SIM_RECORD_DIR: save every 4th road frame (5 fps) with speed, for a video when the desktop cannot be
      # screen-captured. Recording starts at the first engagement, so the video skips the ~20 s of startup.
      if record_dir and start_time is not None and rk.frame % 20 == 0:
        speed = cur_speed()
        frame = road_image.copy()  # MetaDrive's buffer is BGR despite the name (camerad's kernel reads it as BGR)
        label = f"t={time.monotonic() - start_time:5.1f}s  v={speed:4.1f} m/s  {'ENGAGED' if is_engaged else 'disengaged'}"
        cv2.putText(frame, label, (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0) if is_engaged else (0, 0, 255), 3)
        cv2.imwrite(os.path.join(record_dir, f"{rk.frame:08d}.jpg"), frame)

    rk.keep_time()
