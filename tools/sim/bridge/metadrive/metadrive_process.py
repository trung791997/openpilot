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
        return f.read().replace("texture2D(", "texture(")
    dummy_np = NodePath("Dummy")
    dummy_np.setShader(Shader.make(Shader.SL_GLSL, vertex=read(vert), fragment=read(frag)))
    return dummy_np.getState()

  Terrain.make_render_state = staticmethod(_core_profile_render_state)

  # The skybox has the same problem: on macOS MetaDrive picks #version 120 shaders, which the core profile
  # rejects, so the sky draws flat grey with the skybox texture on a stray panel. Its #version 150 shaders work.
  import metadrive.engine.core.sky_box as sky_box
  sky_box.is_mac = lambda: False

C3_POSITION = Vec3(0.0, 0, 1.22)
C3_HPR = Vec3(0, 0,0)


metadrive_simulation_state = namedtuple("metadrive_simulation_state", ["running", "done", "done_info"])
metadrive_vehicle_state = namedtuple("metadrive_vehicle_state", ["velocity", "position", "bearing", "steering_angle", "yaw_rate", "accel"])

def apply_metadrive_patches(arrive_dest_done=True):
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

def metadrive_process(dual_camera: bool, config: dict, camera_array, wide_camera_array, image_lock,
                      controls_recv: Connection, simulation_state_send: Connection, vehicle_state_send: Connection,
                      exit_event, op_engaged, test_duration, test_run):
  arrive_dest_done = config.pop("arrive_dest_done", True)
  apply_metadrive_patches(arrive_dest_done)

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

  record_dir = os.getenv("SIM_RECORD_DIR")
  if record_dir:
    import cv2
    os.makedirs(record_dir, exist_ok=True)

  while not exit_event.is_set():
    vehicle_state = metadrive_vehicle_state(
      velocity=vec3(x=float(env.vehicle.velocity[0]), y=float(env.vehicle.velocity[1]), z=0),
      position=env.vehicle.position,
      bearing=float(math.degrees(env.vehicle.heading_theta)),
      steering_angle=env.vehicle.steering * env.vehicle.MAX_STEERING,
      yaw_rate=yaw_rate,
      accel=accel,
    )
    vehicle_state_send.send(vehicle_state)

    if controls_recv.poll(0):
      while controls_recv.poll(0):
        steer_angle, gas, should_reset = controls_recv.recv()

      steer_metadrive = steer_angle * 1 / (env.vehicle.MAX_STEERING * steer_ratio)
      steer_metadrive = np.clip(steer_metadrive, -1, 1)

      vc = [steer_metadrive, gas]

      if should_reset:
        lane_idx_prev = reset()
        start_time = None

    is_engaged = op_engaged.is_set()
    if is_engaged and start_time is None:
      start_time = time.monotonic()

    if rk.frame % 5 == 0:
      _, _, terminated, _, _ = env.step(vc)
      # Yaw rate over the 50 ms physics step, in MetaDrive's heading sense (metadrive_world maps it to the device gyro).
      # Without it the sim's gyro read zero, paramsd learned a runaway steering angle offset (36 deg in 17 s of
      # torque-mode driving) and the lateral controller believed a wound wheel was straight.
      heading = env.vehicle.heading_theta
      yaw_rate = (heading - heading_prev + math.pi) % (2 * math.pi) - math.pi
      yaw_rate /= 5 / 100
      heading_prev = heading
      speed = float(np.linalg.norm(env.vehicle.velocity))
      accel = (speed - speed_prev) / (5 / 100)
      speed_prev = speed
      timeout = True if start_time is not None and time.monotonic() - start_time >= test_duration else False
      lane_idx_curr, on_lane = get_current_lane_info(env.vehicle)
      out_of_lane = lane_idx_curr != lane_idx_prev or not on_lane
      lane_idx_prev = lane_idx_curr

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
        speed = float(np.linalg.norm(env.vehicle.velocity))
        frame = road_image.copy()  # MetaDrive's buffer is BGR despite the name (camerad's kernel reads it as BGR)
        label = f"t={time.monotonic() - start_time:5.1f}s  v={speed:4.1f} m/s  {'ENGAGED' if is_engaged else 'disengaged'}"
        cv2.putText(frame, label, (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0) if is_engaged else (0, 0, 255), 3)
        cv2.imwrite(os.path.join(record_dir, f"{rk.frame:08d}.jpg"), frame)

    rk.keep_time()
