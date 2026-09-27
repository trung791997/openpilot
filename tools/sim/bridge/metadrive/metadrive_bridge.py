import math
import os
from multiprocessing import Queue

from metadrive.component.sensors.base_camera import _cuda_enable
from metadrive.component.map.pg_map import MapGenerateMethod

from openpilot.tools.sim.bridge.common import SimulatorBridge
from openpilot.tools.sim.bridge.metadrive.metadrive_common import RGBCameraRoad, RGBCameraWide
from openpilot.tools.sim.bridge.metadrive.metadrive_world import MetaDriveWorld
from openpilot.tools.sim.lib.camerad import W, H


def straight_block(length):
  return {
    "id": "S",
    "pre_block_socket_index": 0,
    "length": length
  }

def curve_block(length, angle=45, direction=0, radius=None):
  return {
    "id": "C",
    "pre_block_socket_index": 0,
    "length": length,
    "radius": length if radius is None else radius,
    "angle": angle,
    "dir": direction
  }

# SIM_MAP scenario presets (sim-lat-training): straight length, curve radius, curve angle, alternate turn direction.
# "default" is upstream's loop (120 m radius 90 deg curves, |desired wheel| 17-36 deg at 12 m/s: ~3x the owner's drives).
# "intersection": 20 m radius 90 deg corners on short straights, the |desired| > 45 deg turns at 10-20 mph the replay sim
#   cannot close the loop on (STATUS 172/175 scenarios). "gentle": 400 m radius 30 deg curves on long straights, about the
#   owner's 25-50 mph drives (|desired| ~6 deg), for near-straight wobble with the model in the loop.
MAP_PRESETS = {
  "default": dict(straight=60, radius=120, angle=90, alternate=False),
  "intersection": dict(straight=80, radius=20, angle=90, alternate=True),
  "gentle": dict(straight=200, radius=400, angle=30, alternate=True),
}

def create_map(preset=None, track_size=None):
  preset = preset or os.getenv("SIM_MAP", "default")
  cfg = dict(MAP_PRESETS[preset])
  if track_size is not None:
    cfg["straight"] = track_size
  # SIM_MAP_RADIUS / SIM_MAP_STRAIGHT override the preset's numbers
  radius = float(os.getenv("SIM_MAP_RADIUS", cfg["radius"]))
  straight = float(os.getenv("SIM_MAP_STRAIGHT", cfg["straight"]))
  blocks = [None]
  for i in range(4):
    blocks.append(straight_block(straight))
    blocks.append(curve_block(radius, cfg["angle"], (i % 2) if cfg["alternate"] else 0, radius=radius))
  return dict(
    type=MapGenerateMethod.PG_MAP_FILE,
    lane_num=2,
    lane_width=4.5,
    config=blocks,
  )


class MetaDriveBridge(SimulatorBridge):
  TICKS_PER_FRAME = 5

  def __init__(self, dual_camera, high_quality, test_duration=math.inf, test_run=False):
    super().__init__(dual_camera, high_quality)

    self.should_render = False
    self.test_run = test_run
    self.test_duration = test_duration if self.test_run else math.inf

  def spawn_world(self, queue: Queue):
    sensors = {
      "rgb_road": (RGBCameraRoad, W, H, )
    }

    if self.dual_camera:
      sensors["rgb_wide"] = (RGBCameraWide, W, H)

    config = dict(
      use_render=self.should_render,
      vehicle_config=dict(
        enable_reverse=False,
        render_vehicle=False,
        image_source="rgb_road",
      ),
      sensors=sensors,
      image_on_cuda=_cuda_enable,
      image_observation=True,
      interface_panel=[],
      out_of_route_done=False,
      on_continuous_line_done=False,
      crash_vehicle_done=False,
      crash_object_done=False,
      arrive_dest_done=False,
      # sim-lat-training: MetaDrive ends the world the moment the car leaves the road, which read as an unexplained loss of
      # lateral in the recorded episodes; keep driving and let the recorder score the departure. Not a MetaDrive config key:
      # metadrive_process pops it and patches MetaDriveEnv._is_out_of_road (like arrive_dest_done)
      out_of_road_done=False,
      traffic_density=0.0, # traffic is incredibly expensive
      map_config=create_map(),
      decision_repeat=1,
      physics_world_step_size=self.TICKS_PER_FRAME/100,
      preload_models=False,
      show_logo=False,
      anisotropic_filtering=False
    )

    return MetaDriveWorld(queue, config, self.test_duration, self.test_run, self.dual_camera)
