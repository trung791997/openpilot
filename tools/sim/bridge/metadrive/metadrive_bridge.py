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
  # Large-radius S-curves (curvature 0.004 each way) on a closed loop: 4 x [80 m straight, 120 deg one way, 30 deg back]
  # at R 250 m (MetaDrive's curve block appends a straight of its length, 20 m here), 3.1 km of road inside a 945 m
  # square. MetaDrive paints road and lane lines only inside a 1024 m square (metadrive_process centres it on the map);
  # the earlier open S-road (straight 200, R 400, 30 deg alternating) ran 1.5 km along x and every gentle episode left
  # the road where the paint ended, at x ~540 m.
  "gentle": dict(straight=80, radius=250, tail=20, turns=((120, 0), (30, 1))),
}

def route_map_blocks(path):
  # SIM_MAP=route, SIM_MAP_FILE=JSON list of ["S", length_m] and ["C", radius_m, angle_deg, dir (0/1)] rebuilt from the
  # driver's own corners. A curve block appends a 1 m straight (or ["C", ..., tail_m]). Keep the layout inside MetaDrive's 1024 m painted square.
  import json
  with open(path) as f:
    spec = json.load(f)
  # Straights longer than 100 m are split: on 450 m blocks the lane lookup lost the car 100-200 m in (lane None,
  # out_of_road logged at 0.03-0.13 m from the centre line; drive maps 2026-09-28).
  blocks = []
  for b in spec:
    if b[0] == "S":
      n = max(1, math.ceil(b[1] / 100.0))
      blocks += [straight_block(b[1] / n) for _ in range(n)]
    else:
      # an optional 5th element is the straight after the arc (eased maps: 0.1 m between consecutive arcs)
      blocks.append(curve_block(b[4] if len(b) > 4 else 1, b[2], int(b[3]), radius=b[1]))
  return blocks


def create_map(preset=None, track_size=None):
  preset = preset or os.getenv("SIM_MAP", "default")
  # SIM_LANE_WIDTH: 4.5 m is upstream's; US lanes are ~3.6 m
  lane_width = float(os.getenv("SIM_LANE_WIDTH", "4.5"))
  if preset == "route":
    return dict(type=MapGenerateMethod.PG_MAP_FILE, lane_num=2, lane_width=lane_width,
                config=[None] + route_map_blocks(os.environ["SIM_MAP_FILE"]))
  cfg = dict(MAP_PRESETS[preset])
  if track_size is not None:
    cfg["straight"] = track_size
  # SIM_MAP_RADIUS / SIM_MAP_STRAIGHT override the preset's numbers
  radius = float(os.getenv("SIM_MAP_RADIUS", cfg["radius"]))
  straight = float(os.getenv("SIM_MAP_STRAIGHT", cfg["straight"]))
  blocks = [None]
  for i in range(4):
    blocks.append(straight_block(straight))
    if "turns" in cfg:
      blocks += [curve_block(cfg["tail"], angle, direction, radius=radius) for angle, direction in cfg["turns"]]
    else:
      blocks.append(curve_block(radius, cfg["angle"], (i % 2) if cfg["alternate"] else 0, radius=radius))
  return dict(
    type=MapGenerateMethod.PG_MAP_FILE,
    lane_num=2,
    lane_width=lane_width,
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
