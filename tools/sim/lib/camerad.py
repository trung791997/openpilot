import numpy as np
import os
import time
import pyopencl as cl
import pyopencl.array as cl_array

from msgq.visionipc import VisionIpcServer, VisionStreamType
from cereal import messaging

from openpilot.common.basedir import BASEDIR
from openpilot.system.camerad.cameras.nv12_info import get_nv12_info
from openpilot.tools.sim.lib.common import W, H

STRIDE, Y_HEIGHT, UV_HEIGHT, BUF_SIZE = get_nv12_info(W, H)
UV_OFFSET = STRIDE * Y_HEIGHT
PACKED_NV12_SIZE = W * H * 3 // 2


def pack_nv12(packed):
  assert packed.dtype == np.uint8
  assert packed.size == PACKED_NV12_SIZE, f"{packed.size} != {PACKED_NV12_SIZE}"
  packed = packed.reshape(-1)
  y = packed[:W * H].reshape(H, W)
  uv = packed[W * H:].reshape(H // 2, W)
  yuv = np.zeros(BUF_SIZE, dtype=np.uint8)
  yuv[:STRIDE * Y_HEIGHT].reshape(Y_HEIGHT, STRIDE)[:H, :W] = y
  yuv[UV_OFFSET:UV_OFFSET + UV_HEIGHT * STRIDE].reshape(UV_HEIGHT, STRIDE)[:H // 2, :W] = uv
  return yuv

class Camerad:
  """Simulates the camerad daemon"""
  def __init__(self, dual_camera):
    self.pm = messaging.PubMaster(['roadCameraState', 'wideRoadCameraState'])

    self.frame_road_id = 0
    self.frame_wide_id = 0
    self.vipc_server = VisionIpcServer("camerad")

    self.vipc_server.create_buffers_with_sizes(VisionStreamType.VISION_STREAM_ROAD, 5, W, H, BUF_SIZE, STRIDE, UV_OFFSET)
    if dual_camera:
      self.vipc_server.create_buffers_with_sizes(VisionStreamType.VISION_STREAM_WIDE_ROAD, 5, W, H, BUF_SIZE, STRIDE, UV_OFFSET)

    self.vipc_server.start_listener()

    # set up for pyopencl rgb to yuv conversion
    self.ctx = cl.create_some_context()
    self.queue = cl.CommandQueue(self.ctx)
    cl_arg = f" -DHEIGHT={H} -DWIDTH={W} -DRGB_STRIDE={W * 3} -DUV_WIDTH={W // 2} -DUV_HEIGHT={H // 2} -DRGB_SIZE={W * H} -DCL_DEBUG "

    kernel_fn = os.path.join(BASEDIR, "tools/sim/rgb_to_nv12.cl")
    with open(kernel_fn) as f:
      prg = cl.Program(self.ctx, f.read()).build(cl_arg)
      self.krnl = prg.rgb_to_nv12
    self.Wdiv4 = W // 4 if (W % 4 == 0) else (W + (4 - W % 4)) // 4
    self.Hdiv4 = H // 4 if (H % 4 == 0) else (H + (4 - H % 4)) // 4

  def cam_send_yuv_road(self, yuv):
    self._send_yuv(yuv, self.frame_road_id, 'roadCameraState', VisionStreamType.VISION_STREAM_ROAD)
    self.frame_road_id += 1

  def cam_send_yuv_wide_road(self, yuv):
    self._send_yuv(yuv, self.frame_wide_id, 'wideRoadCameraState', VisionStreamType.VISION_STREAM_WIDE_ROAD)
    self.frame_wide_id += 1

  # Returns: yuv bytes
  def rgb_to_yuv(self, rgb):
    assert rgb.shape == (H, W, 3), f"{rgb.shape}"
    assert rgb.dtype == np.uint8

    rgb_cl = cl_array.to_device(self.queue, rgb)
    yuv_cl = cl_array.empty_like(rgb_cl)
    self.krnl(self.queue, (self.Wdiv4, self.Hdiv4), None, rgb_cl.data, yuv_cl.data).wait()
    output = yuv_cl.get()
    assert output.size == rgb.size, f"{output.size} != {rgb.size}"
    return pack_nv12(output.reshape(-1)[:PACKED_NV12_SIZE]).tobytes()

  def _send_yuv(self, yuv, frame_id, pub_type, yuv_type):
    eof = int(time.monotonic() * 1e9)
    self.vipc_server.send(yuv_type, yuv, frame_id, eof, eof)

    dat = messaging.new_message(pub_type, valid=True)
    msg = {
      "frameId": frame_id,
      "transform": [1.0, 0.0, 0.0,
                    0.0, 1.0, 0.0,
                    0.0, 0.0, 1.0]
    }
    setattr(dat, pub_type, msg)
    self.pm.send(pub_type, dat)
