"""Wire format between the comma's V-ASM process and the Jetson's vasm_usb_server.

Fixed sizes on purpose: the comma is the USB gadget, so a crop goes out on its IN
endpoint as one FunctionFS write and the Jetson reads it as one bulk transfer of
REQ_SIZE. 371,728 bytes is not a multiple of any bulk max-packet size (64, 512,
1024), so every request ends with a short packet and the host read completes
without a zero-length packet.

  request  (comma -> Jetson)  REQ_HDR + crop_sq, uint8 [352,352,3] RGB, row-major
  response (Jetson -> comma)  RESP

side is 0 left / 1 right, carried only so a log on either end can say which.
status 0 is ok; anything else and probs are zeros and must not be used.
"""
import struct

import numpy as np

CROP_SHAPE = (352, 352, 3)
CROP_NBYTES = CROP_SHAPE[0] * CROP_SHAPE[1] * CROP_SHAPE[2]  # 371,712

REQ_MAGIC = b"VAQ1"
RESP_MAGIC = b"VAR1"
_REQ = struct.Struct("<4sIBxxxI")      # magic, seq, side, pad, payload nbytes
_RESP = struct.Struct("<4sIiI3f")      # magic, seq, status, server_us, probs[3]
REQ_HDR_SIZE = _REQ.size               # 16
REQ_SIZE = REQ_HDR_SIZE + CROP_NBYTES  # 371,728
RESP_SIZE = _RESP.size                 # 28

STATUS_OK = 0
STATUS_BAD_REQUEST = 1
STATUS_INFER_FAILED = 2
# unsolicited, Jetson -> comma, about once a second while idle: seq 0xFFFFFFFF,
# server_us = the first 4 bytes of the served ONNX's sha256 (big-endian), so the
# comma uses the Jetson only for the model it verified itself
STATUS_HELLO = 100
STATUS_YIELD = 101   # yield_gate.py: the driving model needs the GPU; the comma runs its own until a plain hello
HELLO_SEQ = 0xFFFFFFFF


def pack_hello(model_sha256_hex: str, status: int = STATUS_HELLO) -> bytes:
  return pack_response(HELLO_SEQ, status, int(model_sha256_hex[:8], 16))


def pack_request(seq: int, side: int, crop: np.ndarray) -> bytes:
  if crop.shape != CROP_SHAPE or crop.dtype != np.uint8:
    raise ValueError(f"crop must be uint8 {CROP_SHAPE}, got {crop.dtype} {crop.shape}")
  return _REQ.pack(REQ_MAGIC, seq & 0xFFFFFFFF, side, CROP_NBYTES) + crop.tobytes(order="C")


def unpack_request(buf) -> tuple[int, int, np.ndarray]:
  """Raises ValueError on anything but a whole, well-formed request."""
  if len(buf) != REQ_SIZE:
    raise ValueError(f"request is {len(buf)} bytes, expected {REQ_SIZE}")
  magic, seq, side, n = _REQ.unpack_from(buf, 0)
  if magic != REQ_MAGIC or n != CROP_NBYTES or side not in (0, 1):
    raise ValueError(f"bad request header magic={magic!r} side={side} nbytes={n}")
  crop = np.frombuffer(buf, dtype=np.uint8, count=CROP_NBYTES, offset=REQ_HDR_SIZE).reshape(CROP_SHAPE)
  return seq, side, crop


def pack_response(seq: int, status: int, server_us: int, probs=(0.0, 0.0, 0.0)) -> bytes:
  return _RESP.pack(RESP_MAGIC, seq & 0xFFFFFFFF, status, min(server_us, 0xFFFFFFFF), *probs)


def unpack_response(buf) -> tuple[int, int, int, tuple[float, float, float]]:
  if len(buf) != RESP_SIZE:
    raise ValueError(f"response is {len(buf)} bytes, expected {RESP_SIZE}")
  magic, seq, status, server_us, p0, p1, p2 = _RESP.unpack(bytes(buf))
  if magic != RESP_MAGIC:
    raise ValueError(f"bad response magic {magic!r}")
  return seq, status, server_us, (p0, p1, p2)
