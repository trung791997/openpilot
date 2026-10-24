"""The gadget's optional second interface, for V-ASM crops to the Jetson.

jetlink-root.sh gadget links ffs.vasm after ffs.jetlink (so Jetlink stays
interface 0) only while /data/jetlink-vasm exists, and mounts it at
/dev/ffs-vasm. A linked FunctionFS function with no descriptors written makes
the UDC bind fail for the whole gadget, and closing its ep0 drops the whole
gadget, so the owner, the one long-lived process, writes these descriptors
before every bind and holds the ep0 for its life, as it does Jetlink's. The
V-ASM process only opens ep1/ep2 (openpilot/starpilot/spot_monitor/jetson_link.py).

The Jetson finds the interface by its class triple, not its number.
"""
from __future__ import annotations

import os
import struct

from jetlink.transport import ffs

MOUNT = '/dev/ffs-vasm'
EP0 = os.path.join(MOUNT, 'ep0')
IFACE_CLASS = (0xFF, 0x56, 0x41)   # vendor, 'V', 'A'; Jetlink's is FF/FF/FF
SS_MAX_PACKET = 1024


def _interface_desc() -> bytes:
  return struct.pack('<BBBBBBBBB', 9, ffs.USB_DT_INTERFACE, 0, 0, 2, *IFACE_CLASS, 1)


def build_descriptors() -> bytes:
  fs = _interface_desc() + ffs._endpoint_desc(ffs.EP_OUT, 64) + ffs._endpoint_desc(ffs.EP_IN, 64)
  hs = _interface_desc() + ffs._endpoint_desc(ffs.EP_OUT, 512) + ffs._endpoint_desc(ffs.EP_IN, 512)
  ss = (_interface_desc()
        + ffs._endpoint_desc(ffs.EP_OUT, SS_MAX_PACKET) + ffs._ss_companion()
        + ffs._endpoint_desc(ffs.EP_IN, SS_MAX_PACKET) + ffs._ss_companion())
  body = struct.pack('<III', 3, 3, 5) + fs + hs + ss
  flags = ffs.FLAG_HAS_FS | ffs.FLAG_HAS_HS | ffs.FLAG_HAS_SS
  return struct.pack('<III', ffs.FUNCTIONFS_DESCRIPTORS_MAGIC_V2, 12 + len(body), flags) + body


def present() -> bool:
  """Whether the gadget was built with the V-ASM interface."""
  return os.path.exists(EP0)


def open_ep0() -> int:
  """Open ep0 and write the descriptors; the caller keeps the fd for its life.
  Raises OSError (EBUSY while a V-ASM process from before still has ep1/ep2)."""
  fd = os.open(EP0, os.O_RDWR)
  try:
    os.write(fd, build_descriptors())
    os.write(fd, ffs.build_strings('vasm'))
  except OSError:
    os.close(fd)
    raise
  return fd
