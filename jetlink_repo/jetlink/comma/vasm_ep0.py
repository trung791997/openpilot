"""The gadget's optional vendor interfaces after Jetlink's: V-ASM crops (ffs.vasm)
and speed-limit vision's sign search (ffs.slv) to the Jetson. Each is opt-in by
its own marker; the module-level names are V-ASM's, SLV the other's.

jetlink-root.sh gadget links ffs.vasm after ffs.jetlink (so Jetlink stays
interface 0) only while /data/jetlink-vasm exists, and mounts it at
/dev/ffs-vasm. A linked FunctionFS function with no descriptors written makes
the UDC bind fail for the whole gadget, and closing its ep0 drops the whole
gadget, so the owner, the one long-lived process, writes these descriptors
before every bind and holds the ep0 for its life, as it does Jetlink's. The
V-ASM process only opens ep1/ep2 (openpilot/starpilot/spot_monitor/jetson_link.py).

The Jetson finds each interface by its class triple, not its number.
"""
from __future__ import annotations

import os
import struct
from typing import NamedTuple

from jetlink.transport import ffs

MOUNT = '/dev/ffs-vasm'
EP0 = os.path.join(MOUNT, 'ep0')
IFACE_CLASS = (0xFF, 0x56, 0x41)   # vendor, 'V', 'A'; Jetlink's is FF/FF/FF
SS_MAX_PACKET = 1024


class Aux(NamedTuple):
  name: str     # the FunctionFS instance, ffs.<name>, and its strings
  mount: str
  iface_class: tuple[int, int, int]

  @property
  def ep0(self) -> str:
    return os.path.join(self.mount, 'ep0')


VASM = Aux('vasm', MOUNT, IFACE_CLASS)
SLV = Aux('slv', '/dev/ffs-slv', (0xFF, 0x53, 0x4C))   # vendor, 'S', 'L'
AUX = (VASM, SLV)   # the order jetlink-root.sh links them in


def _interface_desc(iface_class=IFACE_CLASS) -> bytes:
  return struct.pack('<BBBBBBBBB', 9, ffs.USB_DT_INTERFACE, 0, 0, 2, *iface_class, 1)


def build_descriptors(iface_class=IFACE_CLASS) -> bytes:
  fs = _interface_desc(iface_class) + ffs._endpoint_desc(ffs.EP_OUT, 64) + ffs._endpoint_desc(ffs.EP_IN, 64)
  hs = _interface_desc(iface_class) + ffs._endpoint_desc(ffs.EP_OUT, 512) + ffs._endpoint_desc(ffs.EP_IN, 512)
  ss = (_interface_desc(iface_class)
        + ffs._endpoint_desc(ffs.EP_OUT, SS_MAX_PACKET) + ffs._ss_companion()
        + ffs._endpoint_desc(ffs.EP_IN, SS_MAX_PACKET) + ffs._ss_companion())
  body = struct.pack('<III', 3, 3, 5) + fs + hs + ss
  flags = ffs.FLAG_HAS_FS | ffs.FLAG_HAS_HS | ffs.FLAG_HAS_SS
  return struct.pack('<III', ffs.FUNCTIONFS_DESCRIPTORS_MAGIC_V2, 12 + len(body), flags) + body


def present(aux: Aux = VASM) -> bool:
  """Whether the gadget was built with this interface."""
  return os.path.exists(aux.ep0)


def open_ep0(aux: Aux = VASM) -> int:
  """Open ep0 and write the descriptors; the caller keeps the fd for its life.
  Raises OSError (EBUSY while a client process from before still has ep1/ep2)."""
  fd = os.open(aux.ep0, os.O_RDWR)
  try:
    os.write(fd, build_descriptors(aux.iface_class))
    os.write(fd, ffs.build_strings(aux.name))
  except OSError:
    os.close(fd)
    raise
  return fd
