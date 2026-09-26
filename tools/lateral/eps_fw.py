"""The modified-EPS firmware's torque table, read from a 39990-TBA C020 `.rwd`, as a stage of the lateral sim.

The EPS turns the openpilot steer command into assist through a 9-point table: an input axis (0..1774) and the
torque it asks the motor for. The table is the part of the firmware decoded well enough to model
(eps_tools/rwd/README.md, STATUS 145/146). The EPS's own controller words (tracker, Norm, P and D rows) set how the
motor tracks that torque, and they are NOT modelled here: they stay inside the fitted plant, as they were on the
drives it was fitted on.

Evidence for the choices below (STATUS 146, plant fits on 260-263, 268/271, 276/277, 278/27a at delay 5):
  - AXIS_SCALE 1.0 (full openpilot command = the top of the input axis). The stock C020 table saturates at axis
    1111 of 1774, 0.626 of full, which is where comma capped the stock Civic command (2560 / 4096 = 0.625).
    The owner's table is near-linear, so the logs cannot pin the scale themselves.
  - Row 0. The seven rows share one torque row in the owner's image and differ only in their axes; what selects a
    row is not decoded. No speed schedule tried (row 0/3 split at 11 m/s either way, row 0/6 at 20 m/s) beat row 0
    on every route group, and row 0 fits within 1% of a linear input on all four.
  - The stock table fits 15-25% worse than the owner's on every group, so the logs agree the car ran a modified
    table on those drives.
"""
from __future__ import annotations

import hashlib
import os
import struct

import numpy as np

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
C020_AXIS = 0x137f4     # 7 rows x 9 words, stride 0x12
C020_TORQUE = 0x13872   # 7 rows x 9 words, stride 0x12, directly after the axes
C020_ROW_STRIDE = 0x12
C020_ROWS = 7
AXIS_MAX = 1774
AXIS_SCALE = 1.0
TORQUE_UNIT = 10000.0   # plant input = table torque / TORQUE_UNIT, so plants fitted through different tables share units

# Controller words that change how the motor follows the table. A table swap between images that differ here is
# only half the change; the sim warns.
C020_WORDS = {"tracker": 0x137ee, "norm": 0x29efe, "speed_clamp": 0x1361c}
C020_ROW_WORDS = {"p_row0": 0x13bc0, "d_row0": 0x13ac4}


def decrypt_rwd(path):
  """(flash start, decrypted firmware bytes, raw file bytes) for a 0x5A-format rwd."""
  import importlib.util
  # eps_tools is not under the openpilot package; load the canonical lookup from the repo root.
  spec = importlib.util.spec_from_file_location("check_rwd", os.path.join(REPO_ROOT, "eps_tools", "check_rwd.py"))
  mod = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(mod)
  DECRYPT_LOOKUP = mod.DECRYPT_LOOKUP
  with open(path, "rb") as f:
    raw = f.read()
  if raw[:1] != b"\x5a":
    raise ValueError(f"{path}: not a 0x5A rwd")
  idx = 3
  for _ in range(6):
    cnt = raw[idx]
    idx += 1
    for _ in range(cnt):
      idx += 1 + raw[idx]
  start, length = struct.unpack("!II", raw[idx:idx + 8])
  idx += 8
  return start, bytes(DECRYPT_LOOKUP[b] for b in raw[idx:idx + length]), raw


class EpsTable:
  def __init__(self, axis, torque, row=0, source="", sha1="", words=None):
    self.axis = np.asarray(axis, dtype=float)
    self.torque = np.asarray(torque, dtype=float)
    if self.axis.shape != (9,) or self.torque.shape != (9,) or self.axis[0] != 0 or np.any(np.diff(self.axis) <= 0):
      raise ValueError(f"not a C020 torque table row: axis {self.axis}, torque {self.torque}")
    self.row = int(row)
    self.source = source
    self.sha1 = sha1
    self.words = dict(words or {})

  @staticmethod
  def from_rwd(path, row=0):
    start, fw, raw = decrypt_rwd(path)
    if b"39990-TBA" not in raw[:256] or b"C020" not in raw[:256]:
      raise ValueError(f"{path}: not a 39990-TBA C020 image; the table addresses are C020's")

    def w(addr, n=1):
      return [struct.unpack(">H", fw[addr - start + 2 * i:addr - start + 2 * i + 2])[0] for i in range(n)]
    if not 0 <= row < C020_ROWS:
      raise ValueError(f"row {row}: C020 has {C020_ROWS} rows")
    axis = w(C020_AXIS + row * C020_ROW_STRIDE, 9)
    if axis[-1] != AXIS_MAX:
      raise ValueError(f"{path}: axis row {row} ends at {axis[-1]}, not {AXIS_MAX}; not the C020 layout")
    words = {k: w(a)[0] for k, a in C020_WORDS.items()}
    words.update({k: w(a, 9) for k, a in C020_ROW_WORDS.items()})
    return EpsTable(axis, w(C020_TORQUE + row * C020_ROW_STRIDE, 9), row, os.path.basename(path),
                    hashlib.sha1(raw).hexdigest(), words)

  def drive(self, u):
    """Plant input for a delivered command u (1.0 = full openpilot command): the table torque, signed."""
    u = np.asarray(u, dtype=float)
    return np.sign(u) * np.interp(np.abs(u) * AXIS_SCALE * AXIS_MAX, self.axis, self.torque) / TORQUE_UNIT

  def controller_diff(self, other):
    """Controller words that differ between two images: {name: (self, other)}."""
    return {k: (self.words.get(k), other.words.get(k)) for k in sorted(set(self.words) | set(other.words))
            if self.words.get(k) != other.words.get(k)}

  def to_json(self):
    return {"source": self.source, "sha1": self.sha1, "row": self.row, "axis": self.axis.tolist(),
            "torque": self.torque.tolist(), "words": self.words, "axis_scale": AXIS_SCALE, "torque_unit": TORQUE_UNIT}

  @staticmethod
  def from_json(j):
    if j.get("axis_scale", AXIS_SCALE) != AXIS_SCALE or j.get("torque_unit", TORQUE_UNIT) != TORQUE_UNIT:
      raise ValueError("plant was fitted with a different axis scale or torque unit; refit it")
    return EpsTable(j["axis"], j["torque"], j.get("row", 0), j.get("source", ""), j.get("sha1", ""), j.get("words"))
