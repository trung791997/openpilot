"""Read-only 20 Hz XCP Array B logger for Honda Bosch-A (36802-TBA-A150).

Observes 0x1EC00179 on CAN bus 1 and emits non-blocking single-in-flight
CONNECT/GET_STATUS/GET_SEED/UNLOCK/SHORT_UPLOAD requests on 0x1EC00079 (bus 1)
via an event-driven XCP I/O thread (~100 reads/sec hardware ceiling).
Never modifies CarState, RadarData, CarControl, or longitudinal/lateral control.
"""
from __future__ import annotations

import atexit
from collections import deque
import json
import math
import os
from pathlib import Path
import re
import threading
import time

from openpilot.common.swaglog import cloudlog
from opendbc.car import CanData
from opendbc.car.honda.values import HondaFlags

XCP_BUS = 1
XCP_CTO = 0x1EC00079
XCP_DTO = 0x1EC00179

ARRAY_B_BASE = 0x400371A4
REC0 = 0x400371B4
STRIDE = 136
NUM_SLOTS = 40
MAX_ACTIVE_TARGETS = 40
LEAD_PRIORITY_COUNT = 2

LOG_DIR = Path("/data/media/0/xcp_radar_logs")

EPOCH_NS = 50_000_000          # 20 Hz output logging cadence
LEAD_KIN_INTERVAL_NS = 35_000_000  # ~20-25 Hz kinematics for primary lead
LEAD_META_INTERVAL_NS = 120_000_000  # ~8 Hz alternating age / flags (~4 Hz each) for primary lead
LEAD_LAT_INTERVAL_NS = 300_000_000
LEAD_QUAL_INTERVAL_NS = 500_000_000
LEAD2_KIN_INTERVAL_NS = 90_000_000 # ~10 Hz kinematics for secondary priority target
LEAD2_META_INTERVAL_NS = 250_000_000
OTHER_META_INTERVAL_NS = 300_000_000
OTHER_MAX_STALE_NS = 350_000_000
STATIONARY_MAX_STALE_NS = 2_000_000_000
INACTIVE_PROBE_PERIOD_WITH_LEAD = 6
INACTIVE_PROBE_PERIOD_NO_LEAD = 3
ARRAY_B_RANGE_OFFSET_M = 2.9       # Array B raw X_raw/128 sits ~2.9 m ahead of 0x280 op_tracks.dRel
OP_TRACKS_MAX_AGE_NS = 500_000_000
ROUTE_POLL_INTERVAL_NS = 1_000_000_000

F4_TIMEOUT_NS = 150_000_000     # single SHORT_UPLOAD timeout
HANDSHAKE_RETRY_NS = 2_000_000_000
STARTUP_DELAY_NS = 500_000_000

_BOOT_ID_PATH = Path("/proc/sys/kernel/random/boot_id")


def key_for_seed(seed_be: bytes) -> bytes:
  value = int.from_bytes(seed_be, "big")
  for _ in range(7):
    carry = value >> 31
    value = ((value << 1) | carry) & 0xFFFFFFFF
    if not carry:
      break
    value ^= 0xFB312378
  return value.to_bytes(4, "little")


def decode_flags(flags: int, subflags: int, age: int) -> dict:
  upper = flags & 0xE000
  if upper == 0xE000:
    stage = "measured"
  elif upper == 0xA000:
    stage = "coasting"
  elif upper == 0x6000:
    stage = "birth"
  else:
    stage = "other"

  if (flags & 0x0F00) != 0:
    conf_band = "high"
  elif (flags & 0x0080) != 0:
    conf_band = "med"
  elif (flags & 0x1000) != 0:
    conf_band = "low"
  else:
    conf_band = "none"

  measured = bool(flags & 0x4000)
  ghost = bool(flags & 0x0020)
  in_path = bool(subflags & 0x0400)
  trust_accel = bool(upper == 0xE000 and (flags & 0x1020) == 0 and age >= 5)
  return {
    "stage": stage,
    "measured": measured,
    "conf_band": conf_band,
    "ghost": ghost,
    "in_path": in_path,
    "trust_accel": trust_accel,
  }


def _read_boot_id8() -> str:
  try:
    raw = _BOOT_ID_PATH.read_text().strip().replace("-", "")
    return raw[:8] or "nobootid"
  except Exception:
    return "nobootid"


def _sanitize_tag(raw: str | None) -> str:
  if not raw:
    return "offroad"
  cleaned = re.sub(r"[^A-Za-z0-9_-]+", "_", raw).strip("_")
  return cleaned or "offroad"


def _empty_slot_record() -> dict:
  return {
    "X_raw": 0,
    "Vx_raw": 0,
    "Ax_raw": 0,
    "Y_raw": 0,
    "Vy_raw": 0,
    "raw24": 0x8000,
    "age": 0,
    "prev_age": None,
    "marker": 0,
    "subflags": 0,
    "flags": 0,
    "qual": 0,
    "cls_conf": 0,
    "radar_track_id": 0,
    "kin_updated_ns": 0,
    "lat_updated_ns": 0,
    "age_updated_ns": 0,
    "flags_updated_ns": 0,
    "qual_updated_ns": 0,
    "updated_ns": 0,
    "last_rtt_ns": 0,
  }


class XcpRadarLogger:
  def __init__(self, CP, *, send_lock: threading.Lock | None = None, pm=None, params=None, log_dir: Path = LOG_DIR) -> None:
    self.enabled = False
    self.state = "DISABLED"
    self.in_flight = None  # (cmd_type, slot, offset, size, sent_ns)
    self._init_queue: deque[tuple[int, int, int]] = deque()
    self._init_queued: set[tuple[int, int, int]] = set()
    self.active_slots: dict[int, dict] = {}
    self.slot_cache: dict[int, dict] = {s: _empty_slot_record() for s in range(NUM_SLOTS)}
    self.scan_cursor = 0
    self._rr_cursor = 0
    self._read_seq = 0
    self._last_probe_seq = -INACTIVE_PROBE_PERIOD_WITH_LEAD
    self._next_radar_track_id = 1

    self.last_handshake_tx_ns = -HANDSHAKE_RETRY_NS
    self.last_log_ns = 0
    self.start_ns = time.monotonic_ns()
    self.consecutive_f4_timeouts = 0
    self.seed_bytes: bytes | None = None
    self.last_v_ego = 0.0

    self._op_tracks: list[dict] = []
    self._op_tracks_ns = 0
    self._op_matched_slots: list[int] = []

    self._lock = threading.RLock()
    self._file_lock = threading.Lock()
    self._send_lock = send_lock
    self._pm = pm
    self._params = params
    self._log_dir = Path(log_dir)
    self._boot_id8 = _read_boot_id8()
    self._fd: int | None = None
    self._log_path: Path | None = None
    self._current_route_tag: str | None = None
    self._last_route_check_ns = 0

    self._tx_armed = False
    self._tx_armed_ns = 0
    self._stop_event = threading.Event()
    self._tx_event = threading.Event()
    self._io_thread: threading.Thread | None = None
    self._use_io_thread = False
    self._closed = False

    try:
      if (
        "REPLAY" not in os.environ
        and getattr(CP, "brand", None) == "honda"
        and bool(CP.flags & HondaFlags.BOSCH)
        and not bool(CP.flags & (HondaFlags.BOSCH_RADARLESS | HondaFlags.BOSCH_CANFD))
        and not bool(CP.passive)
      ):
        if self._params is None:
          try:
            from openpilot.common.params import Params
            self._params = Params()
          except Exception:
            self._params = None
        self._log_dir.mkdir(parents=True, exist_ok=True)
        self.enabled = True
        self.state = "NEED_CONNECT"
        atexit.register(self.close)
        if self._pm is not None:
          self._use_io_thread = True
          self._io_thread = threading.Thread(target=self._io_loop, name="xcp_radar_io", daemon=True)
          self._io_thread.start()
    except Exception:
      self.enabled = False
      self.close()

  def _desired_route_tag(self) -> str:
    if self._params is None:
      return "offroad"
    try:
      is_offroad = bool(self._params.get_bool("IsOffroad"))
      if is_offroad:
        return "offroad"
      cur_route = self._params.get("CurrentRoute")
      if isinstance(cur_route, bytes):
        cur_route = cur_route.decode("utf-8", errors="ignore")
      return _sanitize_tag(cur_route)
    except Exception:
      return "offroad"

  def _close_fd_locked(self) -> None:
    if self._fd is not None:
      fd = self._fd
      self._fd = None
      try:
        os.fsync(fd)
      except OSError:
        pass
      try:
        os.close(fd)
      except OSError:
        pass

  def _rotate_log_file_if_needed_locked(self, now_ns: int, *, open_if_missing: bool = False) -> None:
    if not open_if_missing and self._fd is not None and now_ns - self._last_route_check_ns < ROUTE_POLL_INTERVAL_NS:
      return
    self._last_route_check_ns = now_ns
    desired_tag = self._desired_route_tag()
    if self._fd is not None and desired_tag == self._current_route_tag:
      return
    if self._fd is None and not open_if_missing:
      return
    if self._closed:
      return
    self._close_fd_locked()
    self._log_dir.mkdir(parents=True, exist_ok=True)
    ts = time.strftime("%Y%m%d_%H%M%S")
    path = self._log_dir / f"xcp_array_b_20hz_{desired_tag}_{self._boot_id8}_{ts}.jsonl"
    if path.exists():
      path = self._log_dir / f"xcp_array_b_20hz_{desired_tag}_{self._boot_id8}_{ts}_{now_ns % 1_000_000_000}.jsonl"
    flags = os.O_WRONLY | os.O_CREAT | os.O_APPEND
    self._fd = os.open(path, flags, 0o644)
    self._log_path = path
    self._current_route_tag = desired_tag
    tmp_link = self._log_dir / f".latest.{os.getpid()}.{threading.get_ident()}.tmp"
    latest_link = self._log_dir / "latest.jsonl"
    try:
      if tmp_link.exists() or tmp_link.is_symlink():
        tmp_link.unlink()
      os.symlink(path.name, tmp_link)
      os.replace(tmp_link, latest_link)
    except OSError:
      try:
        if tmp_link.exists() or tmp_link.is_symlink():
          tmp_link.unlink()
      except OSError:
        pass

  def _write_line(self, payload_line: str, now_ns: int) -> None:
    data = payload_line.encode("utf-8")
    with self._file_lock:
      if self._closed:
        return
      self._rotate_log_file_if_needed_locked(now_ns, open_if_missing=True)
      fd = self._fd
      if fd is None:
        return
      view = memoryview(data)
      written = 0
      while written < len(data):
        n = os.write(fd, view[written:])
        if n <= 0:
          break
        written += n

  def close(self) -> None:
    if self._closed:
      return
    self._closed = True
    self._stop_event.set()
    self._tx_event.set()
    io_thread = self._io_thread
    if io_thread is not None and io_thread is not threading.current_thread():
      io_thread.join(timeout=0.5)
    with self._file_lock:
      self._close_fd_locked()

  def _alloc_track_id_locked(self) -> int:
    tid = self._next_radar_track_id
    self._next_radar_track_id += 1
    return tid

  def _is_moving_target_locked(self, rec: dict) -> bool:
    if self.last_v_ego < 3.0:
      return True
    return abs(self.last_v_ego + rec["Vx_raw"] / 256.0) >= 1.5

  def _enqueue_init_locked(self, slot: int, *, moving: bool = True) -> None:
    if moving or self.last_v_ego < 3.0:
      # Push moving targets to front so initial flags/age/Y arrive before stale clutter
      for task in ((slot, 22, 6), (slot, 2, 6), (slot, 106, 2)):
        if task not in self._init_queued:
          self._init_queued.add(task)
          self._init_queue.appendleft(task)
    else:
      for task in ((slot, 106, 2), (slot, 22, 6)):
        if task not in self._init_queued:
          self._init_queued.add(task)
          self._init_queue.append(task)

  def _clear_session_locked(self, target_state: str = "NEED_CONNECT") -> None:
    self.state = target_state
    self.in_flight = None
    self._init_queue.clear()
    self._init_queued.clear()
    self.seed_bytes = None
    self.consecutive_f4_timeouts = 0

  def _reset_slot_locked(self, slot: int) -> None:
    self.active_slots.pop(slot, None)
    self.slot_cache[slot] = _empty_slot_record()

  def _handle_f4_reply_locked(self, slot: int, offset: int, size: int, payload: bytes, now_ns: int, sent_ns: int) -> None:
    if len(payload) < 1 + size or not (0 <= slot < NUM_SLOTS):
      return
    data = payload[1 : 1 + size]
    rec = self.slot_cache[slot]
    rtt_ns = max(0, now_ns - sent_ns)
    rec["updated_ns"] = now_ns
    rec["last_rtt_ns"] = rtt_ns

    if offset == 16 and size == 6:
      rec["X_raw"] = int.from_bytes(data[0:2], "big", signed=True)
      rec["Vx_raw"] = int.from_bytes(data[2:4], "big", signed=True)
      rec["Ax_raw"] = int.from_bytes(data[4:6], "big", signed=True)
      rec["kin_updated_ns"] = now_ns
    elif offset == 22 and size == 6:
      rec["Y_raw"] = int.from_bytes(data[0:2], "big", signed=True)
      rec["raw24"] = int.from_bytes(data[2:4], "big", signed=False)
      rec["Vy_raw"] = int.from_bytes(data[4:6], "big", signed=True)
      rec["lat_updated_ns"] = now_ns
    elif offset == 2 and size == 6:
      new_age = int.from_bytes(data[0:2], "big", signed=False)
      rec["marker"] = int.from_bytes(data[2:4], "big", signed=False)
      rec["subflags"] = int.from_bytes(data[4:6], "big", signed=False)
      prev_age = rec["prev_age"]
      if slot in self.active_slots and prev_age is not None and new_age < prev_age:
        rec["radar_track_id"] = self._alloc_track_id_locked()
      rec["age"] = new_age
      rec["prev_age"] = new_age
      rec["age_updated_ns"] = now_ns
    elif offset == 106 and size == 2:
      rec["flags"] = int.from_bytes(data[0:2], "big", signed=False)
      rec["flags_updated_ns"] = now_ns
    elif offset == 127 and size == 2:
      rec["qual"] = int(data[0])
      rec["cls_conf"] = int(data[1])
      rec["qual_updated_ns"] = now_ns

    flags = rec["flags"]
    flags_read = rec["flags_updated_ns"] > 0
    is_valid_bit13 = bool(flags & 0x2000)
    nonzero_kin = (rec["X_raw"] != 0 or rec["Vx_raw"] != 0 or rec["Ax_raw"] != 0)
    nonzero_any = nonzero_kin or (rec["Y_raw"] != 0 or rec["Vy_raw"] != 0)

    if nonzero_kin or (flags_read and is_valid_bit13):
      if slot not in self.active_slots and len(self.active_slots) < MAX_ACTIVE_TARGETS:
        if rec["radar_track_id"] == 0:
          rec["radar_track_id"] = self._alloc_track_id_locked()
        self.active_slots[slot] = rec
        self._enqueue_init_locked(slot, moving=self._is_moving_target_locked(rec))
      elif slot in self.active_slots:
        self.active_slots[slot] = rec

    if slot in self.active_slots:
      if not nonzero_kin and flags_read and not is_valid_bit13:
        self._reset_slot_locked(slot)
      elif not nonzero_any and flags_read and not is_valid_bit13:
        self._reset_slot_locked(slot)
      elif flags_read and not is_valid_bit13 and rec["flags_updated_ns"] >= rec["kin_updated_ns"]:
        self._reset_slot_locked(slot)
      elif offset == 16 and not nonzero_kin and (slot, 106, 2) not in self._init_queued:
        task = (slot, 106, 2)
        self._init_queued.add(task)
        self._init_queue.append(task)

  def _process_dto_locked(self, data: bytes, now_ns: int) -> None:
    if not data or self.in_flight is None:
      return
    pid = data[0]
    if pid not in (0xFF, 0xFE):
      return
    cmd_type, slot, offset, size, sent_ns = self.in_flight
    self.in_flight = None
    if pid == 0xFE:
      self._clear_session_locked("NEED_CONNECT")
      return
    self.consecutive_f4_timeouts = 0
    if cmd_type == "CONNECT":
      self.state = "NEED_STATUS"
    elif cmd_type == "STATUS":
      if len(data) >= 6 and (data[2] & 0x04) == 0:
        self.state = "READY"
      else:
        self.state = "NEED_SEED"
    elif cmd_type == "SEED":
      if len(data) >= 6:
        self.seed_bytes = bytes(data[2:6])
        self.state = "NEED_UNLOCK"
      else:
        self.state = "NEED_SEED"
    elif cmd_type == "UNLOCK":
      self.state = "READY"
    elif cmd_type == "F4":
      self._handle_f4_reply_locked(slot, offset, size, data, now_ns, sent_ns)

  def _next_inactive_probe_locked(self) -> tuple[int, int, int] | None:
    for _ in range(NUM_SLOTS):
      s = self.scan_cursor
      self.scan_cursor = (self.scan_cursor + 1) % NUM_SLOTS
      if s not in self.active_slots:
        return (s, 16, 6)
    return None

  def _select_priority_slots_locked(self) -> list[int]:
    prio: list[int] = []
    for s in self._op_matched_slots:
      if s in self.active_slots and s not in prio:
        prio.append(s)
        if len(prio) >= LEAD_PRIORITY_COUNT:
          return prio

    def _prio_key(item: tuple[int, dict]) -> tuple[int, int, int]:
      s, rec = item
      x_raw = rec["X_raw"]
      y_raw = rec["Y_raw"]
      is_moving = self._is_moving_target_locked(rec)
      in_path = bool(rec["subflags"] & 0x0400) or (x_raw > 0 and abs(y_raw) < 320)
      forward = x_raw > 0
      tier = 0 if (in_path and forward and is_moving) else (1 if (in_path and forward) else (2 if (forward and is_moving) else 3))
      return (tier, x_raw if forward else abs(x_raw) + 100_000, s)

    for s, _ in sorted(self.active_slots.items(), key=_prio_key):
      if s not in prio:
        prio.append(s)
        if len(prio) >= LEAD_PRIORITY_COUNT:
          break
    return prio

  def _next_read_task_locked(self, now_ns: int) -> tuple[int, int, int] | None:
    self._read_seq += 1

    prio_slots = self._select_priority_slots_locked()
    prio_set = set(prio_slots)

    # Primary lead (prio_slots[0]) sits strictly above init queue and inactive probing
    if prio_slots:
      s = prio_slots[0]
      rec = self.slot_cache[s]
      if now_ns - rec["kin_updated_ns"] >= LEAD_KIN_INTERVAL_NS:
        return (s, 16, 6)
      if rec["age_updated_ns"] == 0:
        return (s, 2, 6)
      if rec["flags_updated_ns"] == 0:
        return (s, 106, 2)
      if now_ns - max(rec["age_updated_ns"], rec["flags_updated_ns"]) >= LEAD_META_INTERVAL_NS:
        return (s, 2, 6) if rec["age_updated_ns"] <= rec["flags_updated_ns"] else (s, 106, 2)
      if now_ns - rec["lat_updated_ns"] >= LEAD_LAT_INTERVAL_NS:
        return (s, 22, 6)
      if now_ns - rec["qual_updated_ns"] >= LEAD_QUAL_INTERVAL_NS:
        return (s, 127, 2)

    # Adaptive inactive slot discovery (1/6 when tracking a lead, 1/3 otherwise)
    probe_period = INACTIVE_PROBE_PERIOD_WITH_LEAD if prio_slots else INACTIVE_PROBE_PERIOD_NO_LEAD
    if (self._read_seq - self._last_probe_seq) >= probe_period and len(self.active_slots) < NUM_SLOTS:
      probe = self._next_inactive_probe_locked()
      if probe is not None:
        self._last_probe_seq = self._read_seq
        return probe

    # Newly promoted active slots get initial flags/age/Y (moving targets queued at front)
    while self._init_queue:
      task = self._init_queue.popleft()
      self._init_queued.discard(task)
      if task[0] in self.active_slots:
        return task

    # Anti-starvation check for other active slots: moving traffic at >=2.8 Hz, stationary clutter at 0.5 Hz
    lead1 = prio_slots[0] if prio_slots else -1
    others_all = [s for s in sorted(self.active_slots) if s != lead1]
    if others_all:
      moving_others = [s for s in others_all if self._is_moving_target_locked(self.slot_cache[s])]
      stationary_others = [s for s in others_all if s not in moving_others]
      for group, max_stale_ns in ((moving_others, OTHER_MAX_STALE_NS), (stationary_others, STATIONARY_MAX_STALE_NS)):
        if not group:
          continue
        stale_kin_slot = min(group, key=lambda s: (self.slot_cache[s]["kin_updated_ns"], s))
        if now_ns - self.slot_cache[stale_kin_slot]["kin_updated_ns"] >= max_stale_ns:
          return (stale_kin_slot, 16, 6)
        stale_meta_slot = min(
          group,
          key=lambda s: (min(self.slot_cache[s]["age_updated_ns"], self.slot_cache[s]["flags_updated_ns"]), s),
        )
        rec = self.slot_cache[stale_meta_slot]
        if now_ns - min(rec["age_updated_ns"], rec["flags_updated_ns"]) >= 2 * max_stale_ns:
          return (stale_meta_slot, 2, 6) if rec["age_updated_ns"] <= rec["flags_updated_ns"] else (stale_meta_slot, 106, 2)

    # Secondary priority lead (prio_slots[1]): ~10 Hz kinematics + ~4 Hz alternating age/flags
    if len(prio_slots) > 1:
      s = prio_slots[1]
      rec = self.slot_cache[s]
      if now_ns - rec["kin_updated_ns"] >= LEAD2_KIN_INTERVAL_NS:
        return (s, 16, 6)
      if rec["age_updated_ns"] == 0:
        return (s, 2, 6)
      if rec["flags_updated_ns"] == 0:
        return (s, 106, 2)
      if now_ns - max(rec["age_updated_ns"], rec["flags_updated_ns"]) >= LEAD2_META_INTERVAL_NS:
        return (s, 2, 6) if rec["age_updated_ns"] <= rec["flags_updated_ns"] else (s, 106, 2)
      if now_ns - rec["lat_updated_ns"] >= LEAD_QUAL_INTERVAL_NS:
        return (s, 22, 6)
      if now_ns - rec["qual_updated_ns"] >= LEAD_QUAL_INTERVAL_NS:
        return (s, 127, 2)

    # Remaining non-priority moving targets: balanced oldest-first round-robin over kinematics + metadata
    others = [s for s in sorted(self.active_slots) if s not in prio_set and self._is_moving_target_locked(self.slot_cache[s])]
    if others:
      oldest_kin_slot = min(others, key=lambda s: (self.slot_cache[s]["kin_updated_ns"], s))
      rec = self.slot_cache[oldest_kin_slot]
      meta_candidates = [
        (rec["flags_updated_ns"], (oldest_kin_slot, 106, 2)),
        (rec["age_updated_ns"], (oldest_kin_slot, 2, 6)),
        (rec["lat_updated_ns"], (oldest_kin_slot, 22, 6)),
        (rec["qual_updated_ns"], (oldest_kin_slot, 127, 2)),
      ]
      stalest_meta_ts, stalest_meta_task = min(meta_candidates, key=lambda item: item[0])
      if stalest_meta_ts == 0 or (
        now_ns - stalest_meta_ts >= OTHER_META_INTERVAL_NS and stalest_meta_ts < rec["kin_updated_ns"]
      ):
        return stalest_meta_task
      return (oldest_kin_slot, 16, 6)

    # If no active slots need updating right now, sweep inactive slots faster
    if len(self.active_slots) < NUM_SLOTS:
      probe = self._next_inactive_probe_locked()
      if probe is not None:
        self._last_probe_seq = self._read_seq
        return probe

    if prio_slots:
      return (prio_slots[0], 16, 6)
    return None

  def _build_next_cto_locked(self, now_ns: int) -> bytes | None:
    if not self._tx_armed:
      return None
    if now_ns - self._tx_armed_ns < STARTUP_DELAY_NS:
      return None

    if self.in_flight is not None:
      cmd_type, _, _, _, sent_ns = self.in_flight
      timeout = F4_TIMEOUT_NS if cmd_type == "F4" else HANDSHAKE_RETRY_NS
      if now_ns - sent_ns < timeout:
        return None
      self.in_flight = None
      if cmd_type == "F4":
        self.consecutive_f4_timeouts += 1
        if self.consecutive_f4_timeouts >= 5:
          self._clear_session_locked("NEED_CONNECT")
          return None
      else:
        self._clear_session_locked("NEED_CONNECT")
        return None

    if self.state == "NEED_CONNECT":
      if now_ns - self.last_handshake_tx_ns < HANDSHAKE_RETRY_NS:
        return None
      self.last_handshake_tx_ns = now_ns
      self.in_flight = ("CONNECT", -1, 0, 0, now_ns)
      return b"\xFF\x00" + bytes(6)

    if self.state == "NEED_STATUS":
      self.in_flight = ("STATUS", -1, 0, 0, now_ns)
      return b"\xFD" + bytes(7)

    if self.state == "NEED_SEED":
      self.in_flight = ("SEED", -1, 0, 0, now_ns)
      return b"\xF8\x00\x04" + bytes(5)

    if self.state == "NEED_UNLOCK":
      if self.seed_bytes is None or len(self.seed_bytes) != 4:
        self.state = "NEED_SEED"
        return None
      key = key_for_seed(self.seed_bytes)
      self.seed_bytes = None
      self.in_flight = ("UNLOCK", -1, 0, 0, now_ns)
      return b"\xF7\x04" + key + bytes(2)

    if self.state == "READY":
      task = self._next_read_task_locked(now_ns)
      if task is None:
        return None
      slot, offset, size = task
      addr = REC0 + STRIDE * slot + offset
      pkt = bytes([0xF4, size, 0x00, 0x00]) + addr.to_bytes(4, "big")
      self.in_flight = ("F4", slot, offset, size, now_ns)
      return pkt

    return None

  def _send_cto(self, pkt: bytes) -> None:
    if self._pm is None:
      return
    from openpilot.selfdrive.pandad import can_list_to_can_capnp

    frame = can_list_to_can_capnp([(XCP_CTO, pkt, XCP_BUS)], msgtype="sendcan", valid=True)
    if self._send_lock is not None:
      with self._send_lock:
        self._pm.send("sendcan", frame)
    else:
      self._pm.send("sendcan", frame)

  def _io_loop(self) -> None:
    import openpilot.cereal.messaging as messaging
    from openpilot.selfdrive.pandad import can_capnp_to_list

    can_sock = None
    try:
      while not self._stop_event.is_set() and not self._tx_armed:
        self._tx_event.wait(timeout=0.05)
        self._tx_event.clear()

      if self._stop_event.is_set():
        return

      can_sock = messaging.sub_sock("can", timeout=5)
      while not self._stop_event.is_set():
        can_strs = messaging.drain_sock_raw(can_sock, wait_for_one=True)
        now_ns = time.monotonic_ns()
        if can_strs:
          can_list = can_capnp_to_list(can_strs)
          with self._lock:
            for _, packet in can_list:
              for addr, dat, src in packet:
                if src == XCP_BUS and addr == XCP_DTO:
                  self._process_dto_locked(bytes(dat), now_ns)
            pkt = self._build_next_cto_locked(now_ns)
        else:
          with self._lock:
            pkt = self._build_next_cto_locked(now_ns)
        if pkt is not None:
          self._send_cto(pkt)
    except Exception:
      pass
    finally:
      if can_sock is not None:
        try:
          del can_sock
        except Exception:
          pass

  def _match_openpilot_tracks(self, targets: list[dict], op_tracks: list[dict]) -> tuple[list[int], int | None]:
    if not targets or not op_tracks:
      return [], None
    candidates: list[tuple[float, int, int, float]] = []
    for ti, t in enumerate(targets):
      d_adj = t["dRel_m"] - ARRAY_B_RANGE_OFFSET_M
      for oi, o in enumerate(op_tracks):
        dd = abs(d_adj - o["dRel"])
        dy = abs(t["yRel_m"] - o["yRel"])
        dv = abs(t["vRel_mps"] - o["vRel"])
        max_dd = max(4.5, 0.14 * max(d_adj, o["dRel"], 0.0))
        if dd < max_dd and dy < 2.2 and dv < 3.5:
          cost = dd * dd + (2.0 * dy) * (2.0 * dy) + (0.8 * dv) * (0.8 * dv)
          dist_m = math.hypot(dd, dy)
          candidates.append((cost, ti, oi, dist_m))
    candidates.sort(key=lambda item: item[0])

    used_targets: set[int] = set()
    used_op: set[int] = set()
    op_in_path = [o for o in op_tracks if o["dRel"] > 0.0 and abs(o["yRel"]) < 2.0]
    op_lead_tid = min(op_in_path, key=lambda o: o["dRel"])["tid"] if op_in_path else None
    lead_matched_slots: list[int] = []
    matched_slots: list[int] = []
    op_lead_slot: int | None = None

    for _, ti, oi, dist_m in candidates:
      if ti in used_targets or oi in used_op:
        continue
      used_targets.add(ti)
      used_op.add(oi)
      tid = op_tracks[oi]["tid"]
      targets[ti]["op_track_id"] = tid
      targets[ti]["op_match_dist_m"] = round(dist_m, 3)
      slot = targets[ti]["slot"]
      if tid == op_lead_tid:
        lead_matched_slots.insert(0, slot)
        op_lead_slot = slot
      else:
        matched_slots.append(slot)
    return lead_matched_slots + matched_slots, op_lead_slot

  def _maybe_log_20hz(self, now_ns: int) -> None:
    with self._lock:
      if self.state != "READY":
        return
      if now_ns - self.last_log_ns < EPOCH_NS:
        return
      self.last_log_ns = now_ns
      v_ego = self.last_v_ego
      op_tracks = list(self._op_tracks) if (now_ns - self._op_tracks_ns <= OP_TRACKS_MAX_AGE_NS) else []

      targets: list[dict] = []
      for slot, rec in self.active_slots.items():
        if rec["flags_updated_ns"] <= 0:
          continue
        flags = rec["flags"]
        if (flags & 0x2000) == 0:
          continue
        X = rec["X_raw"]
        Y = rec["Y_raw"]
        Vx = rec["Vx_raw"]
        Vy = rec["Vy_raw"]
        Ax = rec["Ax_raw"]
        if X == 0 and Y == 0 and Vx == 0 and Vy == 0:
          continue
        subflags = rec["subflags"]
        age = rec["age"]
        dec = decode_flags(flags, subflags, age)
        targets.append({
          "slot": slot,
          "radar_track_id": rec["radar_track_id"],
          "dRel_m": round(X / 128.0, 4),
          "vRel_mps": round(Vx / 256.0, 4),
          "yRel_m": round(Y / 128.0, 4),
          "vyRel_mps": round(Vy / 256.0, 4),
          "ax_mps2": round(Ax / 2048.0, 4),
          "flags": flags,
          "subflags": subflags,
          "age": age,
          "qual": rec["qual"],
          "cls_conf": rec["cls_conf"],
          "stage": dec["stage"],
          "measured": dec["measured"],
          "conf_band": dec["conf_band"],
          "ghost": dec["ghost"],
          "in_path": dec["in_path"],
          "trust_accel": dec["trust_accel"],
          "op_track_id": None,
          "op_match_dist_m": None,
          "kin_updated_ns": rec["kin_updated_ns"],
          "age_updated_ns": rec["age_updated_ns"],
          "flags_updated_ns": rec["flags_updated_ns"],
          "updated_ns": rec["updated_ns"],
          "last_rtt_ms": round(rec["last_rtt_ns"] / 1_000_000.0, 2),
        })

    matched_slots, op_lead_slot = self._match_openpilot_tracks(targets, op_tracks)
    with self._lock:
      self._op_matched_slots = matched_slots

    targets.sort(key=lambda t: (abs(t["dRel_m"]), t["slot"]))
    lead = next((t for t in targets if t["slot"] == op_lead_slot), None) if op_lead_slot is not None else None
    if lead is None:
      in_path = [t for t in targets if t["dRel_m"] > 0.0 and abs(t["yRel_m"]) < 2.0]
      if v_ego >= 3.0:
        moving_in_path = [t for t in in_path if abs(v_ego + t["vRel_mps"]) >= 1.5]
        lead = min(moving_in_path or in_path, key=lambda t: t["dRel_m"]) if in_path else None
      else:
        lead = min(in_path, key=lambda t: t["dRel_m"]) if in_path else None
    row = {
      "mono_ns": now_ns,
      "v_ego": round(v_ego, 3),
      "lead": lead,
      "targets": targets,
      "op_tracks": op_tracks,
    }
    self._write_line(json.dumps(row, separators=(",", ":")) + "\n", now_ns)
    cloudlog.event(
      "xcp_array_b_20hz",
      mono_ns=row["mono_ns"],
      v_ego=row["v_ego"],
      lead=row["lead"],
      targets=row["targets"],
      op_tracks=row["op_tracks"],
      debug=True,
    )

  def observe_radar_tracks(self, RD, now_ns: int) -> None:
    if not self.enabled or RD is None:
      return
    try:
      tracks = [
        {
          "tid": int(pt.trackId),
          "dRel": round(float(pt.dRel), 3),
          "yRel": round(float(pt.yRel), 3),
          "vRel": round(float(pt.vRel), 3),
        }
        for pt in RD.points
      ]
      with self._lock:
        self._op_tracks = tracks
        self._op_tracks_ns = int(now_ns)
    except Exception:
      pass

  def observe_can(self, can_list, now_ns: int, v_ego: float = 0.0) -> None:
    if not self.enabled:
      return
    try:
      with self._lock:
        self.last_v_ego = float(v_ego)
        if not self._use_io_thread:
          for _, packet in can_list:
            for addr, dat, src in packet:
              if src == XCP_BUS and addr == XCP_DTO:
                self._process_dto_locked(bytes(dat), now_ns)
      self._maybe_log_20hz(now_ns)
    except Exception:
      pass

  def next_can_sends(self, now_ns: int) -> list[CanData]:
    if not self.enabled:
      return []
    try:
      with self._lock:
        if not self._tx_armed:
          self._tx_armed = True
          self._tx_armed_ns = int(now_ns)
          self._tx_event.set()
        if self._use_io_thread:
          return []
        pkt = self._build_next_cto_locked(now_ns)
        return [CanData(XCP_CTO, pkt, XCP_BUS)] if pkt is not None else []
    except Exception:
      pass
    return []
