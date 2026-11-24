"""Read-only 20 Hz XCP Array B logger for Honda Bosch-A (36802-TBA-A150).

Observes 0x1EC00179 on CAN bus 1 and emits non-blocking single-in-flight
CONNECT/GET_STATUS/GET_SEED/UNLOCK/SHORT_UPLOAD requests on 0x1EC00079 (bus 1)
via an event-driven XCP I/O thread (~100 reads/sec hardware ceiling).
Never modifies CarState, RadarData, CarControl, or longitudinal/lateral control.
"""
from __future__ import annotations

import atexit
from collections import deque
import fcntl
import json
import math
import os
from pathlib import Path
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

EPOCH_NS = 50_000_000          # 20 Hz output logging cadence
XCP_READS_LOG = True           # per-read "xcp_reads" event (logging only; no effect on scheduling or control)
READS_FLUSH_NS = 250_000_000   # xcp_reads flush grid
READS_BUF_MAX = 128            # max buffered read records between flushes
# Evidence (route 109 logged lead kin reads, 996 value-change intervals; 38 ms median read gap limits resolution):
# 0% of change intervals < 29 ms, 2.8% < 30-59 ms, 3.4% < 60 ms, 4.8% < 70 ms, mode 89-91 ms. W = 29 ms is the largest
# hold with < 2% shorter intervals. Sim (stepwise 90 ms truth, 3 seeds): lead kin reads/s 20.8 -> 18.4, lead
# change-to-read latency p50/p90 19.3/37.4 -> 19.4/37.4 ms. A 60 ms hold (3.4% shorter intervals) cut lead kin reads/s to 11.8
# at +2/+3 ms latency; see longsim_frames/xcp_assist/cutin/REPORT.md round 4.
LEAD_KIN_HOLD_NS = 29_000_000
LEAD_KIN_HOLD_MAX_NS = 150_000_000
LEAD_KIN_INTERVAL_NS = 18_000_000  # ~25 Hz (every 40 ms) at 20 ms RTT, ~33 Hz at 10 ms RTT
LEAD_META_INTERVAL_NS = 250_000_000  # ~4 Hz alternating age / flags (~2 Hz each) for primary lead
LEAD_LAT_INTERVAL_NS = 180_000_000   # ~5.5 Hz Y / Vy for primary lead
LEAD_QUAL_INTERVAL_NS = 1_000_000_000
LEAD2_KIN_INTERVAL_NS = 110_000_000  # ~8 Hz kinematics for secondary priority target
LEAD2_LAT_INTERVAL_NS = 150_000_000  # ~6.7 Hz Y / Vy for secondary priority target
LEAD2_META_INTERVAL_NS = 800_000_000
OTHER_LAT_INTERVAL_NS = 250_000_000  # ~4 Hz Y / Vy target cadence for moving cut-in traffic
OTHER_META_INTERVAL_NS = 1_000_000_000
OTHER_QUAL_INTERVAL_NS = 1_000_000_000
OTHER_MAX_STALE_NS = 350_000_000
OTHER_META_MAX_STALE_NS = 1_000_000_000
FAR_OTHER_KIN_INTERVAL_NS = 700_000_000
FAR_OTHER_LAT_INTERVAL_NS = 700_000_000
FAR_OTHER_META_INTERVAL_NS = 1_200_000_000
NEWBORN_AGE_REFRESH_NS = 100_000_000  # re-poll newborn track age < 3 quickly so UI unlocks promptly
NEAR_MOVING_Y_RAW = 768               # ±6.0 m (1/128 m/lsb) keeps adjacent lanes ahead of wide oncoming cars
IN_PATH_Y_RAW = 256                   # ±2.0 m (1/128 m/lsb) matching _maybe_log_20hz fallback width once lat is known
CUTIN_MAX_X_RAW = 40 * 128            # 40.0 m range ceiling for CAN-matched cut-in protection in prio_slots[1]
CUTIN_MAX_ABS_Y_RAW = 4 * 128         # ±4.0 m lateral ceiling for CAN-matched cut-in protection in prio_slots[1]
CUTIN_VY_THRESH_RAW = round(0.3 * 256)  # 0.3 m/s lateral velocity tie-breaker for active cut-in candidates
# Route 00000105--c4d0cd8039 sim: yielding 1 Hz read-only Way B diagnostic reads once prio_slots[1]
# cut-in kin_age >= 140 ms keeps WAYB at 8.82/s while cutting Group 2 >200 ms from 34.40% to 12.16%
# and active cut-in >200 ms from 33.57% to 14.10%, without starving other near moving lanes.
CUTIN_WAYB_YIELD_NS = 140_000_000
# Protected cut-in set: up to CUTIN_PROTECT_N moving, lat-and-kin-read non-lead targets in the cut-in
# band (0 < d < 40 m, 1.2 < |y| < 3.5 m), ranked by time-to-path |y| / max(toward-speed, 0.1) then X,
# plus the CAN-matched prio_slots[1] candidate above, share the urgent step. Each step reads the
# protected slot with the largest staleness ratio over {kin age / CUTIN_PROTECT_KIN_NS,
# lat age / CUTIN_PROTECT_LAT_NS} (kin or Y/Vy read accordingly), once any ratio is >= 1.
# Evidence (offline simulation of route 109, real scheduler == sim rows, 3 seeds, no road evidence;
# longsim_frames/xcp_assist/cutin/REPORT.md round 6, arm D = this cap + guard + LEAD_KIN_HOLD + Way B off), base -> D:
# 70 reads/s: cut-in risk kin_age >200 ms 54.0% -> 28.4% (p90 1184 -> 678 ms), band lat p90 701 -> 399 ms,
# non-band near-moving lat p90 775 -> 600 ms, lead >200 ms 0.43% -> 0.39%, lead lat p90 290 -> 190 ms,
# discovery p50 1441 -> 1228 ms. 48.6 reads/s (synthetic): risk 52.3% -> 48.4% but risk p90 2283 -> 3468 ms.
# Uncapped protection (CL3I) cut risk to ~1% but starved non-band lat and failed two logger tests; kin-only
# protection (C3_150) starved band lat. The cap without the guard (17.0%) failed the standing-traffic and
# adjacent-lat tests during multi-slot discovery.
CUTIN_PROTECT_N = 3
CUTIN_PROTECT_KIN_NS = 150_000_000
CUTIN_PROTECT_LAT_NS = 250_000_000
CUTIN_PROTECT_CAP = 6
CUTIN_PROTECT_WINDOW = 20
# Discovery guard: the protect step is skipped while the oldest queued init item (new-slot kin/lat/age/flags
# reads, moving or stationary) has waited longer than this, so the init queue and the near-moving group behind it are reached even
# when protect + probe would otherwise take every spare read. Peter 2026-11-23: arm D, round 6; offline replay
# evidence (no road evidence): ~/longsim_frames/xcp_assist/cutin/REPORT.md (Round 6).
CUTIN_PROTECT_GUARD_NS = 300_000_000
CUTIN_PROTECT_MAX_X_M = 40.0
CUTIN_PROTECT_MIN_ABS_Y_M = 1.2
CUTIN_PROTECT_MAX_ABS_Y_M = 3.5
STATIONARY_IN_PATH_Y_RAW = 384        # ±3.0 m fast age < 3 refresh for oncoming standing traffic
STATIONARY_INIT_YIELD_READS = 8       # yield 1 read per 8 non-lead cycles to init waiting stopped slots without starving adjacent lanes
STATIONARY_MAX_STALE_NS = 3_000_000_000
INACTIVE_PROBE_PERIOD_WITH_LEAD = 10
INACTIVE_PROBE_PERIOD_NO_LEAD = 3
OP_TRACKS_MAX_AGE_NS = 500_000_000
MEASURED_MAX_KIN_AGE_NS = 200_000_000  # 200 ms freshness ceiling for stage=='measured'
UI_EXTRAP_CAP_NS = 600_000_000  # UI dots: extrapolate a point by vRel/vyRel for at most this long since its last read
ASSIST_SHM_MIN_INTERVAL_NS = 20_000_000  # 50 Hz max assist-only SHM publish on primary-lead kin arrival
OP_MATCH_LOCKED_CLOSING_MAX_DV_MPS = 8.0
OP_MATCH_LOCKED_CLOSING_MAX_DY_M = 1.5
OP_MATCH_LOCKED_CLOSING_MAX_ABS_Y_M = 1.5
OP_MATCH_LOCKED_MIN_VLEAD_MPS = -0.5
ARRAY_B_TRAILER_OFFSET_M = 434.0 / 128.0  # 3.390625 m A150 Array B -> Array A range offset

F4_TIMEOUT_NS = 150_000_000     # single SHORT_UPLOAD timeout
# Quiet drain after any CTO timeout before sending the next CTO so a late ECU reply
# (arriving in 150..200 ms) hits in_flight is None and is discarded instead of mismatching
# a fresh F4/WAYB/CONNECT request or causing a 1-frame pipeline shift.
# Evidence: route 00000105--c4d0cd8039 (6e465d40dc, 9.06 min): 0 active-drive timeouts,
# 2,109/25,463 (8.28%) valid sendcan->can replies < 5 ms (min 1.36 ms), max sendcan->can 28.78 ms.
F4_TIMEOUT_DRAIN_NS = 50_000_000
HANDSHAKE_RETRY_NS = 2_000_000_000
STARTUP_DELAY_NS = 500_000_000

# XCP SHORT_UPLOAD request pipelining depth.
#   1  -> today's strict single-in-flight schedule (default; byte-identical to HEAD).
#   >1 -> allow up to this many READY-state SHORT_UPLOADs outstanding at once so our next
#         request overlaps the radar's ~10 ms reply slot instead of waiting a full RTT.
# SHORT_UPLOAD positive replies carry NO tag, so outstanding requests are matched to replies
# strictly in send (FIFO) order. Any timeout, XCP error (0xFE), or length mismatch flushes the
# pending tail and resyncs to single-in-flight (+ the normal post-timeout quiet drain), so a lost
# or reordered reply cannot silently mis-assign to the wrong request for more than one round.
# Only READY-state F4/WAYB SHORT_UPLOADs pipeline; the CONNECT/STATUS/SEED/UNLOCK handshake is
# always strictly serial (stateful, and its reply gates the next request).
# UNPROVEN on-car: whether this radar tolerates 2 outstanding SHORT_UPLOADs is unknown and needs a
# parked-car probe before any road use. Default stays 1 until that probe passes.
XCP_PIPELINE_DEPTH = max(1, int(os.environ.get("XCP_PIPELINE_DEPTH", "1") or "1"))

# 1 Hz Way B internal-fusion diagnostic probe (Bosch-A 36802-TBA-A150 verified SRAM bases).
# Offline route evidence (bob-unify-car):
# - T1=0x40032AEC and OUT18=0x4003681C (from 8f88b00741) confirmed on 000000f4--618185d0bb and 000000f5--6e8e645f02.
# - P=0x400397FC (-4 from 8343dbb36f's 0x40039800) and full 26-seg scan of 000000fa--148773dcd2 (948d8c2f5, 1483 rounds, 0 XCP errors):
#   - T1+86 bit 0x0010 matched OUT18+16 != 0 on 1481/1483 rounds; T1+28 is lateral Y (1/128 m), T1+40 is vRel (1/256 m/s).
#   - T1+3414 = 0x40033842 (T1[32]+86) stayed static sel_cam_id=7, sel_state=255 on 75/75 nonzero rounds; removed in 277452e602.
# - Garage route 000000ff--ae93650ec9 (277452e602, seg 0, 20 rounds) + full 10-seg drive 00000105--c4d0cd8039
#   (6e465d40dc, 510 rounds, 0 XCP errors):
#   - 259/259 B p_win sweep (0x400397DC..0x400398DE) exposed P_arr[0..3] as 4 x 14 B entries starting at 0x400397E0
#     (0x400397E0/EE/FC/80A, 0xFF idle sentinels at +7); 0x400397FC is P_arr[2]. None is an always-on ACC lead pointer
#     (active counts: P_arr[0]=5/29, P_arr[1]=0/28, P_arr[2]=110/510 idle on 281/384 lead_scan rounds, P_arr[3]=1/27).
#   - Complete 104/104 B T1[18] decode (seg=02): +16..23 counters/metrics, +24..25 X_raw (1/128 m; +3.164 m p50 vs Array B
#     at v_ego >= 5 m/s, standstill fl=0x0e50 freeze quirk up to +8.695 m), +28..29 & +30..31 Y_raw (1/128 m; p50=-0.072 m vs
#     CAN yRel), +38..39 & +40..41 Vx_raw (1/256 m/s; p50=-0.004 m/s vs Array B vRel), +86..87 t1_flags, +91 t1_valid.
#   - OUT18[0..31]+16 4-slot/sec scan + lock tracked live in-lane lead on 384/510 rounds (374/384 in radarTracks, 345/384 == lead.op).
# - Temporary p_win and t1_dump sweeps removed once completed (12 -> 9 reads/sec: p0 + 4 core reads + 4 OUT18 scan reads,
#   always deferred behind _primary_lead_urgent_locked).
# Peter 2026-11-23: off; reads go to cut-in protection; study ~/longsim_frames/xcp_assist/wayb_study/REPORT.md inconclusive.
# Master switch: with False the probe never issues a read (no p0/t1/out18 reads, no cut-in yield read), whatever probe_wayb says.
WAYB_PROBE_ENABLED = False
WAYB_PROBE_INTERVAL_NS = 1_000_000_000
WAYB_DEFAULT_T1_BASE = 0x40032AEC
WAYB_DEFAULT_OUT18_BASE = 0x4003681C
WAYB_DEFAULT_P_BASE = 0x400397FC
WAYB_T1_STRIDE = 104
WAYB_OUT18_STRIDE = 18
WAYB_T1_NUM_SLOTS = 32
WAYB_OUT18_SCAN_PER_ROUND = 4

XCP_UI_SHM_PATH = Path("/dev/shm/xcp_radar_points.json") if Path("/dev/shm").is_dir() else None
XCP_ASSIST_SHM_PATH = Path("/dev/shm/xcp_radar_assist.json") if Path("/dev/shm").is_dir() else None
_DEFAULT_ASSIST_SHM_PATH = object()
_DEFAULT_SENDER_LOCK_PATH = object()
_CLOCK_BOOTTIME = getattr(time, "CLOCK_BOOTTIME", None)


def _default_sender_lock_path() -> Path:
  base = Path("/dev/shm") if Path("/dev/shm").is_dir() else Path("/tmp")
  return base / "honda_xcp_sender.lock"


def _boot_minus_mono_ns() -> int:
  if _CLOCK_BOOTTIME is not None:
    try:
      return max(0, time.clock_gettime_ns(_CLOCK_BOOTTIME) - time.monotonic_ns())
    except OSError:
      pass
  return 0


def _write_atomic_json(path: Path, data: dict) -> None:
  payload = json.dumps(data, separators=(",", ":")).encode("utf-8")
  # per thread: the IO thread and card's thread both write the same paths
  tmp = path.with_name(f".{path.name}.{os.getpid()}.{threading.get_ident()}.tmp")
  tmp.write_bytes(payload)
  os.replace(tmp, path)


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
    "kin_changed_ns": 0,
    "kin_value_changes": 0,
    "lat_updated_ns": 0,
    "age_updated_ns": 0,
    "flags_updated_ns": 0,
    "qual_updated_ns": 0,
    "updated_ns": 0,
    "last_rtt_ns": 0,
  }


class XcpRadarLogger:
  def __init__(
    self,
    CP,
    *,
    send_lock: threading.Lock | None = None,
    pm=None,
    params=None,
    send_topic: str = "sendcan",
    use_io_thread: bool | None = None,
    sender_lock_path: Path | None | object = _DEFAULT_SENDER_LOCK_PATH,
    shm_path: Path | None = XCP_UI_SHM_PATH,
    assist_shm_path: Path | None | object = _DEFAULT_ASSIST_SHM_PATH,
    assist_fast_publish: bool | None = None,
    probe_wayb: bool | None = None,
    pipeline_depth: int | None = None,
  ) -> None:
    if assist_fast_publish is not None:
      self._assist_fast_publish_enabled = bool(assist_fast_publish)
    elif params is not None:
      try:
        from openpilot.starpilot.car.honda.xcp_radar_assist import enabled as xcp_assist_enabled
        self._assist_fast_publish_enabled = bool(xcp_assist_enabled(params, CP))
      except Exception:
        self._assist_fast_publish_enabled = False
    else:
      self._assist_fast_publish_enabled = False
    self.enabled = False
    self.state = "DISABLED"
    self.in_flight = None  # oldest outstanding request (cmd_type, slot, offset, size, sent_ns)
    # Additional outstanding SHORT_UPLOADs beyond the head when pipelining (depth > 1). Always
    # empty at depth 1, so the single-in-flight schedule is byte-identical to HEAD.
    self._pipeline_depth = max(1, int(pipeline_depth if pipeline_depth is not None else XCP_PIPELINE_DEPTH))
    self._inflight_tail: deque[tuple] = deque()
    self.pipeline_collision_skips = 0  # pipelined picks rolled back for hitting an outstanding address
    self._init_queue: deque[tuple[int, int, int]] = deque()
    self._init_queued: set[tuple[int, int, int]] = set()
    self._init_stamp: dict[tuple[int, int, int], int] = {}  # first-seen time of queued init items (guard)
    self.active_slots: dict[int, dict] = {}
    self.slot_cache: dict[int, dict] = {s: _empty_slot_record() for s in range(NUM_SLOTS)}
    self.scan_cursor = 0
    self._rr_cursor = 0
    self._read_seq = 0
    self._last_probe_seq = -INACTIVE_PROBE_PERIOD_WITH_LEAD
    self._stationary_init_skip_count = 0
    self._protect_hist: deque = deque(maxlen=CUTIN_PROTECT_WINDOW)
    self._next_radar_track_id = 1

    self.last_handshake_tx_ns = -HANDSHAKE_RETRY_NS
    self.last_log_ns = 0
    self._next_log_ns = 0
    self._reads_buf: list[tuple] = []
    self._reads_dropped = 0
    self._next_reads_flush_ns = 0
    self.start_ns = time.monotonic_ns()
    self.consecutive_f4_timeouts = 0
    self._timeout_drain_until_ns = 0
    self.f4_timeout_count = 0
    self.wayb_timeout_count = 0
    self.handshake_timeout_count = 0
    self.drain_discard_count = 0
    self.seed_bytes: bytes | None = None
    self.last_v_ego = 0.0

    self._op_tracks: list[dict] = []
    self._op_tracks_ns = 0
    self._op_matched_slots: list[int] = []
    self._op_lead_slot: int | None = None
    self._prev_op_slot_by_tid: dict[int, int] = {}
    self._last_assist_shm_ns: int = 0
    self._assist_shm_dirty: bool = False

    self._lock = threading.RLock()
    self._send_lock = send_lock
    self._pm = pm
    self._send_topic = send_topic
    self._sender_lock_fd: int | None = None
    if sender_lock_path is _DEFAULT_SENDER_LOCK_PATH:
      resolved_lock_path = _default_sender_lock_path() if self._pm is not None else None
    else:
      resolved_lock_path = Path(sender_lock_path) if sender_lock_path is not None else None
    self._wayb_probe_enabled = WAYB_PROBE_ENABLED and bool(probe_wayb if probe_wayb is not None else (pm is not None))
    self._wayb_queue: deque[tuple[str, int, int]] = deque()
    self._wayb_last_round_ns = 0
    self._wayb_t1_base = WAYB_DEFAULT_T1_BASE
    self._wayb_out18_base = WAYB_DEFAULT_OUT18_BASE
    self._wayb_p_base = WAYB_DEFAULT_P_BASE
    self._wayb_lead_slot: int | None = None
    self._wayb_lead_cam_id: int | None = None
    self._wayb_out18_cursor = 0
    self._wayb_state: dict = self._empty_wayb_state()
    self._shm_path = Path(shm_path) if shm_path is not None else None
    if assist_shm_path is _DEFAULT_ASSIST_SHM_PATH:
      self._assist_shm_path = XCP_ASSIST_SHM_PATH if shm_path is XCP_UI_SHM_PATH else None
    else:
      self._assist_shm_path = Path(assist_shm_path) if assist_shm_path is not None else None

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
        if resolved_lock_path is not None:
          fd = os.open(resolved_lock_path, os.O_CREAT | os.O_RDWR, 0o644)
          try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
          except BlockingIOError:
            os.close(fd)
            cloudlog.warning("xcp_radar_logger: sender lock held by another process, refusing second sender")
            return
          self._sender_lock_fd = fd
        self.enabled = True
        self.state = "NEED_CONNECT"
        atexit.register(self.close)
        want_io_thread = (self._pm is not None) if use_io_thread is None else (bool(use_io_thread) and self._pm is not None)
        if want_io_thread:
          self._use_io_thread = True
          self._io_thread = threading.Thread(target=self._io_loop, name="xcp_radar_io", daemon=True)
          self._io_thread.start()
    except Exception:
      self.enabled = False
      self.close()

  def close(self) -> None:
    if self._closed:
      return
    self._closed = True
    self._stop_event.set()
    self._tx_event.set()
    io_thread = self._io_thread
    if io_thread is not None and io_thread is not threading.current_thread():
      io_thread.join(timeout=0.5)
    lock_fd = self._sender_lock_fd
    self._sender_lock_fd = None
    if lock_fd is not None:
      try:
        fcntl.flock(lock_fd, fcntl.LOCK_UN)
      except OSError:
        pass
      try:
        os.close(lock_fd)
      except OSError:
        pass
    if self.enabled:
      for path in (self._shm_path, self._assist_shm_path):
        if path is not None:
          try:
            path.unlink(missing_ok=True)
          except OSError:
            pass

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
      # Push moving targets to front so Y/Vy + age arrive before flags unlocks emission
      for task in ((slot, 106, 2), (slot, 2, 6), (slot, 22, 6)):
        if task not in self._init_queued:
          self._init_queued.add(task)
          self._init_queue.appendleft(task)
    else:
      # Queue lateral read first; only near/in-path stationary targets enqueue age+flags to protect moving bandwidth
      task = (slot, 22, 6)
      if task not in self._init_queued:
        self._init_queued.add(task)
        self._init_queue.append(task)

  @staticmethod
  def _empty_wayb_state() -> dict:
    return {
      "t1_base": hex(WAYB_DEFAULT_T1_BASE),
      "out18_base": hex(WAYB_DEFAULT_OUT18_BASE),
      "p_base": hex(WAYB_DEFAULT_P_BASE),
      "p0_b2_b5": None,
      "p0_valid": None,
      "p0_t1_slot": None,
      "p0_b8": None,
      "p0_slot_queried": None,
      "t1_x_raw": None,
      "t1_u26": None,
      "t1_vx28_raw": None,
      "t1_vx40_raw": None,
      "t1_vy42_raw": None,
      "t1_flags": None,
      "t1_valid": None,
      "out18_cam_id": None,
      "out18_cam_flags": None,
      "slot_source": None,       # "lead_scan" | "p0" | "idle0"
      "lead_slot": None,         # T1/OUT18 slot locked via out18_scan matching current in-lane op_lead tid
      "lead_cam_id": None,       # matched op_lead tid
      "out18_scan": [],          # [[slot, cam_id, cam_flags], ...] from current 1 Hz round (reassigned, never mutated in place)
      "last_err": None,
      "last_err_addr": None,     # hex(addr) of most recent WAYB 0xFE error
      "updated_ns": 0,
    }

  def _record_read_locked(self, kind: int, now_ns: int, dur_ns: int, a, b, c, d) -> None:
    if not XCP_READS_LOG:
      return
    if len(self._reads_buf) >= READS_BUF_MAX:
      self._reads_dropped += 1
      return
    self._reads_buf.append((kind, int(now_ns), int(dur_ns), a, b, c, d))

  def _maybe_flush_reads(self, now_ns: int) -> None:
    if not XCP_READS_LOG:
      return
    with self._lock:
      if not self._reads_buf or now_ns < self._next_reads_flush_ns:
        return
      if now_ns - self._next_reads_flush_ns > READS_FLUSH_NS:
        self._next_reads_flush_ns = now_ns + READS_FLUSH_NS
      else:
        self._next_reads_flush_ns += READS_FLUSH_NS
      buf = self._reads_buf
      self._reads_buf = []
      dropped = int(self._reads_dropped)
    base_ns = buf[0][1]
    records = []
    for kind, t_ns, dur_ns, a, b, c, d in buf:
      dt_us = (t_ns - base_ns) // 1000
      records.append([kind, dt_us, dur_ns // 1000, a, b, c, d])
    cloudlog.event("xcp_reads", base_ns=base_ns, records=records, dropped=dropped, n=len(records), debug=True)

  def _clear_session_locked(self, target_state: str = "NEED_CONNECT") -> None:
    self.state = target_state
    self.in_flight = None
    self._inflight_tail.clear()
    self._init_queue.clear()
    self._init_queued.clear()
    self._wayb_queue.clear()
    self._wayb_last_round_ns = 0
    self._wayb_lead_slot = None
    self._wayb_lead_cam_id = None
    self.seed_bytes = None
    self.consecutive_f4_timeouts = 0
    self._timeout_drain_until_ns = 0
    self._op_lead_slot = None
    self._prev_op_slot_by_tid.clear()
    self._assist_shm_dirty = False

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

    kin_changed = False
    if offset == 16 and size == 6:
      new_x = int.from_bytes(data[0:2], "big", signed=True)
      new_vx = int.from_bytes(data[2:4], "big", signed=True)
      new_ax = int.from_bytes(data[4:6], "big", signed=True)
      if (
        rec["kin_changed_ns"] == 0
        or new_x != rec["X_raw"]
        or new_vx != rec["Vx_raw"]
        or new_ax != rec["Ax_raw"]
      ):
        kin_changed = True
        rec["kin_changed_ns"] = now_ns
        rec["kin_value_changes"] += 1
      rec["X_raw"] = new_x
      rec["Vx_raw"] = new_vx
      rec["Ax_raw"] = new_ax
      rec["kin_updated_ns"] = now_ns
    elif offset == 22 and size == 6:
      rec["Y_raw"] = int.from_bytes(data[0:2], "big", signed=True)
      rec["raw24"] = int.from_bytes(data[2:4], "big", signed=False)
      rec["Vy_raw"] = int.from_bytes(data[4:6], "big", signed=True)
      rec["lat_updated_ns"] = now_ns
      if slot in self.active_slots and not self._is_moving_target_locked(rec):
        if rec["X_raw"] > 0 and (bool(rec["subflags"] & 0x0400) or abs(rec["Y_raw"]) < STATIONARY_IN_PATH_Y_RAW):
          for follow in ((slot, 2, 6), (slot, 106, 2)):
            if follow not in self._init_queued and (
              (follow[1] == 2 and rec["age_updated_ns"] == 0) or (follow[1] == 106 and rec["flags_updated_ns"] == 0)
            ):
              self._init_queued.add(follow)
              self._init_queue.append(follow)
    elif offset == 2 and size == 6:
      new_age = int.from_bytes(data[0:2], "big", signed=False)
      rec["marker"] = int.from_bytes(data[2:4], "big", signed=False)
      rec["subflags"] = int.from_bytes(data[4:6], "big", signed=False)
      prev_age = rec["prev_age"]
      if slot in self.active_slots and prev_age is not None and new_age < prev_age:
        rec["radar_track_id"] = self._alloc_track_id_locked()
        rec["kin_changed_ns"] = 0
        rec["kin_value_changes"] = 0
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

    if (
      offset == 16
      and size == 6
      and kin_changed
      and slot in self.active_slots
      and self._assist_shm_path is not None
      and self._assist_fast_publish_enabled
    ):
      if (self._op_lead_slot is not None and slot == self._op_lead_slot) or slot in self._select_priority_slots_locked()[:1]:
        self._assist_shm_dirty = True

  def _process_dto_locked(self, data: bytes, now_ns: int) -> None:
    if not data:
      return
    pid = data[0]
    if pid not in (0xFF, 0xFE):
      return
    if self.in_flight is None:
      if now_ns < self._timeout_drain_until_ns:
        self.drain_discard_count += 1
      return
    cmd_type, slot, offset, size, sent_ns = self.in_flight
    # Pipelining (depth > 1): untagged SHORT_UPLOAD replies are matched to outstanding requests in
    # strict send (FIFO) order. A positive reply whose length differs from the head's expected size means we
    # have lost sync (a dropped/reordered reply), so flush the pending tail, resync to single, and
    # take the normal post-timeout quiet drain rather than mis-decode. No-op at depth 1 (guarded),
    # which keeps HEAD's "short reply -> _handle no-op" behavior byte-identical.
    if self._pipeline_depth > 1 and pid == 0xFF and cmd_type in ("F4", "WAYB") and len(data) != 1 + size:
      self.in_flight = None
      self._inflight_tail.clear()
      self._timeout_drain_until_ns = now_ns + F4_TIMEOUT_DRAIN_NS
      return
    self.in_flight = None
    if pid == 0xFE:
      if cmd_type == "WAYB":
        self._wayb_state["last_err"] = data.hex()
        self._wayb_state["last_err_addr"] = hex(offset)
        self._wayb_state["updated_ns"] = now_ns
        self._wayb_queue.clear()
        self._inflight_tail.clear()
        return
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
      if len(data) >= 1 + size:
        self._record_read_locked(0, now_ns, max(0, now_ns - sent_ns), slot, offset, size, bytes(data[1 : 1 + size]).hex())
      self._handle_f4_reply_locked(slot, offset, size, data, now_ns, sent_ns)
    elif cmd_type == "WAYB":
      if len(data) >= 1 + size:
        self._record_read_locked(1, now_ns, max(0, now_ns - sent_ns), slot, offset, size, bytes(data[1 : 1 + size]).hex())
      self._handle_wayb_reply_locked(slot, size, data, now_ns)
    # Pipelining: promote the next outstanding request to head. No-op at depth 1 (tail is empty).
    if self.in_flight is None and self._inflight_tail:
      self.in_flight = self._inflight_tail.popleft()

  def _current_op_lead_tid_locked(self, now_ns: int) -> int | None:
    if now_ns - self._op_tracks_ns > OP_TRACKS_MAX_AGE_NS or not self._op_tracks:
      return None
    in_path = [o for o in self._op_tracks if o["dRel"] > 0.0 and abs(o["yRel"]) < 2.0]
    if not in_path:
      return None
    return int(min(in_path, key=lambda o: o["dRel"])["tid"])

  def _handle_wayb_reply_locked(self, tag: str, size: int, payload: bytes, now_ns: int) -> None:
    if len(payload) < 1 + size:
      return
    data = payload[1 : 1 + size]
    wb = self._wayb_state
    if tag == "p0" and size == 7:
      wb["p0_b2_b5"] = int.from_bytes(data[0:4], "big")
      p0_valid = int(data[4])
      p0_t1_slot = int(data[5])
      wb["p0_valid"] = p0_valid
      wb["p0_t1_slot"] = p0_t1_slot
      wb["p0_b8"] = int(data[6])
      cur_lead_tid = self._current_op_lead_tid_locked(now_ns)
      if cur_lead_tid is None or cur_lead_tid != self._wayb_lead_cam_id:
        self._wayb_lead_slot = None
        self._wayb_lead_cam_id = None
      wb["lead_slot"] = self._wayb_lead_slot
      wb["lead_cam_id"] = self._wayb_lead_cam_id
      if self._wayb_lead_slot is not None and 0 <= self._wayb_lead_slot < WAYB_T1_NUM_SLOTS:
        slot = self._wayb_lead_slot
        slot_source = "lead_scan"
      elif p0_valid != 0 and 0 <= p0_t1_slot < WAYB_T1_NUM_SLOTS:
        slot = p0_t1_slot
        slot_source = "p0"
      else:
        slot = 0
        slot_source = "idle0"
      wb["p0_slot_queried"] = slot
      wb["slot_source"] = slot_source
      wb["out18_scan"] = []
      wb["updated_ns"] = now_ns
      t1_slot_base = self._wayb_t1_base + WAYB_T1_STRIDE * slot
      out18_slot_base = self._wayb_out18_base + WAYB_OUT18_STRIDE * slot
      # Queue verified core reads first, then OUT18 lead scan last so any 0xFE on scan cannot drop core reads.
      for item in (
        ("t1_kin", t1_slot_base + 24, 6),
        ("t1_vx", t1_slot_base + 40, 4),
        ("t1_meta", t1_slot_base + 86, 6),
        ("out18", out18_slot_base + 16, 2),
      ):
        self._wayb_queue.append(item)
      for _ in range(WAYB_OUT18_SCAN_PER_ROUND):
        s = self._wayb_out18_cursor
        self._wayb_out18_cursor = (self._wayb_out18_cursor + 1) % WAYB_T1_NUM_SLOTS
        self._wayb_queue.append((f"o18s:{s}", self._wayb_out18_base + WAYB_OUT18_STRIDE * s + 16, 2))
    elif tag == "t1_kin" and size == 6:
      wb["t1_x_raw"] = int.from_bytes(data[0:2], "big", signed=True)
      wb["t1_u26"] = int.from_bytes(data[2:4], "big", signed=True)
      wb["t1_vx28_raw"] = int.from_bytes(data[4:6], "big", signed=True)
      wb["updated_ns"] = now_ns
    elif tag == "t1_vx" and size == 4:
      wb["t1_vx40_raw"] = int.from_bytes(data[0:2], "big", signed=True)
      wb["t1_vy42_raw"] = int.from_bytes(data[2:4], "big", signed=True)
      wb["updated_ns"] = now_ns
    elif tag == "t1_meta" and size == 6:
      wb["t1_flags"] = int.from_bytes(data[0:2], "big", signed=False)
      wb["t1_valid"] = int(data[5])
      wb["updated_ns"] = now_ns
    elif tag == "out18" and size == 2:
      cam_id = int(data[0])
      cam_flags = int(data[1])
      wb["out18_cam_id"] = cam_id
      wb["out18_cam_flags"] = cam_flags
      cur_lead_tid = self._current_op_lead_tid_locked(now_ns)
      if wb.get("slot_source") == "lead_scan":
        if cur_lead_tid is None or cam_id != cur_lead_tid:
          self._wayb_lead_slot = None
          self._wayb_lead_cam_id = None
          wb["lead_slot"] = None
          wb["lead_cam_id"] = None
      elif cur_lead_tid is not None and cam_id != 0 and cam_id == cur_lead_tid:
        q_slot = wb.get("p0_slot_queried")
        if isinstance(q_slot, int) and 0 <= q_slot < WAYB_T1_NUM_SLOTS:
          self._wayb_lead_slot = q_slot
          self._wayb_lead_cam_id = cam_id
          wb["lead_slot"] = q_slot
          wb["lead_cam_id"] = cam_id
      wb["updated_ns"] = now_ns
    elif tag.startswith("o18s:") and size == 2:
      scan_slot = int(tag.split(":", 1)[1])
      cam_id = int(data[0])
      cam_flags = int(data[1])
      wb["out18_scan"] = [*wb["out18_scan"], [scan_slot, cam_id, cam_flags]]
      cur_lead_tid = self._current_op_lead_tid_locked(now_ns)
      if cur_lead_tid is not None and cam_id != 0 and cam_id == cur_lead_tid:
        self._wayb_lead_slot = scan_slot
        self._wayb_lead_cam_id = cam_id
        wb["lead_slot"] = scan_slot
        wb["lead_cam_id"] = cam_id
      wb["updated_ns"] = now_ns

  def _is_near_cutin_candidate_locked(self, slot: int) -> bool:
    if slot not in self._op_matched_slots:
      return False
    rec = self.slot_cache[slot]
    return (
      rec["lat_updated_ns"] > 0
      and 0 < rec["X_raw"] < CUTIN_MAX_X_RAW
      and abs(rec["Y_raw"]) <= CUTIN_MAX_ABS_Y_RAW
    )

  def _is_cutin_band_locked(self, rec: dict) -> bool:
    if rec["lat_updated_ns"] == 0 or rec["kin_updated_ns"] == 0:
      return False
    d = rec["X_raw"] / 128.0
    ay = abs(rec["Y_raw"]) / 128.0
    return (
      0 < d < CUTIN_PROTECT_MAX_X_M
      and CUTIN_PROTECT_MIN_ABS_Y_M < ay < CUTIN_PROTECT_MAX_ABS_Y_M
      and self._is_moving_target_locked(rec)
    )

  def _cutin_protect_slots_locked(self, prio_slots: list[int]) -> list[int]:
    """Top CUTIN_PROTECT_N band targets by time-to-path, then the prio_slots[1] near cut-in candidate if not included."""
    lead = prio_slots[0] if prio_slots else -1
    cands = []
    for s, rec in self.active_slots.items():
      if s == lead or not self._is_cutin_band_locked(rec):
        continue
      if rec["flags_updated_ns"] == 0 or rec["age_updated_ns"] == 0:
        continue  # newborn: let its init reads finish before it is protected
      y = rec["Y_raw"] / 128.0
      toward = max(-math.copysign(1.0, y) * rec["Vy_raw"] / 256.0, 0.1)
      cands.append((abs(y) / toward, rec["X_raw"], s))
    cands.sort()
    top = [s for _, _, s in cands[:CUTIN_PROTECT_N]]
    if len(prio_slots) > 1 and prio_slots[1] not in top and self._is_near_cutin_candidate_locked(prio_slots[1]):
      top.append(prio_slots[1])
    return top

  def _primary_lead_urgent_locked(self, now_ns: int) -> bool:
    prio_slots = self._select_priority_slots_locked()
    if not prio_slots:
      return False
    rec = self.slot_cache[prio_slots[0]]
    return bool(
      now_ns - rec["kin_updated_ns"] >= LEAD_KIN_INTERVAL_NS
      or rec["age_updated_ns"] == 0
      or (rec["age"] < 3 and now_ns - rec["age_updated_ns"] >= NEWBORN_AGE_REFRESH_NS)
      or rec["flags_updated_ns"] == 0
    )

  def _wayb_pending_locked(self, now_ns: int) -> bool:
    return WAYB_PROBE_ENABLED and self._wayb_probe_enabled and (
      bool(self._wayb_queue)
      or self._wayb_last_round_ns == 0
      or now_ns - self._wayb_last_round_ns >= WAYB_PROBE_INTERVAL_NS
    )

  def _cutin_wayb_yield_task_locked(self, now_ns: int) -> tuple[int, int, int] | None:
    prio_slots = self._select_priority_slots_locked()
    if len(prio_slots) > 1 and self._is_near_cutin_candidate_locked(prio_slots[1]):
      s = prio_slots[1]
      if now_ns - self.slot_cache[s]["kin_updated_ns"] >= CUTIN_WAYB_YIELD_NS:
        return (s, 16, 6)
    return None

  def _next_wayb_task_locked(self, now_ns: int, exclude: frozenset = frozenset()) -> tuple[str, int, int] | None:
    # exclude: start addresses of outstanding requests; a blocked task stays queued (not consumed).
    if not WAYB_PROBE_ENABLED or not self._wayb_probe_enabled:
      return None
    if self._wayb_queue:
      if self._wayb_queue[0][1] in exclude:
        return None
      return self._wayb_queue.popleft()
    if self._wayb_last_round_ns == 0 or now_ns - self._wayb_last_round_ns >= WAYB_PROBE_INTERVAL_NS:
      if self._wayb_p_base + 2 in exclude:
        return None
      self._wayb_last_round_ns = now_ns
      return ("p0", self._wayb_p_base + 2, 7)
    return None

  def _next_inactive_probe_locked(self, exclude: frozenset = frozenset()) -> tuple[int, int, int] | None:
    for _ in range(NUM_SLOTS):
      s = self.scan_cursor
      self.scan_cursor = (self.scan_cursor + 1) % NUM_SLOTS
      if s not in self.active_slots and REC0 + STRIDE * s + 16 not in exclude:
        return (s, 16, 6)
    return None

  def _select_priority_slots_locked(self) -> list[int]:
    # Put matched in-path op_lead first; never let off-path _op_matched_slots hijack prio[0]
    # from an in-path xcp_fallback lead. When prio[0] is taken, rank near CAN-matched cut-in
    # candidates (0 < d < 40 m, |y| <= 4.0 m) first for prio[1] ahead of far unmatched slots.
    # Do NOT gate on flags & 0x1020: route 00000105 showed 0x1000 (low-conf) is set on 945/1,912
    # real xcp_fallback frames (0xF040) and 0x0020 (ghost) appears on 86 real fallback frames;
    # filtering either blew up xcp_fallback >200 ms from 0.11-0.32% to 7.28-12.66% in offline sim.
    prio: list[int] = []
    if self._op_lead_slot is not None and self._op_lead_slot in self.active_slots:
      prio.append(self._op_lead_slot)

    op_matched_set = set(self._op_matched_slots)

    def _is_in_path(rec: dict) -> bool:
      return rec["X_raw"] > 0 and (
        (rec["lat_updated_ns"] > 0 and abs(rec["Y_raw"]) < IN_PATH_Y_RAW)
        or (rec["lat_updated_ns"] == 0 and bool(rec["subflags"] & 0x0400))
      )

    if not prio and self.active_slots:
      def _fallback_key(item: tuple[int, dict]) -> tuple[int, int, int, int]:
        s, rec = item
        x_raw = rec["X_raw"]
        y_raw = rec["Y_raw"]
        vy_raw = rec["Vy_raw"]
        forward = x_raw > 0
        in_path = _is_in_path(rec)
        is_moving = self._is_moving_target_locked(rec)
        is_op = s in op_matched_set
        near_cutin_bonus = 0
        if in_path and is_moving:
          tier = 0
        elif in_path:
          tier = 1
        elif is_op and forward:
          tier = 2
          lat_ok = rec["lat_updated_ns"] > 0
          near_cutin = lat_ok and x_raw < CUTIN_MAX_X_RAW and abs(y_raw) <= CUTIN_MAX_ABS_Y_RAW
          moving_lat = lat_ok and abs(vy_raw) > CUTIN_VY_THRESH_RAW
          near_cutin_bonus = 0 if (near_cutin and moving_lat) else (1 if near_cutin else 2)
        elif forward and is_moving:
          tier = 3
        else:
          tier = 4
        return (tier, near_cutin_bonus, x_raw if forward else abs(x_raw) + 100_000, s)

      prio.append(min(self.active_slots.items(), key=_fallback_key)[0])

    rem_matched = [s for s in self._op_matched_slots if s in self.active_slots and s not in prio]
    if len(rem_matched) > 1:
      def _matched_key(idx_s: tuple[int, int]) -> tuple[int, int, int, int, int]:
        idx, s = idx_s
        rec = self.slot_cache[s]
        x_raw = rec["X_raw"]
        y_raw = rec["Y_raw"]
        vy_raw = rec["Vy_raw"]
        lat_ok = rec["lat_updated_ns"] > 0
        near_cutin = lat_ok and 0 < x_raw < CUTIN_MAX_X_RAW and abs(y_raw) <= CUTIN_MAX_ABS_Y_RAW
        moving_lat = lat_ok and abs(vy_raw) > CUTIN_VY_THRESH_RAW
        ghost_pen = 1 if bool(rec["flags"] & 0x0020) else 0
        return (
          0 if near_cutin else 1,
          0 if (near_cutin and moving_lat) else 1,
          ghost_pen,
          x_raw if x_raw > 0 else abs(x_raw) + 100_000,
          idx,
        )

      rem_matched = [s for _, s in sorted(enumerate(rem_matched), key=_matched_key)]

    for s in rem_matched:
      prio.append(s)
      if len(prio) >= LEAD_PRIORITY_COUNT:
        return prio

    if len(prio) < LEAD_PRIORITY_COUNT:
      def _rem_key(item: tuple[int, dict]) -> tuple[int, int, int]:
        s, rec = item
        x_raw = rec["X_raw"]
        forward = x_raw > 0
        in_path = _is_in_path(rec)
        is_moving = self._is_moving_target_locked(rec)
        tier = 0 if (in_path and is_moving) else (1 if in_path else (2 if (forward and is_moving) else 3))
        return (tier, x_raw if forward else abs(x_raw) + 100_000, s)

      for s, _ in sorted(self.active_slots.items(), key=_rem_key):
        if s not in prio:
          prio.append(s)
          if len(prio) >= LEAD_PRIORITY_COUNT:
            break
    return prio

  def _init_oldest_wait_locked(self, now_ns: int) -> int:
    live = [f for f in self._init_queue if f in self._init_queued and f[0] in self.active_slots]
    liveset = set(live)
    for k in [k for k in self._init_stamp if k not in liveset]:
      del self._init_stamp[k]
    for f in live:
      self._init_stamp.setdefault(f, now_ns)
    return (now_ns - min(self._init_stamp[f] for f in live)) if live else 0

  def _pop_init_task_locked(self, *, allow_stationary: bool, exclude: frozenset = frozenset()) -> tuple[int, int, int] | None:
    while self._init_queue:
      front = self._init_queue[0]
      if front not in self._init_queued or front[0] not in self.active_slots:
        self._init_queue.popleft()
        self._init_queued.discard(front)
        continue
      if not allow_stationary and not self._is_moving_target_locked(self.slot_cache[front[0]]):
        return None
      if exclude and REC0 + STRIDE * front[0] + front[1] in exclude:
        return None  # blocked: leave it queued for a later call
      self._init_queue.popleft()
      self._init_queued.discard(front)
      self._init_stamp.pop(front, None)
      if allow_stationary:
        self._stationary_init_skip_count = 0
      return front
    return None

  def _has_pending_init_locked(self) -> bool:
    while self._init_queue:
      front = self._init_queue[0]
      if front in self._init_queued and front[0] in self.active_slots:
        return True
      self._init_queue.popleft()
      self._init_queued.discard(front)
    return False

  def _stale_group_task_locked(
    self,
    group: list[int],
    now_ns: int,
    max_stale_ns: int,
    meta_stale_ns: int,
    lat_stale_ns: int | None = None,
    *,
    fast_newborn_age: bool = False,
    exclude: frozenset = frozenset(),
  ) -> tuple[int, int, int] | None:
    if not group:
      return None
    best_ratio = 0.0
    best_task: tuple[int, int, int] | None = None

    def _consider(ratio: float, task: tuple[int, int, int]) -> None:
      nonlocal best_ratio, best_task
      if exclude and REC0 + STRIDE * task[0] + task[1] in exclude:
        return
      if ratio > best_ratio:
        best_ratio = ratio
        best_task = task

    stale_kin_slot = min(group, key=lambda s: (self.slot_cache[s]["kin_updated_ns"], s))
    kin_age = now_ns - self.slot_cache[stale_kin_slot]["kin_updated_ns"]
    if kin_age >= max_stale_ns:
      _consider(kin_age / max_stale_ns, (stale_kin_slot, 16, 6))

    if lat_stale_ns is not None:
      stale_lat_slot = min(group, key=lambda s: (self.slot_cache[s]["lat_updated_ns"], s))
      lat_ts = self.slot_cache[stale_lat_slot]["lat_updated_ns"]
      if lat_ts == 0:
        _consider(2_000_000.0, (stale_lat_slot, 22, 6))
      else:
        lat_age = now_ns - lat_ts
        if lat_age >= lat_stale_ns:
          _consider(lat_age / lat_stale_ns, (stale_lat_slot, 22, 6))

    if fast_newborn_age:
      newborn = [
        s for s in group
        if self.slot_cache[s]["age"] < 3 and (
          self._is_moving_target_locked(self.slot_cache[s])
          or (self.slot_cache[s]["lat_updated_ns"] > 0 and abs(self.slot_cache[s]["Y_raw"]) < STATIONARY_IN_PATH_Y_RAW)
        )
      ]
      if newborn:
        stale_newborn = min(newborn, key=lambda s: (self.slot_cache[s]["age_updated_ns"], s))
        age_ts = self.slot_cache[stale_newborn]["age_updated_ns"]
        if age_ts == 0:
          _consider(1_000_000.0, (stale_newborn, 2, 6))
        else:
          nb_age = now_ns - age_ts
          if nb_age >= NEWBORN_AGE_REFRESH_NS:
            _consider(nb_age / NEWBORN_AGE_REFRESH_NS, (stale_newborn, 2, 6))

    if best_task is not None:
      return best_task

    stale_meta_slot = min(
      group,
      key=lambda s: (min(self.slot_cache[s]["age_updated_ns"], self.slot_cache[s]["flags_updated_ns"]), s),
    )
    rec = self.slot_cache[stale_meta_slot]
    meta_age = now_ns - min(rec["age_updated_ns"], rec["flags_updated_ns"])
    if meta_age >= meta_stale_ns:
      task = (stale_meta_slot, 2, 6) if rec["age_updated_ns"] <= rec["flags_updated_ns"] else (stale_meta_slot, 106, 2)
      if not exclude or REC0 + STRIDE * task[0] + task[1] not in exclude:
        return task

    return None

  def _round_robin_task_locked(self, others: list[int], now_ns: int) -> tuple[int, int, int] | None:
    if not others:
      return None
    oldest_kin_slot = min(others, key=lambda s: (self.slot_cache[s]["kin_updated_ns"], s))
    rec = self.slot_cache[oldest_kin_slot]
    kin_ts = rec["kin_updated_ns"]
    if rec["flags_updated_ns"] == 0:
      return (oldest_kin_slot, 106, 2)
    if rec["age_updated_ns"] == 0 or (rec["age"] < 3 and now_ns - rec["age_updated_ns"] >= NEWBORN_AGE_REFRESH_NS):
      return (oldest_kin_slot, 2, 6)
    for field_key, interval_ns, task in (
      ("lat_updated_ns", OTHER_LAT_INTERVAL_NS, (oldest_kin_slot, 22, 6)),
      ("flags_updated_ns", OTHER_META_INTERVAL_NS, (oldest_kin_slot, 106, 2)),
      ("age_updated_ns", OTHER_META_INTERVAL_NS, (oldest_kin_slot, 2, 6)),
      ("qual_updated_ns", OTHER_QUAL_INTERVAL_NS, (oldest_kin_slot, 127, 2)),
    ):
      ts = rec[field_key]
      if ts == 0 or (now_ns - ts >= interval_ns and ts < kin_ts):
        return task
    return (oldest_kin_slot, 16, 6)

  def _secondary_lead_task_locked(
    self,
    slot: int,
    now_ns: int,
    *,
    include_meta: bool,
  ) -> tuple[int, int, int] | None:
    rec = self.slot_cache[slot]
    if now_ns - rec["kin_updated_ns"] >= LEAD2_KIN_INTERVAL_NS:
      return (slot, 16, 6)
    if rec["age_updated_ns"] == 0 or (rec["age"] < 3 and now_ns - rec["age_updated_ns"] >= NEWBORN_AGE_REFRESH_NS):
      return (slot, 2, 6)
    if rec["flags_updated_ns"] == 0:
      return (slot, 106, 2)
    if rec["lat_updated_ns"] == 0 or now_ns - rec["lat_updated_ns"] >= LEAD2_LAT_INTERVAL_NS:
      return (slot, 22, 6)
    if not include_meta:
      return None
    if now_ns - max(rec["age_updated_ns"], rec["flags_updated_ns"]) >= LEAD2_META_INTERVAL_NS:
      return (slot, 2, 6) if rec["age_updated_ns"] <= rec["flags_updated_ns"] else (slot, 106, 2)
    if rec["qual_updated_ns"] == 0 or now_ns - rec["qual_updated_ns"] >= OTHER_QUAL_INTERVAL_NS:
      return (slot, 127, 2)
    return None

  def _next_read_task_locked(self, now_ns: int, exclude: frozenset = frozenset()) -> tuple[int, int, int] | None:
    # exclude: start addresses of outstanding requests; blocked tasks are skipped for the next-best one.
    # An empty set leaves the selection unchanged (depth 1).
    def blocked(t: tuple[int, int, int]) -> bool:
      return bool(exclude) and REC0 + STRIDE * t[0] + t[1] in exclude

    self._read_seq += 1

    prio_slots = self._select_priority_slots_locked()
    prio_set = set(prio_slots)

    # Primary lead (prio_slots[0]) urgent tasks stay strictly at the top
    if prio_slots:
      s = prio_slots[0]
      rec = self.slot_cache[s]
      if now_ns - rec["kin_updated_ns"] >= LEAD_KIN_INTERVAL_NS:
        # The radar refreshes the lead about every 90 ms: after a read that returned a changed value, skip lead kin
        # reads until LEAD_KIN_HOLD_NS has passed, then poll every LEAD_KIN_INTERVAL_NS as before.
        # A lead whose value has not changed for LEAD_KIN_HOLD_MAX_NS is polled plainly.
        since_change = now_ns - rec["kin_changed_ns"]
        if (rec["kin_changed_ns"] == 0 or since_change >= LEAD_KIN_HOLD_NS or since_change >= LEAD_KIN_HOLD_MAX_NS) \
            and not blocked((s, 16, 6)):
          return (s, 16, 6)
      if (rec["age_updated_ns"] == 0 or (rec["age"] < 3 and now_ns - rec["age_updated_ns"] >= NEWBORN_AGE_REFRESH_NS)) \
          and not blocked((s, 2, 6)):
        return (s, 2, 6)
      if rec["flags_updated_ns"] == 0 and not blocked((s, 106, 2)):
        return (s, 106, 2)

    # Remaining primary lead (prio_slots[0]) cadence stays above the cut-in protect step and the inactive probing / init queue
    if prio_slots:
      s = prio_slots[0]
      rec = self.slot_cache[s]
      lead_task = None
      if now_ns - max(rec["age_updated_ns"], rec["flags_updated_ns"]) >= LEAD_META_INTERVAL_NS:
        lead_task = (s, 2, 6) if rec["age_updated_ns"] <= rec["flags_updated_ns"] else (s, 106, 2)
      elif now_ns - rec["lat_updated_ns"] >= LEAD_LAT_INTERVAL_NS:
        lead_task = (s, 22, 6)
      elif now_ns - rec["qual_updated_ns"] >= LEAD_QUAL_INTERVAL_NS:
        lead_task = (s, 127, 2)
      if lead_task is not None and not blocked(lead_task):
        self._protect_hist.append(0)  # window counts every scheduler call that gets this far
        return lead_task

    # Bound active near cut-in candidate (prio_slots[1]) and band target kinematic/lateral age ahead of inactive probing
    # and new-slot init bursts so multi-slot discovery cannot push cut-in >400 ms. The protected set (up to
    # CUTIN_PROTECT_N band targets plus that prio_slots[1] candidate) is held to CUTIN_PROTECT_KIN_NS kin age and
    # CUTIN_PROTECT_LAT_NS lat age; the largest staleness ratio is read first. The step may take at most
    # CUTIN_PROTECT_CAP of the last CUTIN_PROTECT_WINDOW scheduler calls; beyond that it falls through to the normal order
    # so other near-moving slots, discovery and standing traffic keep a share at low read rates.
    protect_read = None
    guard_hold = self._init_oldest_wait_locked(now_ns) > CUTIN_PROTECT_GUARD_NS
    if sum(self._protect_hist) < CUTIN_PROTECT_CAP and not guard_hold:
      best = None
      for s in self._cutin_protect_slots_locked(prio_slots):
        rec = self.slot_cache[s]
        for ratio, off in (
          ((now_ns - rec["kin_updated_ns"]) / CUTIN_PROTECT_KIN_NS, 16),
          ((now_ns - rec["lat_updated_ns"]) / CUTIN_PROTECT_LAT_NS, 22),
        ):
          if ratio >= 1.0 and not blocked((s, off, 6)) and (best is None or (-ratio, s) < best[0]):
            best = ((-ratio, s), (s, off, 6))
      if best is not None:
        protect_read = best[1]
    self._protect_hist.append(1 if protect_read is not None else 0)
    if protect_read is not None:
      return protect_read

    # Adaptive inactive slot discovery (1/10 when tracking a matched lead, 1/3 if no lead or op_tracks lead is unmatched)
    has_unmatched_op_in_path = (
      self._op_lead_slot is None
      and (now_ns - self._op_tracks_ns <= OP_TRACKS_MAX_AGE_NS)
      and any(o["dRel"] > 0.0 and abs(o["yRel"]) < 2.0 for o in self._op_tracks)
    )
    probe_period = (
      INACTIVE_PROBE_PERIOD_NO_LEAD
      if (not prio_slots or has_unmatched_op_in_path)
      else INACTIVE_PROBE_PERIOD_WITH_LEAD
    )
    if (self._read_seq - self._last_probe_seq) >= probe_period and len(self.active_slots) < NUM_SLOTS:
      probe = self._next_inactive_probe_locked(exclude)
      if probe is not None:
        self._last_probe_seq = self._read_seq
        return probe

    # Newly promoted moving slots run immediately ahead of other traffic
    if task := self._pop_init_task_locked(allow_stationary=False, exclude=exclude):
      return task

    lead1 = prio_slots[0] if prio_slots else -1
    others_all = [s for s in sorted(self.active_slots) if s != lead1]
    moving_others = [s for s in others_all if self._is_moving_target_locked(self.slot_cache[s])]
    stationary_others = [s for s in others_all if s not in moving_others]
    near_moving_others = [
      s for s in moving_others
      if self.slot_cache[s]["lat_updated_ns"] == 0 or abs(self.slot_cache[s]["Y_raw"]) <= NEAR_MOVING_Y_RAW
    ]
    far_moving_others = [s for s in moving_others if s not in near_moving_others]

    # Periodic anti-starvation yield so standing traffic ahead still initializes Y/age/flags in busy moving traffic
    if self._has_pending_init_locked():
      if self._stationary_init_skip_count >= STATIONARY_INIT_YIELD_READS or not near_moving_others:
        if task := self._pop_init_task_locked(allow_stationary=True, exclude=exclude):
          return task
      self._stationary_init_skip_count += 1
    else:
      self._stationary_init_skip_count = 0

    # Promote near CAN-matched cut-in candidate in prio_slots[1] ahead of near_moving_others
    # meta/own-250ms-lat checks ONLY when no other adjacent moving slot is stale on kin/lat.
    if len(prio_slots) > 1 and self._is_near_cutin_candidate_locked(prio_slots[1]):
      cutin_slot = prio_slots[1]
      other_near_stale = any(
        self.slot_cache[s]["lat_updated_ns"] == 0
        or (now_ns - self.slot_cache[s]["lat_updated_ns"] >= OTHER_LAT_INTERVAL_NS)
        or (now_ns - self.slot_cache[s]["kin_updated_ns"] >= OTHER_MAX_STALE_NS)
        for s in near_moving_others
        if s != cutin_slot
      )
      if not other_near_stale:
        task = self._secondary_lead_task_locked(cutin_slot, now_ns, include_meta=False)
        if task and not blocked(task):
          return task

    # Near-path / adjacent moving lanes stay fresh on both kinematics and Y/Vy (wide oncoming traffic separated below)
    if task := self._stale_group_task_locked(
      near_moving_others, now_ns, OTHER_MAX_STALE_NS, OTHER_META_MAX_STALE_NS,
      lat_stale_ns=OTHER_LAT_INTERVAL_NS, fast_newborn_age=True, exclude=exclude,
    ):
      return task

    # Secondary priority lead (prio_slots[1]): ~10 Hz kinematics, ~6.7 Hz Y/Vy, slower age/flags/qual
    if len(prio_slots) > 1:
      task = self._secondary_lead_task_locked(prio_slots[1], now_ns, include_meta=True)
      if task and not blocked(task):
        return task

    if task := self._stale_group_task_locked(
      far_moving_others, now_ns, FAR_OTHER_KIN_INTERVAL_NS, FAR_OTHER_META_INTERVAL_NS,
      lat_stale_ns=FAR_OTHER_LAT_INTERVAL_NS, exclude=exclude,
    ):
      return task

    # Near-path stationary newborn slots refresh age >= 3 promptly; bulk roadside clutter stays at low cadence
    if task := self._stale_group_task_locked(
      stationary_others, now_ns, STATIONARY_MAX_STALE_NS, 2 * STATIONARY_MAX_STALE_NS, fast_newborn_age=True,
      exclude=exclude,
    ):
      return task

    for grp in (near_moving_others, far_moving_others):
      task = self._round_robin_task_locked([s for s in grp if s not in prio_set], now_ns)
      if task and not blocked(task):
        return task

    if task := self._pop_init_task_locked(allow_stationary=True, exclude=exclude):
      return task

    # If no active slots need updating right now, sweep inactive slots faster
    if len(self.active_slots) < NUM_SLOTS:
      probe = self._next_inactive_probe_locked(exclude)
      if probe is not None:
        self._last_probe_seq = self._read_seq
        return probe

    if prio_slots and not blocked((prio_slots[0], 16, 6)):
      return (prio_slots[0], 16, 6)
    return None

  def _build_next_cto_locked(self, now_ns: int) -> bytes | None:
    if not self._tx_armed:
      return None
    if now_ns - self._tx_armed_ns < STARTUP_DELAY_NS:
      return None

    if self.in_flight is not None:
      cmd_type, to_slot, to_off, to_size, sent_ns = self.in_flight
      timeout = F4_TIMEOUT_NS if cmd_type in ("F4", "WAYB") else HANDSHAKE_RETRY_NS
      if now_ns - sent_ns < timeout:
        # Head still outstanding and not timed out. At depth 1 this is strictly single-in-flight
        # (byte-identical to HEAD). When pipelining, we may overlap another READY SHORT_UPLOAD while
        # the head's reply slot is in flight, as long as we stay under the configured depth.
        if (self._pipeline_depth > 1 and self.state == "READY"
            and self._outstanding_count_locked() < self._pipeline_depth):
          return self._build_ready_cto_locked(now_ns)
        return None
      # Head timed out: its reply (and, when pipelining, every queued reply behind it) can no longer
      # be matched by send order, so flush the tail and resync to single-in-flight.
      self.in_flight = None
      self._inflight_tail.clear()
      self._record_read_locked(2, now_ns, max(0, now_ns - sent_ns), cmd_type, to_slot, to_off, to_size)
      if cmd_type == "F4":
        self.consecutive_f4_timeouts += 1
        self.f4_timeout_count += 1
        if self.consecutive_f4_timeouts >= 5:
          self._clear_session_locked("NEED_CONNECT")
      elif cmd_type == "WAYB":
        self.consecutive_f4_timeouts += 1
        self.wayb_timeout_count += 1
        self._wayb_queue.clear()
        if self.consecutive_f4_timeouts >= 5:
          self._clear_session_locked("NEED_CONNECT")
      else:
        self.handshake_timeout_count += 1
        self._clear_session_locked("NEED_CONNECT")
      self._timeout_drain_until_ns = now_ns + F4_TIMEOUT_DRAIN_NS
      return None

    if now_ns < self._timeout_drain_until_ns:
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
      return self._build_ready_cto_locked(now_ns)

    return None

  def _outstanding_count_locked(self) -> int:
    """Number of SHORT_UPLOAD requests currently outstanding (head + pipelined tail)."""
    return (1 if self.in_flight is not None else 0) + len(self._inflight_tail)

  def _arm_outstanding_locked(self, entry: tuple) -> None:
    """Record a just-sent request as outstanding: head when idle, else append to the FIFO tail."""
    if self.in_flight is None:
      self.in_flight = entry
    else:
      self._inflight_tail.append(entry)

  # Scheduler fields that the READY task selectors mutate as a side effect of choosing a task.
  # A pipelined (tail) pick that is rejected for targeting an outstanding address is rolled back
  # over exactly these, so the rejection leaves no trace (no lost init task, no cursor drift).
  _SCHED_SCALARS = ("_read_seq", "_last_probe_seq", "scan_cursor", "_stationary_init_skip_count", "_wayb_last_round_ns")
  _SCHED_CONTAINERS = ("_protect_hist", "_init_queue", "_init_queued", "_init_stamp", "_wayb_queue")

  def _sched_snapshot_locked(self) -> tuple[dict, dict]:
    scalars = {k: getattr(self, k) for k in self._SCHED_SCALARS}
    containers = {k: (dict(v) if isinstance(v, dict) else list(v))
                  for k in self._SCHED_CONTAINERS for v in (getattr(self, k),)}
    return scalars, containers

  def _sched_restore_locked(self, snap: tuple[dict, dict]) -> None:
    scalars, containers = snap
    for k, v in scalars.items():
      setattr(self, k, v)
    for k, v in containers.items():
      obj = getattr(self, k)  # restore in place: keep the same objects (deque maxlen, aliases)
      obj.clear()
      if isinstance(obj, dict):
        obj.update(v)
      elif isinstance(obj, set):
        obj.update(v)
      else:
        obj.extend(v)

  @staticmethod
  def _entry_addr(entry: tuple) -> int:
    """Absolute start address of an outstanding request entry (F4: slot/offset, WAYB: raw addr)."""
    if entry[0] == "WAYB":
      return entry[2]
    return REC0 + STRIDE * entry[1] + entry[2]

  def _build_ready_cto_locked(self, now_ns: int) -> bytes | None:
    """Pick and arm the next READY-state SHORT_UPLOAD.

    Idle (in_flight is None): exactly HEAD's selection and arming (depth 1 always takes this path).
    Pipelined (a head is outstanding, depth > 1): the pick must target a different (slot, offset),
    i.e. a different start address, than every outstanding request. SHORT_UPLOAD replies do not echo
    the address, so two outstanding reads of one field cannot be told apart if the radar overwrites
    or drops a queued CTO. A colliding (or empty) pick is rolled back and nothing is sent this call."""
    if self.in_flight is None:
      return self._select_ready_cto_locked(now_ns)
    snap = self._sched_snapshot_locked()
    outstanding = frozenset({self._entry_addr(self.in_flight)} | {self._entry_addr(e) for e in self._inflight_tail})
    picked = self._select_ready_cto_locked(now_ns, arm=False, exclude=outstanding)
    # Backstop: selectors skip outstanding targets, so a collision here should not happen.
    if picked is None or self._entry_addr(picked[0]) in outstanding:
      self._sched_restore_locked(snap)
      if picked is not None:
        self.pipeline_collision_skips += 1
      return None
    entry, pkt = picked
    self._arm_outstanding_locked(entry)
    return pkt

  def _select_ready_cto_locked(self, now_ns: int, arm: bool = True, exclude: frozenset = frozenset()):
    """HEAD's READY task selection. With arm=True arms the request and returns the CTO bytes;
    with arm=False returns (entry, pkt) without arming so the caller can vet the target."""
    def _done(entry: tuple, pkt: bytes):
      if arm:
        self._arm_outstanding_locked(entry)
        return pkt
      return entry, pkt

    task: tuple[int, int, int] | None = None
    # Cheap Way B check first: with WAYB_PROBE_ENABLED False it is always False, so the priority-slot ranking inside
    # _primary_lead_urgent_locked is not run a second time on every read (both checks are side-effect free; the read
    # order is unchanged). Offline sim 2026-11-23, routes 109 and 10b, replay evidence only: read sequence and all
    # metrics identical to HEAD f39f8507dc; scheduler CPU per read on x86 157 -> 119 us (109), 186 -> 112 us (10b),
    # noisy (parallel runs); aarch64 not measured. See longsim_frames/xcp_assist/budget/step1_table_s1.out.
    if self._wayb_pending_locked(now_ns) and not self._primary_lead_urgent_locked(now_ns):
      task = self._cutin_wayb_yield_task_locked(now_ns)
      if task is not None and exclude and REC0 + STRIDE * task[0] + task[1] in exclude:
        task = None
      if task is None:
        wb_task = self._next_wayb_task_locked(now_ns, exclude)
        if wb_task is not None:
          tag, addr, size = wb_task
          pkt = bytes([0xF4, size, 0x00, 0x00]) + addr.to_bytes(4, "big")
          return _done(("WAYB", tag, addr, size, now_ns), pkt)
    if task is None:
      task = self._next_read_task_locked(now_ns, exclude)
    if task is None:
      return None
    slot, offset, size = task
    addr = REC0 + STRIDE * slot + offset
    pkt = bytes([0xF4, size, 0x00, 0x00]) + addr.to_bytes(4, "big")
    return _done(("F4", slot, offset, size, now_ns), pkt)

  def _send_cto(self, pkt: bytes) -> None:
    if self._pm is None:
      return
    from openpilot.selfdrive.pandad import can_list_to_can_capnp

    frame = can_list_to_can_capnp([(XCP_CTO, pkt, XCP_BUS)], msgtype=self._send_topic, valid=True)
    if self._send_lock is not None:
      with self._send_lock:
        self._pm.send(self._send_topic, frame)
    else:
      self._pm.send(self._send_topic, frame)

  def arm_tx(self, now_ns: int | None = None) -> None:
    if not self.enabled:
      return
    with self._lock:
      if not self._tx_armed:
        self._tx_armed = True
        self._tx_armed_ns = int(time.monotonic_ns() if now_ns is None else now_ns)
        self._tx_event.set()

  def _build_ctos_locked(self, now_ns: int) -> list[bytes]:
    """Build one CTO, or at depth > 1 keep building until the pipeline is full or nothing is ready."""
    pkts: list[bytes] = []
    while (pkt := self._build_next_cto_locked(now_ns)) is not None:
      pkts.append(pkt)
      if self._pipeline_depth == 1 or self._outstanding_count_locked() >= self._pipeline_depth:
        break
    return pkts

  def step_io(self, can_strs: list[bytes], now_ns: int, *, v_ego: float | None = None, radar_data=None) -> bool:
    """Single-threaded daemon IO step: reply DTO -> next CTO first, then logs/SHM."""
    if not self.enabled:
      return False
    from openpilot.selfdrive.pandad import can_capnp_to_list

    pkts: list[bytes] = []
    can_list = can_capnp_to_list(can_strs) if can_strs else []
    with self._lock:
      if not self._tx_armed:
        self._tx_armed = True
        self._tx_armed_ns = int(now_ns)
        self._tx_event.set()
      for _, packet in can_list:
        for addr, dat, src in packet:
          if src == XCP_BUS and addr == XCP_DTO:
            self._process_dto_locked(bytes(dat), now_ns)
      pkts = self._build_ctos_locked(now_ns)
    for pkt in pkts:
      self._send_cto(pkt)
    if v_ego is not None:
      with self._lock:
        self.last_v_ego = float(v_ego)
    if radar_data is not None:
      self.observe_radar_tracks(radar_data, now_ns)
    if can_strs:
      self._maybe_publish_assist_shm(now_ns)
    self._maybe_log_20hz(now_ns)
    self._maybe_flush_reads(now_ns)
    return bool(pkts)

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
            pkts = self._build_ctos_locked(now_ns)
        else:
          with self._lock:
            pkts = self._build_ctos_locked(now_ns)
        for pkt in pkts:
          self._send_cto(pkt)
        if can_strs:
          self._maybe_publish_assist_shm(now_ns)
        self._maybe_flush_reads(now_ns)
    except Exception:
      pass
    finally:
      if can_sock is not None:
        try:
          del can_sock
        except Exception:
          pass

  def _match_openpilot_tracks(self, targets: list[dict], op_tracks: list[dict], v_ego: float = 0.0) -> tuple[list[int], int | None]:
    with self._lock:
      if not targets or not op_tracks:
        self._prev_op_slot_by_tid = {}
        return [], None
      prev_slot_by_tid = self._prev_op_slot_by_tid
      candidates: list[tuple[float, int, int, float]] = []
      for ti, t in enumerate(targets):
        if int(t.get("lat_updated_ns", 1)) <= 0:
          continue
        is_stationary = v_ego >= 3.0 and abs(v_ego + t["vRel_mps"]) < 0.7
        for oi, o in enumerate(op_tracks):
          dd = abs(t["dRel_m"] - o["dRel"])
          dy = abs(t["yRel_m"] - o["yRel"])
          dv = abs(t["vRel_mps"] - o["vRel"])
          if is_stationary:
            dd_eff = dd
            max_dd = 4.5
          else:
            allow_trailer = t["dRel_m"] >= o["dRel"] and o["dRel"] >= 25.0 and dy < 1.0
            dd_eff = min(dd, abs((t["dRel_m"] - ARRAY_B_TRAILER_OFFSET_M) - o["dRel"])) if allow_trailer else dd
            r = max(t["dRel_m"], o["dRel"], 0.0)
            max_dd = max(5.5, (0.20 if r > 70.0 else 0.14) * r)
          prev_matched_same = prev_slot_by_tid.get(o["tid"]) == t["slot"]
          closing_lag = o["vRel"] - t["vRel_mps"]
          rail_ok = o["vRel"] <= -11.5 and t["vRel_mps"] <= o["vRel"] + 1.0 and dv <= 25.0
          locked_closing_ok = (
            prev_matched_same
            and not is_stationary
            and (v_ego < 3.0 or (v_ego + t["vRel_mps"]) >= OP_MATCH_LOCKED_MIN_VLEAD_MPS)
            and t["yRel_m"] != 0.0
            and abs(o["yRel"]) <= OP_MATCH_LOCKED_CLOSING_MAX_ABS_Y_M
            and dy < OP_MATCH_LOCKED_CLOSING_MAX_DY_M
            and 0.0 < closing_lag <= OP_MATCH_LOCKED_CLOSING_MAX_DV_MPS
          )
          if dd_eff < max_dd and dy < 2.2 and (dv < 3.5 or rail_ok or locked_closing_ok):
            cost = dd_eff * dd_eff + (2.0 * dy) * (2.0 * dy) + (0.8 * dv) * (0.8 * dv)
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
      new_op_slot_by_tid: dict[int, int] = {}

      for _, ti, oi, dist_m in candidates:
        if ti in used_targets or oi in used_op:
          continue
        used_targets.add(ti)
        used_op.add(oi)
        tid = op_tracks[oi]["tid"]
        targets[ti]["op_track_id"] = tid
        targets[ti]["op_dRel"] = op_tracks[oi]["dRel"]
        targets[ti]["op_match_dist_m"] = round(dist_m, 3)
        slot = targets[ti]["slot"]
        new_op_slot_by_tid[tid] = slot
        if tid == op_lead_tid:
          lead_matched_slots.insert(0, slot)
          op_lead_slot = slot
        else:
          matched_slots.append(slot)
      self._prev_op_slot_by_tid = new_op_slot_by_tid
      return lead_matched_slots + matched_slots, op_lead_slot

  def _collect_targets_locked(self, now_ns: int) -> tuple[float, list[dict], list[dict], int | None]:
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
      kin_age_ns = max(0, now_ns - rec["kin_updated_ns"])
      kin_age_ms = round(kin_age_ns / 1_000_000.0, 2)
      is_fresh_kin = rec["kin_updated_ns"] > 0 and kin_age_ns <= MEASURED_MAX_KIN_AGE_NS
      if dec["stage"] == "measured" and not is_fresh_kin:
        stage = "stale"
        measured = False
        trust_accel = False
      else:
        stage = dec["stage"]
        measured = dec["measured"]
        trust_accel = dec["trust_accel"]
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
        "stage": stage,
        "fw_stage": dec["stage"],
        "measured": measured,
        "conf_band": dec["conf_band"],
        "ghost": dec["ghost"],
        "in_path": dec["in_path"],
        "trust_accel": trust_accel,
        "kin_age_ms": kin_age_ms,
        "op_track_id": None,
        "op_dRel": None,
        "op_match_dist_m": None,
        "kin_updated_ns": rec["kin_updated_ns"],
        "kin_changed_ns": rec["kin_changed_ns"],
        "kin_value_changes": rec["kin_value_changes"],
        "lat_updated_ns": rec["lat_updated_ns"],
        "age_updated_ns": rec["age_updated_ns"],
        "flags_updated_ns": rec["flags_updated_ns"],
        "updated_ns": rec["updated_ns"],
        "last_rtt_ms": round(rec["last_rtt_ns"] / 1_000_000.0, 2),
      })

    matched_slots, op_lead_slot = self._match_openpilot_tracks(targets, op_tracks, v_ego)
    self._op_matched_slots = matched_slots
    self._op_lead_slot = op_lead_slot
    return v_ego, op_tracks, targets, op_lead_slot

  def _maybe_log_20hz(self, now_ns: int) -> None:
    with self._lock:
      if self.state != "READY":
        return
      if now_ns < self._next_log_ns:
        return
      if now_ns - self._next_log_ns > EPOCH_NS:
        self._next_log_ns = now_ns + EPOCH_NS
      else:
        self._next_log_ns += EPOCH_NS
      self.last_log_ns = now_ns
      v_ego, op_tracks, targets, op_lead_slot = self._collect_targets_locked(now_ns)
      wayb_probe = (
        dict(self._wayb_state)
        if self._wayb_probe_enabled and self._wayb_state["updated_ns"] > 0
        else None
      )
      consecutive_f4_timeouts = int(self.consecutive_f4_timeouts)
      f4_timeouts = int(self.f4_timeout_count)
      wayb_timeouts = int(self.wayb_timeout_count)
      handshake_timeouts = int(self.handshake_timeout_count)
      drain_discards = int(self.drain_discard_count)
      if self._assist_shm_path is not None:
        self._last_assist_shm_ns = now_ns
        self._assist_shm_dirty = False

    targets.sort(key=lambda t: (abs(t["dRel_m"]), t["slot"]))
    matched_lead = next((t for t in targets if t["slot"] == op_lead_slot), None) if op_lead_slot is not None else None
    if matched_lead is not None:
      lead = dict(matched_lead)
      lead["lead_source"] = "op_lead"
    else:
      in_path = [t for t in targets if t["dRel_m"] > 0.0 and abs(t["yRel_m"]) < 2.0]
      if v_ego >= 3.0:
        moving_in_path = [t for t in in_path if abs(v_ego + t["vRel_mps"]) >= 1.5]
        fallback = min(moving_in_path or in_path, key=lambda t: t["dRel_m"]) if in_path else None
      else:
        fallback = min(in_path, key=lambda t: t["dRel_m"]) if in_path else None
      if fallback is not None:
        lead = dict(fallback)
        lead["lead_source"] = "xcp_fallback"
      else:
        lead = None
    event_kwargs = dict(
      mono_ns=now_ns,
      v_ego=round(v_ego, 3),
      lead=lead,
      targets=targets,
      op_tracks=op_tracks,
      consecutive_f4_timeouts=consecutive_f4_timeouts,
      f4_timeouts=f4_timeouts,
      wayb_timeouts=wayb_timeouts,
      handshake_timeouts=handshake_timeouts,
      drain_discards=drain_discards,
      debug=True,
    )
    if wayb_probe is not None:
      event_kwargs["wayb_probe"] = wayb_probe
    cloudlog.event("xcp_array_b_20hz", **event_kwargs)
    self._write_ui_snapshot(now_ns, v_ego, targets)

  @staticmethod
  def _build_assist_payload(now_ns: int, targets: list[dict]) -> dict:
    tracks_by_op_id: dict[str, dict] = {}
    shm_targets: list[dict] = []
    for t in targets:
      if (
        t.get("stage") == "measured"
        and not t.get("ghost", False)
        and int(t.get("lat_updated_ns", 1)) > 0
      ):
        entry = {
          "slot": t["slot"],
          "rtid": t["radar_track_id"],
          "age": t["age"],
          "stage": t["stage"],
          "ghost": t["ghost"],
          "flags": t["flags"],
          "dRel_m": t["dRel_m"],
          "yRel_m": t["yRel_m"],
          "vRel_mps": t["vRel_mps"],
          "ax_mps2": t["ax_mps2"],
          "kin_age_ms": t["kin_age_ms"],
          "kin_updated_ns": int(t.get("kin_updated_ns", 0)),
          "kin_changed_ns": int(t.get("kin_changed_ns", 0)),
          "kin_value_changes": int(t.get("kin_value_changes", 0)),
          "lat_valid": True,
          "trust_accel": t["trust_accel"],
          "op_track_id": t.get("op_track_id"),
        }
        shm_targets.append(entry)
        op_tid = t.get("op_track_id")
        if op_tid is not None:
          tracks_by_op_id[str(op_tid)] = {k: v for k, v in entry.items() if k != "op_track_id"}
    return {
      "mono_ns": int(now_ns),
      "boot_offset_ns": int(_boot_minus_mono_ns()),
      "tracks_by_op_id": tracks_by_op_id,
      "targets": shm_targets,
    }

  def _maybe_publish_assist_shm(self, now_ns: int) -> None:
    with self._lock:
      if (
        self._assist_shm_path is None
        or not self._assist_fast_publish_enabled
        or self.state != "READY"
        or not self._assist_shm_dirty
        or (now_ns - self._last_assist_shm_ns) < ASSIST_SHM_MIN_INTERVAL_NS
      ):
        return
      self._assist_shm_dirty = False
      self._last_assist_shm_ns = now_ns
      _, _, targets, _ = self._collect_targets_locked(now_ns)
      assist_path = self._assist_shm_path
    try:
      _write_atomic_json(assist_path, self._build_assist_payload(now_ns, targets))
    except Exception:
      pass

  @staticmethod
  def _ui_points(now_ns: int, v_ego: float, targets: list[dict], cap_ns: int = UI_EXTRAP_CAP_NS) -> list[list[float]]:
    """UI dots as [d, y, age_kin_s, age_lat_s, id]; d and y are extrapolated to now_ns by vRel/vyRel (age capped at
    cap_ns). id is the radar track id of the slot (the slot number when the track id is unknown). Display only."""
    points: list[list[float]] = []
    for t in targets:
      if t.get("lat_updated_ns", 1) <= 0 or t.get("ghost", False) or t.get("age", 99) < 3:
        continue
      age_kin = min(max(now_ns - t.get("kin_updated_ns", now_ns), 0), cap_ns)
      age_lat = min(max(now_ns - t.get("lat_updated_ns", now_ns), 0), cap_ns)
      d_ui = t["dRel_m"] + t.get("vRel_mps", 0.0) * (age_kin / 1e9)
      y_ui = t["yRel_m"] + t.get("vyRel_mps", 0.0) * (age_lat / 1e9)
      if d_ui > 0.0 and (v_ego < 3.0 or abs(v_ego + t["vRel_mps"]) >= 1.5 or abs(y_ui) < 2.2):
        tid = t.get("radar_track_id")
        if tid is None:
          tid = t.get("slot", -1)
        points.append([round(d_ui, 2), round(y_ui, 2), round(age_kin / 1e9, 3), round(age_lat / 1e9, 3), int(tid)])
    return points

  def _write_ui_snapshot(self, now_ns: int, v_ego: float, targets: list[dict]) -> None:
    if self._shm_path is None and self._assist_shm_path is None:
      return
    try:
      if self._shm_path is not None:
        _write_atomic_json(self._shm_path, {"mono_ns": int(now_ns), "points": self._ui_points(now_ns, v_ego, targets)})
      if self._assist_shm_path is not None:
        _write_atomic_json(self._assist_shm_path, self._build_assist_payload(now_ns, targets))
    except Exception:
      pass

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
      if not self._use_io_thread:
        self._maybe_publish_assist_shm(now_ns)
        self._maybe_flush_reads(now_ns)
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
