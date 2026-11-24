"""Tests for the read-only Honda Bosch-A XCP Array B 20 Hz diagnostic logger."""
from __future__ import annotations

from types import SimpleNamespace

from opendbc.car.honda.values import CAR, HondaFlags
import openpilot.starpilot.car.honda.xcp_radar_logger as xcp_mod
from openpilot.starpilot.car.honda.xcp_radar_logger import (
  NUM_SLOTS,
  REC0,
  STRIDE,
  UI_EXTRAP_CAP_NS,
  XCP_BUS,
  XCP_DTO,
  XcpRadarLogger,
  decode_flags,
  key_for_seed,
)


def _write_slot(ram: bytearray, slot: int, cfg: dict) -> None:
  base = slot * STRIDE
  ram[base + 2 : base + 4] = int(cfg["age"]).to_bytes(2, "big")
  ram[base + 4 : base + 6] = (0x2000).to_bytes(2, "big")
  ram[base + 6 : base + 8] = int(cfg["sub"]).to_bytes(2, "big")
  ram[base + 8] = slot
  ram[base + 16 : base + 18] = int(round(cfg["x"] * 128)).to_bytes(2, "big", signed=True)
  ram[base + 18 : base + 20] = int(round(cfg["vx"] * 256)).to_bytes(2, "big", signed=True)
  ram[base + 20 : base + 22] = int(round(cfg["ax"] * 2048)).to_bytes(2, "big", signed=True)
  ram[base + 22 : base + 24] = int(round(cfg["y"] * 128)).to_bytes(2, "big", signed=True)
  ram[base + 24 : base + 26] = (0x0010).to_bytes(2, "big")
  ram[base + 26 : base + 28] = int(round(cfg["vy"] * 256)).to_bytes(2, "big", signed=True)
  ram[base + 106 : base + 108] = int(cfg["flags"]).to_bytes(2, "big")
  ram[base + 127] = cfg["qual"]
  ram[base + 128] = cfg["cls"]


def _capture_events(monkeypatch) -> list[dict]:
  rows: list[dict] = []

  def _fake_event(name: str, **kwargs):
    if name == "xcp_array_b_20hz":
      rows.append(kwargs)

  monkeypatch.setattr(xcp_mod.cloudlog, "event", _fake_event)
  return rows


def test_decode_flags_and_seed_key():
  assert key_for_seed(b"\x12\x34\x56\x78") != bytes(4)
  assert decode_flags(0xF040, 0x0400, 20) == {
    "stage": "measured",
    "measured": True,
    "conf_band": "low",
    "ghost": False,
    "in_path": True,
    "trust_accel": False,
  }
  assert decode_flags(0xE0C0, 0x0400, 20) == {
    "stage": "measured",
    "measured": True,
    "conf_band": "med",
    "ghost": False,
    "in_path": True,
    "trust_accel": True,
  }
  assert decode_flags(0xE840, 0x0400, 10)["conf_band"] == "high"
  assert decode_flags(0xA0C0, 0x0400, 20)["stage"] == "coasting"
  assert decode_flags(0xA0C0, 0x0400, 20)["trust_accel"] is False
  assert decode_flags(0x6080, 0x0000, 0)["stage"] == "birth"
  assert decode_flags(0xE0E0, 0x0400, 20)["ghost"] is True
  assert decode_flags(0xE0C0, 0x0400, 4)["trust_accel"] is False


def test_non_honda_bosch_disabled():
  cp = SimpleNamespace(brand="honda", flags=int(HondaFlags.BOSCH | HondaFlags.BOSCH_RADARLESS), passive=False)
  logger = XcpRadarLogger(cp)
  assert not logger.enabled
  assert logger.next_can_sends(1_000_000_000) == []


def test_20hz_scheduler_all_slots_matching_and_track_id(monkeypatch):
  rows = _capture_events(monkeypatch)
  ram = bytearray(NUM_SLOTS * STRIDE)
  active_defs = {
    2: {"x": 18.0, "vx": -0.5, "ax": -1.2, "y": 0.1, "vy": 0.0, "flags": 0xE0C0, "sub": 0x0400, "age": 40, "qual": 25, "cls": 82},
    7: {"x": 42.0, "vx": 1.0, "ax": 0.1, "y": 0.3, "vy": 0.0, "flags": 0xE840, "sub": 0x0400, "age": 80, "qual": 30, "cls": 88},
    13: {"x": 25.0, "vx": -0.2, "ax": 0.0, "y": -3.6, "vy": 0.0, "flags": 0xE0C0, "sub": 0x0000, "age": 15, "qual": 22, "cls": 55},
    19: {"x": 60.0, "vx": 2.0, "ax": 0.0, "y": 3.5, "vy": 0.0, "flags": 0xF040, "sub": 0x0000, "age": 12, "qual": 10, "cls": 25},
    22: {"x": 31.0, "vx": -1.5, "ax": -0.5, "y": 3.7, "vy": 0.1, "flags": 0xE0C0, "sub": 0x0000, "age": 60, "qual": 28, "cls": 75},
    31: {"x": 11.0, "vx": -3.0, "ax": -2.0, "y": -4.0, "vy": 0.2, "flags": 0xE0C0, "sub": 0x0000, "age": 50, "qual": 24, "cls": 79},
    34: {"x": 75.0, "vx": 0.5, "ax": 0.0, "y": -7.0, "vy": 0.0, "flags": 0xE0C0, "sub": 0x0000, "age": 10, "qual": 18, "cls": 60},
    38: {"x": 52.0, "vx": -0.8, "ax": -0.3, "y": 7.2, "vy": -0.1, "flags": 0xA0C0, "sub": 0x0000, "age": 35, "qual": 26, "cls": 77},
  }
  for s, cfg in active_defs.items():
    _write_slot(ram, s, cfg)

  cp = SimpleNamespace(brand="honda", flags=int(HondaFlags.BOSCH), passive=False)
  logger = XcpRadarLogger(cp)
  assert logger.enabled

  t = 1_000_000_000
  assert logger.next_can_sends(t) == []
  t = logger._tx_armed_ns + 600_000_000

  for _ in range(4):
    sends = logger.next_can_sends(t)
    if sends:
      pkt = bytes(sends[0].dat)
      pid = pkt[0]
      if pid == 0xFF:
        rep = b"\xFF\x05\x01\x08\x00\x08\x01\x01"
      elif pid == 0xFD:
        rep = b"\xFF\x00\x05\x00\x00\x00"
      elif pid == 0xF8:
        rep = b"\xFF\x00\x12\x34\x56\x78"
      elif pid == 0xF7:
        assert pkt[2:6] == key_for_seed(b"\x12\x34\x56\x78")
        rep = b"\xFF\x01"
      else:
        raise AssertionError(f"Unexpected pid {pid:#x}")
      logger.observe_can([(t, [(XCP_DTO, rep, XCP_BUS)])], t + 9_000_000, v_ego=15.0)
    t += 10_000_000

  assert logger.state == "READY"
  rd = SimpleNamespace(points=[
    SimpleNamespace(trackId=17, dRel=15.3, yRel=0.1, vRel=-0.5),
    SimpleNamespace(trackId=29, dRel=22.4, yRel=-3.5, vRel=-0.2),
  ])
  logger.observe_radar_tracks(rd, t)

  probed_slots: set[int] = set()
  lead_kin_reads = 0
  for i in range(400):
    if i == 250:
      active_defs[31]["age"] = 1
      active_defs[31]["x"] = 14.0
      _write_slot(ram, 31, active_defs[31])
    if i % 2 == 0:
      logger.observe_radar_tracks(rd, t)
    sends = logger.next_can_sends(t)
    if sends:
      pkt = bytes(sends[0].dat)
      sz = pkt[1]
      addr = int.from_bytes(pkt[4:8], "big")
      off = addr - REC0
      s = off // STRIDE
      probed_slots.add(s)
      if s == 2 and (off % STRIDE) == 16:
        lead_kin_reads += 1
      rep = b"\xFF" + bytes(ram[off : off + sz])
      logger.observe_can([(t, [(XCP_DTO, rep, XCP_BUS)])], t + 9_000_000, v_ego=15.0)
    t += 10_000_000

  logger.close()

  assert probed_slots == set(range(NUM_SLOTS))
  assert lead_kin_reads >= 65
  assert 75 <= len(rows) <= 85
  assert all(r["debug"] is True for r in rows)
  by_slot = {tgt["slot"]: tgt for tgt in rows[-1]["targets"]}
  assert set(by_slot.keys()) == set(active_defs.keys())
  assert by_slot[2]["op_track_id"] == 17
  assert by_slot[13]["op_track_id"] == 29
  assert rows[-1]["lead"]["slot"] == 2
  assert len({tgt["radar_track_id"] for r in rows for tgt in r["targets"] if tgt["slot"] == 31}) >= 2


def _unlock_logger(logger: XcpRadarLogger, t_start: int, v_ego: float = 15.0) -> int:
  t = t_start
  logger.next_can_sends(t)
  t = logger._tx_armed_ns + 600_000_000
  for _ in range(4):
    sends = logger.next_can_sends(t)
    if sends:
      pkt = bytes(sends[0].dat)
      pid = pkt[0]
      if pid == 0xFF:
        rep = b"\xFF\x05\x01\x08\x00\x08\x01\x01"
      elif pid == 0xFD:
        rep = b"\xFF\x00\x05\x00\x00\x00"
      elif pid == 0xF8:
        rep = b"\xFF\x00\x12\x34\x56\x78"
      elif pid == 0xF7:
        rep = b"\xFF\x01"
      else:
        raise AssertionError(f"Unexpected pid {pid:#x}")
      logger.observe_can([(t, [(XCP_DTO, rep, XCP_BUS)])], t + 19_000_000, v_ego=v_ego)
    t += 20_000_000
  assert logger.state == "READY"
  return t


def test_stationary_clutter_does_not_starve_primary_lead_at_20ms_rtt(monkeypatch):
  rows = _capture_events(monkeypatch)
  ram = bytearray(NUM_SLOTS * STRIDE)
  # Slot 5 is the real moving lead; slot 3 is closer stationary in-path clutter; slots 10..29 are roadside stationary clutter
  _write_slot(ram, 5, {"x": 28.0, "vx": -1.0, "ax": -1.5, "y": 0.2, "vy": 0.0, "flags": 0xE0C0, "sub": 0x0400, "age": 40, "qual": 25, "cls": 82})
  _write_slot(ram, 3, {"x": 12.0, "vx": -15.0, "ax": 0.0, "y": 0.1, "vy": 0.0, "flags": 0xE0C0, "sub": 0x0400, "age": 12, "qual": 20, "cls": 60})
  for s in range(10, 30):
    _write_slot(ram, s, {"x": 15.0 + s, "vx": -15.0, "ax": 0.0, "y": -4.0, "vy": 0.0, "flags": 0xE0C0, "sub": 0x0000, "age": 10, "qual": 15, "cls": 50})

  cp = SimpleNamespace(brand="honda", flags=int(HondaFlags.BOSCH), passive=False)
  logger = XcpRadarLogger(cp)
  t = _unlock_logger(logger, 1_000_000_000, v_ego=15.0)

  # 5 seconds at 20 ms RTT (50 reads/sec)
  lead_kin_times: list[int] = []
  for i in range(250):
    if i >= 25:
      # Once slot 5 is discovered, publish openpilot track shifted by -2.9 m
      logger.observe_radar_tracks(SimpleNamespace(points=[SimpleNamespace(trackId=91, dRel=25.1, yRel=0.2, vRel=-1.0)]), t)
    sends = logger.next_can_sends(t)
    if sends:
      pkt = bytes(sends[0].dat)
      sz = pkt[1]
      off = int.from_bytes(pkt[4:8], "big") - REC0
      s = off // STRIDE
      if s == 5 and (off % STRIDE) == 16:
        lead_kin_times.append(t + 19_000_000)
      rep = b"\xFF" + bytes(ram[off : off + sz])
      logger.observe_can([(t, [(XCP_DTO, rep, XCP_BUS)])], t + 19_000_000, v_ego=15.0)
    t += 20_000_000

  logger.close()
  assert len(lead_kin_times) >= 60
  # Check post-warmup gaps between primary lead kinematics reads stay bounded (< 90 ms) despite 21 clutter slots
  post_warmup_gaps_ms = [(b - a) / 1e6 for a, b in zip(lead_kin_times[5:-1], lead_kin_times[6:], strict=True)]
  assert max(post_warmup_gaps_ms) <= 90.0

  matched_rows = [r for r in rows if r["lead"] is not None and r["lead"]["slot"] == 5]
  assert len(matched_rows) >= 50
  assert all(r["lead"]["op_track_id"] == 91 for r in matched_rows[-20:])


def test_offset_matching_and_moving_lead_fallback(monkeypatch):
  rows = _capture_events(monkeypatch)
  ram = bytearray(NUM_SLOTS * STRIDE)
  # Closer stationary in-path clutter (slot 1) vs distant highway moving lead (slot 4 at x=106.0, op dRel=100.0)
  _write_slot(ram, 1, {"x": 8.0, "vx": -15.0, "ax": 0.0, "y": 0.0, "vy": 0.0, "flags": 0xE0C0, "sub": 0x0400, "age": 20, "qual": 20, "cls": 60})
  _write_slot(ram, 4, {"x": 106.0, "vx": -1.0, "ax": -0.5, "y": -0.3, "vy": 0.0, "flags": 0xE0C0, "sub": 0x0400, "age": 30, "qual": 25, "cls": 85})

  cp = SimpleNamespace(brand="honda", flags=int(HondaFlags.BOSCH), passive=False)
  logger = XcpRadarLogger(cp)
  t = _unlock_logger(logger, 1_000_000_000, v_ego=15.0)

  # First 2.0s without op_tracks (verifies moving-lead fallback skips closer stationary clutter in slot 1)
  for i in range(150):
    if i >= 100:
      logger.observe_radar_tracks(SimpleNamespace(points=[SimpleNamespace(trackId=55, dRel=100.0, yRel=-0.3, vRel=-1.0)]), t)
    sends = logger.next_can_sends(t)
    if sends:
      pkt = bytes(sends[0].dat)
      sz = pkt[1]
      off = int.from_bytes(pkt[4:8], "big") - REC0
      rep = b"\xFF" + bytes(ram[off : off + sz])
      logger.observe_can([(t, [(XCP_DTO, rep, XCP_BUS)])], t + 19_000_000, v_ego=15.0)
    t += 20_000_000

  logger.close()
  # Once both slot 1 and slot 4 have flags, lead should pick slot 4 (both via moving fallback and via op_tracks range-proportional tolerance)
  rows_both = [r for r in rows if {tgt["slot"] for tgt in r["targets"]} >= {1, 4}]
  assert rows_both
  assert all(r["lead"]["slot"] == 4 for r in rows_both)
  assert rows_both[-1]["lead"]["op_track_id"] == 55
  assert rows_both[-1]["lead"]["op_dRel"] == 100.0
  assert rows_both[-1]["lead"]["op_match_dist_m"] <= 6.1


def test_tolerance_stationary_flat_and_moving_far_range():
  cp = SimpleNamespace(brand="honda", flags=int(HondaFlags.BOSCH), passive=False)
  logger = XcpRadarLogger(cp)
  targets = [
    {"slot": 1, "dRel_m": 60.0, "yRel_m": 3.0, "vRel_mps": -15.0, "op_track_id": None, "op_dRel": None},
    {"slot": 2, "dRel_m": 58.0, "yRel_m": -3.0, "vRel_mps": -14.8, "op_track_id": None, "op_dRel": None},
    {"slot": 3, "dRel_m": 92.0, "yRel_m": 0.1, "vRel_mps": -1.0, "op_track_id": None, "op_dRel": None},
  ]
  op_tracks = [
    {"tid": 10, "dRel": 54.0, "yRel": 3.0, "vRel": -15.0},
    {"tid": 11, "dRel": 54.0, "yRel": -3.0, "vRel": -14.8},
    {"tid": 12, "dRel": 78.0, "yRel": 0.1, "vRel": -1.0},
  ]
  matched_slots, op_lead_slot = logger._match_openpilot_tracks(targets, op_tracks, v_ego=15.0)
  assert targets[0]["op_track_id"] is None
  assert targets[1]["op_track_id"] == 11
  assert targets[1]["op_dRel"] == 54.0
  assert targets[2]["op_track_id"] == 12
  assert targets[2]["op_dRel"] == 78.0
  assert op_lead_slot == 3
  assert matched_slots == [3, 2]


def test_ui_shm_snapshot_filters_roadside_stationary_clutter_and_unlinks(tmp_path):
  import json
  shm_path = tmp_path / "xcp_radar_points.json"
  cp = SimpleNamespace(brand="honda", flags=int(HondaFlags.BOSCH), passive=False)
  logger = XcpRadarLogger(cp, shm_path=shm_path)
  targets = [
    {"dRel_m": 20.0, "yRel_m": 3.5, "vRel_mps": -15.0, "vyRel_mps": 0.0, "age": 10, "ghost": False, "lat_updated_ns": 5_000_000_000, "kin_updated_ns": 5_000_000_000},   # stationary off-path clutter -> excluded
    {"dRel_m": 15.0, "yRel_m": 0.5, "vRel_mps": -15.0, "vyRel_mps": 0.0, "age": 10, "ghost": False, "lat_updated_ns": 5_000_000_000, "kin_updated_ns": 5_000_000_000},   # stationary in-path -> included
    {"dRel_m": 45.0, "yRel_m": -3.2, "vRel_mps": -2.0, "vyRel_mps": 1.0, "age": 10, "ghost": False, "lat_updated_ns": 4_900_000_000, "kin_updated_ns": 4_950_000_000},   # moving + time-aligned -> (44.9, -3.1)
    {"dRel_m": 30.0, "yRel_m": 0.0, "vRel_mps": 0.0, "vyRel_mps": 0.0, "age": 10, "ghost": False, "lat_updated_ns": 0, "kin_updated_ns": 5_000_000_000},               # uninitialized Y -> excluded
    {"dRel_m": 35.0, "yRel_m": -3.0, "vRel_mps": 0.0, "vyRel_mps": 0.0, "age": 10, "ghost": True, "lat_updated_ns": 5_000_000_000, "kin_updated_ns": 5_000_000_000},     # ghost -> excluded
    {"dRel_m": 40.0, "yRel_m": -3.0, "vRel_mps": 0.0, "vyRel_mps": 0.0, "age": 2, "ghost": False, "lat_updated_ns": 5_000_000_000, "kin_updated_ns": 5_000_000_000},     # newborn age<3 -> excluded
    {"dRel_m": -4.0, "yRel_m": 0.0, "vRel_mps": 0.0, "vyRel_mps": 0.0, "age": 10, "ghost": False, "lat_updated_ns": 5_000_000_000, "kin_updated_ns": 5_000_000_000},      # behind bumper -> excluded
  ]
  logger._write_ui_snapshot(5_000_000_000, 15.0, targets)
  data = json.loads(shm_path.read_text())
  # [d, y, age_kin_s, age_lat_s, id]; id is -1 here: these targets carry neither a radar track id nor a slot
  assert data == {"mono_ns": 5_000_000_000, "points": [[15.0, 0.5, 0.0, 0.0, -1], [44.9, -3.1, 0.05, 0.1, -1]]}
  logger.close()
  assert not shm_path.exists()


def _ui_target(**kw):
  base = {"dRel_m": 30.0, "yRel_m": 0.0, "vRel_mps": -2.0, "vyRel_mps": 0.5, "age": 10, "ghost": False,
          "lat_updated_ns": 5_000_000_000, "kin_updated_ns": 5_000_000_000, "slot": 7}
  base.update(kw)
  return base


def test_ui_points_extrapolate_up_to_one_second_and_report_ages():
  now = 6_000_000_000
  # last read 0.5 s ago on both axes: extrapolated the full 0.5 s (the old 300 ms cap no longer freezes it)
  [p] = XcpRadarLogger._ui_points(now, 15.0, [_ui_target(lat_updated_ns=5_500_000_000, kin_updated_ns=5_500_000_000,
                                                          radar_track_id=31)])
  assert p == [29.0, 0.25, 0.5, 0.5, 31]  # 30 - 2 * 0.5, 0 + 0.5 * 0.5, radar_track_id wins over the slot
  # 3 s old: capped at UI_EXTRAP_CAP_NS (0.6 s)
  [p] = XcpRadarLogger._ui_points(now, 15.0, [_ui_target(lat_updated_ns=3_000_000_000, kin_updated_ns=3_000_000_000)])
  assert p == [28.8, 0.3, 0.6, 0.6, 7]
  assert UI_EXTRAP_CAP_NS == 600_000_000


def test_cutin_lateral_y_vy_prioritized_over_qual_on_adjacent_cars(monkeypatch):
  _capture_events(monkeypatch)
  ram = bytearray(NUM_SLOTS * STRIDE)
  # Lead (slot 2), secondary adjacent car (slot 6), and third moving adjacent car (slot 9), plus 10 stationary clutter slots
  _write_slot(ram, 2, {"x": 25.0, "vx": -0.5, "ax": 0.0, "y": 0.1, "vy": 0.0, "flags": 0xE0C0, "sub": 0x0400, "age": 40, "qual": 25, "cls": 82})
  _write_slot(ram, 6, {"x": 20.0, "vx": 1.0, "ax": 0.0, "y": -3.1, "vy": 0.4, "flags": 0xE0C0, "sub": 0x0000, "age": 20, "qual": 22, "cls": 75})
  _write_slot(ram, 9, {"x": 38.0, "vx": 0.8, "ax": 0.0, "y": 3.2, "vy": -0.3, "flags": 0xE0C0, "sub": 0x0000, "age": 18, "qual": 21, "cls": 74})
  for s in range(15, 25):
    _write_slot(ram, s, {"x": 15.0 + s, "vx": -15.0, "ax": 0.0, "y": -5.0, "vy": 0.0, "flags": 0xE0C0, "sub": 0x0000, "age": 10, "qual": 15, "cls": 50})

  cp = SimpleNamespace(brand="honda", flags=int(HondaFlags.BOSCH), passive=False)
  logger = XcpRadarLogger(cp)
  t = _unlock_logger(logger, 1_000_000_000, v_ego=15.0)
  counts = {s: {16: 0, 22: 0, 127: 0} for s in (2, 6, 9)}
  rd = SimpleNamespace(points=[
    SimpleNamespace(trackId=1, dRel=25.0, yRel=0.1, vRel=-0.5),
    SimpleNamespace(trackId=2, dRel=20.0, yRel=-3.1, vRel=1.0),
  ])
  # 5 seconds at 20 ms RTT (50 reads/sec)
  for i in range(250):
    if i >= 20:
      logger.observe_radar_tracks(rd, t)
    sends = logger.next_can_sends(t)
    if sends:
      pkt = bytes(sends[0].dat)
      sz = pkt[1]
      off = int.from_bytes(pkt[4:8], "big") - REC0
      s = off // STRIDE
      fld = off % STRIDE
      if i >= 50 and s in counts and fld in counts[s]:
        counts[s][fld] += 1
      rep = b"\xFF" + bytes(ram[off : off + sz])
      logger.observe_can([(t, [(XCP_DTO, rep, XCP_BUS)])], t + 19_000_000, v_ego=15.0)
    t += 20_000_000
  logger.close()

  # Over 4s post-warmup: lead keeps ~25 Hz kin (16) and ~5 Hz lat (22); both adjacent moving cars get >=2.5 Hz (22) and more (22) than (127)
  assert counts[2][16] >= 80
  assert counts[2][22] >= 16
  assert counts[6][22] > counts[6][127] and counts[6][22] >= 10
  assert counts[9][22] > counts[9][127] and counts[9][22] >= 10 and counts[9][16] >= 8


def test_standing_traffic_and_adjacent_lat_not_starved_by_wide_traffic(monkeypatch, tmp_path):
  import json
  _capture_events(monkeypatch)
  ram = bytearray(NUM_SLOTS * STRIDE)
  # Lead (slot 2), adjacent moving lanes (slots 5, 8, 11), standing car (slot 12, born age=1), wide oncoming traffic |y|>6m (slots 14..21)
  _write_slot(ram, 2, {"x": 45.0, "vx": -1.0, "ax": 0.0, "y": 0.2, "vy": 0.0, "flags": 0xE0C0, "sub": 0x0400, "age": 40, "qual": 25, "cls": 82})
  for idx, s in enumerate((5, 8, 11)):
    _write_slot(ram, s, {"x": 30.0 + idx * 8.0, "vx": 0.5, "ax": 0.0, "y": -3.2 if idx % 2 == 0 else 3.4, "vy": 0.0,
                         "flags": 0xE0C0, "sub": 0x0000, "age": 25, "qual": 22, "cls": 75})
  _write_slot(ram, 12, {"x": 65.0, "vx": -15.0, "ax": 0.0, "y": 0.3, "vy": 0.0, "flags": 0xE0C0, "sub": 0x0400, "age": 1, "qual": 24, "cls": 80})
  for idx, s in enumerate(range(14, 22)):
    _write_slot(ram, s, {"x": 55.0 + idx * 6.0, "vx": -30.0, "ax": 0.0, "y": 8.0 if idx % 2 == 0 else -8.5, "vy": 0.0,
                         "flags": 0xE0C0, "sub": 0x0000, "age": 20, "qual": 20, "cls": 70})

  shm_path = tmp_path / "xcp_radar_points.json"
  cp = SimpleNamespace(brand="honda", flags=int(HondaFlags.BOSCH), passive=False)
  logger = XcpRadarLogger(cp, shm_path=shm_path)
  t = _unlock_logger(logger, 1_000_000_000, v_ego=15.0)
  rd = SimpleNamespace(points=[SimpleNamespace(trackId=1, dRel=45.0, yRel=0.2, vRel=-1.0)])

  max_adj_lat_age_s = 0.0
  standing_visible_at = None
  for i in range(400):  # 8.0 s at 20 ms RTT
    if i == 15:
      _write_slot(ram, 12, {"x": 62.0, "vx": -15.0, "ax": 0.0, "y": 0.3, "vy": 0.0, "flags": 0xE0C0, "sub": 0x0400, "age": 5, "qual": 24, "cls": 80})
    if i >= 10:
      logger.observe_radar_tracks(rd, t)
    sends = logger.next_can_sends(t)
    if sends:
      pkt = bytes(sends[0].dat)
      sz = pkt[1]
      off = int.from_bytes(pkt[4:8], "big") - REC0
      rep = b"\xFF" + bytes(ram[off : off + sz])
      logger.observe_can([(t, [(XCP_DTO, rep, XCP_BUS)])], t + 19_000_000, v_ego=15.0)
    if i >= 260:  # post-discovery steady state across 11 active moving + 1 standing slots
      for adj in (5, 8, 11):
        lat_ts = logger.slot_cache[adj]["lat_updated_ns"]
        assert lat_ts > 0
        max_adj_lat_age_s = max(max_adj_lat_age_s, (t - lat_ts) / 1e9)
    if standing_visible_at is None and shm_path.exists():
      pts = json.loads(shm_path.read_text())["points"]
      # d is extrapolated by vRel (-15 m/s) for up to UI_EXTRAP_CAP_NS (0.6 s), so the standing car at 62 m may show as far in as 53 m
      if any(53.0 <= px <= 63.0 and abs(py - 0.3) < 0.5 for px, py, *_ in pts):
        standing_visible_at = i * 0.02
    t += 20_000_000

  logger.close()
  assert max_adj_lat_age_s <= 0.9
  assert standing_visible_at is not None and standing_visible_at <= 7.5


def test_stale_measured_downgraded_and_trailer_offset_lead_match(monkeypatch):
  rows = _capture_events(monkeypatch)
  cp = SimpleNamespace(brand="honda", flags=int(HondaFlags.BOSCH), passive=False)
  logger = XcpRadarLogger(cp)
  logger.state = "READY"
  logger.last_v_ego = 9.8
  t0 = 10_000_000_000

  # Slot 33: s4 tid 22 style lead (dd = 5.93 m > 4.5 m, trailer-compensated dd_eff = 2.54 m)
  logger.slot_cache[33].update({
    "X_raw": int(round(34.16 * 128)),
    "Vx_raw": int(round(-2.45 * 256)),
    "Ax_raw": int(round(-0.8 * 2048)),
    "Y_raw": int(round(0.44 * 128)),
    "Vy_raw": 0,
    "age": 120,
    "flags": 0xE0C0,
    "subflags": 0x0400,
    "qual": 25,
    "cls_conf": 80,
    "radar_track_id": 403,
    "kin_updated_ns": t0 - 25_000_000,
    "lat_updated_ns": t0 - 25_000_000,
    "age_updated_ns": t0 - 25_000_000,
    "flags_updated_ns": t0 - 25_000_000,
    "updated_ns": t0 - 25_000_000,
    "last_rtt_ns": 10_000_000,
  })
  logger.active_slots[33] = logger.slot_cache[33]

  # Slot 10: background measured slot with stale kinematics (> 200 ms old)
  logger.slot_cache[10].update({
    "X_raw": int(round(20.84 * 128)),
    "Vx_raw": int(round(-2.14 * 256)),
    "Ax_raw": 0,
    "Y_raw": int(round(3.43 * 128)),
    "Vy_raw": 0,
    "age": 90,
    "flags": 0xE0C0,
    "subflags": 0x0000,
    "qual": 20,
    "cls_conf": 70,
    "radar_track_id": 407,
    "kin_updated_ns": t0 - 350_000_000,
    "lat_updated_ns": t0 - 25_000_000,
    "age_updated_ns": t0 - 25_000_000,
    "flags_updated_ns": t0 - 25_000_000,
    "updated_ns": t0 - 25_000_000,
    "last_rtt_ns": 10_000_000,
  })
  logger.active_slots[10] = logger.slot_cache[10]

  logger.observe_radar_tracks(
    SimpleNamespace(points=[SimpleNamespace(trackId=22, dRel=28.23, yRel=0.18, vRel=-1.04)]),
    t0,
  )
  logger._maybe_log_20hz(t0)
  assert len(rows) == 1
  by_slot = {tgt["slot"]: tgt for tgt in rows[0]["targets"]}
  assert by_slot[33]["stage"] == "measured"
  assert by_slot[33]["fw_stage"] == "measured"
  assert by_slot[33]["measured"] is True
  assert by_slot[33]["trust_accel"] is True
  assert by_slot[33]["kin_age_ms"] == 25.0
  assert by_slot[33]["op_track_id"] == 22
  assert rows[0]["lead"]["slot"] == 33
  assert rows[0]["lead"]["op_track_id"] == 22
  assert rows[0]["lead"]["lead_source"] == "op_lead"

  assert by_slot[10]["stage"] == "stale"
  assert by_slot[10]["fw_stage"] == "measured"
  assert by_slot[10]["measured"] is False
  assert by_slot[10]["trust_accel"] is False
  assert by_slot[10]["kin_age_ms"] == 350.0

  # When op_tracks has no lead, fallback sets lead_source == 'xcp_fallback'
  logger.observe_radar_tracks(SimpleNamespace(points=[]), t0 + 100_000_000)
  logger._maybe_log_20hz(t0 + 100_000_000)
  assert len(rows) == 2
  assert rows[1]["lead"]["slot"] == 33
  assert rows[1]["lead"]["op_track_id"] is None
  assert rows[1]["lead"]["lead_source"] == "xcp_fallback"
  assert logger._next_read_task_locked(t0 + 100_000_000) is not None
  logger.close()


def test_fast_inactive_probe_when_op_lead_unmatched():
  cp = SimpleNamespace(brand="honda", flags=int(HondaFlags.BOSCH), passive=False)
  logger = XcpRadarLogger(cp)
  logger.state = "READY"
  logger.last_v_ego = 16.0
  t0 = 10_000_000_000
  # Seed a non-matching active target so prio_slots is non-empty, plus an unmatched in-path op_track
  logger.slot_cache[2].update({
    "X_raw": int(round(30.0 * 128)),
    "Vx_raw": int(round(-1.0 * 256)),
    "Y_raw": int(round(-3.5 * 128)),
    "age": 20,
    "flags": 0xE0C0,
    "flags_updated_ns": t0,
    "age_updated_ns": t0,
    "lat_updated_ns": t0,
    "kin_updated_ns": t0,
    "qual_updated_ns": t0,
  })
  logger.active_slots[2] = logger.slot_cache[2]
  logger._op_matched_slots = []
  logger.observe_radar_tracks(SimpleNamespace(points=[SimpleNamespace(trackId=41, dRel=83.0, yRel=-0.2, vRel=-12.0)]), t0)

  # After 3 reads (INACTIVE_PROBE_PERIOD_NO_LEAD), should probe an inactive slot (offset 16, size 6 on slot != 2)
  logger._read_seq = 2
  logger._last_probe_seq = 0
  task = logger._next_read_task_locked(t0 + 5_000_000)
  assert task is not None and task[0] != 2 and task[1:] == (16, 6)
  logger.close()


def test_wayb_1hz_probe_reads_a150_bases_and_preserves_lead_rate(monkeypatch):
  rows = _capture_events(monkeypatch)
  monkeypatch.setattr(xcp_mod, "WAYB_PROBE_ENABLED", True)  # Way B is off by default (Peter 2026-11-23); this test exercises the inert probe code
  ram = bytearray(NUM_SLOTS * STRIDE)
  _write_slot(ram, 4, {"x": 32.0, "vx": -1.5, "ax": -0.8, "y": 0.1, "vy": 0.0, "flags": 0xE0C0, "sub": 0x0400, "age": 35, "qual": 25, "cls": 85})

  assert not any(name.startswith("WAYB_PTR_") for name in dir(xcp_mod))
  assert not hasattr(xcp_mod, "WAYB_SEL_CAM_OFFSET")
  assert not hasattr(xcp_mod, "WAYB_P_WIN_START")
  assert not hasattr(xcp_mod, "WAYB_T1_DUMP_CHUNK_SIZE")
  t1_base = 0x40032AEC
  out18_base = 0x4003681C
  p_base = 0x400397FC
  t1_slot = 7
  assert xcp_mod.WAYB_DEFAULT_T1_BASE == t1_base
  assert xcp_mod.WAYB_DEFAULT_OUT18_BASE == out18_base
  assert xcp_mod.WAYB_DEFAULT_P_BASE == p_base

  # Mock bounded A150 SRAM for P (0x400397FC), core T1[7], and OUT18[0..31]+16.
  t1_slot7 = bytearray(bytes((i & 0xFF) for i in range(104)))
  t1_slot7[24:30] = (
    int(round(32.0 * 128)).to_bytes(2, "big", signed=True)
    + (0).to_bytes(2, "big", signed=True)
    + int(round(-1.5 * 256)).to_bytes(2, "big", signed=True)
  )
  t1_slot7[40:44] = int(round(-15.0 * 256)).to_bytes(2, "big", signed=True) + (0).to_bytes(2, "big", signed=True)
  t1_slot7[86:92] = (0x8800).to_bytes(2, "big") + bytes([0, 0, 0, 4])
  out18_table = {s: bytes([0, 0]) for s in range(32)}
  out18_table[t1_slot] = bytes([22, 0x03])

  cp = SimpleNamespace(brand="honda", flags=int(HondaFlags.BOSCH), passive=False)
  logger = XcpRadarLogger(cp, probe_wayb=True)
  assert logger._wayb_probe_enabled is True
  t = _unlock_logger(logger, 1_000_000_000, v_ego=15.0)
  rd = SimpleNamespace(points=[SimpleNamespace(trackId=22, dRel=28.6, yRel=0.1, vRel=-1.5)])

  wayb_p0_reads = 0
  lead_kin_reads = 0
  for i in range(180):  # 3.6 s at 20 ms RTT
    if i >= 15:
      logger.observe_radar_tracks(rd, t)
    sends = logger.next_can_sends(t)
    if sends:
      pkt = bytes(sends[0].dat)
      sz = pkt[1]
      addr = int.from_bytes(pkt[4:8], "big")
      if REC0 <= addr < REC0 + NUM_SLOTS * STRIDE:
        off = addr - REC0
        if (off // STRIDE) == 4 and (off % STRIDE) == 16 and i >= 20:
          lead_kin_reads += 1
        rep = b"\xFF" + bytes(ram[off : off + sz])
      elif addr == p_base + 2 and sz == 7:
        wayb_p0_reads += 1
        rep = b"\xFF" + bytes([0x00, 0x01, 0x00, 0x02, 5, t1_slot, 0])
      elif (t1_base + 104 * t1_slot) <= addr and (addr + sz) <= (t1_base + 104 * (t1_slot + 1)):
        off = addr - (t1_base + 104 * t1_slot)
        assert (off, sz) in ((24, 6), (40, 4), (86, 6))
        rep = b"\xFF" + bytes(t1_slot7[off : off + sz])
      elif sz == 2 and ((addr - out18_base) % 18) == 16 and 0 <= ((addr - out18_base) // 18) < 32:
        s = (addr - out18_base) // 18
        rep = b"\xFF" + out18_table[s]
      else:
        raise AssertionError(f"Unexpected XCP probe address {addr:#x} size={sz}")
      logger.observe_can([(t, [(XCP_DTO, rep, XCP_BUS)])], t + 19_000_000, v_ego=15.0)
    t += 20_000_000

  assert logger.state == "READY"
  logger.close()

  assert 3 <= wayb_p0_reads <= 4
  assert lead_kin_reads >= 45
  wb = rows[-1].get("wayb_probe")
  assert wb is not None
  assert "ptrs_raw" not in wb
  assert "sel_cam_id" not in wb and "sel_flags" not in wb and "sel_state" not in wb
  assert "p_win" not in wb and "t1_dump" not in wb
  assert wb["t1_base"] == hex(t1_base)
  assert wb["out18_base"] == hex(out18_base)
  assert wb["p_base"] == hex(p_base)
  assert wb["p0_valid"] == 5
  assert wb["p0_t1_slot"] == t1_slot
  assert wb["p0_slot_queried"] == t1_slot
  assert wb["t1_x_raw"] == int(round(32.0 * 128))
  assert wb["t1_vx28_raw"] == int(round(-1.5 * 256))
  assert wb["t1_vx40_raw"] == int(round(-15.0 * 256))
  assert wb["t1_flags"] == 0x8800
  assert wb["t1_valid"] == 4
  assert wb["out18_cam_id"] == 22
  assert wb["out18_cam_flags"] == 0x03
  assert len(wb["out18_scan"]) == 4
  assert wb["last_err"] is None
  assert wb["last_err_addr"] is None


def test_wayb_1hz_probe_idle_oob_slot_and_error_isolation(monkeypatch):
  rows = _capture_events(monkeypatch)
  monkeypatch.setattr(xcp_mod, "WAYB_PROBE_ENABLED", True)  # Way B is off by default (Peter 2026-11-23); this test exercises the inert probe code
  ram = bytearray(NUM_SLOTS * STRIDE)
  _write_slot(ram, 4, {"x": 32.0, "vx": -1.5, "ax": -0.8, "y": 0.1, "vy": 0.0, "flags": 0xE0C0, "sub": 0x0400, "age": 35, "qual": 25, "cls": 85})

  t1_base = 0x40032AEC
  out18_base = 0x4003681C
  slot0_t1 = bytes(104)

  cp = SimpleNamespace(brand="honda", flags=int(HondaFlags.BOSCH), passive=False)
  logger = XcpRadarLogger(cp, probe_wayb=True)
  t = _unlock_logger(logger, 1_000_000_000, v_ego=15.0)
  rd = SimpleNamespace(points=[SimpleNamespace(trackId=22, dRel=28.6, yRel=0.1, vRel=-1.5)])

  # Round 1: idle p0 (p0_valid=0, p0_t1_slot=255) -> must query slot 0 (slot_source="idle0")
  # Round 2: out-of-range slot (p0_valid=1, p0_t1_slot=32) -> must clamp/fallback to slot 0 (slot_source="idle0")
  # Round 3: XCP 0xFE error on p0 -> must log last_err="fe22", last_err_addr="0x400397fe", clear _wayb_queue, stay READY
  p0_replies = [
    b"\xFF" + bytes([0x00, 0x00, 0x00, 0x00, 0, 255, 0]),
    b"\xFF" + bytes([0x00, 0x01, 0x00, 0x02, 1, 32, 0]),
    b"\xFE\x22",
  ]
  p0_round = 0
  seen_slot0_t1 = 0
  seen_slot0_out18 = 0

  for i in range(160):  # 3.2 s at 20 ms RTT
    if i >= 10:
      logger.observe_radar_tracks(rd, t)
    sends = logger.next_can_sends(t)
    if sends:
      pkt = bytes(sends[0].dat)
      sz = pkt[1]
      addr = int.from_bytes(pkt[4:8], "big")
      if REC0 <= addr < REC0 + NUM_SLOTS * STRIDE:
        off = addr - REC0
        rep = b"\xFF" + bytes(ram[off : off + sz])
      elif addr == 0x400397FE and sz == 7:
        rep = p0_replies[min(p0_round, len(p0_replies) - 1)]
        p0_round += 1
      elif t1_base <= addr and (addr + sz) <= t1_base + 104:
        off = addr - t1_base
        assert (off, sz) in ((24, 6), (40, 4), (86, 6))
        if off == 24:
          seen_slot0_t1 += 1
        rep = b"\xFF" + slot0_t1[off : off + sz]
      elif sz == 2 and ((addr - out18_base) % 18) == 16 and 0 <= ((addr - out18_base) // 18) < 32:
        if addr == out18_base + 16:
          seen_slot0_out18 += 1
        rep = b"\xFF" + bytes(2)
      else:
        raise AssertionError(f"Out-of-bounds or unexpected Way B address {addr:#x} size={sz}")
      logger.observe_can([(t, [(XCP_DTO, rep, XCP_BUS)])], t + 19_000_000, v_ego=15.0)
    t += 20_000_000

  assert logger.state == "READY"
  assert p0_round >= 3
  assert seen_slot0_t1 == 2
  assert seen_slot0_out18 == 3  # 2 core out18 reads + 1 round-1 out18_scan read on slot 0
  assert len(logger._wayb_queue) == 0
  logger.close()

  wb = rows[-1].get("wayb_probe")
  assert wb is not None
  assert wb["p0_slot_queried"] == 0
  assert wb["slot_source"] == "idle0"
  assert wb["last_err"] == "fe22"
  assert wb["last_err_addr"] == "0x400397fe"
  assert wb["updated_ns"] > 0

  # Verify first-read 0xFE error alone sets updated_ns > 0 and emits wayb_probe
  rows.clear()
  logger2 = XcpRadarLogger(cp, probe_wayb=True)
  t2 = _unlock_logger(logger2, 1_000_000_000, v_ego=15.0)
  for _ in range(30):
    sends = logger2.next_can_sends(t2)
    if sends:
      pkt = bytes(sends[0].dat)
      sz = pkt[1]
      addr = int.from_bytes(pkt[4:8], "big")
      if REC0 <= addr < REC0 + NUM_SLOTS * STRIDE:
        rep = b"\xFF" + bytes(ram[addr - REC0 : addr - REC0 + sz])
      elif addr == 0x400397FE:
        rep = b"\xFE\x22"
      else:
        raise AssertionError(f"Unexpected address {addr:#x}")
      logger2.observe_can([(t2, [(XCP_DTO, rep, XCP_BUS)])], t2 + 19_000_000, v_ego=15.0)
    t2 += 20_000_000
  assert logger2.state == "READY"
  assert len(logger2._wayb_queue) == 0
  logger2.close()
  wb2 = rows[-1].get("wayb_probe")
  assert wb2 is not None and wb2["last_err"] == "fe22" and wb2["last_err_addr"] == "0x400397fe" and wb2["updated_ns"] > 0

  # Verify 0xFE error on an out18_scan read (o18s:1) drops only the rest of round 1 and advances cursor in round 2
  rows.clear()
  logger3 = XcpRadarLogger(cp, probe_wayb=True)
  t3 = _unlock_logger(logger3, 1_000_000_000, v_ego=15.0)
  p0_count = 0
  r1_out18_slots: list[int] = []
  r2_out18_slots: list[int] = []
  saw_r1_core = False
  scan_err_addr = out18_base + 18 * 1 + 16  # 0x4003683e (o18s:1)
  for _ in range(95):  # 1.9 s -> completes rounds 1 and 2 before round 3 starts at +2.0 s
    sends = logger3.next_can_sends(t3)
    if sends:
      pkt = bytes(sends[0].dat)
      sz = pkt[1]
      addr = int.from_bytes(pkt[4:8], "big")
      if REC0 <= addr < REC0 + NUM_SLOTS * STRIDE:
        rep = b"\xFF" + bytes(ram[addr - REC0 : addr - REC0 + sz])
      elif addr == 0x400397FE and sz == 7:
        p0_count += 1
        rep = b"\xFF" + bytes([0, 0, 0, 0, 0, 255, 0])
      elif t1_base <= addr and (addr + sz) <= t1_base + 104:
        off = addr - t1_base
        assert (off, sz) in ((24, 6), (40, 4), (86, 6))
        if p0_count == 1 and off == 24:
          saw_r1_core = True
        rep = b"\xFF" + slot0_t1[off : off + sz]
      elif sz == 2 and ((addr - out18_base) % 18) == 16 and 0 <= ((addr - out18_base) // 18) < 32:
        s = (addr - out18_base) // 18
        if p0_count == 1:
          r1_out18_slots.append(s)
        elif p0_count == 2:
          r2_out18_slots.append(s)
        rep = b"\xFE\x22" if addr == scan_err_addr else b"\xFF" + bytes(2)
      else:
        raise AssertionError(f"Unexpected address {addr:#x} size={sz}")
      logger3.observe_can([(t3, [(XCP_DTO, rep, XCP_BUS)])], t3 + 19_000_000, v_ego=15.0)
    t3 += 20_000_000
  assert logger3.state == "READY"
  assert len(logger3._wayb_queue) == 0
  logger3.close()
  assert p0_count == 2
  assert saw_r1_core
  assert r1_out18_slots == [0, 0, 1]  # core slot 0 + o18s:0 + o18s:1 (0xFE drops o18s:2..3)
  assert r2_out18_slots == [0, 4, 5, 6, 7]  # core slot 0 + o18s:4..7
  wb3 = rows[-1].get("wayb_probe")
  assert wb3 is not None and wb3["last_err"] == "fe22" and wb3["last_err_addr"] == hex(scan_err_addr)
  assert [s for s, _, _ in wb3["out18_scan"]] == [4, 5, 6, 7]


def test_wayb_1hz_out18_scan_locks_inlane_lead_slot_and_releases_on_change(monkeypatch):
  rows = _capture_events(monkeypatch)
  monkeypatch.setattr(xcp_mod, "WAYB_PROBE_ENABLED", True)  # Way B is off by default (Peter 2026-11-23); this test exercises the inert probe code
  ram = bytearray(NUM_SLOTS * STRIDE)
  _write_slot(ram, 4, {"x": 28.4, "vx": -1.0, "ax": -0.2, "y": 0.2, "vy": 0.0, "flags": 0xE0C0, "sub": 0x0400, "age": 35, "qual": 25, "cls": 85})

  t1_base = 0x40032AEC
  out18_base = 0x4003681C
  lead_slot = 13

  t1_tables: dict[int, bytearray] = {s: bytearray(104) for s in range(32)}
  t1_tables[lead_slot][24:30] = (
    int(round(25.0 * 128)).to_bytes(2, "big", signed=True)
    + (0).to_bytes(2, "big", signed=True)
    + int(round(0.2 * 128)).to_bytes(2, "big", signed=True)
  )
  t1_tables[lead_slot][40:44] = int(round(-1.0 * 256)).to_bytes(2, "big", signed=True) + (0).to_bytes(2, "big", signed=True)
  t1_tables[lead_slot][86:92] = (0x4C50).to_bytes(2, "big") + bytes([0, 0, 0, 4])

  out18_table: dict[int, bytes] = {s: bytes([0, 0]) for s in range(32)}
  out18_table[lead_slot] = bytes([42, 0x40])

  cp = SimpleNamespace(brand="honda", flags=int(HondaFlags.BOSCH), passive=False)
  logger = XcpRadarLogger(cp, probe_wayb=True)
  t = _unlock_logger(logger, 1_000_000_000, v_ego=15.0)
  rd = SimpleNamespace(points=[SimpleNamespace(trackId=42, dRel=25.0, yRel=0.2, vRel=-1.0)])

  # Phase 1: 6.2 s (6 rounds) while p0 is idle -> round 4 scans slots 12..15, locking slot=13 on rounds 5..6
  phase1_snapshots: list[dict] = []
  last_snap_upd = 0
  for i in range(310):
    if i >= 10:
      logger.observe_radar_tracks(rd, t)
    sends = logger.next_can_sends(t)
    if sends:
      pkt = bytes(sends[0].dat)
      sz = pkt[1]
      addr = int.from_bytes(pkt[4:8], "big")
      if REC0 <= addr < REC0 + NUM_SLOTS * STRIDE:
        rep = b"\xFF" + bytes(ram[addr - REC0 : addr - REC0 + sz])
      elif addr == 0x400397FE and sz == 7:
        rep = b"\xFF" + bytes([0, 0, 0, 0, 0, 255, 0])
      elif t1_base <= addr and (addr + sz) <= t1_base + 32 * 104:
        s = (addr - t1_base) // 104
        off = (addr - t1_base) % 104
        assert (off, sz) in ((24, 6), (40, 4), (86, 6))
        rep = b"\xFF" + bytes(t1_tables[s][off : off + sz])
      elif sz == 2 and ((addr - out18_base) % 18) == 16 and 0 <= ((addr - out18_base) // 18) < 32:
        rep = b"\xFF" + out18_table[(addr - out18_base) // 18]
      else:
        raise AssertionError(f"Unexpected address {addr:#x} size={sz}")
      logger.observe_can([(t, [(XCP_DTO, rep, XCP_BUS)])], t + 19_000_000, v_ego=15.0)
    wb_cur = rows[-1].get("wayb_probe") if rows else None
    if wb_cur and wb_cur["updated_ns"] != last_snap_upd:
      last_snap_upd = wb_cur["updated_ns"]
      phase1_snapshots.append(wb_cur)
    t += 20_000_000

  locked = [w for w in phase1_snapshots if w.get("slot_source") == "lead_scan"]
  assert len(locked) >= 2
  last_locked = locked[-1]
  assert "p_win" not in last_locked and "t1_dump" not in last_locked
  assert last_locked["lead_slot"] == lead_slot
  assert last_locked["lead_cam_id"] == 42
  assert last_locked["p0_slot_queried"] == lead_slot
  assert last_locked["t1_x_raw"] == int(round(25.0 * 128))
  assert last_locked["t1_vx40_raw"] == int(round(-1.0 * 256))
  assert last_locked["t1_flags"] == 0x4C50

  # Phase 2: clear OUT18[13] and run 2 more rounds -> must release lock back to idle0 / slot 0
  out18_table[lead_slot] = bytes([0, 0])
  for _ in range(110):
    logger.observe_radar_tracks(rd, t)
    sends = logger.next_can_sends(t)
    if sends:
      pkt = bytes(sends[0].dat)
      sz = pkt[1]
      addr = int.from_bytes(pkt[4:8], "big")
      if REC0 <= addr < REC0 + NUM_SLOTS * STRIDE:
        rep = b"\xFF" + bytes(ram[addr - REC0 : addr - REC0 + sz])
      elif addr == 0x400397FE and sz == 7:
        rep = b"\xFF" + bytes([0, 0, 0, 0, 0, 255, 0])
      elif t1_base <= addr and (addr + sz) <= t1_base + 32 * 104:
        s = (addr - t1_base) // 104
        off = (addr - t1_base) % 104
        assert (off, sz) in ((24, 6), (40, 4), (86, 6))
        rep = b"\xFF" + bytes(t1_tables[s][off : off + sz])
      elif sz == 2 and ((addr - out18_base) % 18) == 16 and 0 <= ((addr - out18_base) // 18) < 32:
        rep = b"\xFF" + out18_table[(addr - out18_base) // 18]
      else:
        raise AssertionError(f"Unexpected address {addr:#x} size={sz}")
      logger.observe_can([(t, [(XCP_DTO, rep, XCP_BUS)])], t + 19_000_000, v_ego=15.0)
    t += 20_000_000

  wb_rel = rows[-1].get("wayb_probe")
  assert wb_rel["lead_slot"] is None
  assert wb_rel["lead_cam_id"] is None
  assert wb_rel["slot_source"] == "idle0"
  assert wb_rel["p0_slot_queried"] == 0
  logger.close()


def test_match_openpilot_tracks_keeps_locked_closing_lead_and_railed_u11():
  cp = SimpleNamespace(brand="honda", flags=int(HondaFlags.BOSCH), passive=False)
  logger = XcpRadarLogger(cp)
  op_tracks = [{"tid": 34, "dRel": 45.0, "yRel": -0.1, "vRel": 2.5}]

  # Initial acquisition with dv = 4.5 m/s (> 3.5) must still reject when not previously locked.
  t_wide = [{"slot": 11, "dRel_m": 44.5, "yRel_m": -0.2, "vRel_mps": -2.0, "lat_updated_ns": 1}]
  matched, op_lead_slot = logger._match_openpilot_tracks(t_wide, op_tracks, v_ego=11.0)
  assert matched == [] and op_lead_slot is None and t_wide[0].get("op_track_id") is None

  # Railed U11 (o["vRel"] <= -11.5) matches on initial acquisition up to dv <= 25.0.
  t_rail = [{"slot": 12, "dRel_m": 45.0, "yRel_m": 0.0, "vRel_mps": -17.5, "lat_updated_ns": 1}]
  matched_rail, op_lead_rail = logger._match_openpilot_tracks(
    t_rail, [{"tid": 40, "dRel": 44.5, "yRel": 0.0, "vRel": -12.0}], v_ego=22.0
  )
  assert matched_rail == [12] and op_lead_rail == 12

  # Acquire moving lead within dv < 3.5, then keep locked across closing_lag = 6.0 m/s (dv = 6.0 > 3.5).
  t_acq = [{"slot": 11, "dRel_m": 44.5, "yRel_m": -0.2, "vRel_mps": 0.5, "lat_updated_ns": 1}]
  matched1, lead1 = logger._match_openpilot_tracks(t_acq, op_tracks, v_ego=11.0)
  assert matched1 == [11] and lead1 == 11

  t_brake = [{"slot": 11, "dRel_m": 44.0, "yRel_m": -0.2, "vRel_mps": -3.5, "lat_updated_ns": 1}]
  matched2, lead2 = logger._match_openpilot_tracks(t_brake, op_tracks, v_ego=11.0)
  assert matched2 == [11] and lead2 == 11 and t_brake[0].get("op_track_id") == 34

  # Stationary target, wide lateral dy >= 1.5, zero yRel_m, or out-of-lane |yRel| > 1.5 must NOT use
  # the locked continuation window.
  t_stat = [{"slot": 11, "dRel_m": 44.0, "yRel_m": -0.2, "vRel_mps": -11.0, "lat_updated_ns": 1}]
  matched3, _ = logger._match_openpilot_tracks(t_stat, op_tracks, v_ego=11.0)
  assert matched3 == []

  logger._match_openpilot_tracks(t_acq, op_tracks, v_ego=11.0)
  t_adj = [{"slot": 11, "dRel_m": 44.0, "yRel_m": 1.6, "vRel_mps": -3.5, "lat_updated_ns": 1}]
  matched4, _ = logger._match_openpilot_tracks(t_adj, op_tracks, v_ego=11.0)
  assert matched4 == []

  op_y0 = [{"tid": 34, "dRel": 44.5, "yRel": 0.0, "vRel": 2.5}]
  t_y0_acq = [{"slot": 11, "dRel_m": 44.5, "yRel_m": 0.1, "vRel_mps": 0.5, "lat_updated_ns": 1}]
  logger._match_openpilot_tracks(t_y0_acq, op_y0, v_ego=11.0)
  t_y0_brake = [{"slot": 11, "dRel_m": 44.0, "yRel_m": 0.0, "vRel_mps": -3.5, "lat_updated_ns": 1}]
  matched5, _ = logger._match_openpilot_tracks(t_y0_brake, op_y0, v_ego=11.0)
  assert matched5 == []

  op_out = [{"tid": 35, "dRel": 44.5, "yRel": 1.6, "vRel": 2.5}]
  t_out_acq = [{"slot": 12, "dRel_m": 44.5, "yRel_m": 1.6, "vRel_mps": 0.5, "lat_updated_ns": 1}]
  logger._match_openpilot_tracks(t_out_acq, op_out, v_ego=11.0)
  t_out_brake = [{"slot": 12, "dRel_m": 44.0, "yRel_m": 1.6, "vRel_mps": -3.5, "lat_updated_ns": 1}]
  matched6, _ = logger._match_openpilot_tracks(t_out_brake, op_out, v_ego=11.0)
  assert matched6 == []
  logger.close()


def test_fast_assist_shm_publishes_on_primary_lead_kin_arrival_rate_limited(monkeypatch, tmp_path):
  import json
  rows = _capture_events(monkeypatch)
  ram = bytearray(NUM_SLOTS * STRIDE)
  _write_slot(ram, 5, {"x": 30.0, "vx": -1.5, "ax": -1.0, "y": 0.1, "vy": 0.0, "flags": 0xE0C0, "sub": 0x0400, "age": 35, "qual": 25, "cls": 85})

  ui_path = tmp_path / "xcp_radar_points.json"
  assist_path = tmp_path / "xcp_radar_assist.json"
  cp = SimpleNamespace(brand="honda", flags=int(HondaFlags.BOSCH), passive=False)
  logger = XcpRadarLogger(cp, shm_path=ui_path, assist_shm_path=assist_path, assist_fast_publish=True)
  t = _unlock_logger(logger, 1_000_000_000, v_ego=15.0)
  rd = SimpleNamespace(points=[SimpleNamespace(trackId=12, dRel=30.0, yRel=0.1, vRel=-1.5)])

  # Initialize slot 5 so it becomes prio_slots[0] and reaches first 50 ms epoch log
  for _ in range(15):
    logger.observe_radar_tracks(rd, t)
    sends = logger.next_can_sends(t)
    if sends:
      pkt = bytes(sends[0].dat)
      sz = pkt[1]
      off = int.from_bytes(pkt[4:8], "big") - REC0
      rep = b"\xFF" + bytes(ram[off : off + sz])
      logger.observe_can([(t, [(XCP_DTO, rep, XCP_BUS)])], t + 10_000_000, v_ego=15.0)
    t += 15_000_000

  assert assist_path.exists()
  # Sync cleanly to a fresh 50 ms epoch boundary ahead of any initialization timestamps.
  epoch_ns = max(t, logger.last_log_ns + 60_000_000)
  logger.observe_radar_tracks(rd, epoch_ns)
  logger.observe_can([], epoch_ns, v_ego=15.0)
  assert logger.last_log_ns == epoch_ns
  assert json.loads(assist_path.read_text())["mono_ns"] == epoch_ns
  events_before = len(rows)

  # Drive a direct priority-lead kinematics reply only 22 ms after the 50 ms log epoch (< 50 ms EPOCH_NS):
  # assist SHM must update immediately with new vRel, without emitting a second cloudlog.event.
  t_mid = epoch_ns + 22_000_000
  _write_slot(ram, 5, {"x": 29.8, "vx": -3.25, "ax": -3.0, "y": 0.1, "vy": 0.0, "flags": 0xE0C0, "sub": 0x0400, "age": 36, "qual": 25, "cls": 85})
  with logger._lock:
    logger.in_flight = ("F4", 5, 16, 6, t_mid - 8_000_000)
  rep_mid = b"\xFF" + bytes(ram[5 * STRIDE + 16 : 5 * STRIDE + 22])
  logger.observe_can([(t_mid, [(XCP_DTO, rep_mid, XCP_BUS)])], t_mid, v_ego=15.0)

  assert len(rows) == events_before
  assist_mid = json.loads(assist_path.read_text())
  assert assist_mid["mono_ns"] == t_mid
  assert assist_mid["tracks_by_op_id"]["12"]["vRel_mps"] == -3.25

  # Another reply only 10 ms later (< 20 ms ASSIST_SHM_MIN_INTERVAL_NS) is rate-limited until the next 20 ms window.
  t_soon = t_mid + 10_000_000
  _write_slot(ram, 5, {"x": 29.7, "vx": -3.75, "ax": -3.0, "y": 0.1, "vy": 0.0, "flags": 0xE0C0, "sub": 0x0400, "age": 36, "qual": 25, "cls": 85})
  with logger._lock:
    logger.in_flight = ("F4", 5, 16, 6, t_soon - 5_000_000)
  rep_soon = b"\xFF" + bytes(ram[5 * STRIDE + 16 : 5 * STRIDE + 22])
  logger.observe_can([(t_soon, [(XCP_DTO, rep_soon, XCP_BUS)])], t_soon, v_ego=15.0)
  assert len(rows) == events_before
  assert json.loads(assist_path.read_text())["mono_ns"] == t_mid

  logger.close()


def test_fast_assist_shm_gated_by_param_and_skipped_on_card_loop_when_io_thread_active(monkeypatch, tmp_path):
  import json

  class _FakeParams:
    def __init__(self, root):
      self.root = root

    def get_param_path(self, key: str) -> str:
      return str(self.root / key)

  _capture_events(monkeypatch)
  ram = bytearray(NUM_SLOTS * STRIDE)
  _write_slot(ram, 5, {"x": 30.0, "vx": -1.5, "ax": -1.0, "y": 0.1, "vy": 0.0, "flags": 0xE0C0, "sub": 0x0400, "age": 35, "qual": 25, "cls": 85})

  params_dir = tmp_path / "params"
  params_dir.mkdir()
  (params_dir / "IsReleaseBranch").write_bytes(b"0")
  (params_dir / "AlphaHondaXcpRadarAssist").write_bytes(b"0")
  fake_params = _FakeParams(params_dir)

  cp = SimpleNamespace(
    brand="honda",
    carFingerprint=CAR.HONDA_CIVIC_BOSCH,
    flags=int(HondaFlags.BOSCH),
    alternativeExperience=0,
    openpilotLongitudinalControl=True,
    pcmCruise=True,
    radarUnavailable=False,
    notCar=False,
    passive=False,
    dashcamOnly=False,
  )

  ui_off = tmp_path / "points_off.json"
  assist_off = tmp_path / "assist_off.json"
  logger_no_params = XcpRadarLogger(cp, shm_path=ui_off, assist_shm_path=assist_off)
  assert logger_no_params._assist_fast_publish_enabled is False
  logger_no_params.close()

  logger_off = XcpRadarLogger(cp, params=fake_params, shm_path=ui_off, assist_shm_path=assist_off)
  assert logger_off._assist_fast_publish_enabled is False

  t = _unlock_logger(logger_off, 2_000_000_000, v_ego=15.0)
  rd = SimpleNamespace(points=[SimpleNamespace(trackId=12, dRel=30.0, yRel=0.1, vRel=-1.5)])
  for _ in range(15):
    logger_off.observe_radar_tracks(rd, t)
    sends = logger_off.next_can_sends(t)
    if sends:
      pkt = bytes(sends[0].dat)
      sz = pkt[1]
      off = int.from_bytes(pkt[4:8], "big") - REC0
      rep = b"\xFF" + bytes(ram[off : off + sz])
      logger_off.observe_can([(t, [(XCP_DTO, rep, XCP_BUS)])], t + 10_000_000, v_ego=15.0)
    t += 15_000_000

  # 20 Hz _write_ui_snapshot still runs for V-ASM side_sightings even when AlphaHondaXcpRadarAssist is Off.
  assert assist_off.exists()
  epoch_ns = max(t, logger_off.last_log_ns + 60_000_000)
  logger_off.observe_radar_tracks(rd, epoch_ns)
  logger_off.observe_can([], epoch_ns, v_ego=15.0)
  assert json.loads(assist_off.read_text())["mono_ns"] == epoch_ns

  # Mid-epoch F4 reply must NOT trigger 50 Hz fast-publish when assist toggle is Off.
  t_mid = epoch_ns + 25_000_000
  _write_slot(ram, 5, {"x": 29.7, "vx": -3.5, "ax": -3.0, "y": 0.1, "vy": 0.0, "flags": 0xE0C0, "sub": 0x0400, "age": 36, "qual": 25, "cls": 85})
  with logger_off._lock:
    logger_off.in_flight = ("F4", 5, 16, 6, t_mid - 5_000_000)
  rep_mid = b"\xFF" + bytes(ram[5 * STRIDE + 16 : 5 * STRIDE + 22])
  logger_off.observe_can([(t_mid, [(XCP_DTO, rep_mid, XCP_BUS)])], t_mid, v_ego=15.0)
  assert logger_off._assist_shm_dirty is False
  assert json.loads(assist_off.read_text())["mono_ns"] == epoch_ns
  logger_off.close()

  # With AlphaHondaXcpRadarAssist=1, fast publish enables, and observe_can skips _maybe_publish_assist_shm
  # whenever the dedicated IO thread owns fast publishing.
  (params_dir / "AlphaHondaXcpRadarAssist").write_bytes(b"1")
  logger_on = XcpRadarLogger(
    cp,
    params=fake_params,
    shm_path=tmp_path / "points_on.json",
    assist_shm_path=tmp_path / "assist_on.json",
  )
  assert logger_on._assist_fast_publish_enabled is True
  publish_calls = 0

  def _count_publish(ns: int) -> None:
    nonlocal publish_calls
    publish_calls += 1

  monkeypatch.setattr(logger_on, "_maybe_publish_assist_shm", _count_publish)
  logger_on.state = "READY"
  logger_on._use_io_thread = True
  logger_on._assist_shm_dirty = True
  logger_on.observe_can([], epoch_ns + 25_000_000, v_ego=15.0)
  assert publish_calls == 0
  logger_on.close()


def test_kin_changed_ns_and_value_changes_counter_skip_duplicate_reads(monkeypatch, tmp_path):
  import json
  rows = _capture_events(monkeypatch)
  ram = bytearray(NUM_SLOTS * STRIDE)
  _write_slot(ram, 5, {"x": 30.0, "vx": -1.5, "ax": -1.0, "y": 0.1, "vy": 0.0, "flags": 0xE0C0, "sub": 0x0400, "age": 35, "qual": 25, "cls": 85})

  ui_path = tmp_path / "xcp_radar_points.json"
  assist_path = tmp_path / "xcp_radar_assist.json"
  cp = SimpleNamespace(brand="honda", flags=int(HondaFlags.BOSCH), passive=False)
  logger = XcpRadarLogger(cp, shm_path=ui_path, assist_shm_path=assist_path, assist_fast_publish=True)
  t = _unlock_logger(logger, 3_000_000_000, v_ego=15.0)
  rd = SimpleNamespace(points=[SimpleNamespace(trackId=12, dRel=30.0, yRel=0.1, vRel=-1.5)])

  for _ in range(15):
    logger.observe_radar_tracks(rd, t)
    sends = logger.next_can_sends(t)
    if sends:
      pkt = bytes(sends[0].dat)
      sz = pkt[1]
      off = int.from_bytes(pkt[4:8], "big") - REC0
      rep = b"\xFF" + bytes(ram[off : off + sz])
      logger.observe_can([(t, [(XCP_DTO, rep, XCP_BUS)])], t + 10_000_000, v_ego=15.0)
    t += 15_000_000

  epoch_ns = max(t, logger.last_log_ns + 60_000_000)
  logger.observe_radar_tracks(rd, epoch_ns)
  logger.observe_can([], epoch_ns, v_ego=15.0)
  t0_meta = next(tgt for tgt in rows[-1]["targets"] if tgt["slot"] == 5)
  chg0 = t0_meta["kin_changed_ns"]
  cnt0 = t0_meta["kin_value_changes"]
  assert chg0 > 0 and cnt0 >= 1

  # Duplicate identical (16, 6) bytes 25 ms later (> 20 ms fast-SHM interval):
  # kin_updated_ns advances, but kin_changed_ns/kin_value_changes hold and no 50 Hz SHM write fires.
  t_dup = epoch_ns + 25_000_000
  with logger._lock:
    logger.in_flight = ("F4", 5, 16, 6, t_dup - 5_000_000)
  rep_dup = b"\xFF" + bytes(ram[5 * STRIDE + 16 : 5 * STRIDE + 22])
  logger.observe_can([(t_dup, [(XCP_DTO, rep_dup, XCP_BUS)])], t_dup, v_ego=15.0)
  rec = logger.slot_cache[5]
  assert rec["kin_updated_ns"] == t_dup
  assert rec["kin_changed_ns"] == chg0
  assert rec["kin_value_changes"] == cnt0
  assert logger._assist_shm_dirty is False
  assert json.loads(assist_path.read_text())["mono_ns"] == epoch_ns

  # Changed (16, 6) bytes at +40 ms: updates kin_changed_ns, increments counter, and publishes 50 Hz SHM.
  t_chg = epoch_ns + 40_000_000
  _write_slot(ram, 5, {"x": 29.8, "vx": -3.25, "ax": -3.0, "y": 0.1, "vy": 0.0, "flags": 0xE0C0, "sub": 0x0400, "age": 36, "qual": 25, "cls": 85})
  with logger._lock:
    logger.in_flight = ("F4", 5, 16, 6, t_chg - 5_000_000)
  rep_chg = b"\xFF" + bytes(ram[5 * STRIDE + 16 : 5 * STRIDE + 22])
  logger.observe_can([(t_chg, [(XCP_DTO, rep_chg, XCP_BUS)])], t_chg, v_ego=15.0)
  assert rec["kin_updated_ns"] == t_chg
  assert rec["kin_changed_ns"] == t_chg
  assert rec["kin_value_changes"] == cnt0 + 1
  assist_mid = json.loads(assist_path.read_text())
  assert assist_mid["mono_ns"] == t_chg
  assert assist_mid["tracks_by_op_id"]["12"]["kin_changed_ns"] == t_chg
  assert assist_mid["tracks_by_op_id"]["12"]["kin_value_changes"] == cnt0 + 1

  # Next 20 Hz log carries kin_changed_ns and kin_value_changes.
  epoch2_ns = max(t_chg + 60_000_000, logger.last_log_ns + 60_000_000)
  logger.observe_radar_tracks(rd, epoch2_ns)
  logger.observe_can([], epoch2_ns, v_ego=15.0)
  t2_meta = next(tgt for tgt in rows[-1]["targets"] if tgt["slot"] == 5)
  assert t2_meta["kin_changed_ns"] == t_chg
  assert t2_meta["kin_value_changes"] == cnt0 + 1

  # Age rollover resets kin_changed_ns and kin_value_changes for the new track lifecycle.
  t_roll = epoch2_ns + 10_000_000
  _write_slot(ram, 5, {"x": 29.8, "vx": -3.25, "ax": -3.0, "y": 0.1, "vy": 0.0, "flags": 0xE0C0, "sub": 0x0400, "age": 1, "qual": 25, "cls": 85})
  with logger._lock:
    logger.in_flight = ("F4", 5, 2, 6, t_roll - 5_000_000)
  rep_roll = b"\xFF" + bytes(ram[5 * STRIDE + 2 : 5 * STRIDE + 8])
  logger.observe_can([(t_roll, [(XCP_DTO, rep_roll, XCP_BUS)])], t_roll, v_ego=15.0)
  assert rec["kin_changed_ns"] == 0
  assert rec["kin_value_changes"] == 0

  t_after = t_roll + 15_000_000
  with logger._lock:
    logger.in_flight = ("F4", 5, 16, 6, t_after - 5_000_000)
  rep_after = b"\xFF" + bytes(ram[5 * STRIDE + 16 : 5 * STRIDE + 22])
  logger.observe_can([(t_after, [(XCP_DTO, rep_after, XCP_BUS)])], t_after, v_ego=15.0)
  assert rec["kin_changed_ns"] == t_after
  assert rec["kin_value_changes"] == 1
  logger.close()


def test_timeout_drain_discards_late_wayb_reply_without_corrupting_lead_kin(monkeypatch, tmp_path):
  _capture_events(monkeypatch)
  ram = bytearray(NUM_SLOTS * STRIDE)
  _write_slot(ram, 4, {"x": 32.0, "vx": -1.5, "ax": -0.8, "y": 0.1, "vy": 0.0, "flags": 0xE0C0, "sub": 0x0400, "age": 35, "qual": 25, "cls": 85})

  cp = SimpleNamespace(brand="honda", flags=int(HondaFlags.BOSCH), passive=False)
  logger = XcpRadarLogger(
    cp,
    probe_wayb=True,
    shm_path=tmp_path / "points.json",
    assist_shm_path=tmp_path / "assist.json",
    assist_fast_publish=True,
  )
  t = _unlock_logger(logger, 1_000_000_000, v_ego=15.0)
  rd = SimpleNamespace(points=[SimpleNamespace(trackId=22, dRel=28.6, yRel=0.1, vRel=-1.5)])

  for _ in range(25):
    logger.observe_radar_tracks(rd, t)
    sends = logger.next_can_sends(t)
    if sends:
      pkt = bytes(sends[0].dat)
      sz = pkt[1]
      addr = int.from_bytes(pkt[4:8], "big")
      if REC0 <= addr < REC0 + NUM_SLOTS * STRIDE:
        off = addr - REC0
        rep = b"\xFF" + bytes(ram[off : off + sz])
      else:
        rep = b"\xFF" + bytes(sz)
      logger.observe_can([(t, [(XCP_DTO, rep, XCP_BUS)])], t + 10_000_000, v_ego=15.0)
    t += 20_000_000

  assert logger.state == "READY"
  assert logger.slot_cache[4]["X_raw"] == int(round(32.0 * 128))
  assert logger.slot_cache[4]["Vx_raw"] == int(round(-1.5 * 256))
  logger._assist_shm_dirty = False

  # Simulate in-flight 6-byte WAYB t1_kin read timing out at 150 ms, followed by its late reply at 160 ms.
  t_sent = t + 20_000_000
  with logger._lock:
    logger.in_flight = ("WAYB", "t1_kin", xcp_mod.WAYB_DEFAULT_T1_BASE + 104 * 7 + 24, 6, t_sent)
    logger._wayb_queue.append(("t1_vx", xcp_mod.WAYB_DEFAULT_T1_BASE + 104 * 7 + 40, 4))

  t_to = t_sent + xcp_mod.F4_TIMEOUT_NS
  assert logger.next_can_sends(t_to) == []
  assert logger.in_flight is None
  assert logger.consecutive_f4_timeouts == 1
  assert len(logger._wayb_queue) == 0

  # During the 50 ms drain window, next_can_sends must stay silent so no fresh F4 CTO is in flight.
  assert logger.next_can_sends(t_to + 1_000_000) == []

  # Late 6-byte WAYB t1_kin reply arrives 10 ms after timeout (160 ms after t_sent) and must be discarded.
  t_late = t_to + 10_000_000
  late_t1_reply = (
    b"\xFF"
    + int(round(99.0 * 128)).to_bytes(2, "big", signed=True)
    + (0).to_bytes(2, "big", signed=True)
    + int(round(-12.0 * 256)).to_bytes(2, "big", signed=True)
  )
  logger.observe_can([(t_late, [(XCP_DTO, late_t1_reply, XCP_BUS)])], t_late, v_ego=15.0)
  assert logger.slot_cache[4]["X_raw"] == int(round(32.0 * 128))
  assert logger.slot_cache[4]["Vx_raw"] == int(round(-1.5 * 256))
  assert logger._assist_shm_dirty is False
  assert logger.consecutive_f4_timeouts == 1
  assert logger.next_can_sends(t_to + xcp_mod.F4_TIMEOUT_DRAIN_NS - 1_000_000) == []

  # Once drain window expires, scheduler resumes with primary lead F4 (slot 4, offset 16, size 6).
  t_resume = t_to + xcp_mod.F4_TIMEOUT_DRAIN_NS
  logger.observe_radar_tracks(rd, t_resume)
  sends_after = logger.next_can_sends(t_resume)
  assert len(sends_after) == 1
  assert logger.in_flight is not None and logger.in_flight[:4] == ("F4", 4, 16, 6)

  _write_slot(ram, 4, {"x": 31.5, "vx": -1.6, "ax": -0.7, "y": 0.1, "vy": 0.0, "flags": 0xE0C0, "sub": 0x0400, "age": 36, "qual": 25, "cls": 85})
  rep_fresh = b"\xFF" + bytes(ram[4 * STRIDE + 16 : 4 * STRIDE + 22])
  logger.observe_can([(t_resume + 15_000_000, [(XCP_DTO, rep_fresh, XCP_BUS)])], t_resume + 15_000_000, v_ego=15.0)
  assert logger.slot_cache[4]["X_raw"] == int(round(31.5 * 128))
  assert logger.slot_cache[4]["Vx_raw"] == int(round(-1.6 * 256))
  assert logger.consecutive_f4_timeouts == 0
  assert logger.state == "READY"

  # 5th consecutive timeout transitions to NEED_CONNECT and still enforces the drain before CONNECT.
  t_f5 = t_resume + 2_500_000_000
  with logger._lock:
    logger.consecutive_f4_timeouts = 4
    logger.in_flight = ("F4", 4, 16, 6, t_f5)
  t_f5_to = t_f5 + xcp_mod.F4_TIMEOUT_NS
  assert logger.next_can_sends(t_f5_to) == []
  assert logger.state == "NEED_CONNECT"
  assert logger.next_can_sends(t_f5_to + 1_000_000) == []

  logger.observe_can([(t_f5_to + 10_000_000, [(XCP_DTO, b"\xFF" + bytes(7), XCP_BUS)])], t_f5_to + 10_000_000, v_ego=15.0)
  assert logger.state == "NEED_CONNECT"

  sends_conn = logger.next_can_sends(t_f5_to + xcp_mod.F4_TIMEOUT_DRAIN_NS)
  assert len(sends_conn) == 1
  assert bytes(sends_conn[0].dat) == b"\xFF\x00" + bytes(6)
  assert logger.in_flight is not None and logger.in_flight[0] == "CONNECT"
  logger.close()


def test_timeout_and_drain_discard_counters_logged_in_20hz_event(monkeypatch):
  rows = _capture_events(monkeypatch)
  ram = bytearray(NUM_SLOTS * STRIDE)
  _write_slot(ram, 4, {"x": 32.0, "vx": -1.5, "ax": -0.8, "y": 0.1, "vy": 0.0, "flags": 0xE0C0, "sub": 0x0400, "age": 35, "qual": 25, "cls": 85})

  cp = SimpleNamespace(brand="honda", flags=int(HondaFlags.BOSCH), passive=False)
  logger = XcpRadarLogger(cp, probe_wayb=True)
  t = _unlock_logger(logger, 1_000_000_000, v_ego=15.0)
  for _ in range(15):
    sends = logger.next_can_sends(t)
    if sends:
      pkt = bytes(sends[0].dat)
      sz = pkt[1]
      addr = int.from_bytes(pkt[4:8], "big")
      off = addr - REC0
      rep = b"\xFF" + (bytes(ram[off : off + sz]) if REC0 <= addr < REC0 + NUM_SLOTS * STRIDE else bytes(sz))
      logger.observe_can([(t, [(XCP_DTO, rep, XCP_BUS)])], t + 10_000_000, v_ego=15.0)
    t += 20_000_000

  # 1) WAYB timeout -> wayb_timeout_count=1, consecutive_f4_timeouts=1, logged while consecutive > 0.
  t_wb = t + 20_000_000
  with logger._lock:
    logger.in_flight = ("WAYB", "t1_kin", xcp_mod.WAYB_DEFAULT_T1_BASE + 24, 6, t_wb)
  t_wb_to = t_wb + xcp_mod.F4_TIMEOUT_NS
  assert logger.next_can_sends(t_wb_to) == []
  assert logger.wayb_timeout_count == 1
  assert logger.consecutive_f4_timeouts == 1
  epoch1 = max(t_wb_to + 5_000_000, logger.last_log_ns + xcp_mod.EPOCH_NS)
  logger.observe_can([], epoch1, v_ego=15.0)
  assert rows[-1]["wayb_timeouts"] == 1
  assert rows[-1]["consecutive_f4_timeouts"] == 1
  assert rows[-1]["f4_timeouts"] == 0
  assert rows[-1]["handshake_timeouts"] == 0
  assert rows[-1]["drain_discards"] == 0

  # Late 0xFF DTO during drain -> drain_discards=1; non-DTO 0xAA during drain -> unchanged.
  logger.observe_can([(t_wb_to + 10_000_000, [(XCP_DTO, b"\xFF" + bytes(6), XCP_BUS)])], t_wb_to + 10_000_000, v_ego=15.0)
  logger.observe_can([(t_wb_to + 12_000_000, [(XCP_DTO, b"\xAA" + bytes(6), XCP_BUS)])], t_wb_to + 12_000_000, v_ego=15.0)
  assert logger.drain_discard_count == 1

  # Unsolicited 0xFF DTO after drain window expires (with in_flight is None) -> NOT counted as drain discard.
  t_after_drain = max(epoch1, t_wb_to + xcp_mod.F4_TIMEOUT_DRAIN_NS) + 5_000_000
  logger.observe_can([(t_after_drain, [(XCP_DTO, b"\xFF" + bytes(6), XCP_BUS)])], t_after_drain, v_ego=15.0)
  assert logger.drain_discard_count == 1

  # 2) F4 timeout -> f4_timeout_count=1, consecutive_f4_timeouts=2; late 0xFE during drain -> drain_discards=2, stays READY.
  t_f4 = t_after_drain + 5_000_000
  with logger._lock:
    logger.in_flight = ("F4", 4, 16, 6, t_f4)
  t_f4_to = t_f4 + xcp_mod.F4_TIMEOUT_NS
  assert logger.next_can_sends(t_f4_to) == []
  assert logger.f4_timeout_count == 1
  assert logger.consecutive_f4_timeouts == 2
  logger.observe_can([(t_f4_to + 10_000_000, [(XCP_DTO, b"\xFE\x22", XCP_BUS)])], t_f4_to + 10_000_000, v_ego=15.0)
  assert logger.drain_discard_count == 2
  assert logger.state == "READY"

  # 3) Valid F4 reply resets consecutive_f4_timeouts to 0 while keeping cumulative counts.
  t_ok = t_f4_to + xcp_mod.F4_TIMEOUT_DRAIN_NS + 5_000_000
  with logger._lock:
    logger.in_flight = ("F4", 4, 16, 6, t_ok - 5_000_000)
  logger.observe_can([(t_ok, [(XCP_DTO, b"\xFF" + bytes(ram[4 * STRIDE + 16 : 4 * STRIDE + 22]), XCP_BUS)])], t_ok, v_ego=15.0)
  assert logger.consecutive_f4_timeouts == 0

  # 4) HANDSHAKE (STATUS) timeout -> handshake_timeout_count=1, reconnects cleanly, preserves all cumulative counters.
  t_hs = t_ok + 2_500_000_000
  with logger._lock:
    logger.state = "NEED_STATUS"
    logger.in_flight = ("STATUS", -1, 0, 0, t_hs)
  t_hs_to = t_hs + xcp_mod.HANDSHAKE_RETRY_NS
  assert logger.next_can_sends(t_hs_to) == []
  assert logger.state == "NEED_CONNECT"
  assert logger.handshake_timeout_count == 1
  t_reconn = t_hs_to + xcp_mod.F4_TIMEOUT_DRAIN_NS
  for _ in range(4):
    sends = logger.next_can_sends(t_reconn)
    assert len(sends) == 1
    pid = bytes(sends[0].dat)[0]
    rep = {
      0xFF: b"\xFF\x05\x01\x08\x00\x08\x01\x01",
      0xFD: b"\xFF\x00\x05\x00\x00\x00",
      0xF8: b"\xFF\x00\x12\x34\x56\x78",
      0xF7: b"\xFF\x01",
    }[pid]
    logger.observe_can([(t_reconn, [(XCP_DTO, rep, XCP_BUS)])], t_reconn + 10_000_000, v_ego=15.0)
    t_reconn += 20_000_000
  assert logger.state == "READY"
  epoch2 = max(t_reconn + 10_000_000, logger.last_log_ns + xcp_mod.EPOCH_NS)
  logger.observe_can([], epoch2, v_ego=15.0)
  ev = rows[-1]
  assert ev["consecutive_f4_timeouts"] == 0
  assert ev["f4_timeouts"] == 1
  assert ev["wayb_timeouts"] == 1
  assert ev["handshake_timeouts"] == 1
  assert ev["drain_discards"] == 2
  logger.close()


def test_inpath_fallback_lead_not_starved_by_offpath_op_match(monkeypatch):
  rows = _capture_events(monkeypatch)
  ram = bytearray(NUM_SLOTS * STRIDE)
  # Crawl/moving in-path fallback lead (slot 4, unmatched in CAN) vs off-path CAN-matched target (slot 1 at y=-7.3 m)
  # plus 6 adjacent/clutter slots that compete for near_moving_others / init_queue when v_ego < 3.0.
  _write_slot(ram, 1, {"x": 23.6, "vx": 3.5, "ax": 0.0, "y": -7.3, "vy": 0.0, "flags": 0xE0C0, "sub": 0x0000, "age": 40, "qual": 25, "cls": 82})
  _write_slot(ram, 4, {"x": 25.0, "vx": -2.1, "ax": -0.5, "y": 0.5, "vy": 0.0, "flags": 0xE0C0, "sub": 0x0000, "age": 89, "qual": 25, "cls": 82})
  for s in range(10, 16):
    _write_slot(
      ram,
      s,
      {"x": 12.0 + s, "vx": -2.2, "ax": 0.0, "y": -3.2 if s % 2 == 0 else 3.3, "vy": 0.0, "flags": 0xE0C0, "sub": 0x0000, "age": 20, "qual": 20, "cls": 70},
    )

  cp = SimpleNamespace(brand="honda", flags=int(HondaFlags.BOSCH), passive=False)
  logger = XcpRadarLogger(cp)
  t = _unlock_logger(logger, 1_000_000_000, v_ego=2.2)
  t_unlock = t
  rd = SimpleNamespace(points=[SimpleNamespace(trackId=13, dRel=23.6, yRel=-7.3, vRel=3.5)])

  for i in range(200):  # 4.0 s at 20 ms RTT
    if i >= 15:
      logger.observe_radar_tracks(rd, t)
    sends = logger.next_can_sends(t)
    if sends:
      pkt = bytes(sends[0].dat)
      sz = pkt[1]
      off = int.from_bytes(pkt[4:8], "big") - REC0
      rep = b"\xFF" + bytes(ram[off : off + sz])
      logger.observe_can([(t, [(XCP_DTO, rep, XCP_BUS)])], t + 19_000_000, v_ego=2.2)
    t += 20_000_000

  prio = logger._select_priority_slots_locked()
  assert prio[:2] == [4, 1]
  post_warmup = [r for r in rows if r["mono_ns"] >= t_unlock + 1_500_000_000]
  assert len(post_warmup) >= 35
  assert all(
    r["lead"] is not None
    and r["lead"]["lead_source"] == "xcp_fallback"
    and r["lead"]["slot"] == 4
    and r["lead"]["stage"] == "measured"
    for r in post_warmup
  )
  assert max(r["lead"]["kin_age_ms"] for r in post_warmup) <= 120.0
  logger.close()

  # 2) Approaching stopped in-path fallback (y=1.6 m) must outrank adjacent moving traffic outside 2.0 m (y=2.35 m).
  logger2 = XcpRadarLogger(cp)
  logger2.state = "READY"
  logger2.last_v_ego = 5.0
  t0 = 10_000_000_000
  for s, x_m, y_m, vx_mps in ((6, 80.0, 2.35, 5.0), (28, 40.0, 1.6, -5.0)):
    logger2.slot_cache[s].update({
      "X_raw": int(round(x_m * 128)),
      "Y_raw": int(round(y_m * 128)),
      "Vx_raw": int(round(vx_mps * 256)),
      "age": 30,
      "flags": 0xE0C0,
      "subflags": 0x0000,
      "flags_updated_ns": t0,
      "age_updated_ns": t0,
      "lat_updated_ns": t0,
      "kin_updated_ns": t0,
      "qual_updated_ns": t0,
    })
    logger2.active_slots[s] = logger2.slot_cache[s]
  logger2._op_matched_slots = []
  logger2._op_lead_slot = None
  assert logger2._select_priority_slots_locked()[0] == 28

  # 3) Unmatched in-path CAN op_track still uses INACTIVE_PROBE_PERIOD_NO_LEAD (3) when off-path CAN is matched.
  logger2.slot_cache[1].update({
    "X_raw": int(round(23.6 * 128)),
    "Y_raw": int(round(-7.3 * 128)),
    "Vx_raw": int(round(3.5 * 256)),
    "age": 30,
    "flags": 0xE0C0,
    "flags_updated_ns": t0,
    "age_updated_ns": t0,
    "lat_updated_ns": t0,
    "kin_updated_ns": t0,
    "qual_updated_ns": t0,
  })
  logger2.active_slots[1] = logger2.slot_cache[1]
  logger2._op_matched_slots = [1]
  logger2._op_lead_slot = None
  logger2.observe_radar_tracks(SimpleNamespace(points=[SimpleNamespace(trackId=99, dRel=35.0, yRel=0.3, vRel=-1.0)]), t0)
  logger2._read_seq = 2
  logger2._last_probe_seq = 0
  probe = logger2._next_read_task_locked(t0 + 5_000_000)
  assert probe is not None and probe[0] not in (1, 6, 28) and probe[1:] == (16, 6)

  # 4) Matched in-path op_lead always takes prio_slots[0] over closer moving in-path fallback.
  logger2.slot_cache[3].update({
    "X_raw": int(round(15.0 * 128)),
    "Y_raw": int(round(0.2 * 128)),
    "Vx_raw": int(round(-0.5 * 256)),
    "age": 30,
    "flags": 0xE0C0,
    "flags_updated_ns": t0,
    "age_updated_ns": t0,
    "lat_updated_ns": t0,
    "kin_updated_ns": t0,
    "qual_updated_ns": t0,
  })
  logger2.active_slots[3] = logger2.slot_cache[3]
  logger2._op_lead_slot = 28
  logger2._op_matched_slots = [28, 1]
  assert logger2._select_priority_slots_locked()[0] == 28

  # 5) 0x1000 (conf_band="low", 0xF040) on a close in-path fallback must NOT demote it below
  # off-path CAN matches or farther 0xE0C0 in-path targets (regression for route 00000105 bug).
  logger2._op_lead_slot = None
  logger2._op_matched_slots = [1]
  logger2.slot_cache[28]["flags"] = 0xE0C0
  logger2.slot_cache[3]["flags"] = 0xF040
  assert logger2._select_priority_slots_locked()[0] == 3
  logger2.close()


def test_cutin_prio1_outranks_far_inpath_preempts_near_moving_and_yields_wayb(monkeypatch):
  rows = _capture_events(monkeypatch)
  monkeypatch.setattr(xcp_mod, "WAYB_PROBE_ENABLED", True)  # Way B is off by default (Peter 2026-11-23); this test exercises the inert probe code
  cp = SimpleNamespace(brand="honda", flags=int(HondaFlags.BOSCH), passive=False)
  logger = XcpRadarLogger(cp, probe_wayb=True)
  logger.state = "READY"
  logger.last_v_ego = 18.0
  t0 = 20_000_000_000

  def _seed(s: int, *, x: float, y: float, vx: float, vy: float, flags: int = 0xE0C0, kin_ts: int = t0, lat_ts: int = t0) -> None:
    logger.slot_cache[s].update({
      "X_raw": int(round(x * 128)),
      "Y_raw": int(round(y * 128)),
      "Vx_raw": int(round(vx * 256)),
      "Vy_raw": int(round(vy * 256)),
      "age": 30,
      "flags": flags,
      "subflags": 0x0400 if abs(y) < 1.8 else 0x0000,
      "flags_updated_ns": t0,
      "age_updated_ns": t0,
      "lat_updated_ns": lat_ts,
      "kin_updated_ns": kin_ts,
      "qual_updated_ns": t0,
    })
    logger.active_slots[s] = logger.slot_cache[s]

  # slot 2: matched in-path op_lead
  # slot 7: near CAN-matched laterally moving cut-in (d=24 m, y=3.2 m, vy=-0.5 m/s)
  # slot 8: closer laterally steady CAN-matched adjacent (d=20 m, y=-3.2 m, vy=0.0 m/s)
  # slot 9: far CAN-matched slot (d=60 m, y=3.0 m, vy=-0.6 m/s)
  # slot 10: far unmatched in-path slot (d=70 m, y=0.5 m, vy=0.0 m/s)
  # slot 11: stale near_moving_other competing for bandwidth (kin_age = 500 ms)
  _seed(2, x=45.0, y=0.2, vx=-1.0, vy=0.0)
  _seed(7, x=24.0, y=3.2, vx=-2.0, vy=-0.5)
  _seed(8, x=20.0, y=-3.2, vx=-1.5, vy=0.0)
  _seed(9, x=60.0, y=3.0, vx=-2.0, vy=-0.6)
  _seed(10, x=70.0, y=0.5, vx=-1.0, vy=0.0)
  _seed(11, x=35.0, y=-4.5, vx=-1.0, vy=0.0, kin_ts=t0 - 500_000_000, lat_ts=t0 - 500_000_000)

  # 1) Ranking: near moving cut-in (slot 7) wins prio[1] over far in-path (10), far match (9), and steady match (8).
  logger._op_lead_slot = 2
  logger._op_matched_slots = [2, 8, 9, 7]
  assert logger._select_priority_slots_locked() == [2, 7]

  # 2) Scheduler anti-starvation vs meta promotion:
  # When another near_moving_other (slot 11) is stale on kin/lat, it runs ahead of cut-in slot 7 at 110 ms.
  logger._last_probe_seq = logger._read_seq
  logger.slot_cache[7]["kin_updated_ns"] = t0 - xcp_mod.LEAD2_KIN_INTERVAL_NS
  assert logger._next_read_task_locked(t0) == (11, 22, 6)

  # Once near_moving_others are fresh on kin/lat (only stale on meta), cut-in slot 7 runs kin/lat ahead of meta.
  logger.slot_cache[11]["kin_updated_ns"] = t0
  logger.slot_cache[11]["lat_updated_ns"] = t0
  logger.slot_cache[11]["age_updated_ns"] = t0 - 1_500_000_000
  assert logger._next_read_task_locked(t0) == (7, 16, 6)
  logger.slot_cache[7]["kin_updated_ns"] = t0
  logger.slot_cache[7]["lat_updated_ns"] = t0 - xcp_mod.LEAD2_LAT_INTERVAL_NS
  assert logger._next_read_task_locked(t0) == (7, 22, 6)

  # Non-cut-in prio[1] (far off-path d=60 m, y=8.0 m) does NOT preempt near_moving_others meta (slot 11).
  logger.slot_cache[7]["lat_updated_ns"] = t0
  logger._op_matched_slots = [2, 12]
  _seed(12, x=60.0, y=8.0, vx=-1.5, vy=0.0, kin_ts=t0 - xcp_mod.LEAD2_KIN_INTERVAL_NS - 50_000_000)
  assert logger._select_priority_slots_locked() == [2, 12]
  assert logger._next_read_task_locked(t0) == (11, 2, 6)

  # 3) Direct Way B cut-in yield in _build_next_cto_locked: primary urgency stays False;
  # yields single Way B read when cut-in prio[1] kin_age >= 140 ms while leaving _wayb_queue queued.
  logger._tx_armed = True
  logger._tx_armed_ns = t0 - 1_000_000_000
  logger.in_flight = None
  logger.slot_cache[2]["kin_updated_ns"] = t0
  logger._op_matched_slots = [2, 7]
  wayb_entry = ("t1_kin", xcp_mod.WAYB_DEFAULT_T1_BASE + 24, 6)
  logger._wayb_queue.append(wayb_entry)
  logger.slot_cache[7]["kin_updated_ns"] = t0 - (xcp_mod.CUTIN_WAYB_YIELD_NS - 1_000_000)
  assert logger._primary_lead_urgent_locked(t0) is False
  assert logger._build_next_cto_locked(t0) is not None
  assert logger.in_flight is not None and logger.in_flight[0] == "WAYB"
  assert len(logger._wayb_queue) == 0

  logger.in_flight = None
  logger._wayb_queue.append(wayb_entry)
  logger.slot_cache[7]["kin_updated_ns"] = t0 - xcp_mod.CUTIN_WAYB_YIELD_NS
  assert logger._primary_lead_urgent_locked(t0) is False
  assert logger._build_next_cto_locked(t0) is not None
  assert logger.in_flight is not None and logger.in_flight[:4] == ("F4", 7, 16, 6)
  assert len(logger._wayb_queue) == 1

  logger.in_flight = None
  logger._op_matched_slots = [2, 12]
  logger.slot_cache[12]["kin_updated_ns"] = t0 - 500_000_000
  assert logger._build_next_cto_locked(t0) is not None
  assert logger.in_flight is not None and logger.in_flight[0] == "WAYB"
  logger.close()

  # 4) End-to-end 3.6 s loop at 20 ms RTT: Way B still completes rounds and cut-in prio[1] stays fresh (<200 ms)
  # despite 4 competing near_moving_others and 1 Hz Way B rounds.
  rows.clear()
  ram = bytearray(NUM_SLOTS * STRIDE)
  _write_slot(ram, 2, {"x": 42.0, "vx": -1.0, "ax": 0.0, "y": 0.1, "vy": 0.0, "flags": 0xE0C0, "sub": 0x0400, "age": 35, "qual": 25, "cls": 85})
  _write_slot(ram, 7, {"x": 22.0, "vx": -2.5, "ax": -0.2, "y": 2.9, "vy": -0.6, "flags": 0xE0C0, "sub": 0x0000, "age": 25, "qual": 25, "cls": 80})
  for s in range(12, 16):
    _write_slot(
      ram,
      s,
      {"x": 20.0 + s, "vx": -1.5, "ax": 0.0, "y": -4.2 if s % 2 == 0 else 4.5, "vy": 0.0, "flags": 0xE0C0, "sub": 0x0000, "age": 20, "qual": 20, "cls": 70},
    )

  t1_base = xcp_mod.WAYB_DEFAULT_T1_BASE
  out18_base = xcp_mod.WAYB_DEFAULT_OUT18_BASE
  p_base = xcp_mod.WAYB_DEFAULT_P_BASE
  t1_slot = 7
  t1_slot7 = bytearray(104)
  t1_slot7[86:92] = (0x8800).to_bytes(2, "big") + bytes([0, 0, 0, 4])

  e2e = XcpRadarLogger(cp, probe_wayb=True)
  t = _unlock_logger(e2e, 1_000_000_000, v_ego=18.0)
  t_unlock = t
  rd = SimpleNamespace(points=[
    SimpleNamespace(trackId=101, dRel=38.6, yRel=0.1, vRel=-1.0),
    SimpleNamespace(trackId=202, dRel=18.6, yRel=2.9, vRel=-2.5),
  ])

  wayb_p0_reads = 0
  probe: list[float] = []  # true per-tick kin age of cut-in slot 7 (ms), independent of snapshot cadence
  lead_probe: list[float] = []  # true per-tick kin age of primary lead slot 2 (ms)
  for i in range(180):  # 3.6 s at 20 ms RTT
    if i >= 15:
      e2e.observe_radar_tracks(rd, t)
    sends = e2e.next_can_sends(t)
    if sends:
      pkt = bytes(sends[0].dat)
      sz = pkt[1]
      addr = int.from_bytes(pkt[4:8], "big")
      if REC0 <= addr < REC0 + NUM_SLOTS * STRIDE:
        off = addr - REC0
        rep = b"\xFF" + bytes(ram[off : off + sz])
      elif addr == p_base + 2 and sz == 7:
        wayb_p0_reads += 1
        rep = b"\xFF" + bytes([0x00, 0x01, 0x00, 0x02, 5, t1_slot, 0])
      elif (t1_base + 104 * t1_slot) <= addr and (addr + sz) <= (t1_base + 104 * (t1_slot + 1)):
        off = addr - (t1_base + 104 * t1_slot)
        rep = b"\xFF" + bytes(t1_slot7[off : off + sz])
      elif sz == 2 and ((addr - out18_base) % 18) == 16 and 0 <= ((addr - out18_base) // 18) < 32:
        rep = b"\xFF" + (bytes([101, 0x03]) if ((addr - out18_base) // 18) == t1_slot else bytes(2))
      else:
        raise AssertionError(f"Unexpected XCP probe address {addr:#x} size={sz}")
      e2e.observe_can([(t, [(XCP_DTO, rep, XCP_BUS)])], t + 19_000_000, v_ego=18.0)
    if t >= t_unlock + 1_200_000_000:
      with e2e._lock:
        probe.append((t - e2e.slot_cache[7]["kin_updated_ns"]) / 1e6)
        lead_probe.append((t - e2e.slot_cache[2]["kin_updated_ns"]) / 1e6)
    t += 20_000_000

  assert wayb_p0_reads >= 3
  post = [r for r in rows if r["mono_ns"] >= t_unlock + 1_200_000_000]
  assert len(post) >= 40
  assert all(r["lead"] is not None and r["lead"]["slot"] == 2 and r["lead"]["stage"] == "measured" for r in post)
  assert max(r["lead"]["kin_age_ms"] for r in post) <= 120.0
  assert max(lead_probe) <= 120.0
  cutin_targets = [next((tgt for tgt in r["targets"] if tgt["slot"] == 7), None) for r in post]
  assert all(tgt is not None for tgt in cutin_targets)
  cutin_ka = sorted(tgt["kin_age_ms"] for tgt in cutin_targets if tgt is not None)
  # Anti-starvation fair share keeps competing near_moving_others alive (tested above) while
  # direct Way B yield + CUTIN_URGENT_KIN_NS floor hold cut-in p50 <= 140 ms and true per-tick max <= 400 ms.
  assert cutin_ka[len(cutin_ka) // 2] <= 140.0
  assert sum(1 for ka in cutin_ka if ka > 200.0) <= 12
  assert cutin_ka[-1] <= 400.0
  assert max(probe) <= 400.0
  e2e.close()


def _capture_all(monkeypatch) -> list[tuple[str, dict]]:
  rows: list[tuple[str, dict]] = []
  monkeypatch.setattr(xcp_mod.cloudlog, "event", lambda name, **kw: rows.append((name, kw)))
  return rows


def _ready_logger(monkeypatch):
  rows = _capture_all(monkeypatch)
  cp = SimpleNamespace(brand="honda", flags=int(HondaFlags.BOSCH), passive=False)
  logger = XcpRadarLogger(cp, probe_wayb=True)
  t = _unlock_logger(logger, 1_000_000_000, v_ego=15.0)
  rows.clear()
  with logger._lock:
    logger._reads_buf.clear()
    logger._next_reads_flush_ns = 0
  return logger, rows, t


def test_20hz_grid_gate_with_jittered_ticks(monkeypatch):
  import random
  logger, rows, t0 = _ready_logger(monkeypatch)
  rng = random.Random(1)
  t0 += 1_000_000_000
  times = []
  for i in range(1000):
    now = t0 + i * 10_000_000 + rng.randint(-3_000_000, 3_000_000)
    logger.observe_can([], now, v_ego=15.0)
  times = [kw["mono_ns"] for n, kw in rows if n == "xcp_array_b_20hz"]
  assert abs(len(times) - 200) <= 2
  gaps = [b - a for a, b in zip(times, times[1:])]
  # ticks 10 ms +-3 ms jitter can be 16 ms apart, so a grid fire can land 16 ms late; floor is ~34 ms
  assert min(gaps) >= 30_000_000
  logger.close()


def test_20hz_grid_stall_does_not_burst(monkeypatch):
  logger, rows, t0 = _ready_logger(monkeypatch)
  t0 += 1_000_000_000
  for i in range(20):
    logger.observe_can([], t0 + i * 10_000_000, v_ego=15.0)
  n0 = len([1 for n, _ in rows if n == "xcp_array_b_20hz"])
  t1 = t0 + 20 * 10_000_000 + 1_000_000_000
  for i in range(30):
    logger.observe_can([], t1 + i * 10_000_000, v_ego=15.0)
  evs = [kw["mono_ns"] for n, kw in rows if n == "xcp_array_b_20hz"]
  after = [x for x in evs if x >= t1]
  assert len(evs) - n0 == len(after)
  assert after[0] == t1  # one event right after the stall
  assert 5 <= len(after) <= 7  # 300 ms -> ~6 events, no catch-up burst
  assert min(b - a for a, b in zip(after, after[1:])) >= 40_000_000
  logger.close()


def test_xcp_reads_records_f4_wayb_timeout(monkeypatch):
  logger, rows, t = _ready_logger(monkeypatch)
  t += 1_000_000_000
  with logger._lock:
    logger._next_reads_flush_ns = t + 250_000_000  # hold the first flush so all 3 records land in one event
  # F4 reply: sent at t, received 7 ms later
  with logger._lock:
    logger.in_flight = ("F4", 4, 16, 6, t)
  logger.observe_can([(t, [(XCP_DTO, b"\xFF" + bytes([1, 2, 3, 4, 5, 6]), XCP_BUS)])], t + 7_000_000, v_ego=15.0)
  # Way B reply
  tw = t + 20_000_000
  with logger._lock:
    logger.in_flight = ("WAYB", "t1_kin", xcp_mod.WAYB_DEFAULT_T1_BASE + 24, 3, tw)
  logger.observe_can([(tw, [(XCP_DTO, b"\xFF" + bytes([0xAA, 0xBB, 0xCC]), XCP_BUS)])], tw + 9_000_000, v_ego=15.0)
  # F4 timeout
  tt = tw + 100_000_000
  with logger._lock:
    logger.in_flight = ("F4", 5, 20, 2, tt)
  t_to = tt + xcp_mod.F4_TIMEOUT_NS
  assert logger.next_can_sends(t_to) == []
  assert not [r for r in rows if r[0] == "xcp_reads"]  # nothing flushed before the grid time
  logger.observe_can([], t_to + 300_000_000, v_ego=15.0)
  reads = [kw for n, kw in rows if n == "xcp_reads"]
  assert len(reads) == 1
  ev = reads[0]
  assert ev["n"] == 3 and ev["dropped"] == 0 and ev["debug"] is True
  assert ev["base_ns"] == t + 7_000_000
  r0, r1, r2 = ev["records"]
  assert r0 == [0, 0, 7000, 4, 16, 6, "010203040506"]
  assert r1 == [1, (tw + 9_000_000 - ev["base_ns"]) // 1000, 9000, "t1_kin", xcp_mod.WAYB_DEFAULT_T1_BASE + 24, 3, "aabbcc"]
  assert r2 == [2, (t_to - ev["base_ns"]) // 1000, xcp_mod.F4_TIMEOUT_NS // 1000, "F4", 5, 20, 2]
  # buffer was swapped: no repeat flush, no event while empty
  logger.observe_can([], t_to + 900_000_000, v_ego=15.0)
  assert len([1 for n, _ in rows if n == "xcp_reads"]) == 1
  logger.close()


def test_xcp_reads_overflow_counts_dropped(monkeypatch):
  logger, rows, t = _ready_logger(monkeypatch)
  t += 1_000_000_000
  with logger._lock:
    logger._next_reads_flush_ns = t + 250_000_000
  for i in range(xcp_mod.READS_BUF_MAX + 5):
    with logger._lock:
      logger.in_flight = ("F4", 4, 16, 6, t)
    logger.observe_can([(t, [(XCP_DTO, b"\xFF" + bytes(6), XCP_BUS)])], t + 1_000_000, v_ego=15.0)
  assert logger._reads_dropped == 5
  logger.observe_can([], t + 400_000_000, v_ego=15.0)
  ev = [kw for n, kw in rows if n == "xcp_reads"][0]
  assert ev["n"] == xcp_mod.READS_BUF_MAX and ev["dropped"] == 5
  logger.close()


def test_xcp_reads_log_switch_off(monkeypatch):
  monkeypatch.setattr(xcp_mod, "XCP_READS_LOG", False)
  logger, rows, t = _ready_logger(monkeypatch)
  t += 1_000_000_000
  with logger._lock:
    logger.in_flight = ("F4", 4, 16, 6, t)
  logger.observe_can([(t, [(XCP_DTO, b"\xFF" + bytes(6), XCP_BUS)])], t + 5_000_000, v_ego=15.0)
  with logger._lock:
    logger.in_flight = ("F4", 4, 16, 6, t)
  logger.next_can_sends(t + xcp_mod.F4_TIMEOUT_NS)
  logger.observe_can([], t + 1_000_000_000, v_ego=15.0)
  assert logger._reads_buf == []
  assert not [r for r in rows if r[0] == "xcp_reads"]
  assert [r for r in rows if r[0] == "xcp_array_b_20hz"]
  logger.close()


def test_20hz_event_kwargs_keys_unchanged(monkeypatch):
  logger, rows, t = _ready_logger(monkeypatch)
  logger.observe_can([], t + 1_000_000_000, v_ego=15.0)
  evs = [kw for n, kw in rows if n == "xcp_array_b_20hz"]
  assert evs
  assert set(evs[0]) == {
    "mono_ns", "v_ego", "lead", "targets", "op_tracks", "consecutive_f4_timeouts",
    "f4_timeouts", "wayb_timeouts", "handshake_timeouts", "drain_discards", "debug",
  }
  logger.close()


def _protect_logger(monkeypatch):
  _capture_events(monkeypatch)
  cp = SimpleNamespace(brand="honda", flags=int(HondaFlags.BOSCH), passive=False)
  logger = XcpRadarLogger(cp)
  logger.state = "READY"
  logger.last_v_ego = 18.0
  t0 = 20_000_000_000

  def seed(s: int, *, x: float, y: float, vx: float = -2.0, vy: float = 0.0, kin_age_ms: float = 0.0, lat_ts: int = t0) -> None:
    logger.slot_cache[s].update({
      "X_raw": int(round(x * 128)), "Y_raw": int(round(y * 128)),
      "Vx_raw": int(round(vx * 256)), "Vy_raw": int(round(vy * 256)),
      "age": 30, "flags": 0xE0C0, "subflags": 0x0000,
      "flags_updated_ns": t0, "age_updated_ns": t0, "lat_updated_ns": lat_ts,
      "kin_updated_ns": t0 - int(kin_age_ms * 1e6), "qual_updated_ns": t0,
    })
    logger.active_slots[s] = logger.slot_cache[s]

  # lead slot 2: fresh everywhere so lead steps do not fire
  seed(2, x=45.0, y=0.2, vx=-1.0)
  logger._op_lead_slot = 2
  logger._last_probe_seq = logger._read_seq
  return logger, seed, t0


def test_cutin_protect_top3_band_candidates_and_not_fourth(monkeypatch):
  logger, seed, t0 = _protect_logger(monkeypatch)
  # four band candidates; time-to-path |y|/vy_toward: s5 1.5/1.0, s6 2.0/1.0, s7 2.5/1.0, s8 3.0/0.2
  seed(5, x=20.0, y=1.5, vy=-1.0)
  seed(6, x=22.0, y=-2.0, vy=1.0)
  seed(7, x=25.0, y=2.5, vy=-1.0)
  seed(8, x=30.0, y=3.0, vy=-0.2)
  prio = logger._select_priority_slots_locked()
  assert logger._cutin_protect_slots_locked(prio) == [5, 6, 7]
  # 4th (slot 8) at 200 ms is not protected: the protect step does not read it (stale-group ceiling is 350 ms)
  logger.slot_cache[8]["kin_updated_ns"] = t0 - 200_000_000
  assert 8 not in logger._cutin_protect_slots_locked(prio)
  # a protected one at 150 ms is read; the stalest of the protected set goes first
  logger.slot_cache[8]["kin_updated_ns"] = t0
  logger.slot_cache[5]["kin_updated_ns"] = t0 - xcp_mod.CUTIN_PROTECT_KIN_NS
  logger.slot_cache[6]["kin_updated_ns"] = t0 - xcp_mod.CUTIN_PROTECT_KIN_NS - 20_000_000
  assert logger._next_read_task_locked(t0) == (6, 16, 6)
  # just under the ceiling: not read by the protect step
  logger.slot_cache[5]["kin_updated_ns"] = t0 - xcp_mod.CUTIN_PROTECT_KIN_NS + 1_000_000
  logger.slot_cache[6]["kin_updated_ns"] = t0 - xcp_mod.CUTIN_PROTECT_KIN_NS + 1_000_000
  assert logger._next_read_task_locked(t0) not in ((6, 16, 6), (7, 16, 6))
  logger.close()


def test_cutin_protect_excludes_out_of_band_and_stationary(monkeypatch):
  logger, seed, t0 = _protect_logger(monkeypatch)
  seed(5, x=20.0, y=1.0, vy=-1.0)     # |y| 1.0 inside path width: not band
  seed(6, x=20.0, y=4.0, vy=-1.0)     # |y| 4.0 outside band
  seed(7, x=20.0, y=2.5, vx=-18.0, vy=-1.0)  # v_ego 18 + vrel -18 = stationary
  seed(8, x=45.0, y=2.5, vy=-1.0)     # d 45 m beyond band
  seed(9, x=20.0, y=2.5, vy=-1.0, lat_ts=0)  # lat never read
  prio = logger._select_priority_slots_locked()
  assert logger._cutin_protect_slots_locked(prio) == []
  for s in (5, 6, 7, 8, 9):
    logger.slot_cache[s]["kin_updated_ns"] = t0 - 300_000_000
  assert logger._cutin_protect_slots_locked(prio) == []
  assert logger._next_read_task_locked(t0) not in [(s, 16, 6) for s in (5, 6, 7, 8, 9)]
  logger.close()


def test_cutin_protect_lead_kin_still_preempts(monkeypatch):
  logger, seed, t0 = _protect_logger(monkeypatch)
  seed(5, x=20.0, y=1.5, vy=-1.0, kin_age_ms=300)
  assert logger._next_read_task_locked(t0) == (5, 16, 6)
  logger.slot_cache[2]["kin_updated_ns"] = t0 - xcp_mod.LEAD_KIN_INTERVAL_NS
  assert logger._next_read_task_locked(t0) == (2, 16, 6)
  logger.close()


def test_cutin_protect_yields_to_init_after_guard_wait(monkeypatch):
  logger, seed, t0 = _protect_logger(monkeypatch)
  seed(5, x=20.0, y=1.5, vy=-1.0, kin_age_ms=200)
  seed(12, x=60.0, y=5.0, vx=-2.0)
  init_task = (12, 22, 6)
  logger._init_queue.append(init_task)
  logger._init_queued.add(init_task)
  # init item first seen at t0: it has not waited past the guard, so protect still fires
  assert logger._next_read_task_locked(t0) == (5, 16, 6)
  # later, with everything else fresh and the protected slot stale again
  def refresh(now, kin_age_ms):
    for s_ in (2, 12):
      logger.slot_cache[s_].update({"kin_updated_ns": now, "lat_updated_ns": now, "qual_updated_ns": now,
                                    "age_updated_ns": now, "flags_updated_ns": now})
    logger.slot_cache[5].update({"kin_updated_ns": now - int(kin_age_ms * 1e6), "lat_updated_ns": now,
                                 "qual_updated_ns": now, "age_updated_ns": now, "flags_updated_ns": now})
    logger._protect_hist.clear()
  within = t0 + xcp_mod.CUTIN_PROTECT_GUARD_NS - 10_000_000
  refresh(within, 200)
  assert logger._next_read_task_locked(within) == (5, 16, 6)
  logger._init_queue.append(init_task)
  logger._init_queued.add(init_task)
  after = t0 + xcp_mod.CUTIN_PROTECT_GUARD_NS + 10_000_000
  refresh(after, 200)
  assert logger._next_read_task_locked(after) == init_task
  logger.close()


def test_wayb_probe_disabled_by_default_issues_zero_reads(monkeypatch):
  _capture_events(monkeypatch)
  assert xcp_mod.WAYB_PROBE_ENABLED is False
  cp = SimpleNamespace(brand="honda", flags=int(HondaFlags.BOSCH), passive=False)
  logger = XcpRadarLogger(cp, probe_wayb=True)
  assert logger._wayb_probe_enabled is False
  logger.state = "READY"
  logger.last_v_ego = 18.0
  t0 = 20_000_000_000
  logger._wayb_queue.append(("t1_kin", xcp_mod.WAYB_DEFAULT_T1_BASE + 24, 6))
  for i in range(200):
    now = t0 + i * 20_000_000
    logger.in_flight = None
    logger._build_next_cto_locked(now)
    if logger.in_flight is not None:
      assert logger.in_flight[0] != "WAYB"
  assert logger._next_wayb_task_locked(t0) is None
  assert logger._wayb_pending_locked(t0 + 5_000_000_000) is False


def test_build_cto_skips_lead_urgent_ranking_when_wayb_off_and_keeps_read_order(monkeypatch):
  # Step 1 read-efficiency (2026-11-23): the cheap Way B check runs first, so with Way B off the lead-urgent ranking
  # is not evaluated per read; the chosen read is the plain scheduler's choice.
  logger, seed, t0 = _protect_logger(monkeypatch)
  logger._tx_armed = True
  logger._tx_armed_ns = 0
  seed(5, x=20.0, y=1.5, vy=-1.0, kin_age_ms=300)
  calls = []
  real_urgent = logger._primary_lead_urgent_locked
  monkeypatch.setattr(logger, "_primary_lead_urgent_locked", lambda now: calls.append(now) or real_urgent(now))
  expected = XcpRadarLogger._next_read_task_locked(logger, t0)
  logger._read_seq -= 1
  pkt = logger._build_next_cto_locked(t0)
  assert calls == []
  slot, off, size = expected
  assert logger.in_flight[:4] == ("F4", slot, off, size)
  assert pkt == bytes([0xF4, size, 0x00, 0x00]) + (xcp_mod.REC0 + xcp_mod.STRIDE * slot + off).to_bytes(4, "big")
  logger.close()


def test_build_cto_lead_urgent_still_preempts_wayb_when_enabled(monkeypatch):
  monkeypatch.setattr(xcp_mod, "WAYB_PROBE_ENABLED", True)  # Way B is off by default; this checks the operand order keeps lead priority
  logger, seed, t0 = _protect_logger(monkeypatch)
  logger._wayb_probe_enabled = True
  logger._tx_armed = True
  logger._tx_armed_ns = 0
  logger._wayb_queue.append(("t1_kin", xcp_mod.WAYB_DEFAULT_T1_BASE + 24, 6))
  logger.slot_cache[2]["kin_updated_ns"] = t0 - xcp_mod.LEAD_KIN_INTERVAL_NS
  logger._build_next_cto_locked(t0)
  assert logger.in_flight[:4] == ("F4", 2, 16, 6)
  logger.close()


# --- Step 4: XCP SHORT_UPLOAD request pipelining (flag XCP_PIPELINE_DEPTH; default 1 == off) ---

def _kin6(x: float, vx: float = -1.0, ax: float = 0.0) -> bytes:
  """A size-6 kinematics (offset 16) SHORT_UPLOAD payload body, big-endian signed."""
  return (int(round(x * 128)).to_bytes(2, "big", signed=True)
          + int(round(vx * 256)).to_bytes(2, "big", signed=True)
          + int(round(ax * 2048)).to_bytes(2, "big", signed=True))


def test_pipeline_fifo_matches_two_outstanding_in_send_order(monkeypatch):
  # Depth 2: two outstanding untagged SHORT_UPLOADs are matched to replies strictly in send order.
  logger, seed, t0 = _protect_logger(monkeypatch)
  logger._pipeline_depth = 2
  # Arm two distinct kin reads: slot 5 first, slot 7 second.
  logger._arm_outstanding_locked(("F4", 5, 16, 6, t0))
  logger._arm_outstanding_locked(("F4", 7, 16, 6, t0 + 1_000_000))
  assert logger._outstanding_count_locked() == 2
  assert logger.in_flight[:4] == ("F4", 5, 16, 6)
  assert list(logger._inflight_tail)[0][:4] == ("F4", 7, 16, 6)

  # Reply #1 (for request #1, slot 5): X = 20.0 m. FIFO must assign it to slot 5 and promote slot 7.
  logger._process_dto_locked(b"\xFF" + _kin6(20.0), t0 + 5_000_000)
  assert logger.slot_cache[5]["X_raw"] == int(round(20.0 * 128))
  assert logger.slot_cache[7]["X_raw"] != int(round(20.0 * 128))  # slot 7 untouched by reply #1
  assert logger.in_flight[:4] == ("F4", 7, 16, 6)
  assert logger._outstanding_count_locked() == 1

  # Reply #2 (for request #2, slot 7): X = 35.0 m -> slot 7; slot 5 unchanged.
  logger._process_dto_locked(b"\xFF" + _kin6(35.0), t0 + 6_000_000)
  assert logger.slot_cache[7]["X_raw"] == int(round(35.0 * 128))
  assert logger.slot_cache[5]["X_raw"] == int(round(20.0 * 128))
  assert logger.in_flight is None
  assert logger._outstanding_count_locked() == 0
  logger.close()


def test_pipeline_issues_second_cto_only_while_first_outstanding_and_caps_at_depth(monkeypatch):
  # Depth 2 overlaps a second SHORT_UPLOAD while the first is still in flight; depth 1 does not.
  logger, seed, t0 = _protect_logger(monkeypatch)
  logger._pipeline_depth = 2
  logger._tx_armed = True
  logger._tx_armed_ns = 0
  seed(5, x=20.0, y=1.5, vy=-1.0, kin_age_ms=300)  # a real cut-in protect read task exists

  # Peek at the selector's natural pick (rolled back, no side effects), then make the head a
  # DIFFERENT field so the pipelined pick is distinct and may be issued.
  snap = logger._sched_snapshot_locked()
  natural_entry, _ = logger._select_ready_cto_locked(t0, arm=False)
  logger._sched_restore_locked(snap)
  head = ("F4", 9, 22, 6, t0) if natural_entry[1:3] != (9, 22) else ("F4", 9, 2, 6, t0)
  logger._arm_outstanding_locked(head)
  assert logger._outstanding_count_locked() == 1
  # No reply yet: depth 2 issues a second outstanding request to a different address.
  pkt2 = logger._build_next_cto_locked(t0 + 1_000_000)
  assert pkt2 is not None and logger._outstanding_count_locked() == 2
  assert list(logger._inflight_tail)[0][:3] == natural_entry[:3]
  # At the cap: no further request until a reply arrives.
  assert logger._build_next_cto_locked(t0 + 2_000_000) is None
  assert logger._outstanding_count_locked() == 2
  logger.close()

  # Contrast: default depth 1 never issues a second CTO while one is outstanding.
  logger1, seed1, t1 = _protect_logger(monkeypatch)
  assert logger1._pipeline_depth == 1
  logger1._tx_armed = True
  logger1._tx_armed_ns = 0
  seed1(5, x=20.0, y=1.5, vy=-1.0, kin_age_ms=300)
  assert logger1._build_next_cto_locked(t1) is not None
  assert logger1._build_next_cto_locked(t1 + 10_000_000) is None
  assert logger1._outstanding_count_locked() == 1
  logger1.close()


def _sched_state(logger):
  return (logger._read_seq, logger._last_probe_seq, logger.scan_cursor, logger._stationary_init_skip_count,
          logger._wayb_last_round_ns, list(logger._protect_hist), list(logger._init_queue),
          sorted(logger._init_queued), dict(logger._init_stamp), list(logger._wayb_queue))


def test_pipeline_second_request_must_target_a_different_slot_offset(monkeypatch):
  # SHORT_UPLOAD replies carry no address, so at depth > 1 the pipelined request may never repeat the
  # (slot, offset) of any outstanding request. A colliding pick sends nothing and leaves no trace.
  logger, seed, t0 = _protect_logger(monkeypatch)
  logger._pipeline_depth = 2
  logger._tx_armed = True
  logger._tx_armed_ns = 0
  seed(5, x=20.0, y=1.5, vy=-1.0, kin_age_ms=300)
  logger._enqueue_init_locked(13)  # selector state that a rejected pick must not consume

  # Idle pick goes out as the head (HEAD behavior).
  pkt1 = logger._build_next_cto_locked(t0)
  assert pkt1 is not None and logger._outstanding_count_locked() == 1
  head = logger.in_flight

  # Nothing replied: the selectors skip the outstanding target and issue a DISTINCT second read (no collision skip).
  skips0 = logger.pipeline_collision_skips
  assert logger._build_next_cto_locked(t0 + 1_000_000) is not None
  assert logger._outstanding_count_locked() == 2 and logger.in_flight == head
  assert logger._entry_addr(logger._inflight_tail[0]) != logger._entry_addr(head)
  assert logger.pipeline_collision_skips == skips0
  logger._inflight_tail.clear()

  # Backstop: a selector that ignores the exclude set still cannot repeat an address (rolled back, counted).
  real = logger._next_read_task_locked
  monkeypatch.setattr(logger, "_next_read_task_locked", lambda now, exclude=frozenset(): real(now))
  before = _sched_state(logger)
  assert logger._build_next_cto_locked(t0 + 2_000_000) is None
  assert logger._outstanding_count_locked() == 1 and logger.in_flight == head
  assert _sched_state(logger) == before
  assert logger.pipeline_collision_skips == skips0 + 1
  monkeypatch.undo()

  # Same rule against a tail entry and for a different size at the same start address.
  logger2, seed2, t2 = _protect_logger(monkeypatch)
  logger2._pipeline_depth = 3
  logger2._tx_armed = True
  logger2._tx_armed_ns = 0
  snap = logger2._sched_snapshot_locked()
  natural, _ = logger2._select_ready_cto_locked(t2, arm=False)
  logger2._sched_restore_locked(snap)
  logger2._arm_outstanding_locked(("F4", 9, 22, 6, t2) if natural[1:3] != (9, 22) else ("F4", 9, 2, 6, t2))
  logger2._arm_outstanding_locked(("F4", natural[1], natural[2], 2, t2))  # same (slot, offset), other size
  nxt = logger2._build_next_cto_locked(t2 + 1_000_000)
  assert nxt is not None and logger2._outstanding_count_locked() == 3
  assert len({logger2._entry_addr(e) for e in [logger2.in_flight, *logger2._inflight_tail]}) == 3
  logger2.close()

  # A reply clears the head; the same field is then legal again (single-in-flight, as HEAD).
  logger._process_dto_locked(b"\xFF" + _kin6(20.0)[:head[3]], t0 + 5_000_000)
  assert logger.in_flight is None
  assert logger._build_next_cto_locked(t0 + 6_000_000) is not None
  logger.close()


def test_pipeline_timeout_and_length_mismatch_flush_and_resync(monkeypatch):
  # A head timeout flushes the pipelined tail and resyncs to single-in-flight.
  logger, seed, t0 = _protect_logger(monkeypatch)
  logger._pipeline_depth = 2
  logger._tx_armed = True
  logger._tx_armed_ns = 0
  logger._arm_outstanding_locked(("F4", 5, 16, 6, t0))
  logger._arm_outstanding_locked(("F4", 7, 16, 6, t0 + 1_000_000))
  assert logger._outstanding_count_locked() == 2

  t_to = t0 + xcp_mod.F4_TIMEOUT_NS + 1
  assert logger._build_next_cto_locked(t_to) is None
  assert logger.in_flight is None
  assert len(logger._inflight_tail) == 0
  assert logger._outstanding_count_locked() == 0
  assert logger.consecutive_f4_timeouts == 1
  assert logger.f4_timeout_count == 1
  assert logger._timeout_drain_until_ns == t_to + xcp_mod.F4_TIMEOUT_DRAIN_NS
  logger.close()

  # A positive reply too short for the head's expected size also flushes + resyncs + arms the drain.
  logger2, seed2, t2 = _protect_logger(monkeypatch)
  logger2._pipeline_depth = 2
  logger2._arm_outstanding_locked(("F4", 5, 16, 6, t2))
  logger2._arm_outstanding_locked(("F4", 7, 22, 6, t2 + 1_000_000))
  assert logger2._outstanding_count_locked() == 2
  t_mm = t2 + 5_000_000
  logger2._process_dto_locked(b"\xFF" + bytes(4), t_mm)  # only 1+4 bytes, head expects size 6
  assert logger2.in_flight is None
  assert logger2._outstanding_count_locked() == 0
  assert logger2.slot_cache[5]["X_raw"] == 0  # nothing mis-decoded into slot 5
  assert logger2._timeout_drain_until_ns == t_mm + xcp_mod.F4_TIMEOUT_DRAIN_NS
  logger2.close()


def _drive_cto_capture(logger, ram, t, n, rtt_ns=9_000_000):
  """Drive n 10 ms ticks, replying each tick from the RAM image; capture the CTO byte stream."""
  cto_hex = []
  for _ in range(n):
    sends = logger.next_can_sends(t)
    if sends:
      pkt = bytes(sends[0].dat)
      cto_hex.append(pkt.hex())
      sz = pkt[1]
      off = int.from_bytes(pkt[4:8], "big") - REC0
      body = bytes(ram[off:off + sz]) if 0 <= off <= len(ram) - sz else bytes(sz)
      logger.observe_can([(t, [(XCP_DTO, b"\xFF" + body, XCP_BUS)])], t + rtt_ns, v_ego=15.0)
    t += 10_000_000
  return cto_hex


def test_pipeline_flag_off_schedule_is_byte_identical(monkeypatch):
  # With prompt replies (outstanding never exceeds 1), a depth-2 logger must produce the exact same
  # CTO byte stream as the default depth-1 logger: enabling the flag does not perturb the baseline.
  ram = bytearray(NUM_SLOTS * STRIDE)
  for s, cfg in {
    2: {"x": 45.0, "vx": -1.0, "ax": 0.0, "y": 0.2, "vy": 0.0, "flags": 0xE0C0, "sub": 0x0400, "age": 40, "qual": 25, "cls": 82},
    5: {"x": 20.0, "vx": -2.0, "ax": -0.5, "y": 1.6, "vy": -1.0, "flags": 0xE0C0, "sub": 0x0000, "age": 20, "qual": 22, "cls": 70},
    13: {"x": 30.0, "vx": -0.5, "ax": 0.0, "y": -3.0, "vy": 0.1, "flags": 0xE0C0, "sub": 0x0000, "age": 30, "qual": 24, "cls": 75},
  }.items():
    _write_slot(ram, s, cfg)

  cp = SimpleNamespace(brand="honda", flags=int(HondaFlags.BOSCH), passive=False)
  _capture_events(monkeypatch)

  base = XcpRadarLogger(cp)
  assert base._pipeline_depth == 1
  t = _unlock_logger(base, 1_000_000_000)
  base_seq = _drive_cto_capture(base, ram, t, 200)
  base.close()

  pipe = XcpRadarLogger(cp, pipeline_depth=2)
  assert pipe._pipeline_depth == 2
  t = _unlock_logger(pipe, 1_000_000_000)
  pipe_seq = _drive_cto_capture(pipe, ram, t, 200)
  pipe.close()

  # Depth 2 sends a read alone with a wire size that differs from the last completed read (widened by one
  # byte when allowed), so fold widened CTOs back to their logical read before comparing.
  def _logical(h):
    pkt = bytes.fromhex(h)
    addr = int.from_bytes(pkt[4:8], "big")
    if pkt[0] == 0xF4 and (addr + 1 - REC0) % STRIDE in xcp_mod.XCP_WIDEN_OFFSETS and pkt[1] >= 2:
      pkt = bytes([0xF4, pkt[1] - 1, 0, 0]) + (addr + 1).to_bytes(4, "big")
    return pkt.hex()
  assert base_seq == [_logical(h) for h in pipe_seq]
  assert len(base_seq) > 100  # the drive actually issued reads


def test_pipeline_selector_skips_outstanding_target_depth1_unchanged(monkeypatch):
  # A1: with the head outstanding, the second pick is the next-best distinct target and is not a collision.
  logger, seed, t0 = _protect_logger(monkeypatch)
  seed(5, x=20.0, y=1.5, vy=-1.0, kin_age_ms=300)
  snap = logger._sched_snapshot_locked()
  first = logger._next_read_task_locked(t0)
  logger._sched_restore_locked(snap)
  excl = frozenset({xcp_mod.REC0 + xcp_mod.STRIDE * first[0] + first[1]})
  second = logger._next_read_task_locked(t0, excl)
  assert second is not None and second != first
  assert xcp_mod.REC0 + xcp_mod.STRIDE * second[0] + second[1] not in excl
  logger._sched_restore_locked(snap)
  # Empty exclude set == no argument (depth-1 selection is unchanged).
  assert logger._next_read_task_locked(t0, frozenset()) == first
  logger.close()

  # Depth-1 read order: default logger vs one whose selector gets an explicit empty exclude set.
  ram = bytearray(NUM_SLOTS * STRIDE)
  _write_slot(ram, 2, {"x": 45.0, "vx": -1.0, "ax": 0.0, "y": 0.2, "vy": 0.0, "flags": 0xE0C0, "sub": 0x0400, "age": 40, "qual": 25, "cls": 82})
  _write_slot(ram, 5, {"x": 20.0, "vx": -2.0, "ax": -0.5, "y": 1.6, "vy": -1.0, "flags": 0xE0C0, "sub": 0x0000, "age": 20, "qual": 22, "cls": 70})
  cp = SimpleNamespace(brand="honda", flags=int(HondaFlags.BOSCH), passive=False)
  _capture_events(monkeypatch)
  a = XcpRadarLogger(cp)
  seq_a = _drive_cto_capture(a, ram, _unlock_logger(a, 1_000_000_000), 120)
  a.close()
  b = XcpRadarLogger(cp)
  real = b._next_read_task_locked
  monkeypatch.setattr(b, "_next_read_task_locked", lambda now, exclude=frozenset(): real(now, frozenset()))
  seq_b = _drive_cto_capture(b, ram, _unlock_logger(b, 1_000_000_000), 120)
  b.close()
  assert seq_a == seq_b and len(seq_a) > 50


def test_pipeline_refills_to_depth_in_one_wakeup(monkeypatch):
  # A2: at depth 2 one wakeup builds two CTOs (distinct targets); at depth 1 only one.
  logger, seed, t0 = _protect_logger(monkeypatch)
  logger._pipeline_depth = 2
  logger._tx_armed = True
  logger._tx_armed_ns = 0
  seed(5, x=20.0, y=1.5, vy=-1.0, kin_age_ms=300)
  pkts = logger._build_ctos_locked(t0)
  assert len(pkts) == 2 and len(set(pkts)) == 2
  assert logger._outstanding_count_locked() == 2
  assert logger.pipeline_collision_skips == 0
  assert logger._build_ctos_locked(t0 + 1_000_000) == []
  logger.close()

  logger1, seed1, t1 = _protect_logger(monkeypatch)
  logger1._tx_armed = True
  logger1._tx_armed_ns = 0
  seed1(5, x=20.0, y=1.5, vy=-1.0, kin_age_ms=300)
  assert len(logger1._build_ctos_locked(t1)) == 1
  logger1.close()


def test_pipeline_length_must_match_exactly_at_depth2(monkeypatch):
  # A3: an over-long reply is also a desync at depth > 1.
  logger, seed, t0 = _protect_logger(monkeypatch)
  logger._pipeline_depth = 2
  logger._arm_outstanding_locked(("F4", 5, 16, 6, t0))
  logger._arm_outstanding_locked(("F4", 7, 22, 6, t0 + 1_000_000))
  t_mm = t0 + 5_000_000
  logger._process_dto_locked(b"\xFF" + bytes(8), t_mm)  # 1+8 bytes, head expects 1+6
  assert logger._outstanding_count_locked() == 0
  assert logger.slot_cache[5]["X_raw"] == 0
  assert logger._timeout_drain_until_ns == t_mm + xcp_mod.F4_TIMEOUT_DRAIN_NS
  logger.close()


# --- depth 2: wire-size alternation, widened reads, length-mismatch detection -------------------------------------

def _pipe_sim(fault=None, fault_at=200, pad8=False, depth=2, ms=2500, nslots=3):
  """1 ms ticks against a fake radar that answers every 10 ms in send order. Returns (logger, wrong, same_size_pairs).
  wrong = accepted reads whose stored (slot, offset, size, bytes) differ from what the radar RAM holds there."""
  ram = bytearray(NUM_SLOTS * STRIDE)
  cfgs = {2: dict(x=45.0, vx=-1.0, ax=0.0, y=0.2, vy=0.0, flags=0xE0C0, sub=0x0400, age=40, qual=25, cls=82),
          5: dict(x=20.0, vx=-2.0, ax=-0.5, y=1.6, vy=-1.0, flags=0xE0C0, sub=0, age=20, qual=22, cls=70),
          13: dict(x=30.0, vx=-0.5, ax=0.0, y=-3.0, vy=0.1, flags=0xE0C0, sub=0, age=30, qual=24, cls=75)}
  for s in list(cfgs)[:nslots]:
    _write_slot(ram, s, cfgs[s])
  lg = XcpRadarLogger(SimpleNamespace(brand="honda", flags=int(HondaFlags.BOSCH), passive=False), pipeline_depth=depth)
  t = _unlock_logger(lg, 1_000_000_000)
  q, last, state, wrong, accepted, same = [], None, None, 0, 0, 0
  for i in range(ms):
    t += 1_000_000
    with lg._lock:
      if i % 10 == 0 and q:
        addr, size = q.pop(0)
        off = addr - REC0
        payload = b"\xFF" + bytes(ram[off:off + size])
        if pad8:
          payload = payload.ljust(8, b"\x00")
        if fault == "dup" and state == "armed":  # state-2 style: this CTO's reply is the previous one again
          state, (addr, size, payload) = "done", last
        elif fault == "dup" and state is None and i >= fault_at:
          state = "armed"
        last = (addr, size, payload)
        lg._process_dto_locked(payload, t)
        for kind, _, _, slot, offset, rsize, hexs in lg._reads_buf:
          accepted += 1
          a = REC0 + STRIDE * slot + offset
          wrong += hexs != bytes(ram[a - REC0:a - REC0 + rsize]).hex()
        lg._reads_buf.clear()
      for pkt in lg._build_ctos_locked(t):
        assert len(pkt) == 8 and pkt[0] == 0xF4
        if fault == "loss" and state is None and i >= fault_at:
          state = "done"
          continue  # this CTO never reaches the radar
        q.append((int.from_bytes(pkt[4:8], "big"), pkt[1]))
      wire = [lg._wire_size(e) for e in ([lg.in_flight] if lg.in_flight else []) + list(lg._inflight_tail)]
      same += len(wire) != len(set(wire))
  return lg, wrong, same, accepted


def test_depth2_outstanding_reads_never_share_a_wire_size_and_widened_reads_store_the_same_values():
  lg, wrong, same, accepted = _pipe_sim()
  assert same == 0
  assert accepted > 100 and wrong == 0
  assert lg.pipeline_widened_count > 0 and lg.pipeline_len_mismatch_count == 0
  assert lg._pipeline_depth == 2
  lg.close()


def test_depth2_lost_cto_or_stale_duplicate_is_caught_and_stores_nothing_to_the_wrong_slot():
  for fault in ("loss", "dup"):
    for fault_at in range(100, 260, 7):
      lg, wrong, same, _ = _pipe_sim(fault, fault_at, ms=900, nslots=5 if fault_at % 2 else 3)
      assert wrong == 0 and same == 0, (fault, fault_at)
      assert lg.pipeline_len_mismatch_count >= 1, (fault, fault_at)
      assert lg._pipeline_depth == 2  # one event does not trigger the fallback
      lg.close()


def test_depth2_three_consecutive_mismatches_fall_back_to_depth1_and_reads_resume():
  lg, wrong, same, accepted = _pipe_sim(pad8=True, ms=1000)  # 8-byte padded replies never match a 7-byte reply
  assert lg.pipeline_len_mismatch_count == 3 and lg._pipeline_depth == 1
  assert wrong == 0 and accepted > 50
  lg.close()


def test_depth2_accepted_reply_resets_the_mismatch_streak():
  lg, seed, t0 = _protect_logger(__import__("pytest").MonkeyPatch())
  lg._pipeline_depth = 2
  for n in range(5):  # mismatch, mismatch, good reply, repeat: never 3 in a row
    for k in range(2):
      lg._arm_outstanding_locked(("F4", 5, 16, 6, t0))
      lg._process_dto_locked(b"\xFF\x00", t0 + 1)
    lg._arm_outstanding_locked(("F4", 5, 16, 6, t0))
    lg._process_dto_locked(b"\xFF" + bytes(6), t0 + 2)
  assert lg._pipeline_depth == 2 and lg.pipeline_len_mismatch_count == 10
  lg.close()


def test_0xfe_with_outstanding_tail_arms_the_quiet_drain_and_clear_session_keeps_it():
  for cmd in ("F4", "WAYB"):
    lg, seed, t0 = _protect_logger(__import__("pytest").MonkeyPatch())
    lg._pipeline_depth = 2
    head = ("F4", 5, 16, 6, t0) if cmd == "F4" else ("WAYB", "t1_kin", REC0, 6, t0)
    lg._arm_outstanding_locked(head)
    lg._arm_outstanding_locked(("F4", 7, 22, 6, t0))
    lg._process_dto_locked(b"\xFE\x20", t0 + 1_000_000)
    assert lg._outstanding_count_locked() == 0
    assert lg._timeout_drain_until_ns == t0 + 1_000_000 + xcp_mod.F4_TIMEOUT_DRAIN_NS
    lg._process_dto_locked(b"\xFF" + bytes(6), t0 + 2_000_000)  # the dropped tail's reply is discarded
    assert lg.drain_discard_count == 1
    lg._clear_session_locked("NEED_CONNECT")
    assert lg._timeout_drain_until_ns == t0 + 1_000_000 + xcp_mod.F4_TIMEOUT_DRAIN_NS
    lg.close()


def test_depth2_probe_cursor_stays_on_a_blocked_slot():
  lg, seed, t0 = _protect_logger(__import__("pytest").MonkeyPatch())
  lg.scan_cursor = 7
  assert lg._next_inactive_probe_locked(frozenset({REC0 + STRIDE * 7 + 16})) is None
  assert lg.scan_cursor == 7
  assert lg._next_inactive_probe_locked() == (7, 16, 6) and lg.scan_cursor == 8
  lg.close()


def test_depth2_widened_read_answered_0xfe_falls_back_to_depth1_at_once(monkeypatch):
  logs = []
  monkeypatch.setattr(xcp_mod.cloudlog, "warning", lambda msg, *a, **k: logs.append(msg))
  lg, seed, t0 = _protect_logger(monkeypatch)
  lg._pipeline_depth = 2
  head = ("F4", 5, 16, 6, t0)
  lg._arm_outstanding_locked(head)
  lg._strip_entries.add(head)  # sent as (addr-1, size+1)
  lg._process_dto_locked(b"\xFE\x22", t0 + 1_000_000)
  assert lg._pipeline_depth == 1 and lg._outstanding_count_locked() == 0
  assert lg.slot_cache[5]["X_raw"] == 0 and not lg._reads_buf
  assert len(logs) == 1 and hex(REC0 + STRIDE * 5 + 15) in logs[0] and "size=7" in logs[0]
  # a plain (unwidened) 0xFE at depth 1 adds no second warning, and reads resume at depth 1
  lg._clear_session_locked("READY")
  lg._timeout_drain_until_ns = 0
  lg._tx_armed, lg._tx_armed_ns = True, 0
  pkts = lg._build_ctos_locked(t0 + 2_000_000_000)
  assert len(pkts) == 1 and lg._outstanding_count_locked() == 1 and not lg._strip_entries
  lg._process_dto_locked(b"\xFE\x22", t0 + 2_001_000_000)
  assert len(logs) == 1
  lg.close()


def test_depth2_read_sent_alone_after_a_completed_read_must_not_share_its_wire_size(monkeypatch):
  # X completes, the radar then repeats X's reply; A goes out alone with the same base size. Without the
  # remembered-size rule the stale copy has the right length and is stored as A's data.
  lg, seed, t0 = _protect_logger(monkeypatch)
  lg._pipeline_depth = 2
  lg._tx_armed, lg._tx_armed_ns = True, 0
  seed(5, x=20.0, y=1.5, vy=-1.0, kin_age_ms=300)
  assert lg._build_next_cto_locked(t0) is not None
  x = lg.in_flight
  reply = b"\xFF" + bytes(range(1, 1 + x[3]))
  lg._process_dto_locked(reply, t0 + 5_000_000)
  assert lg.in_flight is None
  reads0 = len(lg._reads_buf)
  pkt = lg._build_next_cto_locked(t0 + 6_000_000)
  assert pkt is not None and lg.in_flight[3] == x[3]  # same base size as X, sent alone
  assert pkt[1] != x[3]  # ... so it goes out widened
  lg._process_dto_locked(reply, t0 + 7_000_000)  # stale copy of X's reply
  assert len(lg._reads_buf) == reads0 and lg.pipeline_len_mismatch_count == 1
  lg.close()


def test_depth2_held_alone_read_goes_out_unwidened_after_the_quiet_window(monkeypatch):
  # No widening possible: a read with the last completed wire size is held (nothing sent, nothing consumed)
  # for at most the 50 ms quiet-drain window, then goes out alone and unwidened.
  lg, seed, t0 = _protect_logger(monkeypatch)
  monkeypatch.setattr(xcp_mod, "XCP_WIDEN_OFFSETS", frozenset())
  lg._pipeline_depth = 2
  lg._tx_armed, lg._tx_armed_ns = True, 0
  seed(5, x=20.0, y=1.5, vy=-1.0, kin_age_ms=300)
  assert lg._build_next_cto_locked(t0) is not None
  x = lg.in_flight
  done = t0 + 5_000_000
  lg._process_dto_locked(b"\xFF" + bytes(range(1, 1 + x[3])), done)
  lg._last_done_wire = x[3]  # same wire size as whatever the scheduler picks next
  lg._timeout_drain_until_ns = 0
  snap = _sched_state(lg)
  assert lg._build_next_cto_locked(done + 10_000_000) is None
  assert lg.in_flight is None and lg.pipeline_size_skips == 1 and _sched_state(lg) == snap
  pkt = lg._build_next_cto_locked(done + 51_000_000)
  assert pkt is not None and pkt[1] == lg.in_flight[3] == x[3] and not lg._strip_entries
  reads0 = len(lg._reads_buf)
  lg._process_dto_locked(b"\xFF" + bytes(range(9, 9 + x[3])), done + 60_000_000)
  assert len(lg._reads_buf) == reads0 + 1 and lg.pipeline_len_mismatch_count == 0
  lg.close()
