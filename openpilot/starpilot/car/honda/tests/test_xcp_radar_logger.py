"""Tests for the read-only Honda Bosch-A XCP Array B 20 Hz diagnostic logger."""
from __future__ import annotations

from types import SimpleNamespace

from opendbc.car.honda.values import HondaFlags
import openpilot.starpilot.car.honda.xcp_radar_logger as xcp_mod
from openpilot.starpilot.car.honda.xcp_radar_logger import (
  NUM_SLOTS,
  REC0,
  STRIDE,
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
  # Closer stationary in-path clutter (slot 1) vs actual distant moving lead (slot 4 at x=107.3, op dRel=100.0 per 1.08*op - 0.7)
  _write_slot(ram, 1, {"x": 8.0, "vx": -15.0, "ax": 0.0, "y": 0.0, "vy": 0.0, "flags": 0xE0C0, "sub": 0x0400, "age": 20, "qual": 20, "cls": 60})
  _write_slot(ram, 4, {"x": 107.3, "vx": -1.0, "ax": -0.5, "y": -0.3, "vy": 0.0, "flags": 0xE0C0, "sub": 0x0400, "age": 30, "qual": 25, "cls": 85})

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
  # Once both slot 1 and slot 4 have flags, lead should pick slot 4 (both via moving fallback and via op_tracks offset match)
  rows_both = [r for r in rows if {tgt["slot"] for tgt in r["targets"]} >= {1, 4}]
  assert rows_both
  assert all(r["lead"]["slot"] == 4 for r in rows_both)
  assert rows_both[-1]["lead"]["op_track_id"] == 55
  assert rows_both[-1]["lead"]["op_match_dist_m"] < 0.2
