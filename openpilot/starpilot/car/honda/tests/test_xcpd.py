"""Unit and integration tests for Option A standalone XCP daemon (xcpd) and single-sender guards."""
from __future__ import annotations

import os
import subprocess
import sys
import threading
from pathlib import Path
from types import SimpleNamespace

from opendbc.car.honda.values import HondaFlags
from opendbc.car.structs import car
from openpilot.cereal import log
from openpilot.cereal.services import QueueSize, SERVICE_LIST
from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.selfdrive.pandad import can_capnp_to_list, can_list_to_can_capnp
import openpilot.starpilot.car.honda.xcp_radar_logger as xcp_mod
from openpilot.starpilot.car.honda.xcp_radar_logger import NUM_SLOTS, REC0, STRIDE, XCP_BUS, XCP_CTO, XCP_DTO, XcpRadarLogger
from openpilot.starpilot.car.honda.tests.test_xcp_radar_logger import _write_slot
from openpilot.starpilot.car.honda.xcpd import run_xcpd
from openpilot.system.manager.process_config import managed_processes, xcpd_enabled


def _bosch_a_cp_bytes(*, flags: int = int(HondaFlags.BOSCH), passive: bool = False, brand: str = "honda") -> bytes:
  msg = car.CarParams.new_message()
  msg.brand = brand
  msg.flags = flags
  msg.passive = passive
  return msg.to_bytes()


class _FakeParams:
  def __init__(self, initial: dict | None = None) -> None:
    self._store = dict(initial or {})

  def get_bool(self, key: str) -> bool:
    val = self._store.get(key)
    if isinstance(val, bool):
      return val
    return val in ("1", b"1", 1, "true", "True")

  def get(self, key: str, *_args, **_kwargs):
    return self._store.get(key)


class _FakeSock:
  def __init__(self, topic: str) -> None:
    self.topic = topic


class _FakePM:
  def __init__(self, topics: list[str], on_send=None) -> None:
    self.topics = list(topics)
    self.sent: list[tuple[str, bytes]] = []
    self._on_send = on_send

  def send(self, topic: str, dat: bytes) -> None:
    self.sent.append((topic, dat))
    if self._on_send is not None:
      self._on_send(topic, dat)


class _ScriptedMessaging:
  """Minimal deterministic messaging shim for exercising run_xcpd without msgq."""

  def __init__(self, ram: bytearray, stop_event: threading.Event, clock: list[int], max_cycles: int = 80) -> None:
    self.ram = ram
    self.stop_event = stop_event
    self.clock = clock
    self.max_cycles = max_cycles
    self.pm: _FakePM | None = None
    self.pm_creations: list[list[str]] = []
    self.car_state_queue: list[object] = []
    self.radar_tracks_queue: list[object] = []
    self.can_queue: list[bytes] = []
    self.drain_calls = 0
    self._unlocked = False

  def sub_sock(self, topic: str, conflate: bool = False, timeout: int | None = None) -> _FakeSock:
    return _FakeSock(topic)

  def recv_one_or_none(self, sock: _FakeSock):
    if sock.topic == "carState" and self.car_state_queue:
      return self.car_state_queue.pop(0)
    if sock.topic == "radarTracks" and self.radar_tracks_queue:
      return self.radar_tracks_queue.pop(0)
    return None

  def PubMaster(self, topics: list[str]) -> _FakePM:
    self.pm_creations.append(list(topics))
    self.pm = _FakePM(topics, on_send=self._handle_tx)
    return self.pm

  def _handle_tx(self, topic: str, raw: bytes) -> None:
    assert topic == "xcpSendcan"
    decoded = can_capnp_to_list([raw], msgtype="xcpSendcan")
    _, frames = decoded[0]
    for addr, dat, bus in frames:
      assert bus == XCP_BUS and addr == XCP_CTO
      pkt = bytes(dat)
      cmd = pkt[0]
      if cmd == 0xFF:
        reply = b"\xFF\x00" + bytes(6)
      elif cmd == 0xFD:
        reply = b"\xFF\x00\x04" + bytes(5)
      elif cmd == 0xF8:
        reply = b"\xFF\x04\x12\x34\x56\x78" + bytes(2)
      elif cmd == 0xF7:
        self._unlocked = True
        reply = b"\xFF" + bytes(7)
      elif cmd == 0xF4:
        sz = pkt[1]
        mem_addr = int.from_bytes(pkt[4:8], "big")
        if REC0 <= mem_addr < REC0 + NUM_SLOTS * STRIDE:
          off = mem_addr - REC0
          reply = b"\xFF" + bytes(self.ram[off : off + sz])
        else:
          reply = b"\xFF" + bytes(sz)
      else:
        raise AssertionError(f"Unexpected CTO cmd {cmd:#x}")
      can_raw = can_list_to_can_capnp([(XCP_DTO, reply, XCP_BUS)], msgtype="can", valid=True)
      self.can_queue.append(can_raw)

  def drain_sock_raw(self, sock: _FakeSock, wait_for_one: bool = False) -> list[bytes]:
    assert sock.topic == "can"
    self.drain_calls += 1
    if self.drain_calls == 1:
      # Jump past STARTUP_DELAY_NS after arming so handshake begins immediately
      self.clock[0] += xcp_mod.STARTUP_DELAY_NS + 10_000_000
    else:
      self.clock[0] += 14_000_000  # ~14 ms Option A cadence
    out = list(self.can_queue)
    self.can_queue.clear()
    if self.drain_calls >= self.max_cycles:
      self.stop_event.set()
    return out


def test_xcp_sendcan_schema_and_service_list():
  assert "xcpSendcan" in log.Event.schema.union_fields
  svc = SERVICE_LIST["xcpSendcan"]
  assert svc.should_log is True
  assert svc.frequency == 100.0
  assert svc.decimation is None
  assert svc.queue_size == QueueSize.SMALL

  pkt = b"\xFF\x00" + bytes(6)
  raw = can_list_to_can_capnp([(XCP_CTO, pkt, XCP_BUS)], msgtype="xcpSendcan", valid=True)
  decoded = can_capnp_to_list([raw], msgtype="xcpSendcan")
  assert len(decoded) == 1
  _, frames = decoded[0]
  assert frames == [(XCP_CTO, pkt, XCP_BUS)]


def test_xcpd_disabled_by_param_exits_without_pubmaster():
  stop = threading.Event()
  msg_mod = _ScriptedMessaging(bytearray(NUM_SLOTS * STRIDE), stop, [1_000_000_000])
  params = _FakeParams({"AlphaHondaXcpDaemon": False, "CarParams": _bosch_a_cp_bytes()})
  run_xcpd(params=params, messaging_mod=msg_mod, stop_event=stop, sender_lock_path=None, shm_path=None, assist_shm_path=None)
  assert msg_mod.pm_creations == []
  assert msg_mod.drain_calls == 0


def test_xcpd_non_bosch_a_sends_zero_frames():
  for kwargs in (
    {"brand": "toyota", "flags": 0},
    {"flags": int(HondaFlags.BOSCH | HondaFlags.BOSCH_RADARLESS)},
    {"flags": int(HondaFlags.BOSCH | HondaFlags.BOSCH_CANFD)},
    {"flags": int(HondaFlags.BOSCH), "passive": True},
  ):
    stop = threading.Event()
    msg_mod = _ScriptedMessaging(bytearray(NUM_SLOTS * STRIDE), stop, [1_000_000_000])
    msg_mod.car_state_queue.append(SimpleNamespace(carState=SimpleNamespace(vEgo=0.0)))
    params = _FakeParams({"AlphaHondaXcpDaemon": True, "CarParams": _bosch_a_cp_bytes(**kwargs)})
    run_xcpd(
      params=params,
      messaging_mod=msg_mod,
      stop_event=stop,
      sender_lock_path=None,
      shm_path=None,
      assist_shm_path=None,
      exit_if_disabled=True,
    )
    assert msg_mod.pm_creations == [["xcpSendcan"]]
    assert msg_mod.pm is not None and msg_mod.pm.sent == []
    assert msg_mod.drain_calls == 0


def test_xcpd_startup_interlock_waits_for_carparams_and_carstate():
  # Scenario A: CarParams already written, carState absent -> must wait and send 0 CTOs
  stop = threading.Event()
  msg_mod = _ScriptedMessaging(bytearray(NUM_SLOTS * STRIDE), stop, [1_000_000_000])
  params = _FakeParams({"AlphaHondaXcpDaemon": True, "CarParams": _bosch_a_cp_bytes()})
  threading.Timer(0.08, stop.set).start()
  run_xcpd(
    params=params,
    messaging_mod=msg_mod,
    stop_event=stop,
    sender_lock_path=None,
    shm_path=None,
    assist_shm_path=None,
    poll_sleep_s=0.01,
  )
  assert msg_mod.pm_creations == []

  # Scenario B: carState arrives first, CarParams appears only on 4th poll -> no PubMaster before CarParams
  stop2 = threading.Event()
  msg_mod2 = _ScriptedMessaging(bytearray(NUM_SLOTS * STRIDE), stop2, [1_000_000_000], max_cycles=12)
  msg_mod2.car_state_queue.append(SimpleNamespace(carState=SimpleNamespace(vEgo=12.5)))

  class _DelayedParams(_FakeParams):
    def __init__(self) -> None:
      super().__init__({"AlphaHondaXcpDaemon": True})
      self.calls = 0

    def get(self, key: str, *_args, **_kwargs):
      if key == "CarParams":
        self.calls += 1
        if self.calls >= 4:
          return _bosch_a_cp_bytes()
      return None

  d_params = _DelayedParams()
  run_xcpd(
    params=d_params,
    messaging_mod=msg_mod2,
    stop_event=stop2,
    clock_ns=lambda: msg_mod2.clock[0],
    sender_lock_path=None,
    shm_path=None,
    assist_shm_path=None,
    poll_sleep_s=0.002,
  )
  assert d_params.calls >= 4
  assert msg_mod2.pm_creations == [["xcpSendcan"]]
  assert len(msg_mod2.pm.sent) >= 1
  # First frame must be CONNECT on xcpSendcan
  _, frames0 = can_capnp_to_list([msg_mod2.pm.sent[0][1]], msgtype="xcpSendcan")[0]
  assert frames0[0][0] == XCP_CTO and bytes(frames0[0][1])[:2] == b"\xFF\x00"


def test_xcpd_emits_only_xcp_sendcan_and_logs_reads(monkeypatch, tmp_path):
  events: list[tuple[str, dict]] = []
  monkeypatch.setattr(xcp_mod.cloudlog, "event", lambda name, **kw: events.append((name, kw)))

  ram = bytearray(NUM_SLOTS * STRIDE)
  _write_slot(ram, 2, {"x": 30.0, "vx": -1.5, "ax": -0.3, "y": 0.2, "vy": 0.0, "flags": 0xE0C0, "sub": 0x0400, "age": 20, "qual": 25, "cls": 80})

  clock = [1_000_000_000]
  stop = threading.Event()
  msg_mod = _ScriptedMessaging(ram, stop, clock, max_cycles=120)
  msg_mod.car_state_queue.append(SimpleNamespace(carState=SimpleNamespace(vEgo=20.0)))
  msg_mod.radar_tracks_queue.append(SimpleNamespace(radarTracks=SimpleNamespace(points=[
    SimpleNamespace(trackId=77, dRel=26.6, yRel=0.2, vRel=-1.5),
  ])))
  params = _FakeParams({"AlphaHondaXcpDaemon": True, "CarParams": _bosch_a_cp_bytes()})
  lock_path = tmp_path / "xcp.lock"

  run_xcpd(
    params=params,
    messaging_mod=msg_mod,
    stop_event=stop,
    clock_ns=lambda: clock[0],
    sender_lock_path=lock_path,
    shm_path=None,
    assist_shm_path=None,
  )

  assert msg_mod._unlocked is True
  assert msg_mod.pm_creations == [["xcpSendcan"]]
  assert all(topic == "xcpSendcan" for topic, _ in msg_mod.pm.sent)
  assert len(msg_mod.pm.sent) >= 50
  ev_20hz = [kw for name, kw in events if name == "xcp_array_b_20hz"]
  ev_reads = [kw for name, kw in events if name == "xcp_reads"]
  assert ev_20hz and ev_20hz[-1]["lead"] is not None and ev_20hz[-1]["lead"]["slot"] == 2
  assert ev_reads and any(rec[0] == 0 for ev in ev_reads for rec in ev["records"])


def test_single_sender_flock_exclusion_in_process_and_cross_process(tmp_path):
  lock_path = tmp_path / "sender.lock"
  shm_path = tmp_path / "ui.json"
  shm_path.write_text('{"mono_ns": 1}')
  cp = SimpleNamespace(brand="honda", flags=int(HondaFlags.BOSCH), passive=False)

  # Subprocess holds flock -> new XcpRadarLogger must refuse TX and never touch shm_path
  child_cmd = (
    f"import fcntl, os, sys, time; fd = os.open({str(lock_path)!r}, os.O_CREAT | os.O_RDWR, 0o644); "
    + "fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB); sys.stdout.write('LOCKED\\n'); sys.stdout.flush(); time.sleep(30)"
  )
  proc = subprocess.Popen(
    [sys.executable, "-c", child_cmd],
    stdout=subprocess.PIPE,
    text=True,
  )
  try:
    assert proc.stdout is not None and proc.stdout.readline().strip() == "LOCKED"
    pm_blocked = _FakePM(["xcpSendcan"])
    logger_blocked = XcpRadarLogger(
      cp,
      pm=pm_blocked,
      send_topic="xcpSendcan",
      use_io_thread=False,
      sender_lock_path=lock_path,
      shm_path=shm_path,
      assist_shm_path=None,
    )
    assert logger_blocked.enabled is False
    assert logger_blocked.state == "DISABLED"
    assert logger_blocked._io_thread is None
    assert logger_blocked.next_can_sends(1_000_000_000) == []
    assert pm_blocked.sent == []
    logger_blocked.close()
    assert shm_path.exists()  # blocked process must not unlink active sender's SHM
  finally:
    proc.terminate()
    proc.wait(timeout=5)

  # After holder exits, lock can be acquired cleanly and blocks a second logger until close()
  pm1 = _FakePM(["sendcan"])
  l1 = XcpRadarLogger(cp, pm=pm1, use_io_thread=False, sender_lock_path=lock_path, shm_path=None, assist_shm_path=None)
  assert l1.enabled is True
  pm2 = _FakePM(["xcpSendcan"])
  l2 = XcpRadarLogger(cp, pm=pm2, send_topic="xcpSendcan", use_io_thread=False, sender_lock_path=lock_path, shm_path=None, assist_shm_path=None)
  assert l2.enabled is False
  l1.close()
  l2.close()

  pm3 = _FakePM(["xcpSendcan"])
  l3 = XcpRadarLogger(cp, pm=pm3, send_topic="xcpSendcan", use_io_thread=False, sender_lock_path=lock_path, shm_path=None, assist_shm_path=None)
  assert l3.enabled is True
  l3.close()


def test_card_gating_and_params_key_and_process_config(monkeypatch):
  from unittest.mock import patch
  from opendbc.car.honda.interface import CarInterface
  from opendbc.car.honda.radar_interface import RadarInterface
  from opendbc.car.honda.values import CAR
  from openpilot.selfdrive.car.card import Car

  with OpenpilotPrefix(), patch.dict(os.environ, {"SIMULATION": "1"}):
    params = Params()
    assert params.get_bool("AlphaHondaXcpDaemon") is False

    # Construct real Honda Bosch-A CarParams & CarInterface without Panda hardware
    params.put_bool("OpenpilotEnabledToggle", True, block=True)
    cp = CarInterface.get_non_essential_params(CAR.HONDA_CIVIC_BOSCH)
    params.put("CarParams", cp.to_bytes(), block=True)

    # Manager gating checks
    assert xcpd_enabled(False, params, cp) is False
    assert xcpd_enabled(True, params, cp) is False
    params.put_bool("AlphaHondaXcpDaemon", True, block=True)
    assert params.get_bool("AlphaHondaXcpDaemon") is True
    assert xcpd_enabled(False, params, cp) is False
    assert xcpd_enabled(True, params, cp) is True

    other_cp = car.CarParams.new_message()
    other_cp.brand = "honda"
    other_cp.flags = int(HondaFlags.BOSCH | HondaFlags.BOSCH_RADARLESS)
    other_cp.passive = False
    assert xcpd_enabled(True, params, other_cp) is False
    assert "xcpd" in managed_processes and managed_processes["xcpd"].enabled is True

    # Real Card constructor skips XcpRadarLogger when AlphaHondaXcpDaemon=1
    card_daemon_on = Car(CI=CarInterface(cp), RI=RadarInterface(cp))
    assert card_daemon_on.xcp_radar_logger is None

    # Default ("0") instantiates in-card XcpRadarLogger as before (use fresh CI so passive is unmutated)
    cp_off = CarInterface.get_non_essential_params(CAR.HONDA_CIVIC_BOSCH)
    params.put_bool("AlphaHondaXcpDaemon", False, block=True)
    monkeypatch.setattr(xcp_mod, "_default_sender_lock_path", lambda: Path(f"/tmp/test_card_xcp_{os.getpid()}.lock"))
    card_daemon_off = Car(CI=CarInterface(cp_off), RI=RadarInterface(cp_off))
    try:
      assert card_daemon_off.xcp_radar_logger is not None
      assert card_daemon_off.xcp_radar_logger.enabled is True
      assert card_daemon_off.xcp_radar_logger._send_topic == "sendcan"
    finally:
      card_daemon_off.xcp_radar_logger.close()
