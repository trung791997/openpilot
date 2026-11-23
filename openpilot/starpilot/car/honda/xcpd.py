"""Dedicated low-latency XCP Array B reader daemon for Honda Bosch-A."""
from __future__ import annotations

from collections.abc import Callable
from pathlib import Path
import signal
import threading
import time

from opendbc.car.structs import car
import openpilot.cereal.messaging as default_messaging
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.starpilot.car.honda.xcp_radar_logger import (
  _DEFAULT_ASSIST_SHM_PATH,
  _DEFAULT_SENDER_LOCK_PATH,
  XCP_UI_SHM_PATH,
  XcpRadarLogger,
)

PARAM_KEY = "AlphaHondaXcpDaemon"


def _drain_latest(sock, recv_fn):
  latest = None
  while True:
    msg = recv_fn(sock)
    if msg is None:
      break
    latest = msg
  return latest


def run_xcpd(
  *,
  params=None,
  messaging_mod=None,
  stop_event: threading.Event | None = None,
  clock_ns: Callable[[], int] = time.monotonic_ns,
  sender_lock_path: Path | None | object = _DEFAULT_SENDER_LOCK_PATH,
  shm_path: Path | None = XCP_UI_SHM_PATH,
  assist_shm_path: Path | None | object = _DEFAULT_ASSIST_SHM_PATH,
  exit_if_disabled: bool = False,
  poll_sleep_s: float = 0.05,
) -> None:
  params = params if params is not None else Params()
  messaging_mod = messaging_mod if messaging_mod is not None else default_messaging
  stop_event = stop_event if stop_event is not None else threading.Event()

  if not params.get_bool(PARAM_KEY):
    return

  car_state_sock = messaging_mod.sub_sock("carState", conflate=True)
  radar_tracks_sock = messaging_mod.sub_sock("radarTracks", conflate=True)

  cp_bytes: bytes | None = None
  car_state_seen = False
  latest_v_ego = 0.0

  while not stop_event.is_set():
    cs_msg = _drain_latest(car_state_sock, messaging_mod.recv_one_or_none)
    if cs_msg is not None:
      car_state_seen = True
      latest_v_ego = float(cs_msg.carState.vEgo)
    if cp_bytes is None:
      raw = params.get("CarParams")
      if raw:
        cp_bytes = raw
    if car_state_seen and cp_bytes:
      break
    stop_event.wait(poll_sleep_s)

  if stop_event.is_set() or not cp_bytes or not car_state_seen:
    return

  with car.CarParams.from_bytes(cp_bytes) as CP_reader:
    CP = CP_reader.as_builder()

  pm = messaging_mod.PubMaster(["xcpSendcan"])
  logger = XcpRadarLogger(
    CP,
    pm=pm,
    params=params,
    send_topic="xcpSendcan",
    use_io_thread=False,
    sender_lock_path=sender_lock_path,
    shm_path=shm_path,
    assist_shm_path=assist_shm_path,
  )
  try:
    if not logger.enabled:
      cloudlog.info("xcpd: logger disabled for current vehicle or lock held; idling")
      if not exit_if_disabled:
        stop_event.wait()
      return

    with logger._lock:
      logger.last_v_ego = latest_v_ego

    can_sock = messaging_mod.sub_sock("can", timeout=5)
    logger.arm_tx(clock_ns())
    try:
      while not stop_event.is_set():
        can_strs = messaging_mod.drain_sock_raw(can_sock, wait_for_one=True)
        now_ns = clock_ns()
        logger.step_io(can_strs, now_ns)

        cs_msg = _drain_latest(car_state_sock, messaging_mod.recv_one_or_none)
        if cs_msg is not None:
          with logger._lock:
            logger.last_v_ego = float(cs_msg.carState.vEgo)

        rd_msg = _drain_latest(radar_tracks_sock, messaging_mod.recv_one_or_none)
        if rd_msg is not None:
          logger.observe_radar_tracks(rd_msg.radarTracks, now_ns)
    finally:
      del can_sock
  finally:
    logger.close()


def main() -> None:
  stop_event = threading.Event()

  def _handle_sig(_signum, _frame) -> None:
    stop_event.set()

  signal.signal(signal.SIGINT, _handle_sig)
  signal.signal(signal.SIGTERM, _handle_sig)
  run_xcpd(stop_event=stop_event)


if __name__ == "__main__":
  main()
