"""Read-only Jetlink status for the home icon and the Android Auto card, polled off the drawing thread."""
from __future__ import annotations

from dataclasses import dataclass
import json
import math
import os
import threading
import time

STALE_NS = 2_500_000_000
# The host's processing time for the last big-model frame (excludes USB); jetlink's runtime record does not carry it
LATENCY_RECORD = "/dev/shm/jetlink/starpilot_latency.json"
LATENCY_FRESH_S = 3.
RUNNING = {'ready': 'ready', 'joining': 'connecting', 'retrying': 'connecting'}


@dataclass(frozen=True)
class JetlinkStatus:
  device: str = ''
  state: str = 'off'  # off, disconnected, connected, connecting, ready, active, unavailable
  latency_ms: float | None = None
  port: str | None = None  # jetlink's port_state: 'connected' (host on the link), 'device' (plugged in), else None
  # the onroad GPU icon, read through hud_icon(): off (the Chestnut path draws), active (green: the big model drives), linked (orange: a
  # host is on the link and the comma model drives), loading (pulsing), crossed (an error or no host)
  hud: str = 'off'
  model: str | None = None  # Status.active_model (the pick once built, else the stand-in), else Status.model
  progress: float | None = None  # Status.progress['frac'] while a stage runs, strictly between 0 and 1


def hud_state(status) -> str:
  """The GPU icon for a jetlink Status with the link enabled, the onroad half of Status.icon() decided from the
  records alone (jetlink/openpilot/status.py Status.icon/phase): no host, or a reason, is crossed; the runtime
  record's active is green; a failed stage is crossed; a join (running joining/retrying) or a stage still running
  is loading; anything else is a host on the link while the comma model drives."""
  if not status.present or status.reason:
    return 'crossed'
  if status.active:
    return 'active'
  stage = str((status.progress or {}).get('stage') or '')
  if stage == 'failed':
    return 'crossed'
  if status.running in ('joining', 'retrying') or stage not in ('', 'ready'):
    return 'loading'
  return 'linked'


def hud_icon(status: JetlinkStatus) -> str | None:
  """What the onroad GPU icon draws for a polled status: None with the link off (the Chestnut path draws), else
  active, linked, loading or crossed. A status that could not be read, or went stale, is crossed."""
  if status.state == 'off':
    return None
  if status.state == 'unavailable':
    return 'crossed'
  return status.hud if status.hud in ('active', 'linked', 'loading', 'crossed') else 'crossed'


def _progress(status) -> float | None:
  progress = status.progress if isinstance(status.progress, dict) else {}
  frac = progress.get('frac')
  if str(progress.get('stage') or '') in ('', 'ready', 'failed') or type(frac) not in (int, float):
    return None
  return float(frac) if 0. < frac < 1. else None


def home_label(status: JetlinkStatus) -> str:
  """The home screen's Jetlink line, from the polled status: '' with the link off."""
  if status.state == 'off':
    return ''
  if status.state == 'unavailable':
    return 'Jetlink error'
  if status.hud == 'crossed':
    return 'Jetlink: no host' if status.state == 'disconnected' else 'Jetlink error'
  if status.progress is not None:
    return f'Jetlink {int(status.progress * 100)}%'
  if status.hud == 'loading':
    return 'Jetlink loading'
  return f'Jetlink: {status.model}' if status.model else 'Jetlink ready'


def _write_latency(model, path) -> None:
  timings = getattr(getattr(model, "client", None), "last_timings", None)  # the big model's client while it drives
  total_us = timings[2] if isinstance(timings, (tuple, list)) and len(timings) == 3 else 0
  record = {"at": time.monotonic(), "server_ms": total_us / 1000 if type(total_us) in (int, float) and total_us > 0 else None}
  os.makedirs(os.path.dirname(path), exist_ok=True)
  tmp = f"{path}.tmp"
  with open(tmp, "w") as f:
    json.dump(record, f)
  os.replace(tmp, path)


def start_latency_writer(model, path=LATENCY_RECORD, period=1.):
  """modeld, before it goes realtime, so the thread keeps normal priority and the frame loop does no file IO."""
  def run():
    while True:
      try:
        _write_latency(model, path)
      except Exception:
        pass
      time.sleep(period)
  thread = threading.Thread(target=run, name="jetlink-latency", daemon=True)
  thread.start()
  return thread


def read_latency(path=LATENCY_RECORD) -> float | None:
  try:
    with open(path) as f:
      record = json.load(f)
    ms, at = record.get("server_ms"), record.get("at")
    fresh = type(at) in (int, float) and 0 <= time.monotonic() - at < LATENCY_FRESH_S
    return float(ms) if fresh and type(ms) in (int, float) and math.isfinite(ms) and ms > 0 else None
  except (OSError, ValueError, AttributeError):
    return None


def read_status() -> JetlinkStatus:
  from openpilot.starpilot.models.jetlink_adapter import hooks, link_on
  if not link_on():
    return JetlinkStatus()  # one file read: nothing of jetlink's loads
  status = hooks().status()
  if status is None:
    return JetlinkStatus(state='unavailable')
  if not status.enabled:
    return JetlinkStatus()  # a Chestnut is fitted
  device = 'iPhone' if status.mode == 'ios' else 'Computer'
  port = status.port_state if status.port_state in ('connected', 'device') else None
  extra = {'hud': hud_state(status), 'model': status.active_model or status.model, 'progress': _progress(status)}
  if not status.present:
    return JetlinkStatus(device, 'disconnected', port=port, **extra)
  if status.active:
    return JetlinkStatus(device, 'active', read_latency(), port, **extra)
  if status.reason:
    return JetlinkStatus(device, 'unavailable', port=port, **extra)
  return JetlinkStatus(device, RUNNING.get(status.running, 'connected'), port=port, **extra)


class JetlinkStatusFeed:
  """Polls once a second only while enabled; snapshot() never does file IO."""
  def __init__(self, read=read_status, period=1.0):
    self.read, self.period = read, period
    self._lock = threading.Lock()
    self._stop = threading.Event()
    self._thread = None
    self._enabled = False
    self._status = JetlinkStatus()
    self._read_at = 0

  def set_enabled(self, enabled):
    with self._lock:
      self._enabled = enabled
      if not enabled:
        self._status = JetlinkStatus()
    if enabled and self._thread is None:
      self._thread = threading.Thread(target=self._run, name='jetlink-status', daemon=True)
      self._thread.start()

  def poll(self):
    with self._lock:
      if not self._enabled:
        return
    try:
      status = self.read()
    except Exception:
      status = JetlinkStatus(state='unavailable')
    with self._lock:
      if self._enabled:
        self._status = status
        self._read_at = time.monotonic_ns()

  def snapshot(self):
    with self._lock:
      status, read_at = self._status, self._read_at
    if status.state != 'off' and time.monotonic_ns() - read_at > STALE_NS:
      return JetlinkStatus(status.device, 'unavailable')
    return status

  def _run(self):
    while not self._stop.is_set():
      self.poll()
      self._stop.wait(self.period)

  def close(self):
    self._stop.set()
    if self._thread is not None:
      self._thread.join(timeout=2)
