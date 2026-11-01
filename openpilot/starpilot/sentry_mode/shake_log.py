"""Observation-only log of how hard the parked car shakes, for choosing Sentry sensitivity and warning time.

Peter raised sensitivity to 0.08 and warning time to 3 s because passing-car rumble near home raised warnings, and
then gentle get-ins recorded nothing (2026-10-31). This log keeps per-second shake statistics so the two can be told
apart from data. It never changes detection, arming, alarms, notifications, clips or timing: it only reads the sample
and decision the runtime already has, every error inside it is swallowed, and file I/O runs on its own thread so the
10 Hz motion loop only appends to a list.

One JSONL file per UTC day under starpilot_storage_root()/sentry/shake/, at most MAX_BYTES in total (oldest file
deleted first). A line is written for each 1 s window whose largest |magnitude delta| exceeds LOG_FLOOR or that had
an event, plus a heartbeat every HEARTBEAT_NS so "armed but quiet" can be told from "not running". Lines are appended
at most every FLUSH_SECONDS with no fsync.

Window line: v, wallNs (window close), monoNs (window start; same clock as event monoTimeNs), n (deltas), max, sum,
over (counts of |delta| > each of THRESHOLDS), d (signed magnitude deltas in order, 4 decimals), armed (any armed
tick), state (last decision state), trig (largest policy trigger count), sens, warnN (policy warning sample count),
events ([{kind, monoNs, queued}]; when queued, monoNs equals the stored event's monoTimeNs).
Heartbeat line: v, hb=1, wallNs, monoNs, state, armed, trig, sens, warnN, and lines/dropped since the last heartbeat.
"""

from __future__ import annotations

import json
import math
import os
import threading
import time
from pathlib import Path
from typing import Any


VERSION = 1
THRESHOLDS = (0.02, 0.03, 0.04, 0.06, 0.08, 0.10, 0.12, 0.15, 0.20, 0.30)
LOG_FLOOR = 0.02  # Parked lsm6ds3trc at the 10 Hz loop: 30 s quiet max |delta| 0.017 (2026-11-01, static read on the comma).
WINDOW_NS = 1_000_000_000
HEARTBEAT_NS = 600_000_000_000
FLUSH_SECONDS = 10.0
MAX_BYTES = 20_000_000
MAX_BUFFER_BYTES = 256_000
MAX_SAMPLE_GAP_NS = 500_000_000  # policy.MAX_SAMPLE_AGE_SECONDS; an older previous sample gives no delta.
ERROR_LOG_NS = 600_000_000_000
FILE_SUFFIX = ".jsonl"


def default_root() -> Path:
  from openpilot.starpilot.storage import starpilot_storage_root
  return starpilot_storage_root() / "sentry/shake"


def _new_window(now_ns: int, state: str) -> dict:
  return {"monoNs": now_ns, "n": 0, "max": 0.0, "sum": 0.0, "over": [0] * len(THRESHOLDS), "d": [],
          "armed": False, "state": state, "trig": 0, "events": []}


class ShakeLog:
  def __init__(self, root: Path | None = None, *, wall_clock=time.time_ns, max_bytes: int = MAX_BYTES,
               background: bool = True):
    self._root = root  # Resolved on the writer thread so construction never raises.
    self.wall_clock = wall_clock
    self.max_bytes = max_bytes
    self.background = background
    self.lock = threading.Lock()
    self.buffer: list[tuple[str, bytes]] = []  # (UTC day, line)
    self.buffer_bytes = 0
    self.stop = threading.Event()
    self.thread: threading.Thread | None = None
    self.last_heartbeat_ns: int | None = None
    self.last_error_ns: int | None = None
    self.lines = self.dropped = 0
    self.previous: tuple[int, float] | None = None  # (sample ns, magnitude)
    self.window: dict | None = None

  # Public entry points never raise.

  def observe(self, now_ns: int, sample: Any, decision: Any, policy: Any, queued: bool = False) -> None:
    try:
      self._observe(int(now_ns), sample, decision, policy, bool(queued))
      if self.background and self.thread is None:
        self.thread = threading.Thread(target=self._writer, name="sentry-shake-log", daemon=True)
        self.thread.start()
    except Exception:
      self._error()

  def flush(self) -> None:
    try:
      self._flush()
    except Exception:
      self._error()

  def close(self) -> None:
    try:
      if self.window is not None:
        self._finish_window()
      self.stop.set()
      if self.thread is not None:
        self.thread.join(timeout=2)
    except Exception:
      self._error()
    if self.thread is None or not self.thread.is_alive():
      self.flush()

  # Internals.

  def _error(self) -> None:
    try:
      now = time.monotonic_ns()
      if self.last_error_ns is None or now - self.last_error_ns >= ERROR_LOG_NS:
        self.last_error_ns = now
        from openpilot.common.swaglog import cloudlog
        cloudlog.exception("sentry shake log failed")
    except Exception:
      pass

  def _writer(self) -> None:
    while not self.stop.wait(FLUSH_SECONDS):
      self.flush()
    self.flush()

  def _observe(self, now_ns: int, sample: Any, decision: Any, policy: Any, queued: bool) -> None:
    window = self.window
    if window is not None and now_ns // WINDOW_NS != window["monoNs"] // WINDOW_NS:
      self._finish_window()
      window = None

    state = str(getattr(decision, "state", ""))
    armed = state == "armed"
    event = getattr(decision, "event", None)
    settings = getattr(policy, "settings", None)
    trig = int(getattr(policy, "trigger_count", 0) or 0)

    if sample is not None:
      stamp_ns = round(float(sample.mono_time) * 1e9)
      magnitude = math.hypot(*sample.acceleration)
      previous, self.previous = self.previous, (stamp_ns, magnitude)
      if previous is not None and 0 < stamp_ns - previous[0] <= MAX_SAMPLE_GAP_NS:
        delta = magnitude - previous[1]
        size = abs(delta)
        if window is None:
          window = self.window = _new_window(now_ns, state)
        window["n"] += 1
        window["max"] = max(window["max"], size)
        window["sum"] += size
        window["d"].append(round(delta, 4))
        over = window["over"]
        for index, threshold in enumerate(THRESHOLDS):
          if size > threshold:
            over[index] += 1
    else:
      self.previous = None

    if event is not None and window is None:
      window = self.window = _new_window(now_ns, state)
    if window is not None:
      window["armed"] = window["armed"] or armed
      window["state"] = state
      window["trig"] = max(window["trig"], trig)
      if settings is not None:
        window["sens"] = float(settings.sensitivity)
        window["warnN"] = int(settings.warning_trigger_count)
      if event is not None:
        window["events"].append({"kind": str(event), "monoNs": now_ns, "queued": queued})

    if self.last_heartbeat_ns is None or now_ns - self.last_heartbeat_ns >= HEARTBEAT_NS or now_ns < self.last_heartbeat_ns:
      self.last_heartbeat_ns = now_ns
      line = {"v": VERSION, "hb": 1, "wallNs": self.wall_clock(), "monoNs": now_ns, "state": state, "armed": armed,
              "trig": trig, "lines": self.lines, "dropped": self.dropped}
      if settings is not None:
        line["sens"] = float(settings.sensitivity)
        line["warnN"] = int(settings.warning_trigger_count)
      self.lines = self.dropped = 0
      self._append(line)

  def _finish_window(self) -> None:
    window, self.window = self.window, None
    if window["max"] > LOG_FLOOR or window["events"]:
      line = {"v": VERSION, "wallNs": self.wall_clock(), **window}
      line["max"] = round(line["max"], 5)
      line["sum"] = round(line["sum"], 5)
      self.lines += 1
      self._append(line)

  def _append(self, line: dict) -> None:
    raw = (json.dumps(line, separators=(",", ":"), allow_nan=False) + "\n").encode()
    day = time.strftime("%Y-%m-%d", time.gmtime(int(line["wallNs"]) // 1_000_000_000))
    with self.lock:
      if self.buffer_bytes + len(raw) > MAX_BUFFER_BYTES:
        self.dropped += 1  # Storage keeps failing; never grow without bound.
        return
      self.buffer.append((day, raw))
      self.buffer_bytes += len(raw)

  def _flush(self) -> None:
    with self.lock:
      pending, self.buffer, self.buffer_bytes = self.buffer, [], 0
    if not pending:
      return
    try:
      self._write(pending)
    except BaseException:
      with self.lock:  # Keep the lines for the next flush, still bounded.
        kept = pending + self.buffer
        while kept and sum(len(raw) for _, raw in kept) > MAX_BUFFER_BYTES:
          kept.pop(0)
          self.dropped += 1
        self.buffer, self.buffer_bytes = kept, sum(len(raw) for _, raw in kept)
      raise

  def _write(self, pending: list[tuple[str, bytes]]) -> None:
    from openpilot.starpilot.sentry_mode.storage import EventStore, _private
    if self._root is None:
      self._root = default_root()
    days: dict[str, list[bytes]] = {}
    for day, raw in pending:
      days.setdefault(day, []).append(raw)
    # Same private, symlink-safe traversal as event storage: 0700 directories, owner-only checks.
    with EventStore(self._root)._directory(create=True) as directory:
      sizes = {}
      with os.scandir(directory) as entries:
        for entry in entries:
          if entry.name.endswith(FILE_SUFFIX) and entry.is_file(follow_symlinks=False):
            sizes[entry.name] = entry.stat(follow_symlinks=False).st_size
      for day in sorted(days):
        name = day + FILE_SUFFIX
        body = b"".join(days[day])
        total = sum(sizes.values()) + len(body)
        for old in sorted(sizes):  # YYYY-MM-DD names sort oldest first.
          if total <= self.max_bytes:
            break
          if old != name:
            os.unlink(old, dir_fd=directory)
            total -= sizes.pop(old)
        if total > self.max_bytes:
          with self.lock:
            self.dropped += len(days[day])  # This day's file alone fills the cap.
          continue
        fd = os.open(name, os.O_WRONLY | os.O_CREAT | os.O_APPEND | os.O_NOFOLLOW | os.O_NONBLOCK, 0o600, dir_fd=directory)
        try:
          if not _private(os.fstat(fd)):
            raise OSError("shake log file is not private")
          view = memoryview(body)
          while view:
            view = view[os.write(fd, view):]
        finally:
          os.close(fd)
        sizes[name] = sizes.get(name, 0) + len(body)
