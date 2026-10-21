"""Diagnostic: dump controlsd's Python stack when one loop iteration stalls.

Why: 15 engaged soft-disables in 10 days of drives (commIssue -> selfdrivedLagging), at most
one per drive, 13 of them within ~1 s of the car reaching an engaged stop. rlogs of 000003aa and
000003a5 show controlsd silent for 430-485 ms while card keeps its 10 ms cadence, and controlsd
RSS jumping +23-25 MB at that instant. Static reading found no code path that explains it, so
this records where the main thread actually is during the stall.

How: faulthandler's C-level SIGALRM handler dumps every thread's traceback without the GIL,
and an ITIMER_REAL one-shot is re-armed every tick. A normal 10 ms tick re-arms long before it
expires, so the signal only fires when one tick runs past STALL_THRESHOLD_S. Per tick this
costs one setitimer() and one /proc read. The handler stays registered for the life of the
process, so a stray SIGALRM can never take SIGALRM's default action (terminate).
"""
import faulthandler
import gc
import os
import signal
import time

from openpilot.common.swaglog import cloudlog

STALL_DUMP_PATH = "/data/stall_dumps/controlsd.txt"
STALL_THRESHOLD_S = 0.15
MAX_STALL_DUMPS = 5


class StallWatchdog:
  def __init__(self, path: str = STALL_DUMP_PATH, threshold: float = STALL_THRESHOLD_S,
               max_dumps: int = MAX_STALL_DUMPS):
    self.threshold = threshold
    self.max_dumps = max_dumps
    self.stalls = 0
    self.enabled = False
    try:
      os.makedirs(os.path.dirname(path), exist_ok=True)
      self._f = open(path, "a", buffering=1)
      self._statm = os.open("/proc/self/statm", os.O_RDONLY)
    except OSError as e:
      cloudlog.warning(f"stall_watchdog disabled: {e}")
      return
    self._page = os.sysconf("SC_PAGE_SIZE")
    self._f.write(f"\n=== controlsd pid {os.getpid()} start {time.strftime('%Y-%m-%d %H:%M:%S')} "
                  f"threshold {threshold * 1000:.0f} ms ===\n")
    faulthandler.register(signal.SIGALRM, file=self._f, all_threads=True, chain=False)
    self._last = time.monotonic()
    self._rss = self._read_rss()
    self.enabled = True
    self._arm()

  def _read_rss(self) -> int:
    os.lseek(self._statm, 0, os.SEEK_SET)
    return int(os.read(self._statm, 128).split()[1]) * self._page

  def _arm(self):
    signal.setitimer(signal.ITIMER_REAL, self.threshold)

  def tick(self):
    if not self.enabled:
      return
    now = time.monotonic()
    dt = now - self._last
    self._last = now
    rss = self._read_rss()
    if dt > self.threshold:
      self.stalls += 1
      msg = (f"stall #{self.stalls}: tick {dt * 1000:.0f} ms, rss {self._rss / 1e6:.1f} -> {rss / 1e6:.1f} MB, "
             f"gc counts {gc.get_count()} enabled {gc.isenabled()}")
      self._f.write(f"{time.strftime('%H:%M:%S')} {msg}\n")
      cloudlog.warning(f"controlsd {msg}")
    self._rss = rss
    if self.stalls >= self.max_dumps:
      signal.setitimer(signal.ITIMER_REAL, 0)
      self.enabled = False
      return
    self._arm()
