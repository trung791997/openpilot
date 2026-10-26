"""The Jetson's add-ons (V-ASM, speed-limit vision) give the GPU back to Jetlink's driving model.

The driving model comes first; the add-ons only use headroom it leaves. While the gate yields, a server
answers every request with STATUS_YIELD and sends yielding hellos, and the comma stops sending and runs its
own CPU model (spot_monitor/jetson_link.py, speed_limits/vision/jetson_net.py) until a plain hello.

While jetlink-server's per-second stats are fresh (its web page's /events stream, signed in with the password
in PASSWORD_FILE, which only the owner writes), it yields for LATENCY_HOLD_S whenever the driving model's frames
came close to their 50 ms budget: served p99 above P99_MAX_MS, or any frame Jetlink counted slow. That is the
deadline itself, so the two guesses below are then not used.

Without that stream (no password file, the server down, no stats for STATS_FRESH_S), it yields, for at least
HOLD_S, when either:
  - the GPU's busy fraction (sysfs load, 0..1000, averaged over about 1 s) is above LOAD_MAX, or
  - this add-on's own pass took more than SLOW_FACTOR times its quiet baseline (the fastest of its last
    BASELINE_N passes) and at least SLOW_MIN_US more: someone else is on the GPU.
On the road (2026-10-25) those yielded 17-56% of every minute while the driving model's served p99 stayed at
18-22 ms. CUDA stream priorities do not help here: they rank streams within one process's context, and
jetlink-server is another process.

With a log callback it reports, for tuning LOAD_MAX/HOLD_S on the road, without changing what it decides:
  - each yield as it ends: how long, what started it, and what extended it (each new trigger restarts HOLD_S),
  - every SUMMARY_S: the fraction of that time spent yielding, the GPU load's mean and peak, and the episodes.
"""
from __future__ import annotations

import collections
import http.client
import json
import os
import threading
import time

GPU_LOAD = '/sys/devices/platform/gpu.0/load'
LOAD_MAX = 0.80
SLOW_FACTOR = 2.0
SLOW_MIN_US = 2000
HOLD_S = 5.0
BASELINE_N = 200
JETLINK = ('127.0.0.1', 5600)
PASSWORD_FILE = os.path.expanduser('~/.config/jetlink-gate.pw')
P99_MAX_MS = 40.0       # of the 50 ms budget; measured 18-22 ms with both add-ons serving
LATENCY_HOLD_S = 1.0    # one stats window: the next one says whether to keep yielding
STATS_FRESH_S = 3.0     # the stats come every second; older than this and the stream is gone
RETRY_S = 5.0
SAMPLE_S = 0.05
EMA_ALPHA = SAMPLE_S / 1.0   # about a 1 s average
SUMMARY_S = 60.0


class YieldGate:
  def __init__(self, path: str = GPU_LOAD, load_max: float = LOAD_MAX, hold_s: float = HOLD_S,
               clock=time.monotonic, start: bool = True, log=None, password_file: str | None = PASSWORD_FILE):
    self.path, self.load_max, self.hold_s, self.clock, self.log = path, load_max, hold_s, clock, log
    self.stats_at = -1e9   # self.clock() of the last live stats window
    self.served_p99 = None
    self.load = 0.0
    self.until = 0.0
    self.reason = ''
    self.stats = {'load_yields': 0, 'slow_yields': 0, 'latency_yields': 0}
    self._passes: dict[int, collections.deque] = {}
    self._lock = threading.Lock()
    self._episode = None   # [start, first reason, Counter of extending triggers, start of its unsummarised part]
    self._window = self._new_window(self.clock())
    if start:
      threading.Thread(target=self._sample_loop, name='yield-gate', daemon=True).start()
      if password_file and os.path.exists(password_file):
        threading.Thread(target=self._stats_loop, args=(password_file,), name='yield-gate-stats', daemon=True).start()

  def latency_live(self) -> bool:
    return self.clock() - self.stats_at < STATS_FRESH_S

  def on_stats(self, stats: dict, now_epoch: float | None = None):
    """One of jetlink-server's per-second stats; a replayed one (opening the stream sends two minutes) is skipped."""
    now_epoch = time.time() if now_epoch is None else now_epoch
    try:
      p99, slow, t = float(stats['served_ms']['p99']), int(stats.get('slow', 0)), float(stats['t'])
    except (KeyError, TypeError, ValueError):
      return
    if now_epoch - t > STATS_FRESH_S:
      return
    with self._lock:
      self.stats_at, self.served_p99 = self.clock(), p99
      w = self._window
      w['served_p99_max'] = max(w['served_p99_max'], p99)
      w['slow_frames'] += slow
    if p99 > P99_MAX_MS or slow > 0:
      self._hold('latency', f'driving model p99 {p99:.1f} ms, {slow} slow', LATENCY_HOLD_S)

  def read_load(self) -> float | None:
    try:
      with open(self.path) as f:
        return int(f.read().strip()) / 1000.0
    except (OSError, ValueError):
      return None

  def sample(self, load: float | None):
    if load is None:
      return
    self.load += EMA_ALPHA * (load - self.load)
    if self.load > self.load_max and not self.latency_live():
      self._hold('load', f'gpu {self.load:.0%}')

  def note(self, kind: int, us: int):
    """One pass of this add-on took us microseconds."""
    d = self._passes.setdefault(kind, collections.deque(maxlen=BASELINE_N))
    base = min(d) if len(d) >= 10 else None
    d.append(us)
    if base is not None and us > SLOW_FACTOR * base and us - base >= SLOW_MIN_US and not self.latency_live():
      self._hold('slow', f'pass {us} us vs {base} us quiet')

  def yielding(self) -> bool:
    return self.clock() < self.until

  def _hold(self, why: str, reason: str, hold_s: float | None = None):
    with self._lock:
      if not self.yielding():
        self.stats[f'{why}_yields'] += 1
        self._end_episode()
        self._episode = [self.clock(), f'{why}: {reason}', collections.Counter(), self.clock()]
      elif self._episode is not None:
        self._episode[2][why] += 1
      self.reason = reason
      self.until = max(self.until, self.clock() + (self.hold_s if hold_s is None else hold_s))

  @staticmethod
  def _new_window(now: float) -> dict:
    return {'start': now, 'yield_s': 0.0, 'samples': 0, 'load_sum': 0.0, 'load_max': 0.0, 'episodes': [],
            'served_p99_max': 0.0, 'slow_frames': 0}

  def _end_episode(self):
    """The episode that has run out, if any, logged and counted toward the window (at its true end, until)."""
    if self._episode is None or self.yielding():
      return
    start, first, extended, unsummarised = self._episode
    self._episode = None
    seconds = self.until - start
    self._window['yield_s'] += self.until - unsummarised
    self._window['episodes'].append(round(seconds, 1))
    if self.log is not None:
      more = ', '.join(f'{n} {why}' for why, n in sorted(extended.items())) or 'nothing'
      self.log(f'yielded {seconds:.1f} s: started by {first}; extended by {more}')

  def tick(self, load: float | None = None):
    """Per sample: close a finished episode, account the load, and every SUMMARY_S log the window."""
    with self._lock:
      self._end_episode()
      w = self._window
      if load is not None:
        w['samples'] += 1
        w['load_sum'] += load
        w['load_max'] = max(w['load_max'], load)
      now = self.clock()
      if now - w['start'] < SUMMARY_S:
        return
      yield_s = w['yield_s']
      if self._episode is not None:   # the open episode's share of this window
        yield_s += now - self._episode[3]
        self._episode[3] = now
      if self.log is not None:
        mean = w['load_sum'] / w['samples'] if w['samples'] else float('nan')
        driving = (f"driving model p99 max {w['served_p99_max']:.1f} ms, {w['slow_frames']} slow" if self.latency_live()
                   else 'no jetlink stats: load and slow-pass gate')
        self.log(f"yielding {yield_s / (now - w['start']):.0%} of the last {now - w['start']:.0f} s; {driving}; gpu load mean "
                 f"{mean:.0%} peak {w['load_max']:.0%}; ended yields (s) {w['episodes']}; stats {self.stats}")
      self._window = self._new_window(now)

  def _stats_loop(self, password_file: str):
    """Sign in to jetlink-server's page and follow its /events stream; on any failure wait RETRY_S and again."""
    last_error = None
    while True:
      try:
        with open(password_file) as f:
          body = json.dumps({'password': f.read()})
        conn = http.client.HTTPConnection(*JETLINK, timeout=STATS_FRESH_S)
        conn.request('POST', '/api/login', body, {'Content-Type': 'application/json', 'X-Jetlink': '1'})
        resp = conn.getresponse()
        resp.read()
        del body
        if resp.status != 200:
          raise OSError(f'sign-in answered {resp.status}')
        cookie = '; '.join(v.split(';', 1)[0] for k, v in resp.getheaders() if k.lower() == 'set-cookie')
        conn = http.client.HTTPConnection(*JETLINK, timeout=STATS_FRESH_S)
        conn.request('GET', '/events', headers={'Cookie': cookie, 'Accept': 'text/event-stream'})
        resp = conn.getresponse()
        if resp.status != 200:
          raise OSError(f'/events answered {resp.status}')
        last_error = None
        if self.log is not None:
          self.log('following jetlink-server stats: yielding on the driving model\'s frame times')
        while True:
          line = resp.fp.readline()
          if not line:
            raise OSError('/events ended')
          if line.startswith(b'data:') and b'"stats"' in line:
            event = json.loads(line[5:])
            if event.get('event') == 'stats':
              self.on_stats(event)
      except (OSError, ValueError, http.client.HTTPException) as e:
        if self.log is not None and str(e) != last_error:   # once per kind of failure, not every RETRY_S
          self.log(f'jetlink stats unavailable ({e}); load and slow-pass gate')
        last_error = str(e)
        time.sleep(RETRY_S)

  def _sample_loop(self):
    while True:
      load = self.read_load()
      self.sample(load)
      self.tick(load)
      time.sleep(SAMPLE_S)
