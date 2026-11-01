"""The large model drives on through one short stall of the comma's own, and hands back for the host's lag, a stall
that repeats or runs long, and a lost link (jetlink joining.STALL_FORGIVEN).

Replayed on a simulated clock through the real JoiningModelState and JetlinkModelState, with modeld's frame loop
around them: camera frames at 20 Hz, the dropped-frame filter, its reset on a handover and the rule that the frame
after a gap is not published. The stall timings are the logged ones (6f at 9:37, 75 at 11:09)."""
from contextlib import contextmanager
from types import SimpleNamespace as NS
from unittest.mock import patch

import numpy as np
import pytest

from jetlink.openpilot import joining, model_state
from jetlink.openpilot.joining import BEHIND, LOST, JoiningModelState
from jetlink.openpilot.model_state import JetlinkModelState
from jetlink.openpilot.testing import RecordingLog

DT = 0.05
# modeld: FirstOrderFilter(0., 10., 1 / MODEL_RUN_FREQ)
ALPHA = DT / (10. + DT)
MODELD_OVERHEAD = 0.004


class Clock:
  def __init__(self):
    self.t = 1000.0

  def __call__(self):
    return self.t

  def advance(self, s):
    self.t += s


class Timing(NS):
  """One frame as the log line says it, in ms: warp, data, send, wait, held, server total, link lost."""

  def __init__(self, warp=1.4, data=2.0, send=1.3, wait=18.0, held=False, server=19.6, lost=False, not_sent=False):
    super().__init__(warp=warp, data=data, send=send, wait=wait, held=held, server=server, lost=lost, not_sent=not_sent)


NORMAL = Timing()
CAMERA = NS(data=bytearray(16))


class Client:
  def __init__(self, clock):
    self.clock = clock
    self.script = iter(())
    self.now = NORMAL
    self.last_state = None
    self.last_output = None
    self.last_timings = (19400, 200, 19600)
    self.t = NS(last_send={}, last_receive={}, datagrams=True, net_drops=lambda: None)
    self.frames_lost = 0
    self.dead = False
    self.nonce = 1

  def drain(self):
    self.clock.advance(self.now.data / 1e3)

  def infer_begin(self, *args, skip_if_busy=False, **kw):
    self.clock.advance(self.now.send / 1e3)
    return None if self.now.not_sent else 1

  def infer_end(self, seq, hold=None):
    if self.now.lost:
      self.dead = True
      raise ConnectionError('link lost')
    self.clock.advance(self.now.wait / 1e3)
    self.last_timings = (19400, 200, int(self.now.server * 1e3))
    if self.now.held and hold is not None:
      return None
    self.last_output = np.ones(4, dtype=np.float32)
    return self.last_output


class Warp:
  def __init__(self, client):
    self.client = client
    self.output = np.zeros(4, dtype=np.float32)

  def start(self, *args):
    self.client.clock.advance(self.client.now.warp / 1e3)

  def wait(self):
    pass


def big_model(clock, log):
  client = Client(clock)
  spec = NS(input_shapes={}, output_slices={'out': slice(0, 4)}, frame_skip=1, packed_nelem=6,
            packed_layout={'desire': (slice(0, 2), (2,)), 'traffic_convention': (slice(2, 4), (2,)),
                           'action_t': (slice(4, 6), (2,))})
  face = NS(constants=None, lat_smooth_seconds=0.1, long_smooth_seconds=0.3, get_action_from_model=None, desire_len=2,
            parser=lambda: NS(parse_outputs=lambda outputs: {'big': outputs['out']}))
  big = JetlinkModelState(client, spec, Warp(client), face=face, log=log, event=lambda *a, **kw: None)
  return big, client


class Modeld:
  """modeld's loop around the joining model, on the simulated clock."""

  def __init__(self, clock):
    self.clock = clock
    self.log = RecordingLog()
    self.small_runs = 0
    self.big, self.client = big_model(clock, self.log)

    def small_run(*args):
      self.small_runs += 1
      clock.advance(0.020)
      return {'small': True}

    small = NS(run=small_run, lat_delay=0.0)
    with patch.object(joining.threading, 'Thread'):
      self.model = JoiningModelState(small, connect=None, build=lambda client, spec: self.big,
                                     progress=NS(report=lambda *a, **kw: None, clear=lambda: None), log=self.log,
                                     in_control=lambda: self.in_control)
    self.in_control = False
    self.model._joined = (self.client, None)
    self.x = 0.0
    self.handovers = 0
    self.frame = -1
    self.published_drop_perc = []
    self.demotes = 0

  def step(self, timing=NORMAL):
    # the next camera frame, or the newest one after a stall
    newest = int((self.clock.t - 1000.0) / DT + 1e-9)
    frame = max(self.frame + 1, newest)
    if frame * DT + 1000.0 > self.clock.t:
      self.clock.t = frame * DT + 1000.0
    dropped = max(0, frame - self.frame - 1) if self.frame >= 0 else 0
    self.frame = frame
    self.x = (1 - ALPHA) * self.x + ALPHA * min(dropped, 10)
    self.model.frame_drop_ratio = self.x / (1 + self.x)
    was_big = self.model._active is not self.model._small
    self.client.now = timing
    output = self.model.run({'img': CAMERA, 'big_img': CAMERA}, {'img': None, 'big_img': None},
                            {'desire_pulse': np.zeros(2), 'traffic_convention': np.zeros(2), 'action_t': np.zeros(2)})
    if self.model.handovers != self.handovers:
      self.handovers = self.model.handovers
      self.x = 0.0
    if was_big and self.model._active is self.model._small:
      self.demotes += 1
    if output is not None and dropped == 0:
      self.published_drop_perc.append(100 * self.x / (1 + self.x))
    self.clock.advance(MODELD_OVERHEAD)

  def drive(self, seconds, timing=NORMAL):
    for _ in range(int(seconds / DT)):
      self.step(timing)

  @property
  def big_drives(self):
    return self.model._active is self.big


@contextmanager
def simulated(clock):
  """jetlink's clocks on the simulated one, and no runtime record."""
  fake = NS(monotonic=clock, perf_counter=clock)
  with patch.object(joining, 'time', fake), patch.object(model_state, 'time', fake), \
       patch.object(joining, 'gadget', NS(MODELD='/nonexistent')):
    yield


@pytest.fixture
def modeld():
  clock = Clock()
  with simulated(clock):
    m = Modeld(clock)
    m.drive(1.0)   # the small model's warmup frames, the swap and the proof
    assert m.big_drives
    yield m


def no_soft_disable(m):
  # selfdrived's modeldLagging: frameDropPerc > 1 on a published modelV2
  assert max(m.published_drop_perc) <= 1.0


def stall_6f(m):
  m.step(Timing(data=292.2, held=True, wait=0.1))


def stall_75(m):
  for data, wait in ((98.3, 0.1), (57.9, 0.3), (134.0, 0.2)):
    m.step(Timing(data=data, held=True, wait=wait))


def test_6f_single_stall_does_not_demote(modeld):
  modeld.drive(60)
  stall_6f(modeld)
  modeld.drive(10)
  assert modeld.big_drives and modeld.demotes == 0
  assert modeld.log.has("in a stall of the comma's own")
  no_soft_disable(modeld)


def test_75_stall_does_not_demote(modeld):
  modeld.drive(60)
  # 10:01, one 59.5 ms frame: one dropped frame, forgiven as today
  modeld.step(Timing(data=59.5, held=True, wait=0.1))
  modeld.drive(67)
  stall_75(modeld)
  modeld.drive(10)
  # a host-side held frame later in the drive: comma ~3 ms, reply 43 ms, held, as today
  modeld.step(Timing(data=1.5, wait=43.0, held=True))
  modeld.drive(5)
  assert modeld.big_drives and modeld.demotes == 0
  no_soft_disable(modeld)


@pytest.mark.parametrize('stall', [stall_6f, stall_75])
def test_todays_rule_demoted_both(modeld, stall):
  # the replay reproduces what the car did: with nothing forgiven, both hand back
  with patch.object(joining, 'STALL_FORGIVEN', -1.0):
    modeld.drive(60)
    stall(modeld)
    modeld.drive(1)
  assert not modeld.big_drives and modeld.demotes == 1
  assert modeld.log.has("behind the large model")


def test_host_late_demotes(modeld):
  modeld.drive(60)
  # the host is late (a held frame that waited for its reply), and the frames after it stall
  modeld.step(Timing(data=1.5, wait=43.0, held=True))
  modeld.step(Timing(data=150.0, held=True, wait=0.1))
  modeld.step(Timing(data=150.0, held=True, wait=0.1))
  assert not modeld.big_drives and modeld.demotes == 1
  assert modeld.log.has('the host was late')
  assert modeld.model._retired[1] == BEHIND


def test_slow_send_demotes(modeld):
  # a send that blocks on a host that is not draining the link is the host's
  modeld.drive(60)
  modeld.step(Timing(send=160.0))
  modeld.drive(1)
  assert not modeld.big_drives and modeld.log.has('the host was late')


def test_slow_server_demotes(modeld):
  modeld.drive(60)
  modeld.step(Timing(server=45.0, wait=40.0))
  stall_6f(modeld)
  modeld.drive(1)
  assert not modeld.big_drives and modeld.log.has('the host was late')


def test_repeated_stall_demotes(modeld):
  # 75 after 11:09: the next stall that tripped the limit came 22 s later
  modeld.drive(60)
  stall_75(modeld)
  modeld.drive(22)
  assert modeld.big_drives
  stall_6f(modeld)
  modeld.drive(1)
  assert not modeld.big_drives and modeld.demotes == 1
  assert modeld.log.has('s ago too')


def test_stall_after_the_repeat_window_is_forgiven(modeld):
  modeld.drive(60)
  stall_6f(modeld)
  modeld.drive(joining.STALL_REPEAT + 5)
  stall_75(modeld)
  modeld.drive(5)
  assert modeld.big_drives and modeld.demotes == 0
  no_soft_disable(modeld)


def test_long_stall_demotes(modeld):
  modeld.drive(60)
  modeld.step(Timing(data=500.0, held=True, wait=0.1))
  modeld.drive(1)
  assert not modeld.big_drives and modeld.log.has('ms stall')


def test_burst_past_the_limit_demotes(modeld):
  # stalls 0.3 s apart are one burst; together they lose more than STALL_FORGIVEN
  modeld.drive(60)
  for _ in range(4):
    modeld.step(Timing(data=150.0, held=True, wait=0.1))
    modeld.drive(0.3)
  assert not modeld.big_drives


def test_link_loss_demotes(modeld):
  modeld.drive(60)
  modeld.step(Timing(lost=True))
  assert not modeld.big_drives and modeld.demotes == 1
  assert modeld.model._retired[1] == LOST


def test_holds_in_a_row_still_demote(modeld):
  # a host that stops answering in time: the hold rule, unchanged
  modeld.drive(60)
  for _ in range(model_state.HOLDS_IN_A_ROW + 1):
    modeld.step(Timing(data=1.5, wait=43.0, held=True))
  assert not modeld.big_drives


def test_stall_in_the_proof_demotes():
  clock = Clock()
  with simulated(clock):
    m = Modeld(clock)
    while not m.big_drives:
      m.step()
    m.drive(0.3)
    stall_6f(m)
    m.drive(0.5)
    assert not m.big_drives
