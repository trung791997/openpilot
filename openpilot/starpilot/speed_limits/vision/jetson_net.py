"""Optional: run speed-limit vision's detector and classifier on the Jetson over the gadget's ffs.slv interface.

Only when the gadget was built with it (/data/jetlink-slv, jetlink-root.sh) and a Jetson's
tools/jetson_slv/slv_usb_server.py is answering with both models this process verified (the first 4 bytes
of each ONNX sha256, in its hello). Any doubt and forward() runs the OpenCV net as before:
  - no hello in HELLO_FRESH_S, or a hello naming other models,
  - no answer within REPLY_TIMEOUT_S, or an answer of the wrong size,
  - a send still in flight from an earlier call,
  - the Jetson yielding its GPU to the driving model (STATUS_YIELD, tools/jetson_vasm/yield_gate.py):
    nothing is sent until its next plain hello, and forward() raises Yielding instead of running the CPU net,
    so the pass is skipped (VisionModelCore.observe: a frame with no detection) and a lazily built core does
    not rebuild its nets for it. The gate holds a yield for at least HOLD_S (5 s): no new sign is read then.

JetsonNet stands in for a cv2.dnn net (setInput/forward), so VisionModelCore's sign logic is unchanged:
only the forward pass moves. With a loader (a core made with load_nets=False) the OpenCV net is built on the
first pass the Jetson does not answer, inside that pass, and dropped again once the Jetson has answered
every pass for UNLOAD_AFTER_S: the comma holds the CPU nets only while it needs them.
The blob blobFromImage made (u8 * 1/255, swapRB, NCHW) goes as the u8 it came from, rint(blob * 255), which is
exact for those values (tests/test_jetson_net.py).

Jetlink's owner holds this interface's ep0 (jetlink.comma.vasm_ep0.SLV); this module opens only ep1
(Jetson -> comma) and ep2 (comma -> Jetson). I/O threads block every signal and write in 8 KB pieces,
as spot_monitor/jetson_link.py, whose reasons apply here unchanged.

Wire format (tools/jetson_slv/slv_wire.py, kept in step by hand):
  request  SLQ1: <4sIBxxxI magic, seq, kind (0 detector, 1 classifier), nbytes> + u8 [3, S, S]
  response SLR1: <4sIiII magic, seq, status, server_us, nfloats> + f32 [nfloats]
  hello:   seq HELLO_SEQ, status STATUS_HELLO, server_us = detector tag, nfloats = classifier tag
"""
from __future__ import annotations

import ctypes
import os
import queue
import signal
import struct
import threading
import time

import numpy as np

MOUNT = '/dev/ffs-slv'
DETECTOR, CLASSIFIER = 0, 1
SIZES = {DETECTOR: 256, CLASSIFIER: 128}
OUT_SHAPES = {DETECTOR: (1, 5, 1344), CLASSIFIER: (1, 19)}
_REQ = struct.Struct("<4sIBxxxI")
_RESP = struct.Struct("<4sIiII")
REQ_MAGIC, RESP_MAGIC = b"SLQ1", b"SLR1"
STATUS_OK, STATUS_HELLO, STATUS_YIELD, HELLO_SEQ = 0, 100, 101, 0xFFFFFFFF

HELLO_FRESH_S = 2.5
REPLY_TIMEOUT_S = 0.05
REOPEN_S = 5.0
WRITE_CHUNK = 8192
UNLOAD_AFTER_S = 60.0   # Jetson-only passes before a lazily built CPU net is dropped again
READ_SIZE = 32768   # the largest answer, the detector's, is 20 + 6720 * 4 = 26900 bytes: one transfer


class Yielding(RuntimeError):
  """The Jetson lent its GPU to the driving model: skip this pass, do not run it on the CPU."""


def _mask_signals():
  signal.pthread_sigmask(signal.SIG_BLOCK, signal.valid_signals())


def tag(sha256: str) -> int:
  return int(sha256[:8], 16)


class SlvLink:
  def __init__(self, detector_sha256: str, classifier_sha256: str, mount: str = MOUNT):
    self.mount = mount
    self.tags = (tag(detector_sha256), tag(classifier_sha256))
    self.last_hello = 0.0
    self.hello_matches = False
    self.yield_seen = False
    self.stats = {'sent': 0, 'answered': 0, 'timeouts': 0, 'skipped_busy': 0, 'wrong_model_hellos': 0, 'bad_answers': 0, 'yields': 0}
    self._lock = threading.Lock()
    self._seq = 0
    self._waiting: dict[int, list] = {}
    self._requests: queue.Queue = queue.Queue(maxsize=1)
    self._writing = threading.Event()
    self._fd_in = self._fd_out = -1
    self._next_open = 0.0
    self._threads_started = False

  def available(self) -> bool:
    return os.path.exists(os.path.join(self.mount, 'ep0'))

  def ready(self) -> bool:
    return self.hello_matches and time.monotonic() - self.last_hello < HELLO_FRESH_S and not self._writing.is_set()

  def yielding(self) -> bool:
    """The Jetson, still there and running these models, said it is yielding and has not said otherwise."""
    return self.yield_seen and time.monotonic() - self.last_hello < HELLO_FRESH_S

  def infer(self, kind: int, image_u8: np.ndarray) -> np.ndarray | None:
    """The model's output, OUT_SHAPES[kind] float32, from the Jetson; None for the caller to run its own."""
    self._ensure_open()
    if not self.ready():
      if self._writing.is_set():
        self.stats['skipped_busy'] += 1
      return None
    size = SIZES[kind]
    if image_u8.shape != (3, size, size) or image_u8.dtype != np.uint8:
      return None
    with self._lock:
      self._seq = (self._seq + 1) % HELLO_SEQ
      seq = self._seq
      slot = [threading.Event(), None]
      self._waiting[seq] = slot
    msg = _REQ.pack(REQ_MAGIC, seq, kind, image_u8.nbytes) + np.ascontiguousarray(image_u8).tobytes()
    try:
      self._requests.put_nowait(msg)
    except queue.Full:
      self._drop(seq)
      return None
    self.stats['sent'] += 1
    if not slot[0].wait(REPLY_TIMEOUT_S):
      self._drop(seq)
      self.stats['timeouts'] += 1
      return None
    status, out = slot[1]
    if status == STATUS_YIELD:
      return None
    shape = OUT_SHAPES[kind]
    if status != STATUS_OK or out.size != int(np.prod(shape)) or not np.isfinite(out).all():
      self.stats['bad_answers'] += 1
      return None
    self.stats['answered'] += 1
    return out.reshape(shape)

  def _drop(self, seq: int):
    with self._lock:
      self._waiting.pop(seq, None)

  def _ensure_open(self):
    if self._fd_in >= 0 or time.monotonic() < self._next_open or not self.available():
      return
    self._next_open = time.monotonic() + REOPEN_S
    try:
      fd_in = os.open(os.path.join(self.mount, 'ep1'), os.O_RDONLY)
    except OSError:
      return
    try:
      fd_out = os.open(os.path.join(self.mount, 'ep2'), os.O_WRONLY)
    except OSError:
      os.close(fd_in)
      return
    self._fd_in, self._fd_out = fd_in, fd_out
    if not self._threads_started:
      self._threads_started = True
      threading.Thread(target=self._read_loop, name='slv-jetson-rx', daemon=True).start()
      threading.Thread(target=self._write_loop, name='slv-jetson-tx', daemon=True).start()

  def _close(self):
    fds, self._fd_in, self._fd_out = (self._fd_in, self._fd_out), -1, -1
    self.hello_matches = False
    self.yield_seen = False
    for fd in fds:
      if fd >= 0:
        try:
          os.close(fd)
        except OSError:
          pass

  def _read_message(self, fd: int) -> bytes:
    # one FunctionFS read is one USB transfer, one whole answer: a short one is dropped, never stitched
    return os.read(fd, READ_SIZE)

  def _read_loop(self):
    _mask_signals()
    while True:
      fd = self._fd_in
      if fd < 0:
        time.sleep(0.5)
        continue
      try:
        buf = self._read_message(fd)
      except OSError:
        self._close()
        time.sleep(0.5)
        continue
      if len(buf) < _RESP.size:
        continue
      magic, seq, status, server_us, nfloats = _RESP.unpack_from(buf)
      if magic != RESP_MAGIC:
        continue
      if status == STATUS_YIELD:
        self.hello_matches = False   # the driving model needs the GPU: wait for a plain hello
        self.yield_seen = (server_us, nfloats) == self.tags if seq == HELLO_SEQ else True   # an answer: we only send to our models
        self.stats['yields'] += 1
      if seq == HELLO_SEQ and status in (STATUS_HELLO, STATUS_YIELD):
        if status == STATUS_HELLO:
          self.hello_matches = (server_us, nfloats) == self.tags
          self.yield_seen = False
          if not self.hello_matches:
            self.stats['wrong_model_hellos'] += 1
        self.last_hello = time.monotonic()
        continue
      self.last_hello = time.monotonic()
      if len(buf) != _RESP.size + 4 * nfloats:
        out = np.zeros(0, np.float32)
      else:
        out = np.frombuffer(buf, np.float32, offset=_RESP.size).copy()
      with self._lock:
        slot = self._waiting.pop(seq, None)
      if slot is not None:
        slot[1] = (status, out)
        slot[0].set()

  def _write_loop(self):
    _mask_signals()
    while True:
      msg = self._requests.get()
      fd = self._fd_out
      if fd < 0:
        continue
      self._writing.set()
      try:
        view = memoryview(msg)
        for off in range(0, len(view), WRITE_CHUNK):
          piece = view[off:off + WRITE_CHUNK]
          while piece:
            piece = piece[os.write(fd, piece):]
      except OSError:
        self._close()
      finally:
        self._writing.clear()


def blob_to_u8(blob: np.ndarray) -> np.ndarray:
  """blobFromImage(u8, 1/255, swapRB) back to the u8 [3, S, S] it came from."""
  return np.rint(blob[0] * 255.0).clip(0, 255).astype(np.uint8)


def _trim():
  """Hand the dropped net's heap back to the kernel; glibc keeps freed arenas otherwise."""
  try:
    ctypes.CDLL('libc.so.6').malloc_trim(0)
  except (OSError, AttributeError):
    pass


class JetsonNet:
  """A cv2.dnn net whose forward() asks the Jetson first.

  net: the CPU net, or None with a loader that builds it (verified and ABI-checked) when a pass needs it."""

  def __init__(self, net, kind: int, link: SlvLink, loader=None, clock=time.monotonic):
    if net is None and loader is None:
      raise ValueError('JetsonNet needs a net or a loader')
    self.net, self.kind, self.link, self.loader, self.clock = net, kind, link, loader, clock
    self.blob = None
    self.last_cpu = 0.0
    self.stats = {'jetson': 0, 'cpu': 0, 'loads': 0, 'unloads': 0, 'yield_skips': 0}

  def setInput(self, blob):  # noqa: N802 (cv2's name)
    self.blob = blob

  def forward(self):
    out = self.link.infer(self.kind, blob_to_u8(self.blob))
    if out is not None:
      self.stats['jetson'] += 1
      if self.loader is not None and self.net is not None and self.clock() - self.last_cpu >= UNLOAD_AFTER_S:
        self.net = None
        self.stats['unloads'] += 1
        _trim()
      return out
    if self.link.yielding():
      self.stats['yield_skips'] += 1
      raise Yielding('the Jetson is yielding its GPU to the driving model')
    self.stats['cpu'] += 1
    if self.net is None:
      self.net = self.loader()
      self.stats['loads'] += 1
    self.last_cpu = self.clock()
    self.net.setInput(self.blob)
    return self.net.forward()


def present(mount: str = MOUNT) -> bool:
  """The gadget has ffs.slv: make the core with load_nets=False and attach() it."""
  return os.path.exists(os.path.join(mount, 'ep0'))


def attach(core, mount: str = MOUNT) -> SlvLink | None:
  """Route core's detector and classifier through the Jetson when the gadget has ffs.slv; None otherwise.
  A core made with load_nets=False gets loaders instead of nets; without ffs.slv it is given its nets here."""
  link = SlvLink(core.DETECTOR_SHA256, core.CLASSIFIER_SHA256, mount=mount)
  if not link.available():
    if getattr(core, 'nets_deferred', False) is True:
      core.net, core.classifier_net = core.load_detector(), core.load_classifier()
      core.nets_deferred = False
    return None
  core.net = JetsonNet(core.net, DETECTOR, link, loader=core.load_detector)
  core.classifier_net = JetsonNet(core.classifier_net, CLASSIFIER, link, loader=core.load_classifier)
  return link
