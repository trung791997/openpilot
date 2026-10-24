"""Optional: run V-ASM's crop on the Jetson over the gadget's second USB interface.

Only when the gadget was built with ffs.vasm (/data/jetlink-vasm, jetlink-root.sh)
and a Jetson's tools/jetson_vasm/vasm_usb_server.py is answering. Any doubt and
infer() returns None, and the caller runs its own CPU model as before:
  - no hello from the Jetson in HELLO_FRESH_S, or the hello names another model
    than the one this process verified (first 4 bytes of the ONNX sha256),
  - no answer within REPLY_TIMEOUT_S,
  - a send still in flight from an earlier call.

Jetlink's owner holds this interface's ep0 (jetlink.comma.vasm_ep0); this
module opens only ep1 (Jetson -> comma) and ep2 (comma -> Jetson), never ep0.

FunctionFS transfers are synchronous and a signal inside one loses bytes
(jetlink.transport.ffs, _IO_SIGNALS), so both I/O threads block every signal.
Writes go in 8 KB pieces: one 372 KB write is one contiguous kmalloc, which an
aged comma serves only after compaction stalls (ffs.FfsTransport.write_chunk).

Wire format: tools/jetson_vasm/vasm_wire.py on openpilot-radar; kept in step by
hand, it is three structs.
"""
from __future__ import annotations

import os
import queue
import signal
import struct
import threading
import time

import numpy as np

MOUNT = '/dev/ffs-vasm'
CROP_SHAPE = (352, 352, 3)
CROP_NBYTES = 352 * 352 * 3
_REQ = struct.Struct("<4sIBxxxI")
_RESP = struct.Struct("<4sIiI3f")
REQ_MAGIC, RESP_MAGIC = b"VAQ1", b"VAR1"
STATUS_OK, STATUS_HELLO, HELLO_SEQ = 0, 100, 0xFFFFFFFF

HELLO_FRESH_S = 2.5
REPLY_TIMEOUT_S = 0.05
REOPEN_S = 5.0
WRITE_CHUNK = 8192
READ_SIZE = 1024


def _mask_signals():
  signal.pthread_sigmask(signal.SIG_BLOCK, signal.valid_signals())


class JetsonLink:
  def __init__(self, model_sha256: str, mount: str = MOUNT):
    self.mount = mount
    self.model_tag = int(model_sha256[:8], 16)
    self.last_hello = 0.0
    self.hello_matches = False
    self.stats = {'sent': 0, 'answered': 0, 'timeouts': 0, 'skipped_busy': 0, 'wrong_model_hellos': 0}
    self._lock = threading.Lock()
    self._seq = 0
    self._waiting: dict[int, list] = {}
    self._requests: queue.Queue = queue.Queue(maxsize=1)
    self._writing = threading.Event()
    self._fd_in = self._fd_out = -1   # ep1 read, ep2 write
    self._next_open = 0.0
    self._threads_started = False

  # -- public ------------------------------------------------------------
  def available(self) -> bool:
    return os.path.exists(os.path.join(self.mount, 'ep0'))

  def ready(self) -> bool:
    return self.hello_matches and time.monotonic() - self.last_hello < HELLO_FRESH_S and not self._writing.is_set()

  def infer(self, side: int, crop_u8: np.ndarray) -> list[float] | None:
    """Class scores [3] from the Jetson, or None for the caller to run its own."""
    self._ensure_open()
    if not self.ready():
      if self._writing.is_set():
        self.stats['skipped_busy'] += 1
      return None
    if crop_u8.shape != CROP_SHAPE or crop_u8.dtype != np.uint8:
      return None
    with self._lock:
      self._seq = (self._seq + 1) % HELLO_SEQ
      seq = self._seq
      slot = [threading.Event(), None]
      self._waiting[seq] = slot
    msg = _REQ.pack(REQ_MAGIC, seq, side, CROP_NBYTES) + np.ascontiguousarray(crop_u8).tobytes()
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
    status, probs = slot[1]
    if status != STATUS_OK:
      return None
    self.stats['answered'] += 1
    return list(probs)

  # -- internals ---------------------------------------------------------
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
      threading.Thread(target=self._read_loop, name='vasm-jetson-rx', daemon=True).start()
      threading.Thread(target=self._write_loop, name='vasm-jetson-tx', daemon=True).start()

  def _close(self):
    fds, self._fd_in, self._fd_out = (self._fd_in, self._fd_out), -1, -1
    self.hello_matches = False
    for fd in fds:
      if fd >= 0:
        try:
          os.close(fd)
        except OSError:
          pass

  def _read_loop(self):
    _mask_signals()
    while True:
      fd = self._fd_in
      if fd < 0:
        time.sleep(0.5)
        continue
      try:
        buf = os.read(fd, READ_SIZE)
      except OSError:
        # EIO/ESHUTDOWN: not configured or unbound; reopen after REOPEN_S
        self._close()
        time.sleep(0.5)
        continue
      if len(buf) != _RESP.size:
        continue
      magic, seq, status, server_us, p0, p1, p2 = _RESP.unpack(buf)
      if magic != RESP_MAGIC:
        continue
      if seq == HELLO_SEQ and status == STATUS_HELLO:
        self.hello_matches = server_us == self.model_tag
        if not self.hello_matches:
          self.stats['wrong_model_hellos'] += 1
        self.last_hello = time.monotonic()
        continue
      self.last_hello = time.monotonic()   # an answer is as good as a hello
      with self._lock:
        slot = self._waiting.pop(seq, None)
      if slot is not None:
        slot[1] = (status, (p0, p1, p2))
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
