"""JetsonNet/SlvLink against FIFOs standing in for the FunctionFS endpoint files (as spot_monitor's test_jetson_link)."""
import os
import threading
import time

import numpy as np
import pytest

from openpilot.starpilot.speed_limits.vision import jetson_net as J
from openpilot.starpilot.speed_limits.vision.model import VisionModelCore, cv2

DET_SHA, CLS_SHA = VisionModelCore.DETECTOR_SHA256, VisionModelCore.CLASSIFIER_SHA256


def _hello(det=DET_SHA, cls=CLS_SHA):
  return J._RESP.pack(J.RESP_MAGIC, J.HELLO_SEQ, J.STATUS_HELLO, J.tag(det), J.tag(cls))


@pytest.fixture
def mount(tmp_path):
  (tmp_path / "ep0").write_bytes(b"")
  os.mkfifo(tmp_path / "ep1")
  os.mkfifo(tmp_path / "ep2")
  return tmp_path


def _server(mount, hello, answer=True, nfloats=None, yield_then_hello=False):
  to_comma = os.open(mount / "ep1", os.O_WRONLY)
  from_comma = os.open(mount / "ep2", os.O_RDONLY)
  os.write(to_comma, hello)
  while True:
    head = b""
    while len(head) < J._REQ.size:
      chunk = os.read(from_comma, J._REQ.size - len(head))
      if not chunk:
        return
      head += chunk
    magic, seq, kind, n = J._REQ.unpack(head)
    body = b""
    while len(body) < n:
      body += os.read(from_comma, n - len(body))
    img = np.frombuffer(body, np.uint8)
    count = int(np.prod(J.OUT_SHAPES[kind])) if nfloats is None else nfloats
    out = np.full(count, img.mean() / 255.0 + kind, np.float32)  # proves the image and the kind arrived
    if yield_then_hello:
      os.write(to_comma, J._RESP.pack(J.RESP_MAGIC, seq, J.STATUS_YIELD, 0, 0))
      time.sleep(0.05)
      os.write(to_comma, hello)
    elif answer:
      time.sleep(0.002)
      os.write(to_comma, J._RESP.pack(J.RESP_MAGIC, seq, J.STATUS_OK, 900, count) + out.tobytes())


class _FifoLink(J.SlvLink):
  """A FIFO splits a 27 KB answer that USB delivers as one transfer: reassemble it by its header here only."""

  def _read_message(self, fd):
    buf = os.read(fd, J._RESP.size)
    if len(buf) == J._RESP.size and J._RESP.unpack(buf)[1] != J.HELLO_SEQ:
      want = J._RESP.size + 4 * J._RESP.unpack(buf)[4]
      while len(buf) < want:
        buf += os.read(fd, want - len(buf))
    return buf


def _link(mount, hello, **kw):
  threading.Thread(target=_server, args=(mount, hello), kwargs=kw, daemon=True).start()
  link = _FifoLink(DET_SHA, CLS_SHA, mount=str(mount))
  link._ensure_open()
  deadline = time.monotonic() + 2
  while link.last_hello == 0 and time.monotonic() < deadline:
    time.sleep(0.01)
  return link


def test_blob_recovers_the_exact_u8_image():
  rng = np.random.default_rng(0)
  bgr = rng.integers(0, 256, (256, 256, 3), dtype=np.uint8)
  bgr[0, 0] = (0, 128, 255)
  blob = cv2.dnn.blobFromImage(bgr, scalefactor=1 / 255.0, size=(256, 256), swapRB=True, crop=False)
  u8 = J.blob_to_u8(blob)
  assert u8.shape == (3, 256, 256)
  assert np.array_equal(u8, bgr[:, :, ::-1].transpose(2, 0, 1))


class _CpuNet:
  def __init__(self, shape):
    self.shape, self.calls = shape, 0

  def setInput(self, blob):  # noqa: N802
    self.blob = blob

  def forward(self):
    self.calls += 1
    return np.zeros(self.shape, np.float32)


def test_detector_and_classifier_round_trip(mount):
  link = _link(mount, _hello())
  det = J.JetsonNet(_CpuNet((1, 5, 1344)), J.DETECTOR, link)
  det.setInput(np.full((1, 3, 256, 256), 51 / 255.0, np.float32))
  out = det.forward()
  assert out.shape == (1, 5, 1344) and out[0, 0, 0] == pytest.approx(51 / 255.0, abs=1e-6)
  cls = J.JetsonNet(_CpuNet((1, 19)), J.CLASSIFIER, link)
  cls.setInput(np.full((1, 3, 128, 128), 102 / 255.0, np.float32))
  out = cls.forward()
  assert out.shape == (1, 19) and out[0, 0] == pytest.approx(1 + 102 / 255.0, abs=1e-6)
  assert det.net.calls == cls.net.calls == 0 and link.stats['answered'] == 2


def test_other_models_are_never_used(mount):
  link = _link(mount, _hello(cls="deadbeef" + "0" * 56))
  net = J.JetsonNet(_CpuNet((1, 19)), J.CLASSIFIER, link)
  net.setInput(np.zeros((1, 3, 128, 128), np.float32))
  net.forward()
  assert net.net.calls == 1 and link.stats['sent'] == 0 and link.stats['wrong_model_hellos'] == 1


def test_silent_server_falls_back_fast(mount):
  link = _link(mount, _hello(), answer=False)
  net = J.JetsonNet(_CpuNet((1, 5, 1344)), J.DETECTOR, link)
  net.setInput(np.zeros((1, 3, 256, 256), np.float32))
  t0 = time.monotonic()
  net.forward()
  assert time.monotonic() - t0 < 0.2 and net.net.calls == 1 and link.stats['timeouts'] == 1


def test_a_split_answer_is_dropped_not_stitched(mount):
  # the production reader: a FIFO hands the detector's answer over in pieces, which must never be used
  threading.Thread(target=_server, args=(mount, _hello()), daemon=True).start()
  link = J.SlvLink(DET_SHA, CLS_SHA, mount=str(mount))
  link._ensure_open()
  deadline = time.monotonic() + 2
  while link.last_hello == 0 and time.monotonic() < deadline:
    time.sleep(0.01)
  net = J.JetsonNet(_CpuNet((1, 5, 1344)), J.DETECTOR, link)
  net.setInput(np.zeros((1, 3, 256, 256), np.float32))
  out = net.forward()
  assert net.net.calls == 1 and not out.any() and link.stats['answered'] == 0


def test_wrong_sized_answer_falls_back(mount):
  link = _link(mount, _hello(), nfloats=19)
  net = J.JetsonNet(_CpuNet((1, 5, 1344)), J.DETECTOR, link)
  net.setInput(np.zeros((1, 3, 256, 256), np.float32))
  net.forward()
  assert net.net.calls == 1 and link.stats['bad_answers'] == 1


def test_no_interface_leaves_the_cpu_nets(tmp_path):
  class Core:
    DETECTOR_SHA256, CLASSIFIER_SHA256 = DET_SHA, CLS_SHA
    net, classifier_net = object(), object()
  core = Core()
  nets = (core.net, core.classifier_net)
  assert J.attach(core, mount=str(tmp_path / "nope")) is None
  assert (core.net, core.classifier_net) == nets


def test_vision_core_reads_the_same_through_jetson_net_on_cpu(tmp_path):
  # with no Jetson answering, the wrapped core gives the same detections as the plain one
  plain = VisionModelCore(is_metric=False)
  wrapped = VisionModelCore(is_metric=False)
  link = J.SlvLink(DET_SHA, CLS_SHA, mount=str(tmp_path))
  wrapped.net = J.JetsonNet(wrapped.net, J.DETECTOR, link)
  wrapped.classifier_net = J.JetsonNet(wrapped.classifier_net, J.CLASSIFIER, link)
  frame = np.random.default_rng(1).integers(0, 256, (1208, 1928, 3), dtype=np.uint8)
  assert plain._collect_detector_classifier_proposals(frame) == wrapped._collect_detector_classifier_proposals(frame)
  assert wrapped.net.stats == {'jetson': 0, 'cpu': 1}


def test_a_yielding_hello_sends_nothing(mount):
  link = _link(mount, J._RESP.pack(J.RESP_MAGIC, J.HELLO_SEQ, J.STATUS_YIELD, J.tag(DET_SHA), J.tag(CLS_SHA)))
  assert link.infer(J.DETECTOR, np.zeros((3, 256, 256), np.uint8)) is None
  assert link.stats['yields'] == 1 and link.stats['sent'] == 0


def test_a_yield_answer_stops_sends_until_a_plain_hello(mount):
  link = _link(mount, _hello(), yield_then_hello=True)
  assert link.infer(J.CLASSIFIER, np.zeros((3, 128, 128), np.uint8)) is None
  assert link.stats['yields'] == 1 and not link.hello_matches
  deadline = time.monotonic() + 1
  while not link.ready() and time.monotonic() < deadline:
    time.sleep(0.01)
  assert link.ready()
