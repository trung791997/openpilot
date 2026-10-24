"""JetsonLink against FIFOs standing in for the FunctionFS endpoint files.

FIFOs do not keep message boundaries as USB transfers do, so the fake server
writes each 28-byte reply on its own with a pause between them.
"""
import os
import threading
import time

import numpy as np
import pytest

from openpilot.starpilot.spot_monitor import jetson_link as J

SHA = "cf2f2c83b12bae56" + "0" * 48


def _hello(tag_hex=SHA):
  return J._RESP.pack(J.RESP_MAGIC, J.HELLO_SEQ, J.STATUS_HELLO, int(tag_hex[:8], 16), 0.0, 0.0, 0.0)


@pytest.fixture
def mount(tmp_path):
  (tmp_path / "ep0").write_bytes(b"")
  os.mkfifo(tmp_path / "ep1")
  os.mkfifo(tmp_path / "ep2")
  return tmp_path


def _server(mount, hello, answer=True, stop=None):
  to_comma = os.open(mount / "ep1", os.O_WRONLY)
  from_comma = os.open(mount / "ep2", os.O_RDONLY)
  os.write(to_comma, hello)
  req_size = J._REQ.size + J.CROP_NBYTES
  while not (stop and stop.is_set()):
    buf = b""
    while len(buf) < req_size:
      chunk = os.read(from_comma, req_size - len(buf))
      if not chunk:
        return
      buf += chunk
    magic, seq, side, n = J._REQ.unpack_from(buf)
    assert magic == J.REQ_MAGIC and n == J.CROP_NBYTES
    crop = np.frombuffer(buf, np.uint8, offset=J._REQ.size)
    if answer:
      time.sleep(0.002)
      os.write(to_comma, J._RESP.pack(J.RESP_MAGIC, seq, J.STATUS_OK, 1000, 0.1, crop.mean() / 255.0, side / 10))


def _link(mount, hello, **kw):
  t = threading.Thread(target=_server, args=(mount, hello), kwargs=kw, daemon=True)
  t.start()
  link = J.JetsonLink(SHA, mount=str(mount))
  link._ensure_open()
  deadline = time.monotonic() + 2
  while link.last_hello == 0 and time.monotonic() < deadline:
    time.sleep(0.01)
  return link


def test_answers_round_trip(mount):
  link = _link(mount, _hello())
  crop = np.full(J.CROP_SHAPE, 51, np.uint8)
  out = link.infer(1, crop)
  assert out is not None
  assert out[1] == pytest.approx(51 / 255.0, abs=1e-6)
  assert out[2] == pytest.approx(0.1, abs=1e-6)


def test_wrong_model_is_never_used(mount):
  link = _link(mount, _hello("deadbeef" + "0" * 56))
  assert link.infer(0, np.zeros(J.CROP_SHAPE, np.uint8)) is None
  assert link.stats['wrong_model_hellos'] == 1 and link.stats['sent'] == 0


def test_silent_server_falls_back(mount):
  link = _link(mount, _hello(), answer=False)
  t0 = time.monotonic()
  assert link.infer(0, np.zeros(J.CROP_SHAPE, np.uint8)) is None
  assert time.monotonic() - t0 < 0.2
  assert link.stats['timeouts'] == 1


def test_no_hello_no_send(mount):
  link = J.JetsonLink(SHA, mount=str(mount))
  assert link.available()
  assert not link.ready()


def test_absent_interface(tmp_path):
  assert not J.JetsonLink(SHA, mount=str(tmp_path / "nope")).available()



def test_cadence_uses_the_jetson_only_while_it_answers():
  assert J.check_interval_ns(1_000_000_000, True) == J.JETSON_INTERVAL_NS == 200_000_000
  assert J.check_interval_ns(1_000_000_000, False) == 1_000_000_000
  assert J.check_interval_ns(300_000_000, False) == 300_000_000
  # per side every 2 * 0.2 s, not below WarningPolicy's 0.2 s smoothing: each check still counts in full
  assert 2 * J.JETSON_INTERVAL_NS / 1e9 >= 0.2
