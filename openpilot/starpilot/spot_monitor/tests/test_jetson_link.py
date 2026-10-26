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


def test_a_yielding_hello_sends_nothing(mount):
  link = _link(mount, J._RESP.pack(J.RESP_MAGIC, J.HELLO_SEQ, J.STATUS_YIELD, int(SHA[:8], 16), 0.0, 0.0, 0.0))
  assert link.infer(0, np.zeros(J.CROP_SHAPE, np.uint8)) is None
  assert link.stats['yields'] == 1 and link.stats['sent'] == 0


def test_cadence_uses_the_jetson_only_while_it_answers():
  assert J.check_interval_ns(1_000_000_000, True) == J.JETSON_INTERVAL_NS == 200_000_000
  assert J.check_interval_ns(1_000_000_000, False) == 1_000_000_000
  assert J.check_interval_ns(300_000_000, False) == 300_000_000
  # per side every 2 * 0.2 s, not below WarningPolicy's 0.2 s smoothing: each check still counts in full
  assert 2 * J.JETSON_INTERVAL_NS / 1e9 >= 0.2


def test_round_trips_time_each_answer_and_split_out_the_jetsons_share(mount):
  link = _link(mount, _hello())
  for _ in range(3):
    assert link.infer(0, np.zeros(J.CROP_SHAPE, np.uint8)) is not None
  rt = link.round_trips.take()
  assert rt['n'] == 3
  assert rt['jetson_ms'] == {'mean': 1.0, 'p50': 1.0, 'p99': 1.0, 'max': 1.0}   # the fake server says 1000 us
  assert rt['total_ms']['p50'] >= 2.0   # it sleeps 2 ms before answering
  assert rt['link_ms']['mean'] == pytest.approx(rt['total_ms']['mean'] - 1.0, abs=0.02)
  assert link.round_trips.take() == {'n': 0}


def test_timeouts_are_not_round_trips(mount):
  link = _link(mount, _hello(), answer=False)
  assert link.infer(0, np.zeros(J.CROP_SHAPE, np.uint8)) is None
  assert link.stats['timeouts'] == 1 and link.round_trips.take() == {'n': 0}


def test_report_writes_counters_and_round_trips(mount, tmp_path, monkeypatch):
  import json
  path = tmp_path / "stats.json"
  monkeypatch.setattr(J, "TIMING_PATH", str(path))
  link = _link(mount, _hello())
  assert link.infer(0, np.zeros(J.CROP_SHAPE, np.uint8)) is not None
  link._next_report = 0.0
  link._maybe_report()
  report = json.loads(path.read_text())
  assert report['link']['answered'] == 1 and report['round_trips']['n'] == 1
  link._maybe_report()   # not due again for TIMING_SECONDS
  assert json.loads(path.read_text())['t'] == report['t']
