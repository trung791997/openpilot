import os
import shutil
import socket
import sys
import tempfile
import threading
import time

import pytest

from openpilot.starpilot.system.starport import link_probe
from openpilot.starpilot.system.starport.supervisor import LinkWatch

POLL = "RSSI=-50\nLINKSPEED=150\nNOISE=9999\nFREQUENCY=5180\nAVG_RSSI=-52\n"


def test_signal_poll_parses_and_drops_noise(monkeypatch):
  monkeypatch.setattr(link_probe, "wpa_request", lambda *a, **k: POLL)
  assert link_probe.signal_poll("aa0") == {"rssi": -50, "avg_rssi": -52, "linkspeed_mbps": 150, "freq": 5180}


def test_signal_poll_garbage_and_none(monkeypatch):
  monkeypatch.setattr(link_probe, "wpa_request", lambda *a, **k: "FAIL\n\x00junk=\nRSSI=abc")
  assert link_probe.signal_poll("aa0") == {}
  monkeypatch.setattr(link_probe, "wpa_request", lambda *a, **k: None)
  assert link_probe.signal_poll("aa0") == {}


def test_other_wifi_keeps_only_state_and_freq(monkeypatch):
  monkeypatch.setattr(link_probe, "wpa_request",
                      lambda *a, **k: "bssid=aa:bb\nssid=Net\nfreq=2437\nwpa_state=COMPLETED\nip_address=1.2.3.4\n")
  assert link_probe.other_wifi() == {"other_state": "COMPLETED", "other_freq": 2437}


def test_wpa_request_against_fake_supervisor():
  tmp_path = tempfile.mkdtemp(prefix="wpa", dir="/tmp")  # AF_UNIX paths are short
  path = os.path.join(tmp_path, "aa0")
  server = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
  server.bind(path)
  server.settimeout(2)
  seen = []

  def serve():
    data, addr = server.recvfrom(256)
    seen.append((data, addr))
    server.sendto(POLL.encode(), addr)

  thread = threading.Thread(target=serve)
  thread.start()
  try:
    assert link_probe.wpa_request("aa0", "SIGNAL_POLL", ctrl_dir=tmp_path) == POLL
    thread.join(2)
    assert seen[0][0] == b"SIGNAL_POLL"
    assert not os.path.exists(seen[0][1])
  finally:
    server.close()
    shutil.rmtree(tmp_path, ignore_errors=True)


def test_wpa_request_missing_socket_is_fast_none(tmp_path):
  started = time.monotonic()
  assert link_probe.wpa_request("aa0", "STATUS", ctrl_dir=str(tmp_path)) is None
  assert time.monotonic() - started < 0.5


def test_tcp_info_on_non_socket_is_empty():
  assert link_probe.tcp_info(object()) == {}
  assert link_probe.tcp_info(None) == {}


@pytest.mark.skipif(sys.platform != "linux", reason="TCP_INFO layout is Linux only")
def test_tcp_info_on_localhost_pair():
  listener = socket.socket()
  listener.bind(("127.0.0.1", 0))
  listener.listen(1)
  client = socket.create_connection(listener.getsockname())
  peer, _ = listener.accept()
  try:
    client.sendall(b"x" * 100)
    info = link_probe.tcp_info(client)
    for key in ("tcp_rtt_ms", "tcp_rttvar_ms", "tcp_rto_ms", "tcp_unacked", "tcp_lost", "tcp_retrans",
                "tcp_total_retrans", "tcp_cwnd", "tcp_last_ack_ms", "tcp_backoff", "sendq_bytes"):
      assert key in info
    assert isinstance(info["sendq_bytes"], int) and info["sendq_bytes"] >= 0
  finally:
    for s in (client, peer, listener):
      s.close()


def test_link_probe_ring_and_deltas(monkeypatch):
  clock = [100.0]
  counters = iter([{"tx_dropped": 5, "tx_packets": 100}, {"tx_dropped": 7, "tx_packets": 150}, {"tx_dropped": 7, "tx_packets": 151}])
  monkeypatch.setattr(link_probe, "signal_poll", lambda i: {"rssi": -40, "freq": 5180})
  monkeypatch.setattr(link_probe, "other_wifi", lambda i="wlan0": {"other_state": "COMPLETED"})
  monkeypatch.setattr(link_probe, "iface_counters", lambda i: next(counters))
  probe = link_probe.LinkProbe("aa0", ring_seconds=10.0, now=lambda: clock[0])
  first = probe.sample()
  assert "tx_dropped_d" not in first
  clock[0] += 4
  second = probe.sample()
  assert second["tx_dropped_d"] == 2 and second["tx_packets_d"] == 50
  clock[0] += 8  # first sample is now 12 s old and trimmed
  probe.sample()
  tail = probe.tail()
  assert len(tail) == 2 and tail[-1]["ago_s"] == 0.0 and tail[0]["ago_s"] == 8.0
  summary = probe.summary()
  assert summary["rssi"] == -40 and summary["other_state"] == "COMPLETED" and "age_s" not in summary


def test_link_watch_survives_probe_exceptions(monkeypatch):
  class Lease:
    interface = "aa0"
    calls = 0

    def still_connected(self):
      Lease.calls += 1
      return True

  def boom(*a, **k):
    raise RuntimeError("probe broke")

  for name in ("signal_poll", "other_wifi", "tcp_info", "iface_counters"):
    monkeypatch.setattr(link_probe, name, boom)
  watch = LinkWatch(Lease(), lambda *a, **k: None, interval=0.01, sock=object())
  watch.PROBE_INTERVAL = 0.0
  watch.start()
  try:
    deadline = time.monotonic() + 2
    while Lease.calls < 5 and time.monotonic() < deadline:
      time.sleep(0.01)
  finally:
    watch.stop()
  assert Lease.calls >= 5 and watch.lost == ""
