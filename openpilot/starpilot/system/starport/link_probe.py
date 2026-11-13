"""Link diagnostics for the Android Auto Wi-Fi link: logging only, never behaviour.

Everything here is exception-safe (a failed probe yields {} or None) and bounded by short socket timeouts.
Nothing logged identifies a network: no MACs, BSSIDs, IPs or SSIDs. Call only from background threads.
"""
from __future__ import annotations

import itertools
import os
import socket
import struct
import time
from collections import deque
from collections.abc import Callable
from typing import Any

_COUNTER = itertools.count()
_TCP_INFO = getattr(socket, "TCP_INFO", 11)
_TIOCOUTQ = 0x5411
# struct tcp_info: 8 bytes of u8 fields, then 24 u32 fields (include/uapi/linux/tcp.h).
_TCP_INFO_U8 = ("state", "ca_state", "retransmits", "probes", "backoff", "options", "wscale", "flags")
_TCP_INFO_U32 = ("rto", "ato", "snd_mss", "rcv_mss", "unacked", "sacked", "lost", "retrans", "fackets",
                 "last_data_sent", "last_ack_sent", "last_data_recv", "last_ack_recv",
                 "pmtu", "rcv_ssthresh", "rtt", "rttvar", "snd_ssthresh", "snd_cwnd", "advmss", "reordering",
                 "rcv_rtt", "rcv_space", "total_retrans")
_TCP_INFO_FORMAT = "=" + "B" * len(_TCP_INFO_U8) + "I" * len(_TCP_INFO_U32)
_TCP_INFO_SIZE = struct.calcsize(_TCP_INFO_FORMAT)


def wpa_request(interface: str, command: str, ctrl_dir: str = "/run/wpa_supplicant", timeout: float = 0.2) -> str | None:
  """One request over wpa_supplicant's control socket; None on any failure."""
  sock = None
  local = f"/tmp/starport_wpa_{os.getpid()}_{next(_COUNTER)}"
  try:
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    sock.settimeout(timeout)
    sock.bind(local)
    sock.connect(os.path.join(ctrl_dir, interface))
    sock.send(command.encode())
    return sock.recv(4096).decode("utf-8", "replace")
  except Exception:
    return None
  finally:
    if sock is not None:
      try:
        sock.close()
      except Exception:
        pass
    try:
      os.unlink(local)
    except OSError:
      pass


def _int(value: str) -> int | None:
  try:
    return int(value.strip())
  except (ValueError, AttributeError):
    return None


def _kv(text: str | None) -> dict[str, str]:
  out: dict[str, str] = {}
  for line in (text or "").splitlines():
    key, sep, value = line.partition("=")
    if sep:
      out[key.strip()] = value.strip()
  return out


def signal_poll(interface: str, **kwargs: Any) -> dict:
  try:
    fields = _kv(wpa_request(interface, "SIGNAL_POLL", **kwargs))
    out: dict[str, int] = {}
    for src, dst in (("RSSI", "rssi"), ("AVG_RSSI", "avg_rssi"), ("LINKSPEED", "linkspeed_mbps"), ("FREQUENCY", "freq")):
      value = _int(fields.get(src, ""))
      if value is not None:
        out[dst] = value
    return out  # NOISE is dropped: drivers report 9999 as "unknown"
  except Exception:
    return {}


def other_wifi(interface: str = "wlan0", **kwargs: Any) -> dict:
  """The comma's own Wi-Fi state only; never its network name, address or BSSID."""
  try:
    fields = _kv(wpa_request(interface, "STATUS", **kwargs))
    out: dict[str, Any] = {}
    if fields.get("wpa_state"):
      out["other_state"] = fields["wpa_state"]
    freq = _int(fields.get("freq", ""))
    if freq is not None:
      out["other_freq"] = freq
    return out
  except Exception:
    return {}


def tcp_info(sock: Any) -> dict:
  """Kernel TCP state of a connected socket (Linux only) plus unsent bytes in its send queue."""
  dup = None
  try:
    import fcntl
    import termios
    dup = socket.socket(fileno=os.dup(sock.fileno()))
    raw = dup.getsockopt(socket.IPPROTO_TCP, _TCP_INFO, 256)
    if len(raw) < _TCP_INFO_SIZE:
      return {}
    values = struct.unpack(_TCP_INFO_FORMAT, raw[:_TCP_INFO_SIZE])
    info = dict(zip(_TCP_INFO_U8 + _TCP_INFO_U32, values))
    out = {"tcp_rtt_ms": round(info["rtt"] / 1000.0, 1), "tcp_rttvar_ms": round(info["rttvar"] / 1000.0, 1),
           "tcp_rto_ms": round(info["rto"] / 1000.0, 1), "tcp_unacked": info["unacked"], "tcp_lost": info["lost"],
           "tcp_retrans": info["retrans"], "tcp_total_retrans": info["total_retrans"], "tcp_cwnd": info["snd_cwnd"],
           "tcp_last_ack_ms": info["last_ack_recv"], "tcp_backoff": info["backoff"]}
    try:
      out["sendq_bytes"] = struct.unpack("i", fcntl.ioctl(dup.fileno(), getattr(termios, "TIOCOUTQ", _TIOCOUTQ), b"\0\0\0\0"))[0]
    except Exception:
      pass
    return out
  except Exception:
    return {}
  finally:
    if dup is not None:
      try:
        dup.close()
      except Exception:
        pass


_COUNTERS = ("tx_packets", "tx_errors", "tx_dropped", "rx_packets")


def iface_counters(interface: str) -> dict:
  out: dict[str, int] = {}
  base = f"/sys/class/net/{interface}"
  for name, path in [(c, f"{base}/statistics/{c}") for c in _COUNTERS] + [("carrier_changes", f"{base}/carrier_changes")]:
    try:
      with open(path) as handle:
        out[name] = int(handle.read().strip())
    except (OSError, ValueError):
      pass
  return out


class LinkProbe:
  def __init__(self, interface: str, other_interface: str = "wlan0", ring_seconds: float = 30.0,
               now: Callable[[], float] = time.monotonic):
    self.interface, self.other_interface, self.ring_seconds, self._now = interface, other_interface, ring_seconds, now
    self._ring: deque[dict] = deque()
    self._previous: dict[str, int] | None = None

  def sample(self, sock: Any = None) -> dict:
    out: dict[str, Any] = {}
    try:
      out.update(signal_poll(self.interface))
      out.update(other_wifi(self.other_interface))
      if sock is not None:
        out.update(tcp_info(sock))
      counters = iface_counters(self.interface)
      if self._previous is not None:
        for name in _COUNTERS:
          if name in counters and name in self._previous:
            out[f"{name}_d"] = counters[name] - self._previous[name]
      self._previous = counters
      if "carrier_changes" in counters:
        out["carrier_changes"] = counters["carrier_changes"]
    except Exception:
      pass
    now = self._now()
    out["age_s"] = now
    self._ring.append(out)
    while self._ring and now - self._ring[0]["age_s"] > self.ring_seconds:
      self._ring.popleft()
    return out

  def summary(self) -> dict:
    if not self._ring:
      return {}
    latest = self._ring[-1]
    keys = ("rssi", "linkspeed_mbps", "freq", "other_state", "other_freq", "tcp_rtt_ms", "tcp_unacked",
            "tcp_total_retrans", "sendq_bytes", "tx_dropped_d")
    return {key: latest[key] for key in keys if key in latest}

  def tail(self) -> list[dict]:
    now = self._now()
    return [{**{k: v for k, v in sample.items() if k != "age_s"}, "ago_s": round(now - sample["age_s"], 1)}
            for sample in list(self._ring)]
