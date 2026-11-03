"""Rejoin the phone hotspot on wlan0 when it comes up after the comma has given up on it.

The dash map feed (hu_feed.py) rides on the phone hotspot. The phone starts that hotspot only once it reaches the
car's Bluetooth, so the comma often boots first; NetworkManager then spends its autoconnect retries (4 by default),
blocks the connection for about five minutes and may not rescan. 2026-11-03: wlan0 sat "disconnected" with the
hotspot in range at 100 % signal until a manual rescan, after which it joined within a minute.

Every CHECK_EVERY s, while wlan0 is down and not already connecting: if a saved Wi-Fi network for wlan0 is in the
last scan, bring it up (at most once per CON_UP_EVERY s); otherwise ask wlan0 for a fresh scan, read on the next
check. A scan takes the shared radio off-channel and can stall StarPort's car video on aa0 (video stalls dropped
projection on 2026-11-01), so scans back off: every SCAN_EVERY s for SCAN_FAST_FOR s after wlan0 drops or the daemon
starts, then the gap doubles after each scan up to SCAN_MAX s; a new drop starts over. The network is whichever saved, autoconnecting station network for wlan0
(the phone hotspot, or home Wi-Fi) is in range, most recently used first -- the same choice NetworkManager would make.

It never touches aa0 (StarPort's car link on the same radio), and it does nothing while the car link is joining
(a scan can disturb that join) or while a projection session has paused wlan0's autoconnect to use wlan0 itself.
While wlan0 is up it reads one sysfs file and runs no command. Every nmcli failure is logged and swallowed.
"""
from __future__ import annotations

import os
import subprocess
import threading
import time
from collections.abc import Callable
from datetime import datetime

INTERFACE = "wlan0"
CAR_INTERFACE = "aa0"
CAR_CONNECTION = "starpilot-starport"
POLL_EVERY = 2.         # s between sysfs reads of wlan0's link, to log a drop
CHECK_EVERY = 15.       # s between rejoin checks while wlan0 is down
SCAN_EVERY = 15.        # s between scans at first: never more often
SCAN_FAST_FOR = 120.    # s after wlan0 drops (or the daemon starts) that scans stay SCAN_EVERY apart
SCAN_MAX = 120.         # s: the longest gap the doubling reaches
CON_UP_EVERY = 30.      # s: never start a join more often
SAVED_EVERY = 300.      # s a read of the saved connections is reused
COMMAND_TIMEOUT = 5.
CON_UP_WAIT = 25        # s nmcli waits for the join to finish
# Supervisor stages from the car sending its Wi-Fi details until TCP is up: aa0 is joining the car then.
CAR_JOIN_STATES = ("wifi_start", "wifi_info", "joining_wifi", "connecting_tcp")
BUSY = ("connecting", "deactivating")  # nmcli device states of an interface in the middle of a change

Runner = Callable[[list, float], str]


def nmcli(args: list, timeout: float = COMMAND_TIMEOUT) -> str:
  """Run nmcli at low priority in English (C.UTF-8 keeps non-ASCII SSIDs intact); stdout, or raise."""
  result = subprocess.run(["nice", "-n", "19", "nmcli", *args], capture_output=True, text=True, timeout=timeout,
                          env={**os.environ, "LC_ALL": "C.UTF-8"}, check=False)
  if result.returncode != 0:
    raise RuntimeError((result.stderr or result.stdout).strip()[:200] or f"nmcli exit {result.returncode}")
  return result.stdout


def fields(line: str) -> list[str]:
  """Split one ``nmcli -t`` line on unescaped colons."""
  out, field, escaped = [], [], False
  for char in line:
    if escaped:
      field.append(char)
      escaped = False
    elif char == "\\":
      escaped = True
    elif char == ":":
      out.append("".join(field))
      field = []
    else:
      field.append(char)
  out.append("".join(field))
  return out


def operstate(interface: str = INTERFACE) -> str:
  try:
    with open(f"/sys/class/net/{interface}/operstate") as f:
      return f.read().strip()
  except OSError:
    return ""


def _now_text() -> str:
  return datetime.now().isoformat(timespec="seconds")


class HotspotRejoin:
  def __init__(self, log: Callable[..., None], car_joining: Callable[[], bool] = lambda: False,
               run: Runner | None = None, link: Callable[[], str] = operstate, wall: Callable[[], str] = _now_text):
    self.log, self.car_joining, self.link, self.wall = log, car_joining, link, wall
    self.run = run or nmcli
    self._last_error = ""
    self._was_up: bool | None = None
    self._checked_at = -CHECK_EVERY
    self._scanned_at = -SCAN_EVERY
    self._scan_gap = SCAN_EVERY
    self._down_since: float | None = None  # when the current outage began (a drop, or the first look at a down wlan0)
    self._con_up_at = -CON_UP_EVERY
    self._saved: list[dict] = []
    self._saved_at = -SAVED_EVERY
    self._stop = threading.Event()
    self._thread: threading.Thread | None = None

  def start(self) -> HotspotRejoin:
    self._thread = threading.Thread(target=self._loop, name="wlan0_rejoin", daemon=True)
    self._thread.start()
    return self

  def close(self) -> None:
    self._stop.set()

  def _loop(self) -> None:
    while not self._stop.wait(POLL_EVERY):
      try:
        self.poll(time.monotonic())
      except Exception as error:  # never let the watchdog take anything else down
        self.log("wlan0_rejoin_error", error=str(error)[:200] or type(error).__name__)

  def _nmcli(self, args: list) -> str | None:
    """stdout, or None after logging the failure (a repeat of the last failure is not logged again)."""
    try:
      return self.run(args, COMMAND_TIMEOUT)
    except Exception as error:
      text = f"{' '.join(args[:4])}: {str(error)[:200] or type(error).__name__}"
      if text != self._last_error:
        self._last_error = text
        self.log("wlan0_nmcli_failed", error=text)
      return None

  def _restart_scans(self, now: float) -> None:
    self._down_since, self._scan_gap, self._scanned_at = now, SCAN_EVERY, -SCAN_EVERY

  def poll(self, now: float) -> None:
    """Every POLL_EVERY s: note a drop at once, and check for a rejoin every CHECK_EVERY s while down."""
    up = self.link() == "up"
    if up != self._was_up:
      if up and self._was_up is False:
        self.log("wlan0_up", ts=self.wall())
      elif not up and self._was_up:
        self.log("wlan0_dropped", ts=self.wall(), reason=self._reason())
        self._restart_scans(now)
      self._was_up = up
    if not up and now - self._checked_at >= CHECK_EVERY:
      self._checked_at = now
      self.check(now)

  def _reason(self) -> str:
    out = self._nmcli(["-t", "-g", "GENERAL.STATE,GENERAL.REASON", "device", "show", INTERFACE])
    return " / ".join(line.strip() for line in out.splitlines() if line.strip()) if out else ""

  def _devices(self) -> dict[str, tuple[str, str]] | None:
    out = self._nmcli(["-t", "-f", "DEVICE,STATE,CONNECTION", "device"])
    if out is None:
      return None
    devices = {}
    for line in out.splitlines():
      parts = fields(line)
      if len(parts) >= 3:
        devices[parts[0]] = (parts[1], parts[2])
    return devices

  def saved(self, now: float) -> list[dict]:
    """Saved, autoconnecting station networks wlan0 may join, most recently used first."""
    if now - self._saved_at < SAVED_EVERY:
      return self._saved
    out = self._nmcli(["-t", "-f", "NAME,UUID,TYPE,AUTOCONNECT,TIMESTAMP", "connection", "show"])
    if out is None:
      return self._saved
    found = []
    for line in out.splitlines():
      parts = fields(line)
      if len(parts) < 5 or parts[2] != "802-11-wireless" or parts[3] != "yes" or parts[0] == CAR_CONNECTION:
        continue
      detail = self._nmcli(["-t", "-g", "802-11-wireless.ssid,802-11-wireless.mode,connection.interface-name",
                            "connection", "show", "uuid", parts[1]])
      if detail is None:
        continue
      lines = (detail.splitlines() + ["", "", ""])[:3]
      ssid, mode, interface = (":".join(fields(value.strip())) for value in lines)
      if ssid and mode in ("", "infrastructure") and interface in ("", INTERFACE):
        found.append({"name": parts[0], "uuid": parts[1], "ssid": ssid, "used": int(parts[4]) if parts[4].isdigit() else 0})
    self._saved = sorted(found, key=lambda c: -c["used"])
    self._saved_at = now
    return self._saved

  def _visible(self) -> set[str] | None:
    out = self._nmcli(["-t", "-f", "SSID", "device", "wifi", "list", "ifname", INTERFACE, "--rescan", "no"])
    return None if out is None else {":".join(fields(line)) for line in out.splitlines() if line}

  def check(self, now: float) -> str:
    """One rejoin decision; returns what it did, for tests and logs."""
    if self.car_joining():
      return "car_joining"
    devices = self._devices()
    if devices is None or INTERFACE not in devices:
      return "no_device"
    state, _ = devices[INTERFACE]
    car_state = devices.get(CAR_INTERFACE, ("", ""))[0]
    if car_state.startswith(BUSY):
      return "car_joining"
    if state.startswith("connected"):
      return "connected"
    if state.startswith(BUSY):
      return "activating"
    if state != "disconnected":
      return "unavailable"  # unmanaged, unavailable (radio off) or unknown: not ours to change
    paused = self._nmcli(["-t", "-g", "GENERAL.AUTOCONNECT", "device", "show", INTERFACE])
    if paused is None:
      return "nmcli_failed"
    if paused.strip() == "no":
      return "paused"  # a projection session holds wlan0 for the car
    saved = self.saved(now)
    if not saved:
      return "no_saved"
    visible = self._visible()
    if visible is None:
      return "nmcli_failed"
    target = next((c for c in saved if c["ssid"] in visible), None)
    if target is not None:
      if now - self._con_up_at < CON_UP_EVERY:
        return "backoff"
      self._con_up_at = now
      started, error = time.monotonic(), ""
      try:
        self.run(["-w", str(CON_UP_WAIT), "connection", "up", "uuid", target["uuid"], "ifname", INTERFACE], CON_UP_WAIT + 5.)
      except Exception as failure:
        error = str(failure)[:200] or type(failure).__name__
      self.log("wlan0_rejoin", ts=self.wall(), connection=target["name"], ok=not error, s=round(time.monotonic() - started, 1),
               **({"error": error} if error else {}))
      return "con_up"
    if self._down_since is None:
      self._down_since = now
    if now - self._scanned_at < self._scan_gap:
      return "backoff"
    self._scanned_at = now
    if now - self._down_since >= SCAN_FAST_FOR:
      self._scan_gap = min(self._scan_gap * 2, SCAN_MAX)
    self._nmcli(["device", "wifi", "rescan", "ifname", INTERFACE])
    return "scan"
