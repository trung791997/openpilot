"""StarPilot UploadRlogs: which rlogs go up, when, and the shutdown hold.

Pure Python on temp dirs. The uploader and power monitor are compiled from their source with stub
collaborators (as in test_uploader_preferences.py), so these run where the compiled params library and
cereal cannot load (macOS); test_uploader.py and test_power_monitoring.py cover the real modules on device.
"""
import ast
import datetime
import os
import threading
import time
from collections.abc import Iterator
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

from openpilot.system.loggerd import rlog_upload
from openpilot.system.loggerd.rlog_upload import (ANCHOR_ATTR_NAME, current_route, ensure_anchor, get_anchor,
                                                  in_upload_window, rlogs_pending)
from openpilot.system.loggerd.xattr_cache import _getxattr, getxattr, setxattr

LOGGERD = Path(__file__).parents[1]
HARDWARE_DIR = Path(__file__).parents[2] / "hardware"
UPLOAD_ATTR = "user.upload.test"
OLD = "00000004--0ac3964c96"
MID = "00000005--4c4e99b08b"
NEW = "00000006--77aa11bb22"


def seg(root: Path, route: str, num: int, *names: str, uploaded: tuple[str, ...] = (), lock: bool = False) -> Path:
  d = root / f"{route}--{num}"
  d.mkdir(parents=True, exist_ok=True)
  for name in names:
    (d / name).write_bytes(b"x" * 16)
    if name in uploaded:
      setxattr(str(d / name), UPLOAD_ATTR, b"1")
  if lock:
    (d / "rlog.lock").write_bytes(b"")
  return d


def mark(path: Path) -> None:
  setxattr(str(path), UPLOAD_ATTR, b"1")


# ---------------- rlog_upload.py ----------------

def test_anchor_is_created_at_the_drive_in_progress_and_kept(tmp_path):
  seg(tmp_path, OLD, 0, "rlog")
  seg(tmp_path, MID, 0, "rlog")
  seg(tmp_path, MID, 1, "rlog")
  (tmp_path / "boot").mkdir()
  assert get_anchor(str(tmp_path)) is None
  assert current_route(str(tmp_path)) == MID
  assert ensure_anchor(str(tmp_path)) == MID
  assert _getxattr(str(tmp_path), ANCHOR_ATTR_NAME) == MID.encode()
  seg(tmp_path, NEW, 0, "rlog")
  assert ensure_anchor(str(tmp_path)) == MID, "the anchor moves only when it is missing"


def test_empty_root_has_no_anchor(tmp_path):
  assert ensure_anchor(str(tmp_path)) is None
  assert get_anchor(str(tmp_path)) is None


def test_upload_window_excludes_backlog_and_non_segments():
  assert not in_upload_window(f"{OLD}--0", MID)
  assert in_upload_window(f"{MID}--3", MID)
  assert in_upload_window(f"{NEW}--0", MID)
  assert not in_upload_window("boot", MID)
  assert not in_upload_window(f"{NEW}--0", None)
  assert not in_upload_window("2024-01-01--10-00-00--0", MID), "old date-named routes sort first"


def test_rlogs_pending(tmp_path):
  root = str(tmp_path)
  seg(tmp_path, OLD, 0, "rlog")  # backlog: never holds
  d = seg(tmp_path, MID, 0, "rlog")
  assert not rlogs_pending(root, UPLOAD_ATTR), "no anchor yet: nothing is pending"
  assert ensure_anchor(root) == MID
  assert rlogs_pending(root, UPLOAD_ATTR)
  assert not rlogs_pending(root, UPLOAD_ATTR, owns=lambda fn: False), "another provider's recording"
  assert rlogs_pending(root, "user.upload.other"), "upload marks are per provider"
  mark(d / "rlog")
  assert not rlogs_pending(root, UPLOAD_ATTR)
  (seg(tmp_path, NEW, 0) / "rlog").write_bytes(b"")
  assert not rlogs_pending(root, UPLOAD_ATTR), "an empty rlog is not pending"
  seg(tmp_path, NEW, 1, "rlog", lock=True)
  assert not rlogs_pending(root, UPLOAD_ATTR), "a locked segment is never uploaded, so it must not hold"


# ---------------- uploader.py (Uploader compiled from source) ----------------

class FakeParams:
  def __init__(self):
    self.values: dict[str, object] = {"UploadRlogs": False, "IsOffroad": False}

  def get(self, key):
    return self.values.get(key)

  def get_bool(self, key):
    return bool(self.values[key])


def load_uploader(params: FakeParams):
  tree = ast.parse((LOGGERD / "uploader.py").read_text())
  body = [n for n in tree.body if isinstance(n, (ast.FunctionDef, ast.ClassDef)) and
          n.name in ("get_directory_sort", "listdir_by_creation", "Uploader")]
  ns: dict = {"os": os, "time": time, "datetime": datetime, "Iterator": Iterator,
              "Api": lambda _: None, "Params": lambda: params,
              "cloudlog": NS(event=lambda *a, **k: None, exception=lambda *a, **k: None, debug=lambda *a, **k: None),
              "getxattr": getxattr, "setxattr": setxattr, "owns_recording": lambda path, root: True,
              "UPLOAD_ATTR_NAME": UPLOAD_ATTR, "UPLOAD_ATTR_VALUE": b"1",
              "RLOG_NAMES": rlog_upload.RLOG_NAMES, "ensure_anchor": ensure_anchor, "get_anchor": get_anchor,
              "in_upload_window": in_upload_window}
  exec(compile(ast.Module(body=body, type_ignores=[]), "uploader.py", "exec"), ns)
  return ns["Uploader"]


@pytest.fixture
def make(tmp_path, monkeypatch):
  monkeypatch.setattr(os, "nice", lambda _: 0)
  params = FakeParams()

  def build():
    return load_uploader(params)("0000000000000000", str(tmp_path)), params
  return build


def drain(up, metered=False) -> list[str]:
  sent = []
  while (d := up.next_file_to_upload(metered)) is not None:
    name, key, fn = d
    assert key not in sent, f"{key} offered twice"
    sent.append(key)
    setxattr(fn, UPLOAD_ATTR, b"1")
  return sent


def test_off_while_driving_sends_only_qlogs(make, tmp_path):
  seg(tmp_path, MID, 0, "qlog", "rlog")
  up, params = make()
  assert drain(up) == [f"{MID}--0/qlog"]
  assert get_anchor(str(tmp_path)) is None, "driving with the toggle off sets no anchor"


def test_on_sends_rlogs_after_qlogs_oldest_first_without_backlog(make, tmp_path):
  for i in (0, 1):
    seg(tmp_path, OLD, i, "qlog", "rlog", uploaded=("qlog",))  # pre-toggle backlog
  for i in (0, 1, 2, 10):
    seg(tmp_path, MID, i, "qlog", "rlog")
  up, params = make()
  params.values["UploadRlogs"] = True
  sent = drain(up, metered=True)  # rlogs ignore metered: phone hotspots report metered
  assert sent == [f"{MID}--{i}/qlog" for i in (0, 1, 2, 10)] + [f"{MID}--{i}/rlog" for i in (0, 1, 2, 10)]
  assert get_anchor(str(tmp_path)) == MID
  for i in (0, 1):
    assert getxattr(str(tmp_path / f"{OLD}--{i}" / "rlog"), UPLOAD_ATTR) is None, "backlog rlog sent"
  # the next drive: its qlogs still lead, then its rlogs
  seg(tmp_path, NEW, 0, "qlog", "rlog")
  assert drain(up) == [f"{NEW}--0/qlog", f"{NEW}--0/rlog"]
  assert not rlogs_pending(str(tmp_path), UPLOAD_ATTR)


def test_off_uploads_rlogs_once_parked_and_keeps_anchor(make, tmp_path):
  seg(tmp_path, MID, 0, "rlog")
  up, params = make()
  assert up.next_file_to_upload(False) is None
  params.values["IsOffroad"] = True
  assert drain(up) == [f"{MID}--0/rlog"]
  params.values["IsOffroad"] = False
  seg(tmp_path, NEW, 0, "rlog")
  assert up.next_file_to_upload(False) is None, "toggle off: nothing while driving"
  assert get_anchor(str(tmp_path)) == MID
  params.values["IsOffroad"] = True
  assert drain(up) == [f"{NEW}--0/rlog"]


def test_locked_segment_rlog_waits(make, tmp_path):
  seg(tmp_path, MID, 0, "qlog", "rlog", lock=True)
  up, params = make()
  params.values["UploadRlogs"] = True
  assert up.next_file_to_upload(False) is None


def test_settled_route_rlogs_are_still_sent(make, tmp_path):
  # a route whose qlog is uploaded would settle; once buried under 3 newer routes a settled route is only
  # rescanned when its mtime changes, so a pending in-window rlog must keep it unsettled until it is sent
  setxattr(str(tmp_path), ANCHOR_ATTR_NAME, MID.encode())
  seg(tmp_path, MID, 0, "qlog", "rlog", uploaded=("qlog",))
  for i in range(3):
    seg(tmp_path, NEW, i, "qlog", "rlog", uploaded=("qlog", "rlog"))
  up, params = make()
  for _ in range(3):  # driving with the toggle off: several scans, nothing sent
    assert drain(up) == []
  assert f"{MID}--0" not in up.settled
  assert all(f"{NEW}--{i}" in up.settled for i in range(3))
  params.values["IsOffroad"] = True
  assert drain(up) == [f"{MID}--0/rlog"]
  assert drain(up) == []
  assert f"{MID}--0" in up.settled, "settles once its rlog is uploaded"


def test_first_anchor_clears_settled_routes(make, tmp_path):
  # the anchor is set after the scan that settled the drive in progress: it must be rescanned, not skipped
  seg(tmp_path, MID, 0, "qlog", "rlog", uploaded=("qlog",))
  for i in range(3):
    seg(tmp_path, MID, i + 1, "qlog", "rlog", uploaded=("qlog", "rlog"))
  up, params = make()
  assert drain(up) == []
  assert f"{MID}--0" in up.settled
  params.values["UploadRlogs"] = True
  assert drain(up) == [f"{MID}--0/rlog"]


def test_pending_rlog_keeps_route_unsettled(make, tmp_path):
  setxattr(str(tmp_path), ANCHOR_ATTR_NAME, MID.encode())
  seg(tmp_path, MID, 0, "qlog", "rlog", uploaded=("qlog",))
  up, params = make()
  assert drain(up) == []  # driving, toggle off: rlog waits
  assert f"{MID}--0" not in up.settled


# ---------------- power_monitoring.py (shutdown hold) ----------------

class Clock:
  def __init__(self, now=100_000.0):
    self.now = now

  def monotonic(self):
    return self.now


def load_power_monitoring(clock: Clock, params: dict):
  tree = ast.parse((HARDWARE_DIR / "power_monitoring.py").read_text())
  body = [n for n in tree.body if not isinstance(n, (ast.Import, ast.ImportFrom))]
  policy = NS(enabled=False, cutoff_tenths=118, delay_hours=1)
  stored = dict(params)

  class Params:
    def get(self, key):
      return stored.get(key)

    def get_bool(self, key):
      return bool(stored.get(key, False))

    def put(self, key, value):
      stored[key] = value

  events = []
  ns: dict = {"time": NS(monotonic=clock.monotonic), "threading": threading, "Params": Params,
              "HARDWARE": NS(get_current_power_draw=lambda: 0), "PowerPolicy": lambda: policy,
              "effective": lambda saved: saved, "read_saved": lambda _: policy,
              "cloudlog": NS(event=lambda *a, **k: events.append(a), exception=lambda *a, **k: events.append(a))}
  exec(compile(ast.Module(body=body, type_ignores=[]), "power_monitoring.py", "exec"), ns)
  ns["MAX_TIME_OFFROAD_S"] = 3600
  return ns, events


@pytest.fixture
def pm_env(tmp_path, monkeypatch):
  import openpilot.common.hardware.hw as hw
  import openpilot.starpilot.connect.provider as provider
  monkeypatch.setattr(hw.Paths, "log_root", staticmethod(lambda: str(tmp_path)))
  monkeypatch.setattr(provider, "active_provider", lambda: NS(upload_attribute=UPLOAD_ATTR))
  monkeypatch.setattr(provider, "owns_recording", lambda fn, root: True)
  clock = Clock()
  ns, events = load_power_monitoring(clock, {})
  pm = ns["PowerMonitoring"]()
  pm.car_battery_capacity_uWh = 30e6
  pm.car_voltage_mV = 12e3
  return NS(pm=pm, clock=clock, root=tmp_path, events=events, ns=ns)


def past_timeout(env, seconds):
  return env.clock.now - 3600 - seconds


def test_timeout_held_while_rlogs_pending_with_no_cap(pm_env):
  seg(pm_env.root, MID, 0, "rlog")
  ensure_anchor(str(pm_env.root))
  for seconds in (60, 10 * 3600):
    pm_env.pm.rlogs_checked_at = None
    assert not pm_env.pm.should_shutdown(False, True, past_timeout(pm_env, seconds), True)
  assert any("held for rlog upload" in str(e) for e in pm_env.events)


def test_hold_ends_once_uploaded(pm_env):
  d = seg(pm_env.root, MID, 0, "rlog")
  ensure_anchor(str(pm_env.root))
  ts = past_timeout(pm_env, 60)
  assert not pm_env.pm.should_shutdown(False, True, ts, True)
  mark(d / "rlog")
  pm_env.clock.now += 10
  assert not pm_env.pm.should_shutdown(False, True, ts, True), "rechecks only every RLOG_UPLOAD_CHECK_S"
  pm_env.clock.now += pm_env.ns["RLOG_UPLOAD_CHECK_S"]
  assert pm_env.pm.should_shutdown(False, True, ts, True)


def test_not_held_without_pending(pm_env):
  seg(pm_env.root, OLD, 0, "rlog")  # backlog only, and no anchor
  assert pm_env.pm.should_shutdown(False, True, past_timeout(pm_env, 60), True)
  ensure_anchor(str(pm_env.root))
  mark(pm_env.root / f"{OLD}--0" / "rlog")
  pm_env.pm.rlogs_checked_at = None
  assert pm_env.pm.should_shutdown(False, True, past_timeout(pm_env, 60), True)


def test_not_held_before_timeout(pm_env):
  seg(pm_env.root, MID, 0, "rlog")
  ensure_anchor(str(pm_env.root))
  assert not pm_env.pm.should_shutdown(False, True, pm_env.clock.now - 600, True)
  assert pm_env.pm.rlogs_checked_at is None, "nothing is scanned until the timer expires"


def test_low_voltage_never_held(pm_env):
  seg(pm_env.root, MID, 0, "rlog")
  ensure_anchor(str(pm_env.root))
  pm_env.pm.car_voltage_mV = 10.8e3
  assert pm_env.pm.should_shutdown(False, True, past_timeout(pm_env, 60), True)
  assert pm_env.pm.should_shutdown(False, True, pm_env.clock.now - 600, True), "low voltage before the timeout too"


def test_battery_exhausted_never_held(pm_env):
  seg(pm_env.root, MID, 0, "rlog")
  ensure_anchor(str(pm_env.root))
  pm_env.pm.car_battery_capacity_uWh = 0
  assert pm_env.pm.should_shutdown(False, True, past_timeout(pm_env, 60), True)


def test_hold_check_failure_does_not_hold(pm_env, monkeypatch):
  monkeypatch.setattr(rlog_upload, "rlogs_pending", lambda *a, **k: (_ for _ in ()).throw(OSError("boom")))
  assert pm_env.pm.should_shutdown(False, True, past_timeout(pm_env, 60), True)
