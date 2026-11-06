"""wlan0 rejoins the phone hotspot by itself once it is in range, without hammering nmcli or touching the car link."""

import json

from openpilot.starpilot.system.starport import scan_quiet, wifi_rejoin as wr
from openpilot.starpilot.system.starport.identity import boot_id

SAVED = "".join(line + "\n" for line in ("iqpilot connection Galaxy S20 FE 5G6CC3:8a97:802-11-wireless:yes:1793727569",
                                         "starpilot-starport:aaaa:802-11-wireless:no:1793727579",
                                         "Hotspot:hhhh:802-11-wireless:no:0",
                                         "lte:llll:gsm:yes:0",
                                         "openpilot connection LHE Casa:cccc:802-11-wireless:yes:0"))
# fourth line of each detail is 802-11-wireless.band: "" (unset) for both saved profiles below
DETAIL = {"8a97": "Galaxy S20 FE 5G6CC3\ninfrastructure\n\n\n", "cccc": "LHE Casa\ninfrastructure\nwlan0\n\n"}


class Nmcli:
  def __init__(self, wlan0="disconnected", aa0="connected", visible=("Galaxy S20 FE 5G6CC3",), autoconnect="yes",
               saved=SAVED, detail=None):
    self.wlan0, self.aa0, self.autoconnect = wlan0, aa0, autoconnect
    self.visible = list(visible)  # each entry an SSID (defaults to 2412 MHz) or an (SSID, freq_mhz) tuple
    self.saved, self.detail = saved, DETAIL if detail is None else detail
    self.calls: list[list] = []
    self.fail: set[str] = set()

  def __call__(self, args, timeout):
    self.calls.append(args)
    key = " ".join(args)
    for word in self.fail:
      if word in key:
        raise RuntimeError(f"{word} failed")
    if args[:3] == ["device", "set", "wlan0"] and args[3] == "autoconnect":
      self.autoconnect = args[4]
      return ""
    if args[-1:] == ["device"]:
      return f"wlan0:{self.wlan0}:\naa0:{self.aa0}:starpilot-starport\nlo:connected (externally):lo\n"
    if "GENERAL.AUTOCONNECT" in key:
      return self.autoconnect + "\n"
    if "GENERAL.STATE,GENERAL.REASON" in key:
      return "30 (disconnected)\n8 (The supplicant is now unavailable)\n"
    if args[-2:] == ["connection", "show"]:
      return self.saved
    if args[:2] == ["-t", "-g"] and "uuid" in args:
      return self.detail[args[-1]]
    if "list" in args:
      out = []
      for entry in self.visible:
        ssid, freq = entry if isinstance(entry, tuple) else (entry, 2412)
        out.append(ssid.replace(":", "\\:") + f":{freq} MHz\n")
      return "".join(out)
    return ""

  def did(self, *words):
    return [c for c in self.calls if all(w in c for w in words)]


def make(fake, joining=False, link="down"):
  logs = []
  watch = wr.HotspotRejoin(log=lambda event, **f: logs.append((event, f)), car_joining=lambda: joining, run=fake,
                           link=lambda: link, wall=lambda: "2026-11-03T11:38:40")
  return watch, logs


def test_connected_does_nothing():
  fake = Nmcli(wlan0="connected")
  watch, logs = make(fake)
  assert watch.check(100.) == "connected"
  assert not fake.did("rescan") and not fake.did("up") and logs == []


def test_link_up_runs_no_command():
  fake = Nmcli()
  watch, _ = make(fake, link="up")
  for t in range(0, 120, 2):
    watch.poll(float(t))
  assert fake.calls == []


def test_disconnected_and_hotspot_visible_brings_it_up():
  fake = Nmcli()
  watch, logs = make(fake)
  assert watch.check(100.) == "con_up"
  up = fake.did("up")
  assert up == [["-w", str(wr.CON_UP_WAIT), "connection", "up", "uuid", "8a97", "ifname", "wlan0"]]
  assert not fake.did("rescan")
  assert logs == [("wlan0_rejoin", {"ts": "2026-11-03T11:38:40", "connection": "iqpilot connection Galaxy S20 FE 5G6CC3",
                                    "ok": True, "s": logs[0][1]["s"]})]


def test_most_recently_used_saved_network_wins():
  fake = Nmcli(visible=("LHE Casa", "Galaxy S20 FE 5G6CC3"))
  watch, _ = make(fake)
  watch.check(100.)
  assert fake.did("up")[0][5] == "8a97"


def test_ap_car_and_other_interface_profiles_are_never_used():
  fake = Nmcli(visible=("AAWireless-b1115560", "weedle"))
  watch, _ = make(fake)
  assert watch.check(100.) == "scan"
  assert not fake.did("up")
  assert {c["uuid"] for c in watch.saved(100.)} == {"8a97", "cccc"}


def test_hotspot_absent_scans_only():
  fake = Nmcli(visible=("Someone Else",))
  watch, _ = make(fake)
  assert watch.check(100.) == "scan"
  assert fake.did("rescan") == [["device", "wifi", "rescan", "ifname", "wlan0"]]
  assert not fake.did("up")


def test_activating_waits():
  for state in ("connecting (prepare)", "connecting (getting IP configuration)", "deactivating"):
    fake = Nmcli(wlan0=state)
    watch, _ = make(fake)
    assert watch.check(100.) == "activating"
    assert not fake.did("rescan") and not fake.did("up")


def test_unavailable_or_unmanaged_left_alone():
  for state in ("unavailable", "unmanaged"):
    fake = Nmcli(wlan0=state)
    watch, _ = make(fake)
    assert watch.check(100.) == "unavailable"
    assert not fake.did("rescan") and not fake.did("up")


def test_car_link_joining_skips():
  fake = Nmcli()
  watch, _ = make(fake, joining=True)
  assert watch.check(100.) == "car_joining"
  assert fake.calls == []
  fake = Nmcli(aa0="connecting (configuring)")
  watch, _ = make(fake)
  assert watch.check(100.) == "car_joining"
  assert not fake.did("rescan") and not fake.did("up")


def test_projection_holding_wlan0_skips():
  fake = Nmcli(autoconnect="no")
  watch, _ = make(fake)
  assert watch.check(100.) == "paused"
  assert not fake.did("rescan") and not fake.did("up")


def test_backoff_respected():
  fake = Nmcli()
  watch, _ = make(fake)
  # con up at most once per CON_UP_EVERY, even when the join failed
  fake.fail.add("up uuid")
  assert watch.check(100.) == "con_up"
  assert watch.check(115.) == "backoff"
  assert watch.check(129.9) == "backoff"
  assert watch.check(130.) == "con_up"
  assert len(fake.did("up")) == 2
  # scans at most once per SCAN_EVERY
  fake = Nmcli(visible=())
  watch, _ = make(fake)
  assert [watch.check(t) for t in (100., 105., 114.9, 115.)] == ["scan", "backoff", "backoff", "scan"]
  assert len(fake.did("rescan")) == 2


def scan_times(fake, watch, start, end, step=15.):
  times, t = [], start
  while t < end:
    before = len(fake.did("rescan"))
    watch.check(t)
    if len(fake.did("rescan")) > before:
      times.append(t)
    t += step
  return times


def test_scans_back_off_after_two_minutes():
  fake = Nmcli(visible=())
  watch, _ = make(fake)
  # 15 s apart for the first 2 minutes, then 30, 60, 120, 120
  assert scan_times(fake, watch, 0., 600.) == [0., 15., 30., 45., 60., 75., 90., 105., 120., 150., 210., 330., 450., 570.]


def test_new_drop_resets_scan_backoff():
  fake = Nmcli(visible=())
  state = {"link": "down"}
  watch, _ = make(fake)
  watch.link = lambda: state["link"]
  for t in range(0, 600, 2):
    watch.poll(float(t))
  assert watch._scan_gap == wr.SCAN_MAX
  state["link"] = "up"
  watch.poll(600.)
  state["link"] = "down"
  rescans = len(fake.did("rescan"))
  for t in range(602, 662, 2):
    watch.poll(float(t))
  assert watch._scan_gap == wr.SCAN_EVERY
  assert len(fake.did("rescan")) - rescans == 4  # 602, 618, 634, 650: back to 15 s


def test_visible_network_joins_even_while_scans_are_slow():
  fake = Nmcli(visible=())
  watch, _ = make(fake)
  scan_times(fake, watch, 0., 400.)
  assert watch._scan_gap == wr.SCAN_MAX
  fake.visible = ["Galaxy S20 FE 5G6CC3"]
  assert watch.check(405.) == "con_up"


def test_poll_checks_every_15s_while_down():
  fake = Nmcli(visible=())
  watch, _ = make(fake)
  for t in range(0, 60, 2):
    watch.poll(float(t))
  assert len(fake.did("rescan")) == 4  # t = 0, 16, 32, 48


def test_failed_join_is_logged_with_error():
  fake = Nmcli()
  fake.fail.add("up uuid")
  watch, logs = make(fake)
  watch.check(100.)
  event, f = logs[-1]
  assert event == "wlan0_rejoin" and f["ok"] is False and "failed" in f["error"]


def test_nmcli_failure_swallowed_and_logged_once():
  for word in ("DEVICE,STATE", "GENERAL.AUTOCONNECT", "connection show", "list", "rescan"):
    fake = Nmcli(visible=())
    fake.fail.add(word)
    watch, logs = make(fake)
    watch.check(100.)
    watch.check(200.)
    failures = [f for e, f in logs if e == "wlan0_nmcli_failed"]
    assert len(failures) == 1, word
  # nmcli missing entirely
  def missing(args, timeout):
    raise FileNotFoundError("nmcli")
  watch, logs = make(missing)
  assert watch.check(100.) == "no_device"
  watch.poll(200.)


def test_drop_and_return_logged_once_each():
  fake = Nmcli(wlan0="connected")
  state = {"link": "up"}
  logs = []
  watch = wr.HotspotRejoin(log=lambda event, **f: logs.append((event, f)), run=fake, link=lambda: state["link"],
                           wall=lambda: "2026-11-03T11:39:18")
  watch.poll(0.)
  state["link"] = "dormant"
  watch.poll(2.)
  watch.poll(4.)
  state["link"] = "up"
  watch.poll(6.)
  events = [e for e, _ in logs]
  assert events == ["wlan0_dropped", "wlan0_up"]
  assert logs[0][1] == {"ts": "2026-11-03T11:39:18", "reason": "30 (disconnected) / 8 (The supplicant is now unavailable)"}


def test_no_drop_logged_at_start_when_down():
  fake = Nmcli(wlan0="connected")
  watch, logs = make(fake)
  watch.poll(0.)
  assert not [e for e, _ in logs if e == "wlan0_dropped"]


def test_ssid_with_colon_matches():
  fake = Nmcli(visible=("Peter:s phone",))
  watch, _ = make(fake)
  watch._saved, watch._saved_at = [{"name": "p", "uuid": "pppp", "ssid": "Peter:s phone", "used": 1}], 100.
  assert watch.check(100.) == "con_up"


def test_terse_fields_unescape():
  assert wr.fields(r"Peter\:s phone:conn\\x") == ["Peter:s phone", "conn\\x"]


def test_watchdog_thread_errors_never_raise(monkeypatch):
  monkeypatch.setattr(wr, "POLL_EVERY", 0.001)
  watch, logs = make(Nmcli())
  def boom():
    watch._stop.set()
    raise OSError("boom")
  watch.link = boom
  watch._loop()
  assert logs == [("wlan0_rejoin_error", {"error": "boom"})]


# ---- streaming: keep wlan0 quiet while Android Auto is on the car screen (2026-11-05 16:02 aa0 drop) ----


class Beat:
  """The supervisor's streaming heartbeat: None when not projecting, else the time of the last stats update."""
  def __init__(self, at=None):
    self.at = at

  def __call__(self):
    return self.at


def make_quiet(fake, tmp_path, beat, joining=None, link="down"):
  logs, state = [], {"link": link, "joining": joining or (lambda: False)}
  watch = wr.HotspotRejoin(log=lambda event, **f: logs.append((event, f)), car_joining=lambda: state["joining"](), run=fake,
                           link=lambda: state["link"], wall=lambda: "2026-11-05T16:02:08", streaming_beat=beat,
                           lease=tmp_path / "lease.json", marker=tmp_path / "paused.json",
                           session_marker=tmp_path / "session.json")
  return watch, logs, state


def events(logs):
  return [e for e, _ in logs]


def test_streaming_check_neither_scans_nor_joins(tmp_path):
  fake = Nmcli()  # wlan0 down, hotspot visible: without projection this would join it
  watch, _, _ = make_quiet(fake, tmp_path, Beat(99.5))
  assert watch.check(100.) == "streaming"
  assert not fake.did("rescan") and not fake.did("up") and not fake.did("list")


def test_streaming_drop_pauses_autoconnect_and_never_scans(tmp_path):
  fake = Nmcli(visible=())
  beat = Beat(0.)
  watch, logs, state = make_quiet(fake, tmp_path, beat, link="up")
  watch.poll(0.)
  assert fake.calls == []  # connected wlan0 while projecting: nothing is run, nothing is disconnected
  assert scan_quiet.quiet(1., tmp_path / "lease.json")
  state["link"] = "down"
  for t in range(2, 300, 2):
    beat.at = float(t) - 0.5
    watch.poll(float(t))
  assert fake.did("device", "set", "wlan0", "autoconnect", "no") == [["device", "set", "wlan0", "autoconnect", "no"]]
  assert fake.autoconnect == "no"
  assert not fake.did("rescan") and not fake.did("up") and not fake.did("disconnect")
  assert json.loads((tmp_path / "paused.json").read_text()) == {"interface": "wlan0", "boot": boot_id()}
  assert "wlan0_scans_paused" in events(logs) and scan_quiet.quiet(299., tmp_path / "lease.json")


def test_projection_end_restores_autoconnect_and_scans(tmp_path):
  fake = Nmcli(visible=())
  beat = Beat(0.)
  watch, logs, _ = make_quiet(fake, tmp_path, beat)
  watch.poll(0.)
  assert fake.autoconnect == "no"
  beat.at = None  # projection stopped
  watch.poll(2.)
  assert fake.autoconnect == "yes" and not (tmp_path / "paused.json").exists()
  assert not (tmp_path / "lease.json").exists() and not scan_quiet.quiet(2., tmp_path / "lease.json")
  assert "wlan0_scans_resumed" in events(logs)
  for t in range(4, 40, 2):
    watch.poll(float(t))
  assert fake.did("rescan")  # the fast-scan window starts over once projection ends


def test_stale_beat_restores_scans(tmp_path):
  """A hung stream loop or a dead supervisor stops the beat; scans and autoconnect come back on their own."""
  fake = Nmcli(visible=())
  watch, _, _ = make_quiet(fake, tmp_path, Beat(0.))
  watch.poll(1.)
  assert fake.autoconnect == "no"
  watch.poll(scan_quiet.STALE_AFTER - 0.1)
  assert fake.autoconnect == "no"
  watch.poll(scan_quiet.STALE_AFTER)
  assert fake.autoconnect == "yes"
  assert watch.check(100.) != "streaming"


def test_lease_expires_without_renewal(tmp_path):
  lease = tmp_path / "lease.json"
  assert not scan_quiet.quiet(0., lease)  # no lease
  assert scan_quiet.hold(10., lease)
  assert scan_quiet.quiet(9.9, lease) and not scan_quiet.quiet(10., lease)
  scan_quiet.hold(1000., lease)  # implausibly far ahead (another boot's clock): ignored
  assert not scan_quiet.quiet(0., lease)
  lease.write_text("garbage")
  assert not scan_quiet.quiet(0., lease)
  scan_quiet.drop(lease)
  scan_quiet.drop(lease)
  assert not lease.exists()


def test_fresh_beat():
  assert scan_quiet.fresh(5., 5.) and scan_quiet.fresh(5., 5. + scan_quiet.STALE_AFTER - 0.01)
  assert not scan_quiet.fresh(None, 5.) and not scan_quiet.fresh(5., 5. + scan_quiet.STALE_AFTER)
  assert not scan_quiet.fresh(10., 5.)  # a beat from the future is not trusted


def test_autoconnect_already_off_is_left_alone(tmp_path):
  fake = Nmcli(autoconnect="no")  # a session projecting over wlan0 (or the user) turned it off
  beat = Beat(0.)
  watch, _, _ = make_quiet(fake, tmp_path, beat)
  watch.poll(0.)
  beat.at = None
  for t in range(2, 60, 2):
    watch.poll(float(t))
  assert not fake.did("device", "set") and fake.autoconnect == "no"


def test_restart_after_crash_restores_from_marker(tmp_path):
  (tmp_path / "paused.json").write_text(json.dumps({"interface": "wlan0", "boot": boot_id()}))
  fake = Nmcli(autoconnect="no", wlan0="connected")
  watch, logs, _ = make_quiet(fake, tmp_path, Beat(None), link="up")
  watch.poll(0.)
  assert fake.autoconnect == "yes" and not (tmp_path / "paused.json").exists()
  assert "wlan0_scans_resumed" in events(logs)


def test_marker_from_an_earlier_boot_is_discarded(tmp_path):
  (tmp_path / "paused.json").write_text(json.dumps({"interface": "wlan0", "boot": "an-old-boot"}))
  fake = Nmcli(wlan0="connected")
  watch, _, _ = make_quiet(fake, tmp_path, Beat(None), link="up")
  watch.poll(0.)
  assert not (tmp_path / "paused.json").exists() and fake.calls == []


def test_restore_waits_for_the_car_join(tmp_path):
  fake = Nmcli(visible=())
  beat, joining = Beat(0.), {"on": False}
  watch, _, _ = make_quiet(fake, tmp_path, beat, joining=lambda: joining["on"])
  watch.poll(0.)
  beat.at, joining["on"] = None, True  # projection dropped and a new car join started at once
  watch.poll(2.)
  assert fake.autoconnect == "no"
  joining["on"] = False
  watch.poll(4.)
  assert fake.autoconnect == "yes"


def test_session_holding_wlan0_restores_it_itself(tmp_path):
  fake = Nmcli(visible=())
  beat = Beat(0.)
  watch, logs, _ = make_quiet(fake, tmp_path, beat)
  watch.poll(0.)
  (tmp_path / "session.json").write_text(json.dumps({"interface": "wlan0", "boot": boot_id()}))
  beat.at = None
  watch.poll(2.)
  assert fake.autoconnect == "no" and not fake.did("autoconnect", "yes")
  assert not (tmp_path / "paused.json").exists() and ("wlan0_scans_resumed", {"ts": "2026-11-05T16:02:08",
                                                                              "held_by_session": True}) in logs


def test_failed_toggles_retry_slowly(tmp_path):
  fake = Nmcli(visible=())
  fake.fail = {"autoconnect no"}
  beat = Beat(0.)
  watch, _, _ = make_quiet(fake, tmp_path, beat)
  for t in range(0, 30, 2):
    beat.at = float(t)
    watch.poll(float(t))
  assert len(fake.did("autoconnect", "no")) == 2  # t=0 and t=16 s
  assert not (tmp_path / "paused.json").exists()
  fake.fail = set()
  beat.at = 32.
  watch.poll(32.)  # the second failure was at t=16 s
  assert fake.autoconnect == "no"
  fake.fail = {"autoconnect yes"}
  beat.at = None
  for t in range(34, 48, 2):
    watch.poll(float(t))
  assert len(fake.did("autoconnect", "yes")) == 1 and (tmp_path / "paused.json").exists()
  assert watch.check(100.) == "paused"  # still off: no scan meanwhile
  fake.fail = set()
  watch.poll(50.)
  assert fake.autoconnect == "yes" and not (tmp_path / "paused.json").exists()


def test_close_restores_autoconnect_and_drops_the_lease(tmp_path):
  fake = Nmcli(visible=())
  watch, _, _ = make_quiet(fake, tmp_path, Beat(0.))
  watch.poll(0.)
  assert (tmp_path / "lease.json").exists() and fake.autoconnect == "no"
  watch.close()
  assert not (tmp_path / "lease.json").exists() and fake.autoconnect == "yes"


# ---- band: a saved profile pinned to "bg" or "a" must not be joined off a frequency it disallows (2026-11-06) ----

SAVED_BG = "iqpilot connection Galaxy S20 FE 5G6CC3:8a97:802-11-wireless:yes:1793727569\n"
DETAIL_BG = {"8a97": "Galaxy S20 FE 5G6CC3\ninfrastructure\n\nbg\n"}  # band pinned to 2.4 GHz
SAVED_OPEN = "openpilot connection LHE Casa:cccc:802-11-wireless:yes:0\n"
DETAIL_OPEN = {"cccc": "LHE Casa\ninfrastructure\nwlan0\n\n"}  # band unset: either is fine


def test_band_bg_visible_only_on_5ghz_is_wrong_band_logged_once(tmp_path):
  fake = Nmcli(visible=[("Galaxy S20 FE 5G6CC3", 5745)], saved=SAVED_BG, detail=DETAIL_BG)
  watch, logs = make(fake)
  assert watch.check(100.) == "scan"  # wrong band: no join, keep scanning in case the phone switches bands
  assert watch.check(115.) == "scan"
  assert not fake.did("up")
  wrong = [f for e, f in logs if e == "wlan0_hotspot_wrong_band"]
  assert len(wrong) == 1
  assert wrong[0]["band"] == "bg" and wrong[0]["freqs"] == [5745] and wrong[0]["connection"] == "iqpilot connection Galaxy S20 FE 5G6CC3"
  # back on 2.4 GHz: joins
  fake.visible = [("Galaxy S20 FE 5G6CC3", 2412)]
  assert watch.check(130.) == "con_up"
  assert fake.did("up")


def test_band_unset_profile_joins_on_5ghz():
  fake = Nmcli(visible=[("LHE Casa", 5745)], saved=SAVED_OPEN, detail=DETAIL_OPEN)
  watch, _ = make(fake)
  assert watch.check(100.) == "con_up"
  assert fake.did("up")[0][5] == "cccc"


def test_nothing_visible_at_all_still_falls_back_to_scan():
  fake = Nmcli(visible=[], saved=SAVED_BG, detail=DETAIL_BG)
  watch, _ = make(fake)
  assert watch.check(100.) == "scan"


# ---- quiet join: one join per QUIET_JOIN_EVERY s while Android Auto streams (2026-11-06 band fix + this) ----


def test_quiet_join_once_per_window_then_rechecks(tmp_path):
  fake = Nmcli(visible=[("Galaxy S20 FE 5G6CC3", 2412)])
  beat = Beat(-0.5)
  watch, logs, _ = make_quiet(fake, tmp_path, beat)
  # down + quiet: poll() pauses wlan0 autoconnect, and its own check() (CHECK_EVERY is due at t=0) does the first join
  watch.poll(0.)
  assert fake.autoconnect == "no"
  assert fake.did("up") == [["-w", str(wr.CON_UP_WAIT), "connection", "up", "uuid", "8a97", "ifname", "wlan0"]]
  assert [f for e, f in logs if e == "wlan0_rejoin"][-1]["quiet"] is True
  beat.at = 0.5
  assert watch.check(1.) == "streaming"  # within the window: no second join
  assert len(fake.did("up")) == 1
  beat.at = wr.QUIET_JOIN_EVERY - 0.5
  assert watch.check(wr.QUIET_JOIN_EVERY) == "quiet_join"  # window elapsed: joins again
  assert len(fake.did("up")) == 2
  assert not fake.did("rescan")


def test_quiet_session_marker_blocks_the_join(tmp_path):
  fake = Nmcli(visible=[("Galaxy S20 FE 5G6CC3", 2412)])
  beat = Beat(-0.5)
  watch, _, _ = make_quiet(fake, tmp_path, beat)
  watch.poll(0.)  # pauses, and its own check() does the first quiet join
  ups = len(fake.did("up"))
  (tmp_path / "session.json").write_text(json.dumps({"interface": "wlan0", "boot": boot_id()}))
  beat.at = 1.5
  assert watch.check(2.) == "paused"
  assert len(fake.did("up")) == ups  # no further join once a session holds wlan0


def test_quiet_without_our_pause_does_not_join(tmp_path):
  fake = Nmcli(visible=[("Galaxy S20 FE 5G6CC3", 2412)])
  watch, _, _ = make_quiet(fake, tmp_path, Beat(99.5))  # check() called directly: poll() never paused it
  assert watch.check(100.) == "streaming"
  assert not fake.did("up") and not fake.did("list") and not fake.did("rescan")


def test_projection_end_rearms_the_quiet_join(tmp_path):
  fake = Nmcli(visible=[("Galaxy S20 FE 5G6CC3", 2412)])
  beat = Beat(-0.5)
  watch, _, _ = make_quiet(fake, tmp_path, beat)
  watch.poll(0.)  # pauses, and its own check() does the first quiet join
  assert len(fake.did("up")) == 1
  beat.at = None  # projection ends
  watch.poll(2.)
  assert watch._quiet_join_at == -wr.QUIET_JOIN_EVERY
  beat.at = 2.5  # projecting again almost at once
  watch.poll(3.)  # re-pauses; CHECK_EVERY isn't due yet, so no check() inside this poll
  beat.at = 3.5
  assert watch.check(4.) == "quiet_join"  # re-armed: does not wait out the old window
  assert len(fake.did("up")) == 2


def test_supervisor_streaming_beat():
  import threading
  import time
  from openpilot.starpilot.system.starport import supervisor
  sup = supervisor.Supervisor.__new__(supervisor.Supervisor)
  sup._lock, sup._status, sup._stats_at = threading.RLock(), {"state": "joining_wifi"}, 0.0
  sup._set(stats={"fps": 30})
  assert sup.streaming_beat() is None
  before = time.monotonic()
  for state in ("streaming", "suspended"):
    sup._set(state=state)
    sup._set(stats={"fps": 30})
    assert sup.streaming_beat() >= before
  sup._set(state="idle")
  assert sup.streaming_beat() is None
