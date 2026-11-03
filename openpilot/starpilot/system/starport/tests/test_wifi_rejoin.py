"""wlan0 rejoins the phone hotspot by itself once it is in range, without hammering nmcli or touching the car link."""

from openpilot.starpilot.system.starport import wifi_rejoin as wr

SAVED = "".join(line + "\n" for line in ("iqpilot connection Galaxy S20 FE 5G6CC3:8a97:802-11-wireless:yes:1793727569",
                                         "starpilot-starport:aaaa:802-11-wireless:no:1793727579",
                                         "Hotspot:hhhh:802-11-wireless:no:0",
                                         "lte:llll:gsm:yes:0",
                                         "openpilot connection LHE Casa:cccc:802-11-wireless:yes:0"))
DETAIL = {"8a97": "Galaxy S20 FE 5G6CC3\ninfrastructure\n\n", "cccc": "LHE Casa\ninfrastructure\nwlan0\n"}


class Nmcli:
  def __init__(self, wlan0="disconnected", aa0="connected", visible=("Galaxy S20 FE 5G6CC3",), autoconnect="yes"):
    self.wlan0, self.aa0, self.visible, self.autoconnect = wlan0, aa0, list(visible), autoconnect
    self.calls: list[list] = []
    self.fail: set[str] = set()

  def __call__(self, args, timeout):
    self.calls.append(args)
    key = " ".join(args)
    for word in self.fail:
      if word in key:
        raise RuntimeError(f"{word} failed")
    if args[-1:] == ["device"]:
      return f"wlan0:{self.wlan0}:\naa0:{self.aa0}:starpilot-starport\nlo:connected (externally):lo\n"
    if "GENERAL.AUTOCONNECT" in key:
      return self.autoconnect + "\n"
    if "GENERAL.STATE,GENERAL.REASON" in key:
      return "30 (disconnected)\n8 (The supplicant is now unavailable)\n"
    if args[-2:] == ["connection", "show"]:
      return SAVED
    if args[:2] == ["-t", "-g"] and "uuid" in args:
      return DETAIL[args[-1]]
    if "list" in args:
      return "".join(ssid.replace(":", "\\:") + "\n" for ssid in self.visible)
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
