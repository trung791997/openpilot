"""The UI's Wi-Fi manager asks for no scan while StarPort holds its scan_quiet lease (Android Auto on the car)."""
from unittest import mock

import openpilot.system.ui.lib.wifi_manager as wifi_manager
from openpilot.starpilot.system.starport import scan_quiet


def _manager():
  wm = wifi_manager.WifiManager.__new__(wifi_manager.WifiManager)
  wm._exit = True  # keep stop() in __del__ from doing anything
  wm._wifi_device = "/org/freedesktop/NetworkManager/Devices/3"
  wm._router_main = mock.MagicMock()
  return wm


def test_uses_the_starport_lease():
  assert wifi_manager._starport_scan_quiet is scan_quiet.quiet


def test_no_scan_while_projecting(monkeypatch):
  monkeypatch.setattr(wifi_manager, "_starport_scan_quiet", lambda: True)
  wm = _manager()
  wm._request_scan()
  wm._router_main.send_and_get_reply.assert_not_called()


def test_scans_when_the_lease_is_gone(monkeypatch):
  monkeypatch.setattr(wifi_manager, "_starport_scan_quiet", lambda: False)
  wm = _manager()
  wm._request_scan()
  assert wm._router_main.send_and_get_reply.call_count == 1
