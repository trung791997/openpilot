from openpilot.system.ui.lib.wifi_manager import tethering_band


def test_mici_hotspot_5ghz():
  assert tethering_band("mici") == {'band': ('s', 'a'), 'channel': ('u', 36)}


def test_other_devices_hotspot_2ghz():
  for device in ("tici", "tizi", "pc"):
    assert tethering_band(device) == {'band': ('s', 'bg')}
