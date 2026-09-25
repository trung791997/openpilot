import subprocess

import pytest

from openpilot.system.hardware.base import LPABase
from openpilot.system.hardware.tici.hardware import Tici


def test_comma_profile_detection_without_lpa(mocker):
  hardware = Tici()
  modem = mocker.MagicMock()
  modem.Get.return_value = "Quectel"

  mocker.patch.object(hardware, "get_sim_info", return_value={"sim_id": "8985235000000000000"})
  mocker.patch.object(hardware, "get_modem", return_value=modem)
  mocker.patch.object(hardware, "get_device_type", return_value="mici")
  get_sim_lpa = mocker.patch.object(hardware, "get_sim_lpa")
  mocker.patch("openpilot.system.hardware.tici.hardware.os.path.exists", return_value=True)

  hardware.configure_modem()

  get_sim_lpa.assert_not_called()
  assert LPABase.is_comma_profile("8985235000000000000")
  assert not LPABase.is_comma_profile("8900000000000000000")


# on-disk profile at boot on a comma 4 (2026-09-25): blank APN, NM defaults
BOOT_PROFILE = "\nyes\nno\nunknown\n-1\n0\n0\n"
APPLIED_PROFILE = "fast.t-mobile.com\nno\nno\nunknown\n0\n10\n3\n"
PPP_OFF = b"/usr/sbin/pppd\0nodetach\0ttyUSB3\0lcp-echo-failure\x000\0lcp-echo-interval\x000\0"
PPP_ON = b"/usr/sbin/pppd\0nodetach\0ttyUSB3\0lcp-echo-failure\x003\0lcp-echo-interval\x0010\0"


def _eg916_hardware(mocker, tmp_path, model="EG916Q-GL", profile=BOOT_PROFILE, ppp_args=None, apn="fast.t-mobile.com"):
  hardware = Tici()
  modem = mocker.MagicMock()
  modem.Get.side_effect = lambda iface, prop, **kw: model if prop == 'Model' else "Quectel"
  mocker.patch.object(hardware, "get_sim_info", return_value={"sim_id": ""})
  mocker.patch.object(hardware, "get_modem", return_value=modem)
  mocker.patch.object(hardware, "get_device_type", return_value="mici")
  mocker.patch("openpilot.system.hardware.tici.hardware.os.system")

  params = mocker.MagicMock()
  params.get.side_effect = lambda k: apn if k == "GsmApn" else None
  params.get_bool.side_effect = lambda k: {"GsmRoaming": True, "GsmMetered": True}.get(k, False)
  mocker.patch("openpilot.system.hardware.tici.hardware.Params", return_value=params)

  def check_output(cmd, **kw):
    if cmd[0] == "pgrep":
      if ppp_args is None:
        raise subprocess.CalledProcessError(1, cmd)
      return "4242\n"
    return profile
  mocker.patch("openpilot.system.hardware.tici.hardware.subprocess.check_output", side_effect=check_output)

  if ppp_args is not None:
    cmdline = tmp_path / "cmdline"
    cmdline.write_bytes(ppp_args)
    real_open = open
    mocker.patch("builtins.open", lambda p, *a, **k: real_open(cmdline, *a, **k) if p == "/proc/4242/cmdline" else real_open(p, *a, **k))
  return hardware


def _nmcli_calls(call):
  return [c.args[0] for c in call.call_args_list if "nmcli" in c.args[0]]


def _value(cmd, key):
  return cmd[cmd.index(key) + 1]


# NM passes the echo options even when they are off (seen on a comma 4, 2026-09-25)
@pytest.mark.parametrize("ppp_args", [PPP_OFF, b"/usr/sbin/pppd\0nodetach\0ttyUSB3\0"])
def test_eg916_restarts_session_without_echo(mocker, tmp_path, ppp_args):
  hardware = _eg916_hardware(mocker, tmp_path, profile=APPLIED_PROFILE, ppp_args=ppp_args)
  call = mocker.patch("openpilot.system.hardware.tici.hardware.subprocess.call")

  hardware.configure_modem()

  modify, up = _nmcli_calls(call)
  assert "--temporary" in modify
  assert _value(modify, "connection.autoconnect-retries") == "0"
  assert _value(modify, "ppp.lcp-echo-interval") == "10"
  assert _value(modify, "ppp.lcp-echo-failure") == "3"
  assert up[-3:] == ["connection", "up", "lte"]


def test_eg916_applies_saved_apn_at_boot(mocker, tmp_path):
  # echo already right, but the session came up on the blank on-disk APN
  hardware = _eg916_hardware(mocker, tmp_path, profile=BOOT_PROFILE, ppp_args=PPP_ON)
  call = mocker.patch("openpilot.system.hardware.tici.hardware.subprocess.call")

  hardware.configure_modem()

  modify, up = _nmcli_calls(call)
  assert _value(modify, "gsm.apn") == "fast.t-mobile.com"
  assert _value(modify, "gsm.auto-config") == "no"
  assert _value(modify, "gsm.home-only") == "no"  # GsmRoaming on
  assert _value(modify, "connection.metered") == "unknown"  # GsmMetered on
  assert up[-3:] == ["connection", "up", "lte"]


def test_eg916_blank_apn_uses_auto_config(mocker, tmp_path):
  hardware = _eg916_hardware(mocker, tmp_path, apn=None)
  call = mocker.patch("openpilot.system.hardware.tici.hardware.subprocess.call")

  hardware.configure_modem()

  modify, = _nmcli_calls(call)
  assert _value(modify, "gsm.apn") == ""
  assert _value(modify, "gsm.auto-config") == "yes"


def test_eg916_leaves_matching_session(mocker, tmp_path):
  hardware = _eg916_hardware(mocker, tmp_path, profile=APPLIED_PROFILE, ppp_args=PPP_ON)
  call = mocker.patch("openpilot.system.hardware.tici.hardware.subprocess.call")

  hardware.configure_modem()

  calls = _nmcli_calls(call)
  assert len(calls) == 1 and "modify" in calls[0]


def test_eg916_no_session_yet(mocker, tmp_path):
  hardware = _eg916_hardware(mocker, tmp_path, ppp_args=None)
  call = mocker.patch("openpilot.system.hardware.tici.hardware.subprocess.call")

  hardware.configure_modem()

  calls = _nmcli_calls(call)
  assert len(calls) == 1 and "modify" in calls[0]


def test_other_modems_untouched(mocker, tmp_path):
  hardware = _eg916_hardware(mocker, tmp_path, model="EG25-G")
  call = mocker.patch("openpilot.system.hardware.tici.hardware.subprocess.call")

  hardware.configure_modem()

  assert _nmcli_calls(call) == []
