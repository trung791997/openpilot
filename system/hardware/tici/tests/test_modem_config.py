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


def _eg916_hardware(mocker, model):
  hardware = Tici()
  modem = mocker.MagicMock()
  modem.Get.side_effect = lambda iface, prop, **kw: model if prop == 'Model' else "Quectel"
  mocker.patch.object(hardware, "get_sim_info", return_value={"sim_id": ""})
  mocker.patch.object(hardware, "get_modem", return_value=modem)
  mocker.patch.object(hardware, "get_device_type", return_value="mici")
  mocker.patch("openpilot.system.hardware.tici.hardware.os.system")
  return hardware


def _nmcli_calls(call):
  return [c.args[0] for c in call.call_args_list if "nmcli" in c.args[0]]


def test_eg916_ppp_keepalive_restarts_session_without_echo(mocker, tmp_path):
  hardware = _eg916_hardware(mocker, "EG916Q-GL")
  call = mocker.patch("openpilot.system.hardware.tici.hardware.subprocess.call")
  mocker.patch("openpilot.system.hardware.tici.hardware.subprocess.check_output", return_value="4242\n")
  cmdline = tmp_path / "cmdline"
  cmdline.write_bytes(b"/usr/sbin/pppd\0nodetach\0ttyUSB3\0")
  real_open = open
  mocker.patch("builtins.open", lambda p, *a, **k: real_open(cmdline, *a, **k) if p == "/proc/4242/cmdline" else real_open(p, *a, **k))

  hardware.configure_modem()

  modify, up = _nmcli_calls(call)
  assert modify[modify.index("connection.autoconnect-retries") + 1] == "0"
  assert modify[modify.index("ppp.lcp-echo-interval") + 1] == "10"
  assert modify[modify.index("ppp.lcp-echo-failure") + 1] == "3"
  assert "--temporary" in modify
  assert up[-3:] == ["connection", "up", "lte"]


def test_eg916_ppp_keepalive_leaves_session_with_echo(mocker, tmp_path):
  hardware = _eg916_hardware(mocker, "EG916Q-GL")
  call = mocker.patch("openpilot.system.hardware.tici.hardware.subprocess.call")
  mocker.patch("openpilot.system.hardware.tici.hardware.subprocess.check_output", return_value="4242\n")
  cmdline = tmp_path / "cmdline"
  cmdline.write_bytes(b"/usr/sbin/pppd\0lcp-echo-failure\x003\0lcp-echo-interval\x0010\0")
  real_open = open
  mocker.patch("builtins.open", lambda p, *a, **k: real_open(cmdline, *a, **k) if p == "/proc/4242/cmdline" else real_open(p, *a, **k))

  hardware.configure_modem()

  calls = _nmcli_calls(call)
  assert len(calls) == 1 and "modify" in calls[0]


def test_eg916_ppp_keepalive_no_session_yet(mocker):
  import subprocess
  hardware = _eg916_hardware(mocker, "EG916Q-GL")
  call = mocker.patch("openpilot.system.hardware.tici.hardware.subprocess.call")
  mocker.patch("openpilot.system.hardware.tici.hardware.subprocess.check_output", side_effect=subprocess.CalledProcessError(1, "pgrep"))

  hardware.configure_modem()

  calls = _nmcli_calls(call)
  assert len(calls) == 1 and "modify" in calls[0]


def test_other_modems_get_no_ppp_keepalive(mocker):
  hardware = _eg916_hardware(mocker, "EG25-G")
  call = mocker.patch("openpilot.system.hardware.tici.hardware.subprocess.call")

  hardware.configure_modem()

  assert _nmcli_calls(call) == []
