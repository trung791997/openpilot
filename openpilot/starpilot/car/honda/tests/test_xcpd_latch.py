"""Per-drive latch of the manager's xcpd_enabled decision (AlphaHondaXcpDaemon)."""
from __future__ import annotations

from types import SimpleNamespace

from opendbc.car import gen_empty_fingerprint
from opendbc.car.honda.interface import CarInterface as HondaInterface
from opendbc.car.honda.values import CAR as HondaCAR, HondaFlags
from openpilot.system.manager import process_config

KEY = "AlphaHondaXcpDaemon"


def _civic_cp():
  return HondaInterface.get_params(HondaCAR.HONDA_CIVIC_BOSCH, gen_empty_fingerprint(), [], True, False, False)


class _Params:
  def __init__(self, on):
    self.on = on

  def get_bool(self, key):
    assert key == KEY
    return self.on


def test_xcpd_enabled_latches_per_drive():
  cp = _civic_cp()
  params = _Params(False)
  process_config._xcpd_latch = None
  try:
    assert process_config.xcpd_enabled(True, params, cp) is False
    params.on = True  # flipped mid-drive: unchanged
    assert process_config.xcpd_enabled(True, params, cp) is False
    assert process_config.xcpd_enabled(False, params, cp) is False
    assert process_config.xcpd_enabled(True, params, cp) is True  # next drive picks it up
    params.on = False
    assert process_config.xcpd_enabled(True, params, cp) is True  # Off mid-drive keeps xcpd
    assert process_config.xcpd_enabled(False, params, cp) is False
    assert process_config.xcpd_enabled(True, params, cp) is False
    # An unsupported CP stays False even with the param On.
    params.on = True
    process_config.xcpd_enabled(False, params, cp)
    radarless = SimpleNamespace(brand="honda", flags=int(HondaFlags.BOSCH | HondaFlags.BOSCH_RADARLESS), passive=False)
    assert process_config.xcpd_enabled(True, params, radarless) is False
  finally:
    process_config._xcpd_latch = None


def test_xcpd_starts_once_carparams_arrive():
  # First onroad tick: manager's carParams is still the empty default; xcpd must start once the real CP shows up.
  params = _Params(True)
  process_config._xcpd_latch = None
  try:
    empty = SimpleNamespace(brand="", flags=0, passive=False)
    assert process_config.xcpd_enabled(True, params, empty) is False
    params.on = False  # mid-drive flip after the latch: ignored
    assert process_config.xcpd_enabled(True, params, _civic_cp()) is True
  finally:
    process_config._xcpd_latch = None
