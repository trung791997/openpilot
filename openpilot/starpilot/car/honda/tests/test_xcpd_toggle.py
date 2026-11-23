"""Tests for the Galaxy AlphaHondaXcpDaemon toggle and the per-drive latch of xcpd_enabled."""
from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import Mock

from opendbc.car import gen_empty_fingerprint
from opendbc.car.honda.interface import CarInterface as HondaInterface
from opendbc.car.honda.values import CAR as HondaCAR, HondaFlags
from openpilot.common.params import Params
from openpilot.starpilot.car.honda.xcpd_toggle import KEY, LABEL, capability
from openpilot.starpilot.galaxy.settings import AuthorityContext, SettingsGateway
from openpilot.starpilot.ui.feature_settings_owner import FeatureSettingsOwner
from openpilot.starpilot.ui.feature_settings_state import FeaturePage
from openpilot.starpilot.ui.honda_xcpd_feature import HondaXcpDaemonFeature


def _civic_cp():
  return HondaInterface.get_params(HondaCAR.HONDA_CIVIC_BOSCH, gen_empty_fingerprint(), [], True, False, False)


def test_capability_supported_and_unsupported(tmp_path):
  params = Params(str(tmp_path))
  cp = _civic_cp()
  assert capability(cp, params) is not None
  assert capability(None, params) is None
  other = SimpleNamespace(brand="toyota", carFingerprint="TOYOTA_RAV4_TSS2_2023", radarUnavailable=False,
                          notCar=False, passive=False, dashcamOnly=False)
  assert capability(other, params) is None
  radarless = SimpleNamespace(brand=cp.brand, carFingerprint=cp.carFingerprint, radarUnavailable=False, notCar=False,
                              passive=False, dashcamOnly=False, flags=int(cp.flags) | int(HondaFlags.BOSCH_RADARLESS),
                              alternativeExperience=0, openpilotLongitudinalControl=True, pcmCruise=False)
  assert capability(radarless, params) is None
  canfd = SimpleNamespace(**{**vars(radarless), "flags": int(cp.flags) | int(HondaFlags.BOSCH_CANFD)})
  assert capability(canfd, params) is None


def _owner(params, cp, galaxy=True):
  return FeatureSettingsOwner(params, lambda _: True, vehicle_fingerprint=lambda: cp.carFingerprint,
                              vehicle_params=lambda: cp, galaxy=lambda: galaxy)


def test_rows_and_apply(tmp_path):
  params = Params(str(tmp_path))
  cp = _civic_cp()
  owner = _owner(params, cp)
  feature = HondaXcpDaemonFeature(owner, galaxy=lambda: True)
  row, = feature.rows()
  assert row.key == KEY and row.label == LABEL and row.value == "Off" and row.default_value == "Off"
  assert row.choices == ("Off", "On") and row.available

  def request(value, **kw):
    base = dict(key=KEY, value=value, expected=None, dependencies=(), related_source=None, display_unit=None,
                direction=0, capability=row.capability, vehicle_fingerprint=row.vehicle_fingerprint)
    base.update(kw)
    return SimpleNamespace(**base)

  assert feature.apply(request("On"))
  assert params.get_bool(KEY) is True
  assert feature.apply(request("Off", expected=b"1"))
  assert params.get_bool(KEY) is False
  # Wrong capability / fingerprint / key / value are rejected and leave the file alone.
  assert not feature.apply(request("On", expected=b"0", capability=("honda", "x")))
  assert not feature.apply(request("On", expected=b"0", vehicle_fingerprint="OTHER"))
  assert not feature.apply(request("On", expected=b"0", key="AlphaHondaXcpRadarAssist"))
  assert not feature.apply(request("Maybe", expected=b"0"))
  assert params.get_bool(KEY) is False

  # Hidden when not galaxy, and on an unsupported car.
  assert HondaXcpDaemonFeature(owner, galaxy=lambda: False).rows() == ()
  toyota = SimpleNamespace(brand="toyota", carFingerprint="TOYOTA_RAV4_TSS2_2023", radarUnavailable=False,
                           notCar=False, passive=False, dashcamOnly=False)
  assert HondaXcpDaemonFeature(_owner(params, toyota), galaxy=lambda: True).rows() == ()


def test_galaxy_integration(tmp_path):
  params = Params(str(tmp_path))
  cp = _civic_cp()
  ctx_src = Mock(sample=lambda: AuthorityContext(True, cp, cp.to_bytes(), editing_saved_vehicle=False))
  gateway = SettingsGateway(params, ctx_src, clock=lambda: 10)
  try:
    for page_name in ("vehicle", "profiles"):
      page = gateway.page(page_name, "sess", b"gen")
      row = next(r for r in page["rows"] if r["label"] == LABEL)
      assert row["value"] == "Off" and row["choices"] == ["Off", "On"] and row["available"] is True
    page = gateway.page("vehicle", "sess", b"gen")
    idx = next(i for i, r in enumerate(page["rows"]) if r["label"] == LABEL)
    intent = gateway.preview(page["view"], idx, 1, "sess", b"gen")
    assert intent["proposed"] == "On"
    assert "Applies on the next drive" in intent["question"]
    assert gateway.confirm(intent["intent"], "sess", b"gen")
    assert params.get_bool(KEY) is True
  finally:
    gateway.close()
  non_galaxy = _owner(params, cp, galaxy=False)
  assert all(r.key != KEY for r in non_galaxy.snapshot(FeaturePage.VEHICLE, parked=True, system_long=True,
                                                       lateral_context=True, metric=False).rows)
