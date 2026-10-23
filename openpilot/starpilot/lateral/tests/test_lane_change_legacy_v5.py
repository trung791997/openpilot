import json

from openpilot.starpilot.lateral.lane_change_preferences import LiveLaneChangePolicy, decode

# The document saved on the car before the StarPilot 7.0 sync (2026-10-23).
LEGACY = {"autoDelayS": 1.0, "autoLaneChange": True, "closeGap": False, "closeGapSeconds": 0.75,
          "durationS": 5.777777777777778, "enabled": True, "minimumLaneWidthM": 0.0, "minimumSpeedMps": 2.2352,
          "onePerSignal": True, "turnDesireSpeedMps": 8.9408, "turnDesires": True, "version": 5}


def _raw(doc):
  return json.dumps(doc).encode()


def test_legacy_v5_document_migrates():
  policy = decode(_raw(LEGACY))
  assert policy is not None
  assert policy.enabled and policy.auto_lane_change and policy.turn_signal_guidance
  assert policy.auto_delay_s == 1.0 and policy.duration_s == LEGACY["durationS"]
  assert not decode(_raw({**LEGACY, "turnDesires": False})).turn_signal_guidance


def test_legacy_v5_bad_values_still_reject():
  for bad in ({**LEGACY, "turnDesires": 1}, {**LEGACY, "turnDesireSpeedMps": "x"}):
    assert decode(_raw(bad)) is None
  both = {**LEGACY, "turnSignalGuidance": True}
  assert decode(_raw(both)) is None


class _Params:
  def __init__(self, path):
    self.path = path

  def get_param_path(self, _key):
    return str(self.path)


def test_unrecognised_document_disables_lane_changes(tmp_path):
  path = tmp_path / "LaneChangePreferences"
  path.write_bytes(b'{"version": 9}')
  policy = LiveLaneChangePolicy(_Params(path), now_ns=1).applied
  assert not policy.enabled and not policy.turn_signal_guidance and not policy.auto_lane_change


def test_legacy_document_enables_auto_lane_change_live(tmp_path):
  path = tmp_path / "LaneChangePreferences"
  path.write_bytes(_raw(LEGACY))
  policy = LiveLaneChangePolicy(_Params(path), now_ns=1).applied
  assert policy.enabled and policy.auto_lane_change and policy.turn_signal_guidance
