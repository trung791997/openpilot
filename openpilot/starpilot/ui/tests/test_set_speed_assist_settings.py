from pathlib import Path
import tempfile

from openpilot.common.params import Params, ParamKeyType
from openpilot.starpilot.ui.feature_settings_owner import FeatureSettingsOwner
from openpilot.starpilot.ui.feature_settings_state import FeatureSettingsRequest


def test_set_speed_assist_is_default_off_persistent_and_editable_without_a_vehicle():
  with tempfile.TemporaryDirectory() as directory:
    params = Params(directory)
    assert params.get_type('SetSpeedAssist') == ParamKeyType.BOOL
    assert params.get_default_value('SetSpeedAssist') is False
    owner = FeatureSettingsOwner(params, lambda group: group == 'preferences', vehicle_fingerprint=lambda: None)
    state = owner.snapshot('profiles', parked=False, system_long=False, lateral_context=False, metric=False)
    row = next(row for row in state.rows if row.key == 'SetSpeedAssist')
    assert row.label == 'Set Speed Assist'
    assert row.value == row.default_value == 'Off'
    assert row.available and row.vehicle_fingerprint is None
    assert owner.apply(FeatureSettingsRequest(row.key, row.source, 'On'))
    assert Params(directory).get('SetSpeedAssist') is True
    assert Path(params.get_param_path(row.key)).read_bytes() == b'1'
    assert not owner.apply(FeatureSettingsRequest(row.key, row.source, 'Off'))  # a stale source is refused
    assert owner.apply(FeatureSettingsRequest(row.key, b'1', 'Off'))
    assert Path(params.get_param_path(row.key)).read_bytes() == b'0'
