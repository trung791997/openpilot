"""The car screen draws radar points exactly when the comma's Radar Point Display setting is on."""

from pathlib import Path
from types import SimpleNamespace
import importlib.util
import unittest

from openpilot.starpilot.system.starport.projection_layout import projection_customization, default_layout_for_viewport
from openpilot.starpilot.ui.onroad_customization import default_document as comma_default_document

ROOT = Path(__file__).parents[1]
UI_STATE = ROOT.parents[2] / 'selfdrive/ui/ui_state.py'
spec = importlib.util.spec_from_file_location('private_projection_onroad_radar', ROOT / 'projection_onroad.py')
assert spec is not None and spec.loader is not None
projection = importlib.util.module_from_spec(spec)
spec.loader.exec_module(projection)


class TestProjectionRadarPoints(unittest.TestCase):
  def paint(self, customization):
    seen = []
    view = projection.ProjectionOnroad.__new__(projection.ProjectionOnroad)
    view.camera = SimpleNamespace(stream_type=0, render_camera_model_layer=lambda _rect, **kw: seen.append(kw))
    view._theme = SimpleNamespace(road_style=lambda _colors: {})
    state = SimpleNamespace(customization=customization, appearance=SimpleNamespace(
      camera_view=0, show_lead_indicator=False, lead_info_mode=0, lead_info_metric=False), reverse_driver_camera=False)
    view._camera_layer(None, state)
    return seen[-1]['radar_points']

  def projected(self, comma_document):
    # What render() hands _camera_layer: the comma's document run through projection_customization.
    return projection_customization(default_layout_for_viewport((2880, 1080)), comma_document)

  def test_follows_the_comma_setting(self):
    base = comma_default_document()
    self.assertIs(self.paint(self.projected({**base, 'radarPoints': True})), True)
    self.assertIs(self.paint(self.projected({**base, 'radarPoints': False})), False)
    self.assertIs(self.paint(self.projected(base)), False, 'missing key means off')

  def test_non_bool_values_are_off(self):
    base = comma_default_document()
    for value in (1, 'true', None):
      self.assertIs(self.paint(self.projected({**base, 'radarPoints': value})), False)

  def test_radar_tracks_is_subscribed(self):
    # Starport draws from the shared ui_state SubMaster; the points need radarTracks in it.
    text = UI_STATE.read_text()
    services = text.split('self.sm = messaging.SubMaster(', 1)[1].split('\n    )', 1)[0]
    self.assertIn('"radarTracks"', services)


if __name__ == '__main__':
  unittest.main()
