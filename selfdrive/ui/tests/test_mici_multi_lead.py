from types import SimpleNamespace

import pytest

from openpilot.selfdrive.ui.lib.starpilot_visuals import multi_lead_ui_enabled
from openpilot.selfdrive.ui.mici.onroad import model_renderer as mr
from openpilot.selfdrive.ui.mici.onroad.hud_renderer import ICBM_SET_SPEED_SCALE, icbm_ceiling_active, set_speed_scale


class FakeParams:
  def __init__(self, values):
    self.values = values

  def get_bool(self, key, block=False, default=False):
    return bool(self.values.get(key, default))


@pytest.mark.parametrize("developer_ui", [False, True])
@pytest.mark.parametrize("widgets", [False, True])
@pytest.mark.parametrize("adjacent", [False, True])
def test_multi_lead_ui_needs_all_three_toggles(developer_ui, widgets, adjacent):
  params = FakeParams({"DeveloperUI": developer_ui, "DeveloperWidgets": widgets, "AdjacentLeadsUI": adjacent})
  assert multi_lead_ui_enabled(params) is (developer_ui and widgets and adjacent)


@pytest.mark.parametrize("redneck, has_cp, op_long, expected", [
  (True, True, False, True),    # ICBM on a stock-ACC car
  (True, True, True, False),    # openpilot longitudinal: ICBM does not run
  (True, False, False, False),  # no CarParams yet
  (False, True, False, False),  # ICBM off
])
def test_icbm_ceiling_active(redneck, has_cp, op_long, expected):
  assert icbm_ceiling_active(redneck, has_cp, op_long) is expected


def _lead(status=True, v_lead=20.0):
  return SimpleNamespace(status=status, vLead=v_lead, dRel=30.0, yRel=0.0, vRel=0.0)


def _vehicle(x):
  return mr.LeadVehicle(glow=[(x, 0.0)] * 3, chevron=[(x + 10, 110.0), (x, 100.0), (x - 10, 110.0)], fill_alpha=0)


@pytest.fixture
def overlay(monkeypatch):
  renderer = object.__new__(mr.ModelRenderer)
  fans, labels = [], []
  monkeypatch.setattr(mr.rl, "draw_triangle_fan", lambda pts, n, color: fans.append(color))
  monkeypatch.setattr(mr.ui_state, "starpilot_toggles", {}, raising=False)
  monkeypatch.setattr(mr.ui_state, "is_metric", False, raising=False)
  renderer._draw_lead_label = lambda chevron, text: labels.append((chevron[1][0], text))
  return renderer, fans, labels


def test_overlay_labels_every_drawn_lead_and_draws_adjacent_markers(overlay):
  renderer, fans, labels = overlay
  renderer._lead_vehicles = [_vehicle(500), _vehicle(520)]
  renderer._adjacent_lead_vehicles = [_vehicle(200), _vehicle(800)]
  radar_state = SimpleNamespace(leadOne=_lead(v_lead=20.0), leadTwo=_lead(status=False))
  sp_radar = SimpleNamespace(leadLeft=_lead(v_lead=25.0), leadRight=_lead(v_lead=30.0))

  renderer._draw_multi_lead_overlay(radar_state, sp_radar)

  assert labels == [(500, "45 mph"), (200, "56 mph"), (800, "67 mph")]
  # glow + fill for each adjacent lead, in their own colours with a visible minimum alpha
  assert len(fans) == 4
  assert (fans[1].r, fans[1].g, fans[1].b) == (0, 150, 255)
  assert (fans[3].r, fans[3].g, fans[3].b) == (180, 0, 255)
  assert fans[1].a == mr.ADJACENT_LEAD_MIN_ALPHA


def test_overlay_without_adjacent_data_labels_only_in_lane_leads(overlay):
  renderer, fans, labels = overlay
  renderer._lead_vehicles = [_vehicle(500), mr.LeadVehicle()]
  renderer._adjacent_lead_vehicles = [mr.LeadVehicle(), mr.LeadVehicle()]
  radar_state = SimpleNamespace(leadOne=_lead(v_lead=10.0), leadTwo=_lead(v_lead=12.0))

  renderer._draw_multi_lead_overlay(radar_state, None)

  assert labels == [(500, "22 mph")]
  assert fans == []


@pytest.mark.parametrize("icbm, changed, expected", [
  (True, False, ICBM_SET_SPEED_SCALE),  # persistent ICBM ceiling: compact box
  (True, True, 1.0),                     # set speed just changed: stock pop-up size
  (False, True, 1.0),
  (False, False, 1.0),
])
def test_set_speed_scale(icbm, changed, expected):
  assert set_speed_scale(icbm, changed) == expected
  assert 0.4 < ICBM_SET_SPEED_SCALE < 0.7
