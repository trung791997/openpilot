import json
from collections import Counter
from types import SimpleNamespace

import pytest

from openpilot.selfdrive.ui.onroad.starpilot import nav_lane_prompt


class FakeParams:
  def __init__(self, values):
    self.values = values
    self.reads = Counter()

  def get(self, key):
    self.reads[key] += 1
    if isinstance(self.values.get(key), Exception):
      raise self.values[key]
    return self.values.get(key)


def prompt(**kw):
  d = {"armed": True, "side": "right", "kind": "exit", "distance_m": 900.0, "window_m": 1200.0, "ts": 100.0, "turn_lane": False}
  d.update(kw)
  return d


@pytest.fixture
def widget(monkeypatch):
  ui = SimpleNamespace(params_memory=FakeParams({"NavLaneMovePrompt": prompt()}), is_metric=False)
  clock = [100.0]
  monkeypatch.setattr(nav_lane_prompt, "ui_state", ui)
  monkeypatch.setattr(nav_lane_prompt.time, "monotonic", lambda: clock[0])
  monkeypatch.setattr(nav_lane_prompt.gui_app, "font", lambda _: None)
  return nav_lane_prompt.NavLaneMovePromptRenderer(), ui, clock


def test_armed_fresh_prompt_shows(widget):
  w, _, _ = widget
  w._update_state()
  assert w._prompt["side"] == "right" and w._prompt["kind"] == "exit"


def test_json_string_is_accepted(widget):
  w, ui, _ = widget
  ui.params_memory.values["NavLaneMovePrompt"] = json.dumps(prompt(side="left"))
  w._update_state()
  assert w._prompt["side"] == "left"


@pytest.mark.parametrize("raw", [
  prompt(armed=False), prompt(ts=97.0), prompt(ts=101.0), prompt(side="center"), prompt(side=None),
  {}, None, "", "{not json", "[]", prompt(ts="x"), prompt(distance_m="far"), prompt(ts=None),
])
def test_hidden_cases(widget, raw):
  w, ui, _ = widget
  ui.params_memory.values["NavLaneMovePrompt"] = raw
  w._update_state()
  assert w._prompt is None


def test_missing_key_hides(widget):
  w, ui, _ = widget
  ui.params_memory.values["NavLaneMovePrompt"] = KeyError("NavLaneMovePrompt")
  w._update_state()
  assert w._prompt is None


def test_reads_are_bounded_at_sixty_fps(widget):
  w, ui, clock = widget
  for frame in range(60):
    clock[0] = 100.0 + frame / 60
    w._update_state()
  assert 8 <= ui.params_memory.reads["NavLaneMovePrompt"] <= 10


def test_stale_prompt_stops_drawing_between_reads(widget, monkeypatch):
  w, _, clock = widget
  w._update_state()
  drawn = []
  monkeypatch.setattr(nav_lane_prompt.rl, "draw_rectangle_rounded", lambda *a: drawn.append(a))
  clock[0] = 102.5
  w._last_update = clock[0]
  w._render(nav_lane_prompt.rl.Rectangle(0, 0, 536, 240))
  assert drawn == []


def test_blocked_flag_is_carried(widget):
  w, ui, _ = widget
  ui.params_memory.values["NavLaneMovePrompt"] = prompt(blocked=True)
  w._update_state()
  assert w._prompt["blocked"] is True
  ui.params_memory.values["NavLaneMovePrompt"] = prompt()
  w._last_update = -1e9
  w._update_state()
  assert w._prompt["blocked"] is False
