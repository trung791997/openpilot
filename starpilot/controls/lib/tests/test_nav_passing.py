from dataclasses import replace

from openpilot.starpilot.controls.lib.nav_passing import EXIT_JAM_WARN_DIST, NavPassingAdvisor, PassingInputs, edge_gap

NAV = {"valid": True, "maneuverType": "off ramp", "maneuverModifier": "slight right", "maneuverDistance": 8000.0}
JAM_NAV = dict(NAV, exitJamDistance=2500.0, exitJamSide="right")


def jam_inputs(**kw):
  base = PassingInputs(
    nav_state=dict(JAM_NAV), lane_width_left=3.6, lane_width_right=3.6, lane_detection_width=2.5,
    left_blinker=False, right_blinker=False, exit_jam_warn=True,
  )
  return replace(base, **kw)


def test_edge_gap_measures_road_edge_beyond_the_outer_line_at_20m():
  x = [0.0, 10.0, 20.0, 40.0]
  assert abs(edge_gap(x, [-5.4] * 4, x, [-6.0] * 4) - 0.6) < 1e-6    # left shoulder: edge just past the line
  assert abs(edge_gap(x, [5.4] * 4, x, [9.0] * 4) - 3.6) < 1e-6      # real lane: shoulder-width gap beyond it


def test_only_the_jammed_exit_prompt_is_left():
  # keep-right, merge and passing prompts are gone: a merge step or open right lane arms nothing on its own
  merge_nav = dict(NAV, maneuverType="merge", maneuverModifier="slight left", maneuverDistance=200.0)
  for nav in (dict(NAV), merge_nav):
    out = NavPassingAdvisor().update(jam_inputs(nav_state=nav), 0.0)
    assert not out["armed"] and out["phase"] == ""


def test_jammed_exit_suggests_moving_over_early():
  out = NavPassingAdvisor().update(jam_inputs(), 0.0)
  assert out["armed"] and out["phase"] == "exitjam" and out["side"] == "right" and out["distance"] == 2500.0
  cases = [
    {"exit_jam_warn": False},
    {"lane_width_right": 1.0},                       # already in the exit-side lane
    {"edge_gap_right": 0.3},                         # the only "lane" on the right is the shoulder
    {"right_blinker": True},
    {"nav_state": dict(JAM_NAV, valid=False)},
    {"nav_state": dict(JAM_NAV, exitJamDistance=EXIT_JAM_WARN_DIST + 100.0)},
    {"nav_state": dict(JAM_NAV, exitJamDistance=-1.0)},
  ]
  for kw in cases:
    assert not NavPassingAdvisor().update(jam_inputs(**kw), 0.0)["armed"], kw
