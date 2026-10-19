from dataclasses import replace

from openpilot.starpilot.controls.lib.nav_passing import (
  EXIT_JAM_WARN_DIST, KEEP_RIGHT_COOLDOWN_S, KEEP_RIGHT_SECONDS, KEEP_RIGHT_SHOW_S, MERGE_WARN_DIST, MPH,
  Lead, NavPassingAdvisor, PassingInputs, edge_gap,
)

LIMIT = 65 * MPH
NAV = {"valid": True, "maneuverType": "off ramp", "maneuverModifier": "slight right", "maneuverDistance": 8000.0}


def inputs(**kw):
  base = PassingInputs(
    nav_state=dict(NAV), v_ego=55 * MPH, speed_limit=LIMIT, min_lane_change_speed=20 * MPH,
    lead=Lead(True, 30.0, 55 * MPH), lead_right=Lead(False),
    lane_width_left=3.6, lane_width_right=3.6, lane_detection_width=2.5,
    vasm_live=True, blocked_left=False, blocked_right=False, left_blinker=False, right_blinker=False,
  )
  return replace(base, **kw)


def run(adv, i, t0, seconds, dt=0.5):
  out, t = None, t0
  while t <= t0 + seconds:
    out = adv.update(i, t)
    t += dt
  return out, t


def test_slow_lead_with_a_free_left_lane_prompts_nothing():
  # the suggest-passing prompt is gone: being held up by a slow lead never arms anything on its own
  out, _ = run(NavPassingAdvisor(), inputs(keep_right=False), 0.0, 30.0)
  assert not out["armed"]


# D-115 keep-right reminder
def keep_inputs(**kw):
  base = dict(keep_right=True, lead=Lead(False), v_ego=LIMIT)
  base.update(kw)
  return inputs(**base)


def test_keep_right_after_a_minute_then_shows_and_cools_down():
  adv = NavPassingAdvisor()
  out, t = run(adv, keep_inputs(), 0.0, KEEP_RIGHT_SECONDS - 1.0)
  assert not out["armed"]
  out, t = run(adv, keep_inputs(), t, 1.5)
  assert out["armed"] and out["side"] == "right" and out["phase"] == "keepright"
  out, t = run(adv, keep_inputs(), t, KEEP_RIGHT_SHOW_S)
  assert not out["armed"]
  out, t = run(adv, keep_inputs(), t, KEEP_RIGHT_COOLDOWN_S - KEEP_RIGHT_SHOW_S - 5.0)
  assert not out["armed"]


def test_keep_right_ends_on_blinker():
  adv = NavPassingAdvisor()
  out, t = run(adv, keep_inputs(), 0.0, KEEP_RIGHT_SECONDS + 1.0)
  assert out["armed"]
  assert not adv.update(keep_inputs(right_blinker=True), t)["armed"]
  assert not adv.update(keep_inputs(), t + 0.5)["armed"]


def test_keep_right_blocked_cases():
  cases = [
    {"keep_right": False},
    {"vasm_live": False},
    {"lane_width_right": 1.0},                                   # already rightmost
    {"edge_gap_right": 0.3},                                     # right "lane" is the shoulder
    {"blocked_right": True},
    {"lead_right": Lead(True, 60.0, LIMIT - 10 * MPH)},          # right lane is slower
    {"lead": Lead(True, 30.0, 50 * MPH), "v_ego": 50 * MPH},     # stuck behind a slow lead
    {"left_blinker": True},
    {"speed_limit": 35 * MPH},
    {"nav_state": dict(NAV, maneuverModifier="slight left", maneuverDistance=1000.0)},
  ]
  for kw in cases:
    adv = NavPassingAdvisor()
    out, _ = run(adv, keep_inputs(**kw), 0.0, KEEP_RIGHT_SECONDS + 5.0)
    assert not out["armed"], kw


def test_keep_right_allows_a_faster_right_lane_car():
  adv = NavPassingAdvisor()
  out, _ = run(adv, keep_inputs(lead_right=Lead(True, 60.0, LIMIT + 2 * MPH)), 0.0, KEEP_RIGHT_SECONDS + 1.0)
  assert out["armed"]


def test_edge_gap_measures_road_edge_beyond_the_outer_line_at_20m():
  x = [0.0, 10.0, 20.0, 40.0]
  assert abs(edge_gap(x, [-5.4] * 4, x, [-6.0] * 4) - 0.6) < 1e-6    # left shoulder: edge just past the line
  assert abs(edge_gap(x, [5.4] * 4, x, [9.0] * 4) - 3.6) < 1e-6      # real lane: shoulder-width gap beyond it


# D-117 merge, jammed exit
MERGE_NAV = dict(NAV, maneuverType="merge", maneuverModifier="slight left", maneuverDistance=200.0)


def merge_inputs(**kw):
  base = dict(merge_warn=True, nav_state=dict(MERGE_NAV), lead=Lead(False), vasm_live=False)
  base.update(kw)
  return inputs(**base)


def test_merge_announced_with_side_and_vasm_state():
  out = NavPassingAdvisor().update(merge_inputs(), 0.0)
  assert out["armed"] and out["phase"] == "merge" and out["side"] == "left" and not out["blocked"]
  out = NavPassingAdvisor().update(merge_inputs(blocked_left=True), 0.0)
  assert out["armed"] and out["blocked"]


def test_merge_quiet_cases():
  cases = [
    {"merge_warn": False},
    {"nav_state": dict(MERGE_NAV, maneuverDistance=MERGE_WARN_DIST + 50.0)},
    {"nav_state": dict(MERGE_NAV, maneuverType="off ramp")},
    {"nav_state": dict(MERGE_NAV, maneuverModifier="straight")},
    {"left_blinker": True},                          # already merging
  ]
  for kw in cases:
    assert not NavPassingAdvisor().update(merge_inputs(**kw), 0.0)["armed"], kw


JAM_NAV = dict(NAV, exitJamDistance=2500.0, exitJamSide="right")


def jam_inputs(**kw):
  base = dict(exit_jam_warn=True, nav_state=dict(JAM_NAV), lead=Lead(False), vasm_live=False)
  base.update(kw)
  return inputs(**base)


def test_jammed_exit_suggests_moving_over_early():
  out = NavPassingAdvisor().update(jam_inputs(), 0.0)
  assert out["armed"] and out["phase"] == "exitjam" and out["side"] == "right" and out["distance"] == 2500.0
  cases = [
    {"exit_jam_warn": False},
    {"lane_width_right": 1.0},                       # already in the exit-side lane
    {"edge_gap_right": 0.3},                         # the only "lane" on the right is the shoulder
    {"right_blinker": True},
    {"nav_state": dict(JAM_NAV, exitJamDistance=EXIT_JAM_WARN_DIST + 100.0)},
    {"nav_state": dict(JAM_NAV, exitJamDistance=-1.0)},
  ]
  for kw in cases:
    assert not NavPassingAdvisor().update(jam_inputs(**kw), 0.0)["armed"], kw
