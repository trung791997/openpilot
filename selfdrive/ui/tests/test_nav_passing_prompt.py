from openpilot.selfdrive.ui.onroad.starpilot.nav_passing_prompt import MERGE_BLOCKED, TEXT, format_jam, parse_passing_prompt


def test_parse_passing_prompt_needs_armed_fresh_side_and_phase():
  raw = {"armed": True, "side": "right", "phase": "keepright", "ts": 100.0}
  assert parse_passing_prompt(raw, 101.0) == {"side": "right", "phase": "keepright", "ts": 100.0, "distance": 0.0, "speed": 0.0,
                                              "blocked": False}
  assert parse_passing_prompt('{"armed": true, "side": "left", "phase": "merge", "ts": 100.0}', 100.5)["phase"] == "merge"
  assert parse_passing_prompt(raw, 103.0) is None, "stale: the planner stopped refreshing it"
  assert parse_passing_prompt({**raw, "armed": False}, 101.0) is None
  assert parse_passing_prompt({**raw, "phase": "bogus"}, 101.0) is None
  assert parse_passing_prompt({**raw, "side": ""}, 101.0) is None
  assert parse_passing_prompt("not json", 101.0) is None
  assert parse_passing_prompt({}, 101.0) is None


def test_d115_keep_right_phase_parses():
  assert parse_passing_prompt({"armed": True, "side": "right", "phase": "keepright", "ts": 1.0}, 1.5)["phase"] == "keepright"


def test_removed_pass_exit_turn_and_arrive_banners_do_not_show():
  for phase in ("pass", "return", "exit", "turn", "arrive"):
    assert phase not in TEXT
    assert parse_passing_prompt({"armed": True, "side": "right", "phase": phase, "ts": 1.0, "distance": 900.0}, 1.5) is None


def test_d116_jam_notice_has_no_side_and_needs_distance_and_speed():
  raw = {"armed": True, "side": "", "phase": "jam", "ts": 1.0, "distance": 1200.0, "speed": 20.1}
  assert parse_passing_prompt(raw, 1.5)["distance"] == 1200.0
  assert parse_passing_prompt({**raw, "side": "left"}, 1.5) is None
  assert parse_passing_prompt({**raw, "distance": 0.0}, 1.5) is None
  assert parse_passing_prompt({**raw, "speed": "nan"}, 1.5) is None
  assert format_jam(1200.0, 20.1168, False) == ("0.7 mi", "45 mph")
  assert format_jam(120.0, 20.0, False) == ("394 ft", "45 mph")
  assert format_jam(1200.0, 12.5, True) == ("1.2 km", "45 km/h")


def test_d117_merge_and_exitjam_phases_parse():
  merge = {"armed": True, "side": "left", "phase": "merge", "ts": 1.0}
  assert parse_passing_prompt(merge, 1.5)["blocked"] is False
  assert parse_passing_prompt({**merge, "blocked": True}, 1.5)["blocked"] is True
  assert MERGE_BLOCKED.format(side="left") == "Car beside you on the left: wait"
  for phase in ("exitjam",):
    raw = {"armed": True, "side": "right", "phase": phase, "ts": 1.0, "distance": 900.0}
    assert parse_passing_prompt(raw, 1.5)["distance"] == 900.0
    assert parse_passing_prompt({**raw, "distance": 0.0}, 1.5) is None, "needs a distance to show"
    assert parse_passing_prompt({**raw, "distance": "inf"}, 1.5) is None
    assert parse_passing_prompt({**raw, "side": ""}, 1.5) is None
