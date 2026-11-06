"""A Vision limit far above Mapbox is a misread, not a sign (map_observation.MapboxCeiling).

Route 0000009b--2188fdfd3f: Mapbox 55, camera read 80 off a highway 78 shield and the
car took the 80; later Mapbox 50 then 30, camera read 70 at 19 mph and that was taken too.
"""
import pytest

from openpilot.starpilot.speed_limits import acceptance as acc, selection as sel
from openpilot.starpilot.speed_limits.map_observation import MAPBOX_RECENT_NS, MapboxCeiling
from openpilot.starpilot.speed_limits.tests.test_turn_override import UNKNOWN, Drive, online, vision

S = 1_000_000_000


def test_far_above_mapbox_is_dropped():
  rule = MapboxCeiling()
  assert rule.step(vision(80), online(55), 0)
  assert rule.step(vision(70, sign='s2'), online(30), S)
  assert rule.step(vision(70, sign='s3'), online(50), 2 * S)  # 20 over


def test_close_to_mapbox_passes():
  rule = MapboxCeiling()
  assert not rule.step(vision(55), online(55), 0)
  assert not rule.step(vision(70, sign='s2'), online(55), S)  # 15 over: a real step up Mapbox is a little behind on
  assert not rule.step(vision(25, sign='s3'), online(55), 2 * S)  # lower is never this rule's business
  assert not rule.step(UNKNOWN, online(55), 3 * S)


def test_recent_mapbox_covers_a_dropout_then_expires():
  rule = MapboxCeiling()
  rule.step(vision(50), online(50), 0)
  assert rule.step(vision(70, sign='s2'), UNKNOWN, MAPBOX_RECENT_NS)  # 11:40: Mapbox blank for 2 s around the 70
  assert rule.step(vision(70, sign='s2'), UNKNOWN, MAPBOX_RECENT_NS + 1)  # still the same misread
  assert not rule.step(vision(70, sign='s3'), UNKNOWN, MAPBOX_RECENT_NS + 2)  # a new read, no Mapbox to check
  assert not MapboxCeiling().step(vision(80), UNKNOWN, 0)  # no Mapbox this drive: Vision unchecked


def test_a_passed_sign_is_not_re_judged_and_a_misread_clears_when_mapbox_agrees():
  rule = MapboxCeiling()
  assert not rule.step(vision(50), online(50), 0)
  assert not rule.step(vision(50), online(25), S)  # turned onto a slower road: TurnOverride's case
  assert rule.step(vision(75, sign='s2'), online(55), 2 * S)
  assert not rule.step(vision(75, sign='s2'), online(70), 3 * S)  # Mapbox caught up: a real raise
  assert not rule.step(vision(75, sign='s2'), online(55), 4 * S)


def test_runtime_shield_read_never_selected_and_the_sign_before_it_holds(monkeypatch):
  monkeypatch.setattr('openpilot.starpilot.gps.source.gps_mode', lambda now=None: 'backup')
  car = Drive()
  values, chosen = car.tick(55, 55, 90.)
  assert chosen.selected_source is sel.Source.VISION
  values, chosen = car.tick(80, 55, 90., episode=2)  # the 78 shield
  assert values[sel.Source.VISION].kind is acc.ObservationKind.UNKNOWN
  assert values[sel.Source.ONLINE].kind is acc.ObservationKind.UNKNOWN  # SignFirst: same road as the 55 sign
  assert chosen.selected_source is None  # acceptance keeps the 55 already taken
  assert car.runtime.vision_held.candidate.speed_mps == pytest.approx(vision(80).candidate.speed_mps)  # drawer shows the read
  values, chosen = car.tick(55, 55, 90., episode=3)
  assert chosen.selected_source is sel.Source.VISION and car.runtime.vision_held.kind is acc.ObservationKind.UNKNOWN


def test_runtime_shield_read_during_mapbox_gap(monkeypatch):
  monkeypatch.setattr('openpilot.starpilot.gps.source.gps_mode', lambda now=None: 'backup')
  car = Drive()
  car.tick(50, 50, 90.)
  values, chosen = car.tick(70, 0, 90., episode=2)
  assert chosen.selected_source is not sel.Source.VISION
  values, chosen = car.tick(70, 30, 90., episode=2)
  assert chosen.selected_source is sel.Source.ONLINE  # a new Mapbox value after the sign: 30
  car.runtime.reset()
  assert car.runtime.mapbox_ceiling.mapbox_mps is None
