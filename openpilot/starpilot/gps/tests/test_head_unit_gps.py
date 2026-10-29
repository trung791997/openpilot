import math
from unittest.mock import patch

import pytest

from openpilot.cereal import messaging
from openpilot.starpilot.gps.head_unit import NOMINAL_ACCURACY_M, HeadUnitGps, head_unit_gps, read_head_unit_gps
from openpilot.starpilot.gps.source import HEAD_UNIT_MAX_AGE_NS, select_location, select_source
from openpilot.starpilot.gps.tests.test_phone_gps import FIX as PHONE_FIX, Bus, Messages
from openpilot.starpilot.gps.phone import PhoneGpsPublisher, read_phone_gps
from openpilot.starpilot.navigation.runtime import publish_status
from openpilot.starpilot.navigation.status import NavigationStatusSource
from openpilot.starpilot.system.android_auto.wire import field, parse_fields

NOW = 101_000_000_000


def location(changes=None):
  """AA LocationData as the head unit sends it: 1 time, 2 lat e7, 3 lon e7, 4 accuracy e3, 5 altitude e2, 6 speed e3, 7 bearing e6."""
  values = {1: 77, 2: 401_234_567, 3: -1_101_234_567, 4: 4_000, 5: 12_345, 6: 20_000, 7: 90_000_000, **(changes or {})}
  return parse_fields(b"".join(field(number, value) for number, value in values.items() if value is not None))


def handed_off(tmp_path, now=NOW, changes=None):
  path = tmp_path / "gps.json"
  assert HeadUnitGps(path=path, mono_clock=lambda: now).publish(location(changes))
  return path


def test_location_data_is_scaled_and_signed():
  gps, timestamp = head_unit_gps(location(), NOW)
  assert timestamp == 77 and gps["sourceMonoTime"] == NOW and gps["hasFix"]
  assert (gps["latitude"], gps["longitude"]) == (40.1234567, -110.1234567)
  assert (gps["horizontalAccuracy"], gps["altitude"], gps["speed"], gps["bearingDeg"]) == (4., 123.45, 20., 90.)
  assert math.isfinite(gps["bearingAccuracyDeg"]) and math.isnan(gps["verticalAccuracy"]) and math.isnan(gps["speedAccuracy"])


def test_missing_accuracy_claims_a_usable_nominal_one():
  assert head_unit_gps(location({4: None}), NOW)[0]["horizontalAccuracy"] == NOMINAL_ACCURACY_M
  assert head_unit_gps(location({4: 0}), NOW)[0]["horizontalAccuracy"] == NOMINAL_ACCURACY_M


def test_heading_is_trusted_only_while_moving():
  gps = head_unit_gps(location({6: 500}), NOW)[0]
  assert gps["speed"] == .5 and math.isnan(gps["bearingDeg"]) and math.isnan(gps["bearingAccuracyDeg"])
  gps = head_unit_gps(location({6: None}), NOW)[0]
  assert math.isnan(gps["speed"]) and math.isnan(gps["bearingDeg"])


@pytest.mark.parametrize("changes", [{2: None}, {3: None}, {2: 0, 3: 0}, {2: 910_000_000}, {3: 1_810_000_000}, {4: 20_000_000}])
def test_unusable_positions_are_refused(changes):
  with pytest.raises(ValueError):
    head_unit_gps(location(changes), NOW)


def test_publisher_counts_repeats_and_rejections_without_positions(tmp_path):
  clock = [NOW]
  sink = HeadUnitGps(path=tmp_path / "gps.json", mono_clock=lambda: clock[0])
  assert sink.publish(location())
  clock[0] += 1_000_000_000
  assert not sink.publish(location())  # same head-unit timestamp: the car lost its own fix and repeats the last one
  assert not sink.publish(location({2: None}))
  assert sink.publish(location({1: 78}))
  assert sink.stats() == {"received": 4, "accepted": 2, "repeated": 1, "rejected": 1, "last_error": "Missing position"}
  assert read_head_unit_gps(clock[0], tmp_path / "gps.json")[0] == clock[0]


def test_without_timestamps_an_identical_moving_fix_is_a_repeat(tmp_path):
  sink = HeadUnitGps(path=tmp_path / "gps.json", mono_clock=lambda: NOW)
  assert sink.publish(location({1: None}))
  assert not sink.publish(location({1: None}))
  assert sink.publish(location({1: None, 6: 0}))
  assert sink.publish(location({1: None, 6: 0}))  # parked: the same position is still a fresh fix


def test_handoff_expires_and_ignores_damage(tmp_path):
  path = handed_off(tmp_path)
  assert read_head_unit_gps(NOW + HEAD_UNIT_MAX_AGE_NS, path) is not None
  assert read_head_unit_gps(NOW + HEAD_UNIT_MAX_AGE_NS + 1, path) is None
  assert read_head_unit_gps(NOW, tmp_path / "missing.json") is None
  path.write_text("{")
  assert read_head_unit_gps(NOW, path) is None


def publish(sm, tmp_path, phone=False):
  """navigationd publishes the head-unit fix (and the phone's) inside starpilotNavigation."""
  phone_fix = None
  if phone:
    PhoneGpsPublisher(path=tmp_path / "phone.json", mono_clock=lambda: NOW).publish(dict(PHONE_FIX))
    phone_fix = read_phone_gps(NOW, tmp_path / "phone.json")
  publish_status(Bus(sm, NOW), {"status": "idle"}, phone_fix, head_unit_fix=read_head_unit_gps(NOW, handed_off(tmp_path)))


@pytest.mark.parametrize("mode", ["backup", "comma"])
def test_head_unit_is_car_gps_in_every_mode_but_phone_first(tmp_path, mode):
  sm = Messages()
  publish(sm, tmp_path, phone=True)
  name, (stamp, gps) = select_source(sm, NOW, mode)
  assert (name, stamp, gps.longitude) == ("car", NOW, -110.1234567)
  assert select_source(sm, NOW, "phone")[0] == "phone"


def test_comma_and_can_car_gps_come_before_the_head_unit(tmp_path):
  sm = Messages()
  publish(sm, tmp_path)
  car = messaging.new_message("starpilotCarState", valid=True, logMonoTime=NOW)
  car.starpilotCarState.gps = {"sourceMonoTime": NOW - 1, "latitude": 41., "longitude": -111., "horizontalAccuracy": 5., "hasFix": True}
  sm.send("starpilotCarState", car)
  assert select_location(sm, NOW, "comma")[1].longitude == -111.
  native = messaging.new_message("gpsLocation", valid=True, logMonoTime=NOW - 10_000_000)
  native.gpsLocation = {"source": "qcomdiag", "hasFix": True, "horizontalAccuracy": 5., "longitude": -90., "latitude": 30.}
  sm.send("gpsLocation", native)
  assert select_location(sm, NOW, "comma")[1].longitude == -90.


def test_navigation_uses_its_own_head_unit_fix_before_publishing_it(tmp_path):
  fix = read_head_unit_gps(NOW, handed_off(tmp_path))
  assert select_source(Messages(), NOW, "comma", head_unit_fix=fix) == ("car", fix)
  assert select_source(Messages(), NOW, "comma") is None


def test_published_head_unit_fix_expires_after_three_seconds(tmp_path):
  sm = Messages()
  publish(sm, tmp_path)
  assert select_location(sm, NOW + HEAD_UNIT_MAX_AGE_NS, "comma") is not None
  assert select_location(sm, NOW + HEAD_UNIT_MAX_AGE_NS + 1, "comma") is None


def test_map_marker_names_the_head_unit_as_car(tmp_path):
  sm = Messages()
  with patch('openpilot.cereal.messaging.SubMaster', return_value=sm):
    status = NavigationStatusSource(mono_clock=lambda: NOW - 1_000_000_000, boot_clock=lambda: NOW)
  status.mono_clock = lambda: NOW
  publish(sm, tmp_path)
  with patch('openpilot.starpilot.navigation.status.gps_mode', return_value='comma'):
    position = status.map_position()
  assert position["source"] == "car" and position["bearing"] == 90. and position["longitude"] == -110.1234567
