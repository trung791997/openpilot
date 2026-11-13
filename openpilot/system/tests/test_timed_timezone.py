from openpilot.system import timed
from openpilot.starpilot.timezone_map import TimezoneMap

NEW_YORK, CHICAGO, PHOENIX = (40.71, -74.0), (41.88, -87.63), (33.45, -112.07)


def make(stored):
  return timed.GpsTimezone(store=lambda zone: stored.append(zone) or True)


def test_saves_the_clear_zone_once_and_again_when_it_changes():
  stored = []
  gps = make(stored)
  assert gps.update(*NEW_YORK, now=0.) == 'America/New_York'
  assert gps.update(*NEW_YORK, now=60.) is None
  assert gps.update(*CHICAGO, now=120.) == 'America/Chicago'
  assert gps.update(*PHOENIX, now=180.) == 'America/Phoenix'
  assert stored == ['America/New_York', 'America/Chicago', 'America/Phoenix']


def test_checks_at_most_once_a_minute():
  stored = []
  gps = make(stored)
  gps.update(*NEW_YORK, now=0.)
  assert gps.update(*CHICAGO, now=30.) is None
  assert gps.update(*CHICAGO, now=60.) == 'America/Chicago'


def test_a_border_keeps_the_current_zone():
  # The Indiana-Illinois line west of Indianapolis: cells there straddle Eastern and Central.
  zone_map = TimezoneMap.load()
  border = next((40.0, lon / 1000) for lon in range(-88200, -86800, 5)
                if zone_map.zone_at(40.0, lon / 1000) == 'America/Chicago' and zone_map.clear_zone_at(40.0, lon / 1000) is None)
  stored = []
  gps = make(stored)
  gps.update(*NEW_YORK, now=0.)
  assert gps.update(*border, now=60.) is None
  assert stored == ['America/New_York']


def test_a_missing_map_turns_the_lookup_off():
  def missing():
    raise OSError('gone')
  stored = []
  gps = timed.GpsTimezone(load=missing, store=lambda zone: stored.append(zone) or True)
  assert gps.update(*NEW_YORK, now=0.) is None
  assert gps.update(*NEW_YORK, now=600.) is None
  assert stored == []


def test_the_shipped_map_covers_the_fleet_and_keeps_dst():
  zone_map = TimezoneMap.load()
  assert zone_map.zone_at(*NEW_YORK) == 'America/New_York'
  assert zone_map.zone_at(21.31, -157.86) == 'Pacific/Honolulu'
  assert zone_map.zone_at(51.5, -0.12) == 'Europe/London'
  assert zone_map.zone_at(10.82, 106.63) == 'Asia/Jakarta'  # Ho Chi Minh: merged with the same UTC+7, no DST
