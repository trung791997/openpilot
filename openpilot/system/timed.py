#!/usr/bin/env python3
import datetime
import subprocess
import time
from typing import NoReturn

import openpilot.cereal.messaging as messaging
from openpilot.common.time_helpers import min_date, MAX_DATE, system_time_valid
from openpilot.common.swaglog import cloudlog
from openpilot.common.params import Params
from openpilot.common.gps import get_gps_location_service
from openpilot.starpilot.local_timezone import store_timezone
from openpilot.starpilot.timezone_map import TimezoneMap


def set_time(new_time):
  diff = datetime.datetime.now(datetime.UTC).replace(tzinfo=None) - new_time
  if abs(diff) < datetime.timedelta(seconds=10):
    cloudlog.debug(f"Time diff too small: {diff}")
    return

  cloudlog.debug(f"Setting time to {new_time}")
  try:
    subprocess.run(f"TZ=UTC date -s '{new_time}'", shell=True, check=True)
  except subprocess.CalledProcessError:
    cloudlog.exception("timed.failed_setting_time")


class GpsTimezone:
  """Saves the GPS position's zone as the local zone (next to the phone's, from Galaxy) when it changes, offline."""
  INTERVAL = 60.

  def __init__(self, load=TimezoneMap.load, store=store_timezone):
    self.load, self.store = load, store
    self.map: TimezoneMap | None = None
    self.zone: str | None = None
    self.next_check = 0.

  def update(self, latitude: float, longitude: float, now: float) -> str | None:
    if now < self.next_check or self.load is None:
      return None
    self.next_check = now + self.INTERVAL
    if self.map is None:
      try:
        self.map = self.load()
      except (OSError, ValueError, MemoryError):
        cloudlog.exception("timed.timezone_map_unavailable")
        self.load = None
        return None
    zone = self.map.clear_zone_at(latitude, longitude)
    if zone is None or zone == self.zone or not self.store(zone):
      return None
    self.zone = zone
    return zone


def clock_message():
  msg = messaging.new_message('clocks')
  msg.valid = system_time_valid()
  mono = time.monotonic_ns()
  boot = time.clock_gettime_ns(time.CLOCK_BOOTTIME)
  if time.monotonic_ns() - mono <= 1_000_000:
    # Retain the existing wire fields for replay's clock pair, independently of wall-clock corrections.
    msg.logMonoTime = mono
    msg.clocks.deprecated.monotonicNanos = mono
    msg.clocks.deprecated.bootTimeNanos = boot
  msg.clocks.wallTimeNanos = time.time_ns()
  return msg


def main() -> NoReturn:
  """
    timed has two responsibilities:
    - getting the current time from GPS
    - publishing the time in the logs

    AGNOS will also use NTP to update the time.
  """

  params = Params()
  gps_location_service = get_gps_location_service(params)

  pm = messaging.PubMaster(['clocks'])
  sm = messaging.SubMaster([gps_location_service])
  gps_timezone = GpsTimezone()
  while True:
    sm.update(1000)

    pm.send('clocks', clock_message())

    gps = sm[gps_location_service]
    gps_time = datetime.datetime.fromtimestamp(gps.unixTimestampMillis / 1000., datetime.UTC).replace(tzinfo=None)
    if not sm.updated[gps_location_service] or (time.monotonic() - sm.logMonoTime[gps_location_service] / 1e9) > 2.0:
      continue
    if not gps.hasFix:
      continue
    if (zone := gps_timezone.update(gps.latitude, gps.longitude, time.monotonic())) is not None:
      cloudlog.event("timed.timezone", zone=zone)
    if gps_time < min_date() or gps_time > MAX_DATE:
      continue

    set_time(gps_time)
    time.sleep(10)

if __name__ == "__main__":
  main()
