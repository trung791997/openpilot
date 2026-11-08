import os
import tempfile
import unittest
from unittest.mock import patch

from openpilot.common.params import Params
from openpilot.starpilot.sentry_mode.pause import KEY, MAX_SECONDS, MIN_VALID_NOW, clear_pause, is_paused, read_pause, set_pause_until


class TestPause(unittest.TestCase):
  def setUp(self):
    self.directory = tempfile.TemporaryDirectory()
    self.addCleanup(self.directory.cleanup)
    self.environment = patch.dict(os.environ, PARAMS_ROOT=self.directory.name, OPENPILOT_PREFIX="sentrypause")
    self.environment.start()
    self.addCleanup(self.environment.stop)
    self.params = Params()
    self.now = MIN_VALID_NOW + 1_000_000

  def test_absent_is_not_paused(self):
    self.assertIsNone(read_pause(self.params))
    self.assertFalse(is_paused(self.params, self.now))

  def test_duration_pause_expires(self):
    set_pause_until(self.params, self.now + 1800, now=self.now)
    self.assertEqual(read_pause(self.params), self.now + 1800)
    self.assertTrue(is_paused(self.params, self.now + 1799))
    self.assertFalse(is_paused(self.params, self.now + 1800))
    # Expiry clears the saved value rather than leaving a stale epoch behind.
    self.assertIsNone(read_pause(self.params))

  def test_until_specific_time(self):
    until = self.now + 3 * 86400
    set_pause_until(self.params, until, now=self.now)
    self.assertTrue(is_paused(self.params, until - 1))
    self.assertFalse(is_paused(self.params, until))

  def test_invalid_saved_value_is_not_paused_and_is_cleared(self):
    self.params.put(KEY, "not-a-number", block=True)
    self.assertIsNone(read_pause(self.params))
    self.assertFalse(is_paused(self.params, self.now))
    self.assertEqual(self.params.get(KEY), None)

  def test_invalid_clock_fails_active(self):
    set_pause_until(self.params, MIN_VALID_NOW + 10, now=MIN_VALID_NOW)
    self.assertFalse(is_paused(self.params, MIN_VALID_NOW - 1))
    self.assertFalse(is_paused(self.params, 0))

  def test_more_than_thirty_days_rejected_on_set_and_not_paused_on_read(self):
    with self.assertRaises(ValueError):
      set_pause_until(self.params, self.now + MAX_SECONDS + 1, now=self.now)
    # A stored value too far out (e.g. written by another version) is read as not-paused.
    self.params.put(KEY, str(self.now + MAX_SECONDS + 1), block=True)
    self.assertFalse(is_paused(self.params, self.now))

  def test_past_time_rejected_on_set(self):
    with self.assertRaises(ValueError):
      set_pause_until(self.params, self.now - 1, now=self.now)
    with self.assertRaises(ValueError):
      set_pause_until(self.params, self.now, now=self.now)

  def test_clear(self):
    set_pause_until(self.params, self.now + 60, now=self.now)
    clear_pause(self.params)
    self.assertIsNone(read_pause(self.params))
    self.assertFalse(is_paused(self.params, self.now))


if __name__ == "__main__":
  unittest.main()
