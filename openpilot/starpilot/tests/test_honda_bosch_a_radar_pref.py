import os
import tempfile
import unittest

from openpilot.starpilot.vehicle_preferences import honda_bosch_a_radar_requested


class FakeParams:
  def __init__(self, root):
    self.root = root

  def get_param_path(self, key):
    return os.path.join(self.root, key)


class TestHondaBoschARadarPreference(unittest.TestCase):
  def _requested(self, value):
    with tempfile.TemporaryDirectory() as root:
      if value is not None:
        with open(os.path.join(root, "HondaBoschARadar"), "wb") as f:
          f.write(value)
      return honda_bosch_a_radar_requested(FakeParams(root))

  def test_missing_is_default_on(self):
    self.assertTrue(self._requested(None))

  def test_exact_one_is_on(self):
    self.assertTrue(self._requested(b"1"))

  def test_zero_is_off(self):
    self.assertFalse(self._requested(b"0"))

  def test_malformed_is_off(self):
    for value in (b"", b"2", b"1\n", b"true", b"x" * 9):
      with self.subTest(value=value):
        self.assertFalse(self._requested(value))

  def test_directory_is_off(self):
    with tempfile.TemporaryDirectory() as root:
      os.mkdir(os.path.join(root, "HondaBoschARadar"))
      self.assertFalse(honda_bosch_a_radar_requested(FakeParams(root)))


if __name__ == "__main__":
  unittest.main()
