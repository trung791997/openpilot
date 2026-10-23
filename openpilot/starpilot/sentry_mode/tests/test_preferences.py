import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from openpilot.common.params import Params
from openpilot.starpilot.sentry_mode.policy import Settings
from openpilot.starpilot.sentry_mode.preferences import KEY, Preferences, decode, encode, enabled, read_preferences


class TestPreferences(unittest.TestCase):
  def test_codec_requires_exact_finite_document(self):
    value = Preferences(True, Settings(0.04, 1.0))
    self.assertEqual(decode(encode(value)), value)
    corrupt = (
      b'{}', b'{"version":1,"enabled":true,"sensitivity":0.04,"warningTimeSeconds":1,"extra":0}',
      b'{"version":1,"enabled":true,"enabled":false,"sensitivity":0.04,"warningTimeSeconds":1}',
      b'{"version":1,"enabled":true,"sensitivity":NaN,"warningTimeSeconds":1}',
      b'{"version":1,"enabled":true,"sensitivity":0.04,"warningTimeSeconds":1e400}',
      b'{"version":1,"enabled":1,"sensitivity":0.04,"warningTimeSeconds":1}',
      b'[' * 250 + b'0' + b']' * 250,
      b'{"capture":"video","enabled":true,"sensitivity":0.04,"version":1,"warningTimeSeconds":1}',
      # Pictures is the implicit default; writing it out would make two byte forms for one choice.
      b'{"capture":"pictures","enabled":true,"sensitivity":0.04,"version":1,"warningTimeSeconds":1}',
    )
    for raw in corrupt:
      with self.subTest(raw=raw[:30]):
        self.assertIsNone(decode(raw))

  def test_capture_choice_is_optional_and_only_written_for_clips(self):
    old = b'{"enabled":true,"sensitivity":0.04,"version":1,"warningTimeSeconds":1.0}'
    self.assertEqual(decode(old).capture, "pictures")
    self.assertEqual(encode(decode(old)), old)
    clip = Preferences(True, Settings(0.04, 1.0), "clip")
    self.assertEqual(decode(encode(clip)), clip)
    with self.assertRaises(ValueError):
      encode(Preferences(True, capture="video"))

  def test_actual_params_absent_corrupt_and_oversized_are_off_without_rewrite(self):
    with tempfile.TemporaryDirectory() as directory, patch.dict(os.environ, PARAMS_ROOT=directory, OPENPILOT_PREFIX="sentrytest"):
      params = Params()
      self.assertFalse(enabled(params))
      raw = encode(Preferences(True))
      Path(params.get_param_path(KEY)).write_bytes(raw)
      self.assertTrue(enabled(params))
      for invalid in (b'{"version":2}', b'x' * 513):
        Path(params.get_param_path(KEY)).write_bytes(invalid)
        saved = read_preferences(params)
        self.assertFalse(saved.valid)
        self.assertFalse(enabled(params))
        self.assertEqual(Path(params.get_param_path(KEY)).read_bytes(), invalid)

  def test_unreadable_sources_do_not_arm(self):
    with tempfile.TemporaryDirectory() as directory, patch.dict(os.environ, PARAMS_ROOT=directory, OPENPILOT_PREFIX="sentrytest"):
      params = Params()
      source = Path(params.get_param_path(KEY))
      source.symlink_to(Path(directory) / "elsewhere")
      self.assertFalse(read_preferences(params).readable)
      self.assertFalse(enabled(params))


if __name__ == "__main__":
  unittest.main()
