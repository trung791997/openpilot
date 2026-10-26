from unittest.mock import patch

import pytest

from openpilot.starpilot.ui.speed_limit_shapes import SpeedLimitShapes


def test_native_window_required_without_headless_context():
  with patch('openpilot.starpilot.ui.speed_limit_shapes.rl.is_window_ready', return_value=False):
    with pytest.raises(RuntimeError, match='graphics context'):
      SpeedLimitShapes()._load()


def test_headless_context_gets_past_the_window_check():
  # The projection renderer owns an EGL context but never opens a raylib window.
  with patch('openpilot.starpilot.ui.speed_limit_shapes.rl.is_window_ready', return_value=False), \
       patch('openpilot.starpilot.ui.speed_limit_shapes.rl.load_image') as load_image, \
       patch('openpilot.starpilot.ui.speed_limit_shapes.rl.unload_image'), \
       patch('openpilot.starpilot.ui.speed_limit_shapes.rl.load_texture_from_image') as load_texture, \
       patch('openpilot.starpilot.ui.speed_limit_shapes.rl.set_texture_filter'), \
       patch('openpilot.starpilot.ui.speed_limit_shapes.rl.set_texture_wrap'):
    load_image.return_value.width, load_image.return_value.height = 256, 176
    load_texture.return_value.id = 7
    shapes = SpeedLimitShapes(headless_context=True)
    shapes._load()
    assert shapes.texture is load_texture.return_value
