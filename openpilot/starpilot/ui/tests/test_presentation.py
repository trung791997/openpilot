import unittest
import tempfile
import hashlib
import json
from contextlib import contextmanager
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from openpilot.starpilot.ui.raylib import rl

from openpilot.starpilot.ui.presentation import (BitmapFonts, FontRole, Profile, default_font_directory, font_filename, font_path,
                                                 free_glyph_images, validate_bitmap_font)


@contextmanager
def graphics_context(test):
  owns_context = not rl.is_window_ready()
  if owns_context:
    rl.init_window(64, 64, "Glyph release fixture")
  if not rl.is_window_ready():
    test.skipTest("no graphics context on this host")
  try:
    yield
  finally:
    if owns_context:
      rl.close_window()


def has_cpu_copy(image) -> bool:
  try:
    return image.data is not None
  except ValueError:  # a zero-width glyph (space) still owns an allocation; raylib_nb refuses a 0-byte view of it
    return True


class TestBitmapFonts(unittest.TestCase):
  def test_supplied_sora_brand_is_pinned_and_independent_of_external_font_bundle(self):
    filename = font_filename(Profile.COMPACT, FontRole.BRAND)
    self.assertEqual(filename, "Sora-800.fnt")
    path = font_path(Path("/unavailable-external-fonts"), filename)
    self.assertEqual(path.parent.name, "fonts")
    validate_bitmap_font(path)
    metadata = json.loads(path.parents[2].joinpath("sora-brand-font.json").read_text())
    self.assertEqual((metadata["source"]["axis"], metadata["source"]["value"]), ("wght", 800))

  def test_invalid_descriptor_is_rejected_before_native_parser(self):
    with tempfile.TemporaryDirectory() as directory:
      path = Path(directory) / "font.fnt"
      path.write_text("not a bitmap font\n")
      with self.assertRaisesRegex(ValueError, "Invalid bitmap font"):
        validate_bitmap_font(path)

  def test_changed_atlas_is_rejected_even_with_reviewed_descriptor(self):
    with tempfile.TemporaryDirectory() as directory:
      path = Path(directory) / "Inter-Regular.fnt"
      descriptor, atlas = b'reviewed descriptor', b'reviewed atlas'
      path.write_bytes(descriptor)
      path.with_suffix('.png').write_bytes(atlas)
      manifest = {'files': [{'file': name, 'bytes': len(data), 'sha256': hashlib.sha256(data).hexdigest()}
                            for name, data in ((path.name, descriptor), (path.with_suffix('.png').name, atlas))]}
      with patch.object(Path, 'read_text', return_value=json.dumps(manifest)):
        validate_bitmap_font(path)
        path.with_suffix('.png').write_bytes(b'corrupt atlas')
        with self.assertRaisesRegex(ValueError, 'reviewed font manifest'):
          validate_bitmap_font(path)

  def test_requires_context_before_native_allocation(self):
    with patch("openpilot.starpilot.ui.presentation.rl.is_window_ready", return_value=False), \
         patch("openpilot.starpilot.ui.presentation.rl.load_font") as load:
      with self.assertRaisesRegex(RuntimeError, "graphics context"):
        BitmapFonts(Profile.LARGE, Path("unused"))
      load.assert_not_called()

  def test_failed_resource_load_releases_previously_loaded_fonts(self):
    allocated = SimpleNamespace(texture=SimpleNamespace(id=1), glyphCount=1)
    with patch("openpilot.starpilot.ui.presentation.rl.is_window_ready", return_value=True), \
         patch("openpilot.starpilot.ui.presentation.validate_bitmap_font"), \
         patch("openpilot.starpilot.ui.presentation.rl.get_font_default", return_value=SimpleNamespace(texture=SimpleNamespace(id=99))), \
         patch.object(Path, "is_file", return_value=True), \
         patch("openpilot.starpilot.ui.presentation.rl.load_font", side_effect=[allocated, RuntimeError("broken asset")]), \
         patch("openpilot.starpilot.ui.presentation.rl.gen_texture_mipmaps"), \
         patch("openpilot.starpilot.ui.presentation.rl.set_texture_filter"), \
         patch("openpilot.starpilot.ui.presentation.rl.unload_font") as unload:
      with self.assertRaisesRegex(RuntimeError, "broken asset"):
        BitmapFonts(Profile.LARGE, Path("unused"))
      unload.assert_called_once_with(allocated)

  def test_failed_load_cannot_adopt_or_unload_the_shared_default_font(self):
    default = SimpleNamespace(texture=SimpleNamespace(id=99), glyphCount=95)
    with patch("openpilot.starpilot.ui.presentation.rl.is_window_ready", return_value=True), \
         patch("openpilot.starpilot.ui.presentation.validate_bitmap_font"), \
         patch("openpilot.starpilot.ui.presentation.rl.get_font_default", return_value=default), \
         patch.object(Path, "is_file", return_value=True), \
         patch("openpilot.starpilot.ui.presentation.rl.load_font", return_value=default), \
         patch("openpilot.starpilot.ui.presentation.rl.unload_font") as unload:
      with self.assertRaisesRegex(RuntimeError, "Unable to load bitmap font"):
        BitmapFonts(Profile.LARGE, Path("unused"))
      unload.assert_not_called()

  def test_drawing_does_not_double_apply_an_application_text_scale(self):
    fonts = object.__new__(BitmapFonts)
    fonts.profile = Profile.COMPACT
    fonts._fonts = {"Inter-Regular.fnt": SimpleNamespace(texture=SimpleNamespace(id=1))}
    with patch("openpilot.starpilot.ui.presentation.rl._orig_draw_text_ex", create=True) as raw_draw, \
         patch("openpilot.starpilot.ui.presentation.rl.draw_text_ex") as scaled_draw:
      fonts.draw("sample", FontRole.ROMAN, 36, 10, 20)
      scaled_draw.assert_not_called()
      self.assertAlmostEqual(raw_draw.call_args.args[3], 36 * 1.16)

  def test_freeing_glyph_images_frees_each_cpu_bitmap_once(self):
    with graphics_context(self):
      font = rl.load_font(str(default_font_directory() / "Inter-Regular.fnt"))
      try:
        self.assertNotEqual(font.texture.id, 0)
        held = [has_cpu_copy(font.glyphs[index].image) for index in range(font.glyphCount)]
        metrics = [(g.value, g.offsetX, g.offsetY, g.advanceX) for g in font.glyphs]
        recs = [(r.x, r.y, r.width, r.height) for r in font.recs]
        self.assertTrue(any(held))
        self.assertEqual(free_glyph_images(font), sum(held))  # every CPU bitmap freed in raylib's own glyph array
        self.assertEqual(free_glyph_images(font), 0)  # and only once: the second pass finds nothing left
        self.assertTrue(all(font.glyphs[index].image.data is None for index in range(font.glyphCount)))
        self.assertEqual([(g.value, g.offsetX, g.offsetY, g.advanceX) for g in font.glyphs], metrics)
        self.assertEqual([(r.x, r.y, r.width, r.height) for r in font.recs], recs)
      finally:
        rl.unload_font(font)  # frees each now-NULL glyph image again: free(NULL), no double free

  def test_glyph_release_is_opt_in_and_trims_the_heap_once(self):
    for release in (False, True):
      loaded = []

      def load(_, loaded=loaded):
        loaded.append(SimpleNamespace(texture=SimpleNamespace(id=len(loaded) + 1), glyphCount=1))
        return loaded[-1]

      with patch("openpilot.starpilot.ui.presentation.rl.is_window_ready", return_value=True), \
           patch("openpilot.starpilot.ui.presentation.validate_bitmap_font"), \
           patch("openpilot.starpilot.ui.presentation.rl.get_font_default", return_value=SimpleNamespace(texture=SimpleNamespace(id=99))), \
           patch.object(Path, "is_file", return_value=True), \
           patch("openpilot.starpilot.ui.presentation.rl.load_font", side_effect=load), \
           patch("openpilot.starpilot.ui.presentation.rl.gen_texture_mipmaps"), \
           patch("openpilot.starpilot.ui.presentation.rl.set_texture_filter"), \
           patch("openpilot.starpilot.ui.presentation.free_glyph_images") as free, \
           patch("openpilot.starpilot.ui.presentation.return_freed_heap") as trim:
        BitmapFonts(Profile.LARGE, Path("unused"), release_glyph_images=release)
      self.assertEqual([call.args[0] for call in free.call_args_list], loaded if release else [])
      self.assertEqual(trim.call_count, int(release))

  def test_fonts_without_cpu_glyph_copies_draw_and_measure_the_same(self):
    with graphics_context(self):
      self._draw_and_measure_the_same()

  def _draw_and_measure_the_same(self):
    width, height = 900, 160
    target = rl.load_render_texture(width, height)
    try:
      def snapshot(fonts):
        measured, pixels = [], []
        for role in FontRole:
          for text in ("Starting navigation 0.3 mi", "Hello 123 km/h"):
            size = fonts.measure(text, role, 48)
            measured.append((role, text, size.width, size.height, fonts.horizontal_ink(text, role, 48), fonts.vertical_ink(text, role, 48)))
          rl.begin_texture_mode(target)
          rl.clear_background(rl.BLANK)
          fonts.draw("Starting navigation 0.3 mi 123", role, 48, 10, 40)
          rl.end_texture_mode()
          image = rl.load_image_from_texture(target.texture)
          data = bytes(image.data)
          self.assertEqual(len(data), width * height * 4)
          pixels.append(data)
          rl.unload_image(image)
        return measured, pixels

      with BitmapFonts(Profile.LARGE, default_font_directory()) as kept:
        expected = snapshot(kept)
      self.assertTrue(all(any(pixels) for pixels in expected[1]), 'the fixture must draw visible text')
      with BitmapFonts(Profile.LARGE, default_font_directory(), release_glyph_images=True) as released:
        self.assertTrue(all(font.glyphs[index].image.data is None
                            for font in released._fonts.values() for index in range(font.glyphCount)))
        self.assertEqual(snapshot(released), expected)
    finally:
      rl.unload_render_texture(target)


if __name__ == "__main__":
  unittest.main()


class TestLocalizedPresentation(unittest.TestCase):
  def test_measure_and_draw_use_the_same_translated_text_and_font(self):
    fonts = object.__new__(BitmapFonts)
    fonts.profile = Profile.COMPACT
    fonts._measurements = {}
    font = SimpleNamespace(texture=SimpleNamespace(id=44))
    with patch('openpilot.starpilot.ui.presentation.tr', side_effect=lambda text: {'Home': 'ホーム'}.get(text, text)), \
         patch.object(fonts, '_text_font', return_value=font) as select, \
         patch('openpilot.starpilot.ui.presentation.rl.measure_text_ex', return_value=SimpleNamespace(x=80, y=24)) as measure, \
         patch('openpilot.starpilot.ui.presentation.rl._orig_draw_text_ex', create=True) as draw:
      assert fonts.measure('Home', FontRole.NORMAL, 20).width == 80
      fonts.draw('Home', FontRole.NORMAL, 20, 10, 20)
      assert measure.call_args.args[1] == draw.call_args.args[1] == 'ホーム'
      assert select.call_args.args[:2] == ('ホーム', FontRole.NORMAL)
      fonts.draw('StarPilot', FontRole.BRAND, 20, 10, 20)
      assert draw.call_args.args[1] == 'StarPilot'

  def test_native_language_font_uses_bundled_noto_and_releases_previous(self):
    fonts = object.__new__(BitmapFonts)
    old = SimpleNamespace(texture=SimpleNamespace(id=22), glyphCount=1)
    new = SimpleNamespace(texture=SimpleNamespace(id=23), glyphCount=100)
    fonts._language_fonts = {'vi': old}
    fonts._measurements = {}
    fonts._vertical_ink = {}
    fonts._horizontal_ink = {}
    with patch('openpilot.starpilot.ui.presentation.multilang') as language, \
         patch('openpilot.starpilot.ui.presentation.rl.load_font_ex', return_value=new) as load, \
         patch('openpilot.starpilot.ui.presentation.rl.get_font_default', return_value=SimpleNamespace(texture=SimpleNamespace(id=99))), \
         patch('openpilot.starpilot.ui.presentation.rl.unload_font') as unload, \
         patch('openpilot.starpilot.ui.presentation.rl.gen_texture_mipmaps'), \
         patch('openpilot.starpilot.ui.presentation.rl.set_texture_filter'):
      language.language = 'ja'
      assert fonts._text_font('ホーム', FontRole.NORMAL) is new
      assert load.call_args.args[0].endswith('NotoSansCJKjp-Regular.otf')
      unload.assert_called_once_with(old)
      assert fonts._language_fonts == {'ja': new}
      assert fonts._text_font('保存', FontRole.NORMAL) is new
      load.assert_called_once()

  def test_home_statistics_use_shared_localized_measurement_and_drawing(self):
    from unittest.mock import Mock
    from openpilot.starpilot.ui.home_stats import DriveStatsDashboard
    from openpilot.starpilot.ui.home_state import DriveStatsData, DriveSummary
    fonts = Mock(spec=BitmapFonts, font=Mock(side_effect=[object(), object(), object()]),
                 measure=Mock(return_value=SimpleNamespace(width=80, height=24)), draw=Mock())
    data = DriveStatsData(DriveSummary(), DriveSummary(), DriveSummary(), (), ())
    dashboard = DriveStatsDashboard(fonts, data)
    measured = dashboard._measure(dashboard._font_bold, 'PERSONAL RECORDS', 20)
    assert (measured.x, measured.y) == (80, 24)
    dashboard._draw_text(dashboard._font_bold, 'PERSONAL RECORDS', SimpleNamespace(x=10, y=20), 20, 0, 'color')
    fonts.measure.assert_called_once_with('PERSONAL RECORDS', FontRole.BOLD, 20)
    fonts.draw.assert_called_once_with('PERSONAL RECORDS', FontRole.BOLD, 20, 10, 20, 'color', spacing=0)
