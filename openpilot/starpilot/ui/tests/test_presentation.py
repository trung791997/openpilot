import unittest
import tempfile
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import pyray as rl

from openpilot.starpilot.ui.presentation import (BitmapFonts, FontRole, Profile, default_font_directory, font_filename, font_path,
                                                 free_glyph_images, validate_bitmap_font)


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
    glyphs = rl.ffi.new("GlyphInfo[3]")
    pixels = [rl.ffi.new("unsigned char[4]"), rl.ffi.new("unsigned char[4]")]
    glyphs[0].image.data, glyphs[2].image.data = pixels
    font = SimpleNamespace(glyphCount=3, glyphs=glyphs)
    freed = []
    with patch("openpilot.starpilot.ui.presentation.rl.unload_image", side_effect=lambda image: freed.append(image.data)):
      free_glyph_images(font)
      free_glyph_images(font)
    self.assertEqual(freed, [rl.ffi.cast("void *", pixel) for pixel in pixels])
    self.assertTrue(all(glyphs[index].image.data == rl.ffi.NULL for index in range(3)))

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
    owns_context = not rl.is_window_ready()
    if owns_context:
      rl.init_window(64, 64, "Glyph release fixture")
    if not rl.is_window_ready():
      self.skipTest("no graphics context on this host")
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
          pixels.append(bytes(rl.ffi.buffer(image.data, width * height * 4)))
          rl.unload_image(image)
        return measured, pixels

      with BitmapFonts(Profile.LARGE, default_font_directory()) as kept:
        expected = snapshot(kept)
      self.assertTrue(all(any(pixels) for pixels in expected[1]), 'the fixture must draw visible text')
      with BitmapFonts(Profile.LARGE, default_font_directory(), release_glyph_images=True) as released:
        self.assertTrue(all(font.glyphs[index].image.data == rl.ffi.NULL
                            for font in released._fonts.values() for index in range(font.glyphCount)))
        self.assertEqual(snapshot(released), expected)
    finally:
      rl.unload_render_texture(target)
      if owns_context:
        rl.close_window()


if __name__ == "__main__":
  unittest.main()
