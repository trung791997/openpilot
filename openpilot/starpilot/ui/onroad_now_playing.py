"""The Android Auto now-playing card; both live rendering and previews use this drawing."""
import time
import unicodedata
from importlib.resources import as_file, files

import pyray as rl

from openpilot.starpilot.ui import clip
from openpilot.starpilot.soloist.now_playing import elapsed_ms
from openpilot.starpilot.system.android_auto.projection_now_playing import image_type
from openpilot.starpilot.system.android_auto.projection_layout import NOW_PLAYING_SIZE, NOW_PLAYING_WIDGET
from openpilot.starpilot.ui.presentation import FontRole, TextSize

ACCENT = rl.Color(199, 174, 247, 255)
INK = rl.Color(15, 13, 23, 255)
TEXT = rl.Color(245, 242, 252, 255)
MUTED = rl.Color(184, 178, 200, 255)
DIM = rl.Color(184, 178, 200, 90)
BUTTON_R = 30
CORNER = 28
CONTROLS = ('shuffle', 'previous', 'play_pause', 'next')
COVER = 60  # cover art square at (x + 20, y + 10); title and artist start at x + TEXT_LEFT
TEXT_LEFT = 96
SCROLL_SPEED = 40  # px/s for a name too wide to fit
SCROLL_GAP = 60
SCROLL_PAUSE = 2.0  # s held at the start of each pass
TRACK_FONTS = {FontRole.SEMI_BOLD: 'Inter-SemiBold.ttf', FontRole.NORMAL: 'Inter-Regular.ttf'}
TRACK_FONT_PX = 48  # drawn at 27-37 px; same Inter metrics as the bitmap atlases to within 0.3 px
# Always cut ASCII as well: raylib 5.5 segfaulted packing a four-glyph atlas ("EVxg" at 64 px) on the dev VM.
ASCII = range(32, 127)


def card_layout(placed, track, viewport_width):
  """(card rect, {control: hit rect}) for what is drawn, or None. Rendering and taps share it."""
  if not placed or not placed['enabled'] or not track:
    return None
  rect = rl.Rectangle(placed['x'], placed['y'], *NOW_PLAYING_SIZE)
  h = rect.height
  if track['status'] != 'playing' and placed.get('collapse'):
    # Paused: just the play button, at whichever end of the card sits nearer the screen edge.
    right = rect.x + rect.width / 2 > viewport_width / 2
    button = rl.Rectangle(rect.x + rect.width - h if right else rect.x, rect.y, h, h)
    return button, {'play_pause': button}
  slot = rect.width / len(CONTROLS)
  return rect, {name: rl.Rectangle(rect.x + index * slot, rect.y + 80, slot, h - 80) for index, name in enumerate(CONTROLS)}


def _center(box: rl.Rectangle) -> rl.Vector2:
  return rl.Vector2(box.x + box.width / 2, box.y + box.height / 2)


def _play_pause(center: rl.Vector2, playing: bool, r: float = BUTTON_R) -> None:
  """An accent disc with the pause bars while playing, else the play triangle."""
  rl.draw_circle_v(center, r, ACCENT)
  k = r / BUTTON_R
  if playing:
    for dx in (-9, 3):
      rl.draw_rectangle_rounded(rl.Rectangle(center.x + dx * k, center.y - 12 * k, 6 * k, 24 * k), .5, 4, INK)
  else:
    rl.draw_triangle(rl.Vector2(center.x - 7 * k, center.y - 13 * k), rl.Vector2(center.x - 7 * k, center.y + 13 * k),
                     rl.Vector2(center.x + 13 * k, center.y), INK)


def _skip(center: rl.Vector2, forward: bool) -> None:
  """A triangle against a bar, pointing the skip direction."""
  d = 1 if forward else -1
  tip, back = center.x + 10 * d, center.x - 10 * d
  points = (rl.Vector2(back, center.y - 13), rl.Vector2(back, center.y + 13), rl.Vector2(tip, center.y))
  rl.draw_triangle(*(points if forward else (points[1], points[0], points[2])), TEXT)
  rl.draw_rectangle_rounded(rl.Rectangle(tip + (0 if forward else -5), center.y - 13, 5, 26), .5, 4, TEXT)


def _shuffle(center: rl.Vector2, state) -> None:
  """Two crossing arrows: accent with a dot when on, white when off, dim when the player cannot shuffle."""
  color = DIM if state is None else ACCENT if state else TEXT
  x, y = center.x, center.y
  for y0, y1 in ((-10, 10), (10, -10)):
    rl.draw_line_ex(rl.Vector2(x - 17, y + y0), rl.Vector2(x + 13, y + y1), 4, color)
    rl.draw_triangle(rl.Vector2(x + 10, y + y1 - 7), rl.Vector2(x + 10, y + y1 + 7), rl.Vector2(x + 20, y + y1), color)
  if state:
    rl.draw_circle_v(rl.Vector2(x, y + 22), 3, color)


def scroll_offset(text_width, width, seconds):
  """How far a too-wide line has scrolled left: still for SCROLL_PAUSE, then one pass to where its repeat starts."""
  if text_width <= width:
    return 0.0
  travel = text_width + SCROLL_GAP
  return min(travel, max(0.0, seconds % (SCROLL_PAUSE + travel / SCROLL_SPEED) - SCROLL_PAUSE) * SCROLL_SPEED)


def _note(box: rl.Rectangle) -> None:
  """Placeholder cover for players without art (phone audio): a dim tile with a music note."""
  rl.draw_rectangle_rounded(box, .2, 8, rl.Color(255, 255, 255, 28))
  x, y = box.x + box.width / 2, box.y + box.height / 2
  rl.draw_circle_v(rl.Vector2(x - 6, y + 12), 8, MUTED)
  rl.draw_line_ex(rl.Vector2(x + 1, y + 12), rl.Vector2(x + 1, y - 17), 4, MUTED)
  rl.draw_line_ex(rl.Vector2(x + 1, y - 15), rl.Vector2(x + 13, y - 8), 4, MUTED)


class TrackText:
  """Title and artist text. The bitmap atlases hold only ASCII, so a name with any other letter (Peter's Vietnamese
  songs, "Em Về Tinh Khôi") is drawn from the Inter TTF cut to ASCII plus that name's letters: a 512 px atlas per role,
  replaced when the name changes and freed with the card."""

  def __init__(self, fonts):
    self.fonts = fonts
    self._cut = {}  # role -> (text, font or None when the bitmap atlas covers it or the TTF failed to load)

  def _font(self, text, role):
    covers = getattr(self.fonts, 'covers', None)
    if covers is None or role not in TRACK_FONTS:
      return None
    cut = self._cut.get(role)
    if cut is None or cut[0] != text:
      self._unload(role)
      self._cut[role] = (text, None if covers(text, role) else self._load(text, role))
    return self._cut[role][1]

  @staticmethod
  def _load(text, role):
    codepoints = sorted({*ASCII, *map(ord, text)})
    buffer = rl.ffi.new("int[]", codepoints)
    with as_file(files("openpilot.selfdrive").joinpath("assets/fonts", TRACK_FONTS[role])) as path:
      font = rl.load_font_ex(str(path), TRACK_FONT_PX, rl.ffi.cast("int *", buffer), len(codepoints))
    # A missing file gives back raylib's shared default font; never own or draw with it.
    if font.texture.id in (0, rl.get_font_default().texture.id) or font.glyphCount == 0:
      return None
    rl.gen_texture_mipmaps(font.texture)
    rl.set_texture_filter(font.texture, rl.TextureFilter.TEXTURE_FILTER_TRILINEAR)
    return font

  def _unload(self, role):
    cut = self._cut.pop(role, None)
    if cut is not None and cut[1] is not None and rl.is_window_ready():
      rl.unload_font(cut[1])

  def close(self):
    for role in list(self._cut):
      self._unload(role)

  def measure(self, text, role, size):
    if (font := self._font(text, role)) is None:
      return self.fonts.measure(text, role, size)
    measured = rl.measure_text_ex(font, text, size * self.fonts.profile.font_scale, 0)  # noqa: TID251
    return TextSize(measured.x, measured.y)

  def draw(self, text, role, size, x, y, color):
    if (font := self._font(text, role)) is None:
      self.fonts.draw(text, role, size, x, y, color)
      return
    draw = getattr(rl, "_orig_draw_text_ex", rl.draw_text_ex)
    draw(font, text, rl.Vector2(x, y), size * self.fonts.profile.font_scale, 0, color)


def nfc(text):
  """Composed letters: a name sent as e + combining marks would otherwise stack loose accents."""
  return unicodedata.normalize('NFC', text)


class NowPlayingWidget:
  def __init__(self, fonts, clock=time.time, ticks=time.monotonic):  # noqa: TID251 epoch-ms track anchors
    self.fonts = fonts
    self.text = TrackText(fonts)
    self.clock = clock
    self.ticks = ticks
    self._scroll = None  # (text, start): a new name starts its scroll from the held start
    self.track = None
    self.artwork = ('', None)  # (url, image bytes) from the feed thread
    self._cover = None  # (url, texture or None), built on the render thread

  def _fit(self, text, role, size, width):
    measured = self.text.measure(text, role, size)
    return size * width / measured.width if measured.width > width else size

  def _cover_texture(self):
    url, data = self.artwork
    if self._cover is not None and self._cover[0] == url:
      return self._cover[1]
    self._drop_cover()
    texture = None
    if url and data and (kind := image_type(data)):
      image = rl.load_image_from_memory(kind, data, len(data))
      if image.width > 0 and image.height > 0:
        rl.image_resize(image, COVER * 2, COVER * 2)
        texture = rl.load_texture_from_image(image)
        rl.set_texture_filter(texture, rl.TextureFilter.TEXTURE_FILTER_BILINEAR)
      rl.unload_image(image)
    self._cover = (url, texture)
    return texture

  def close(self):
    self._drop_cover()
    self.text.close()

  def _drop_cover(self):
    if self._cover is not None and self._cover[1] is not None and rl.is_window_ready():
      rl.unload_texture(self._cover[1])
    self._cover = None

  def _scrolling(self, text, role, size, x, y, width, color):
    text_width = self.text.measure(text, role, size).width
    if text_width <= width:
      self.text.draw(text, role, size, x, y, color)
      return
    now = self.ticks()
    if self._scroll is None or self._scroll[0] != text:
      self._scroll = (text, now)
    offset = scroll_offset(text_width, width, now - self._scroll[1])
    clip.begin_scissor_mode(x, y, width, size + 8)
    try:
      for start in (x - offset, x - offset + text_width + SCROLL_GAP):
        if start < x + width:
          self.text.draw(text, role, size, start, y, color)
    finally:
      clip.end_scissor_mode()

  def render(self, state):
    placed = state.customization['layouts']['large'].get(NOW_PLAYING_WIDGET)
    track = self.track
    if state.alert.size != 'none' or (shape := card_layout(placed, track, getattr(state, 'viewport_width', 0))) is None:
      return
    rect, buttons = shape
    background = rl.Color(15, 13, 23, round(235 * placed.get('opacity', 85) / 100))
    playing = track['status'] == 'playing'
    rl.draw_rectangle_rounded(rect, 2 * CORNER / min(rect.width, rect.height), 16, background)
    if len(buttons) == 1:
      _play_pause(_center(rect), False)
      return
    cover = rl.Rectangle(rect.x + 20, rect.y + 10, COVER, COVER)
    if (texture := self._cover_texture()) is not None:
      rl.draw_texture_pro(texture, rl.Rectangle(0, 0, texture.width, texture.height), cover, rl.Vector2(0, 0), 0, rl.WHITE)
    else:
      _note(cover)
    text_x, text_width = rect.x + TEXT_LEFT, rect.width - TEXT_LEFT - CORNER
    title = nfc(track['title'])
    self.text.draw(title, FontRole.SEMI_BOLD, self._fit(title, FontRole.SEMI_BOLD, 30, text_width), text_x, rect.y + 10, TEXT)
    if artist := nfc(track['artist'] or track['album'] or ''):
      self._scrolling(artist, FontRole.NORMAL, 22, text_x, rect.y + 44, text_width, MUTED)
    left, width = rect.x + CORNER, rect.width - 2 * CORNER
    bar = rl.Rectangle(left, rect.y + 76, width, 4)
    rl.draw_rectangle_rounded(bar, 1.0, 4, rl.Color(255, 255, 255, 50))
    duration = track.get('durationMs')
    if duration and track.get('position'):
      done = min(1.0, elapsed_ms(track, self.clock() * 1000) / duration)
      if done > 0:
        rl.draw_rectangle_rounded(rl.Rectangle(bar.x, bar.y, max(bar.height, bar.width * done), bar.height), 1.0, 4, ACCENT)
    _shuffle(_center(buttons['shuffle']), track.get('shuffle'))
    _skip(_center(buttons['previous']), False)
    _play_pause(_center(buttons['play_pause']), playing, 26)
    _skip(_center(buttons['next']), True)


class NowPlayingTouch:
  """Card taps: a button fires when the finger lifts on the button it went down on. Taps on the card never fall through."""
  def __init__(self, send):
    self.send = send
    self.pressed = None

  def cancel(self):
    self.pressed = None

  def touch(self, kind, x, y, state, placed, track) -> bool:
    """True while the card owns the touch."""
    shape = None if kind == 'cancel' or state is None or state.alert.size != 'none' else card_layout(placed, track, state.viewport_width)
    if shape is None:
      claimed = self.pressed is not None
      self.pressed = None
      return claimed
    point = rl.Vector2(x, y)
    hit = next((name for name, box in shape[1].items() if rl.check_collision_point_rec(point, box)), None)
    if kind == 'down':
      self.pressed = hit
      return rl.check_collision_point_rec(point, shape[0])
    if kind == 'up':
      pressed, self.pressed = self.pressed, None
      if pressed is not None and pressed == hit:
        if pressed != 'shuffle':
          self.send(pressed, {})
        elif track.get('shuffle') is not None:
          self.send('shuffle', {'enabled': not track['shuffle']})
      return pressed is not None or rl.check_collision_point_rec(point, shape[0])
    return self.pressed is not None
