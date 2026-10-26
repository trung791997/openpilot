"""The Android Auto now-playing card; both live rendering and previews use this drawing."""
import math
import time
import unicodedata
from importlib.resources import as_file, files

import pyray as rl

from openpilot.starpilot.ui import clip
from openpilot.starpilot.soloist.now_playing import ELSEWHERE, elapsed_ms
from openpilot.starpilot.system.android_auto.projection_now_playing import image_type
from openpilot.starpilot.system.android_auto.projection_layout import NOW_PLAYING_SIZE, NOW_PLAYING_WIDGET
from openpilot.starpilot.ui.presentation import FontRole, TextSize

ACCENT = rl.Color(199, 174, 247, 255)  # without cover art; with it the card takes its color from the art
INK = rl.Color(15, 13, 23, 255)
TEXT = rl.Color(245, 242, 252, 255)
MUTED = rl.Color(214, 210, 224, 255)
DIM = rl.Color(184, 178, 200, 90)
DISC = rl.Color(255, 255, 255, 46)
SPOTIFY_GREEN = rl.Color(30, 215, 96, 255)  # Smart Shuffle is drawn in Spotify's green, as in its app
OFF = rl.Color(196, 192, 206, 255)  # a shuffle or repeat that is off; opaque, so overlapping strokes do not show
# One UI 9's media card: device line, centered "title • artist", a thick seek bar with times under it, and a row of
# shuffle, previous, play/pause, next and repeat over the darkened cover art.
CONTROLS = ('shuffle', 'previous', 'play_pause', 'next', 'repeat')
CORNER = 64
PAD = 46
LINE_Y = 70  # center of the title line
BAR_Y = 122
TIMES_Y = 140
ROW_TOP = 164  # the button row: hit boxes from here to the bottom of the card, icons centered in them
SLOT = .16  # button spacing as a share of the card width
BUTTON_R = 36
COLLAPSED = 120  # the paused, collapsed card: just a play button this big
PILL = (340, 96)  # "Play on car", while Spotify plays on another device
REPEAT_NEXT = {'off': 'context', 'context': 'track', 'track': 'off'}
SCROLL_SPEED = 40  # px/s for a name too wide to fit
SCROLL_GAP = 60
SCROLL_PAUSE = 2.0  # s held at the start of each pass
TRACK_FONTS = {FontRole.SEMI_BOLD: 'Inter-SemiBold.ttf', FontRole.NORMAL: 'Inter-Regular.ttf'}
TRACK_FONT_PX = 48  # drawn at 27-37 px; same Inter metrics as the bitmap atlases to within 0.3 px
# Always cut ASCII as well: raylib 5.5 segfaulted packing a four-glyph atlas ("EVxg" at 64 px) on the dev VM.
ASCII = range(32, 127)
# Changing songs: Spotify reports "buffering" for a moment, which the feed drops, so the card holds the last song
# instead of blinking out; the play button spins until the player answers a tap; the new cover and title fade in.
HOLD = 4.0  # s the last song stays up while the feed has none
COVER_HOLD = 3.0  # s the old cover stays while the new song has no cover yet
COVER_FADE = .8
TITLE_FADE = .6
TITLE_SLIDE = 10  # px the new title rises while it fades in
PRESS_GROW = .15  # s for the gray circle behind a tapped button to open
PRESS_FADE = .35  # s for it to fade once the finger lifts
PRESS_MAX = 2.0  # s a held finger keeps it up
PENDING = 3.0  # s at most the play button spins waiting for the player to answer


def card_layout(placed, track, viewport_width):
  """(card rect, {control: hit rect}) for what is drawn, or None. Rendering and taps share it."""
  if not placed or not placed['enabled'] or not track:
    return None
  rect = rl.Rectangle(placed['x'], placed['y'], *NOW_PLAYING_SIZE)
  if track['status'] == ELSEWHERE:
    # Another device took Spotify: one pill at the bottom center of the card's place that takes playback back.
    pill = rl.Rectangle(rect.x + (rect.width - PILL[0]) / 2, rect.y + rect.height - PILL[1], *PILL)
    return pill, {'play_here': pill}
  if track['status'] != 'playing' and placed.get('collapse'):
    # Paused: just the play button, in the bottom corner nearer the screen edge.
    right = rect.x + rect.width / 2 > viewport_width / 2
    button = rl.Rectangle(rect.x + rect.width - COLLAPSED if right else rect.x, rect.y + rect.height - COLLAPSED, COLLAPSED, COLLAPSED)
    return button, {'play_pause': button}
  slot = rect.width * SLOT
  left = rect.x + rect.width / 2 - slot * len(CONTROLS) / 2
  return rect, {name: rl.Rectangle(left + index * slot, rect.y + ROW_TOP, slot, rect.height - ROW_TOP)
                for index, name in enumerate(CONTROLS)}


def _ease(t: float) -> float:
  t = min(1.0, max(0.0, t))
  return t * t * (3 - 2 * t)


def _faded(color: rl.Color, alpha: float) -> rl.Color:
  return rl.Color(color.r, color.g, color.b, round(color.a * alpha))


def _play_pause(center: rl.Vector2, playing: bool, r: float = BUTTON_R, disc=DISC, ink=TEXT, spin: float | None = None) -> None:
  """A disc with the pause bars while playing, else the play triangle; a turning arc while the player is busy."""
  rl.draw_circle_v(center, r, disc)
  k = r / BUTTON_R
  if spin is not None:
    start = spin * 360 % 360
    rl.draw_ring(center, 13 * k, 17 * k, start, start + 270, 24, ink)
  elif playing:
    for dx in (-10, 3):
      rl.draw_rectangle_rounded(rl.Rectangle(center.x + dx * k, center.y - 13 * k, 7 * k, 26 * k), .5, 4, ink)
  else:
    rl.draw_triangle(rl.Vector2(center.x - 8 * k, center.y - 14 * k), rl.Vector2(center.x - 8 * k, center.y + 14 * k),
                     rl.Vector2(center.x + 14 * k, center.y), ink)


def _skip(center: rl.Vector2, forward: bool) -> None:
  """A translucent disc with a triangle against a bar, pointing the skip direction."""
  rl.draw_circle_v(center, BUTTON_R, DISC)
  d = 1 if forward else -1
  tip, back = center.x + 9 * d, center.x - 9 * d
  points = (rl.Vector2(back, center.y - 12), rl.Vector2(back, center.y + 12), rl.Vector2(tip, center.y))
  rl.draw_triangle(*(points if forward else (points[1], points[0], points[2])), TEXT)
  rl.draw_rectangle_rounded(rl.Rectangle(tip + (0 if forward else -5), center.y - 12, 5, 24), .5, 4, TEXT)


def _toggle_color(state, on):
  # On is white with a dot under it, as on One UI's card; the cover's accent color could vanish into the cover.
  return DIM if state is None else TEXT if on else OFF


def _bezier(p0, p1, p2, p3, steps=10):
  return [rl.Vector2(*((1 - t) ** 3 * a + 3 * (1 - t) ** 2 * t * b + 3 * (1 - t) * t * t * c + t ** 3 * d
                       for a, b, c, d in zip((p0.x, p0.y), (p1.x, p1.y), (p2.x, p2.y), (p3.x, p3.y), strict=True)))
          for t in (i / steps for i in range(steps + 1))]


def _stroke(points, thick, color) -> None:
  """A round-capped polyline."""
  for a, b in zip(points, points[1:], strict=False):
    rl.draw_line_ex(a, b, thick, color)
  for point in points[::len(points) - 1]:
    rl.draw_circle_v(point, thick / 2, color)


SPARKLE_STEPS = 48


def _sparkle(center: rl.Vector2, r: float, color) -> None:
  """Spotify's four-point Smart Shuffle star: points on the axes, sides curving in (a superellipse with p < 1)."""
  p = .75
  points = []
  for i in range(SPARKLE_STEPS):
    angle = 2 * math.pi * i / SPARKLE_STEPS
    c, s = math.cos(angle), math.sin(angle)
    reach = r / (abs(c) ** p + abs(s) ** p) ** (1 / p)
    points.append(rl.Vector2(center.x + reach * c, center.y - reach * s))
  for i in range(SPARKLE_STEPS):
    rl.draw_triangle(center, points[i], points[(i + 1) % SPARKLE_STEPS], color)


def _shuffle_arrows(x: float, y: float, k: float, thick: float, color) -> None:
  """Two crossing curves with open arrowheads, centered on (x, y) and scaled by k."""
  for y0, y1 in ((-10, 10), (10, -10)):
    _stroke(_bezier(rl.Vector2(x - 18 * k, y + y0 * k), rl.Vector2(x - 2 * k, y + y0 * k), rl.Vector2(x - 2 * k, y + y1 * k),
                    rl.Vector2(x + 16 * k, y + y1 * k)), thick, color)
    _stroke([rl.Vector2(x + 11 * k, y + (y1 - 6) * k), rl.Vector2(x + 17 * k, y + y1 * k),
             rl.Vector2(x + 11 * k, y + (y1 + 6) * k)], thick, color)


def _shuffle(center: rl.Vector2, state, smart=False) -> None:
  """Spotify's shuffle: two crossing curves with open arrowheads. A dot when on, dim when the player cannot shuffle.
  Smart Shuffle is Spotify's own icon, all green: a large star top-left, small arrows bottom-right, the dot under them."""
  if smart:
    color = SPOTIFY_GREEN
    _sparkle(rl.Vector2(center.x - 14, center.y - 13), 15, color)
    _shuffle_arrows(center.x + 12, center.y + 7, .8, 3.5, color)
  else:
    color = _toggle_color(state, state)
    _shuffle_arrows(center.x + 2, center.y, 1, 3.5, color)
  if state or smart:
    rl.draw_circle_v(rl.Vector2(center.x, center.y + 26), 3, color)


def _repeat(center: rl.Vector2, state) -> None:
  """A loop of two arrows: white with a dot when on, plus a small bar for repeating one song; dim without a player."""
  color = _toggle_color(state, state in ('context', 'track'))
  x, y = center.x, center.y
  rl.draw_line_ex(rl.Vector2(x - 14, y - 9), rl.Vector2(x + 12, y - 9), 4, color)
  rl.draw_line_ex(rl.Vector2(x + 14, y + 9), rl.Vector2(x - 12, y + 9), 4, color)
  rl.draw_line_ex(rl.Vector2(x - 16, y - 11), rl.Vector2(x - 16, y + 4), 4, color)
  rl.draw_line_ex(rl.Vector2(x + 16, y + 11), rl.Vector2(x + 16, y - 4), 4, color)
  rl.draw_triangle(rl.Vector2(x + 10, y - 16), rl.Vector2(x + 10, y - 2), rl.Vector2(x + 19, y - 9), color)
  rl.draw_triangle(rl.Vector2(x - 10, y + 16), rl.Vector2(x - 10, y + 2), rl.Vector2(x - 19, y + 9), color)
  if state == 'track':
    rl.draw_rectangle_rounded(rl.Rectangle(x - 2, y - 5, 4, 10), .5, 4, color)
  if state in ('context', 'track'):
    rl.draw_circle_v(rl.Vector2(x, y + 24), 3, color)


def _spotify_texture(size: float) -> rl.Texture:
  from openpilot.system.ui.lib.application import gui_app  # its texture cache loads the PNG once; imported late, it is the whole UI stack
  return gui_app.texture("icons/spotify.png", size, size)


def _spotify(center: rl.Vector2, r: float = 13) -> None:
  """Spotify's own mark (icons/spotify.png, rasterized from the official vector logo), centered on center."""
  texture = _spotify_texture(2 * r)
  rl.draw_texture_pro(texture, rl.Rectangle(0, 0, texture.width, texture.height),
                      rl.Rectangle(center.x - r, center.y - r, 2 * r, 2 * r), rl.Vector2(0, 0), 0, rl.WHITE)


def _wave(left, right, y, accent, phase) -> None:
  """One UI's soft wave riding on the played part of the seek bar: two slow, overlapping swells."""
  for speed, length, height, alpha in ((1.0, 70, 14, 70), (-.7, 110, 10, 55)):
    color = rl.Color((accent.r + 255) // 2, (accent.g + 255) // 2, (accent.b + 255) // 2, alpha)
    x = left
    while x < right:
      rise = height * (.55 + .45 * math.sin(x / length * 2 * math.pi + phase * speed))
      rise *= min(1.0, (x - left) / 30, (right - x) / 30)  # taper into the bar at both ends
      rl.draw_rectangle_rec(rl.Rectangle(x, y - rise, min(2, right - x), rise), color)
      x += 2


def clock_text(ms):
  seconds = max(0, int(ms // 1000))
  return f'{seconds // 60:02d}:{seconds % 60:02d}'


def accent_from(colors, count):
  """The card color from cover pixels: the average weighted by how colorful each pixel is, lifted to a bright
  shade so the seek bar reads over the dark card. None for a gray cover."""
  total = r = g = b = 0.0
  for index in range(count):
    pixel = colors[index]
    high, low = max(pixel.r, pixel.g, pixel.b), min(pixel.r, pixel.g, pixel.b)
    weight = (high - low) * high / 255
    total += weight
    r, g, b = r + pixel.r * weight, g + pixel.g * weight, b + pixel.b * weight
  if total < count * 6:
    return None
  r, g, b = r / total, g / total, b / total
  lift = 235 / max(r, g, b, 1)
  return rl.Color(round(min(255, r * lift)), round(min(255, g * lift)), round(min(255, b * lift)), 255)


def scroll_offset(text_width, width, seconds):
  """How far a too-wide line has scrolled left: still for SCROLL_PAUSE, then one pass to where its repeat starts."""
  if text_width <= width:
    return 0.0
  travel = text_width + SCROLL_GAP
  return min(travel, max(0.0, seconds % (SCROLL_PAUSE + travel / SCROLL_SPEED) - SCROLL_PAUSE) * SCROLL_SPEED)


def pill_rows(height, r, step=1):
  """(y, height, inset) strips that fill a rounded rect of this height: the cover drawn strip by strip takes the
  card's shape, so it needs no shader or mask."""
  r = min(r, height / 2)
  for top in (True, False):
    if not top and height > 2 * r:
      yield r, height - 2 * r, 0.0
    y = 0.0
    while y < r:
      h = min(step, r - y)
      dy = r - (y + h / 2)
      yield (y if top else height - y - h), h, r - math.sqrt(r * r - dy * dy)
      y += h


class TrackText:
  """Title and artist text. The bitmap atlases hold only ASCII, so a name with any other letter (Peter's Vietnamese
  songs, "Em Về Tinh Khôi") is drawn from the Inter TTF cut to ASCII plus that name's letters: a 512 px atlas per line,
  replaced when the name changes and freed with the card."""

  def __init__(self, fonts):
    self.fonts = fonts
    self._cut = {}  # (role, slot) -> (text, font or None when the bitmap atlas covers it or the TTF failed to load)

  def _font(self, text, role, slot):
    covers = getattr(self.fonts, 'covers', None)
    if covers is None or role not in TRACK_FONTS:
      return None
    key = (role, slot)
    cut = self._cut.get(key)
    if cut is None or cut[0] != text:
      self._unload(key)
      self._cut[key] = (text, None if covers(text, role) else self._load(text, role))
    return self._cut[key][1]

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

  def _unload(self, key):
    cut = self._cut.pop(key, None)
    if cut is not None and cut[1] is not None and rl.is_window_ready():
      rl.unload_font(cut[1])

  def close(self):
    for key in list(self._cut):
      self._unload(key)

  def measure(self, text, role, size, slot=None):
    """``slot`` keeps one cut per line, so two lines in the same font do not swap atlases every frame."""
    if (font := self._font(text, role, slot)) is None:
      return self.fonts.measure(text, role, size)
    measured = rl.measure_text_ex(font, text, size * self.fonts.profile.font_scale, 0)  # noqa: TID251
    return TextSize(measured.x, measured.y)

  def draw(self, text, role, size, x, y, color, slot=None):
    if (font := self._font(text, role, slot)) is None:
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
    self._cover = None  # (url, blurred texture or None, accent or None), built on the render thread
    self._old_cover = None  # (texture, start): the last song's cover, fading out under the new one
    self._bare_since = None  # when the cover url went empty while a cover was up
    self._shown = None  # (track, when): the last song the feed had, held through a skip
    self._line = None  # ((title, artist), start of its fade-in)
    self._press = None  # (button, down, up or None) for the gray circle behind a tapped button
    self._pending = None  # (song key, start): a skip or play/pause the player has not answered yet

  def press(self, name):
    """The finger went down on a button."""
    self._press = (name, self.ticks(), None)

  def release(self, fired):
    """The finger lifted; ``fired`` is the button it sent, or None."""
    now = self.ticks()
    if self._press is not None:
      self._press = (self._press[0], self._press[1], now)
    if fired in ('previous', 'next', 'play_pause', 'play_here') and (track := self.showing()):
      self._pending = (_song_key(track), now)

  def showing(self):
    """The song on the card: the feed's, or the last one for HOLD seconds after the feed lost it."""
    if self.track:
      return self.track
    if self._shown is not None and self.ticks() - self._shown[1] < HOLD:
      return self._shown[0]
    return None

  def on_screen(self, state):
    """Whether the card is drawn this frame; the torque bar steps aside for it."""
    placed = state.customization['layouts']['large'].get(NOW_PLAYING_WIDGET)
    return state.alert.size == 'none' and card_layout(placed, self.showing(), getattr(state, 'viewport_width', 0)) is not None

  def _cover_art(self):
    """(background texture, accent color); either is None without usable art. One card-sized texture per cover."""
    url, data = self.artwork
    now = self.ticks()
    if url or self._cover is None or self._cover[1] is None:
      self._bare_since = None
    else:
      # Between songs the feed has no cover for a moment; keep the old one up rather than flash the plain card.
      self._bare_since = now if self._bare_since is None else self._bare_since
      if now - self._bare_since < COVER_HOLD:
        return self._cover[1:]
    if self._cover is not None and self._cover[0] == url:
      return self._cover[1:]
    if url and self._cover is not None and self._cover[1] is not None:
      self._drop_old_cover()
      self._old_cover = (self._cover[1], now)  # fades out under the new cover, then is freed
      self._cover = None
    self._drop_cover()
    texture = accent = None
    if url and data and (kind := image_type(data)):
      image = rl.load_image_from_memory(kind, data, len(data))
      if image.width > 0 and image.height > 0:
        # The middle band of the square cover, at the card's shape, so the picture is not squashed.
        # Whole pixels only: raylib 5.5 corrupted the heap cropping 680x580 to a band starting at y 149.75 (dev VM).
        band = min(image.height, round(image.width * NOW_PLAYING_SIZE[1] / NOW_PLAYING_SIZE[0]))
        rl.image_crop(image, rl.Rectangle(0, (image.height - band) // 2, image.width, band))
        rl.image_resize(image, *NOW_PLAYING_SIZE)  # 660 KB of texture, freed with the song
        colors = rl.load_image_colors(image)
        accent = accent_from(colors, NOW_PLAYING_SIZE[0] * NOW_PLAYING_SIZE[1])
        rl.unload_image_colors(colors)
        texture = rl.load_texture_from_image(image)
        rl.set_texture_filter(texture, rl.TextureFilter.TEXTURE_FILTER_BILINEAR)
      rl.unload_image(image)
    self._cover = (url, texture, accent)
    return texture, accent

  def close(self):
    self._drop_cover()
    self._drop_old_cover()
    self.text.close()

  def _drop_cover(self):
    if self._cover is not None and self._cover[1] is not None and rl.is_window_ready():
      rl.unload_texture(self._cover[1])
    self._cover = None

  def _drop_old_cover(self):
    if self._old_cover is not None and rl.is_window_ready():
      rl.unload_texture(self._old_cover[0])
    self._old_cover = None

  def _scrolling(self, text, line_width, draw, x, y, width, height):
    """Run ``draw(left)`` once when the line fits ``width``; otherwise scroll it left inside a clip, then repeat."""
    if line_width <= width:
      draw(x)
      return
    now = self.ticks()
    if self._scroll is None or self._scroll[0] != text:
      self._scroll = (text, now)
    offset = scroll_offset(line_width, width, now - self._scroll[1])
    clip.begin_scissor_mode(x, y, width, height)
    try:
      for start in (x - offset, x - offset + line_width + SCROLL_GAP):
        if start < x + width:
          draw(start)
    finally:
      clip.end_scissor_mode()

  def _covers(self, rect, texture, opacity):
    """The cover behind the card, the last song's cover fading out under a new one."""
    if self._old_cover is None:
      self._background(rect, texture, opacity)
      return
    fade = _ease((self.ticks() - self._old_cover[1]) / COVER_FADE)
    if fade >= 1:
      self._drop_old_cover()
      self._background(rect, texture, opacity)
      return
    new = opacity * fade
    self._background(rect, self._old_cover[0], opacity * (1 - fade) / (1 - new))  # together they stay at opacity
    self._background(rect, texture, new)

  def _background(self, rect, texture, opacity):
    if texture is None:
      rl.draw_rectangle_rounded(rect, 2 * CORNER / min(rect.width, rect.height), 24, rl.Color(15, 13, 23, round(235 * opacity)))
      return
    tint = rl.Color(140, 140, 140, round(255 * opacity))  # darkened so white text reads on any cover
    sx, sy = texture.width / rect.width, texture.height / rect.height
    for y, height, inset in pill_rows(rect.height, CORNER):
      rl.draw_texture_pro(texture, rl.Rectangle(inset * sx, y * sy, (rect.width - 2 * inset) * sx, height * sy),
                          rl.Rectangle(rect.x + inset, rect.y + y, rect.width - 2 * inset, height), rl.Vector2(0, 0), 0, tint)

  def _title_line(self, rect, track):
    """The "title • artist" line, centered; a line too wide scrolls, like the phone. A new song's line fades in."""
    title, artist = nfc(track['title']), nfc(track['artist'] or track['album'] or '')
    now = self.ticks()
    if self._line is None or self._line[0] != (title, artist):
      self._line = ((title, artist), now if self._line is not None else now - TITLE_FADE)  # the first song is just there
    fade = _ease((now - self._line[1]) / TITLE_FADE)
    color = _faded(TEXT, fade)
    size, dot = 28, 34
    title_width = self.text.measure(title, FontRole.SEMI_BOLD, size, 'title').width
    artist_width = self.text.measure(artist, FontRole.SEMI_BOLD, size, 'artist').width if artist else 0
    line_width = title_width + (dot + artist_width if artist else 0)
    width = rect.width - 2 * PAD
    top = rect.y + LINE_Y - 18
    rise = TITLE_SLIDE * (1 - fade)

    def draw(x):
      self.text.draw(title, FontRole.SEMI_BOLD, size, x, top + rise, color, 'title')
      if artist:
        rl.draw_circle_v(rl.Vector2(x + title_width + dot / 2, rect.y + LINE_Y + rise), 3.5, color)
        self.text.draw(artist, FontRole.SEMI_BOLD, size, x + title_width + dot, top + rise, color, 'artist')

    left = rect.x + PAD if line_width > width else rect.x + rect.width / 2 - line_width / 2
    self._scrolling(f'{title}\n{artist}', line_width, draw, left, top - 4, width, 44)

  def _seek_bar(self, rect, track, accent):
    left, right, y = rect.x + 50, rect.x + rect.width - 50, rect.y + BAR_Y
    rest = rl.Color(255, 255, 255, 60)
    duration = track.get('durationMs')
    if not duration:
      rl.draw_rectangle_rounded(rl.Rectangle(left, y - 4, right - left, 8), 1.0, 6, rest)
      return
    elapsed = min(duration, elapsed_ms(track, self.clock() * 1000) if track.get('position') else 0)
    thumb = left + (right - left) * elapsed / duration
    if thumb - left > 4:
      if track['status'] == 'playing':
        _wave(left, thumb, y - 4, accent, self.ticks() * 2)
      rl.draw_rectangle_rounded(rl.Rectangle(left, y - 4, thumb - left, 8), 1.0, 6, accent)
    if right - thumb > 24:
      rl.draw_rectangle_rounded(rl.Rectangle(thumb + 22, y - 4, right - thumb - 22, 8), 1.0, 6, rest)
    rl.draw_circle_v(rl.Vector2(thumb, y), 22, rl.Color(accent.r, accent.g, accent.b, 45))
    rl.draw_circle_v(rl.Vector2(thumb, y), 15, TEXT)
    rl.draw_circle_v(rl.Vector2(thumb, y), 12, accent)
    total = clock_text(duration)
    self.fonts.draw(clock_text(elapsed), FontRole.NORMAL, 18, left, rect.y + TIMES_Y, MUTED)
    self.fonts.draw(total, FontRole.NORMAL, 18, right - self.fonts.measure(total, FontRole.NORMAL, 18).width, rect.y + TIMES_Y, MUTED)

  def render(self, state):
    placed = state.customization['layouts']['large'].get(NOW_PLAYING_WIDGET)
    now = self.ticks()
    if self.track:
      # No held song behind the "Play on car" pill: it would come back for a moment if the feed then went quiet.
      self._shown = (self.track, now) if self.track['status'] != ELSEWHERE else None
    track = self.showing()
    if state.alert.size != 'none' or (shape := card_layout(placed, track, getattr(state, 'viewport_width', 0))) is None:
      return
    rect, buttons = shape
    opacity = placed.get('opacity', 85) / 100
    if 'play_here' in buttons:
      self._play_here(rect, opacity, now, track)
      return
    if len(buttons) == 1:
      center = _center(rect)
      rl.draw_circle_v(center, rect.width / 2, rl.Color(15, 13, 23, round(235 * opacity)))
      _play_pause(center, False, rect.width * .36, self._cover_art()[1] or ACCENT, INK, now if self._busy(track, now) else None)
      return
    texture, accent = self._cover_art()
    accent = accent or ACCENT
    self._covers(rect, texture, opacity)
    label = nfc(track.get('device') or ('Phone' if track.get('source') == 'phone' else 'Spotify'))
    self.text.draw(label, FontRole.SEMI_BOLD, 20, rect.x + PAD, rect.y + 20, MUTED, 'device')
    if track.get('source') == 'spotify':
      _spotify(rl.Vector2(rect.x + rect.width - PAD - 10, rect.y + 32), 13)  # clear of the top edge and the corner
    self._title_line(rect, track)
    self._seek_bar(rect, track, accent)
    self._press_circle(buttons, now)
    _shuffle(_center(buttons['shuffle']), track.get('shuffle'), track.get('smartShuffle') is True)
    _skip(_center(buttons['previous']), False)
    _play_pause(_center(buttons['play_pause']), track['status'] == 'playing', spin=now if self._busy(track, now) else None)
    _skip(_center(buttons['next']), True)
    _repeat(_center(buttons['repeat']), track.get('repeat'))

  def _play_here(self, rect, opacity, now, track):
    rl.draw_rectangle_rounded(rect, 1.0, 24, rl.Color(15, 13, 23, round(235 * opacity)))
    if (pressed := self._press_alpha('play_here', now)) > 0:
      rl.draw_rectangle_rounded(rect, 1.0, 24, rl.Color(255, 255, 255, round(50 * pressed)))
    mark = rl.Vector2(rect.x + 30 + 22, rect.y + rect.height / 2)
    if self._busy(track, now):
      start = now * 360 % 360
      rl.draw_ring(mark, 16, 21, start, start + 270, 24, TEXT)  # until the song shows up here
    else:
      _spotify(mark, 22)
    self.text.draw('Play on car', FontRole.SEMI_BOLD, 34, mark.x + 22 + 20, rect.y + rect.height / 2 - 20, TEXT, 'play_here')

  def _press_alpha(self, name, now):
    """How strongly the tapped ``name`` is lit: 1 while held, fading once the finger lifts, else 0."""
    if self._press is None or self._press[0] != name:
      return 0.0
    down, up = self._press[1:]
    if up is None:
      return 1.0 if now - down < PRESS_MAX else 0.0
    return max(0.0, 1 - (now - up) / PRESS_FADE)

  def _busy(self, track, now):
    """Spin the play button while the card holds a song the feed lost, or until the player answers a tap."""
    if track is not self.track:
      return True
    if self._pending is not None and (_song_key(track) != self._pending[0] or now - self._pending[1] >= PENDING):
      self._pending = None
    return self._pending is not None

  def _press_circle(self, buttons, now):
    """A gray circle opens behind a pressed button and fades once the finger lifts, like the phone's card."""
    if self._press is None:
      return
    name, down, up = self._press
    alpha = 1.0 if up is None else 1 - (now - up) / PRESS_FADE
    if alpha <= 0 or name not in buttons or (up is None and now - down >= PRESS_MAX):
      self._press = None if alpha <= 0 or up is None else self._press
      return
    radius = BUTTON_R + 10 * _ease((now - down) / PRESS_GROW)
    rl.draw_circle_v(_center(buttons[name]), radius, rl.Color(255, 255, 255, round(70 * alpha)))


def _song_key(track):
  """What changes when the player answers a skip or play/pause: the song, play state or position anchor."""
  return track.get('title'), track.get('artist'), track.get('status'), track.get('position')


def _center(box: rl.Rectangle) -> rl.Vector2:
  return rl.Vector2(box.x + box.width / 2, box.y + box.height / 2)


class NowPlayingTouch:
  """Card taps: a button fires when the finger lifts on the button it went down on. Taps on the card never fall through."""
  def __init__(self, send, feedback=None):
    self.send = send
    self.feedback = feedback  # the card: press(button) and release(button sent or None) animate it
    self.pressed = None

  def cancel(self):
    self._lift(None)

  def _lift(self, fired):
    if self.pressed is not None and self.feedback is not None:
      self.feedback.release(fired)
    self.pressed = None

  def touch(self, kind, x, y, state, placed, track) -> bool:
    """True while the card owns the touch."""
    shape = None if kind == 'cancel' or state is None or state.alert.size != 'none' else card_layout(placed, track, state.viewport_width)
    if shape is None:
      claimed = self.pressed is not None
      self._lift(None)
      return claimed
    point = rl.Vector2(x, y)
    hit = next((name for name, box in shape[1].items() if rl.check_collision_point_rec(point, box)), None)
    if kind == 'down':
      self.pressed = hit
      if hit is not None and self.feedback is not None:
        self.feedback.press(hit)
      return rl.check_collision_point_rec(point, shape[0])
    if kind == 'up':
      pressed, fired = self.pressed, None
      if pressed is not None and pressed == hit:
        if pressed == 'shuffle':
          if track.get('shuffle') is not None:
            self.send('shuffle', {'enabled': not track['shuffle']})
        elif pressed == 'repeat':
          if track.get('repeat') in REPEAT_NEXT:
            self.send('repeat', {'mode': REPEAT_NEXT[track['repeat']]})
        elif pressed == 'play_here':
          self.send('activate', {})  # Spotify Connect: this car becomes the playing device again
          fired = pressed
        else:
          self.send(pressed, {})
          fired = pressed
      self._lift(fired)
      return pressed is not None or rl.check_collision_point_rec(point, shape[0])
    return self.pressed is not None
