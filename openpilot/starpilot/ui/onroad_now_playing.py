"""The Android Auto now-playing card; both live rendering and previews use this drawing."""
import time

import pyray as rl

from openpilot.starpilot.soloist.now_playing import elapsed_ms
from openpilot.starpilot.system.android_auto.projection_layout import NOW_PLAYING_SIZE, NOW_PLAYING_WIDGET
from openpilot.starpilot.ui.presentation import FontRole

ACCENT = rl.Color(199, 174, 247, 255)
INK = rl.Color(15, 13, 23, 255)
TEXT = rl.Color(245, 242, 252, 255)
MUTED = rl.Color(184, 178, 200, 255)
DIM = rl.Color(184, 178, 200, 90)
BUTTON_R = 30
CORNER = 28
CONTROLS = ('shuffle', 'previous', 'play_pause', 'next')


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


class NowPlayingWidget:
  def __init__(self, fonts, clock=time.time):  # noqa: TID251 epoch-ms track anchors
    self.fonts = fonts
    self.clock = clock
    self.track = None

  def _fit(self, text, role, size, width):
    measured = self.fonts.measure(text, role, size)
    return size * width / measured.width if measured.width > width else size

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
    left, width = rect.x + CORNER, rect.width - 2 * CORNER
    rows = ((track['title'], FontRole.SEMI_BOLD, 30, 10, TEXT), (track['artist'] or track['album'], FontRole.NORMAL, 22, 44, MUTED))
    for text, role, size, y, color in rows:
      if text:
        self.fonts.draw(text, role, self._fit(text, role, size, width), left, rect.y + y, color)
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
