"""Car-screen look for Android Auto: set-speed/limit card, atom badge, edge glow, orange road."""

import math
import time

import pyray as rl

from openpilot.starpilot.ui.onroad_customization import placement
from openpilot.starpilot.ui.onroad_large_widgets import wheel_box
from openpilot.starpilot.ui.presentation import FontRole
from openpilot.starpilot.ui.unified_speed_presentation import resolve_unified_speed

ORANGE = rl.Color(255, 140, 30, 255)
RED = rl.Color(235, 40, 50, 255)
YELLOW = rl.Color(255, 205, 0, 255)
LEAD_YELLOW = rl.Color(255, 196, 36, 255)
BLUE = rl.Color(40, 130, 255, 255)
GREEN = rl.Color(40, 200, 90, 255)
LABEL = rl.Color(176, 168, 255, 255)
CARD_FILL = rl.Color(18, 20, 31, 235)
CARD_WIDTH = 346
GLOW_SPAN = 64

ROAD_STYLE = {'pathMode': 'color', 'path': '#FF8A1FD0', 'pathEdge': '#FF9A2EFF', 'laneLines': '#FF9A2EFF'}


def road_style(saved):
  return {**ROAD_STYLE, **saved}


def limit_relation(max_text, limit_text):
  try:
    maximum, limit = int(max_text), int(limit_text)
  except ValueError:
    return None
  return 'Matches set speed' if maximum == limit else 'Below set speed' if limit < maximum else 'Above set speed'


class SpeedLimitCard:
  """Replaces the shared cruise/limit card on the car screen; same placement key, own look."""

  def __init__(self, fonts, outline_color=None):
    self.fonts = fonts
    self.outline_color = outline_color

  def close(self):
    pass

  def _block(self, x, y, label, number, unit, status, size=104):
    fonts = self.fonts
    fonts.draw(label, FontRole.MEDIUM, 28, x, y, LABEL)
    fonts.draw(number, FontRole.SET_SPEED, size, x, y + 30, rl.WHITE)
    width = fonts.measure(number, FontRole.SET_SPEED, size).width
    fonts.draw(unit, FontRole.NORMAL, 26, x + width + 10, y + 30 + size - 38, rl.Color(230, 230, 240, 220))
    fonts.draw(status, FontRole.NORMAL, 28, x, y + size + 40, LABEL)

  def render(self, content, state):
    shown = resolve_unified_speed(state)
    if shown.mode == 'hidden':
      return
    saved = placement(state.customization, 'large', 'cruise_limits')
    x, y = saved['x'], saved['y']
    has_limit = shown.mode in ('split', 'limit_only')
    has_max = shown.mode in ('split', 'max_only')
    height = (20 + (190 if has_max else 0) + (230 if has_limit else 0) + 20)
    rect = rl.Rectangle(x, y, CARD_WIDTH, height)
    rl.draw_rectangle_rounded(rect, .08, 12, CARD_FILL)
    base = self.outline_color(state) if self.outline_color else ORANGE
    rl.draw_rectangle_rounded_lines_ex(rect, .08, 12, 3, rl.Color(base.r, base.g, base.b, 150))
    pad = 28
    top = y + 20
    if has_max:
      self._block(x + pad, top, 'MAX SET', shown.max_text, shown.unit, shown.status if shown.max_text != '–' else 'Not set')
      top += 190
    if has_limit:
      relation = limit_relation(shown.max_text, shown.limit_text) or shown.status
      self._block(x + pad, top, 'SLC LIMIT', shown.limit_text, shown.unit, relation)
      self.fonts.draw(f'Road limit {shown.posted_text}', FontRole.NORMAL, 28, x + pad, top + 176, rl.WHITE)


class AtomBadge:
  """The comma's wheel badge position, drawn as an orange atom."""

  _texture = None

  def prepare(self):
    pass

  def close(self):
    pass

  @staticmethod
  def _ellipse(cx, cy, a, b, angle, color, thickness):
    cos, sin = math.cos(angle), math.sin(angle)
    points = []
    for step in range(49):
      t = step / 48 * 2 * math.pi
      px, py = a * math.cos(t), b * math.sin(t)
      points.append(rl.Vector2(cx + px * cos - py * sin, cy + px * sin + py * cos))
    for first, second in zip(points, points[1:], strict=False):
      rl.draw_line_ex(first, second, thickness, color)

  def render(self, content, state):
    box = wheel_box(content, state)
    cx, cy, radius = box.x + box.width / 2, box.y + box.height / 2, box.width / 2
    rl.draw_circle(int(cx), int(cy), radius, rl.Color(20, 20, 28, 170))
    for fraction, alpha in ((.97, 90), (.85, 55)):
      rl.draw_ring(rl.Vector2(cx, cy), radius * fraction - 1.5, radius * fraction, 0, 360, 64, rl.Color(232, 90, 40, alpha))
    color = rl.Color(232, 90, 40, 255)
    for degrees in (0, 60, 120):
      self._ellipse(cx, cy, radius * .62, radius * .24, math.radians(degrees - 30), color, max(4., radius * .1))
    rl.draw_circle(int(cx), int(cy), radius * .12, color)


def glow_color(mode_label):
  if mode_label == 'Chill Mode':
    return GREEN
  return ORANGE if mode_label == 'Experimental Mode' else BLUE


def edge_glow(color, x, y, width, height, span=GLOW_SPAN, alpha=150):
  edge = rl.Color(color.r, color.g, color.b, alpha)
  fade = rl.Color(color.r, color.g, color.b, 0)
  x, y, width, height = int(x), int(y), int(width), int(height)
  rl.draw_rectangle_gradient_v(x, y, width, span, edge, fade)
  rl.draw_rectangle_gradient_v(x, y + height - span, width, span, fade, edge)
  rl.draw_rectangle_gradient_h(x, y, span, height, edge, fade)
  rl.draw_rectangle_gradient_h(x + width - span, y, span, height, fade, edge)


def side_colors(state):
  """(left, right) warning colors: red for a blind spot, yellow for a turn signal, None when quiet."""
  signals = getattr(state, 'border_signals', None)
  if signals is None or not state.camera_available or state.alert.critical:
    return None, None
  def pick(blindspot, blinker):
    return RED if blindspot else YELLOW if blinker else None
  return pick(signals.left_blindspot, signals.left_blinker), pick(signals.right_blindspot, signals.right_blinker)


def side_glow(color, x, y, width, height, left, now=None):
  if color is None:
    return
  now = time.monotonic() if now is None else now
  period = .5 if color is RED else .9
  level = .5 + .5 * math.sin(now / period * 2 * math.pi)
  span = int(GLOW_SPAN * 2.2)
  edge = rl.Color(color.r, color.g, color.b, int(110 + 145 * level))
  fade = rl.Color(color.r, color.g, color.b, 0)
  x, y, width, height = int(x), int(y), int(width), int(height)
  if left:
    rl.draw_rectangle_gradient_h(x, y, span, height, edge, fade)
  else:
    rl.draw_rectangle_gradient_h(x + width - span, y, span, height, fade, edge)


def lead_badge(fonts, content, present):
  """The video's fixed yellow LEAD badge: a road wedge with a car, and the word LEAD under it."""
  if not present:
    return
  unit = content.width * .05
  cx, top = content.x + content.width * .28, content.y + content.height * .12
  half_top, half_bottom, height = unit * .38, unit * .62, unit * .78
  a, b = rl.Vector2(cx - half_top, top), rl.Vector2(cx + half_top, top)
  c, d = rl.Vector2(cx + half_bottom, top + height), rl.Vector2(cx - half_bottom, top + height)
  fill = rl.Color(YELLOW.r, 190, 20, 70)
  rl.draw_triangle(a, d, c, fill)
  rl.draw_triangle(a, c, b, fill)
  stroke = max(3., unit * .06)
  for p, q in ((a, b), (b, c), (c, d), (d, a)):
    rl.draw_line_ex(p, q, stroke, LEAD_YELLOW)
  body = rl.Rectangle(cx - unit * .22, top + height * .52, unit * .44, unit * .22)
  rl.draw_rectangle_rounded(body, .4, 6, LEAD_YELLOW)
  rl.draw_rectangle_rounded(rl.Rectangle(cx - unit * .15, top + height * .30, unit * .30, unit * .2), .5, 6, LEAD_YELLOW)
  size = max(24, int(unit * .56))
  width = fonts.measure('LEAD', FontRole.BOLD, size).width
  fonts.draw('LEAD', FontRole.BOLD, size, cx - width / 2, top + height + unit * .12, LEAD_YELLOW)
