"""Car-screen look for Android Auto: set-speed/limit card, atom badge, edge glow, orange road."""

import math
import time
from dataclasses import replace

import pyray as rl

from openpilot.starpilot.system.android_auto.projection_layout import SPEED_CARD_SIZE
from openpilot.starpilot.ui.onroad_customization import placement, rgba
from openpilot.starpilot.ui.onroad_state import AlertSize, slc_controls
from openpilot.starpilot.ui.onroad_large_widgets import wheel_box
from openpilot.starpilot.ui.presentation import FontRole, Profile
from openpilot.starpilot.ui.unified_speed_presentation import resolve_unified_speed

ORANGE = rl.Color(255, 140, 30, 255)
RED = rl.Color(235, 40, 50, 255)
YELLOW = rl.Color(255, 205, 0, 255)
LEAD_YELLOW = rl.Color(255, 196, 36, 255)
BLUE = rl.Color(40, 130, 255, 255)
GREEN = rl.Color(40, 200, 90, 255)
LABEL = rl.Color(176, 168, 255, 255)
CARD_FILL = rl.Color(18, 20, 31, 235)
CARD_WIDTH = SPEED_CARD_SIZE[0]
CARD_PAD = 28
BUTTON_HEIGHT = 72
BUTTON_GAP = 12
FEEDBACK_HEIGHT = 40
BUTTON_FILL = rl.Color(255, 255, 255, 30)
BUTTON_PRESSED = rl.Color(255, 255, 255, 70)
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


def _card_body(state):
  """(x, y, has_max, has_limit, height before any buttons) of the card, or None while it is hidden."""
  shown = resolve_unified_speed(state)
  if shown.mode == 'hidden':
    return None
  saved = placement(state.customization, 'large', 'cruise_limits')
  has_limit = shown.mode in ('split', 'limit_only')
  has_max = shown.mode in ('split', 'max_only')
  return saved['x'], saved['y'], has_max, has_limit, 20 + (190 if has_max else 0) + (230 if has_limit else 0) + 20


def card_height(state):
  """The card's full drawn height, buttons and feedback line included; 0 while it is hidden."""
  body = _card_body(state)
  if body is None:
    return 0
  height = body[4]
  if card_controls(Profile.LARGE, state):
    height += BUTTON_HEIGHT + 20
  if state.alert.size == AlertSize.NONE and state.speed_limit.action_feedback:
    height += FEEDBACK_HEIGHT
  return height


def card_controls(profile, state):
  """The speed-limit buttons (Use limit, Accept / Decline) as a row at the bottom of the card, not loose over the
  camera. Same controls and requests as the comma screen; only where they sit changes."""
  controls = slc_controls(profile, state)
  body = _card_body(state)
  if not controls or body is None:
    return controls
  x, y, _, _, height = body
  left, top = x + CARD_PAD, y + height - 4
  width = (CARD_WIDTH - 2 * CARD_PAD - BUTTON_GAP * (len(controls) - 1)) / len(controls)
  return tuple(replace(control, bounds=(round(left + i * (width + BUTTON_GAP)), top,
                                        round(left + i * (width + BUTTON_GAP) + width), top + BUTTON_HEIGHT))
               for i, control in enumerate(controls))


class SpeedLimitCard:
  """Replaces the shared cruise/limit card on the car screen; same placement key, own look."""

  def __init__(self, fonts, outline_color=None, pressed=lambda: None):
    self.fonts = fonts
    self.outline_color = outline_color
    self.pressed = pressed

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
    body = _card_body(state)
    if body is None:
      return
    shown = resolve_unified_speed(state)
    x, y, has_max, has_limit, height = body
    controls = card_controls(Profile.LARGE, state)
    feedback = state.speed_limit.action_feedback if state.alert.size == AlertSize.NONE else None
    height = card_height(state)
    rect = rl.Rectangle(x, y, CARD_WIDTH, height)
    # The layout's Background opacity arrives as the card fill's alpha (fully opaque by default).
    fill_alpha = rgba(state.customization, 'cardFill', 'large', 'cruise_limits')[3]
    rl.draw_rectangle_rounded(rect, .08, 12, rl.Color(CARD_FILL.r, CARD_FILL.g, CARD_FILL.b, round(CARD_FILL.a * fill_alpha / 255)))
    base = (self.outline_color(state) if self.outline_color else ORANGE) or GRAY
    rl.draw_rectangle_rounded_lines_ex(rect, .08, 12, 3, rl.Color(base.r, base.g, base.b, 150))
    pad = CARD_PAD
    top = y + 20
    if has_max:
      self._block(x + pad, top, 'MAX SET', shown.max_text, shown.unit, shown.status if shown.max_text != '–' else 'Not set')
      top += 190
    if has_limit:
      relation = limit_relation(shown.max_text, shown.limit_text) or shown.status
      self._block(x + pad, top, 'SLC LIMIT', shown.limit_text, shown.unit, relation)
      self.fonts.draw(f'Road limit {shown.posted_text}', FontRole.NORMAL, 28, x + pad, top + 176, rl.WHITE)
    for control in controls:
      left, top, right, bottom = control.bounds
      button = rl.Rectangle(left, top, right - left, bottom - top)
      fill = BUTTON_PRESSED if self.pressed() == control.request else BUTTON_FILL
      rl.draw_rectangle_rounded(button, 1, 12, fill)
      width = self.fonts.measure(control.label, FontRole.SEMI_BOLD, 30).width
      a, b = self.fonts.vertical_ink('Accept Decline', FontRole.SEMI_BOLD, 30)  # one baseline for both buttons
      self.fonts.draw(control.label, FontRole.SEMI_BOLD, 30, (left + right - width) / 2, (top + bottom) / 2 - (b - a) / 2 - a, rl.WHITE)
    if feedback:
      self.fonts.draw(feedback, FontRole.NORMAL, 26, x + pad, y + height - FEEDBACK_HEIGHT - 8, LABEL)


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


GRAY = rl.Color(145, 155, 149, 255)


def glow_color(mode_label):
  if mode_label == 'Chill Mode':
    return GREEN
  return ORANGE if mode_label == 'Experimental Mode' else BLUE


def state_glow(state, mode_label):
  """Mirror the comma screen's engagement: none when off, gray on override, blue lateral-only, mode color with long."""
  if getattr(state, 'driver_override_active', False) or getattr(state, 'longitudinal_overridden', False):
    return GRAY
  if getattr(state, 'longitudinal_active', False):
    return glow_color(mode_label)
  return BLUE if getattr(state, 'lateral_active', False) else None


def edge_glow(color, x, y, width, height, span=GLOW_SPAN, alpha=150):
  if color is None:
    return
  edge = rl.Color(color.r, color.g, color.b, alpha)
  fade = rl.Color(color.r, color.g, color.b, 0)
  x, y, width, height = int(x), int(y), int(width), int(height)
  rl.draw_rectangle_gradient_v(x, y, width, span, edge, fade)
  rl.draw_rectangle_gradient_v(x, y + height - span, width, span, fade, edge)
  rl.draw_rectangle_gradient_h(x, y, span, height, edge, fade)
  rl.draw_rectangle_gradient_h(x + width - span, y, span, height, fade, edge)


def side_colors(state, vasm_left=False, vasm_right=False):
  """(left, right) warning colors: red for a blind spot (car or V-ASM), yellow for a turn signal, None when quiet."""
  signals = getattr(state, 'border_signals', None)
  if signals is None or not state.camera_available or state.alert.critical:
    return None, None
  def pick(blindspot, blinker):
    return RED if blindspot else YELLOW if blinker else None
  return (pick(signals.left_blindspot or vasm_left, signals.left_blinker),
          pick(signals.right_blindspot or vasm_right, signals.right_blinker))


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


def overlay(fonts, content, state, mode_label, lead_present, vasm=(False, False)):
  """Everything the car screen draws over the onroad view: mode glow, LEAD badge and side warnings."""
  edge_glow(state_glow(state, mode_label), content.x, content.y, content.width, content.height)
  appearance = getattr(state, 'appearance', None)
  if appearance is not None and appearance.show_lead_indicator:
    lead_badge(fonts, content, lead_present)
  left, right = side_colors(state, *vasm)
  side_glow(left, content.x, content.y, content.width, content.height, True)
  side_glow(right, content.x, content.y, content.width, content.height, False)


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
