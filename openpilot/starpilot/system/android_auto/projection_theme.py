"""Car-screen look for Android Auto: set-speed/limit card, atom badge, edge glow, orange road."""

import math
import time
import weakref
from dataclasses import dataclass, replace
from pathlib import Path

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


def overlay(fonts, content, state, mode_label, lead_present, vasm=(False, False), lead=None, trend=None,
            personality=None):
  """Everything the car screen draws over the onroad view: mode glow, LEAD badge and side warnings."""
  edge_glow(state_glow(state, mode_label), content.x, content.y, content.width, content.height)
  appearance = getattr(state, 'appearance', None)
  if appearance is not None and appearance.show_lead_indicator:
    lead_badge(fonts, content, lead_present, lead, trend, personality=personality, avoid=card_rect(state))
  left, right = side_colors(state, *vasm)
  side_glow(left, content.x, content.y, content.width, content.height, True)
  side_glow(right, content.x, content.y, content.width, content.height, False)


LEAD_TEAL = rl.Color(70, 220, 160, 255)
LEAD_RED = rl.Color(255, 64, 56, 255)
CAR_BODY = rl.Color(214, 220, 232, 255)
CAR_SHADE = rl.Color(150, 158, 175, 255)
CAR_GLASS = rl.Color(28, 32, 44, 255)
CAR_LIGHT = rl.Color(248, 250, 253, 255)
CAR_LOW = rl.Color(122, 130, 148, 255)
GLASS_TOP = rl.Color(92, 104, 128, 255)
TIRE = rl.Color(14, 16, 20, 255)
TAIL_DIM = rl.Color(170, 30, 36, 255)
STEADY, BRAKING, PULLING_AWAY = 'steady', 'braking', 'pulling_away'


@dataclass(frozen=True)
class LeadView:
  d_rel: float     # m to the lead
  v_rel: float     # m/s, + when the lead pulls away
  a_lead: float    # m/s^2, the radar's filtered lead acceleration
  v_ego: float = 0.  # m/s


class LeadTrend:
  """Braking / pulling away / steady, held long enough that a noisy radar acceleration does not flicker the badge.

  Red needs the lead braking at -1 m/s^2 or harder for 0.4 s and clears above -0.5 m/s^2; teal needs it speeding up
  and the gap opening for 0.6 s. Either clears only after 0.8 s back in the dead band."""
  BRAKE_ON, BRAKE_OFF = -1.0, -.5
  AWAY_ON, AWAY_OFF, AWAY_GAP = .6, .25, .5
  ENTER = {BRAKING: .4, PULLING_AWAY: .6}
  LEAVE = .8

  def __init__(self, clock=time.monotonic):
    self.clock = clock
    self.state = STEADY
    self._candidate, self._since = STEADY, None

  def reset(self):
    self.state, self._candidate, self._since = STEADY, STEADY, None

  def update(self, lead: LeadView | None) -> str:
    if lead is None or not all(math.isfinite(v) for v in (lead.a_lead, lead.v_rel)):
      self.reset()
      return self.state
    now = self.clock()
    braking = lead.a_lead <= (self.BRAKE_OFF if self.state == BRAKING else self.BRAKE_ON)
    away = (lead.a_lead >= (self.AWAY_OFF if self.state == PULLING_AWAY else self.AWAY_ON) and
            lead.v_rel >= (0. if self.state == PULLING_AWAY else self.AWAY_GAP))
    wanted = BRAKING if braking else PULLING_AWAY if away else STEADY
    if wanted == self.state:
      self._candidate, self._since = wanted, None
      return self.state
    if wanted != self._candidate or self._since is None:
      self._candidate, self._since = wanted, now
    # braking replaces pulling away (or steady) as soon as it qualifies; anything else waits out LEAVE first
    wait = self.ENTER[wanted] if wanted != STEADY else self.LEAVE
    if self.state != STEADY and wanted != BRAKING:
      wait = max(wait, self.LEAVE)
    if now - self._since >= wait:
      self.state, self._since = wanted, None
    return self.state


def gap_bars(personality) -> int:
  """1-3 bars under the car for the following-distance setting, like the Honda cluster's own distance bars:
  aggressive 1, standard 2, relaxed 3. They used to show the time gap to the lead, which at low speed stays near
  2 s even for a close car, so the bars sat at 3 (or 2 when stopped) however close the lead was (Peter, 2026-10-29)."""
  # ui_state holds the capnp ordinal (aggressive 0, standard 1, relaxed 2); a reader may also hold the name
  name = {0: 'aggressive', 1: 'standard', 2: 'relaxed'}.get(personality, str(personality)) if isinstance(personality, int) \
    else str(personality)
  return 1 if name == 'aggressive' else 3 if name == 'relaxed' else 2


def _mix(a, b, t):
  return rl.Color(int(a.r + (b.r - a.r) * t), int(a.g + (b.g - a.g) * t), int(a.b + (b.b - a.b) * t), int(a.a + (b.a - a.a) * t))


def _fade(color, alpha):
  return rl.Color(color.r, color.g, color.b, int(alpha))


def _rounded(points, radius, segments=5):
  """Convex polygon with each corner filleted; radius is one value or one per corner."""
  out = []
  for i, (px, py) in enumerate(points):
    r = radius[i] if isinstance(radius, (list, tuple)) else radius
    ends = []
    for qx, qy in (points[i - 1], points[(i + 1) % len(points)]):
      d = math.hypot(qx - px, qy - py) or 1.
      t = min(r, d / 2) / d
      ends.append((px + (qx - px) * t, py + (qy - py) * t))
    (sx, sy), (ex, ey) = ends
    for k in range(segments + 1):
      t = k / segments
      out.append(((1 - t) ** 2 * sx + 2 * t * (1 - t) * px + t * t * ex, (1 - t) ** 2 * sy + 2 * t * (1 - t) * py + t * t * ey))
  return out


def _shade(*stops):
  """Vertical gradient through (y, color) stops."""
  def at(y):
    if y <= stops[0][0]:
      return stops[0][1]
    for (y0, c0), (y1, c1) in zip(stops, stops[1:], strict=False):
      if y <= y1:
        return _mix(c0, c1, (y - y0) / max(y1 - y0, 1e-6))
    return stops[-1][1]
  return at


def _fill(points, color_at):
  """Convex polygon (clockwise on screen) as a triangle fan with a per-vertex vertical gradient."""
  mx, my = sum(x for x, _ in points) / len(points), sum(y for _, y in points) / len(points)
  rl.rl_begin(rl.RL_TRIANGLES)
  for i in range(len(points)):
    for x, y in ((mx, my), points[(i + 1) % len(points)], points[i]):
      c = color_at(y)
      rl.rl_color4ub(c.r, c.g, c.b, c.a)
      rl.rl_vertex2f(x, y)
  rl.rl_end()


def _glow(x: float, y: float, radius: float, inner, outer) -> None:
  # The comma's raylib takes a Vector2 centre, desktop raylib takes ints; the wrong one
  # raises TypeError and kills the car view, leaving the head unit on the starting frame.
  if len(rl.ffi.typeof(rl.rl.DrawCircleGradient).args) == 4:
    rl.draw_circle_gradient(rl.Vector2(x, y), radius, inner, outer)
  else:
    rl.draw_circle_gradient(int(x), int(y), radius, inner, outer)


# Rear view of Peter's Crystal Black Civic Sport, rendered from the 3D model in tools/head_unit/civic
# (render_lead_sprites.py): tail lamps dim, and the same frame with the brake lamps lit.
LEAD_SPRITE_DIR = Path(__file__).parents[2] / 'ui/assets'
LEAD_SPRITE, LEAD_SPRITE_BRAKE = 'lead_civic.png', 'lead_civic_brake.png'
SPRITE_CAR_WIDTH = .898        # share of the sprite's width that is the car (the rest is rim light and shadow)
SPRITE_GROUND = .814           # tyre contact, as a share of the sprite's height
SPRITE_LAMPS = ((.152, .457), (.844, .457))
BAR_FILL = rl.Color(236, 240, 246, 230)
SPRITE_TINT = rl.Color(255, 255, 255, 255)  # rl.WHITE is a tuple in some pyray builds; _fade needs a Color

# Following-distance road, in depth units from the near edge (1) away from us; on screen everything scales by 1/z.
GAP_STEP = .45                 # one bar plus the space after it
BAR_DEPTH = GAP_STEP * .4
CAR_CLEAR = .06                # space between the last bar and the car's tyres
ROAD_FAR = 1 + 3 * GAP_STEP + .25
CARD_CLEARANCE = 24            # px kept between the badge's road and the speed card (2026-10-29 photo: the near bar
                               # and left lane line ran into the card's right edge on the 800x480 car screen)

# Sprite textures live in the graphics context. The fonts are made per context and closed before it is destroyed, so
# a cache keyed on them never hands an old texture to a newer context (Android Auto reconnects make a new one).
_sprites = weakref.WeakKeyDictionary()
SPRITE_CACHE = 6


def _load_sprite(path, width):
  """The sprite shrunk once on the CPU to the width it is drawn at; None if it cannot be loaded."""
  if not path.is_file():
    return None
  image = rl.load_image(str(path))
  try:
    if image.data == rl.ffi.NULL or image.width <= 0 or image.height <= 0:
      return None
    rl.image_resize(image, width, max(1, round(image.height * width / image.width)))
    texture = rl.load_texture_from_image(image)
  finally:
    if image.data != rl.ffi.NULL:
      rl.unload_image(image)
  if texture.id == 0:
    return None
  rl.set_texture_filter(texture, rl.TextureFilter.TEXTURE_FILTER_BILINEAR)
  return texture


def lead_sprite(fonts, name, width):
  """Texture of `name` exactly `width` px wide for this graphics context, or None to draw the vector car instead."""
  try:
    cache = _sprites.setdefault(fonts, {})
    key = (name, int(width))
    if key not in cache:
      if len(cache) >= SPRITE_CACHE:  # the content size changed: drop the oldest size
        old = cache.pop(next(iter(cache)))
        if old is not None:
          rl.unload_texture(old)
      cache[key] = None  # a load that raises is not retried every frame
      if key[1] > 0:
        cache[key] = _load_sprite(LEAD_SPRITE_DIR / name, key[1])
    return cache[key]
  except Exception:
    return None


def card_rect(state):
  """(left, top, right, bottom) of the speed card as drawn, buttons included, or None while it is hidden."""
  body = _card_body(state)
  if body is None or not placement(state.customization, 'large', 'cruise_limits').get('enabled', True):
    return None
  x, y = body[0], body[1]
  return x, y, x + CARD_WIDTH, y + card_height(state)


def badge_geometry(content, personality, avoid=None):
  """Where the lead badge draws, like the Honda cluster's distance display: the road runs away from us and the lead
  car sits just past the last bar, so it is close and large at 1 bar and further away and smaller at 3. If its spot
  overlaps `avoid` (the speed card's rect), it slides sideways off the card rather than drawing over it."""
  unit = content.width * .05
  w = unit * 1.25
  cx, top = content.x + content.width * .28, content.y + content.height * .12
  horizon, depth = top - w * .2, w * 2.
  bars = gap_bars(personality)
  lane = w * 1.15

  def y(z):
    return horizon + depth / z

  if avoid is not None:
    # widest at the near edge (lane lines plus half their stroke); the top is the highest car roof over all three
    # settings, so the badge does not hop sideways when the distance setting changes
    half = lane + max(1.5, unit * .085) / 2
    roofs = [y(z) - w * 1.7 / z * 1.1 for z in (1 + n * GAP_STEP + CAR_CLEAR for n in (1, 2, 3))]
    extent_top, extent_bottom = min(roofs), y(1) + unit * .1
    left, card_top, right, card_bottom = avoid
    if left < cx + half and right > cx - half and card_top < extent_bottom and card_bottom > extent_top:
      cx = right + CARD_CLEARANCE + half if (left + right) / 2 <= cx else left - CARD_CLEARANCE - half

  car_z = 1 + bars * GAP_STEP + CAR_CLEAR
  return {
    'cx': cx, 'unit': unit, 'y': y, 'lane': lane, 'car_width': w * 1.7 / car_z, 'car_ground': y(car_z),
    'bars': [(1 + i * GAP_STEP, 1 + i * GAP_STEP + BAR_DEPTH) for i in range(bars)],
  }


def _vector_car(cx, top, w, h, braking, pulse):
  """The drawn car, for when the Civic sprite cannot be loaded."""
  # car seen from behind, lit from above: shadow on the road, tires, cabin with tinted rear glass, mirrors,
  # trunk deck catching the light over a darker rear panel, wrap-around tail lights, plate and bumper
  def at(fx, fy):
    return cx + fx * w, top + fy * h

  def box(x0, y0, x1, y1):
    return [at(x0, y0), at(x1, y0), at(x1, y1), at(x0, y1)]

  rl.draw_ellipse(int(cx), int(top + h * .99), w * .62, h * .09, _fade(TIRE, 70))
  rl.draw_ellipse(int(cx), int(top + h * .985), w * .54, h * .06, _fade(TIRE, 140))
  for side in (-1, 1):
    _fill(_rounded(box(side * .36 - .105, .70, side * .36 + .105, 1.), w * .045),
          _shade((top + h * .7, rl.Color(58, 62, 74, 255)), (top + h, TIRE)))

  belt = top + h * .45
  _fill(_rounded([at(-.27, 0), at(.27, 0), at(.42, .45), at(-.42, .45)], [w * .11, w * .11, w * .02, w * .02]),
        _shade((top, CAR_LIGHT), (belt, CAR_BODY)))
  _fill(_rounded([at(-.19, .11), at(.19, .11), at(.29, .37), at(-.29, .37)], [w * .05, w * .05, w * .02, w * .02]),
        _shade((top + h * .11, GLASS_TOP), (top + h * .37, CAR_GLASS)))
  _fill([at(-.04, .13), at(.03, .13), at(-.13, .355), at(-.20, .355)], lambda y: _fade(CAR_LIGHT, 40))
  if braking:  # third brake light along the top of the rear glass
    _fill(_rounded(box(-.08, .125, .08, .15), w * .012), lambda y: _mix(TAIL_DIM, LEAD_RED, pulse))
  for side in (-1, 1):
    _fill(_rounded([at(side * .40, .33), at(side * .55, .32), at(side * .55, .41), at(side * .41, .42)][::side], w * .03),
          _shade((top + h * .32, CAR_BODY), (top + h * .42, CAR_SHADE)))

  _fill(_rounded([at(-.44, .43), at(.44, .43), at(.5, .53), at(.49, .76), at(-.49, .76), at(-.5, .53)],
                 [w * .06, w * .06, w * .03, w * .06, w * .06, w * .03]),
        _shade((top + h * .43, CAR_LIGHT), (top + h * .51, CAR_BODY), (top + h * .76, CAR_LOW)))
  # crease where the trunk deck turns down into the rear panel: a bright edge over a soft shadow
  rl.draw_line_ex(rl.Vector2(*at(-.47, .525)), rl.Vector2(*at(.47, .525)), max(1., h * .012), _fade(CAR_LIGHT, 230))
  rl.draw_line_ex(rl.Vector2(*at(-.46, .54)), rl.Vector2(*at(.46, .54)), max(1., h * .014), _fade(CAR_LOW, 150))

  tail = _mix(TAIL_DIM, LEAD_RED, pulse) if braking else TAIL_DIM
  for side in (-1, 1):
    lamp = [at(side * .17, .56), at(side * .495, .55), at(side * .48, .655), at(side * .21, .64)][::side]
    if braking:
      lx, ly = at(side * .33, .60)
      _glow(lx, ly, w * .26, _fade(LEAD_RED, 170 * pulse), _fade(LEAD_RED, 0))
    _fill(_rounded(lamp, w * .025), _shade((top + h * .55, _mix(tail, CAR_LIGHT, .25)), (top + h * .65, _mix(tail, TIRE, .25))))
    if braking:
      _fill(_rounded([at(side * .25, .58), at(side * .44, .575), at(side * .43, .615), at(side * .27, .61)][::side], w * .015),
            lambda y: _fade(rl.Color(255, 196, 186, 255), 230 * pulse))

  _fill(_rounded(box(-.14, .585, .14, .705), w * .02), _shade((top + h * .585, CAR_LOW), (top + h * .705, CAR_SHADE)))
  _fill(_rounded(box(-.115, .605, .115, .69), w * .012), _shade((top + h * .605, CAR_LIGHT), (top + h * .69, CAR_BODY)))

  _fill(_rounded([at(-.49, .72), at(.49, .72), at(.48, .86), at(-.47, .86)], [w * .04, w * .04, w * .07, w * .07]),
        _shade((top + h * .72, CAR_BODY), (top + h * .86, CAR_LOW)))
  rl.draw_line_ex(rl.Vector2(*at(-.47, .725)), rl.Vector2(*at(.47, .725)), max(1., h * .012), _fade(CAR_GLASS, 140))
  _fill(_rounded(box(-.40, .83, .40, .875), w * .02), lambda y: rl.Color(46, 50, 62, 255))


def lead_badge(fonts, content, present, lead=None, trend=None, now=None, personality=None, avoid=None):
  """Lead car on the following-distance bars, like the Honda cluster: the Civic sits closer at 1 bar and further away
  at 3. A soft halo carries the state: yellow while the lead holds speed, red with its brake lamps lit while it brakes
  (pulsing when it brakes hard), teal while it pulls away."""
  if not present:
    return
  state = trend or STEADY
  braking = state == BRAKING
  accent = LEAD_RED if braking else LEAD_TEAL if state == PULLING_AWAY else LEAD_YELLOW
  now = time.monotonic() if now is None else now
  hard = braking and lead is not None and lead.a_lead <= -2.5
  pulse = .5 + .5 * math.sin(now / .45 * 2 * math.pi) if hard else 1.

  g = badge_geometry(content, personality, avoid)
  cx, y, lane = g['cx'], g['y'], g['lane']
  car_w, ground = g['car_width'], g['car_ground']
  car_h = car_w * .875  # roof to tyres

  _glow(cx, ground - car_h * .45, car_w * .95, _fade(accent, 85 * pulse), _fade(accent, 0))

  # lane lines from the near edge into the distance, fading as they go
  stroke = max(1.5, g['unit'] * .085)
  steps = 8
  for side in (-1, 1):
    for i in range(steps):
      z0 = 1 + (ROAD_FAR - 1) * i / steps
      z1 = 1 + (ROAD_FAR - 1) * (i + 1) / steps
      rl.draw_line_ex(rl.Vector2(cx + side * lane / z0, y(z0)), rl.Vector2(cx + side * lane / z1, y(z1)),
                      max(1., stroke / z0), _fade(BAR_FILL, 200 - 150 * i / steps))

  # one bar per distance step, drawn on the road in perspective
  for near, far in g['bars']:
    half_near, half_far = lane * .8 / near, lane * .8 / far
    _fill([(cx - half_far, y(far)), (cx + half_far, y(far)), (cx + half_near, y(near)), (cx - half_near, y(near))],
          lambda _: BAR_FILL)

  sprite_w = round(car_w / SPRITE_CAR_WIDTH)
  sprite = lead_sprite(fonts, LEAD_SPRITE, sprite_w)
  if sprite is None:
    _vector_car(cx, ground - car_w * .74, car_w, car_w * .74, braking, pulse)
    return
  x0, y0 = round(cx - sprite.width / 2), round(ground - sprite.height * SPRITE_GROUND)
  source = rl.Rectangle(0, 0, sprite.width, sprite.height)
  dest = rl.Rectangle(x0, y0, sprite.width, sprite.height)
  rl.draw_texture_pro(sprite, source, dest, rl.Vector2(0, 0), 0., SPRITE_TINT)
  if braking:
    lit = lead_sprite(fonts, LEAD_SPRITE_BRAKE, sprite_w)
    if lit is not None:
      rl.draw_texture_pro(lit, source, dest, rl.Vector2(0, 0), 0., _fade(SPRITE_TINT, 255 * pulse))
    for fx, fy in SPRITE_LAMPS:
      _glow(x0 + sprite.width * fx, y0 + sprite.height * fy, car_w * .2, _fade(LEAD_RED, 120 * pulse), _fade(LEAD_RED, 0))
