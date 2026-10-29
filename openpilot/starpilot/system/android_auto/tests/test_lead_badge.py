"""Static: the LEAD badge's braking / pulling-away state, held so a noisy radar acceleration does not flicker it."""
import gc
import struct
from types import SimpleNamespace

import pyray
import pytest

from openpilot.starpilot.system.android_auto import projection_theme as theme
from openpilot.starpilot.system.android_auto.projection_theme import (BRAKING, PULLING_AWAY, STEADY, LeadTrend,
                                                                       LeadView, gap_bars)


def run(samples, dt=.05):
  clock = [0.]
  trend = LeadTrend(clock=lambda: clock[0])
  states = []
  for a_lead, v_rel in samples:
    states.append(trend.update(LeadView(30., v_rel, a_lead, 20.)))
    clock[0] += dt
  return states


def test_steady_lead_stays_yellow():
  assert set(run([(.3, 0.), (-.6, -.2), (.5, .3)] * 40)) == {STEADY}


def test_braking_needs_four_tenths_and_holds_through_a_blip():
  states = run([(-1.4, -1.)] * 6 + [(-1.4, -1.)] * 10 + [(-.7, -1.)] * 4 + [(.1, 0.)] * 30)
  assert states[5] == STEADY            # 0.25 s of braking is not enough
  assert states[9] == BRAKING           # 0.45 s is
  assert states[17] == BRAKING          # -0.7 is still inside the -0.5 release band
  assert states[20 + 10] == BRAKING     # 0.5 s after it eased off: held
  assert states[-1] == STEADY           # 0.8 s after it eased off: cleared


def test_alternating_noise_never_turns_red():
  assert BRAKING not in run([(-1.6, 0.), (.2, 0.)] * 60)


def test_pulling_away_needs_the_gap_opening():
  assert PULLING_AWAY not in run([(1., -.5)] * 40)  # speeding up but still closing
  assert run([(1., 1.)] * 40)[-1] == PULLING_AWAY


def test_braking_overrides_pulling_away_without_waiting_out_the_hold():
  states = run([(1., 1.)] * 20 + [(-2., -.5)] * 10)
  assert states[19] == PULLING_AWAY and states[-1] == BRAKING


def test_lost_or_bad_lead_resets():
  clock = [0.]
  trend = LeadTrend(clock=lambda: clock[0])
  for _ in range(20):
    trend.update(LeadView(30., -1., -2., 20.))
    clock[0] += .05
  assert trend.state == BRAKING
  assert trend.update(None) == STEADY
  assert trend.update(LeadView(30., float('nan'), -2., 20.)) == STEADY


@pytest.mark.parametrize('personality, bars', (('aggressive', 1), ('standard', 2), ('relaxed', 3), (0, 1), (1, 2), (2, 3), (None, 2)))
def test_gap_bars_follow_the_personality(personality, bars):
  assert gap_bars(personality) == bars


def test_gap_bars_take_the_capnp_enum():
  from cereal import log
  for name, bars in (('aggressive', 1), ('standard', 2), ('relaxed', 3)):
    assert gap_bars(log.LongitudinalPersonality.schema.enumerants[name]) == bars


@pytest.mark.parametrize('nargs', [4, 5])
def test_glow_matches_either_raylib_gradient_signature(monkeypatch, nargs):
  # The comma's raylib takes (Vector2, radius, inner, outer); passing ints there killed the car view.
  calls = []
  fake = SimpleNamespace(ffi=SimpleNamespace(typeof=lambda fn: SimpleNamespace(args=[0] * nargs)),
                         rl=SimpleNamespace(DrawCircleGradient=None), Vector2=lambda x, y: ('v', x, y),
                         draw_circle_gradient=lambda *a: calls.append(a))
  monkeypatch.setattr(theme, 'rl', fake)
  theme._glow(10.6, 20.2, 5., 'in', 'out')
  assert calls == [(('v', 10.6, 20.2), 5., 'in', 'out')] if nargs == 4 else [(10, 20, 5., 'in', 'out')]


# Static: the badge mimics the Honda cluster's following-distance display with a render of Peter's Civic.
CONTENT = SimpleNamespace(x=40., y=30., width=1200., height=500.)


class Fonts:
  """Stands in for BitmapFonts (one per graphics context, weakref-able); any text drawn through it is recorded."""
  def __init__(self):
    self.text = []

  def __getattr__(self, name):
    return lambda *a, **k: self.text.append((name, a))


class Raylib:
  """Real pyray types and constants; every draw / load / unload call is recorded instead of touching a GL context."""
  def __init__(self):
    self.calls = []

  def __getattr__(self, name):
    if name.startswith(('draw_', 'rl_', 'unload_', 'load_', 'image_', 'set_texture')):
      return lambda *a: self.calls.append((name, a))
    return getattr(pyray, name)

  def named(self, name):
    return [a for n, a in self.calls if n == name]


@pytest.fixture
def rl(monkeypatch):
  fake = Raylib()
  monkeypatch.setattr(theme, 'rl', fake)
  return fake


def texture(width, height=None):
  return SimpleNamespace(id=1, width=width, height=round(width * 247 / 256) if height is None else height)


@pytest.mark.parametrize('personality', ['aggressive', 'standard', 'relaxed'])
def test_one_bar_per_distance_setting(personality):
  assert len(theme.badge_geometry(CONTENT, personality)['bars']) == gap_bars(personality)


def test_car_is_closer_and_larger_at_one_bar():
  near, mid, far = (theme.badge_geometry(CONTENT, p) for p in ('aggressive', 'standard', 'relaxed'))
  assert near['car_width'] > mid['car_width'] > far['car_width']
  assert near['car_ground'] > mid['car_ground'] > far['car_ground']  # lower on screen is nearer


@pytest.mark.parametrize('personality', ['aggressive', 'standard', 'relaxed'])
def test_car_sits_just_past_the_last_bar(personality):
  g = theme.badge_geometry(CONTENT, personality)
  next_bar = 1 + len(g['bars']) * theme.GAP_STEP  # the car's tyres stand where one more bar would start
  assert g['y'](next_bar + theme.BAR_DEPTH) < g['car_ground'] < g['y'](next_bar) < g['y'](g['bars'][-1][1])
  near_edges = [g['y'](near) for near, _ in g['bars']]
  assert near_edges == sorted(near_edges, reverse=True)  # the first bar is the nearest one



# Static: on Peter's 800x480 car screen (a 1860x1116 canvas) with the speed card where he placed it, the badge's near
# bar and left lane line ran into the card's right edge (2026-10-29 photo).
SCREEN = SimpleNamespace(x=0., y=0., width=1860., height=1116.)
PETER_CARD = (81, 210, 81 + theme.CARD_WIDTH, 210 + 460 + theme.BUTTON_HEIGHT + 20)


def road_span(g):
  half = g['lane'] + max(1.5, g['unit'] * .085) / 2
  return g['cx'] - half, g['cx'] + half


def test_badge_used_to_overlap_the_speed_card():
  assert road_span(theme.badge_geometry(SCREEN, 'standard'))[0] < PETER_CARD[2]


@pytest.mark.parametrize('personality', ['aggressive', 'standard', 'relaxed'])
def test_badge_slides_right_off_the_speed_card(personality):
  g = theme.badge_geometry(SCREEN, personality, PETER_CARD)
  assert road_span(g)[0] == pytest.approx(PETER_CARD[2] + theme.CARD_CLEARANCE)
  assert g['cx'] == theme.badge_geometry(SCREEN, 'standard', PETER_CARD)['cx']  # no hop when the setting changes


def test_badge_stays_put_when_the_card_is_clear_of_it():
  home = theme.badge_geometry(SCREEN, 'standard')['cx']
  below = (81, 700, 81 + theme.CARD_WIDTH, 1100)
  assert theme.badge_geometry(SCREEN, 'standard', below)['cx'] == home
  assert theme.badge_geometry(SCREEN, 'standard', None)['cx'] == home


def test_badge_slides_left_off_a_card_on_its_right():
  home = theme.badge_geometry(SCREEN, 'standard')['cx']
  card = (home - 40, 150, home - 40 + theme.CARD_WIDTH, 650)
  assert road_span(theme.badge_geometry(SCREEN, 'standard', card))[1] == pytest.approx(card[0] - theme.CARD_CLEARANCE)


def test_card_rect_only_while_the_card_draws(monkeypatch):
  shown = {'enabled': True}
  monkeypatch.setattr(theme, '_card_body', lambda state: (81, 210, True, True, 480))
  monkeypatch.setattr(theme, 'card_height', lambda state: 560)
  monkeypatch.setattr(theme, 'placement', lambda customization, profile, key: shown)
  state = SimpleNamespace(customization={})
  assert theme.card_rect(state) == (81, 210, 81 + theme.CARD_WIDTH, 770)
  shown['enabled'] = False
  assert theme.card_rect(state) is None
  monkeypatch.setattr(theme, '_card_body', lambda state: None)
  shown['enabled'] = True
  assert theme.card_rect(state) is None


def test_overlay_passes_the_card_to_the_badge(monkeypatch):
  seen = []
  monkeypatch.setattr(theme, 'edge_glow', lambda *a: None)
  monkeypatch.setattr(theme, 'state_glow', lambda *a: None)
  monkeypatch.setattr(theme, 'side_colors', lambda *a: (None, None))
  monkeypatch.setattr(theme, 'card_rect', lambda state: PETER_CARD)
  monkeypatch.setattr(theme, 'lead_badge', lambda *a, **k: seen.append(k['avoid']))
  state = SimpleNamespace(appearance=SimpleNamespace(show_lead_indicator=True))
  theme.overlay(Fonts(), SCREEN, state, 'chill', True)
  assert seen == [PETER_CARD]

def test_no_lead_word(rl, monkeypatch):
  monkeypatch.setattr(theme, '_load_sprite', lambda path, width: texture(width))
  fonts = Fonts()
  for personality in ('aggressive', 'relaxed'):
    for state in (theme.STEADY, theme.BRAKING, theme.PULLING_AWAY):
      theme.lead_badge(fonts, CONTENT, True, LeadView(30., -1., -3., 20.), state, now=0., personality=personality)
  assert fonts.text == [] and not [n for n, _ in rl.calls if 'text' in n.split('_')]


def test_nothing_drawn_without_a_lead(rl):
  theme.lead_badge(Fonts(), CONTENT, False, personality='standard')
  assert rl.calls == []


def test_sprite_stands_on_the_road_inside_the_screen(rl, monkeypatch):
  monkeypatch.setattr(theme, '_load_sprite', lambda path, width: texture(width))
  for personality in ('aggressive', 'standard', 'relaxed'):
    rl.calls.clear()
    theme.lead_badge(Fonts(), CONTENT, True, None, theme.STEADY, now=0., personality=personality)
    (sprite, _, dest, _, _, tint), = rl.named('draw_texture_pro')
    g = theme.badge_geometry(CONTENT, personality)
    assert dest.y + dest.height * theme.SPRITE_GROUND == pytest.approx(g['car_ground'], abs=1.)
    assert dest.width * theme.SPRITE_CAR_WIDTH == pytest.approx(g['car_width'], abs=1.)
    assert dest.y >= CONTENT.y and abs(dest.x + dest.width / 2 - g['cx']) <= 1.
    assert (tint.r, tint.g, tint.b, tint.a) == (255, 255, 255, 255)


def test_brake_lamps_light_and_pulse_when_braking_hard(rl, monkeypatch):
  loaded = []
  monkeypatch.setattr(theme, '_load_sprite', lambda path, width: loaded.append(path.name) or texture(width))
  fonts = Fonts()
  theme.lead_badge(fonts, CONTENT, True, LeadView(30., -1., -1., 20.), theme.BRAKING, now=0., personality='standard')
  base, lit = rl.named('draw_texture_pro')
  assert loaded == [theme.LEAD_SPRITE, theme.LEAD_SPRITE_BRAKE] and lit[5].a == 255
  rl.calls.clear()
  theme.lead_badge(fonts, CONTENT, True, LeadView(30., -1., -3., 20.), theme.BRAKING, now=.45 * .75,
                   personality='standard')
  assert rl.named('draw_texture_pro')[1][5].a == 0  # hard braking: the lit frame pulses


def test_vector_car_when_the_sprite_will_not_load(rl, monkeypatch):
  drawn = []
  monkeypatch.setattr(theme, '_vector_car', lambda *a: drawn.append(a))
  monkeypatch.setattr(theme, '_load_sprite', lambda path, width: None)
  theme.lead_badge(Fonts(), CONTENT, True, None, theme.STEADY, now=0., personality='standard')
  assert len(drawn) == 1 and rl.named('draw_texture_pro') == []


def test_vector_car_when_loading_raises_and_no_retry_each_frame(rl, monkeypatch):
  tries, drawn = [], []
  def boom(path, width):
    tries.append(width)
    raise OSError('no GL context')
  monkeypatch.setattr(theme, '_vector_car', lambda *a: drawn.append(a))
  monkeypatch.setattr(theme, '_load_sprite', boom)
  fonts = Fonts()
  for _ in range(5):
    theme.lead_badge(fonts, CONTENT, True, None, theme.STEADY, now=0., personality='standard')
  assert len(drawn) == 5 and len(tries) == 1


def test_vector_car_when_the_fonts_cannot_key_the_cache(rl, monkeypatch):
  drawn = []
  monkeypatch.setattr(theme, '_vector_car', lambda *a: drawn.append(a))
  monkeypatch.setattr(theme, '_load_sprite', lambda path, width: texture(width))
  theme.lead_badge(SimpleNamespace(), CONTENT, True, None, theme.STEADY, now=0., personality='standard')
  assert len(drawn) == 1


def test_sprite_cache_is_per_context_and_bounded(rl, monkeypatch):
  loads = []
  monkeypatch.setattr(theme, '_load_sprite', lambda path, width: loads.append(width) or texture(width))
  first, second = Fonts(), Fonts()
  a = theme.lead_sprite(first, theme.LEAD_SPRITE, 100)
  assert theme.lead_sprite(first, theme.LEAD_SPRITE, 100.4) is a and loads == [100]
  assert theme.lead_sprite(second, theme.LEAD_SPRITE, 100) is not a and loads == [100, 100]
  for width in range(101, 101 + theme.SPRITE_CACHE):
    theme.lead_sprite(first, theme.LEAD_SPRITE, width)
  assert rl.named('unload_texture') == [(a,)] and len(theme._sprites[first]) == theme.SPRITE_CACHE
  del first
  gc.collect()
  assert len(theme._sprites) == 1  # the closed context's textures are not kept


def test_load_shrinks_on_the_cpu_and_frees_the_image(rl, monkeypatch):
  image = SimpleNamespace(data=object(), width=256, height=247)
  calls = []
  monkeypatch.setattr(rl, 'load_image', lambda path: calls.append(('load', path)) or image, raising=False)
  monkeypatch.setattr(rl, 'load_texture_from_image', lambda img: SimpleNamespace(id=9), raising=False)
  tex = theme._load_sprite(theme.LEAD_SPRITE_DIR / theme.LEAD_SPRITE, 128)
  assert tex.id == 9 and rl.named('image_resize') == [(image, 128, 124)] and rl.named('unload_image') == [(image,)]
  assert rl.named('set_texture_filter') == [(tex, pyray.TextureFilter.TEXTURE_FILTER_BILINEAR)]
  monkeypatch.setattr(rl, 'load_texture_from_image', lambda img: SimpleNamespace(id=0), raising=False)
  assert theme._load_sprite(theme.LEAD_SPRITE_DIR / theme.LEAD_SPRITE, 128) is None
  assert theme._load_sprite(theme.LEAD_SPRITE_DIR / 'missing.png', 128) is None and len(calls) == 2


@pytest.mark.parametrize('name', [theme.LEAD_SPRITE, theme.LEAD_SPRITE_BRAKE])
def test_sprites_ship_as_matching_rgba_pngs(name):
  head = (theme.LEAD_SPRITE_DIR / name).read_bytes()[:26]
  assert head[:8] == b'\x89PNG\r\n\x1a\n'
  assert struct.unpack('>II', head[16:24]) == (256, 247) and head[25] == 6  # 8-bit RGBA, one crop for both
