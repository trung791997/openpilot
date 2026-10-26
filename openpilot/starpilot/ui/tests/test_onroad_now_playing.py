from types import SimpleNamespace

import pytest

TRACK = {'source': 'spotify', 'status': 'playing', 'title': 'Midnight City', 'artist': 'M83', 'album': '',
         'durationMs': 200000, 'position': {'ms': 50000, 'at': 0, 'speed': 0.0}, 'shuffle': False}


@pytest.fixture
def widget(monkeypatch):
  from openpilot.starpilot.ui import onroad_now_playing
  from openpilot.starpilot.ui.onroad_now_playing import NowPlayingWidget
  drawn = []
  for name in ('draw_rectangle_rounded', 'draw_triangle', 'draw_circle_v', 'draw_line_ex'):
    monkeypatch.setattr(onroad_now_playing.rl, name, lambda *args, name=name: drawn.append((name, args)))
  texts = []
  fonts = SimpleNamespace(measure=lambda text, role, size: SimpleNamespace(width=len(text) * size * .5),
                          draw=lambda text, *args: texts.append(text))
  return NowPlayingWidget(fonts, clock=lambda: 0.0), drawn, texts


def state(enabled=True, opacity=85, alert='none', collapse=False, x=10):
  placed = {'x': x, 'y': 20, 'enabled': enabled, 'opacity': opacity, 'collapse': collapse}
  return SimpleNamespace(alert=SimpleNamespace(size=alert), viewport_width=1860,
                         customization={'layouts': {'large': {'now_playing': placed}}})


def test_draws_text_and_background_with_the_chosen_opacity(widget):
  card, drawn, texts = widget
  card.track = TRACK
  card.render(state(opacity=50))
  assert texts == ['Midnight City', 'M83']
  assert drawn[0][0] == 'draw_rectangle_rounded' and drawn[0][1][3].a == round(235 * .5)


def test_paused_shows_a_play_mark_and_playing_shows_pause_bars(widget):
  card, drawn, _ = widget
  card.track = {**TRACK, 'status': 'paused'}
  card.render(state())
  # Previous, next and shuffle draw three or four triangles; the play mark adds one more.
  paused = sum(name == 'draw_triangle' for name, _ in drawn)
  drawn.clear()
  card.track = TRACK
  card.render(state())
  assert sum(name == 'draw_triangle' for name, _ in drawn) == paused - 1


def test_shuffle_dot_only_when_on(widget):
  card, drawn, _ = widget
  card.track = TRACK
  card.render(state())
  off = sum(name == 'draw_circle_v' for name, _ in drawn)
  drawn.clear()
  card.track = {**TRACK, 'shuffle': True}
  card.render(state())
  assert sum(name == 'draw_circle_v' for name, _ in drawn) == off + 1


def test_collapsed_paused_card_is_just_the_play_button(widget):
  card, drawn, texts = widget
  card.track = {**TRACK, 'status': 'paused'}
  card.render(state(collapse=True))
  assert texts == [] and sum(name == 'draw_triangle' for name, _ in drawn) == 1


@pytest.mark.parametrize('current', [state(enabled=False), state(alert='full')])
def test_hidden_when_disabled_or_an_alert_is_up(widget, current):
  card, drawn, texts = widget
  card.track = TRACK
  card.render(current)
  assert drawn == [] and texts == []


def test_nothing_drawn_without_a_track(widget):
  card, drawn, texts = widget
  card.render(state())
  assert drawn == [] and texts == []


def test_buttons_span_the_card_in_order():
  from openpilot.starpilot.ui.onroad_now_playing import CONTROLS, card_layout
  rect, buttons = card_layout(state().customization['layouts']['large']['now_playing'], TRACK, 1860)
  assert (rect.width, rect.height) == (600, 140) and tuple(buttons) == CONTROLS
  assert [box.x for box in buttons.values()] == [10, 160, 310, 460]
  assert all(box.y == 100 and box.height == 60 for box in buttons.values())


@pytest.mark.parametrize('x, expected', [(10, 10), (1230, 1690)])
def test_collapsed_play_button_sits_at_the_end_nearer_the_edge(x, expected):
  from openpilot.starpilot.ui.onroad_now_playing import card_layout
  placed = state(collapse=True, x=x).customization['layouts']['large']['now_playing']
  rect, buttons = card_layout(placed, {**TRACK, 'status': 'paused'}, 1860)
  assert list(buttons) == ['play_pause'] and (rect.x, rect.width, rect.height) == (expected, 140, 140)


def tap(touch, current, track, x, y, lift=None):
  placed = current.customization['layouts']['large']['now_playing']
  down = touch.touch('down', x, y, current, placed, track)
  up = touch.touch('up', *(lift or (x, y)), current, placed, track)
  return down, up


@pytest.mark.parametrize('x, action', [(80, 'shuffle'), (230, 'previous'), (380, 'play_pause'), (530, 'next')])
def test_tap_sends_the_button_under_the_finger(x, action):
  from openpilot.starpilot.ui.onroad_now_playing import NowPlayingTouch
  sent = []
  touch = NowPlayingTouch(lambda *call: sent.append(call))
  assert tap(touch, state(), TRACK, x, 130) == (True, True)
  assert sent == [(action, {'enabled': True} if action == 'shuffle' else {})]


def test_shuffle_tap_toggles_off_and_is_ignored_when_the_player_cannot_shuffle():
  from openpilot.starpilot.ui.onroad_now_playing import NowPlayingTouch
  sent = []
  touch = NowPlayingTouch(lambda *call: sent.append(call))
  tap(touch, state(), {**TRACK, 'shuffle': True}, 80, 130)
  tap(touch, state(), {**TRACK, 'shuffle': None}, 80, 130)
  assert sent == [('shuffle', {'enabled': False})]


def test_card_swallows_taps_but_only_fires_on_the_same_button():
  from openpilot.starpilot.ui.onroad_now_playing import NowPlayingTouch
  sent = []
  touch = NowPlayingTouch(lambda *call: sent.append(call))
  assert tap(touch, state(), TRACK, 300, 40) == (True, True)  # title area: claimed, no action
  assert tap(touch, state(), TRACK, 530, 130, lift=(230, 130)) == (True, True)  # slid to another button
  assert tap(touch, state(), TRACK, 900, 130) == (False, False)  # off the card
  assert sent == []


def test_alert_or_missing_card_releases_the_touch():
  from openpilot.starpilot.ui.onroad_now_playing import NowPlayingTouch
  sent = []
  touch = NowPlayingTouch(lambda *call: sent.append(call))
  placed = state().customization['layouts']['large']['now_playing']
  assert touch.touch('down', 530, 130, state(), placed, TRACK)
  assert touch.touch('up', 530, 130, state(alert='full'), placed, TRACK)
  assert not touch.touch('down', 530, 130, None, placed, TRACK)
  assert sent == []


def test_scroll_holds_then_runs_one_pass_and_restarts():
  from openpilot.starpilot.ui.onroad_now_playing import SCROLL_GAP, SCROLL_PAUSE, SCROLL_SPEED, scroll_offset
  assert scroll_offset(100, 200, 5.0) == 0.0
  assert scroll_offset(300, 200, SCROLL_PAUSE - .1) == 0.0
  assert scroll_offset(300, 200, SCROLL_PAUSE + 1) == pytest.approx(SCROLL_SPEED)
  cycle = SCROLL_PAUSE + (300 + SCROLL_GAP) / SCROLL_SPEED
  assert scroll_offset(300, 200, cycle + .5) == 0.0


def test_long_artist_scrolls_inside_a_clip_and_short_one_does_not(widget, monkeypatch):
  from openpilot.starpilot.ui import onroad_now_playing
  card, _, texts = widget
  clips = []
  monkeypatch.setattr(onroad_now_playing.clip, 'begin_scissor_mode', lambda *args: clips.append(args))
  monkeypatch.setattr(onroad_now_playing.clip, 'end_scissor_mode', lambda: clips.append('end'))
  card.track = TRACK
  card.render(state())
  assert clips == []
  card.track = {**TRACK, 'artist': 'A very long list of featured artists on this one track'}
  now = [100.0]
  card.ticks = lambda: now[0]
  texts.clear()
  card.render(state())
  assert texts[1:] == [card.track['artist']] and clips[-1] == 'end'  # held at the start: one copy
  now[0] += onroad_now_playing.SCROLL_PAUSE + 6  # 240 px in: the next copy has come into view
  texts.clear()
  card.render(state())
  assert texts[1:] == [card.track['artist']] * 2 and clips[-1] == 'end'
  card.track = {**card.track, 'artist': card.track['artist'] + ' and more'}
  texts.clear()
  card.render(state())
  assert texts[1:] == [card.track['artist']]  # a new name starts held again
  # The clip is the text column beside the cover: x + 96 to the right corner.
  assert clips[0][:3] == (10 + 96, 20 + 44, 600 - 96 - 28)


def test_cover_texture_is_built_once_per_cover_and_drawn(widget, monkeypatch):
  from openpilot.starpilot.ui import onroad_now_playing
  rl = onroad_now_playing.rl
  card, drawn, _ = widget
  loads, unloads = [], []
  image = SimpleNamespace(width=300, height=300)
  monkeypatch.setattr(rl, 'load_image_from_memory', lambda kind, data, size: loads.append(kind) or image)
  monkeypatch.setattr(rl, 'image_resize', lambda *args: None)
  monkeypatch.setattr(rl, 'unload_image', lambda *args: None)
  monkeypatch.setattr(rl, 'load_texture_from_image', lambda image: SimpleNamespace(width=120, height=120, id=len(loads)))
  monkeypatch.setattr(rl, 'set_texture_filter', lambda *args: None)
  monkeypatch.setattr(rl, 'is_window_ready', lambda: True)
  monkeypatch.setattr(rl, 'unload_texture', lambda texture: unloads.append(texture.id))
  monkeypatch.setattr(rl, 'draw_texture_pro', lambda *args: drawn.append(('draw_texture_pro', args)))
  card.track = TRACK
  card.artwork = ('https://i.scdn.co/image/a', b'\xff\xd8\xff\xe0 jpeg')
  card.render(state())
  card.render(state())
  assert loads == ['.jpg'] and sum(name == 'draw_texture_pro' for name, _ in drawn) == 2
  dest = next(args for name, args in drawn if name == 'draw_texture_pro')[2]
  assert (dest.x, dest.y, dest.width, dest.height) == (30, 30, 60, 60)
  card.artwork = ('', None)
  drawn.clear()
  card.render(state())
  assert unloads == [1] and not any(name == 'draw_texture_pro' for name, _ in drawn)


def test_bytes_that_are_not_an_image_get_the_placeholder(widget, monkeypatch):
  from openpilot.starpilot.ui import onroad_now_playing
  card, _, _ = widget
  monkeypatch.setattr(onroad_now_playing.rl, 'load_image_from_memory', lambda *args: pytest.fail('decoded a non-image'))
  card.track = TRACK
  card.artwork = ('https://i.scdn.co/image/a', b'<html>')
  card.render(state())
  assert card._cover == ('https://i.scdn.co/image/a', None)


class CoveringFonts:
  """Bitmap fonts that, like the real atlases, have glyphs for ASCII only."""
  profile = SimpleNamespace(font_scale=1.0)

  def __init__(self):
    self.drawn = []

  def covers(self, text, role):
    return text.isascii()

  def measure(self, text, role, size):
    return SimpleNamespace(width=len(text) * size * .5)

  def draw(self, text, *args):
    self.drawn.append(text)


@pytest.fixture
def cut_fonts(monkeypatch):
  from openpilot.starpilot.ui import onroad_now_playing
  loads, unloads, ttf_drawn = [], [], []
  monkeypatch.setattr(onroad_now_playing.TrackText, '_load', staticmethod(lambda text, role: loads.append((text, role)) or f'font:{text}'))
  monkeypatch.setattr(onroad_now_playing.rl, 'is_window_ready', lambda: True)
  monkeypatch.setattr(onroad_now_playing.rl, 'unload_font', unloads.append)
  monkeypatch.setattr(onroad_now_playing.rl, 'draw_text_ex', lambda font, text, *args: ttf_drawn.append(text))
  monkeypatch.setattr(onroad_now_playing.rl, 'measure_text_ex', lambda font, text, size, spacing: SimpleNamespace(x=len(text) * size * .5, y=size))
  monkeypatch.delattr(onroad_now_playing.rl, '_orig_draw_text_ex', raising=False)
  return onroad_now_playing.TrackText(CoveringFonts()), loads, unloads, ttf_drawn


def test_ascii_names_stay_on_the_bitmap_atlas(cut_fonts):
  from openpilot.starpilot.ui.presentation import FontRole
  text, loads, _, ttf_drawn = cut_fonts
  text.draw('Midnight City', FontRole.SEMI_BOLD, 30, 0, 0, None)
  assert text.fonts.drawn == ['Midnight City'] and loads == [] and ttf_drawn == []


def test_vietnamese_name_is_cut_from_the_ttf_once_and_freed_on_the_next_song(cut_fonts):
  from openpilot.starpilot.ui.presentation import FontRole
  text, loads, unloads, ttf_drawn = cut_fonts
  title = 'Em Về Tinh Khôi'
  for _ in range(3):
    text.measure(title, FontRole.SEMI_BOLD, 30)
    text.draw(title, FontRole.SEMI_BOLD, 30, 0, 0, None)
  assert loads == [(title, FontRole.SEMI_BOLD)] and ttf_drawn == [title] * 3 and text.fonts.drawn == []
  text.draw('Midnight City', FontRole.SEMI_BOLD, 30, 0, 0, None)
  assert unloads == [f'font:{title}'] and text.fonts.drawn == ['Midnight City']
  text.draw('Nắng Ấm Xa Dần', FontRole.NORMAL, 22, 0, 0, None)
  text.close()
  assert unloads == [f'font:{title}', 'font:Nắng Ấm Xa Dần']


def test_failed_ttf_load_falls_back_to_the_atlas(cut_fonts, monkeypatch):
  from openpilot.starpilot.ui import onroad_now_playing
  from openpilot.starpilot.ui.presentation import FontRole
  text, _, unloads, _ = cut_fonts
  monkeypatch.setattr(onroad_now_playing.TrackText, '_load', staticmethod(lambda text, role: None))
  text.draw('Khôi', FontRole.SEMI_BOLD, 30, 0, 0, None)
  text.close()
  assert text.fonts.drawn == ['Khôi'] and unloads == []


def test_decomposed_accents_are_composed_before_drawing(widget):
  card, _, texts = widget
  card.track = {**TRACK, 'title': 'Về', 'artist': 'Khôi'}
  card.render(state())
  assert texts == ['Về', 'Khôi']
