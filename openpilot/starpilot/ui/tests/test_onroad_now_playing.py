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
