from types import SimpleNamespace

import pytest

TRACK = {'source': 'spotify', 'status': 'playing', 'title': 'Midnight City', 'artist': 'M83', 'album': '',
         'durationMs': 200000, 'position': {'ms': 50000, 'at': 0, 'speed': 0.0}, 'shuffle': False, 'repeat': 'off'}
TIMES = ['00:50', '03:20']
# Centers of shuffle, previous, play/pause, next and repeat for the card at x 10, y 20.
BUTTONS = {'shuffle': 125, 'previous': 228, 'play_pause': 330, 'next': 432, 'repeat': 535}
ROW = 234


@pytest.fixture
def widget(monkeypatch):
  from openpilot.starpilot.ui import onroad_now_playing
  from openpilot.starpilot.ui.onroad_now_playing import NowPlayingWidget
  drawn = []
  for name in ('draw_rectangle_rounded', 'draw_triangle', 'draw_circle_v', 'draw_line_ex', 'draw_ring', 'draw_texture_pro',
               'draw_rectangle_rec'):
    monkeypatch.setattr(onroad_now_playing.rl, name, lambda *args, name=name: drawn.append((name, args)))
  monkeypatch.setattr(onroad_now_playing, '_spotify_texture', lambda size: SimpleNamespace(path='icons/spotify.png', width=size, height=size))
  texts = []
  fonts = SimpleNamespace(measure=lambda text, role, size: SimpleNamespace(width=len(text) * size * .5),
                          draw=lambda text, *args: texts.append(text))
  return NowPlayingWidget(fonts, clock=lambda: 0.0), drawn, texts


def state(enabled=True, opacity=85, alert='none', collapse=False, x=10):
  placed = {'x': x, 'y': 20, 'enabled': enabled, 'opacity': opacity, 'collapse': collapse}
  return SimpleNamespace(alert=SimpleNamespace(size=alert), viewport_width=1860,
                         customization={'layouts': {'large': {'now_playing': placed}}})


def count(drawn, kind):
  return sum(name == kind for name, _ in drawn)


def test_draws_device_song_and_times_over_the_chosen_opacity(widget):
  card, drawn, texts = widget
  card.track = TRACK
  card.render(state(opacity=50))
  assert texts == ['Spotify', 'Midnight City', 'M83', *TIMES]
  assert drawn[0][0] == 'draw_rectangle_rounded' and drawn[0][1][3].a == round(235 * .5)


@pytest.mark.parametrize('track, label', [({'device': 'Starpilot Auto'}, 'Starpilot Auto'),
                                          ({'source': 'phone', 'device': ''}, 'Phone')])
def test_top_line_names_where_the_music_plays(widget, track, label):
  card, _, texts = widget
  card.track = {**TRACK, **track}
  card.render(state())
  assert texts[0] == label


def test_paused_shows_a_play_mark_and_playing_shows_pause_bars(widget):
  card, drawn, _ = widget
  card.track = {**TRACK, 'status': 'paused'}
  card.render(state())
  paused = count(drawn, 'draw_triangle')
  drawn.clear()
  card.track = TRACK
  card.render(state())
  assert count(drawn, 'draw_triangle') == paused - 1


def test_shuffle_dot_only_when_on(widget):
  card, drawn, _ = widget
  card.track = TRACK
  card.render(state())
  off = count(drawn, 'draw_circle_v')
  drawn.clear()
  card.track = {**TRACK, 'shuffle': True}
  card.render(state())
  assert count(drawn, 'draw_circle_v') == off + 1


def test_smart_shuffle_adds_the_star(widget):
  card, drawn, _ = widget
  card.track = {**TRACK, 'shuffle': True}
  card.render(state())
  plain = count(drawn, 'draw_triangle')
  drawn.clear()
  card.track = {**TRACK, 'shuffle': True, 'smartShuffle': True}
  card.render(state())
  assert count(drawn, 'draw_triangle') == plain + 8


def test_spotify_mark_sits_top_right_and_only_for_spotify(widget):
  card, drawn, _ = widget
  for source, marks in (('spotify', 1), ('phone', 0)):
    drawn.clear()
    card.track = {**TRACK, 'source': source}
    card.render(state())
    logos = [args for name, args in drawn if name == 'draw_texture_pro' and getattr(args[0], 'path', '') == 'icons/spotify.png']
    assert len(logos) == marks  # one mark, top right; none in the title line
    assert all((d[2].x + d[2].width / 2, d[2].y + d[2].height / 2) == (10 + 640 - 46 - 10, 20 + 32) for d in logos)


def test_repeat_shows_a_dot_when_on_and_a_bar_for_one_song(widget):
  card, drawn, _ = widget
  shown = {}
  for mode in ('off', 'context', 'track'):
    drawn.clear()
    card.track = {**TRACK, 'repeat': mode}
    card.render(state())
    shown[mode] = (count(drawn, 'draw_circle_v'), count(drawn, 'draw_rectangle_rounded'))
  assert shown['context'] == (shown['off'][0] + 1, shown['off'][1])
  assert shown['track'] == (shown['off'][0] + 1, shown['off'][1] + 1)


def test_seek_bar_without_a_length_has_no_thumb_or_times(widget):
  card, drawn, texts = widget
  card.track = {**TRACK, 'durationMs': 0}
  card.render(state())
  assert texts == ['Spotify', 'Midnight City', 'M83']


def test_collapsed_paused_card_is_just_the_play_button(widget):
  card, drawn, texts = widget
  card.track = {**TRACK, 'status': 'paused'}
  card.render(state(collapse=True))
  assert texts == [] and count(drawn, 'draw_triangle') == 1


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


def test_buttons_sit_in_a_centered_row_in_order():
  from openpilot.starpilot.ui.onroad_now_playing import CONTROLS, card_layout
  rect, buttons = card_layout(state().customization['layouts']['large']['now_playing'], TRACK, 1860)
  assert (rect.width, rect.height) == (640, 264) and tuple(buttons) == CONTROLS
  assert [round(box.x + box.width / 2) for box in buttons.values()] == list(BUTTONS.values())
  assert all(box.y == 20 + 164 and box.y + box.height == 20 + 264 for box in buttons.values())


@pytest.mark.parametrize('x, expected', [(10, 10), (1210, 1730)])
def test_collapsed_play_button_sits_in_the_bottom_corner_nearer_the_edge(x, expected):
  from openpilot.starpilot.ui.onroad_now_playing import card_layout
  placed = state(collapse=True, x=x).customization['layouts']['large']['now_playing']
  rect, buttons = card_layout(placed, {**TRACK, 'status': 'paused'}, 1860)
  assert list(buttons) == ['play_pause'] and (rect.x, rect.y, rect.width, rect.height) == (expected, 164, 120, 120)


def tap(touch, current, track, x, y, lift=None):
  placed = current.customization['layouts']['large']['now_playing']
  down = touch.touch('down', x, y, current, placed, track)
  up = touch.touch('up', *(lift or (x, y)), current, placed, track)
  return down, up


@pytest.mark.parametrize('action', list(BUTTONS))
def test_tap_sends_the_button_under_the_finger(action):
  from openpilot.starpilot.ui.onroad_now_playing import NowPlayingTouch
  sent = []
  touch = NowPlayingTouch(lambda *call: sent.append(call))
  assert tap(touch, state(), TRACK, BUTTONS[action], ROW) == (True, True)
  args = {'shuffle': {'enabled': True}, 'repeat': {'mode': 'context'}}.get(action, {})
  assert sent == [(action, args)]


def test_shuffle_tap_toggles_off_and_is_ignored_when_the_player_cannot_shuffle():
  from openpilot.starpilot.ui.onroad_now_playing import NowPlayingTouch
  sent = []
  touch = NowPlayingTouch(lambda *call: sent.append(call))
  tap(touch, state(), {**TRACK, 'shuffle': True}, BUTTONS['shuffle'], ROW)
  tap(touch, state(), {**TRACK, 'shuffle': None}, BUTTONS['shuffle'], ROW)
  assert sent == [('shuffle', {'enabled': False})]


def test_repeat_tap_cycles_all_one_song_off_and_is_ignored_for_phone_audio():
  from openpilot.starpilot.ui.onroad_now_playing import NowPlayingTouch
  sent = []
  touch = NowPlayingTouch(lambda *call: sent.append(call))
  for mode in ('context', 'track', None):
    tap(touch, state(), {**TRACK, 'repeat': mode}, BUTTONS['repeat'], ROW)
  assert sent == [('repeat', {'mode': 'track'}), ('repeat', {'mode': 'off'})]


def test_card_swallows_taps_but_only_fires_on_the_same_button():
  from openpilot.starpilot.ui.onroad_now_playing import NowPlayingTouch
  sent = []
  touch = NowPlayingTouch(lambda *call: sent.append(call))
  assert tap(touch, state(), TRACK, 300, 90) == (True, True)  # title line: claimed, no action
  assert tap(touch, state(), TRACK, BUTTONS['next'], ROW, lift=(BUTTONS['previous'], ROW)) == (True, True)  # slid to another button
  assert tap(touch, state(), TRACK, 900, ROW) == (False, False)  # off the card
  assert sent == []


def test_alert_or_missing_card_releases_the_touch():
  from openpilot.starpilot.ui.onroad_now_playing import NowPlayingTouch
  sent = []
  touch = NowPlayingTouch(lambda *call: sent.append(call))
  placed = state().customization['layouts']['large']['now_playing']
  assert touch.touch('down', BUTTONS['next'], ROW, state(), placed, TRACK)
  assert touch.touch('up', BUTTONS['next'], ROW, state(alert='full'), placed, TRACK)
  assert not touch.touch('down', BUTTONS['next'], ROW, None, placed, TRACK)
  assert sent == []


def test_scroll_holds_then_runs_one_pass_and_restarts():
  from openpilot.starpilot.ui.onroad_now_playing import SCROLL_GAP, SCROLL_PAUSE, SCROLL_SPEED, scroll_offset
  assert scroll_offset(100, 200, 5.0) == 0.0
  assert scroll_offset(300, 200, SCROLL_PAUSE - .1) == 0.0
  assert scroll_offset(300, 200, SCROLL_PAUSE + 1) == pytest.approx(SCROLL_SPEED)
  cycle = SCROLL_PAUSE + (300 + SCROLL_GAP) / SCROLL_SPEED
  assert scroll_offset(300, 200, cycle + .5) == 0.0


def test_long_song_line_scrolls_inside_a_clip_and_short_one_does_not(widget, monkeypatch):
  from openpilot.starpilot.ui import onroad_now_playing
  card, _, texts = widget
  clips = []
  monkeypatch.setattr(onroad_now_playing.clip, 'begin_scissor_mode', lambda *args: clips.append(args))
  monkeypatch.setattr(onroad_now_playing.clip, 'end_scissor_mode', lambda: clips.append('end'))
  card.track = TRACK
  card.render(state())
  assert clips == []
  card.track = {**TRACK, 'artist': 'A very long list of featured artists on this one track'}
  line = [card.track['title'], card.track['artist']]
  now = [100.0]
  card.ticks = lambda: now[0]
  texts.clear()
  card.render(state())
  assert texts[1:-2] == line and clips[-1] == 'end'  # held at the start: one copy
  now[0] += onroad_now_playing.SCROLL_PAUSE + 14  # 560 px in: the next copy has come into view
  texts.clear()
  card.render(state())
  assert texts[1:-2] == line * 2 and clips[-1] == 'end'
  card.track = {**card.track, 'artist': card.track['artist'] + ' and more'}
  texts.clear()
  card.render(state())
  assert texts[1:-2] == [card.track['title'], card.track['artist']]  # a new name starts held again
  # The clip is the card between its side paddings.
  assert clips[0] == (10 + 46, 20 + 70 - 22, 640 - 2 * 46, 44)


def test_card_shape_strips_cover_the_card_and_round_its_corners():
  from openpilot.starpilot.ui.onroad_now_playing import pill_rows
  rows = sorted(pill_rows(264, 64))
  assert sum(height for _, height, _ in rows) == pytest.approx(264)
  assert all(a[0] + a[1] == pytest.approx(b[0]) for a, b in zip(rows, rows[1:], strict=False))
  assert rows[0][2] > 50 and rows[-1][2] > 50 and min(inset for *_, inset in rows) == 0


def test_accent_is_the_colorful_part_of_the_cover_and_none_for_gray():
  from openpilot.starpilot.ui.onroad_now_playing import accent_from
  red, gray = SimpleNamespace(r=200, g=40, b=60), SimpleNamespace(r=128, g=128, b=128)
  accent = accent_from([red] * 3 + [gray] * 7, 10)
  assert accent.r == 235 and accent.g < 60 and accent.b < 80
  assert accent_from([gray] * 10, 10) is None


@pytest.mark.parametrize('ms, shown', [(0, '00:00'), (58_900, '00:58'), (269_000, '04:29'), (3_725_000, '62:05')])
def test_times_read_like_the_phone(ms, shown):
  from openpilot.starpilot.ui.onroad_now_playing import clock_text
  assert clock_text(ms) == shown


def test_cover_is_loaded_once_per_cover_and_drawn_in_the_card_shape(widget, monkeypatch):
  from openpilot.starpilot.ui import onroad_now_playing
  rl = onroad_now_playing.rl
  card, drawn, _ = widget
  loads, unloads, steps = [], [], []
  image = SimpleNamespace(width=300, height=300)
  monkeypatch.setattr(rl, 'load_image_from_memory', lambda kind, data, size: loads.append(kind) or image)
  for name in ('image_crop', 'image_resize'):
    monkeypatch.setattr(rl, name, lambda *args, name=name: steps.append(name))
  crops = []
  monkeypatch.setattr(rl, 'image_crop', lambda image, box: steps.append('image_crop') or crops.append(box))
  monkeypatch.setattr(rl, 'load_image_colors', lambda image: [SimpleNamespace(r=200, g=40, b=60)] * (640 * 264))
  monkeypatch.setattr(rl, 'unload_image_colors', lambda colors: steps.append('colors freed'))
  monkeypatch.setattr(rl, 'unload_image', lambda *args: None)
  monkeypatch.setattr(rl, 'load_texture_from_image', lambda image: SimpleNamespace(width=640, height=264, id=len(loads)))
  monkeypatch.setattr(rl, 'set_texture_filter', lambda *args: None)
  monkeypatch.setattr(rl, 'is_window_ready', lambda: True)
  monkeypatch.setattr(rl, 'unload_texture', lambda texture: unloads.append(texture.id))
  card.track = TRACK
  card.artwork = ('https://i.scdn.co/image/a', b'\xff\xd8\xff\xe0 jpeg')
  card.render(state())
  strips = [args[2] for name, args in drawn if name == 'draw_texture_pro' and not hasattr(args[0], 'path')]  # not the logo
  card.render(state())
  assert loads == ['.jpg'] and steps == ['image_crop', 'image_resize', 'colors freed']
  assert sum(dest.height for dest in strips) == pytest.approx(264) and min(dest.x for dest in strips) == 10
  box = crops[0]  # 300 px cover: the middle 124 px band, on whole pixels (fractions corrupt raylib's heap)
  assert (box.x, box.y, box.width, box.height) == (0, 88, 300, 124)
  assert card._cover[2].r == 235  # the seek bar and toggles take the cover's red
  assert not any(name == 'draw_rectangle_rounded' and args[0].height == 264 for name, args in drawn)  # no plain fill
  card.artwork = ('', None)
  drawn.clear()
  card.render(state())
  assert unloads == [1] and count(drawn, 'draw_texture_pro') == 1  # just the Spotify mark


def test_bytes_that_are_not_an_image_get_the_plain_card(widget, monkeypatch):
  from openpilot.starpilot.ui import onroad_now_playing
  card, _, _ = widget
  monkeypatch.setattr(onroad_now_playing.rl, 'load_image_from_memory', lambda *args: pytest.fail('decoded a non-image'))
  card.track = TRACK
  card.artwork = ('https://i.scdn.co/image/a', b'<html>')
  card.render(state())
  assert card._cover == ('https://i.scdn.co/image/a', None, None)


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
  assert texts[1:3] == ['Về', 'Khôi']
