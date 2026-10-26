from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

import pytest

from openpilot.starpilot.ui.onroad_navigation_favorites import NavigationFavorites


@pytest.mark.parametrize('key', ['nav_home', 'nav_work', 'bookmark'])
@pytest.mark.parametrize('display', ['icons', 'words'])
@pytest.mark.parametrize('background', [True, False])
@pytest.mark.parametrize('opacity', [0, 40, 100])
def test_appearance_scales_ink_and_optional_background(key, display, background, opacity):
  fonts = Mock()
  fonts.measure.return_value = NS(width=10)
  card = NavigationFavorites(fonts)
  state = NS(customization={'layouts': {'large': {key: {'x': 100, 'y': 200, 'enabled': True,
                                                      'display': display, 'opacity': opacity, 'background': background}}}},
             alert=NS(size='none'))
  # A missing favorite also exercises the status badge's opacity and background.
  with patch('pyray.draw_rectangle_rounded') as fill, patch('pyray.draw_rectangle_rounded_lines_ex') as border, \
       patch('pyray.draw_line_ex') as lines, patch('pyray.draw_circle') as badge, \
       patch('pyray.draw_circle_v') as disc, patch('pyray.draw_ring') as ring:
    card.render(key, state)
  if display == 'icons':
    # Icons are round glass buttons, like the driver-monitor circle and the now-playing card.
    assert fill.call_count == border.call_count == 0
    assert disc.call_count == ring.call_count == int(background)
    if background:
      assert disc.call_args.args[-1].a == round(225 * opacity / 100)
      assert ring.call_args.args[-1].a == round(40 * opacity / 100)
  else:
    assert disc.call_count == ring.call_count == 0
    assert fill.call_count == border.call_count == int(background)
  if background and display == 'words':
    assert fill.call_args.args[-1].a == round(245 * opacity / 100)
    assert border.call_args.args[-1].a == round((255 if key == 'bookmark' else 170) * opacity / 100)
  assert all(call.args[-1].a == round(255 * opacity / 100) for call in fonts.draw.call_args_list)
  if display == 'icons':
    assert lines.call_count > 0
    assert all(call.args[-1].a == round(255 * opacity / 100) for call in lines.call_args_list)
    assert badge.call_count == int(background and key != 'bookmark')
    if background and key != 'bookmark':
      assert badge.call_args.args[-1].a == round(255 * opacity / 100)


@pytest.mark.parametrize('display', ['words', 'icons'])
def test_bookmark_stays_visible_during_navigation_and_needs_no_favorite(display):
  fonts = Mock()
  fonts.measure.return_value = NS(width=10)
  card = NavigationFavorites(fonts)
  placed = {'x': 100, 'y': 200, 'enabled': True, 'display': display, 'opacity': 50, 'background': False}
  state = NS(customization={'layouts': {'large': {'bookmark': placed, 'nav_home': {**placed, 'enabled': True}}}},
             alert=NS(size='none'))
  home = {'id': 'home', 'label': 'home'}
  card.document = {'favorites': [home], 'destination': home, 'enabled': True, 'token': 'key'}
  with patch('pyray.draw_rectangle_rounded') as background, patch('pyray.draw_rectangle_rounded_lines_ex'), \
       patch('pyray.draw_line_ex') as lines, patch('pyray.draw_circle_v') as disc:
    card.render('bookmark', state)
  background.assert_not_called()
  disc.assert_not_called()
  assert [call.args[0] for call in fonts.draw.call_args_list] == (['Bookmark', 'Save event'] if display == 'words' else [])
  if display == 'icons':
    assert lines.call_count == 5
  fonts.draw.reset_mock()
  placed['enabled'] = False
  card.render('bookmark', state)
  fonts.draw.assert_not_called()


@pytest.mark.parametrize('key', ['nav_home', 'nav_work'])
def test_card_changes_to_end_navigation_only_for_its_destination(key):
  fonts = Mock()
  fonts.measure.return_value = NS(width=200)
  card = NavigationFavorites(fonts)
  state = NS(customization={'layouts': {'large': {key: {'x': 100, 'y': 200, 'enabled': True}}}}, alert=NS(size='none'))
  label = key.removeprefix('nav_')
  place = {'id': label, 'label': label}
  card.document = {'favorites': [place], 'destination': None, 'enabled': True, 'token': 'key'}
  with patch('pyray.draw_rectangle_rounded'), patch('pyray.draw_rectangle_rounded_lines_ex'):
    card.render(key, state)
    assert [call.args[0] for call in fonts.draw.call_args_list] == ['Navigate', label.title()]
    assert fonts.draw.call_args_list[0].args[4] < fonts.draw.call_args_list[1].args[4]
    fonts.draw.reset_mock()
    card.document['destination'] = place
    card.render(key, state)
    assert [call.args[0] for call in fonts.draw.call_args_list] == ['End route', label.title()]
    fonts.draw.reset_mock()
    card.document['destination'] = {'id': 'another-place'}
    card.render(key, state)
    assert [call.args[0] for call in fonts.draw.call_args_list] == ['Navigate', label.title()]
    for change in ['alert', 'removed']:
      fonts.draw.reset_mock()
      state.alert.size = 'full' if change == 'alert' else 'none'
      state.customization['layouts']['large'][key]['enabled'] = change != 'removed'
      card.render(key, state)
      fonts.draw.assert_not_called()


def test_missing_favorite_shows_setup_hint():
  fonts = Mock()
  fonts.measure.return_value = NS(width=200)
  card = NavigationFavorites(fonts)
  state = NS(customization={'layouts': {'large': {'nav_home': {'x': 100, 'y': 200, 'enabled': True}}}}, alert=NS(size='none'))
  with patch('pyray.draw_rectangle_rounded'), patch('pyray.draw_rectangle_rounded_lines_ex'):
    card.render('nav_home', state)
  assert [call.args[0] for call in fonts.draw.call_args_list] == ['Set in Galaxy', 'Home']


@pytest.mark.parametrize('key', ['nav_home', 'nav_work'])
@pytest.mark.parametrize('status,badge', [('ready', ''), ('active', ''), ('missing', '+'), ('disabled', '!'), ('error', '!')])
def test_icon_rendering_preserves_navigation_status_without_word_card(key, status, badge):
  fonts = Mock()
  fonts.measure.return_value = NS(width=10)
  card = NavigationFavorites(fonts)
  state = NS(customization={'layouts': {'large': {key: {'x': 100, 'y': 200, 'enabled': True, 'display': 'icons'}}}},
             alert=NS(size='none'))
  label = key.removeprefix('nav_')
  place = {'id': label, 'label': label}
  card.document = {'favorites': [] if status == 'missing' else [place], 'destination': place if status == 'active' else None,
                   'enabled': status != 'disabled', 'token': 'key'}
  card.error = 'Try again' if status == 'error' else ''
  with patch('pyray.draw_circle_v') as background, patch('pyray.draw_ring') as ring, \
       patch('pyray.draw_line_ex') as lines, patch('pyray.draw_circle'):
    card.render(key, state)
  center, radius = background.call_args.args[:2]
  assert (center.x, center.y, radius) == (155, 255, 55)
  assert (ring.call_args.args[-1].r, ring.call_args.args[-1].g) == ((240, 150) if status == 'active' else (255, 255))
  assert lines.call_count > 0
  if status == 'active':
    assert lines.call_count == 2, 'the favorite icon is replaced by a full X'
  assert [call.args[0] for call in fonts.draw.call_args_list] == ([badge] if badge else [])
  ink = lines.call_args.args[-1]
  assert (ink.r, ink.g) == ((240, 150) if status == 'active' else (160, 157) if status in ('disabled', 'missing') else (240, 238))


@pytest.mark.parametrize('active', ['home', 'work'])
@pytest.mark.parametrize('display', ['words', 'icons'])
def test_other_favorite_is_hidden_until_route_ends(active, display):
  fonts = Mock()
  fonts.measure.return_value = NS(width=200)
  card = NavigationFavorites(fonts)
  placements = {f'nav_{label}': {'x': 100, 'y': 200, 'enabled': True, 'display': display} for label in ('home', 'work')}
  state = NS(customization={'layouts': {'large': placements}}, alert=NS(size='none'))
  places = [{'id': label, 'label': label} for label in ('home', 'work')]
  card.document = {'favorites': places, 'destination': next(p for p in places if p['id'] == active), 'enabled': True, 'token': 'key'}
  other = 'nav_work' if active == 'home' else 'nav_home'
  with patch('pyray.draw_rectangle_rounded') as card_fill, patch('pyray.draw_rectangle_rounded_lines_ex'), \
       patch('pyray.draw_circle_v') as icon_fill, patch('pyray.draw_ring'), patch('pyray.draw_line_ex'):
    background = icon_fill if display == 'icons' else card_fill
    card.render(other, state)
    background.assert_not_called()
    # Ending the route restores the other button without changing the saved layout.
    card.document['destination'] = None
    card.render(other, state)
    background.assert_called_once()
