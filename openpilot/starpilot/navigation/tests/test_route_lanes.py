"""Lane guidance and the "toward" line for the head unit's dash map (display only).

The steps follow the Mapbox Directions v5 format: banner `sub` lane components and the maneuver's
intersection `lanes` (docs.mapbox.com/api/navigation/directions, "Lane object" and "Banner instruction object").
"""
import copy

from openpilot.starpilot.navigation.route_engine import LANE_MERGE_RIGHT, NavigationRoute, lanes, toward

STRAIGHT, RIGHT, SLIGHT_RIGHT, LEFT, UTURN = 1, 4, 2, 64, 16

# I-75 north, 1.1 km before exit 34 on the right: four lanes, the two right ones go to the exit
HIGHWAY = [
  {'distance': 1112., 'duration': 40., 'name': 'North Central Expressway', 'ref': 'US 75',
   'maneuver': {'type': 'depart', 'location': [0., 0.]},
   'bannerInstructions': [
     {'distanceAlongGeometry': 1112., 'primary': {'text': 'Exit 34 I 635 East', 'type': 'off ramp', 'modifier': 'slight right'},
      'secondary': {'text': 'Garland', 'components': [{'text': 'Garland', 'type': 'text'}]}},
     {'distanceAlongGeometry': 400., 'primary': {'text': 'Exit 34 I 635 East', 'type': 'off ramp', 'modifier': 'slight right'},
      'secondary': {'text': 'Garland', 'components': [{'text': 'Garland', 'type': 'text'}]},
      'sub': {'text': '', 'components': [
        {'type': 'lane', 'text': '', 'directions': ['straight'], 'active': False},
        {'type': 'lane', 'text': '', 'directions': ['straight'], 'active': False},
        {'type': 'lane', 'text': '', 'directions': ['straight', 'slight right'], 'active': True, 'active_direction': 'slight right'},
        {'type': 'lane', 'text': '', 'directions': ['slight right'], 'active': True, 'active_direction': 'slight right'}]}}]},
  {'distance': 0., 'duration': 0., 'name': 'I 635', 'destinations': 'I 635 East: Garland',
   'maneuver': {'type': 'off ramp', 'modifier': 'slight right', 'location': [.01, 0.]},
   'intersections': [{'location': [.01, 0.], 'lanes': [
     {'valid': False, 'active': False, 'indications': ['straight']},
     {'valid': True, 'active': True, 'indications': ['slight right'], 'valid_indication': 'slight right'}]}]},
]
GEOMETRY = {'distance': 1112., 'duration': 40., 'geometry': {'coordinates': [[0., 0.], [.01, 0.]]}}


def instruction(steps):
  return NavigationRoute({**GEOMETRY, 'legs': [{'steps': steps}]}).progress((.005, 0.), 25., 90.).instruction


def test_the_banner_lanes_reach_the_instruction_left_to_right():
  result = instruction(copy.deepcopy(HIGHWAY))
  assert result['lanes'] == [
    {'directions': STRAIGHT, 'active': False, 'activeDirection': 0},
    {'directions': STRAIGHT, 'active': False, 'activeDirection': 0},
    {'directions': STRAIGHT | SLIGHT_RIGHT, 'active': True, 'activeDirection': SLIGHT_RIGHT},
    {'directions': SLIGHT_RIGHT, 'active': True, 'activeDirection': SLIGHT_RIGHT}]
  assert result['toward'] == 'Garland'


def test_without_banner_lanes_the_maneuver_intersection_lanes_are_used():
  steps = copy.deepcopy(HIGHWAY)
  del steps[0]['bannerInstructions'][1]['sub']
  assert instruction(steps)['lanes'] == [{'directions': STRAIGHT, 'active': False, 'activeDirection': 0},
                                         {'directions': SLIGHT_RIGHT, 'active': True, 'activeDirection': SLIGHT_RIGHT}]
  # an intersection lane without `active` counts as one to use when it is valid
  del steps[1]['intersections'][0]['lanes'][1]['active']
  assert instruction(steps)['lanes'][1]['active'] is True


def test_no_lanes_given_means_none_sent():
  steps = copy.deepcopy(HIGHWAY)
  del steps[0]['bannerInstructions'][1]['sub']
  del steps[1]['intersections']
  assert instruction(steps)['lanes'] == []
  steps[1]['intersections'] = []
  assert instruction(steps)['lanes'] == []
  steps[1]['intersections'] = [{'lanes': []}]
  assert instruction(steps)['lanes'] == []


def test_a_city_left_turn_with_a_shared_lane_and_an_unpainted_lane():
  banners = [{'primary': {'text': 'Main Street', 'type': 'turn', 'modifier': 'left'},
              'sub': {'components': [{'type': 'lane', 'directions': ['left', 'uturn'], 'active': True, 'active_direction': 'left'},
                                     {'type': 'lane', 'directions': ['left', 'straight'], 'active': True},
                                     {'type': 'lane', 'directions': ['none'], 'active': False},
                                     {'type': 'lane', 'directions': ['right'], 'active': False}]}}]
  assert lanes(banners, {}, 'turn', 'left') == ((LEFT | UTURN, True, LEFT), (LEFT | STRAIGHT, True, STRAIGHT),
                                                (STRAIGHT, False, 0), (RIGHT, False, 0))


def test_a_lane_to_use_gets_the_arrow_it_carries():
  # the highlighted arrow is always one painted on the lane: unknown or missing active_direction -> its first arrow
  banners = [{'sub': {'components': [{'type': 'lane', 'directions': ['right', 'straight'], 'active': True,
                                      'active_direction': 'left'}]}}]
  assert lanes(banners, {}, 'turn', 'right') == ((RIGHT | STRAIGHT, True, STRAIGHT),)


def test_merge_lanes_get_the_merge_arrow():
  banners = [{'sub': {'components': [{'type': 'lane', 'directions': ['straight'], 'active': False},
                                     {'type': 'lane', 'directions': ['straight'], 'active': True}]}}]
  assert lanes(banners, {}, 'merge', 'slightRight') == ((STRAIGHT, False, 0),
                                                        (STRAIGHT | LANE_MERGE_RIGHT, True, LANE_MERGE_RIGHT))


def test_malformed_or_too_many_lanes_are_dropped_whole():
  bad = [{'sub': {'components': [{'type': 'lane', 'directions': ['straight'], 'active': True},
                                 {'type': 'lane', 'directions': 'right', 'active': False}]}}]
  assert lanes(bad, {}, 'turn', 'right') == ()
  unknown = [{'sub': {'components': [{'type': 'lane', 'directions': ['sideways'], 'active': True}]}}]
  assert lanes(unknown, {}, 'turn', 'right') == ()
  many = [{'sub': {'components': [{'type': 'lane', 'directions': ['straight'], 'active': True}] * 13}}]
  assert lanes(many, {}, 'continue', 'straight') == ()
  assert lanes('nonsense', {'intersections': 'nonsense'}, 'turn', 'left') == ()


def test_toward_comes_from_the_banner_else_the_destinations():
  assert toward([{'secondary': {'text': '  Garland  Mesquite '}}], {}, 'I 635 East') == 'Garland Mesquite'
  assert toward([{}], {'destinations': 'I 635 East: Garland, Mesquite'}, 'I 635 East') == 'Garland, Mesquite'
  assert toward([], {'destinations': 'Downtown'}, 'Exit 3') == 'Downtown'
  assert toward([], {}, 'Main Street') == ''
  # already in the instruction: not repeated
  assert toward([{'secondary': {'text': 'Garland'}}], {}, 'Exit 34 toward Garland') == ''
  assert toward([{'secondary': {'text': 'Garland'}}], {}, 'Garland Road') == ''
  assert len(toward([{'secondary': {'text': 'x' * 200}}], {}, '')) == 80


def test_the_arrive_step_has_no_lanes_or_destination():
  steps = copy.deepcopy(HIGHWAY)
  steps[1]['bannerInstructions'] = copy.deepcopy(steps[0]['bannerInstructions'])
  route = NavigationRoute({**GEOMETRY, 'legs': [{'steps': steps}]})
  assert (route.steps[-1].lanes, route.steps[-1].toward) == ((), '')
  assert route.steps[0].lanes and route.steps[0].toward == 'Garland'


def test_the_lanes_fit_the_navigation_message():
  from openpilot.cereal import messaging
  from openpilot.starpilot.navigation.wire import navigation_envelope
  message = messaging.new_message('starpilotNavigation', valid=True)
  message.starpilotNavigation = navigation_envelope({'instruction': instruction(copy.deepcopy(HIGHWAY))})
  sent = message.starpilotNavigation.navigation.instruction
  assert [(lane.directions, lane.active, lane.activeDirection) for lane in sent.lanes] == [
    (1, False, 0), (1, False, 0), (3, True, 2), (2, True, 2)]
  assert sent.toward == 'Garland'
