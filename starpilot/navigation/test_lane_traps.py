from openpilot.starpilot.navigation.lane_traps import control_ahead
from openpilot.starpilot.navigation.route_engine import MapboxRouteEngine

STRAIGHT = {"valid": True, "indications": ["straight"]}


def mapbox_route(intersections: list) -> dict:
  # due east along the equator, ~111 m per 0.001 deg: 0 -> 0.03 deg is ~3.3 km
  coords = [[i * 0.001, 0.0] for i in range(31)]
  return {
    "distance": 3330.0, "duration": 120.0,
    "geometry": {"coordinates": coords},
    "legs": [{"steps": [
      {"maneuver": {"type": "depart", "instruction": "Head east", "location": coords[0]},
       "distance": 3330.0, "duration": 120.0, "intersections": intersections},
      {"maneuver": {"type": "arrive", "instruction": "Arrive", "location": coords[-1]},
       "distance": 0.0, "duration": 0.0},
    ]}],
  }


def parse(intersections: list):
  route = MapboxRouteEngine._parse_route(mapbox_route(intersections))
  assert route is not None
  return route


def test_malformed_intersections_do_not_break_the_route():
  route = parse([{"location": [0.010, 0.0], "lanes": ["junk", None]}, "junk", {"traffic_signal": True},
                 {"location": [0.012, 0.0], "traffic_signal": True}])
  control = control_ahead(route, 0.0, 2000.0)
  assert control is not None and control["kind"] == "signal" and 1300.0 < control["distance"] < 1350.0


def test_traffic_controls_are_parsed_and_found_in_route_order():
  route = parse([
    {"location": [0.004, 0.0], "traffic_signal": True},
    {"location": [0.006, 0.0], "lanes": [STRAIGHT, STRAIGHT]},           # lanes only: not a control
    {"location": [0.008, 0.0], "stop_sign": True},
    {"location": [0.020, 0.0], "railway_crossing": True},
    {"location": [0.025, 0.0], "traffic_signal": False},
  ])
  assert [kind for _, kind in route.steps[0].junction_controls] == ["signal", "stop", "rail"]
  first = control_ahead(route, 0.0, 600.0)
  assert first["kind"] == "signal" and 440.0 < first["distance"] < 450.0
  assert control_ahead(route, 500.0, 600.0)["kind"] == "stop"   # the signal is behind
  assert control_ahead(route, 1000.0, 600.0) is None              # the crossing is 1.2 km on
  assert control_ahead(route, 2000.0, 600.0)["kind"] == "rail"
  assert control_ahead(None, 0.0, 600.0) is None
