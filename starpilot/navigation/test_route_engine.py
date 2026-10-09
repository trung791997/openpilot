from cereal import log

from openpilot.starpilot.navigation.route_engine import Coordinate, MapboxRouteEngine, NavigationRoute


def make_route() -> NavigationRoute:
  route_data = {
    "geometry": [
      {"latitude": 0.0, "longitude": 0.0},
      {"latitude": 0.0, "longitude": 0.001},
      {"latitude": 0.0, "longitude": 0.002},
      {"latitude": 0.001, "longitude": 0.002},
    ],
    "steps": [
      {
        "maneuver": "depart",
        "instruction": "Head east",
        "distance": 220.0,
        "duration": 20.0,
        "location": {"latitude": 0.0, "longitude": 0.0},
        "modifier": "straight",
        "bannerInstructions": [
          {
            "distanceAlongGeometry": 80.0,
            "primary": {
              "text": "Turn right",
              "type": "turn",
              "modifier": "right",
            },
            "secondary": {"text": "onto Market St"},
          },
        ],
      },
      {
        "maneuver": "turn",
        "instruction": "Turn right onto Market St",
        "distance": 111.0,
        "duration": 10.0,
        "location": {"latitude": 0.0, "longitude": 0.002},
        "modifier": "right",
        "bannerInstructions": [],
      },
      {
        "maneuver": "arrive",
        "instruction": "Your destination is on the right",
        "distance": 0.0,
        "duration": 0.0,
        "location": {"latitude": 0.001, "longitude": 0.002},
        "modifier": "right",
        "bannerInstructions": [],
      },
    ],
    "totalDistance": 331.0,
    "totalDuration": 30.0,
    "maxspeed": [
      {"speed": 35.0, "unit": "mph"},
      {"speed": 35.0, "unit": "mph"},
      {"speed": 25.0, "unit": "mph"},
      {"speed": 25.0, "unit": "mph"},
    ],
  }
  route = NavigationRoute.from_mapbox_route(route_data)
  assert route is not None
  return route


def test_route_progress_and_instruction_payload():
  route = make_route()
  progress = route.get_progress(Coordinate(0.0, 0.0015))

  assert progress is not None
  assert progress.current_step_index == 0
  assert progress.distance_remaining > 0
  assert route.upcoming_turn_modifier(progress, Coordinate(0.0, 0.0015), 20.0) == "right"

  payload = route.build_instruction_payload(progress)
  assert payload["maneuverPrimaryText"] == "Turn right"
  assert payload["maneuverSecondaryText"] == "onto Market St"
  assert payload["maneuverType"] == "turn"
  assert payload["maneuverModifier"] == "right"


def test_route_off_route_and_arrival_detection():
  route = make_route()
  off_route_progress = route.get_progress(Coordinate(0.0020, 0.0010))
  misaligned_progress = route.get_progress(Coordinate(0.0, 0.0015))
  arrive_progress = route.get_progress(Coordinate(0.0010, 0.0020))

  assert off_route_progress is not None
  assert misaligned_progress is not None
  assert arrive_progress is not None
  assert route.off_route_distance_exceeded(off_route_progress, 10.0)
  assert route.route_bearing_misaligned(misaligned_progress.closest_segment_index, 270.0, 10.0)
  assert route.arrived(arrive_progress, 0.5)


def test_route_progress_projects_onto_segments_for_destination_step():
  route = make_route()
  progress = route.get_progress(Coordinate(0.00098, 0.0020))

  assert progress is not None
  assert progress.next_step is not None
  assert progress.next_step.maneuver == "arrive"
  assert progress.distance_remaining <= 5.0
  assert route.arrived(progress, 0.5)


def test_route_bearing_misaligned_catches_ninety_degree_wrong_turn():
  route = make_route()
  progress = route.get_progress(Coordinate(0.0, 0.0015))

  assert progress is not None
  assert route.route_bearing_misaligned(progress.closest_segment_index, 0.0, 6.0)


def test_lane_payload_uses_capnp_enum_names():
  route_data = {
    "geometry": [
      {"latitude": 0.0, "longitude": 0.0},
      {"latitude": 0.0, "longitude": 0.001},
      {"latitude": 0.0, "longitude": 0.002},
    ],
    "steps": [
      {
        "maneuver": "turn",
        "instruction": "Keep left",
        "distance": 120.0,
        "duration": 12.0,
        "location": {"latitude": 0.0, "longitude": 0.001},
        "modifier": "slight left",
        "bannerInstructions": [
          {
            "distanceAlongGeometry": 80.0,
            "primary": {
              "text": "Keep left",
              "type": "turn",
              "modifier": "slight left",
            },
            "sub": {
              "components": [
                {
                  "type": "lane",
                  "active": True,
                  "directions": ["straight", "slight left"],
                  "active_direction": "slight left",
                },
              ],
            },
          },
        ],
      },
    ],
    "totalDistance": 120.0,
    "totalDuration": 12.0,
  }

  route = NavigationRoute.from_mapbox_route(route_data)
  assert route is not None
  progress = route.get_progress(Coordinate(0.0, 0.0005))
  assert progress is not None

  payload = route.build_instruction_payload(progress)
  msg = log.Event.new_message()
  msg.init("navInstruction")
  msg.navInstruction.lanes = payload["lanes"]

  assert len(msg.navInstruction.lanes) == 1
  assert list(msg.navInstruction.lanes[0].directions) == [
    log.NavInstruction.Direction.straight,
    log.NavInstruction.Direction.slightLeft,
  ]
  assert msg.navInstruction.lanes[0].activeDirection == log.NavInstruction.Direction.slightLeft


def mapbox_route(duration: float, end_longitude: float) -> dict:
  return {
    "distance": 1000.0,
    "duration": duration,
    "geometry": {"coordinates": [[0.0, 0.0], [end_longitude, 0.0]]},
    "legs": [{
      "steps": [
        {"maneuver": {"type": "depart", "instruction": "Head east", "location": [0.0, 0.0]}, "distance": 1000.0, "duration": duration},
        {"maneuver": {"type": "arrive", "instruction": "Arrive", "location": [end_longitude, 0.0]}, "distance": 0.0, "duration": 0.0},
      ],
    }],
  }


class DirectionsSession:
  def __init__(self, payload, status_code=200):
    self.payload = payload
    self.status_code = status_code
    self.params = None

  def get(self, url, params=None, timeout=None):
    self.params = params
    session = self

    class Response:
      status_code = session.status_code

      def json(self):
        return session.payload

    return Response()


def test_fetch_routes_returns_main_route_then_alternatives():
  session = DirectionsSession({"code": "Ok", "routes": [mapbox_route(600.0, 0.01), mapbox_route(720.0, 0.011)]})
  routes = MapboxRouteEngine(session).fetch_routes("token", Coordinate(0.0, 0.0), {"latitude": 0.0, "longitude": 0.01})
  assert [route.total_duration for route in routes] == [600.0, 720.0]
  assert session.params["alternatives"] == "true"


def test_fetch_route_picks_the_requested_alternative():
  session = DirectionsSession({"code": "Ok", "routes": [mapbox_route(600.0, 0.01), mapbox_route(720.0, 0.011)]})
  engine = MapboxRouteEngine(session)
  destination = {"latitude": 0.0, "longitude": 0.01, "routeId": "alt-1"}
  assert engine.fetch_route("token", Coordinate(0.0, 0.0), destination).total_duration == 720.0
  destination["routeId"] = "main"
  assert engine.fetch_route("token", Coordinate(0.0, 0.0), destination).total_duration == 600.0
  assert session.params["alternatives"] == "false"


def test_fetch_routes_handles_errors():
  destination = {"latitude": 0.0, "longitude": 0.01}
  assert MapboxRouteEngine(DirectionsSession({"code": "NoRoute", "routes": []})).fetch_routes("token", Coordinate(0.0, 0.0), destination) == []
  assert MapboxRouteEngine(DirectionsSession({}, status_code=401)).fetch_routes("token", Coordinate(0.0, 0.0), destination) == []
  assert MapboxRouteEngine(DirectionsSession({})).fetch_routes("", Coordinate(0.0, 0.0), destination) == []


def test_fetch_routes_applies_exclude_parameters_for_tolls_highways_ferries():
  session = DirectionsSession({"code": "Ok", "routes": [mapbox_route(600.0, 0.01)]})
  engine = MapboxRouteEngine(session)

  # Avoid tolls only
  engine.fetch_routes("token", Coordinate(0.0, 0.0), {"latitude": 0.0, "longitude": 0.01, "avoid_tolls": True})
  assert session.params["exclude"] == "toll"

  # Avoid tolls and highways
  engine.fetch_routes("token", Coordinate(0.0, 0.0), {
    "latitude": 0.0,
    "longitude": 0.01,
    "avoid_tolls": True,
    "avoid_highways": True,
  })
  assert session.params["exclude"] == "toll,motorway"

  # Avoid all three: tolls, highways, ferries
  engine.fetch_routes("token", Coordinate(0.0, 0.0), {
    "latitude": 0.0,
    "longitude": 0.01,
    "avoid_tolls": True,
    "avoid_highways": True,
    "avoid_ferries": True,
  })
  assert session.params["exclude"] == "toll,motorway,ferry"


def test_fetch_routes_prefer_eco_ranks_fuel_efficient_route_first():
  # Route 1: 15 miles (24140 m) highway route in 15 min (900 s) -> ~26.8 m/s (60 mph) -> higher aero drag & distance
  route_highway = {
    "distance": 24140.0,
    "duration": 900.0,
    "geometry": {"coordinates": [[0.0, 0.0], [0.1, 0.0]]},
    "legs": [{
      "steps": [
        {"maneuver": {"type": "depart", "instruction": "Highway", "location": [0.0, 0.0]}, "distance": 24140.0, "duration": 900.0},
        {"maneuver": {"type": "arrive", "instruction": "Arrive", "location": [0.1, 0.0]}, "distance": 0.0, "duration": 0.0},
      ],
    }],
  }
  # Route 2: 10 miles (16093 m) local route in 16 min (960 s) -> ~16.7 m/s (37 mph) -> lower aero drag & shorter distance
  route_local = {
    "distance": 16093.0,
    "duration": 960.0,
    "geometry": {"coordinates": [[0.0, 0.0], [0.1, 0.0]]},
    "legs": [{
      "steps": [
        {"maneuver": {"type": "depart", "instruction": "Local", "location": [0.0, 0.0]}, "distance": 16093.0, "duration": 960.0},
        {"maneuver": {"type": "arrive", "instruction": "Arrive", "location": [0.1, 0.0]}, "distance": 0.0, "duration": 0.0},
      ],
    }],
  }

  session = DirectionsSession({"code": "Ok", "routes": [route_highway, route_local]})
  engine = MapboxRouteEngine(session)

  # Without prefer_eco, route order is unchanged (highway first)
  routes_standard = engine.fetch_routes("token", Coordinate(0.0, 0.0), {"latitude": 0.0, "longitude": 0.1})
  assert routes_standard[0].total_distance == 24140.0

  # With prefer_eco, the more fuel-efficient local route is promoted to index 0 and marked eco recommended
  routes_eco = engine.fetch_routes("token", Coordinate(0.0, 0.0), {"latitude": 0.0, "longitude": 0.1, "prefer_eco": True})
  assert routes_eco[0].total_distance == 16093.0
  assert routes_eco[0].is_eco_recommended is True
  assert routes_eco[0].eco_savings_pct > 0.0



def single_step_route(distance: float, duration: float) -> dict:
  return {
    "distance": distance,
    "duration": duration,
    "geometry": {"coordinates": [[0.0, 0.0], [0.1, 0.0]]},
    "legs": [{
      "steps": [
        {"maneuver": {"type": "depart", "instruction": "Go", "location": [0.0, 0.0]}, "distance": distance, "duration": duration},
        {"maneuver": {"type": "arrive", "instruction": "Arrive", "location": [0.1, 0.0]}, "distance": 0.0, "duration": 0.0},
      ],
    }],
  }


def test_fetch_route_prefer_eco_main_requests_alternatives():
  # "main" with prefer_eco must see the alternatives, or it cannot pick the eco route the previews showed.
  session = DirectionsSession({"code": "Ok", "routes": [single_step_route(24140.0, 900.0), single_step_route(16093.0, 960.0)]})
  engine = MapboxRouteEngine(session)
  route = engine.fetch_route("token", Coordinate(0.0, 0.0), {"latitude": 0.0, "longitude": 0.1, "routeId": "main", "prefer_eco": True})
  assert session.params["alternatives"] == "true"
  assert route.total_distance == 16093.0
  assert route.is_eco_recommended is True


def test_fetch_route_prefer_eco_alt_indexes_reordered_list():
  # Mapbox order [A highway, B longer highway, C local]; C is the eco pick, so the list shown is [C, A, B].
  routes = [single_step_route(24140.0, 900.0), single_step_route(30000.0, 1100.0), single_step_route(16093.0, 960.0)]
  session = DirectionsSession({"code": "Ok", "routes": routes})
  engine = MapboxRouteEngine(session)
  destination = {"latitude": 0.0, "longitude": 0.1, "prefer_eco": True}
  shown = engine.fetch_routes("token", Coordinate(0.0, 0.0), destination)
  assert [r.total_distance for r in shown] == [16093.0, 24140.0, 30000.0]
  for index, expected in ((0, 16093.0), (1, 24140.0), (2, 30000.0)):
    route_id = "main" if index == 0 else f"alt-{index}"
    picked = engine.fetch_route("token", Coordinate(0.0, 0.0), {**destination, "routeId": route_id})
    assert picked.total_distance == expected
    assert session.params["alternatives"] == "true"
