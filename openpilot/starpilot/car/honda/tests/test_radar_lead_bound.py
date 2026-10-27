"""Unconfirmed / passed lead bound and its STATUS carrier (static). Shapes from route 34 T479 (STATUS 9) and route 37
T903 (the model lead moved on past the radar lead and calls for no braking)."""
from types import SimpleNamespace

import pytest

from opendbc.car import structs
from openpilot.cereal import messaging
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.car.card import Car
from openpilot.selfdrive.controls.radard import RadarD
from openpilot.starpilot.car.honda import radar_point_status
from openpilot.starpilot.car.honda.radar_lead_bound import (CAMERA_CLOSING_MAX, CAMERA_DECEL_MIN, DISTANCE_W, RELEASE_S,
                                                           RISE_S, UNCONFIRMED_W, WeakLeadBound, camera_passed_ratio,
                                                           tell_weight)
from openpilot.starpilot.car.honda.radar_reversal_bound import W, ReversalBound
from openpilot.starpilot.car.honda.tests.test_radar_reversal_bound import ABSENT, V_EGO, radar_lead, reverse, vision


def test_tell_weight():
  assert tell_weight(9, 0.0) == UNCONFIRMED_W
  assert tell_weight(7, 0.62) == 0.0  # the largest ratio among the four real far brakes
  assert tell_weight(0, 0.62) == 0.0  # status not known
  assert tell_weight(3, 0.0) == 0.0   # only 9 counts as unconfirmed
  assert tell_weight(7, 0.875) == pytest.approx(DISTANCE_W / 2)
  assert tell_weight(7, 1.02) == DISTANCE_W  # route 37 at the start of the tap
  assert tell_weight(9, 1.02) == max(UNCONFIRMED_W, DISTANCE_W)


def test_camera_passed_ratio():
  # route 37 T903: SUV at 45 m, the model lead moved on to 62 m, closing 0.5 m/s, a 0
  assert camera_passed_ratio(45.0, 62.0, 0.5, 0.0) == pytest.approx(17.0 / 15.5)
  assert camera_passed_ratio(45.0, 62.0, CAMERA_CLOSING_MAX, CAMERA_DECEL_MIN) == pytest.approx(17.0 / 15.5)
  # route 37 T273: a pickup slowing to turn left at 82 m, the model lead at 109 m closing at 11.7 m/s: a real slowdown
  assert camera_passed_ratio(81.7, 108.8, 11.7, -0.3) == 0.0
  assert camera_passed_ratio(45.0, 62.0, CAMERA_CLOSING_MAX + 0.1, 0.0) == 0.0  # the model lead calls for braking
  assert camera_passed_ratio(45.0, 62.0, 0.5, CAMERA_DECEL_MIN - 0.1) == 0.0
  assert camera_passed_ratio(36.0, 27.5, 0.0, 0.0) == 0.0  # the model lead is nearer than the radar lead
  assert camera_passed_ratio(16.0, 18.0, 0.0, 0.0) == pytest.approx(0.4)  # the 5 m floor


def update(bound, lead, status=7, ratio=0.0, prob=0.95):
  return bound.update([lead, ABSENT], [status, 0], [ratio, 0.0], [prob, 0.0])[0]


def test_new_track_takes_its_tell_at_once():
  bound = WeakLeadBound()
  assert update(bound, radar_lead(55, 66.0, 15.0, -3.0), status=9) == UNCONFIRMED_W
  assert update(bound, radar_lead(56, 66.0, 15.0, -3.0), status=7) == 0.0  # and a new confirmed one drops it at once


def test_established_track_rises_and_releases_at_the_slew_rates():
  bound = WeakLeadBound()
  lead = radar_lead(32, 47.0, 28.7, 0.0)
  assert update(bound, lead, ratio=0.5) == 0.0
  rise = DISTANCE_W * DT_MDL / RISE_S
  assert update(bound, lead, ratio=1.1) == pytest.approx(rise)
  for _ in range(round(RISE_S / DT_MDL)):
    weight = update(bound, lead, ratio=1.1)
  assert weight == pytest.approx(DISTANCE_W)
  release = DISTANCE_W * DT_MDL / RELEASE_S
  assert update(bound, lead, ratio=0.5) == pytest.approx(DISTANCE_W - release)
  for _ in range(round(RELEASE_S / DT_MDL)):
    weight = update(bound, lead, ratio=0.5)
  assert weight == pytest.approx(0.0, abs=1e-9)


def test_urgent_unwitnessed_and_vision_leads_get_no_weight():
  bound = WeakLeadBound()
  assert update(bound, radar_lead(55, 29.0, 15.0, -3.0), status=9) == 0.0           # range under 30 m
  assert update(bound, radar_lead(55, 66.0, V_EGO - 17.0, -3.0), status=9) == 0.0   # TTC 3.9 s
  assert update(bound, radar_lead(55, 66.0, 15.0, -3.0), status=9, prob=0.2) == 0.0
  vision_lead = {'present': True, 'radar': False, 'radarTrackId': -1, 'dRel': 66.0, 'vRel': -7.0,
                 'vLead': 15.0, 'vLeadK': 15.0, 'aLeadK': -3.0}
  assert update(bound, vision_lead, status=9) == 0.0


def test_floor_weight_blends_once_at_the_larger_weight():
  raw = radar_lead(32, 47.0, 24.8, -2.9)  # route 37: vLead dipped to 24.8 while the model lead held 29, a 0
  model = vision(29.0, 0.0)
  rb = ReversalBound()
  out = rb.apply([raw, ABSENT], [model, vision()], V_EGO, V_EGO, [0.95, 0.0], {32: None}, floor_weights=(0.3, 0.0))[0]
  assert out['vLead'] == pytest.approx(24.8 + 0.3 * (29.0 - 24.8))
  assert out['vRel'] == pytest.approx(out['vLead'] - V_EGO)
  assert out['aLeadK'] == pytest.approx(-2.9 + 0.3 * 2.9)
  assert out['dRel'] == raw['dRel'] and out['present'] and out['radar'] and rb.weights[0] == 0.3
  rb = ReversalBound()
  reverse(rb, tid=32, d=47.0)
  out = rb.apply([raw, ABSENT], [model, vision()], V_EGO, V_EGO, [0.95, 0.0], {32: None}, floor_weights=(0.3, 0.0))[0]
  assert rb.weights[0] == W and out['vLead'] == pytest.approx(24.8 + W * (29.0 - 24.8))


class FakeSM(dict):
  def __init__(self, events):
    super().__init__({event.which(): getattr(event, event.which()) for event in events})
    self.logMonoTime = {event.which(): event.logMonoTime for event in events}


def companion(stamp, statuses, tracks_stamp=None):
  event = messaging.new_message('starpilotCarState', valid=True)
  event.logMonoTime = stamp
  radar_point_status.write(event.starpilotCarState, stamp if tracks_stamp is None else tracks_stamp, statuses)
  return event.as_reader()


def tracks(stamp):
  event = messaging.new_message('radarTracks', valid=True)
  event.logMonoTime = stamp
  return event.as_reader()


def test_point_status_round_trip_and_pairing():
  stamp = 5_000_000_000
  assert radar_point_status.read(FakeSM([tracks(stamp), companion(stamp, {60: 7, 55: 9})])) == {55: 9, 60: 7}
  late = stamp + radar_point_status.PAIR_TOLERANCE_NS + 1
  assert radar_point_status.read(FakeSM([tracks(late), companion(stamp, {55: 9})])) == {}
  assert radar_point_status.read(FakeSM([tracks(stamp), companion(stamp, {55: 9}, tracks_stamp=0)])) == {}
  assert radar_point_status.read(FakeSM([tracks(stamp)])) == {}  # no carrier: other radars, older logs


class FakePM:
  def __init__(self):
    self.sent = []

  def send(self, service, message):
    self.sent.append((service, message.as_reader() if hasattr(message, 'as_reader') else message))


def one_point():
  rr = structs.RadarData()
  point = rr.init('points', 1)[0]
  point.trackId = 55
  point.dRel = 66.0
  return rr


def test_card_sends_the_statuses_after_the_tracks_they_describe():
  car = SimpleNamespace(RI=SimpleNamespace(point_statuses={55: 9}), pm=FakePM())
  Car.radar_publish(car, one_point())
  (tracks_service, tracks_msg), (status_service, status_msg) = car.pm.sent
  assert (tracks_service, status_service) == ('radarTracks', 'starpilotCarState')
  assert status_msg.starpilotCarState.radarTracksMonoTime == tracks_msg.logMonoTime > 0
  assert [(row.trackId, row.objectStatus) for row in status_msg.starpilotCarState.radarPointStatus] == [(55, 9)]
  assert car.radar_point_status == (tracks_msg.logMonoTime, {55: 9})


def test_card_sends_no_carrier_for_other_radars():
  car = SimpleNamespace(RI=SimpleNamespace(), pm=FakePM())
  Car.radar_publish(car, one_point())
  assert [service for service, _ in car.pm.sent] == ['radarTracks']


def radard_tick(sm, tick, statuses, v_lead=5.4, d_rel=66.0, v_ego=16.0, track_id=55, model_x=None, model_v=13.5):
  stamp = 1_000_000_000 + tick * 50_000_000
  model_event = messaging.new_message('modelV2', valid=True)
  model_event.logMonoTime = stamp
  model_event.modelV2.velocity.x = [v_ego]
  for lead in model_event.modelV2.init('leadsV3', 2):
    lead.x = [d_rel + 1.52 if model_x is None else model_x]
    lead.y = [0.0]
    lead.v = [model_v]
    lead.a = [0.0]
    lead.xStd = lead.yStd = lead.vStd = [1.0]
    lead.prob = 0.99
  car_event = messaging.new_message('carState', valid=True)
  car_event.logMonoTime = stamp
  car_event.carState.vEgo = v_ego
  radar_event = messaging.new_message('radarTracks', valid=True)
  radar_event.logMonoTime = stamp
  point = radar_event.radarTracks.init('points', 1)[0]
  point.trackId = track_id
  point.dRel = d_rel
  point.vRel = v_lead - v_ego
  point.deprecated.measured = True
  events = [model_event.as_reader(), car_event.as_reader(), radar_event.as_reader()]
  if statuses is not None:
    events.append(companion(stamp, statuses))
  sm.update_msgs(stamp / 1e9, events)


def run_radard(statuses, ticks=60, shape=lambda tick: {}):
  sm = messaging.SubMaster(['modelV2', 'carState', 'radarTracks', 'starpilotCarState'])
  radar = RadarD(honda_bosch_a_radar=True)
  for tick in range(ticks):
    radard_tick(sm, tick, statuses, **shape(tick))
    radar.update(sm, sm['radarTracks'])
  return radar


def test_route_34_shape_through_radard():
  # track 55 at 66 m read ~5 m/s against a model lead at 13.5 m/s, a 0; TTC 6.2 s, so not urgent
  plain = run_radard(None).radar_state.leadOne
  assert plain.radar and plain.radarTrackId == 55
  assert run_radard({55: 7}).radar_state.leadOne.to_dict() == plain.to_dict()  # confirmed: as before the carrier
  unconfirmed = run_radard({55: 9})
  bounded = unconfirmed.radar_state.leadOne
  assert unconfirmed.tracks[55].objectStatus == 9
  assert bounded.radar and bounded.radarTrackId == 55 and bounded.dRel == plain.dRel
  assert bounded.vLead == pytest.approx(plain.vLead + UNCONFIRMED_W * (13.5 - plain.vLead), abs=0.05)


def route_37_shape(model_v):
  def shape(tick):
    if tick < 30:  # the SUV ahead at 53 m, matched through the strict gate
      return {'track_id': 32, 'd_rel': 53.0, 'v_lead': 28.6, 'v_ego': 29.8, 'model_x': 53.0 + 1.52, 'model_v': 28.6}
    # it moves out: the radar reads it slowing to 25.3 at 45 m while the model lead moves on to 64 m
    return {'track_id': 32, 'd_rel': 45.0, 'v_lead': 25.3, 'v_ego': 29.8, 'model_x': 64.0 + 1.52, 'model_v': model_v}
  return shape


def test_route_37_shape_through_radard():
  # control: the model lead closing at 11.7 m/s calls for braking itself (route 37 T273's pickup): no bound
  plain = run_radard(None, shape=route_37_shape(18.1)).radar_state.leadOne
  assert plain.radar and plain.radarTrackId == 32 and plain.vLead < 26.0
  bounded = run_radard(None, shape=route_37_shape(29.3)).radar_state.leadOne  # route 37 T903: closing 0.5 m/s
  assert bounded.radar and bounded.radarTrackId == 32 and bounded.dRel == plain.dRel
  assert bounded.vLead == pytest.approx(plain.vLead + DISTANCE_W * (29.3 - plain.vLead), abs=0.05)
