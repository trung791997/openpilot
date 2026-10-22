"""leadLeft/leadRight producer (lib/adjacent_leads.py), ported from iq-nav-on-pr10 test_radard_adjacent_rail.py.

ADJACENT_RAIL_GATE: a Bosch-A track whose U11 sits on the rail while its range closes faster is not a leadLeft/leadRight.
Shape of 0000028f seg 6 track 49: a stationary roadside return at yRel ~3, closing at ~vEgo (21 m/s) while U11
reads the low rail (-13.5 as logged at 1/64; -12.0 at 1/72), published as a ~16 mph left-lane car.

Port note: 7.0's Honda Track carries no range history, so the fit and latch live in AdjacentRailMonitor; each sweep
here is a Track.update followed by one monitor.update, as radard does. Static only: no radard, no replay.
"""
from types import SimpleNamespace

import numpy as np

from opendbc.car.honda.bosch_a_radar import BOSCH_A_FREQ_HZ
from openpilot.selfdrive.controls.lib import adjacent_leads as al
from openpilot.starpilot.car.honda.radar import Track

RAIL = al.BOSCH_A_U11_LOW_RAIL_MPS
DT = 1.0 / BOSCH_A_FREQ_HZ  # Bosch-A sweep period
V_EGO = 21.0


def kalman(dt=DT):
  # radard.KalmanParams' lookup, inlined so this test does not import radard (msgq is aarch64-only here)
  dts = [i * 0.01 for i in range(1, 21)]
  k0 = [0.12287673, 0.14556536, 0.16522756, 0.18281627, 0.1988689, 0.21372394, 0.22761098, 0.24069424, 0.253096,
        0.26491023, 0.27621103, 0.28705801, 0.29750003, 0.30757767, 0.31732515, 0.32677158, 0.33594201, 0.34485814,
        0.35353899, 0.36200124]
  k1 = [0.29666309, 0.29330885, 0.29042818, 0.28787125, 0.28555364, 0.28342219, 0.28144091, 0.27958406, 0.27783249,
        0.27617149, 0.27458948, 0.27307714, 0.27162685, 0.27023228, 0.26888809, 0.26758976, 0.26633338, 0.26511557,
        0.26393339, 0.26278425]
  return SimpleNamespace(A=[[1.0, dt], [0.0, 1.0]], C=[1.0, 0.0], K=[[np.interp(dt, dts, k0)], [np.interp(dt, dts, k1)]])


def lanes():
  def line(y):
    return SimpleNamespace(x=[0.0, 200.0], y=[y, y])
  # model y == -yRel; laneLines[1] is the left boundary, [2] the right
  return SimpleNamespace(laneLines=[line(-5.4), line(-1.8), line(1.8), line(5.4)])


class Rig:
  def __init__(self, *tracks):
    self.tracks = {t.identifier: t for t in tracks}
    self.monitor = al.AdjacentRailMonitor()

  def sweep(self, track, d_rel, y_rel, v_rel, t, measured=True):
    track.update(d_rel, y_rel, v_rel, V_EGO + v_rel, measured, measured)
    self.monitor.update(self.tracks, t)

  def drive(self, track, n, *, v_rel, closing, d0=75.0, y_rel=3.2, t0=0.0):
    """n measured sweeps: U11 reads v_rel while the range closes at `closing` m/s."""
    for i in range(n):
      self.sweep(track, d0 - closing * i * DT, y_rel, v_rel, t0 + i * DT)
    return t0 + n * DT

  def cycle(self, t, *rows):
    """One radard cycle: every (track, d_rel, y_rel, v_rel) updated first, then one monitor.update."""
    for track, d_rel, y_rel, v_rel in rows:
      track.update(d_rel, y_rel, v_rel, V_EGO + v_rel, True, True)
    self.monitor.update(self.tracks, t)

  def state(self, track):
    return self.monitor.state(track)

  def lead(self, left=True, bosch_a=True, lead_track_id=-1):
    return al.get_adjacent_lead(self.tracks, False, lanes(), left=left, honda_bosch_a=bosch_a, monitor=self.monitor,
                                lead_track_id=lead_track_id)


def new_track(identifier=49, v_lead=V_EGO + RAIL):
  return Track(identifier, v_lead, kalman())


def test_rail_constant_is_the_1_72_decode():
  assert RAIL == -12.0


def test_railed_stationary_track_is_not_an_adjacent_lead_on_bosch_a():
  track = new_track()
  rig = Rig(track)
  rig.drive(track, 12, v_rel=RAIL, closing=V_EGO)
  assert rig.state(track).rail_range_inconsistent
  assert al.potential_adjacent_lead(track, True, False, lanes(), -1)  # the lane test alone would take it
  assert not rig.lead()['status']


def test_gate_is_bosch_a_only():
  track = new_track()
  rig = Rig(track)
  rig.drive(track, 12, v_rel=RAIL, closing=V_EGO)
  assert rig.lead(bosch_a=False)['status']


def test_gate_can_be_disabled(monkeypatch):
  monkeypatch.setattr(al, "ADJACENT_RAIL_GATE", False)
  monkeypatch.setattr(al, "ADJACENT_RAIL_CONFIRM", True)  # the gate switch turns off both
  track = new_track()
  rig = Rig(track)
  rig.drive(track, 12, v_rel=RAIL, closing=V_EGO)
  assert rig.lead()['status']


def test_railed_track_whose_range_agrees_stays_eligible():
  track = new_track()
  rig = Rig(track)
  rig.drive(track, 12, v_rel=RAIL, closing=-RAIL)
  assert not rig.state(track).rail_range_inconsistent
  assert rig.lead()['status']


def test_off_rail_moving_car_stays_eligible():
  track = new_track(v_lead=V_EGO - 2.0)
  rig = Rig(track)
  rig.drive(track, 12, v_rel=-2.0, closing=2.0)
  assert not rig.state(track).rail_range_inconsistent
  assert rig.lead()['status']


def test_latch_needs_consecutive_updates_and_clears_off_the_rail():
  track = new_track()
  rig = Rig(track)
  # the short fit needs RANGE_VREL_SAMPLES sweeps; the latch then needs ADJACENT_RAIL_GATE_UPDATES more
  t = rig.drive(track, al.RANGE_VREL_SAMPLES + al.ADJACENT_RAIL_GATE_UPDATES - 2, v_rel=RAIL, closing=V_EGO)
  assert not rig.state(track).rail_range_inconsistent
  t = rig.drive(track, 1, v_rel=RAIL, closing=V_EGO, d0=track.dRel - V_EGO * DT, t0=t)
  assert rig.state(track).rail_range_inconsistent
  rig.drive(track, 1, v_rel=RAIL + 1.0, closing=V_EGO, d0=track.dRel - V_EGO * DT, t0=t)
  assert not rig.state(track).rail_range_inconsistent


def test_duplicate_cycle_does_not_advance_the_latch():
  track = new_track()
  rig = Rig(track)
  t = rig.drive(track, al.RANGE_VREL_SAMPLES + al.ADJACENT_RAIL_GATE_UPDATES - 2, v_rel=RAIL, closing=V_EGO)
  count = rig.state(track).rail_range_count
  rig.sweep(track, track.dRel, track.yRel, RAIL, t - DT, measured=False)
  assert rig.state(track).rail_range_count == count
  assert not rig.state(track).rail_range_inconsistent


def test_closer_railed_point_does_not_hide_a_real_adjacent_car():
  stationary = new_track(49)
  car = new_track(44, v_lead=V_EGO - 1.0)
  rig = Rig(stationary, car)
  for i in range(12):
    rig.cycle(i * DT, (stationary, 30.0 - V_EGO * i * DT, 3.2, RAIL), (car, 40.0 - 1.0 * i * DT, 3.2, -1.0))
  lead = rig.lead()
  assert lead['status'] and lead['radarTrackId'] == 44


# ADJACENT_RAIL_CONFIRM: shape of 000002cb segs 8-12 (Job replay): oncoming cars on the left, U11 on the rail,
# range closing ~ -39, holds a median 0.3 s -- shorter than the latch needs.
def test_short_railed_hold_before_any_fit_is_not_an_adjacent_lead():
  track = new_track()
  rig = Rig(track)
  rig.drive(track, 4, v_rel=RAIL, closing=39.0)
  state = rig.state(track)
  assert not state.vRelRangeFresh and not state.rail_range_inconsistent
  assert state.rail_unconfirmed(track.vRel)
  assert not rig.lead()['status']


def test_railed_oncoming_car_is_dropped_before_the_latch():
  track = new_track()
  rig = Rig(track)
  rig.drive(track, al.RANGE_VREL_SAMPLES, v_rel=RAIL, closing=39.0)
  state = rig.state(track)
  assert state.vRelRangeFresh and not state.rail_range_inconsistent
  assert not rig.lead()['status']


def test_confirm_can_be_disabled(monkeypatch):
  monkeypatch.setattr(al, "ADJACENT_RAIL_CONFIRM", False)
  track = new_track()
  rig = Rig(track)
  rig.drive(track, 4, v_rel=RAIL, closing=39.0)
  assert rig.lead()['status']


def test_confirm_is_bosch_a_only():
  track = new_track()
  rig = Rig(track)
  rig.drive(track, 4, v_rel=RAIL, closing=39.0)
  assert rig.lead(bosch_a=False)['status']


def test_slow_object_whose_fit_agrees_with_the_rail_is_kept():
  # L31 at 2cb: vEgo 16.8, range closing -11.4 while U11 reads the rail -- a slow object, not oncoming
  track = Track(31, 16.8 + RAIL, kalman())
  rig = Rig(track)
  for i in range(12):
    track.update(60.0 - 11.4 * i * DT, 2.6, RAIL, 16.8 + RAIL, True, True)
    rig.monitor.update(rig.tracks, i * DT)
  assert not rig.state(track).rail_unconfirmed(track.vRel)
  assert rig.lead()['status']


def test_off_rail_track_without_a_fit_is_unaffected():
  track = new_track(v_lead=V_EGO - 3.0)
  rig = Rig(track)
  rig.drive(track, 2, v_rel=-3.0, closing=3.0)
  assert not rig.state(track).rail_unconfirmed(track.vRel)
  assert rig.lead()['status']


# --- port-specific: lane sides, exclusions, wire shape, per-track state lifecycle

def test_sides_follow_the_model_lane_lines():
  left = new_track(10, v_lead=V_EGO - 1.0)
  right = new_track(11, v_lead=V_EGO - 1.0)
  inlane = new_track(12, v_lead=V_EGO - 1.0)
  rig = Rig(left, right, inlane)
  for i in range(6):
    rig.cycle(i * DT, (left, 30.0, 3.2, -1.0), (right, 35.0, -3.2, -1.0), (inlane, 20.0, 0.2, -1.0))
  assert rig.lead(left=True)['radarTrackId'] == 10
  assert rig.lead(left=False)['radarTrackId'] == 11


def test_lead_track_slow_and_standstill_are_excluded():
  track = new_track(10, v_lead=V_EGO - 1.0)
  rig = Rig(track)
  rig.drive(track, 6, v_rel=-1.0, closing=1.0)
  assert not rig.lead(lead_track_id=10)['status']
  assert not al.get_adjacent_lead(rig.tracks, True, lanes(), left=True, honda_bosch_a=True, monitor=rig.monitor)['status']
  slow = new_track(20, v_lead=0.5)
  rig2 = Rig(slow)
  for i in range(6):
    slow.update(30.0, 3.2, 0.5 - V_EGO, 0.5, True, True)
    rig2.monitor.update(rig2.tracks, i * DT)
  assert not rig2.lead()['status']  # vLead < 1


def test_wire_shape_matches_custom_lead_data():
  track = new_track(10, v_lead=V_EGO - 1.0)
  rig = Rig(track)
  rig.drive(track, 6, v_rel=-1.0, closing=1.0)
  lead = rig.lead()
  assert set(lead) == {'dRel', 'yRel', 'vRel', 'vLead', 'vLeadK', 'aLeadK', 'aLeadTau', 'status', 'fcw', 'modelProb',
                       'radar', 'radarTrackId'}
  assert lead['status'] and lead['radar'] and not lead['fcw'] and lead['vRel'] == -1.0
  assert all(isinstance(lead[k], float) for k in ('dRel', 'yRel', 'vRel', 'vLead', 'vLeadK', 'aLeadK', 'aLeadTau'))


def test_recreated_track_starts_a_fresh_history():
  track = new_track()
  rig = Rig(track)
  rig.drive(track, 12, v_rel=RAIL, closing=V_EGO)
  assert rig.state(track).rail_range_inconsistent
  reborn = new_track()  # radard dropped and re-created id 49
  rig.tracks[49] = reborn
  rig.sweep(reborn, 40.0, 3.2, -1.0, 1.0)
  state = rig.state(reborn)
  assert state is not None and not state.rail_range_inconsistent and len(state.range_hist) == 1
  assert rig.state(track) is None


def test_no_lane_lines_means_no_label():
  track = new_track(10, v_lead=V_EGO - 1.0)
  rig = Rig(track)
  rig.drive(track, 6, v_rel=-1.0, closing=1.0)
  assert not al.get_adjacent_lead(rig.tracks, False, SimpleNamespace(laneLines=[]), monitor=rig.monitor)['status']
