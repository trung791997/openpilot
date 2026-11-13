"""One Bosch-A gate (long_tuning_active: verified Bosch-A Honda, openpilot long, radar on), no owner toggles: every part and both planners.

Static tests with a recording stand-in for the native acados solver; no replay or road evidence.
"""
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
from openpilot.cereal import log, messaging
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LEAD_T_IDXS_MODEL
from openpilot.selfdrive.controls.radard import RADAR_TO_CAMERA
from openpilot.selfdrive.controls.lib.accel_boost import AccelBoost
from openpilot.starpilot.longitudinal import bosch_a_release as release_module, bosch_a_toggles, human_following
from openpilot.starpilot.longitudinal.honda_bosch_a_long import HondaBoschALongEnhancement
from openpilot.starpilot.longitudinal.upstream import mpc, planner, plannerd


def car(fingerprint='HONDA_CIVIC_BOSCH', brand='honda', long=True, radar_unavailable=False):
  return SimpleNamespace(brand=brand, carFingerprint=fingerprint, openpilotLongitudinalControl=long,
                         radarUnavailable=radar_unavailable, pcmCruise=False, longitudinalActuatorDelay=0.15,
                         vEgoStopping=0.5, vEgoStarting=0.5, stopAccel=-2.0, stoppingDecelRate=0.8,
                         steerRatio=15.38, wheelbase=2.7)


BOSCH_A = car()
RADAR_OFF = car(radar_unavailable=True)
NOT_BOSCH_A = (car('TOYOTA_RAV4_TSS2_2023', brand='toyota'), car('HONDA_ACCORD'), car(long=False))


class RecordingSolver:
  """Deterministic stand-in for AcadosOcpSolverCython: logs every call and derives x from the stage parameters, so any
  change in what the planner hands the MPC changes both the log and the planner output."""
  def __init__(self, *_):
    self.calls, self.p, self.x0 = [], {}, np.zeros(3)

  def _log(self, *args):
    self.calls.append(tuple(np.copy(a).tolist() if isinstance(a, np.ndarray) else a for a in args))

  def reset(self):
    self._log('reset')

  def cost_set(self, stage, key, value):
    self._log('cost_set', stage, key, value)

  def set(self, stage, key, value):
    self._log('set', stage, key, value)
    if key == 'p':
      self.p[stage] = np.copy(value)

  def constraints_set(self, stage, key, value):
    self._log('constraints_set', stage, key, value)
    self.x0 = np.copy(value)

  def solve(self):
    return 0

  def get_stats(self, _):
    return [0.001]

  def get(self, stage, key):
    if key == 'u':
      return np.array([0.0])
    p = self.p.get(stage, np.zeros(8))
    a = 0.5 * float(p[1]) - 0.02 * max(0.0, 60.0 - float(p[2]))
    return np.array([self.x0[0] + stage, self.x0[1] + 0.1 * stage * a, a])


def submaster(frame, personality=log.LongitudinalPersonality.standard):
  services = ('carControl', 'carState', 'controlsState', 'vehicleParameters', 'radarState', 'modelV2', 'selfdriveState')
  msgs = {name: messaging.new_message(name) for name in services}
  cs = msgs['carState'].carState
  cs.vEgo, cs.aEgo, cs.vCruise, cs.canValid = 22.0 - 0.05 * frame, -0.2, 100.0, True
  # Frame 0 is the engage frame: 'off' resets the planner state to vEgo, as on the car.
  msgs['controlsState'].controlsState.longControlState = 'off' if frame == 0 else 'pid'
  ss = msgs['selfdriveState'].selfdriveState
  ss.enabled, ss.personality = True, personality
  rs = msgs['radarState'].radarState
  d_rels = (32.0 - 0.3 * frame, 60.0)
  for track, (lead, d_rel) in enumerate(zip((rs.leadOne, rs.leadTwo), d_rels, strict=True)):
    lead.present, lead.dRel, lead.vLead, lead.vLeadK, lead.aLeadK, lead.aLeadTau, lead.modelProb, lead.radar = \
      True, d_rel, 18.0, 18.0, -0.8, 1.5, 0.9, True
    lead.radarTrackId = track + 1
  model = msgs['modelV2'].modelV2
  model.meta.disengagePredictions.gasPressProbs = [0.5, 0.5]
  model.velocity.x = [22.0] * 33
  model.init('leadsV3', 3)
  for i, model_lead in enumerate(model.leadsV3):
    # Matches leadOne/leadTwo (StarPilot's horizon check) but brakes harder than the radar aLeadK extrapolation.
    d_rel = d_rels[min(i, 1)]
    model_lead.prob = 0.9
    model_lead.t = LEAD_T_IDXS_MODEL.tolist()
    model_lead.x = [d_rel + RADAR_TO_CAMERA + 18.0 * t - 0.75 * t * t for t in model_lead.t]
    model_lead.y = [0.0] * len(model_lead.t)
    model_lead.v = [max(0.0, 18.0 - 1.5 * t) for t in model_lead.t]
    model_lead.a = [-1.5] * len(model_lead.t)
  return {name: getattr(msg, name) for name, msg in msgs.items()}


def run(planner_obj, frames=6, personality=log.LongitudinalPersonality.standard):
  out = []
  for frame in range(frames):
    planner_obj.update(submaster(frame, personality))
    out.append((float(planner_obj.output_a_target), bool(planner_obj.output_should_stop), bool(planner_obj.fcw),
                planner_obj.v_desired_trajectory.tolist(), planner_obj.a_desired_trajectory.tolist(),
                planner_obj.j_desired_trajectory.tolist()))
  return out, planner_obj.mpc.solver.calls


def upstream(cp):
  with patch.object(mpc, 'AcadosOcpSolverCython', RecordingSolver):
    return planner.LongitudinalPlanner(cp)


class TestCarGates(unittest.TestCase):
  def test_stock_brake_feel_is_always_on_for_bosch_a_with_radar(self):
    self.assertTrue(HondaBoschALongEnhancement(BOSCH_A, 0.05).stock_feel)
    self.assertFalse(HondaBoschALongEnhancement(car(radar_unavailable=True), 0.05).stock_feel)
    for cp in NOT_BOSCH_A:
      self.assertFalse(HondaBoschALongEnhancement(cp, 0.05).stock_feel, cp)

  def test_human_following_needs_bosch_a_radar_on_or_off(self):
    self.assertTrue(human_following.enabled_for(BOSCH_A))
    self.assertTrue(human_following.enabled_for(RADAR_OFF))  # vision-only shares it
    for cp in NOT_BOSCH_A:
      self.assertFalse(human_following.enabled_for(cp), cp)

  def test_every_part_follows_the_one_gate(self):
    from openpilot.starpilot.longitudinal.bosch_a_release import BoschARelease
    from openpilot.starpilot.longitudinal.bosch_a_stop_gap import BoschAStopGap
    for cp in (BOSCH_A, RADAR_OFF, *NOT_BOSCH_A):
      gate = bosch_a_toggles.long_tuning_active(cp)  # radar on
      supported = bosch_a_toggles.vehicle_supported(cp)  # radar on or off
      enh = HondaBoschALongEnhancement(cp, 0.05)
      self.assertEqual(gate, cp is BOSCH_A, cp)
      self.assertEqual(supported, cp in (BOSCH_A, RADAR_OFF), cp)
      self.assertEqual(
        (enh.stock_feel, enh.coast_enabled, enh.newborn_hold is not None, enh.drop_bound is not None,
         enh._need_floor is not None, BoschARelease(cp).coast_enabled, BoschAStopGap(cp).enabled,
         AccelBoost(lead_gate=gate).lead_gate),
        (gate,) * 8, cp)
      self.assertEqual((human_following.enabled_for(cp), BoschARelease(cp).enabled), (supported,) * 2, cp)

  def test_radar_off_radar_parts_inert_and_vision_parts_active(self):
    from openpilot.common.realtime import DT_MDL
    from openpilot.starpilot.longitudinal.bosch_a_release import BoschARelease
    from openpilot.starpilot.longitudinal.bosch_a_stop_gap import BoschAStopGap
    release = BoschARelease(RADAR_OFF)
    # vision-only keeps smooth and action_t (as at 786d62d5d2); coast needs the radar
    self.assertTrue(release.enabled)
    self.assertFalse(release.coast_enabled)
    self.assertEqual(release.action_t(0.5), min(0.5 + DT_MDL, release_module.PLANNER_ACTION_T_S))
    self.assertLess(release.action_t(0.5), 0.5 + DT_MDL)
    self.assertEqual(release.smooth(-1.0, -0.2, False), release_module.brake_release_limited_target(-1.0, -0.2, release.dt))
    self.assertLess(release.smooth(-1.0, -0.2, False), -0.2)  # the release limit binds
    self.assertEqual(BoschARelease(NOT_BOSCH_A[0]).action_t(0.5), 0.5 + DT_MDL)
    self.assertEqual(BoschARelease(NOT_BOSCH_A[0]).smooth(-1.0, -0.2, False), -0.2)
    self.assertTrue(human_following.enabled_for(RADAR_OFF))
    self.assertFalse(BoschAStopGap(RADAR_OFF).enabled)
    leads = SimpleNamespace(present=True, dRel=20.0, vLead=10.0, vRel=-12.0, aLeadK=-1.0, radar=True, modelProb=0.9)
    enh = HondaBoschALongEnhancement(RADAR_OFF, 0.05)
    out = enh.step(lead_one=leads, lead_two=leads, prev=-0.5, target=-3.0, v_ego=22.0, stopping=False, standstill=False,
                   reset_state=False, fcw=False, follow_s=1.5)
    self.assertEqual(out, -3.0)
    self.assertEqual(enh.need_floor(lead_one=leads, lead_two=leads, target=-0.3, v_ego=22.0, reset_state=False), -0.3)
    radar_state = SimpleNamespace(leadOne=leads, leadTwo=leads)
    self.assertIs(enh.planner_radar_state(radar_state, 0.0), radar_state)

  def test_accel_boost_lead_gate_off_and_default_on(self):
    sm = {'selfdriveState': SimpleNamespace(enabled=True, experimentalMode=True, personality=0),
          'carState': SimpleNamespace(gasPressed=False, vEgo=17.0), 'radarState': log.RadarState.new_message()}
    sm['radarState'].leadOne.present, sm['radarState'].leadOne.dRel = True, 20.0
    sm['radarState'].leadOne.vRel, sm['radarState'].leadOne.aLeadK = -3.0, -1.0
    gated, ungated, default = AccelBoost(lead_gate=True), AccelBoost(lead_gate=False), AccelBoost()
    for _ in range(40):
      for b in (gated, ungated, default):
        b.update(sm, 0.0, 1.0, 1.0)
    self.assertEqual(gated.lead_scale, 0.0)
    self.assertEqual(ungated.lead_scale, 1.0)
    self.assertEqual(default.lead_scale, 0.0)  # no argument == gate on

  def test_no_toggle_keys_remain(self):
    self.assertFalse(hasattr(bosch_a_toggles, 'KEYS'))
    self.assertFalse(hasattr(bosch_a_toggles, 'read_toggles'))

  def test_long_tuning_active_gate(self):
    self.assertTrue(bosch_a_toggles.long_tuning_active(BOSCH_A))
    self.assertFalse(bosch_a_toggles.long_tuning_active(None))
    self.assertFalse(bosch_a_toggles.long_tuning_active(RADAR_OFF))
    self.assertFalse(bosch_a_toggles.long_tuning_active(SimpleNamespace()))  # unreadable CP
    for cp in NOT_BOSCH_A:
      self.assertFalse(bosch_a_toggles.long_tuning_active(cp), cp)


class TestUpstreamPlanner(unittest.TestCase):
  def test_bosch_a_parts_on_and_other_cars_are_upstream(self):
    bosch = upstream(BOSCH_A)
    self.assertTrue(bosch.bosch_a_long.stock_feel)
    self.assertIsNotNone(bosch.bosch_a_model_leads)  # HumanFollowing + HumanAcceleration
    self.assertTrue(bosch.bosch_a_release.enabled)
    self.assertTrue(bosch.bosch_a_stop_gap.enabled)
    self.assertTrue(bosch.accel_boost.lead_gate)
    vision = upstream(RADAR_OFF)  # vision-only Bosch-A: HF/HA and release smoothing stay, the radar parts are off
    self.assertIsNone(vision.bosch_a_long)
    self.assertIsNotNone(vision.bosch_a_model_leads)
    self.assertTrue(vision.bosch_a_release.enabled)
    self.assertFalse(vision.bosch_a_release.coast_enabled)
    self.assertFalse(vision.bosch_a_stop_gap.enabled)
    self.assertFalse(vision.accel_boost.lead_gate)
    for cp in NOT_BOSCH_A:
      self.assertFalse(upstream(cp).accel_boost.lead_gate, cp)
    for cp in NOT_BOSCH_A:
      other = upstream(cp)
      self.assertIsNone(other.bosch_a_long, cp)
      self.assertIsNone(other.bosch_a_model_leads, cp)
      self.assertFalse(other.bosch_a_release.enabled, cp)
      self.assertFalse(other.bosch_a_stop_gap.enabled, cp)

  def test_non_bosch_a_car_is_stock_upstream_output(self):
    # No Bosch-A part runs for these cars: no hook state, and two fresh planners agree bit for bit.
    for cp in NOT_BOSCH_A:
      stock = upstream(cp)
      self.assertIsNone(stock.bosch_a_long)
      self.assertIsNone(stock.bosch_a_model_leads)
      self.assertEqual(run(stock), run(upstream(cp)), cp)

  def test_bosch_a_kwarg_is_ignored(self):
    with patch.object(mpc, 'AcadosOcpSolverCython', RecordingSolver):
      ignored = planner.LongitudinalPlanner(BOSCH_A, bosch_a=object())
    self.assertEqual(run(ignored), run(upstream(BOSCH_A)))

  def test_stock_brake_feel_runs_step_on_bosch_a(self):
    stock = upstream(BOSCH_A)
    self.assertIsNotNone(stock.bosch_a_long)
    self.assertTrue(stock.bosch_a_long.stock_feel)
    seen = []
    original = stock.bosch_a_long.step
    def spy(**kwargs):
      result = original(**kwargs)
      seen.append((kwargs, result))
      return result
    stock.bosch_a_long.step = spy
    outputs, _ = run(stock)
    self.assertEqual(len(seen), len(outputs))
    for frame, ((kwargs, result), output) in enumerate(zip(seen, outputs, strict=True)):
      self.assertEqual(output[0], float(np.clip(result, mpc.ACCEL_MIN, mpc.ACCEL_MAX)))
      self.assertEqual(kwargs['lead_one'].dRel, submaster(frame)['radarState'].leadOne.dRel)  # raw radarState leads
      self.assertEqual(kwargs['reset_state'], frame == 0)
      self.assertIn('fcw', kwargs)

  def test_human_following_passes_cap_and_lead_paths_on_bosch_a(self):
    from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib import long_mpc
    with patch.object(long_mpc, 'qualified_model_leads', side_effect=lambda sm, now_ns, history: sm['modelV2']):
      stock = upstream(BOSCH_A)
    self.assertIsNotNone(stock.bosch_a_model_leads)
    self.assertTrue(stock.bosch_a_long.stock_feel)  # always on for Bosch-A
    seen = []
    original = stock.mpc.update
    def spy(radarstate, personality=None, **kwargs):
      seen.append(kwargs)
      return original(radarstate, personality=personality, **kwargs)
    stock.mpc.update = spy
    outputs, _ = run(stock)
    self.assertEqual(len(seen), len(outputs))
    for kwargs in seen:
      self.assertEqual(set(kwargs), {'acceleration_max', 'lead_paths'})
      self.assertTrue(0.0 <= kwargs['acceleration_max'] <= mpc.ACCEL_MAX)
      self.assertEqual(len(kwargs['lead_paths']), 2)
      self.assertTrue(any(path is not None for path in kwargs['lead_paths']))
    np.testing.assert_array_equal(stock.mpc.params[:, 1], seen[-1]['acceleration_max'])

  def test_human_following_aggressive_personality_runs_without_cap(self):
    # HumanAcceleration skips aggressive, so no acceleration_max is passed; hook 3/3 must not assume it (plannerd
    # KeyError on the car, 2026-11-12).
    from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib import long_mpc
    with patch.object(long_mpc, 'qualified_model_leads', side_effect=lambda sm, now_ns, history: sm['modelV2']):
      stock = upstream(BOSCH_A)
    seen = []
    original = stock.mpc.update
    def spy(radarstate, personality=None, **kwargs):
      seen.append(kwargs)
      return original(radarstate, personality=personality, **kwargs)
    stock.mpc.update = spy
    outputs, _ = run(stock, personality=log.LongitudinalPersonality.aggressive)
    self.assertEqual(len(outputs), 6)
    for kwargs in seen:
      self.assertNotIn('acceleration_max', kwargs)

  def test_human_following_without_fresh_model_keeps_stock_lead_paths(self):
    stock = upstream(BOSCH_A)  # the real freshness check rejects the fake sm
    seen = []
    original = stock.mpc.update
    def spy(radarstate, personality=None, **kwargs):
      seen.append(kwargs['lead_paths'])
      return original(radarstate, personality=personality, **kwargs)
    stock.mpc.update = spy
    run(stock, frames=2)
    self.assertEqual(seen, [(None, None), (None, None)])

  def test_mpc_rejects_out_of_range_cap(self):
    with patch.object(mpc, 'AcadosOcpSolverCython', RecordingSolver):
      stock = mpc.LongitudinalMpc()
    radar = submaster(0)['radarState']
    for bad in (float('nan'), float('inf'), -0.1, mpc.ACCEL_MAX + 0.1):
      stock.update(radar, acceleration_max=bad)
      np.testing.assert_array_equal(stock.params[:, 1], mpc.ACCEL_MAX)
    stock.update(radar, acceleration_max=0.5)
    np.testing.assert_array_equal(stock.params[:, 1], 0.5)

  def test_plannerd_builds_the_planner_without_toggles(self):
    calls = []
    class SM:
      def update(self):
        raise InterruptedError('bounded end of mocked loop')
    with patch.object(plannerd, 'config_realtime_process'), \
         patch.object(plannerd, 'Params', return_value=SimpleNamespace(get=lambda key, **kwargs: b'CP')), \
         patch.object(plannerd.messaging, 'log_from_bytes', return_value=BOSCH_A), \
         patch.object(plannerd, 'LongitudinalPlanner', side_effect=lambda cp, **kw: calls.append(kw)), \
         patch.object(plannerd, 'LaneDepartureWarning', return_value=object()), \
         patch.object(plannerd.messaging, 'PubMaster'), \
         patch.object(plannerd.messaging, 'SubMaster', return_value=SM()):
      with self.assertRaises(InterruptedError):
        plannerd.main()
    self.assertEqual(calls, [{}])
    self.assertFalse(hasattr(plannerd, 'read_bosch_a_toggles'))


class TestStarPilotPlanner(unittest.TestCase):
  def build(self, cp):
    from openpilot.selfdrive.controls.lib import longitudinal_planner
    from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib import long_mpc
    with patch.object(long_mpc, 'AcadosOcpSolverCython', RecordingSolver):
      return longitudinal_planner.LongitudinalPlanner(cp)

  def test_bosch_a_parts_are_always_on(self):
    stock = self.build(BOSCH_A)
    self.assertTrue(stock.human_driving)
    self.assertTrue(stock.bosch_a_long.stock_feel)
    self.assertTrue(stock.bosch_a_release.enabled)
    self.assertTrue(stock.bosch_a_stop_gap.enabled)
    self.assertTrue(stock.accel_boost.lead_gate)

  def test_radar_off_keeps_only_the_vision_parts(self):
    vision = self.build(RADAR_OFF)
    self.assertTrue(vision.human_driving)
    self.assertTrue(vision.bosch_a_release.enabled)
    self.assertFalse(vision.bosch_a_long.stock_feel)
    self.assertFalse(vision.bosch_a_long.coast_enabled)
    self.assertFalse(vision.bosch_a_release.coast_enabled)
    self.assertFalse(vision.bosch_a_stop_gap.enabled)
    self.assertFalse(vision.accel_boost.lead_gate)

  def test_other_cars_are_inert(self):
    for cp in NOT_BOSCH_A:
      other = self.build(cp)
      self.assertFalse(other.human_driving, cp)
      self.assertFalse(other.bosch_a_long.stock_feel, cp)
      self.assertFalse(other.bosch_a_release.enabled, cp)
      self.assertFalse(other.bosch_a_stop_gap.enabled, cp)
      self.assertFalse(other.accel_boost.lead_gate, cp)

  def test_bosch_a_kwarg_is_ignored(self):
    from openpilot.selfdrive.controls.lib import longitudinal_planner
    from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib import long_mpc
    with patch.object(long_mpc, 'AcadosOcpSolverCython', RecordingSolver):
      self.assertTrue(longitudinal_planner.LongitudinalPlanner(BOSCH_A, bosch_a=object()).human_driving)

  def test_plannerd_has_no_toggle_reader(self):
    from openpilot.selfdrive.controls import plannerd as starpilot_plannerd
    self.assertFalse(hasattr(starpilot_plannerd, 'read_bosch_a_toggles'))


if __name__ == '__main__':
  unittest.main()
