"""
Set Speed Assist at the Accel Boost site of both longitudinal planners: StarPilot's
(selfdrive/controls/lib/longitudinal_planner.py) and the stock one (starpilot/longitudinal/upstream/planner.py).

These are scripted unit tests of the planner hook (static / unit test), not road evidence.
"""
import functools
import unittest
from unittest.mock import Mock, patch

from opendbc.car.honda.interface import CarInterface as HondaInterface
from opendbc.car.honda.values import CAR as HONDA
from opendbc.car.hyundai.interface import CarInterface as HyundaiInterface
from opendbc.car.hyundai.values import CAR as HYUNDAI
from openpilot.cereal import log
from openpilot.selfdrive.controls.lib import longitudinal_planner as starpilot_planner
from openpilot.selfdrive.controls.lib.accel_boost import AccelBoost
from openpilot.starpilot.longitudinal.set_speed_assist.controller import SetSpeedAssistController
from openpilot.starpilot.longitudinal.set_speed_assist.tests.helpers import FakeParams
from openpilot.starpilot.longitudinal.tests.test_encouragement import frame
from openpilot.starpilot.longitudinal.upstream import planner as upstream_planner

V_EGO = 20.
FRAMES = 160
AGGRESSIVE = log.LongitudinalPersonality.aggressive
STANDARD = log.LongitudinalPersonality.standard
EPS = 1e-9


def car_params(factory, car):
  cp = factory.get_non_essential_params(car)
  cp.openpilotLongitudinalControl = True
  return cp


CARS = {'ioniq6': (HyundaiInterface, HYUNDAI.HYUNDAI_IONIQ_6), 'civic': (HondaInterface, HONDA.HONDA_CIVIC)}


def make_frame(tick, *, a_desired=0., gas=False, personality=STANDARD, lead=None):
  sm, now = frame(tick)
  sm.all_checks = lambda services=None: True  # the upstream planner publishes with no service list
  sm['carState'].vEgo = V_EGO
  sm['carState'].aEgo = 0.
  sm['carState'].gasPressed = gas
  sm['controlsState'].curvature = 0.
  sm['selfdriveState'].enabled = True
  sm['selfdriveState'].personality = personality
  sm['carControl'].longActive = True
  sm['modelV2'].velocity.x = [V_EGO] * 33
  sm['modelV2'].orientationRate.z = [0.] * 33
  sm['modelV2'].action.desiredAcceleration = a_desired
  if lead is not None:
    d_rel, v_rel = lead
    sm['radarState'].leadOne.present = True
    sm['radarState'].leadOne.radar = True
    sm['radarState'].leadOne.dRel = d_rel
    sm['radarState'].leadOne.vRel = v_rel
    sm['radarState'].leadOne.vLead = V_EGO + v_rel
    sm['radarState'].leadOne.vLeadK = V_EGO + v_rel
    sm['radarState'].leadOne.aLeadK = 0.
    sm['radarState'].leadOne.aLeadTau = 1.5
    sm['radarState'].leadOne.modelProb = 1.
  return sm, now


def over_cars(*cars):
  """Run a test body once per car, each as its own subTest."""
  def decorate(test):
    @functools.wraps(test)
    def run(self):
      for car in cars:
        with self.subTest(planner=self.kind, car=car):
          test(self, car_params(*CARS[car]))
    return run
  return decorate


BOTH_CARS = ('ioniq6', 'civic')


def before_this_change(assist, accel_boost, sm, *, a_model, a_mpc, a_cruise, **_):
  """The Accel Boost call each planner made before Set Speed Assist."""
  return accel_boost.update(sm, a_model, a_mpc, a_cruise), False


# the hook as the planners import it, and the run each controller instance belongs to
REAL_HOOK = {'starpilot': starpilot_planner.set_speed_assist_candidate, 'upstream': upstream_planner.set_speed_assist_candidate}
RUNS = {}


def route(kind):
  """The module-level hook for a planner: records every call for the PlannerRun that owns the
  controller, and runs the real hook or, for a baseline run, the pre-change Accel Boost call."""
  def hook(assist, *args, **kwargs):
    run = RUNS[id(assist)]
    result = (before_this_change if run.baseline else REAL_HOOK[kind])(assist, *args, **kwargs)
    run.calls.append((kwargs, result))
    return result
  return hook


class PlannerRun:
  """One planner instance with its hook calls recorded. enabled: the SetSpeedAssist param;
  None runs the hook as it was before this change (a baseline to compare against)."""

  def __init__(self, module, cp, enabled, *, kind):
    self.kind, self.baseline = kind, enabled is None
    self.planner = module.LongitudinalPlanner(cp, init_v=V_EGO)
    self.params = FakeParams(SetSpeedAssist=bool(enabled))
    self.planner.set_speed_assist = SetSpeedAssistController(params=self.params, dt=.05)
    self.calls = []

  def __enter__(self):
    RUNS[id(self.planner.set_speed_assist)] = self
    return self

  def __exit__(self, *exc):
    RUNS.pop(id(self.planner.set_speed_assist), None)

  def step(self, tick, **frame_args):
    sm, now = make_frame(tick, **frame_args)
    if self.kind == 'starpilot':
      self.planner.update(sm, now_ns=now)
    else:
      self.planner.update(sm)
    pm = Mock()
    self.planner.publish(sm, pm)
    plan = next(call.args[1] for call in pm.send.call_args_list if call.args[0] == 'longitudinalPlan')
    published = plan.longitudinalPlan.accelBoost
    kwargs, (candidate, owned) = self.calls[-1]
    return {'candidate': candidate, 'owned': owned, 'args': kwargs, 'output': self.planner.output_a_target,
            'boost': self.planner.accel_boost.total_boost, 'published': published}


PLANNERS = {'starpilot': starpilot_planner, 'upstream': upstream_planner}


class HookTests:
  """The planner-hook assertions, run on both planners."""
  kind: str

  def setUp(self):
    module = PLANNERS[self.kind]
    patcher = patch.object(module, 'set_speed_assist_candidate', route(self.kind))
    patcher.start()
    self.addCleanup(patcher.stop)

  def new(self, cp, enabled):
    return PlannerRun(PLANNERS[self.kind], cp, enabled, kind=self.kind)

  @over_cars(*BOTH_CARS)
  def test_param_off_is_what_the_planner_did_before(self, cp):
    # (a) a scripted sequence with boosts: aggressive personality (initial boost), gas while eligible
    with self.new(cp, False) as now, self.new(cp, None) as before:
      boosted = False
      for tick in range(FRAMES):
        args = {'a_desired': 0.1 * ((tick // 20) % 3), 'gas': 60 <= tick < 100, 'personality': AGGRESSIVE if tick < 120 else STANDARD,
                'lead': (45., 0.) if tick >= 130 else None}
        a, b = now.step(tick, **args), before.step(tick, **args)
        self.assertFalse(a['owned'])
        for key in ('candidate', 'output', 'boost', 'published'):
          self.assertEqual(a[key], b[key], (tick, key))
        boosted |= a['published'] > 0.
      self.assertTrue(boosted)  # the sequence did exercise Accel Boost
      self.assertEqual(vars(now.planner.accel_boost), vars(before.planner.accel_boost))

  @over_cars(*BOTH_CARS)
  def test_param_on_publishes_zero_and_leaves_accel_boost_alone(self, cp):
    # (b) Accel Boost is never called: its state stays at the initial one even where it would have boosted
    with self.new(cp, True) as run, self.new(cp, None) as before:
      initial = vars(AccelBoost(lead_gate=run.planner.accel_boost.lead_gate))
      for tick in range(FRAMES):
        args = {'gas': 60 <= tick < 100, 'personality': AGGRESSIVE}
        out, ref = run.step(tick, **args), before.step(tick, **args)
        self.assertTrue(out['owned'])
        self.assertEqual(out['published'], 0.)
        self.assertEqual(vars(run.planner.accel_boost), initial)
      self.assertGreater(ref['boost'], 0.)  # the same run with Accel Boost does boost

  @over_cars(*BOTH_CARS)
  def test_param_on_after_a_boost_publishes_zero_and_keeps_the_state(self, cp):
    # Accel Boost has boosted with the param off; switching on stops publishing it and does not advance or clear it
    with self.new(cp, False) as run:
      for tick in range(40):
        self.assertAlmostEqual(run.step(tick, personality=AGGRESSIVE)['published'], run.planner.accel_boost.total_boost, places=6)  # float32 on the wire
      boosted = vars(run.planner.accel_boost).copy()
      self.assertGreater(boosted['total_boost'], 0.)
      run.params.values['SetSpeedAssist'] = True
      for tick in range(40, 140):
        out = run.step(tick, personality=AGGRESSIVE, gas=tick >= 100)
        self.assertEqual(out['published'], 0.)
        self.assertEqual(vars(run.planner.accel_boost), boosted)
      self.assertTrue(out['owned'])

  @over_cars(*BOTH_CARS)
  def test_param_on_raises_e2e_toward_the_chill_floor_on_an_open_road(self, cp):
    # (b) the e2e candidate rises from the model's 0 toward the floor, and the planner's output
    # never goes over the MPC or cruise candidate
    with self.new(cp, True) as run:
      for tick in range(FRAMES):
        out = run.step(tick)
        args = out['args']
        floor = min(float(args['a_mpc']), float(args['a_cruise_prev']))
        self.assertEqual(args['a_model'], 0.)
        self.assertTrue(args['is_e2e'])
        self.assertGreaterEqual(out['candidate'], args['a_model'])
        self.assertLessEqual(out['candidate'], max(args['a_model'], floor) + EPS)
        self.assertLessEqual(out['output'], min(float(args['a_mpc']), args['a_cruise']) + EPS)
      self.assertGreater(out['candidate'], 0.5)  # lifted toward the approach pace
      self.assertEqual(out['output'], out['candidate'])
      self.assertGreater(args['v_target'], V_EGO)  # the cruise target after every ceiling, above today's speed here

  @over_cars(*BOTH_CARS)
  def test_param_on_uses_the_previous_frames_cruise_candidate(self, cp):
    with self.new(cp, True) as run:
      last = 0.
      for tick in range(30):
        out = run.step(tick)
        self.assertEqual(out['args']['a_cruise_prev'], last)
        last = out['args']['a_cruise']

  @over_cars(*BOTH_CARS)
  def test_param_on_behind_a_lead_is_chill(self, cp):
    # (c) the lead's MPC candidate binds: nothing is lifted over it and the output is what it is with the param off
    with self.new(cp, True) as on, self.new(cp, False) as off:
      for tick in range(FRAMES):
        args = {'lead': (22., -3.)}
        a, b = on.step(tick, **args), off.step(tick, **args)
        self.assertLessEqual(a['candidate'], max(a['args']['a_model'], float(a['args']['a_mpc'])) + EPS)
        self.assertLessEqual(a['output'], float(a['args']['a_mpc']) + EPS)
        self.assertAlmostEqual(a['output'], b['output'], places=9, msg=tick)
      self.assertLess(a['output'], 0.)  # it is braking for the lead

  @over_cars('ioniq6')
  def test_switch_off_hands_accel_boost_back_after_the_lift_is_out(self, cp):
    with self.new(cp, True) as run:
      for tick in range(60):
        self.assertTrue(run.step(tick, personality=AGGRESSIVE)['owned'])
      self.assertGreater(run.planner.set_speed_assist.lift, 0.)
      run.params.values['SetSpeedAssist'] = False
      owned = [run.step(60 + i, personality=AGGRESSIVE)['owned'] for i in range(80)]
      self.assertTrue(owned[0])  # the lift is still easing out: Accel Boost stays off
      self.assertFalse(owned[-1])
      self.assertEqual(owned, sorted(owned, reverse=True))  # handed back once, not flapping
      self.assertGreater(run.step(140, personality=AGGRESSIVE)['published'], 0.)


class TestStarPilotPlannerHook(HookTests, unittest.TestCase):
  kind = 'starpilot'


class TestUpstreamPlannerHook(HookTests, unittest.TestCase):
  kind = 'upstream'


if __name__ == "__main__":
  unittest.main()
