"""Static: the LEAD badge's braking / pulling-away state, held so a noisy radar acceleration does not flicker it."""
import pytest

from openpilot.starpilot.system.android_auto.projection_theme import (BRAKING, PULLING_AWAY, STEADY, LeadTrend,
                                                                       LeadView, gap_bars)


def run(samples, dt=.05):
  clock = [0.]
  trend = LeadTrend(clock=lambda: clock[0])
  states = []
  for a_lead, v_rel in samples:
    states.append(trend.update(LeadView(30., v_rel, a_lead, 20.)))
    clock[0] += dt
  return states


def test_steady_lead_stays_yellow():
  assert set(run([(.3, 0.), (-.6, -.2), (.5, .3)] * 40)) == {STEADY}


def test_braking_needs_four_tenths_and_holds_through_a_blip():
  states = run([(-1.4, -1.)] * 6 + [(-1.4, -1.)] * 10 + [(-.7, -1.)] * 4 + [(.1, 0.)] * 30)
  assert states[5] == STEADY            # 0.25 s of braking is not enough
  assert states[9] == BRAKING           # 0.45 s is
  assert states[17] == BRAKING          # -0.7 is still inside the -0.5 release band
  assert states[20 + 10] == BRAKING     # 0.5 s after it eased off: held
  assert states[-1] == STEADY           # 0.8 s after it eased off: cleared


def test_alternating_noise_never_turns_red():
  assert BRAKING not in run([(-1.6, 0.), (.2, 0.)] * 60)


def test_pulling_away_needs_the_gap_opening():
  assert PULLING_AWAY not in run([(1., -.5)] * 40)  # speeding up but still closing
  assert run([(1., 1.)] * 40)[-1] == PULLING_AWAY


def test_braking_overrides_pulling_away_without_waiting_out_the_hold():
  states = run([(1., 1.)] * 20 + [(-2., -.5)] * 10)
  assert states[19] == PULLING_AWAY and states[-1] == BRAKING


def test_lost_or_bad_lead_resets():
  clock = [0.]
  trend = LeadTrend(clock=lambda: clock[0])
  for _ in range(20):
    trend.update(LeadView(30., -1., -2., 20.))
    clock[0] += .05
  assert trend.state == BRAKING
  assert trend.update(None) == STEADY
  assert trend.update(LeadView(30., float('nan'), -2., 20.)) == STEADY


@pytest.mark.parametrize('personality, bars', (('aggressive', 1), ('standard', 2), ('relaxed', 3), (0, 1), (1, 2), (2, 3), (None, 2)))
def test_gap_bars_follow_the_personality(personality, bars):
  assert gap_bars(personality) == bars


def test_gap_bars_take_the_capnp_enum():
  from cereal import log
  for name, bars in (('aggressive', 1), ('standard', 2), ('relaxed', 3)):
    assert gap_bars(log.LongitudinalPersonality.schema.enumerants[name]) == bars


@pytest.mark.parametrize('nargs', [4, 5])
def test_glow_matches_either_raylib_gradient_signature(monkeypatch, nargs):
  # The comma's raylib takes (Vector2, radius, inner, outer); passing ints there killed the car view.
  from types import SimpleNamespace
  from openpilot.starpilot.system.android_auto import projection_theme as theme
  calls = []
  fake = SimpleNamespace(ffi=SimpleNamespace(typeof=lambda fn: SimpleNamespace(args=[0] * nargs)),
                         rl=SimpleNamespace(DrawCircleGradient=None), Vector2=lambda x, y: ('v', x, y),
                         draw_circle_gradient=lambda *a: calls.append(a))
  monkeypatch.setattr(theme, 'rl', fake)
  theme._glow(10.6, 20.2, 5., 'in', 'out')
  assert calls == [(('v', 10.6, 20.2), 5., 'in', 'out')] if nargs == 4 else [(10, 20, 5., 'in', 'out')]
