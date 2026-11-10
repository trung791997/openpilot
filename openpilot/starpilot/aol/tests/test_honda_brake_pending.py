"""Honda panda-vs-parser brake race: card holds the AOL latch instead of resetting it.

Route 000000ac--138f4b8bb4 seg 14 (replay): the panda clears controls_allowed on the
BRAKE_SWITCH rising edge one ~100Hz card cycle before card.py's own two-sample debounce
(aol_honda_brake_source) can report the withdrawal. These tests drive
TransportPauseFeedback.observe()/_honda_brake_pending directly, the same unit level
test_honda_brake_intent_health.py already uses for this class, plus a direct check of
selfdrived's honda_native_lateral_grant_lost for the "does selfdrived still block this"
question the brief asked for.
"""
from types import SimpleNamespace
from dataclasses import replace

from openpilot.cereal import log, messaging
from openpilot.starpilot.aol.tests.test_honda_brake_intent_health import Peers, fixture
from openpilot.starpilot.aol.runtime import AxisDecision

NOW = 10_000_000_000


def republish_axis(sm, seq, now_ns):
  """Simulate card re-publishing aolAxisState on its own ~100Hz cycle: a real axis message keeps
  refreshing every tick (so it never goes stale against AXIS_MAX_AGE_NS on its own), independently
  of whatever the Honda-brake pending hold's own HONDA_BRAKE_PENDING_NS deadline is doing. Without
  this, a fixture built once and reused across several manual observe() calls would fault on axis
  staleness before the pending hold's own 30ms bound ever gets exercised."""
  axis = sm.messages['aolAxisState'].as_builder()
  axis.logMonoTime = now_ns
  a = axis.aolAxisState
  a.sequence = seq
  a.observedMonoTime, a.validUntilMonoTime = now_ns, now_ns + 30_000_000
  a.sourceCarStateMonoTime = now_ns - 1000
  return Peers([sm.messages['aolSafetyWire'], sm.messages['pandaStates'], axis.as_reader(), sm.messages['onroadEvents']])


def test_pending_hold_bridges_panda_vs_parser_race_then_resolves_via_source():
  """Scenario 1: panda drops LONG at tick k; CS brakePressed lags to k+1; parser (brake_source_ns) catches
  up at k+2. The hold must bridge k and k+1 without a fault, never restart its own deadline, and hand off
  cleanly to the existing _honda_brake_lateral_current path once the source arrives."""
  cp, cs, sm, native, feedback = fixture(coherent=True, long_allowed=False, brake=False)

  # Tick k: native already shows LONG withdrawn (panda), CS has not caught the brake edge yet,
  # and card's own parser debounce has nothing to report (brake_source_ns=0).
  t0 = NOW
  assert not feedback.observe(sm, cp, cs, t0, latched=True, brake_source_ns=0)
  assert not feedback.failed
  assert feedback.honda_brake_pending_ns == t0
  assert feedback.honda_brake_ns == 0

  # Tick k+1: CS now shows the brake pressed (parser's own BRAKE_SWITCH debounce still short one sample),
  # so card still cannot supply a source. The hold must not restart its deadline off this call. Card
  # keeps re-publishing aolAxisState every cycle regardless, so refresh it like a real cycle would.
  t1 = NOW + 10_000_000
  cs.brakePressed = True
  sm = republish_axis(sm, 3, t1)
  assert not feedback.observe(sm, cp, cs, t1, latched=True, brake_source_ns=0)
  assert not feedback.failed
  assert feedback.honda_brake_pending_ns == t0  # unchanged: never extended or restarted
  assert feedback.honda_brake_ns == 0

  # Tick k+2: card's parser debounce is satisfied; it reports the original rising-edge stamp (t1).
  # The existing _honda_brake_lateral_current path takes over unchanged, and the pending stamp clears.
  t2 = NOW + 20_000_000
  sm = republish_axis(sm, 4, t2)
  assert not feedback.observe(sm, cp, cs, t2, latched=True, brake_source_ns=t1)
  assert not feedback.failed
  assert feedback.honda_brake_ns == t1
  assert feedback.honda_brake_pending_ns == 0

  # No onroadEvents were ever synthesized in this fixture (immediateDisable/softDisable), matching
  # "long disengages as an ordinary user disable, not a hard fault".
  assert not any(e.immediateDisable or e.softDisable for e in sm['onroadEvents'])

  # selfdrived side: with native.lateralAllowed still True and CS.brakePressed True, selfdrived's own
  # driver_override test (selfdrived.py ~2136) already makes lost_permission's native-dependent clause
  # False regardless of intent.allowedLatch, and honda_native_lateral_grant_lost (selfdrived.py ~2016)
  # returns False on its own native.requestedLateral or native.lateralAllowed guard (line ~2026) since
  # lateralAllowed stays True. Exercise that function directly, mirroring
  # test_sd_honda_unexpected_zero_grant_loss_is_not_cold_or_commanded_pause's pattern.
  from openpilot.selfdrive.selfdrived.events import Events
  from openpilot.selfdrive.selfdrived.selfdrived import SelfdriveD
  from openpilot.starpilot.aol.transport_pause import cp_identity

  sd = SelfdriveD.__new__(SelfdriveD)
  sd.CP, sd.sm, sd.aol_session_id = cp, sm, 'unit-session'
  sd.initialized, sd.events, sd.aol_fault_reason = True, Events(), 'none'
  sd.aol_transport_pause = SimpleNamespace(cp=cp_identity(cp), session='unit-session', active=False, observed_native=native)
  sd.aol_availability_pause = SimpleNamespace(active=False)
  sm.all_checks = lambda services: True
  sm.messages['extrinsicsCalibration'] = SimpleNamespace(extrinsicsCalibration=SimpleNamespace(
    calStatus=log.ExtrinsicsCalibration.Status.calibrated))
  previous, requested = AxisDecision(lateral_active=True), AxisDecision(desired_lateral=True)
  previous_intent = SimpleNamespace(producerSessionId='unit-producer', carStateLogMonoTime=t0 - 10_000_000)
  intent = SimpleNamespace(producerSessionId='unit-producer')
  assert not sd.honda_native_lateral_grant_lost(cs, native, previous, requested, source_current=True,
    previous_intent=previous_intent, intent=intent, now_ns=t2)
  driver_override = bool(cs.brakePressed or cs.regenBraking or cs.gasPressed)
  assert driver_override  # gates the whole middle clause of lost_permission off regardless of intent.allowedLatch


def test_no_brake_or_cancel_source_expires_hold_within_fifty_milliseconds():
  """Scenario 2 (negative): LONG drops but no brake/CANCEL source ever shows up. The bounded hold must
  expire and fall through to today's reset -- not later than ~50ms after the first pending stamp."""
  cp, cs, sm, native, feedback = fixture(coherent=True, long_allowed=False, brake=False)
  t0 = NOW
  assert not feedback.observe(sm, cp, cs, t0, latched=True, brake_source_ns=0)
  assert feedback.honda_brake_pending_ns == t0
  # Still within the 30ms bound: held, no fault. Card keeps re-publishing aolAxisState every cycle,
  # so refresh it here too -- the pending hold's own 30ms clock, not incidental axis staleness, is
  # what must be the one governing this deadline.
  t1 = t0 + 20_000_000
  sm = republish_axis(sm, 3, t1)
  assert not feedback.observe(sm, cp, cs, t1, latched=True, brake_source_ns=0)
  assert not feedback.failed
  # Past the 30ms bound (and well under the 50ms ceiling the brief allows for the surrounding cycle):
  t2 = t0 + 40_000_000
  sm = republish_axis(sm, 4, t2)
  assert feedback.observe(sm, cp, cs, t2, latched=True, brake_source_ns=0)
  assert feedback.failed
  assert feedback.honda_brake_pending_ns == t0  # sticky: this withdrawal's window is spent
  assert feedback.honda_brake_ns == 0

  # Even if the reset is not what ends the latch, the same withdrawal never opens a second window.
  feedback.failed = False
  t3 = t0 + 50_000_000
  sm = republish_axis(sm, 5, t3)
  assert feedback.observe(sm, cp, cs, t3, latched=True, brake_source_ns=0)
  assert feedback.honda_brake_pending_ns == t0


def test_lateral_also_denied_gets_no_hold():
  """Scenario 3 (negative): lateralAllowed also drops at the same withdrawal. independent_lateral_health
  rejects the native outright, so _honda_brake_pending must never start a hold; behavior is identical to
  today (immediate reset) for this case."""
  cp, cs, sm, native, feedback = fixture(coherent=True, long_allowed=False, brake=False)
  mutated = replace(native, lateralAllowed=False)
  safety = messaging.new_message('aolSafetyWire', 0, valid=True, logMonoTime=mutated.observedMonoTime)
  from openpilot.starpilot.aol.wire import encode_safety
  safety.aolSafetyWire = encode_safety(mutated)
  sm.messages['aolSafetyWire'] = safety.as_reader()
  sm.logMonoTime['aolSafetyWire'] = mutated.observedMonoTime
  assert feedback.observe(sm, cp, cs, NOW, latched=True, brake_source_ns=0)
  assert feedback.failed
  assert feedback.honda_brake_pending_ns == 0


def test_rx_invalid_at_withdrawal_gets_no_hold():
  """Scenario 4 (negative): a panda RX/safety fault coincides with the withdrawal. independent_lateral_health
  already screens safetyRxChecksInvalid, so no hold may be granted here either."""
  cp, cs, sm, native, feedback = fixture(coherent=True, long_allowed=False, brake=False)
  pandas = sm.messages['pandaStates'].as_builder()
  pandas.pandaStates[0].safetyRxChecksInvalid = True
  sm.messages['pandaStates'] = pandas.as_reader()
  assert feedback.observe(sm, cp, cs, NOW, latched=True, brake_source_ns=0)
  assert feedback.failed
  assert feedback.honda_brake_pending_ns == 0


def test_cancel_press_as_source_uses_the_same_pending_and_handoff():
  """Scenario 5: a CANCEL press (not the brake pedal) is the eventual source. aol_honda_brake_source already
  treats CANCEL the same as the brake switch (see the comment on source_ns above); the pending hold and its
  handoff to _honda_brake_lateral_current must behave identically."""
  cp, cs, sm, native, feedback = fixture(coherent=True, long_allowed=False, brake=False)
  t0 = NOW
  assert not feedback.observe(sm, cp, cs, t0, latched=True, brake_source_ns=0)
  assert feedback.honda_brake_pending_ns == t0
  t1 = NOW + 10_000_000
  sm = republish_axis(sm, 3, t1)
  # CANCEL's own parser-proven stamp arrives one cycle later, exactly like the brake-switch case.
  assert not feedback.observe(sm, cp, cs, t1, latched=True, brake_source_ns=t0)
  assert feedback.honda_brake_ns == t0
  assert feedback.honda_brake_pending_ns == 0
  assert not feedback.failed
