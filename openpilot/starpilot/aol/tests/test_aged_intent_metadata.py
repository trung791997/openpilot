"""An aged conflating cache is metadata, never a new raw integrity violation."""
from contextlib import ExitStack
from dataclasses import replace
from unittest.mock import patch

import pytest
from opendbc.car.gm.tests.test_bolt_pedal import params
from opendbc.car.gm.values import CAR
from openpilot.cereal import messaging
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.selfdrive.selfdrived.events import EventName
from openpilot.starpilot.aol.intent_companion import IntentCompanion
from openpilot.starpilot.aol.wire import decode_intent, encode_intent
from openpilot.starpilot.aol.tests import test_ioniq_availability as fixture
from openpilot.starpilot.aol.tests.test_input_deadline_continuity import intent

BASE = 10_000_000_000
SERVICE = 'aolIntentWire'


def drain(companion, message):
  with patch('openpilot.starpilot.aol.intent_companion.messaging.recv_one_or_none', side_effect=[message.as_reader(), None]):
    assert companion.drain()


def test_actual_bolt_source_wait_survives_raw_successor_before_conflated_cache_recovers():
  with OpenpilotPrefix(), ExitStack() as cleanup:
    case = fixture.IoniqAvailabilityTests(methodName='runTest')
    cleanup.callback(case.doCleanups)
    cp = params(CAR.CHEVROLET_BOLT_CC_2018_2021, pedal=True, camera=True)
    sd, controls, step = case._authority_callers(False, cp=cp, genuine_events=True)
    for tick in (0, 1):
      axis, _, command = step(tick, native_request_long=False, panda_controls_allowed=False)
      assert axis.lateralActive and command.latActive and not command.longActive
    old = intent(BASE + 10_000_000, 2)
    for tick in (4, 7):
      axis, _, command = step(tick, offset_ns=1_000_000 if tick == 4 else 0, timeout=True, state_stamp=BASE + 10_000_000,
        intent_message=old, native_request_long=False, panda_controls_allowed=False)
      assert str(axis.faultReason) == 'availabilityPause'
      assert not axis.lateralActive and not command.latActive and not command.longActive
    epoch = sd.aol_intent_companion.integrity_epoch
    original_update = sd.sm.update_msgs

    def cached_poll(now, messages):
      original_update(now, [m for m in messages if m.which() != SERVICE])
      sd.sm.alive[SERVICE] = False

    successor = intent(BASE + 110_000_000, 12)
    with patch.object(sd.sm, 'update_msgs', side_effect=cached_poll):
      axis, state, command = step(11, timeout=True, state_stamp=BASE + 10_000_000,
        intent_message=successor, native_request_long=False, panda_controls_allowed=False)
    assert sd.aol_intent_companion.integrity_epoch == epoch
    assert str(axis.faultReason) == 'availabilityPause' and axis.desiredLateral and axis.nativeAcknowledged
    assert not axis.lateralActive and not axis.longitudinalActive and axis.sourceCarStateMonoTime == 0
    assert not command.latActive and not command.longActive and not state.enabled
    assert EventName.controlsMismatch not in sd.events.names and not sd.aol_authority_lost
    axis, _, command = step(12, native_request_long=False, panda_controls_allowed=False)
    assert axis.lateralActive and command.latActive and not command.longActive
    assert EventName.controlsMismatch not in sd.events.names
