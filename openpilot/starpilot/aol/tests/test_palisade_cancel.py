"""Mixed Palisade Pause/Resume retains only existing, current lateral authority."""
from dataclasses import replace
from types import SimpleNamespace
from typing import cast
import pytest
from opendbc.car import structs
from openpilot.cereal import log
from openpilot.selfdrive.selfdrived.events import Events
from openpilot.selfdrive.selfdrived.selfdrived import SelfdriveD
from openpilot.starpilot.aol.intent import AolSettings, AOL_TOGGLE
from openpilot.starpilot.aol.runtime import AxisDecision
from openpilot.starpilot.aol.wire import IntentState, SafetyState, PandaSlot
from openpilot.starpilot.aol.tests.test_classic_brake_withdrawal import profile
from openpilot.starpilot.car.hyundai.aol import create_intent


class Messages(dict):
  ready = True
  def all_checks(self, services=None):
    return self.ready


@pytest.mark.parametrize('identity', ('palisade_hdai', 'palisade_hdaii'))
@pytest.mark.parametrize('fault', ('none', 'wrong_bus', 'stale', 'unmarked', 'stock'))
def test_cancel_source_is_bound_to_real_mixed_parser_receipt(identity, fault):
  from opendbc.can import CANPacker
  from opendbc.car import Bus
  from opendbc.car.hyundai.interface import CarInterface
  from openpilot.selfdrive.car.card import Car
  cp = profile(identity)
  if fault == 'unmarked':
    cp.alternativeExperience = 0
  elif fault == 'stock':
    cp.openpilotLongitudinalControl, cp.pcmCruise = False, True
    cp.safetyConfigs[0].safetyParam &= ~4
  ci = CarInterface(cp)
  parser = ci.can_parsers[Bus.pt]
  packer = CANPacker(parser.dbc.name)
  for tick in range(15):
    now = 2_000_000_000 + tick * 20_000_000
    packet = packer.make_can_msg('CLU11', 2 if fault == 'wrong_bus' else parser.bus,
      {'CF_Clu_CruiseSwState': 4 if tick == 14 else 0, 'CF_Clu_AliveCnt1': tick})
    cs = ci.update([(now, [packet])])
  if fault == 'wrong_bus':
    cs.buttonEvents = [structs.CarState.ButtonEvent(type='cancel', pressed=True)]
  received = now - 30_000_001 if fault == 'stale' else now
  caller = SimpleNamespace(CI=ci, _aol_can_observation=(cs, received, now, now))
  assert Car.aol_cancel_source(cast(Car, caller), cs, now) == (now if fault == 'none' else 0)
