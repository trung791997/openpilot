"""Nidec pedal actual acquisition/parser/Card/Controls/native recovery."""
from opendbc.car.honda.values import CAR
from openpilot.starpilot.aol.tests.test_honda_recovery_lifecycle import TestHondaCurrentCaller as CurrentCaller

class TestHondaPedal(CurrentCaller):
  def test_pedal_0_eps_reverse_recovery(self):
    self.run_current_caller(candidate=CAR.HONDA_CIVIC, word=544, pedal=True)

  def test_pedal_1_eps_reverse_recovery(self):
    self.run_current_caller(candidate=CAR.ACURA_ILX, word=548, pedal=True)

  def test_pedal_2_eps_reverse_recovery(self):
    self.run_current_caller(candidate=CAR.HONDA_ODYSSEY_TWN, word=804, pedal=True)

  # The panda's brake reaches pandad's native read before Card's CAN batch (00000098 seg29 1742.8 s): Card saw
  # LONG withdrawn with brakePressed still False, cleared the latch and selfdrived raised controlsMismatch.
  # The Clarity itself can't run here (its cruiseState.enabled stays False); the Civic pedal shares word 544.
  def test_pedal_brake_race(self):
    self.run_current_caller(candidate=CAR.HONDA_CIVIC, word=544, pedal=True, brake_race=True)
