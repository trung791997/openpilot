"""Honda CANCEL ends only longitudinal; the AOL latch keeps lateral (real FW/parser/Card/Controls/native)."""
from opendbc.car.honda.values import CAR
from openpilot.starpilot.aol.tests.test_honda_recovery_lifecycle import TestHondaCurrentCaller as CurrentCaller


class TestHondaCancel(CurrentCaller):
  # Peter's route 000000a2 segs 15/16: CANCEL dropped the AOL latch (disarming_fault) and lateral with it.
  def test_civic_bosch_alpha_cancel(self):
    self.run_current_caller(candidate=CAR.HONDA_CIVIC_BOSCH, word=2082, alpha=True, cancel=True)

  # The Clarity can't run here (its cruiseState.enabled stays False); the Civic pedal shares word 544.
  def test_pedal_cancel(self):
    self.run_current_caller(candidate=CAR.HONDA_CIVIC, word=544, pedal=True, cancel=True)
