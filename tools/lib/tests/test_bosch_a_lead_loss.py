"""Tests for tools/bosch_a_lead_loss.py: the episode logic on hand-built message streams.

Each detector has a negative control (D-009): a held radar lead, or a loss with no vision lead to
corroborate it, must produce no episode, or "found one" and "always finds one" look identical.
"""

import importlib.util
import os

_SCRIPT = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "..", "bosch_a_lead_loss.py"))
_spec = importlib.util.spec_from_file_location("bosch_a_lead_loss", _SCRIPT)
ll = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(ll)

DT = 0.05


def _drive(frames, vision=(0.9, 40.0), engaged=True, tracks_during_loss=()):
  """frames: list of track ids (int = radar lead with that id, None = no radar lead) at 20 Hz."""
  c = ll.LeadLossCensus()
  c.car_control(engaged, engaged)
  c.model(*vision)
  for i, tid in enumerate(frames):
    c.live_tracks([tid] if tid is not None else tracks_during_loss)
    c.radar_state(i * DT, tid is not None, tid is not None, tid if tid is not None else 0)
  return c


def test_held_lead_is_not_a_loss():
  c = _drive([7] * 100)
  assert c.episodes == []
  assert abs(c.radar_lead_s - 100 * DT) < 1e-6


def test_parser_drop_is_counted_and_attributed_to_the_parser():
  c = _drive([7] * 20 + [None] * 30 + [7] * 10)
  (e,) = c.episodes
  assert e["lost_id"] == 7 and e["why"] == "radar-back" and e["returned_id"] == 7
  assert abs((e["end"] - e["start"]) - 30 * DT) < 1e-6
  assert c.summary()["parser_losses"] == 1


def test_selection_drop_is_not_attributed_to_the_parser():
  c = _drive([7] * 20 + [None] * 30 + [7] * 10, tracks_during_loss=[7])
  assert len(c.episodes) == 1
  assert c.summary()["parser_losses"] == 0


def test_short_gap_is_below_the_threshold():
  assert _drive([7] * 20 + [None] * 10 + [7] * 10).episodes == []


def test_lead_not_held_long_enough_does_not_start_a_loss():
  assert _drive([7] * 5 + [None] * 40).episodes == []


def test_no_vision_lead_means_no_loss():
  assert _drive([7] * 20 + [None] * 40, vision=(0.1, 40.0)).episodes == []
  assert _drive([7] * 20 + [None] * 40, vision=(0.9, 120.0)).episodes == []


def test_disengaged_means_no_loss():
  assert _drive([7] * 20 + [None] * 40, engaged=False).episodes == []


def test_loss_ends_when_vision_goes_away():
  c = ll.LeadLossCensus()
  c.car_control(True, True)
  c.model(0.9, 40.0)
  t = 0.0
  for _ in range(20):
    c.radar_state(t, True, True, 7)
    t += DT
  for i in range(60):
    if i == 30:
      c.model(0.1, 40.0)
    c.radar_state(t, False, False, 0)
    t += DT
  (e,) = c.episodes
  assert e["why"] == "vision-gone"
  assert abs((e["end"] - e["start"]) - 30 * DT) < 1e-6
