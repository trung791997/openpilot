from opendbc.can import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.honda.carcontroller import icbm_counter_sync_step
from opendbc.car.honda.hondacan import CanBus, spam_buttons_command
from opendbc.car.honda.values import CAR, DBC, CruiseButtons


def _run(car_counters, decreasing):
  last, phase, sent = -1, 0, []
  for car_counter in car_counters:
    counter, last, phase = icbm_counter_sync_step(car_counter, last, phase, decreasing)
    sent.append(counter)
  return sent


def _per_car_frame(car_counters, decreasing):
  # 100 Hz control loop sees each 25 Hz car frame four times; only a new counter can trigger a send.
  sent = _run([c for c in car_counters for _ in range(4)], decreasing)
  return [next((c for c in sent[i:i + 4] if c is not None), None) for i in range(0, len(sent), 4)]


def test_decrease_presses_two_car_frames_then_releases_two_with_car_counter_plus_one():
  counters = [i % 4 for i in range(8)]
  assert _per_car_frame(counters, decreasing=True) == [1, 2, None, None, 1, 2, None, None]


def test_increase_releases_for_three_car_frames():
  counters = [i % 4 for i in range(10)]
  assert _per_car_frame(counters, decreasing=False) == [1, 2, None, None, None, 2, 3, None, None, None]


def test_only_one_send_per_car_frame():
  sent = _run([0, 0, 0, 0, 1, 1, 1, 1], decreasing=True)
  assert sent == [1, None, None, None, 2, None, None, None]


def test_no_press_without_a_car_counter():
  assert _run([-1] * 8, decreasing=True) == [None] * 8


def test_spam_buttons_command_uses_explicit_counter():
  fingerprint = CAR.HONDA_CIVIC_BOSCH
  packer = CANPacker(DBC[fingerprint][Bus.pt])
  can = CanBus(structs.CarParams(carFingerprint=fingerprint))
  for counter in (2, 0, 3):
    addr, dat, _bus = spam_buttons_command(packer, can, CruiseButtons.DECEL_SET, fingerprint, counter=counter)
    assert addr == 0x296
    assert (dat[3] >> 4) & 3 == counter
    assert dat[0] >> 5 == CruiseButtons.DECEL_SET
