from openpilot.system.ubloxd.pigeond import INVALID_REINIT_SECONDS, LateAiding, ReceiveGuard

FRAME = b"\xb5\x62\x01\x07\x00\x00"


def test_good_data_passes_through():
  guard = ReceiveGuard(0.)
  assert guard.check(FRAME, 1.) == (FRAME, False, False)


def test_stray_zero_before_a_frame_is_dropped_without_reinit():
  guard = ReceiveGuard(0.)
  assert guard.check(b"\x00\x00" + FRAME, 1.) == (FRAME, True, False)
  assert guard.check(FRAME, 1.1) == (FRAME, False, False)


def test_brief_zero_burst_does_not_power_cycle():
  guard = ReceiveGuard(0.)
  guard.check(FRAME, 10.)
  results = [guard.check(b"\x00" * 8, 10. + i * .1) for i in range(int(INVALID_REINIT_SECONDS * 10))]
  assert [first for _, first, _ in results] == [True] + [False] * (len(results) - 1)
  assert not any(reinit for _, _, reinit in results)
  assert guard.check(FRAME, 13.) == (FRAME, False, False)


def test_garbage_that_lasts_still_reinits():
  guard = ReceiveGuard(0.)
  guard.check(FRAME, 10.)
  assert not guard.check(b"\x00\x13\x7f", 11.)[2]
  assert guard.check(b"\x00\x13\x7f", 10. + INVALID_REINIT_SECONDS + .1)[2]
  guard.reset(20.)
  assert not guard.check(b"\x00", 21.)[2]


def test_zero_with_garbage_after_publishes_the_rest():
  guard = ReceiveGuard(0.)
  assert guard.check(b"\x00\x13\x7f", 1.)[0] == b"\x13\x7f"


def test_aided_start_never_sends_again():
  late = LateAiding(True, 0.)
  assert not any(late.due(float(t)) for t in range(1, 100))


def test_unaided_start_checks_the_clock_once_a_second_until_aided():
  late = LateAiding(False, 0.)
  assert not late.due(.5)
  assert late.due(1.)
  assert not late.due(1.5)
  assert late.due(2.)  # clock still invalid: keep checking
  late.aided = True
  assert not late.due(10.)


def test_reinit_without_a_valid_clock_waits_for_it_again():
  late = LateAiding(True, 0.)
  late.reset(False, 50.)
  assert not late.due(50.5) and late.due(51.)
