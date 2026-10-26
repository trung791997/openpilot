from openpilot.system.ubloxd.pigeond import INVALID_REINIT_SECONDS, ReceiveGuard

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
