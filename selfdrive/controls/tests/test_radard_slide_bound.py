"""SLIDE_BOUND (radard.py, D-102): a Bosch-A lead track that slides from the in-lane lead onto a slower car in the
next lane reads as the lead braking at ~10 m/s^2. Route 00000311 41:34: track 27 went 21 -> 5.6 m/s in 1.5 s while
the camera lead held 13-14 m/s. The lead's speed is floored at the camera's, on every slot carrying that track."""
from types import SimpleNamespace

from openpilot.selfdrive.controls import radard

DT = 0.05  # modelV2 cadence


def lead(v, d=60.0, trk=27):
  return SimpleNamespace(status=True, radar=True, radarTrackId=trk, vLead=v, vLeadK=v, vRel=v - 20.0, aLeadK=-1.0, dRel=d)


def vis(v, x=60.0, prob=0.99, a=-0.3):
  return SimpleNamespace(v=[v], x=[x], prob=prob, a=[a])


def run(sb, lead_v, cam_v, n, t0=0.0, others_fn=list, d=60.0, prob=0.99, a=-0.3):
  """Feed n ticks; lead_v / cam_v are functions of tick index. Returns the last lead and others."""
  out = None
  for i in range(n):
    ld, ot = lead(lead_v(i), d), others_fn()
    fired = sb.step(ld, ot, vis(cam_v(i), d, prob, a), cam_v(i), t0 + i * DT)
    out = (ld, ot, fired)
  return out


def slide(sb, cam_v=lambda i: 13.5, others_fn=list):
  # 2 s steady at 14 m/s, then the track drops 10 m/s^2 for 1 s
  run(sb, lambda i: 14.0, cam_v, 40)
  return run(sb, lambda i: 14.0 - 10.0 * i * DT, lambda i: cam_v(40 + i), 21, t0=40 * DT, others_fn=others_fn)


def test_slide_is_floored_at_camera_speed():
  ld, _, fired = slide(radard.SlideBound())
  assert fired
  assert abs(ld.vLead - (13.5 - radard.SLIDE_MARGIN)) < 1e-6
  assert abs(ld.vRel - (ld.vLead - 20.0)) < 1e-6 and ld.vLeadK == ld.vLead
  assert ld.aLeadK >= -0.3 - radard.SLIDE_A_MARGIN - 1e-6


def test_same_track_in_lead_two_is_floored_other_tracks_are_not():
  _, others, _ = slide(radard.SlideBound(), others_fn=lambda: [lead(4.0, trk=27), lead(4.0, trk=9)])
  assert abs(others[0].vLead - (13.5 - radard.SLIDE_MARGIN)) < 1e-6
  assert others[1].vLead == 4.0


def test_camera_trending_down_does_not_fire():
  # 299 seg 49: a real slowing lead, camera speed falling ~1.6 m/s^2
  sb = radard.SlideBound()
  _, _, fired = slide(sb, cam_v=lambda i: 20.0 - 1.6 * i * DT)
  assert not fired


def test_slow_decel_does_not_fire():
  sb = radard.SlideBound()
  run(sb, lambda i: 14.0, lambda i: 13.5, 40)
  _, _, fired = run(sb, lambda i: 14.0 - 5.0 * i * DT, lambda i: 13.5, 21, t0=40 * DT)
  assert not fired


def test_short_range_or_unsure_camera_does_not_fire():
  sb = radard.SlideBound()
  run(sb, lambda i: 14.0, lambda i: 13.5, 40, d=15.0)
  assert not run(sb, lambda i: 14.0 - 10.0 * i * DT, lambda i: 13.5, 21, t0=40 * DT, d=15.0)[2]
  sb = radard.SlideBound()
  run(sb, lambda i: 14.0, lambda i: 13.5, 40, prob=0.6)
  assert not run(sb, lambda i: 14.0 - 10.0 * i * DT, lambda i: 13.5, 21, t0=40 * DT, prob=0.6)[2]


def test_latch_holds_then_releases():
  sb = radard.SlideBound()
  slide(sb)
  # track stays slow; the latch holds without re-checking the decel
  t1 = 61 * DT
  assert run(sb, lambda i: 5.0, lambda i: 13.5, 20, t0=t1)[2]
  # camera loses the object: released, and no fresh decel to re-enter
  assert not run(sb, lambda i: 5.0, lambda i: 13.5, 1, t0=t1 + 20 * DT, prob=0.5)[2]
  assert not run(sb, lambda i: 5.0, lambda i: 13.5, 5, t0=t1 + 21 * DT)[2]


def test_latch_expires_after_hold():
  sb = radard.SlideBound()
  slide(sb)
  n = int(radard.SLIDE_HOLD_S / DT) + 10
  _, _, fired = run(sb, lambda i: 5.0, lambda i: 13.5, n, t0=61 * DT)
  assert not fired
