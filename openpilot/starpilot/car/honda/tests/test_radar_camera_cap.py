"""Camera cap (static). The positive case is shaped on b0 seg17 track 53: radar 70 m closing at 5 m/s, the camera
lead at the same range holding it (closing ~0.5 m/s, a 0, range slope matching)."""
from collections import deque
from types import SimpleNamespace

import pytest

from openpilot.common.realtime import DT_MDL
from openpilot.starpilot.car.honda.radar_camera_cap import (CAP_DA_RATE, CAP_DV_RATE, CAP_GAP_MIN, CAP_MIN_D, CAP_PROB,
                                                            RADAR_TO_CAMERA, CameraCap)
from openpilot.starpilot.car.honda.radar_lead_bound import TREND_SAMPLES

V_EGO = 25.0
TID = 53


def radar_lead(tid=TID, d=70.0, v_rel=-5.0, a=-0.4):
  return {'dRel': d, 'yRel': 0.0, 'vRel': v_rel, 'vLead': V_EGO + v_rel, 'vLeadK': V_EGO + v_rel, 'aLeadK': a,
          'aLeadTau': 0.6, 'present': True, 'modelProb': 0.95, 'radar': True, 'radarTrackId': tid}


def vision(d=70.0, v_rel=-0.5, a=0.0):
  return SimpleNamespace(x=[d + RADAR_TO_CAMERA], v=[V_EGO + v_rel], a=[a])


def ranges(d=70.0, slope=-0.5):
  return deque([d + RADAR_TO_CAMERA + slope * (i - (TREND_SAMPLES - 1)) * DT_MDL for i in range(TREND_SAMPLES)],
               maxlen=TREND_SAMPLES)


ABSENT = {'present': False}
TRACKS = {TID: object(), 54: object()}


def step(cap, lead=None, model=None, prob=0.95, camera=None, tracks=TRACKS):
  lead = lead or radar_lead()
  return cap.apply([lead, ABSENT], [model or vision(), vision()], V_EGO, [prob, 0.0], tracks,
                   [camera if camera is not None else ranges(), deque()])[0]


def run(cap, frames, **kw):
  out = None
  for _ in range(frames):
    out = step(cap, **kw)
  return out


def test_raises_to_camera_closing_speed_slewed():
  cap = CameraCap()
  out = step(cap)
  assert out['vRel'] == pytest.approx(-5.0 + CAP_DV_RATE * DT_MDL)
  out = run(cap, 20)
  assert out['vRel'] == pytest.approx(-0.5)
  assert out['vLead'] == pytest.approx(V_EGO - 0.5)
  assert out['vLeadK'] == pytest.approx(V_EGO - 0.5)
  assert out['dRel'] == 70.0 and out['aLeadK'] == pytest.approx(0.0)


def test_lifts_aleadk_slewed_never_lowers():
  cap = CameraCap()
  out = step(cap, lead=radar_lead(a=-2.0))
  assert out['aLeadK'] == pytest.approx(-2.0 + CAP_DA_RATE * DT_MDL)
  out = run(CameraCap(), 20, model=vision(a=-0.3))
  assert out['aLeadK'] == pytest.approx(-0.3)
  out = run(CameraCap(), 20, lead=radar_lead(a=0.5))   # radar lead already accelerating harder: untouched
  assert out['aLeadK'] == 0.5


def test_releases_slewed_when_camera_drops():
  cap = CameraCap()
  run(cap, 20)
  out = step(cap, prob=CAP_PROB - 0.01)
  assert out['vRel'] == pytest.approx(-0.5 - CAP_DV_RATE * DT_MDL)
  out = run(cap, 20, prob=0.0)
  assert out['vRel'] == pytest.approx(-5.0)


def test_new_track_restarts_at_zero():
  cap = CameraCap()
  run(cap, 20)
  out = step(cap, lead=radar_lead(tid=54))
  assert out['vRel'] == pytest.approx(-5.0 + CAP_DV_RATE * DT_MDL)


@pytest.mark.parametrize('kw', [
  {'lead': radar_lead(d=CAP_MIN_D - 1.0)},                        # near: radar speed stands
  {'prob': CAP_PROB - 0.01},                                      # unconfident camera
  {'model': vision(d=100.0)},                                     # camera on another object (> 30 % off)
  {'model': vision(a=-1.0)},                                      # camera lead slowing: no witness
  {'camera': ranges(slope=-6.0)},                                 # camera range slope disagrees with its speed
  {'camera': deque([float('nan')] * TREND_SAMPLES)},              # no camera history
  {'model': vision(v_rel=-5.0 + CAP_GAP_MIN - 0.1)},              # gap not over CAP_GAP_MIN
  {'tracks': {}},                                                 # track gone
])
def test_gates_leave_lead_untouched(kw):
  cap = CameraCap()
  out = run(cap, 10, **kw)
  base = kw.get('lead', radar_lead())
  assert out['vRel'] == base['vRel'] and out['vLead'] == base['vLead']


def test_never_more_braking_and_vision_leads_untouched():
  cap = CameraCap()
  out = run(cap, 10, model=vision(v_rel=-8.0))   # camera closes faster than radar: no cap (that is CameraSlowBound's side)
  assert out['vRel'] == -5.0
  vision_lead = dict(radar_lead(), radar=False)
  assert run(CameraCap(), 10, lead=vision_lead) == vision_lead
