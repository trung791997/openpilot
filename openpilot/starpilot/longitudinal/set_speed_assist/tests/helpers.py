"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Ported from zoompilot (e2e_set_speed tests helpers, commits e6baf0b50 and 2e9506a87) as unittest, see ../NOTICE.md.
"""
import math

import numpy as np

import openpilot.cereal.messaging as messaging
from openpilot.cereal import log
from openpilot.starpilot.longitudinal.set_speed_assist.controller import T_IDXS

# update() arguments for an engaged, lead-free e2e frame
ENGAGED = {'is_e2e': True, 'reset_state': False}


class FakeParams:
  """Dict-backed Params: the bool read the controller makes."""

  def __init__(self, **values):
    self.values = dict(values)

  def get_bool(self, key):
    return bool(self.values.get(key, False))


def near(a: float, b: float, rel: float = 1e-6, abs_tol: float = 1e-12) -> bool:
  return math.isclose(a, b, rel_tol=rel, abs_tol=abs_tol)


def plan_ramp(v_ego: float, drop: float) -> np.ndarray:
  """The model's plan easing from v_ego down by drop over the first 10 s of the horizon."""
  return v_ego - drop * np.minimum(T_IDXS, 10.) / 10.


def build_sm(v_ego: float, plan_v=None, yaw_rate=0., lane_change=False, lead=None) -> dict:
  """Readers for the services the controller reads. plan_v and yaw_rate take a scalar or a
  33-point trajectory on the model's T_IDXS. lead: (dRel, vRel) of a present leadOne."""
  cs = messaging.new_message('carState')
  cs.carState.vEgo = v_ego

  md = messaging.new_message('modelV2')
  vel = np.full(len(T_IDXS), v_ego) if plan_v is None else np.broadcast_to(np.asarray(plan_v, dtype=float), T_IDXS.shape)
  md.modelV2.velocity.x = vel.tolist()
  md.modelV2.orientationRate.z = np.broadcast_to(np.asarray(yaw_rate, dtype=float), T_IDXS.shape).tolist()
  md.modelV2.meta.laneChangeState = log.LaneChangeState.laneChangeStarting if lane_change else log.LaneChangeState.off

  rs = messaging.new_message('radarState')
  if lead is not None:
    rs.radarState.leadOne.present = True
    rs.radarState.leadOne.dRel, rs.radarState.leadOne.vRel = (float(x) for x in lead)

  return {'carState': cs.carState.as_reader(), 'modelV2': md.modelV2.as_reader(), 'radarState': rs.radarState.as_reader()}
