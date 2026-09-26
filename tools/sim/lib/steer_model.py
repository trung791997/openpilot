"""Steering response of a real car, fitted from its rlogs by tools/sim/eps_fit.py (SIM_STEER_MODEL).

The sim normally hands MetaDrive openpilot's desired steering angle, so the car's lateral tune and its
EPS never run. With a model, the bridge instead feeds the car's torque command (carOutput) through the
fitted response and steers MetaDrive with the resulting steering-wheel angle, which is also what the
car reads back on STEERING_SENSORS. Outside the fitted speed and command range it extrapolates.
"""
import json
from collections import deque


class SteerModel:
  def __init__(self, path: str):
    with open(path) as f:
      m = json.load(f)
    p = m["params"]
    self.a, self.b0, self.b1, self.c0, self.c1, self.bias = p["a"], p["b0"], p["b1"], p["c0"], p["c1"], p["bias"]
    self.delay = m["delay_frames"]
    self.dt = m["dt"]
    self.reset(0.0)

  def reset(self, angle: float) -> None:
    self.angle, self.rate = angle, 0.0
    self.u = deque([0.0] * (self.delay + 1), maxlen=self.delay + 1)

  def update(self, torque: float, v_ego: float) -> float:
    """Advance one dt (100 Hz) with the normalised torque command; returns steering-wheel angle (deg)."""
    self.u.append(torque)
    u_delayed = self.u[0]
    self.rate = self.a * self.rate + (self.b0 + self.b1 * v_ego) * u_delayed + (self.c0 + self.c1 * v_ego * v_ego) * self.angle + self.bias
    self.angle += self.rate * self.dt
    return self.angle
