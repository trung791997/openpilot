#!/usr/bin/env python3
"""Open-loop replay of one recorded sim run through LatControlClarityEps in two NRDR_OVERRIDE_MODEs (James's condition 4).

Both modes get the same recorded inputs, frame by frame (carState angle/rate/vEgo/steeringPressed/steeringTorque,
liveParameters sr/stiffness/offset/roll, desiredCurvature, latActive), so any difference in the output is the mode's own.
Hands-off: the pass mark is max |C - A| == 0.0.

--blip T,N,TQ forces |steeringTorque| = max(|tq|, TQ) (driver's sign; the plan's when hands-off) for N frames from T s, into both runs of the pair, and
reports the pair's max change in output against the same mode without the blip (Kevin's blip_30 / blip_45 in open loop).
steeringPressed follows the car's raw rule during the blip: |tq| > --thr.

Needs the patched controller: --ctl FILE loads a patched copy (the owner file stays untouched); without it the tree's file is used.

  .venv/bin/python tools/sim/override_openloop.py /tmp/simroutes/ovr_handsoff_30_0_1800_A [--modes A,C] [--blip 5,3,2100]
Sim evidence only.
"""
import argparse
import math
from types import SimpleNamespace

import numpy as np

from cereal import car
from opendbc.car.vehicle_model import VehicleModel
from openpilot.common.realtime import DT_CTRL
import openpilot.selfdrive.controls.lib.latcontrol_clarity_eps as lce_tree
lce = lce_tree


def load_ctl(path):
  """A patched copy of the controller loaded from a file, so the replay never needs the owner file swapped."""
  import importlib.util
  spec = importlib.util.spec_from_file_location("latcontrol_clarity_eps_patched", path)
  mod = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(mod)
  return mod


def replay(d, cp, mode, blip=None, thr=1800.0):
  if hasattr(lce, "OVERRIDE_MODE"):
    lce.OVERRIDE_MODE = mode
  elif mode != "A":
    raise SystemExit("controller is not patched: no OVERRIDE_MODE (apply .claude/ovr_bc.patch first)")
  ctl = lce.LatControlClarityEps(cp, None, DT_CTRL)
  vm = VehicleModel(cp)
  t = d["t"] - d["t"][0]
  out = np.zeros(len(t))
  blip_i = None
  if blip is not None:
    t_b, n_b, tq_b = blip
    i0 = int(np.searchsorted(t, t_b))
    blip_i = range(i0, min(i0 + int(n_b), len(t)))
  for i in range(len(t)):
    tq, pressed = float(d["eps_torque"][i]), bool(d["pressed"][i])
    if blip_i is not None and i in blip_i:
      tq = math.copysign(max(abs(tq), tq_b), tq if tq else (d["des_angle"][i] or 1.0))  # driver's sign; hands-off: the plan's
      pressed = abs(tq) > thr
    cs = SimpleNamespace(steeringAngleDeg=float(d["angle"][i]), steeringRateDeg=float(d["rate"][i]), vEgo=float(d["v"][i]),
                         steeringPressed=pressed, steeringTorque=tq)
    if d["sr"][i] > 0:
      vm.update_params(max(float(d["stiff"][i]), 0.1), float(d["sr"][i]))
    params = SimpleNamespace(roll=float(d["roll"][i]), angleOffsetDeg=float(d["offset"][i]))
    o, _, _ = ctl.update(bool(d["active"][i]), cs, vm, params, False, float(d["des_curv"][i]), False, 0.0, None, None, None)
    out[i] = o
  return t, out


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("run")
  ap.add_argument("--modes", default="A,C")
  ap.add_argument("--blip", default=None, help="T,N,TQ")
  ap.add_argument("--thr", type=float, default=1800.0)
  ap.add_argument("--ctl", default=None, help="patched controller file (e.g. .claude/ovr_ctl/latcontrol_clarity_eps_C_v2.py)")
  a = ap.parse_args()
  global lce
  if a.ctl:
    lce = load_ctl(a.ctl)
  d = np.load(f"{a.run}/lat_pid_sim.npz", allow_pickle=True)
  with car.CarParams.from_bytes(d["cp_bytes"].tobytes()) as cp_r:
    cp = cp_r.as_builder()
  m0, m1 = a.modes.split(",")
  t, o0 = replay(d, cp, m0, thr=a.thr)
  _, o1 = replay(d, cp, m1, thr=a.thr)
  act = d["active"] > 0
  print(f"{a.run}: {len(t)} frames, {act.sum()} active; max |eps_torque| {np.abs(d['eps_torque']).max():.1f}")
  diff = np.abs(o1 - o0)
  print(f"  {m1} vs {m0}: max |d output| = {diff.max()!r} (frames differing: {(diff > 0).sum()})")
  print(f"  recorded co_torque vs replay {m0}: max |d| = {np.abs(d['co_torque'][act] - o0[act]).max():.4f} (open loop; info only)")
  if a.blip:
    blip = tuple(float(x) for x in a.blip.split(","))
    for m, base in ((m0, o0), (m1, o1)):
      _, ob = replay(d, cp, m, blip, a.thr)
      db = np.abs(ob - base)
      w = (t >= blip[0] - 0.1) & (t < blip[0] + 0.6)  # Kevin's window, 4.9-5.6 s for a blip at 5.0 s
      k = int(np.argmax(np.where(w, db, 0.0)))
      step_b, step_0 = np.abs(np.diff(ob))[w[1:]].max(), np.abs(np.diff(base))[w[1:]].max()
      print(f"  blip {a.blip} mode {m}: window max |d output| {db[w].max():.4f} at t {t[k]:.2f} s (v {d['v'][k]:.1f} m/s, "
            f"plan {d['des_angle'][k]:.1f} deg); max per-frame step {step_b:.4f} (no blip {step_0:.4f}); whole run {db.max():.4f}")


if __name__ == "__main__":
  main()
