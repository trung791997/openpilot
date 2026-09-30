#!/usr/bin/env python3
"""Synthetic route .npz files (extract.py format) with planted facts, plus a test that runs analyze.py on them.

Usage: PYTHONPATH=<openpilot repo> python make_synth.py OUTDIR [--test]

Planted facts (route synth_stock_A unless noted):
  P1 radar4xx (0x400-0x416,0x420,0x440) at 50 Hz on bus 1, own clock; 0x1DF shares that clock.
  P2 0x33D at 28.7 Hz on bus 2; the bank (0x280-0x2FF, 80 frames/sweep) starts 4 ms after every 2nd 0x33D
     frame (14.35 Hz) on bus 2 -> bank locked to 0x33D, independent of radar4xx / 0x1DF / 0xE4.
  P3 radar4xx + 0x1DF silent 120.0-125.0 s; the bank keeps publishing.
  P4 0x410 byte 2 = round(5 * vEgo(t - 0.2 s)); byte 5 changes only at 150 s and 250 s; every radar4xx frame has a
     Honda counter at B7[5:4] and honda_checksum in B7[3:0]. 0x1DB byte 3 = round(2 * lead dRel).
  P5 0x1DF packed with opendbc's CANPacker on the Civic Bosch DBC: brake episode 1 onset 100.3 s, 0.3 s after a
     lead swap at 100.0 s (cut-in, dRel 60 -> 25 m); episode 2 onset 199.67 s, 1.0 s after lead vRel crosses -1 m/s,
     no swap; a 0.3 s dip below -1 at 250 s must NOT count. AEB_PREPARE asserted 150.0-151.0 s; ACC_HUD CHIME
     150.2 s; ACC_HUD HUD_LEAD changes at 100.0 s.
  P6 bank objects packed with CANPacker on honda_bosch_a_radar: slots 1-3 parked cars (|y|<=4) whose U11 vRel =
     -vEgo(t - 0.15 s) while their range tracks the true position (no lag); slot 4 an oncoming car pinned at the
     U11 low rail (raw 0); slot 0 the lead; slots 5-15 invalid.
  synth_alpha_B: op_long route (0x1DF only as sendcan), same bank/33D structure, no stall.
  synth_sparse_C: carState + 0xE4 only (everything else must print absent, not crash).
"""
import json
import os
import re
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
RNG = np.random.default_rng(7)
RADAR4XX = list(range(0x400, 0x417)) + [0x420, 0x440]


def honda_chk(addr, d):
  from opendbc.car.honda.hondacan import honda_checksum
  return honda_checksum(addr, None, bytearray(d))


class Can:
  def __init__(self):
    self.rows = []  # (t_true, src, addr, kind, bytes8)

  def add(self, t, bus, addr, dat, kind=0):
    self.rows.append((t, bus, addr, kind, bytes(dat)[:8].ljust(8, b"\0")))


def ego_profile(T, events):
  t = np.arange(0, T, 0.01)
  a = np.zeros_like(t)
  for t0, t1, acc in events:
    a[(t >= t0) & (t < t1)] = acc
  v = 10.0 + np.cumsum(a) * 0.01
  return t, np.maximum(v, 2.0), a


def build(route, T, op_long, stall, with_lead_script, sparse=False):
  from opendbc.can.packer import CANPacker
  from opendbc.car.honda.radar_interface import BOSCH_A_MAIN_IDS, BOSCH_A_AUX_IDS
  pt = CANPacker("honda_civic_hatchback_ex_2017_can_generated")
  bk = CANPacker("honda_bosch_a_radar")
  ev = [(40, 42, -2.0), (50, 52, 1.5), (100.6, 103, -2.5), (110, 113, 1.5), (160, 162, -1.8), (170, 172, 1.6),
        (199.8, 202, -2.0), (215, 218, 1.5), (260, 262, -2.0), (270, 272, 1.5)]
  cs_t, vE, aE = ego_profile(T, [e for e in ev if e[1] < T])
  vEgo = lambda tq: np.interp(tq, cs_t, vE)
  # lead (radarState, 20 Hz)
  rs_t = np.arange(0, T, 0.05)
  vRel = np.zeros_like(rs_t)
  if with_lead_script:
    m = (rs_t >= 100.0) & (rs_t < 105.0)
    vRel[m] = -4.0 + 4.0 * (rs_t[m] - 100.0) / 5.0
    m = (rs_t >= 198.0) & (rs_t < 200.0)
    vRel[m] = -3.0 * (rs_t[m] - 198.0) / 2.0
    m = (rs_t >= 200.0) & (rs_t < 206.0)
    vRel[m] = -3.0 + 3.0 * (rs_t[m] - 200.0) / 6.0
  dRel = 60.0 + np.cumsum(vRel) * 0.05
  if with_lead_script:
    dRel[rs_t >= 100.0] -= 35.0  # cut-in: dRel jumps 60 -> 25 at 100.0
  status = np.ones_like(rs_t)
  lead_d = lambda tq: np.interp(tq, rs_t, dRel)
  lead_v = lambda tq: np.interp(tq, rs_t, vRel)

  C = Can()
  # E4 rx bus 0 100 Hz, own clock
  for t in np.arange(0.003, T, 0.01 * (1 - 3e-4)):
    C.add(t, 0, 0xE4, RNG.integers(0, 256, 8, dtype=np.uint8))
  if not sparse:
    # radar clock: radar4xx + 0x1DF at 50 Hz
    k = 0
    for t in np.arange(0.0037, T, 0.02 * (1 + 2e-4)):
      k += 1
      if stall and 120.0 <= t < 125.0:
        continue
      for addr in RADAR4XX:
        d = RNG.integers(0, 256, 8, dtype=np.uint8)
        d[0] = addr & 0xFF
        if addr == 0x410:
          d[1] = RNG.integers(0, 256)
          d[2] = int(np.clip(round(5 * vEgo(t - 0.2)), 0, 255))
          d[3:5] = 0
          d[5] = 1 if 150.0 <= t < 250.0 else 0
          d[6] = 0x40
        d[7] = (k % 4) << 4
        d[7] |= honda_chk(addr, d)
        C.add(t + 0.0001 * (addr & 0x1F), 1, addr, d)
      # 0x1DF
      if op_long:
        vals = dict(ACCEL_COMMAND=0.0, CONTROL_ON=5, GAS_COMMAND=0)
        C.add(t, 0, 0x1DF, pt.pack(0x1DF, vals), kind=1)
      else:
        cmd = 0.0
        if 100.3 <= t < 100.6: cmd = -0.35 - (t - 100.3) / 0.3 * 0.85
        elif 100.6 <= t < 102.0: cmd = -2.0
        elif 102.0 <= t < 102.3: cmd = -0.9  # soft gap > 0.15 s, then a 2nd hard run in the SAME brake:
        elif 102.3 <= t < 103.0: cmd = -1.4  # must stay one episode (regression: duplicate onset rows)
        elif 103.0 <= t < 103.2: cmd = -0.5
        elif 199.67 <= t < 199.9: cmd = -0.4
        elif 199.9 <= t < 201.5: cmd = -1.6
        elif 250.0 <= t < 250.3: cmd = -1.5
        elif 259.0 <= t < 260.0: cmd = -2.5   # moving brake that ends in the standstill hold ->
        elif 260.0 <= t < 262.0: cmd = -4.0   # class 'stop', floor frames all under STANDSTILL, peak while moving -2.5
        elif 280.0 <= t < 281.0: cmd = -1.5   # right after CONTROL_ON 0->5 at 279.0 -> class 'engage'
        vals = dict(ACCEL_COMMAND=cmd, CONTROL_ON=0 if 277.0 <= t < 279.0 else 5, GAS_COMMAND=0, BRAKE_REQUEST=int(cmd < -0.3),
                    STANDSTILL=int(260.0 <= t < 262.0), AEB_PREPARE=int(150.0 <= t < 151.0))
        C.add(t, 1, 0x1DF, pt.pack(0x1DF, vals))
    # 10 Hz: 0x30C, 0x39F, 0x1DB, 0x1EF (radar clock-ish)
    for t in np.arange(0.011, T, 0.1):
      if not op_long:
        C.add(t, 1, 0x30C, pt.pack(0x30C, dict(CRUISE_SPEED=90, ACC_ON=1, HUD_LEAD=2 if t >= 100.0 else 1,
                                                CHIME=1 if 150.2 <= t < 150.7 else 0, PCM_SPEED=90)))
        C.add(t, 1, 0x39F, pt.pack(0x39F, dict(SET_TO_1=1, SET_TO_64=64)))
      d = np.zeros(8, np.uint8)
      d[3] = int(np.clip(round(2 * lead_d(t)), 0, 255))
      d[1] = RNG.integers(0, 256)
      C.add(t + 0.002, 1, 0x1DB, d)
      C.add(t + 0.003, 1, 0x1EF, bytes([0x80, 0xFF, 0, 75, 30, 0, 0, 0]))
    for t in np.arange(0.5, T, 1.0):
      C.add(t, 1, 0x640, bytes([1, 2, 3, 4, 5, 6, 7, 8]))
      C.add(t + 0.001, 1, 0x641, RNG.integers(0, 256, 8, dtype=np.uint8))
    for t in np.arange(0.006, T, 0.02):
      C.add(t, 0, 0x1DD, RNG.integers(0, 256, 8, dtype=np.uint8))
    # 0x33D and the bank
    x_st = {1: 70.0, 2: 45.0, 3: 20.0}
    y_st = {1: 4.0, 2: -4.0, 3: 3.0}
    tid_st = {1: 10, 2: 11, 3: 12}
    next_tid = 13
    x_onc = 80.0
    sweep = 0
    t_prev = 0.0
    for j, t33 in enumerate(np.arange(0.011, T, 0.0348432)):
      C.add(t33, 2, 0x33D, RNG.integers(0, 256, 8, dtype=np.uint8))
      if j % 2:
        continue
      ts = t33 + 0.004
      dt = ts - t_prev
      t_prev = ts
      ve = vEgo(ts)
      vmid = vEgo(ts - dt / 2)  # midpoint integration: range carries no lag
      for s in x_st:
        x_st[s] -= vmid * dt
        if x_st[s] < 5.0:
          x_st[s] = 75.0
          tid_st[s] = next_tid
          next_tid = next_tid + 1 if next_tid < 63 else 13
      x_onc -= (vmid + 15.0) * dt
      if x_onc < 5.0:
        x_onc = 80.0
      fidx = sweep % 16
      sweep += 1
      objs = {}
      if with_lead_script or True:
        objs[0] = (lead_d(ts), 0.0, lead_v(ts), 1 if ts < 100.0 else 2)
      for s in x_st:
        objs[s] = (x_st[s], y_st[s], -vEgo(ts - 0.15), tid_st[s])
      objs[4] = (x_onc, -7.0, -ve - 15.0, 5)
      n = 0
      for slot in range(16):
        ids = BOSCH_A_MAIN_IDS[slot] + [BOSCH_A_AUX_IDS[slot]]
        if slot in objs:
          x, y, vr, tid = objs[slot]
          rr = int(round((x + 3.0) * 16))
          ar = int(round(np.arctan2(y, x) * 2048)) + 1024
          vraw = int(np.clip(round(vr * 64 + 864), 0, 1728))
          f = [dict(STATUS=1, FRAME_IDX=fidx, RANGE_RAW=rr, AZIMUTH_RAW=ar, RANGE_SIGMA_RAW=2),
               dict(FRAME_IDX=fidx, OBJECT_EXISTENCE_PROBABILITY_RAW=120),
               dict(FRAME_IDX=fidx, LIFECYCLE_RAW=100),
               dict(FRAME_IDX=fidx, TRACK_ID=tid),
               dict(FRAME_IDX=fidx, REL_VELOCITY_RAW=vraw, REL_VELOCITY_UNCERTAINTY_RAW=50, RANGE_RATIO_RAW=500)]
        else:
          f = [dict(STATUS=0xF, FRAME_IDX=fidx, RANGE_RAW=0xFFF, AZIMUTH_RAW=0x7FF),
               dict(FRAME_IDX=fidx), dict(FRAME_IDX=fidx, LIFECYCLE_RAW=0xFFF), dict(FRAME_IDX=fidx, TRACK_ID=0),
               dict(FRAME_IDX=fidx, REL_VELOCITY_RAW=0x7FE)]
        for addr, vals in zip(ids, f):
          C.add(ts + 0.0002 * n, 2, addr, bk.pack(addr, vals))
          n += 1

  # panda batching: each frame is logged at the first 10 ms batch boundary after it arrives
  g = np.arange(0.0, T + 0.02, 0.01)
  grid = np.sort(g + RNG.normal(0, 0.0005, len(g)))
  rows = sorted(C.rows, key=lambda r: r[0])
  tt = np.array([r[0] for r in rows])
  tb = grid[np.clip(np.searchsorted(grid, tt + 0.0005), 0, len(grid) - 1)]
  out = dict(can_t=tb, can_src=np.array([r[1] for r in rows], np.int16), can_addr=np.array([r[2] for r in rows], np.uint16),
             can_is_sendcan=np.array([r[3] for r in rows], np.int8),
             can_dat=np.frombuffer(b"".join(r[4] for r in rows), np.uint8).reshape(-1, 8))
  out["can_bus"] = (out["can_src"] % 128).astype(np.int8)
  cs_s = slice(None)
  out.update(cs_t=cs_t, cs_vEgo=vE, cs_aEgo=aE, cs_brakePressed=np.zeros_like(cs_t), cs_gasPressed=np.zeros_like(cs_t),
             cs_cruiseEnabled=np.ones_like(cs_t), cs_cruiseSpeed=np.full_like(cs_t, 25.0))
  if sparse:
    for k in ("rs_t", "rs_status", "rs_dRel", "rs_vRel", "rs_aLeadK", "rs_yRel", "md_t", "md_prob", "md_x0", "md_v0", "md_a0"):
      out[k] = np.zeros(0)
  else:
    out.update(rs_t=rs_t, rs_status=status, rs_dRel=dRel, rs_vRel=vRel, rs_aLeadK=np.zeros_like(rs_t), rs_yRel=np.zeros_like(rs_t),
               md_t=rs_t, md_prob=np.full_like(rs_t, 0.9), md_x0=dRel + 1.5, md_v0=vEgo(rs_t) + vRel, md_a0=np.zeros_like(rs_t))
  out.update(cc_t=cs_t[::1], cc_accel=np.zeros_like(cs_t), cc_longActive=np.zeros_like(cs_t), ctl_t=cs_t, ctl_enabled=np.ones_like(cs_t),
             lt_t=rs_t, lt_n=np.full_like(rs_t, 5))
  acc_bus = 1 if (not op_long and not sparse) else None
  meta = dict(route=route, op_long=op_long, fingerprint="HONDA_CIVIC_BOSCH" if not sparse else None, acc_bus=acc_bus,
              n_segments=int(T // 60) + 1, duration=float(T))
  out["meta"] = np.array(json.dumps(meta))
  return out


def generate(outdir):
  os.makedirs(outdir, exist_ok=True)
  specs = [("synth_stock_A", 300.0, False, True, True, False), ("synth_alpha_B", 90.0, True, False, False, False),
           ("synth_sparse_C", 60.0, None, False, False, True)]
  for name, T, op, stall, lead, sparse in specs:
    out = build(name, T, op, stall, lead, sparse)
    np.savez_compressed(os.path.join(outdir, name + ".npz"), **out)
    print("wrote", name, len(out["can_t"]), "frames")


CHECKS = [
  ("inv: bank on bus 2 only", r"bank\s+on bus 2: 80 ids"),
  ("inv: radar4xx 50 Hz per-ID period ~20 ms", r"radar4xx\s+b1 rx\s+\|\s+\d+\s+[\d.]+\s+25\s+20\.0"),
  ("inv: sparse route groups absent", r"synth_sparse_C.*\n\s+absent: 0x1DF"),
  ("phase: bank locked to 0x33D", r"\[log\] bank locked to 0x33D"),
  ("phase: independent of radar4xx", r"\[log\] bank independent of radar4xx"),
  ("phase: independent of 0x1DF", r"\[log\] bank independent of 0x1DF"),
  ("phase: bank continued through radar4xx stall", r"independent of radar4xx.*through 1 radar4xx stalls the bank kept 100% .*not driven"),
  ("phase: 0x201 absent", r"0x201: unresolvable or absent"),
  ("decode: 0x410 B2 ~ vEgo lag +0.2", r"0x410 .*\n\s+\[log\] candidate\s+B2\w*\s+~ vEgo\s+rho=\+(0\.9\d|1\.00) lag=\+0\.2s"),
  ("decode: 0x410 counter B7[5:4] mod4", r"0x410 .*counter: B7\[5:4\] mod4"),
  ("decode: 0x410 honda checksum 100%", r"0x410 .*honda_checksum: len\[8\] match 100%"),
  ("decode: 0x410 B5 event-only at 150/250", r"event-only B5: 2 changes.*@150\.\ds 00->01.*@250\.\ds 01->00"),
  ("decode: 0x1DB B3 ~ dRel (or md_x0)", r"0x1DB .*\n\s+\[log\] candidate\s+B3\w*\s+~ (dRel|md_x0)\s+rho=\+(0\.9|1\.0)"),
  ("timing: DBC name", r"DBC used .*honda_civic_hatchback_ex_2017_can_generated"),
  ("timing: 4 episodes on stock route", r"synth_stock_A: 0x1DF@b1 .* episodes=4"),
  ("timing: classes brake=2 engage=1 stop=1", r"episode classes: brake=2, engage=1, stop=1"),
  ("timing: floor episode is a stop", r"peak at the -4\.00 floor: 1 episodes, classes stop=1;"),
  ("timing: peak while moving excludes the hold", r"peak while moving median -2\.00 p10 -2\.\d\d min -2\.50 m/s2, n=3"),
  ("timing: moving brakes 1/3 within 0.5 s of swap", r"moving brakes \(brake\+stop.*within 0\.5 s of a swap 1/3"),
  ("timing: floor frames all stopped", r"-4\.00 floor frames sent while stopped .*median 100%"),
  ("timing: episode1 onset 100.3 dt_sw 0.3", r"\s100\.[23]\s+-2\.00 .*\|\s+0\.[23]\d\s+0\.[23]\d n\s+\|.*HUD_LEAD"),
  ("timing: episode2 onset 199.7 dt_vth ~1.0", r"\s199\.[67]\s+-1\.60 .*\s+(0\.9\d|1\.0\d) n"),
  ("timing: onsets within 0.5 s of swap counted over all 4", r"onsets within 0\.5 s after a lead swap: 1/4"),
  ("timing: AEB_PREPARE at 150", r"0x1DF AEB_PREPARE=1 at 150\.0"),
  ("timing: CHIME at 150.2", r"0x30C CHIME=1 at 150\.[23]"),
  ("timing: alpha route skipped", r"synth_alpha_B: op_long route, skipped"),
  ("objects: low rail ~20%", r"pooled U11 low-rail share \(raw 0 = -13\.5 m/s\) (1[89]|2\d)\.\d%"),
  ("phase: 0x33D period ~34.8 ms", r"vs 0x33D    0x33D@b2 Pref=  34\.[789]ms"),
  ("phase: 0xE4 at 100 Hz flagged unresolvable", r"0xE4rx.*UNRESOLVABLE"),
  ("objects: U11 lag ~150 ms", r"U11 vRel vs -vEgo: best lag median 1[4-6]\d ms"),
  ("objects: dRel slope lag ~0 ms", r"dRel slope vs -vEgo: best lag median (-?1?\d) ms"),
]


def test(outdir):
  env = dict(os.environ)
  r = subprocess.run([sys.executable, os.path.join(HERE, "analyze.py"), outdir], capture_output=True, text=True, env=env)
  out = r.stdout
  print(out)
  if r.returncode:
    print("STDERR:", r.stderr[-3000:])
  nl = out.count("\n")
  fails = []
  for name, pat in CHECKS:
    ok = re.search(pat, out) is not None
    print(("PASS " if ok else "FAIL ") + name)
    if not ok:
      fails.append(name)
  print(f"output lines: {nl}; ERROR lines: {out.count('ERROR')}")
  if "ERROR" in out:
    fails.append("errors present")
  print("ALL PASS" if not fails else f"{len(fails)} FAILED: {fails}")
  return not fails


if __name__ == "__main__":
  outdir = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "synth")
  generate(outdir)
  if "--test" in sys.argv:
    sys.exit(0 if test(outdir) else 1)
