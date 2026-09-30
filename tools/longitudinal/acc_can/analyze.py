#!/usr/bin/env python3
"""Read-only analysis of per-route .npz files written by extract.py (Honda Bosch radar / ACC CAN study).

Usage:  PYTHONPATH=<openpilot repo> python analyze.py DIR [--routes a,b] [--section inv,phase,decode,timing,objects]

Prints text only. Writes nothing. Every printed conclusion is tagged [log] (it comes from logged CAN / state,
not from firmware or road validation). Sections:
  inv     frame inventory per ID group x bus x direction, and per-ID bus membership for radar4xx and the bank
  phase   is the 0x280 object bank phase-locked to radar4xx / 0x1DF / 0x33D / 0xE4(rx) / 0x201, and does it
          keep publishing through their stalls (and vice versa)
  decode  byte profile, counter, Honda checksum, and correlation search for undecoded radar-TX IDs
  timing  stock ACC (op_long false) brake-onset timing vs lead swaps and vRel threshold; AEB/FCW bit assertions
  objects bank decoded with the repo's Bosch-A parser definitions; stationary-target velocity lag test
"""
import argparse
import glob
import json
import os
import sys
import traceback
from collections import defaultdict

import numpy as np

# ----------------------------------------------------------------------------------------------------------
# groups
# ----------------------------------------------------------------------------------------------------------
RADAR4XX = set(range(0x400, 0x417)) | {0x420, 0x440}
BANK = set(range(0x280, 0x300))
CARIN = {0x183, 0x1DC, 0x1DD, 0x1E1}
GROUPS = [
  ("0x1DF", {0x1DF}), ("0x30C", {0x30C}), ("0x39F", {0x39F}), ("0x1EF", {0x1EF}), ("0x1FA", {0x1FA}),
  ("radar4xx", RADAR4XX), ("0x640/641", {0x640, 0x641}), ("0x1DB", {0x1DB}), ("0xE4", {0xE4}), ("0x33D", {0x33D}),
  ("bank", BANK), ("0x201", {0x201}), ("0x24x", set(range(0x240, 0x24B))), ("0x450", {0x450}), ("0x588", {0x588}),
  ("carin", CARIN),
]
DECODE_IDS = sorted(set(range(0x400, 0x417)) | {0x420, 0x440, 0x640, 0x641, 0x1DB, 0x1EF, 0x1FA})
BANK_MARKER = 0x280          # first frame of a sweep (slot 0 f0); one per sweep
KINDS = ("rx", "echo", "tx")  # rx: can src<128; echo: can src>=128 (panda TX echo); tx: sendcan (openpilot TX)

OUT_LINES = [0]


def P(*a):
  s = " ".join(str(x) for x in a)
  OUT_LINES[0] += s.count("\n") + 1
  print(s)


def short(route, n=22):
  return route if len(route) <= n else "~" + route[-(n - 1):]


def fmt(x, nd=2, w=None):
  if x is None or (isinstance(x, float) and not np.isfinite(x)):
    s = "-"
  else:
    s = f"{x:.{nd}f}"
  return s.rjust(w) if w else s


def compress_ids(ids):
  ids = sorted(ids)
  out, i = [], 0
  while i < len(ids):
    j = i
    while j + 1 < len(ids) and ids[j + 1] == ids[j] + 1:
      j += 1
    out.append(f"0x{ids[i]:X}" if i == j else f"0x{ids[i]:X}-0x{ids[j]:X}")
    i = j + 1
  return ",".join(out)


def tb_compact(e):
  tb = traceback.extract_tb(e.__traceback__)
  where = " <- ".join(f"{os.path.basename(f.filename)}:{f.lineno}" for f in tb[-3:][::-1])
  return f"{type(e).__name__}: {e} @ {where}"


# ----------------------------------------------------------------------------------------------------------
# loading / basic helpers
# ----------------------------------------------------------------------------------------------------------
class Route:
  def __init__(self, path):
    d = np.load(path, allow_pickle=False)
    self.a = {k: d[k] for k in d.files}
    try:
      self.meta = json.loads(str(self.a.get("meta", np.array("{}"))))
    except Exception:
      self.meta = {}
    self.name = self.meta.get("route") or os.path.basename(path)[:-4]
    n = len(self.g("can_t"))
    self.t = self.g("can_t").astype(np.float64)
    self.bus = self.g("can_bus").astype(np.int16) if n else np.zeros(0, np.int16)
    src = self.g("can_src").astype(np.int16) if n else np.zeros(0, np.int16)
    self.addr = self.g("can_addr").astype(np.int32) if n else np.zeros(0, np.int32)
    send = self.g("can_is_sendcan").astype(np.int8) if n else np.zeros(0, np.int8)
    self.kind = np.where(send == 1, 2, np.where(src >= 128, 1, 0)).astype(np.int8)
    dat = self.a.get("can_dat")
    self.dat = dat.reshape(-1, 8) if dat is not None and dat.size else np.zeros((0, 8), np.uint8)
    op = self.meta.get("op_long")
    if op is None:  # infer: stock if 0x1DF is only received
      m = self.addr == 0x1DF
      op = bool(np.any(self.kind[m] == 2)) if m.any() else None
      self.op_inferred = True
    else:
      self.op_inferred = False
    self.op_long = op
    self.duration = float(self.meta.get("duration") or (self.t.max() - self.t.min() if n else 0.0))

  def g(self, k):
    v = self.a.get(k)
    return v if v is not None else np.zeros(0)

  def sel(self, ids, bus=None, kind=None):
    m = np.isin(self.addr, list(ids))
    if bus is not None:
      m &= self.bus == bus
    if kind is not None:
      m &= self.kind == KINDS.index(kind)
    return m

  def best_bus(self, ids, kind="rx"):
    m = self.sel(ids, kind=kind)
    if not m.any():
      return None
    b, c = np.unique(self.bus[m], return_counts=True)
    return int(b[np.argmax(c)])

  def frames(self, addr, bus=None, kind="rx"):
    if bus is None:
      bus = self.best_bus({addr}, kind)
      if bus is None:
        return np.zeros(0), np.zeros((0, 8), np.uint8), None
    m = self.sel({addr}, bus, kind)
    t = self.t[m]
    o = np.argsort(t, kind="stable")
    return t[o], self.dat[m][o], bus

  def batch_info(self):
    if len(self.t) < 10:
      return None, None
    u = np.unique(self.t)
    return len(u) / len(self.t), float(np.median(np.diff(u))) * 1e3 if len(u) > 2 else None


def zoh(ts, vs, tq, max_gap=0.5):
  """zero-order hold: value of the most recent sample at or before tq; nan if none within max_gap s."""
  out = np.full(len(tq), np.nan)
  if len(ts) == 0:
    return out
  i = np.searchsorted(ts, tq, side="right") - 1
  ok = (i >= 0)
  ic = np.clip(i, 0, len(ts) - 1)
  ok &= (tq - ts[ic]) <= max_gap
  out[ok] = vs[ic[ok]]
  return out


def interp(ts, vs, tq):
  out = np.full(len(tq), np.nan)
  if len(ts) < 2:
    return out
  ok = (tq >= ts[0]) & (tq <= ts[-1])
  out[ok] = np.interp(tq[ok], ts, vs)
  return out


def period_stats(t):
  if len(t) < 3:
    return None
  """(period, p95 jitter). period = mean inter-arrival excluding gaps > 3x the median: unbiased under
  timestamp batching, where the median of quantised intervals is not."""
  dt = np.diff(np.sort(t))
  md = float(np.median(dt))
  ok = dt < 3 * md if md > 0 else np.ones(len(dt), bool)
  per = float(np.mean(dt[ok])) if ok.any() else md
  jit = float(np.percentile(np.abs(dt - per), 95))
  return per, jit


def rankdata(a):
  a = np.asarray(a, dtype=np.float64)
  sorter = np.argsort(a, kind="mergesort")
  inv = np.empty(len(a), np.intp)
  inv[sorter] = np.arange(len(a))
  s = a[sorter]
  obs = np.r_[True, s[1:] != s[:-1]]
  dense = obs.cumsum()[inv]
  cnt = np.r_[np.nonzero(obs)[0], len(a)]
  return 0.5 * (cnt[dense] + cnt[dense - 1] + 1)


def zscore(x):
  s = x.std()
  return (x - x.mean()) / s if s > 0 else None


# ----------------------------------------------------------------------------------------------------------
# opendbc access (DBC definitions + get_raw_value, checksum)
# ----------------------------------------------------------------------------------------------------------
_DBC_CACHE = {}
USED = defaultdict(set)


def load_dbc(name):
  if name not in _DBC_CACHE:
    from opendbc.can.dbc import DBC
    _DBC_CACHE[name] = DBC(name)
  return _DBC_CACHE[name]


def decode_msg(dbc_name, addr, dat):
  """Decode every signal of `addr` for all rows using opendbc's DBC signal definitions. The bit walk mirrors
  opendbc.can.parser.get_raw_value (vectorised); rows are spot-checked against get_raw_value itself."""
  from opendbc.can.parser import get_raw_value
  msg = load_dbc(dbc_name).addr_to_msg[addr]
  d = dat.astype(np.int64)
  out = {}
  for name, sig in msg.sigs.items():
    ret = np.zeros(len(d), np.int64)
    i, bits = sig.msb // 8, sig.size
    while 0 <= i < 8 and bits > 0:
      lsb = sig.lsb if (sig.lsb // 8) == i else i * 8
      msb = sig.msb if (sig.msb // 8) == i else (i + 1) * 8 - 1
      size = msb - lsb + 1
      part = (d[:, i] >> (lsb - i * 8)) & ((1 << size) - 1)
      ret |= part << (bits - size)
      bits -= size
      i = i - 1 if sig.is_little_endian else i + 1
    for r in range(min(3, len(d))):  # spot check against opendbc itself
      assert int(ret[r]) == get_raw_value(bytes(dat[r]), sig), f"decode mismatch {msg.name}.{name}"
    raw = ret.copy()
    if sig.is_signed:
      raw = np.where(raw >= (1 << (sig.size - 1)), raw - (1 << sig.size), raw)
    out[name] = raw * sig.factor + sig.offset
    out[name + "#raw"] = ret
  USED["dbc"].add(dbc_name)
  return out


def find_pt_dbc(fingerprint):
  """Honda Bosch pt DBC from the fingerprint via opendbc.car.honda.values.DBC; else search the dbc dir."""
  try:
    from opendbc.car import Bus
    from opendbc.car.honda.values import DBC as HDBC
    for k, v in HDBC.items():
      if fingerprint and (str(k) == fingerprint or getattr(k, "name", None) == fingerprint or
                          getattr(k, "value", None) == fingerprint):
        if Bus.pt in v:
          name = v[Bus.pt]
          d = load_dbc(name)
          if 0x1DF in d.addr_to_msg:
            return name, f"values.DBC[{fingerprint}][pt]"
  except Exception:
    pass
  from opendbc.can.dbc import DBC_PATH
  cands = []
  for p in sorted(glob.glob(os.path.join(DBC_PATH, "*.dbc"))):
    try:
      txt = open(p).read()
    except Exception:
      continue
    if "BO_ 479 ACC_CONTROL" in txt and "BO_ 780 ACC_HUD" in txt and ("honda" in p or "acura" in p):
      cands.append(os.path.basename(p)[:-4])
  pref = [c for c in cands if "civic_hatchback" in c] or cands
  if not pref:
    raise RuntimeError("no Honda DBC with ACC_CONTROL/ACC_HUD found")
  return pref[0], "dbc-dir search (fingerprint lookup failed)"


def get_checksum_fn():
  try:
    from opendbc.car.honda.hondacan import honda_checksum
    USED["checksum"].add("opendbc.car.honda.hondacan.honda_checksum")
    return honda_checksum
  except Exception as e:
    USED["checksum"].add(f"local copy (import failed: {type(e).__name__})")

    def honda_checksum(address, sig, d):
      s, addr = 0, address
      while addr:
        s += addr & 0xF
        addr >>= 4
      for i in range(len(d)):
        x = d[i]
        if i == len(d) - 1:
          x >>= 4
        s += (x & 0xF) + (x >> 4)
      s = 8 - s
      return s & 0xF
    return honda_checksum


# ----------------------------------------------------------------------------------------------------------
# section: inv
# ----------------------------------------------------------------------------------------------------------
def inv_route(R, acc):
  fr, bq = R.batch_info()
  groups_present = []
  absent = []
  for gname, ids in GROUPS:
    m = np.isin(R.addr, list(ids))
    if not m.any():
      absent.append(gname)
      continue
    groups_present.append(gname)
    for b in np.unique(R.bus[m]):
      for k in range(3):
        mm = m & (R.bus == b) & (R.kind == k)
        if not mm.any():
          continue
        key = ("alpha" if R.op_long else "stock", gname, int(b), KINDS[k])
        a = acc["inv"].setdefault(key, {"n": 0, "dur": 0.0, "per": [], "jit": [], "ids": set()})
        a["n"] += int(mm.sum())
        a["dur"] += R.duration
        for addr in np.unique(R.addr[mm]):
          ps = period_stats(R.t[mm & (R.addr == addr)])
          a["ids"].add(int(addr))
          if ps:
            a["per"].append(ps[0])
            a["jit"].append(ps[1])
  # per-ID bus membership for radar4xx and bank
  for gname, ids in (("radar4xx", RADAR4XX), ("bank", BANK)):
    m = np.isin(R.addr, list(ids)) & (R.kind == 0)
    sig = defaultdict(list)
    for addr in np.unique(R.addr[m]):
      buses = tuple(sorted(set(int(x) for x in np.unique(R.bus[m & (R.addr == addr)]))))
      sig[buses].append(int(addr))
    acc["membership"].append((R.name, gname, {k: v for k, v in sig.items()}))
  op = "alpha(op_long)" if R.op_long else ("stock" if R.op_long is False else "unknown")
  if R.op_inferred:
    op += "(inferred)"
  acc["inv_meta"].append(
    f"  {short(R.name, 34):34s} {op:16s} fp={R.meta.get('fingerprint')} acc_bus={R.meta.get('acc_bus')} "
    f"dur={R.duration:.0f}s segs={R.meta.get('n_segments')} frames={len(R.t)} "
    f"ts_distinct={fmt(fr, 3)} batch={fmt(bq, 1)}ms")
  if absent:
    acc["inv_meta"].append(f"      absent: {', '.join(absent)}")


def inv_print(acc):
  P("route meta (ts_distinct = distinct CAN timestamps / frames; batch = median spacing of distinct timestamps;"
    " phase resolution is limited to about one batch):")
  for l in acc["inv_meta"]:
    P(l)
  for mode in ("stock", "alpha"):
    keys = sorted(k for k in acc["inv"] if k[0] == mode)
    if not keys:
      continue
    P(f"-- {mode} routes pooled: group bus dir | frames  rate_Hz  nIDs  per-ID period ms (gap-free mean)  per-ID p95 |dt-period| ms")
    for gname, _ in GROUPS:
      rows = [k for k in keys if k[1] == gname]
      if not rows:
        P(f"  {gname:10s} absent")
        continue
      for k in rows:
        a = acc["inv"][k]
        per = np.median(a["per"]) * 1e3 if a["per"] else np.nan
        jit = np.median(a["jit"]) * 1e3 if a["jit"] else np.nan
        rate = a["n"] / a["dur"] if a["dur"] > 0 else np.nan
        P(f"  {gname:10s} b{k[2]} {k[3]:4s} | {a['n']:8d} {rate:8.1f} {len(a['ids']):4d} {per:8.2f} {jit:8.2f}")
  P("-- per-ID bus membership, rx only (same physical wire?):")
  pooled = defaultdict(lambda: defaultdict(set))
  for route, gname, sig in acc["membership"]:
    for buses, ids in sig.items():
      pooled[gname][buses] |= set(ids)
  for gname in ("radar4xx", "bank"):
    if not pooled[gname]:
      P(f"  {gname}: absent")
      continue
    for buses, ids in sorted(pooled[gname].items()):
      P(f"  {gname:8s} on bus {'+'.join(map(str, buses))}: {len(ids)} ids {compress_ids(ids)}")
    all_b = [set(b) for b in pooled[gname]]
    P(f"  [log] {gname}: {'every ID seen on a single bus' if all(len(b) == 1 for b in all_b) else 'some IDs seen on several buses'}"
      f" (bus sets: {sorted(tuple(sorted(b)) for b in all_b)})")
  if pooled["radar4xx"] and pooled["bank"]:
    rb = set().union(*[set(b) for b in pooled["radar4xx"]])
    bb = set().union(*[set(b) for b in pooled["bank"]])
    P(f"  [log] radar4xx buses {sorted(rb)} vs bank buses {sorted(bb)}: "
      f"{'shared bus ' + str(sorted(rb & bb)) if rb & bb else 'no shared bus'}")


# ----------------------------------------------------------------------------------------------------------
# section: phase
# ----------------------------------------------------------------------------------------------------------
def ref_defs(R):
  """reference name -> (ids for marker selection, kind)"""
  return [("radar4xx", RADAR4XX), ("0x1DF", {0x1DF}), ("0x33D", {0x33D}), ("0xE4rx", {0xE4}), ("0x201", {0x201})]


def phase_pair(bank_t, ref_t, grid, rng):
  pb = period_stats(bank_t)
  pr = period_stats(ref_t)
  if pb is None or pr is None:
    return None
  Pb, Pr = pb[0], pr[0]
  i = np.searchsorted(ref_t, bank_t, side="right") - 1
  ok = i >= 0
  off = bank_t[ok] - ref_t[i[ok]]
  ok2 = off < 3 * Pr  # ignore samples inside reference stalls
  off = off[ok2]
  if len(off) < 30:
    return None
  ph = 2 * np.pi * np.mod(off, Pr) / Pr
  z = np.mean(np.exp(1j * ph))
  Rv = float(np.abs(z))
  cmean_ms = float(np.mod(np.angle(z), 2 * np.pi) / (2 * np.pi) * Pr * 1e3)
  # null: independent uniform process at the bank period, random phase, snapped to the same timestamp grid
  r0 = []
  for _ in range(5):
    tt = np.arange(bank_t[0] + rng.uniform(0, Pb), bank_t[-1], Pb * (1 + rng.uniform(-1e-3, 1e-3)))
    j = np.clip(np.searchsorted(grid, tt, side="left"), 0, len(grid) - 1)
    tt = grid[j]
    k = np.searchsorted(ref_t, tt, side="right") - 1
    kk = k >= 0
    o = tt[kk] - ref_t[k[kk]]
    o = o[o < 3 * Pr]
    if len(o) > 10:
      r0.append(float(np.abs(np.mean(np.exp(2j * np.pi * np.mod(o, Pr) / Pr)))))
  R0 = float(np.median(r0)) if r0 else np.nan

  def stall_cover(a_t, Pa, b_t, Pb_):
    gaps = np.diff(a_t)
    gi = np.nonzero(gaps > 5 * Pa)[0]
    if len(gi) == 0:
      return 0, 0.0, np.nan
    exp = present = 0.0
    tot = 0.0
    for g in gi:
      t0, t1 = a_t[g], a_t[g + 1]
      tot += t1 - t0
      exp += (t1 - t0) / Pb_
      present += np.count_nonzero((b_t > t0 + 0.5 * Pb_) & (b_t < t1 - 0.5 * Pb_)) + 1
    return len(gi), tot, min(1.0, present / exp) if exp > 0 else np.nan

  s_ref = stall_cover(ref_t, Pr, bank_t, Pb)
  s_bank = stall_cover(bank_t, Pb, ref_t, Pr)
  return dict(Pb=Pb, Pr=Pr, n=len(off), R=Rv, R0=R0, med=float(np.median(off)) * 1e3, cmean=cmean_ms,
              s_ref=s_ref, s_bank=s_bank)


def phase_route(R, acc):
  rng = np.random.default_rng(0)
  mk = R.sel({BANK_MARKER}, kind="rx")
  if mk.sum() < 50:
    acc["phase_lines"].append(f"  {short(R.name)}: bank marker 0x{BANK_MARKER:X} absent/too few ({int(mk.sum())})")
    return
  grid = np.unique(R.t)
  _, bq = R.batch_info()
  for bb in np.unique(R.bus[mk]):
    bank_t = np.sort(R.t[mk & (R.bus == bb)])
    if len(bank_t) < 50:
      continue
    for rname, ids in ref_defs(R):
      m = R.sel(ids, kind="rx")
      if m.sum() < 50:
        acc["phase_lines"].append(f"  {short(R.name)} bank b{bb} vs {rname:8s}: absent")
        continue
      best = None
      for rb in np.unique(R.bus[m]):
        mm = m & (R.bus == rb)
        addrs, cnt = np.unique(R.addr[mm], return_counts=True)
        raddr = int(addrs[np.argmax(cnt)])
        ref_t = np.sort(R.t[mm & (R.addr == raddr)])
        if len(ref_t) < 50:
          continue
        res = phase_pair(bank_t, ref_t, grid, rng)
        if res is None:
          continue
        res.update(ref=rname, raddr=raddr, bb=int(bb), rb=int(rb), route=R.name)
        unres = bq is not None and res["Pr"] * 1e3 < 1.5 * bq
        res["unres"] = unres
        n_s, tot, frac = res["s_ref"]
        n_b, totb, fracb = res["s_bank"]
        acc["phase_lines"].append(
          f"  {short(R.name)} bank b{bb} vs {rname:8s} 0x{raddr:X}@b{rb} Pref={res['Pr']*1e3:6.1f}ms n={res['n']:6d}"
          f" R={res['R']:.2f} R0={res['R0']:.2f} med_off={res['med']:6.1f}ms cmean={res['cmean']:5.1f}ms"
          f" | ref stalls {n_s} ({tot:.1f}s) bank present {fmt(frac)} | bank stalls {n_b} ({totb:.1f}s) ref present {fmt(fracb)}"
          + (" UNRESOLVABLE(ref period ~ ts batch)" if unres else ""))
        acc["phase"][rname].append(res)
        if best is None or res["R"] > best["R"]:
          best = res


def phase_verdict(rname, rs):
  good = [r for r in rs if not r["unres"]]
  if not good:
    return f"[log] {rname}: unresolvable or absent"
  Rm = float(np.median([r["R"] for r in good]))
  R0m = float(np.median([r["R0"] for r in good]))
  best = max(good, key=lambda r: r["R"])
  tag = (f"bank locked to {rname}" if Rm >= 0.6 and Rm - R0m >= 0.3 else
         f"bank independent of {rname}" if Rm < 0.3 or Rm - R0m < 0.1 else f"bank weakly/ambiguously tied to {rname}")
  s = f"[log] {tag} (median R={Rm:.2f} over {len(good)} route-bus pairs, null R0={R0m:.2f}; best 0x{best['raddr']:X}@b{best['rb']} R={best['R']:.2f})"
  n_st = sum(r["s_ref"][0] for r in good)
  if n_st:
    fr = np.nanmean([r["s_ref"][2] for r in good if r["s_ref"][0]])
    s += f"; through {n_st} {rname} stalls the bank kept {fr:.0%} of its frames"
    s += " -> bank not driven by it" if fr > 0.5 else " -> bank stalls with it"
  n_bs = sum(r["s_bank"][0] for r in good)
  if n_bs:
    fr = np.nanmean([r["s_bank"][2] for r in good if r["s_bank"][0]])
    s += f"; through {n_bs} bank stalls {rname} kept {fr:.0%}"
  return s


def phase_print(acc):
  P(f"bank timing marker = 0x{BANK_MARKER:X} (one frame per sweep). R = circular concentration of (t_bank - t_prev_ref) mod Pref;"
    " R0 = same statistic for a random-phase process at the bank period snapped to this route's timestamp grid (null).")
  for l in acc["phase_lines"]:
    P(l)
  for rname, _ in ref_defs(None):
    P("  " + phase_verdict(rname, acc["phase"].get(rname, [])))


# ----------------------------------------------------------------------------------------------------------
# section: decode
# ----------------------------------------------------------------------------------------------------------
LAGS = np.round(np.arange(-1.0, 1.0001, 0.1), 2)
TARGETS = ["vEgo", "aEgo", "dRel", "vRel", "cruiseSpeed", "accelCmd", "brakePressed", "md_x0"]


def target_series(R, pt_dbc_name):
  """name -> (t, v) with nan for invalid."""
  cs_t = R.g("cs_t")
  out = {}
  for n in ("vEgo", "aEgo", "cruiseSpeed", "brakePressed"):
    out[n] = (cs_t, R.g("cs_" + n))
  rs_t, st = R.g("rs_t"), R.g("rs_status")
  for n in ("dRel", "vRel"):
    v = R.g("rs_" + n).astype(float).copy()
    if len(v):
      v[st < 0.5] = np.nan
    out[n] = (rs_t, v)
  md_t = R.g("md_t")
  x0 = R.g("md_x0").astype(float).copy()
  if len(x0):
    x0[R.g("md_prob") < 0.5] = np.nan
  out["md_x0"] = (md_t, x0)
  out["accelCmd"] = (np.zeros(0), np.zeros(0))
  try:
    accb = R.meta.get("acc_bus")
    t, d, b = R.frames(0x1DF, bus=accb, kind="rx") if accb is not None else R.frames(0x1DF, kind="rx")
    if len(t) == 0:
      t, d, b = R.frames(0x1DF, kind="tx")
    if len(t) and pt_dbc_name:
      out["accelCmd"] = (t, decode_msg(pt_dbc_name, 0x1DF, d)["ACCEL_COMMAND"])
  except Exception:
    pass
  return out


def field_matrix(dat):
  d = dat.astype(np.int64)
  names, cols, bset = [], [], []
  for i in range(8):
    names += [f"B{i}u8", f"B{i}s8"]
    cols += [d[:, i], np.where(d[:, i] >= 128, d[:, i] - 256, d[:, i])]
    bset += [{i}, {i}]
  for i in range(7):
    u = d[:, i] * 256 + d[:, i + 1]
    names += [f"B{i}{i+1}u16be", f"B{i}{i+1}s16be"]
    cols += [u, np.where(u >= 32768, u - 65536, u)]
    bset += [{i, i + 1}, {i, i + 1}]
  for i in range(8):
    names += [f"B{i}hi", f"B{i}lo"]
    cols += [d[:, i] >> 4, d[:, i] & 15]
    bset += [{i}, {i}]
  return names, np.array(cols, dtype=np.float64), bset


def counter_scan(dat):
  """2- and 4-bit fields (any bit offset of the big-endian 64-bit word) that step +1 mod 2^w on >=90% of frames."""
  if len(dat) < 20:
    return []
  w64 = np.zeros(len(dat), np.uint64)
  for i in range(8):
    w64 = (w64 << np.uint64(8)) | dat[:, i].astype(np.uint64)
  res = []
  for w in (4, 2):
    for sh in range(0, 64 - w + 1):
      v = ((w64 >> np.uint64(sh)) & np.uint64((1 << w) - 1)).astype(np.int64)
      f = float(np.mean(np.mod(np.diff(v), 1 << w) == 1))
      if f >= 0.9:
        byte = 7 - sh // 8
        res.append((w, byte, sh % 8 + w - 1, sh % 8, f))
  # drop 2-bit hits contained in a 4-bit hit's low bits
  out = []
  for r in res:
    if r[0] == 2 and any(q[0] == 4 and q[1] == r[1] and q[3] == r[3] for q in res):
      continue
    out.append(r)
  return out


def checksum_scan(addr, dat, fn):
  best = None
  for L in range(8, 0, -1):
    if L < 8 and np.any(dat[:, L:]):
      continue
    sub = dat[:: max(1, len(dat) // 2000)]
    ok = np.mean([fn(addr, None, bytearray(bytes(r[:L]))) == (r[L - 1] & 0xF) for r in sub])
    if best is None or ok > best[1]:
      best = (L, float(ok))
  return best


def decode_route(R, acc, pt_dbc_name):
  tg = None
  for addr in DECODE_IDS:
    t, d, b = R.frames(addr, kind="rx")
    if len(t) < 20:
      continue
    if tg is None:
      tg = target_series(R, pt_dbc_name)
    A = acc["decode"].setdefault(addr, dict(n=0, routes=[], prof=[], cnt=[], chk=[], ev=[], rho=[], names=None, bset=None))
    A["n"] += len(t)
    A["routes"].append(f"{short(R.name, 12)}:b{b}")
    # event-only bytes (full data)
    for i in range(8):
      ch = np.nonzero(np.diff(d[:, i].astype(int)) != 0)[0]
      A["ev"].append((i, len(ch), len(d), [(R.name, float(t[j + 1]), int(d[j, i]), int(d[j + 1, i])) for j in ch[:6]]))
    A["prof"].append(d[:: max(1, len(d) // 3000)])
    A["cnt"].append(counter_scan(d[:5000]))
    A["chk"].append(checksum_scan(addr, d, acc["chkfn"]))
    # correlation data (subsample)
    idx = np.arange(0, len(t), max(1, len(t) // 3000))
    ts = t[idx]
    names, F, bset = field_matrix(d[idx])
    A["names"], A["bset"] = names, bset
    rho = np.full((F.shape[0], len(TARGETS), len(LAGS)), np.nan)
    nn = np.zeros((F.shape[0], len(TARGETS)))
    for si, s in enumerate(TARGETS):
      st, sv = tg[s]
      M = np.array([zoh(st, sv, ts - lag, 0.5) for lag in LAGS])  # lag>0: field at t matches target at t-lag
      valid = np.all(np.isfinite(M), axis=0)
      if valid.sum() < 50:
        continue
      Mz = [zscore(rankdata(M[li, valid])) for li in range(len(LAGS))]
      if any(z is None for z in Mz):
        continue  # target constant on this route
      Mz = np.array(Mz)
      for fi in range(F.shape[0]):
        fv = F[fi, valid]
        if np.mean(np.diff(fv) != 0) < 0.01:
          continue  # stationary or event-only field
        fz = zscore(rankdata(fv))
        if fz is None:
          continue
        rho[fi, si] = Mz @ fz / len(fz)
        nn[fi, si] = valid.sum()
    A["rho"].append((rho, nn))


def decode_print(acc):
  P("lag convention: lag>0 means the field at time t matches the target at t-lag (field FOLLOWS target)."
    " rho is Spearman on up to 3000 frames/route; autocorrelated series inflate |rho| — treat as candidates.")
  if not acc["decode"]:
    P("  [log] no undecoded radar-TX IDs present")
  for addr in DECODE_IDS:
    A = acc["decode"].get(addr)
    if A is None:
      continue
    prof = np.concatenate(A["prof"])
    cells = []
    for i in range(8):
      u = np.unique(prof[:, i])
      cells.append(f"{u[0]:02X}" if len(u) == 1 else f"#{len(u)}")
    cnts = [c for cl in A["cnt"] for c in cl]
    cstr = "none"
    if cnts:
      k = defaultdict(list)
      for w, byte, hi, lo, f in cnts:
        k[(w, byte, hi, lo)].append(f)
      cstr = ", ".join(f"B{b}[{h}:{l}] mod{1 << w} {np.mean(v):.0%}" for (w, b, h, l), v in sorted(k.items())[:3])
    chk = [c for c in A["chk"] if c]
    chstr = "-"
    if chk:
      Ls = sorted(set(c[0] for c in chk))
      chstr = f"len{Ls} match {np.mean([c[1] for c in chk]):.0%}"
    P(f"0x{addr:03X} n={A['n']} [{' '.join(A['routes'][:3])}{' ...' if len(A['routes']) > 3 else ''}] bytes: {' '.join(cells)}"
      f" | counter: {cstr} | honda_checksum: {chstr}")
    # correlations: per-route Spearman, pooled as an n-weighted mean (avoids between-route confounding)
    num = sum(np.nan_to_num(r) * n[:, :, None] for r, n in A["rho"])
    den = sum(np.where(np.isfinite(r), 1.0, 0.0) * n[:, :, None] for r, n in A["rho"])
    with np.errstate(invalid="ignore", divide="ignore"):
      pooled = np.where(den > 0, num / np.maximum(den, 1e-9), np.nan)
    ntot = sum(n for _, n in A["rho"])
    best = []  # (|rho|, -width, rho, field idx, target, lag, n)
    for fi in range(pooled.shape[0]):
      for si, s in enumerate(TARGETS):
        row = pooled[fi, si]
        if not np.isfinite(row).any():
          continue
        li = int(np.nanargmax(np.abs(row)))
        best.append((round(abs(row[li]), 3), -len(A["bset"][fi]), float(row[li]), fi, s, float(LAGS[li]), int(ntot[fi, si])))
    best.sort(reverse=True)
    used_bytes, shown = set(), 0
    for ab, _w, rho, fi, s, lag, n in best:
      if A["bset"][fi] & used_bytes:
        continue
      used_bytes |= A["bset"][fi]
      tag = "[log] candidate" if ab >= 0.5 else "weak"
      P(f"    {tag:15s} {A['names'][fi]:10s} ~ {s:12s} rho={rho:+.2f} lag={lag:+.1f}s n={n}")
      shown += 1
      if shown == 3:
        break
    if not best:
      P("    no varying field / no target overlap")
    # event-only bytes
    evs = defaultdict(lambda: [0, 0, []])
    for i, nch, n, lst in A["ev"]:
      evs[i][0] += nch
      evs[i][1] += n
      evs[i][2] += lst
    for i, (nch, n, lst) in sorted(evs.items()):
      if 0 < nch < 0.01 * n:
        times = ", ".join(f"{short(r, 8)}@{tt:.1f}s {a:02X}->{b_:02X}" for r, tt, a, b_ in lst[:5])
        P(f"    [log] event-only B{i}: {nch} changes/{n} frames: {times}{' ...' if nch > 5 else ''}")


# ----------------------------------------------------------------------------------------------------------
# section: timing (stock routes)
# ----------------------------------------------------------------------------------------------------------
ALERT_KEYS = ("AEB", "FCW", "FCM", "CMBS", "ALERT", "CHIME", "APPLY_BRAKES")


def lead_swaps(rs_t, st, d):
  sw = []
  for i in range(1, len(rs_t)):
    on0, on1 = st[i - 1] > 0.5, st[i] > 0.5
    if on0 != on1:
      sw.append(rs_t[i])
    elif on1 and abs(d[i] - d[i - 1]) > 4.0 and rs_t[i] - rs_t[i - 1] <= 0.15:
      sw.append(rs_t[i])
  return np.array(sw)


def runs(mask):
  """[(i0, i1_inclusive)] of True runs."""
  if not len(mask):
    return []
  m = np.r_[False, mask, False].astype(int)
  dm = np.diff(m)
  return list(zip(np.nonzero(dm == 1)[0], np.nonzero(dm == -1)[0] - 1))


def timing_route(R, acc):
  if R.op_long:
    acc["timing_lines"].append(f"  {short(R.name)}: op_long route, skipped")
    return
  name, how = find_pt_dbc(R.meta.get("fingerprint"))
  acc["timing_dbc"].add(f"{name} ({how})")
  accb = R.meta.get("acc_bus")
  t, d, b = R.frames(0x1DF, bus=accb, kind="rx") if accb is not None else R.frames(0x1DF, kind="rx")
  if len(t) < 20:
    acc["timing_lines"].append(f"  {short(R.name)}: 0x1DF rx absent on acc_bus={accb}")
    return
  AC = decode_msg(name, 0x1DF, d)
  cmd = AC["ACCEL_COMMAND"]
  th, dh, bh = R.frames(0x30C, kind="rx")
  HUD = decode_msg(name, 0x30C, dh) if len(th) else {}
  tr, dr, br = R.frames(0x39F, kind="rx")
  RH = decode_msg(name, 0x39F, dr) if len(tr) and 0x39F in load_dbc(name).addr_to_msg else {}
  rs_t, st, dR, vR = R.g("rs_t"), R.g("rs_status"), R.g("rs_dRel"), R.g("rs_vRel")
  cs_t, vE = R.g("cs_t"), R.g("cs_vEgo")
  sw = lead_swaps(rs_t, st, dR)

  def lead_at(tq):
    dd = zoh(rs_t, dR, np.array([tq]))[0]
    vv = zoh(rs_t, vR, np.array([tq]))[0]
    s_ = zoh(rs_t, st, np.array([tq]))[0]
    if not (s_ > 0.5):
      return np.nan, np.nan, np.nan
    ttc = dd / -vv if vv < -0.1 else np.inf
    return dd, vv, ttc

  # episodes
  # One episode per brake: a run of cmd < -0.3 (brief dropouts merged) that holds cmd < -1.0 for >= 0.5 s somewhere.
  # Episodes used to be the hard runs themselves, each walked back to its onset, so two hard runs inside one brake
  # were counted twice with the same onset.
  soft = []
  for i0, i1 in runs(cmd < -0.3):
    if soft and t[i0] - t[soft[-1][1]] < 0.15:
      soft[-1] = (soft[-1][0], i1)
    else:
      soft.append((i0, i1))
  eps = []
  for s0, s1 in soft:
    hr = []
    for h0, h1 in runs(cmd[s0:s1 + 1] < -1.0):
      if hr and t[s0 + h0] - t[s0 + hr[-1][1]] < 0.15:
        hr[-1] = (hr[-1][0], h1)
      else:
        hr.append((h0, h1))
    if any(t[s0 + h1] - t[s0 + h0] >= 0.5 for h0, h1 in hr):
      eps.append((s0, s1))
  acc["timing_lines"].append(f"  {short(R.name)}: 0x1DF@b{b} n={len(t)} 0x30C@b{bh} 0x39F@b{br} swaps={len(sw)} episodes={len(eps)}")
  hud_keys = [k for k in HUD if not k.endswith("#raw") and k not in ("COUNTER", "CHECKSUM")]
  for i0, i1 in eps:
    ton = t[i0]
    peak = float(cmd[i0:i1 + 1].min())
    L0, L1 = lead_at(ton), lead_at(ton - 1.0)
    prev = sw[sw <= ton + 1e-6]
    dsw = ton - prev[-1] if len(prev) else np.nan
    # vRel threshold run
    k = np.searchsorted(rs_t, ton, side="right") - 1
    dvt = np.nan
    if k >= 0 and st[k] > 0.5 and vR[k] < -1.0:
      k0 = k
      while k0 > 0 and st[k0 - 1] > 0.5 and vR[k0 - 1] < -1.0:
        k0 -= 1
      dvt = ton - rs_t[k0]
    ve = zoh(cs_t, vE, np.array([ton]))[0]
    stopped = bool(np.isfinite(L0[1]) and ve + L0[1] < 1.0)
    chg = []
    if len(th):
      w = (th > ton - 1) & (th < ton + 1)
      for kk in hud_keys:
        v = HUD[kk][w]
        if len(v) > 1 and np.any(v[1:] != v[:-1]):
          chg.append(kk)
    # classify: 'hold' = standstill hold (car stopped / STANDSTILL set), 'engage' = ACC just engaged, else 'brake'
    seg = slice(i0, i1 + 1)
    ss = AC.get("STANDSTILL", np.zeros(len(cmd)))[seg]
    ss_share = float(np.mean(ss > 0)) if len(ss) else 0.0
    v_min = float(np.nanmin(zoh(cs_t, vE, t[seg]))) if len(cs_t) else np.nan
    floor_share = float(np.mean(np.abs(cmd[seg] - peak) < 0.005)) if peak <= -3.99 else 0.0
    co = AC.get("CONTROL_ON", np.zeros(len(cmd)))
    w2 = (t >= ton - 2.0) & (t <= ton)
    co_w = co[w2]
    engaged_change = bool(len(co_w) > 1 and np.any(co_w[1:] != co_w[:-1]))
    ce_t, ce = R.g("cs_t"), R.g("cs_cruiseEnabled")
    if len(ce_t):
      cw = ce[(ce_t >= ton - 2.0) & (ce_t <= ton)]
      engaged_change |= bool(len(cw) > 1 and np.any(cw[1:] != cw[:-1]))
    engaged_change |= "ACC_ON" in chg
    cls = "hold" if (ss_share > 0.5 or ve < 0.5) else ("engage" if engaged_change else "brake")
    rec = dict(route=R.name, ton=ton, peak=peak, L0=L0, L1=L1, dsw=dsw, dvt=dvt, stopped=stopped, chg=chg,
               cls=cls, ve=float(ve), v_min=v_min, ss_share=ss_share, floor_share=floor_share, dur=float(t[i1] - t[i0]))
    acc["timing_eps"].append(rec)
  # alert-bit assertions
  for src, D, tt in (("0x1DF", AC, t), ("0x30C", HUD, th), ("0x39F", RH, tr)):
    for kk in D:
      if kk.endswith("#raw") or not any(a in kk for a in ALERT_KEYS):
        continue
      v = D[kk]
      rises = np.nonzero((v[1:] != 0) & (v[:-1] == 0))[0] + 1
      if len(v) and v[0] != 0:
        rises = np.r_[0, rises]
      for ri in rises:
        L = lead_at(tt[ri])
        acc["timing_alerts"].append((R.name, src, kk, float(tt[ri]), float(v[ri]), L))


def timing_print(acc):
  P("DBC used for 0x1DF/0x30C/0x39F: " + ("; ".join(sorted(acc["timing_dbc"])) or "none"))
  P("episode = one run of ACCEL_COMMAND < -0.3 (dropouts <0.15 s merged) holding < -1.0 for >=0.5 s; onset = its first frame. lead swap = radarState"
    " dRel jump >4 m within 0.15 s or status change. dt_sw = onset - last swap; dt_vth = onset - start of vRel<-1 run.")
  for l in acc["timing_lines"]:
    P(l)
  E = acc["timing_eps"]
  if E:
    P(f"  {'route':10s} {'t_on':>7s} {'peak':>5s} | {'d@on':>5s} {'v@on':>5s} {'ttc':>5s} | {'d@-1':>5s} {'v@-1':>5s} {'ttc':>5s}"
      f" | {'dt_sw':>5s} {'dt_vth':>6s} stop | {'class':6s} {'vEgo':>5s} {'vmin':>5s} {'ss%':>4s} {'dur':>5s} | 0x30C changed +-1s")
    for e in E[:400]:
      P(f"  {short(e['route'], 10):10s} {e['ton']:7.1f} {e['peak']:5.2f} | {fmt(e['L0'][0],1,5)} {fmt(e['L0'][1],1,5)} {fmt(e['L0'][2],1,5)}"
        f" | {fmt(e['L1'][0],1,5)} {fmt(e['L1'][1],1,5)} {fmt(e['L1'][2],1,5)} | {fmt(e['dsw'],2,5)} {fmt(e['dvt'],2,6)} "
        f"{'Y' if e['stopped'] else 'n'}    | {e['cls']:6s} {fmt(e['ve'],1,5)} {fmt(e['v_min'],1,5)} {e['ss_share']*100:4.0f} {e['dur']:5.1f}"
        f" | {','.join(e['chg'][:5]) or '-'}")
    if len(E) > 400:
      P(f"  ... {len(E) - 400} more episodes not shown")

    def q(x):
      x = np.array([v for v in x if np.isfinite(v)])
      if not len(x):
        return "n=0"
      return f"n={len(x)} median={np.median(x):.2f}s p25={np.percentile(x, 25):.2f} p75={np.percentile(x, 75):.2f}"
    dsw = [e["dsw"] for e in E]
    P(f"  [log] pooled onset - last swap: {q(dsw)}")
    P(f"  [log] pooled onset - vRel<-1 start: {q([e['dvt'] for e in E])}")
    fin = [v for v in dsw if np.isfinite(v)]
    within = sum(0 <= v <= 0.5 for v in fin)
    P(f"  [log] onsets within 0.5 s after a lead swap: {within}/{len(E)} ({within / len(E):.0%});"
      f" stopped-lead episodes {sum(e['stopped'] for e in E)}/{len(E)}")
    from collections import Counter
    cc = Counter(e["cls"] for e in E)
    P(f"  [log] episode classes: " + ", ".join(f"{k}={v}" for k, v in sorted(cc.items())))
    fl = [e for e in E if e["peak"] <= -3.99]
    if fl:
      fc = Counter(e["cls"] for e in fl)
      P(f"  [log] peak at the -4.00 floor: {len(fl)} episodes, classes " + ", ".join(f"{k}={v}" for k, v in sorted(fc.items()))
        + f"; median vEgo at onset {np.nanmedian([e['ve'] for e in fl]):.1f} m/s, median min vEgo {np.nanmedian([e['v_min'] for e in fl]):.1f},"
        f" median STANDSTILL share {np.median([e['ss_share'] for e in fl]):.0%}, median share of frames at the floor {np.median([e['floor_share'] for e in fl]):.0%}")
    B = [e for e in E if e["cls"] == "brake"]
    P(f"  [log] brake-only (hold and engage removed): onset - last swap {q([e['dsw'] for e in B])}")
    P(f"  [log] brake-only: onset - vRel<-1 start {q([e['dvt'] for e in B])}")
    fb = [e["dsw"] for e in B if np.isfinite(e["dsw"])]
    wb = sum(0 <= v <= 0.5 for v in fb)
    if B:
      P(f"  [log] brake-only: onsets within 0.5 s after a lead swap {wb}/{len(B)} ({wb / len(B):.0%}); stopped-lead {sum(e['stopped'] for e in B)}/{len(B)}")
  else:
    P("  [log] no stock brake episodes found")
  A = acc["timing_alerts"]
  P(f"  alert-type bit assertions (rising edges of *AEB*/*FCW*/*FCM*/*CMBS*/*ALERT*/*CHIME*/*APPLY_BRAKES*): {len(A)}")
  for r, src, k, tt, v, L in A[:25]:
    P(f"    [log] {short(r, 10)} {src} {k}={v:g} at {tt:.2f}s lead d={fmt(L[0],1)} vRel={fmt(L[1],1)} ttc={fmt(L[2],1)}")
  if len(A) > 25:
    P(f"    ... {len(A) - 25} more")


# ----------------------------------------------------------------------------------------------------------
# section: objects
# ----------------------------------------------------------------------------------------------------------
def objects_imports():
  from opendbc.car.honda import radar_interface as ri
  need = ["BOSCH_A_DBC_NAME", "BOSCH_A_NUM_SLOTS", "BOSCH_A_MAIN_IDS", "BOSCH_A_AUX_IDS", "BOSCH_A_RANGE_SCALE_M",
          "BOSCH_A_RANGE_OFFSET_M", "BOSCH_A_AZIMUTH_SCALE_RAD", "BOSCH_A_AZIMUTH_CENTER", "BOSCH_A_STATUS_INVALID",
          "BOSCH_A_RANGE_RAW_INVALID", "BOSCH_A_ANGLE_RAW_INVALID", "BOSCH_A_TRACK_ID_MIN", "BOSCH_A_TRACK_ID_MAX",
          "BOSCH_A_DIRECT_VREL_INVALID", "BOSCH_A_DIRECT_VREL_MIN_RAW", "BOSCH_A_DIRECT_VREL_MAX_RAW",
          "BOSCH_A_USE_TAN_LATERAL_PROJECTION", "_bosch_a_direct_vrel"]
  miss = [n for n in need if not hasattr(ri, n)]
  if miss:
    raise ImportError(f"radar_interface lacks {miss}")
  return ri, need


def decode_bank(R, ri):
  """rows: t, slot, track_id, dRel, yRel, vrel(m/s or nan), vrel_raw(-1 if none) -- valid objects only."""
  dbc = ri.BOSCH_A_DBC_NAME
  bus = R.best_bus({BANK_MARKER}, "rx")
  rows = []
  for slot in range(ri.BOSCH_A_NUM_SLOTS):
    f0, f1, f2, f3 = ri.BOSCH_A_MAIN_IDS[slot]
    aux = ri.BOSCH_A_AUX_IDS[slot]
    t0, d0, _ = R.frames(f0, bus=bus)
    t3, d3, _ = R.frames(f3, bus=bus)
    if len(t0) == 0 or len(t3) == 0:
      continue
    s0 = decode_msg(dbc, f0, d0)
    s3 = decode_msg(dbc, f3, d3)
    ta, da, _ = R.frames(aux, bus=bus)
    sa = decode_msg(dbc, aux, da) if len(ta) else None

    def match(tq, idxq, tt, idx):
      j = np.clip(np.searchsorted(tt, tq), 1, len(tt) - 1) if len(tt) > 1 else np.zeros(len(tq), int)
      if len(tt) > 1:
        j = np.where(np.abs(tt[j - 1] - tq) < np.abs(tt[j] - tq), j - 1, j)
      ok = (np.abs(tt[j] - tq) < 0.04) & (idx[j] == idxq)
      return j, ok
    fi0 = s0["FRAME_IDX#raw"]
    j3, ok3 = match(t0, fi0, t3, s3["FRAME_IDX#raw"])
    status, rraw, araw = s0["STATUS#raw"], s0["RANGE_RAW#raw"], s0["AZIMUTH_RAW#raw"]
    tid = np.where(ok3, s3["TRACK_ID#raw"][j3], -1)
    valid = ok3 & (status != ri.BOSCH_A_STATUS_INVALID) & (rraw != ri.BOSCH_A_RANGE_RAW_INVALID) & \
        (araw != ri.BOSCH_A_ANGLE_RAW_INVALID) & (tid >= ri.BOSCH_A_TRACK_ID_MIN) & (tid <= ri.BOSCH_A_TRACK_ID_MAX)
    dRel = ri.BOSCH_A_RANGE_SCALE_M * rraw + ri.BOSCH_A_RANGE_OFFSET_M
    az = ri.BOSCH_A_AZIMUTH_SCALE_RAD * (araw - ri.BOSCH_A_AZIMUTH_CENTER)
    yRel = dRel * (np.tan(az) if ri.BOSCH_A_USE_TAN_LATERAL_PROJECTION else np.sin(az))
    vraw = np.full(len(t0), -1)
    vunc = np.full(len(t0), -1)
    if sa is not None:
      ja, oka = match(t0, fi0, ta, sa["FRAME_IDX#raw"])
      vraw = np.where(oka, sa["REL_VELOCITY_RAW#raw"][ja], -1)
      vunc = np.where(oka, sa["REL_VELOCITY_UNCERTAINTY_RAW#raw"][ja], -1)
    vrel = np.array([np.nan if r < 0 else (lambda x: np.nan if x is None else x)(ri._bosch_a_direct_vrel(int(r), int(u)))
                     for r, u in zip(vraw, vunc)])
    for k in np.nonzero(valid)[0]:
      rows.append((t0[k], slot, tid[k], dRel[k], yRel[k], vrel[k], vraw[k]))
  if not rows:
    return np.zeros((0, 7)), bus
  A = np.array(rows, dtype=np.float64)
  return A[np.argsort(A[:, 0], kind="stable")], bus


def best_lag(ts, y, cs_t, target, lags):
  """lag (s) minimising RMS of y(t) - target(t - lag); returns (lag, at_boundary)."""
  err = []
  for L in lags:
    x = interp(cs_t, target, ts - L)
    ok = np.isfinite(x) & np.isfinite(y)
    err.append(np.sqrt(np.mean((y[ok] - x[ok]) ** 2)) if ok.sum() >= 8 else np.inf)
  err = np.array(err)
  if not np.isfinite(err).any():
    return None
  i = int(np.argmin(err))
  return float(lags[i]), i in (0, len(lags) - 1)


def objects_route(R, acc, ri):
  A, bus = decode_bank(R, ri)
  if len(A) == 0:
    acc["obj_lines"].append(f"  {short(R.name)}: no valid bank objects (bank absent or all slots invalid)")
    return
  cs_t, vE, aE = R.g("cs_t"), R.g("cs_vEgo"), R.g("cs_aEgo")
  vr = A[:, 6]
  live = vr >= 0
  live &= vr != ri.BOSCH_A_DIRECT_VREL_INVALID
  low = int(np.sum(vr[live] == ri.BOSCH_A_DIRECT_VREL_MIN_RAW))
  high = int(np.sum(vr[live] == ri.BOSCH_A_DIRECT_VREL_MAX_RAW))
  acc["obj_rail"][0] += low
  acc["obj_rail"][1] += high
  acc["obj_rail"][2] += int(live.sum())
  ntr = len(np.unique(A[:, 2]))
  acc["obj_lines"].append(f"  {short(R.name)}: bank@b{bus} valid object rows={len(A)} track ids={ntr} "
                          f"U11 live rows={int(live.sum())} low-rail(raw {ri.BOSCH_A_DIRECT_VREL_MIN_RAW})={low / max(1, live.sum()):.1%} "
                          f"high-rail(raw {ri.BOSCH_A_DIRECT_VREL_MAX_RAW})={high / max(1, live.sum()):.1%}")
  if len(cs_t) < 10:
    return
  # transient indicator on the object timeline: any |aEgo|>1 within +-1 s
  tr_t = cs_t[np.abs(aE) > 1.0]
  lags = np.round(np.arange(-0.1, 0.6001, 0.01), 3)  # -100 ms margin so a true 0 lag is interior
  nv = nd = 0
  for tid in np.unique(A[:, 2]):
    T = A[A[:, 2] == tid]
    # contiguous runs of this track id
    brk = np.nonzero(np.diff(T[:, 0]) >= 0.3)[0]
    starts = np.r_[0, brk + 1]
    ends = np.r_[brk, len(T) - 1]
    for s, e in zip(starts, ends):
      S = T[s:e + 1]
      if len(S) < 10 or len(tr_t) == 0:
        continue
      k = np.searchsorted(tr_t, S[:, 0])
      near = np.zeros(len(S), bool)
      for kk in (k - 1, k):
        kc = np.clip(kk, 0, len(tr_t) - 1)
        near |= np.abs(tr_t[kc] - S[:, 0]) <= 1.0
      for a, b in runs(near):
        W = S[a:b + 1]
        if len(W) < 10 or W[-1, 0] - W[0, 0] < 0.7:
          continue
        ve = interp(cs_t, vE, W[:, 0])
        if not np.all(np.isfinite(ve)) or np.ptp(ve) < 1.0:
          continue
        gs = W[:, 5] + ve  # ground speed from U11
        if not (np.all(np.abs(W[:, 4]) < 10) and np.all((W[:, 3] > 5) & (W[:, 3] < 80))):
          continue
        okv = np.isfinite(W[:, 5]) & (W[:, 6] != ri.BOSCH_A_DIRECT_VREL_MIN_RAW) & (W[:, 6] != ri.BOSCH_A_DIRECT_VREL_MAX_RAW)
        if okv.sum() >= 10 and np.nanmedian(np.abs(gs[okv])) < 1.5:
          r = best_lag(W[okv, 0], W[okv, 5], cs_t, -vE, lags)
          if r:
            acc["obj_vlag"].append(r)
            nv += 1
        # dRel slope (centered +-2 samples) — stationarity judged from the slope itself
        if len(W) >= 10:
          sl = np.full(len(W), np.nan)
          sl[2:-2] = (W[4:, 3] - W[:-4, 3]) / (W[4:, 0] - W[:-4, 0])
          okd = np.isfinite(sl)
          if okd.sum() >= 8 and np.median(np.abs(sl[okd] + ve[okd])) < 1.5:
            r = best_lag(W[okd, 0], sl[okd], cs_t, -vE, lags)
            if r:
              acc["obj_dlag"].append(r)
              nd += 1
  acc["obj_lines"].append(f"      stationary-target transient windows: vRel(U11) {nv}, dRel-slope {nd}")


def objects_print(acc, used):
  P("parser definitions used from opendbc/car/honda/radar_interface.py: " + ", ".join(used))
  P("frame decode: opendbc DBC '" + acc.get("obj_dbc", "?") + "' signals STATUS/FRAME_IDX/RANGE_RAW/AZIMUTH_RAW (f0), TRACK_ID/"
    "FRAME_IDX (f3), REL_VELOCITY_RAW/REL_VELOCITY_UNCERTAINTY_RAW/FRAME_IDX (aux); f0/f3/aux matched by FRAME_IDX within 40 ms.")
  for l in acc["obj_lines"]:
    P(l)
  lo, hi, n = acc["obj_rail"]
  if n:
    P(f"  [log] pooled U11 low-rail share (raw 0 = -13.5 m/s) {lo / n:.1%}, high-rail share {hi / n:.1%} of {n} live rows")

  def q(v, lbl):
    if not v:
      P(f"  [log] {lbl}: no qualifying stationary-target transient windows")
      return
    L = np.array([x[0] for x in v]) * 1e3
    nb = sum(x[1] for x in v)
    P(f"  [log] {lbl}: best lag median {np.median(L):.0f} ms, IQR {np.percentile(L, 25):.0f}-{np.percentile(L, 75):.0f} ms,"
      f" n={len(L)} windows ({nb} at search boundary -100/600 ms)")
  q(acc["obj_vlag"], "U11 vRel vs -vEgo")
  q(acc["obj_dlag"], "dRel slope vs -vEgo")
  P("  interpretation: a radar Doppler velocity should lag carState vEgo by ~0-60 ms; a camera- or fusion-sourced value"
    " would typically lag >=100 ms. Caveats: carState vEgo itself is filtered, CAN timestamps are batched (see inv),"
    " and the dRel slope uses a centred 5-sample difference.")


# ----------------------------------------------------------------------------------------------------------
# main
# ----------------------------------------------------------------------------------------------------------
def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("dir")
  ap.add_argument("--routes", default="")
  ap.add_argument("--section", default="inv,phase,decode,timing,objects")
  args = ap.parse_args()
  secs = [s.strip() for s in args.section.split(",") if s.strip()]
  files = sorted(glob.glob(os.path.join(args.dir, "*.npz")))
  if args.routes:
    want = [r.strip() for r in args.routes.split(",") if r.strip()]
    files = [f for f in files if any(w in os.path.basename(f) for w in want)]
  P(f"analyze.py: {len(files)} route file(s) in {args.dir}; sections {','.join(secs)}. All conclusions are [log] only.")
  if not files:
    return
  acc = dict(inv={}, inv_meta=[], membership=[], phase=defaultdict(list), phase_lines=[], decode={},
             timing_lines=[], timing_eps=[], timing_alerts=[], timing_dbc=set(), obj_lines=[], obj_vlag=[], obj_dlag=[],
             obj_rail=[0, 0, 0], errors=defaultdict(list), chkfn=get_checksum_fn())
  ri = used = None
  if "objects" in secs:
    try:
      ri, used = objects_imports()
      acc["obj_dbc"] = ri.BOSCH_A_DBC_NAME
    except Exception as e:
      acc["errors"]["objects"].append(f"  objects skipped: parser import failed: {tb_compact(e)}")
  pt_dbc_default = None
  for f in files:
    try:
      R = Route(f)
    except Exception as e:
      P(f"LOAD FAIL {os.path.basename(f)}: {tb_compact(e)}")
      continue
    pt_name = None
    try:
      pt_name = find_pt_dbc(R.meta.get("fingerprint"))[0]
      pt_dbc_default = pt_name
    except Exception:
      pt_name = pt_dbc_default
    for s, fn in (("inv", lambda: inv_route(R, acc)), ("phase", lambda: phase_route(R, acc)),
                  ("decode", lambda: decode_route(R, acc, pt_name)), ("timing", lambda: timing_route(R, acc)),
                  ("objects", lambda: objects_route(R, acc, ri) if ri is not None else None)):
      if s not in secs:
        continue
      try:
        fn()
      except Exception as e:
        acc["errors"][s].append(f"  {short(R.name)}: ERROR {tb_compact(e)}")
    del R
  printers = dict(inv=lambda: inv_print(acc), phase=lambda: phase_print(acc), decode=lambda: decode_print(acc),
                  timing=lambda: timing_print(acc), objects=lambda: objects_print(acc, used or []))
  for s in ("inv", "phase", "decode", "timing", "objects"):
    if s not in secs:
      continue
    P(f"\n===== {s} =====")
    for l in acc["errors"].get(s, []):
      P(l)
    if s == "objects" and ri is None:
      continue
    try:
      printers[s]()
    except Exception as e:
      P(f"  PRINT ERROR {tb_compact(e)}")
  P(f"\nopendbc used: DBCs {sorted(USED['dbc'])}; checksum {sorted(USED['checksum'])}; decode = opendbc.can.dbc.DBC signal"
    " definitions + get_raw_value bit walk (spot-checked).")


if __name__ == "__main__":
  main()
