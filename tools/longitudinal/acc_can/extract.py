"""Read-only CAN + state extraction from route rlogs.
Usage: ACC_CAN_ROOT=<routes dir> ACC_CAN_OUT=<out dir> extract.py [route_dir ...]  (default: every route under ROOT).
Run from the openpilot repo root with PYTHONPATH=. ; writes one <route>.npz per route; never commit the outputs."""
import sys, os, json, time, traceback
from pathlib import Path
import numpy as np

ROOT = Path(os.environ.get("ACC_CAN_ROOT", "/routes"))
OUT = Path(os.environ.get("ACC_CAN_OUT", "acc_can_out")); OUT.mkdir(parents=True, exist_ok=True)
WANT = {0x1DF,0x1EF,0x1FA,0x30C,0x33D,0x39F,0x1DB,0xE4,0x640,0x641,0x420,0x440,
        0x201,0x450,0x588,0x1DD,0x1DC,0x1E1,0x183} | set(range(0x400,0x417)) | set(range(0x240,0x24B)) | set(range(0x280,0x300))

def segment_files(route_dir):
  segs = []
  for d in route_dir.iterdir():
    if d.is_dir() and d.name.isdigit():
      for name in ("rlog.zst", "rlog.bz2", "rlog"):
        if (d / name).exists():
          segs.append((int(d.name), d / name)); break
  if not segs and (route_dir / route_dir.name).is_dir():
    return segment_files(route_dir / route_dir.name)
  return [p for _, p in sorted(segs)]

def nz(x):
  return float(x)

def extract(route_dir):
  from openpilot.tools.lib.logreader import LogReader
  can = {k: [] for k in ("t","src","addr","kind","dat")}
  S = {n: [] for n in ("cs","rs","md","cc","ctl","lt")}
  meta = {"route": route_dir.name, "op_long": None, "fingerprint": None, "acc_bus": None}
  t0 = None; last_t = 0.0
  segs = segment_files(route_dir)
  for path in segs:
    for attempt in range(4):
      try:
        msgs = list(LogReader(str(path), sort_by_time=True)); break
      except Exception as e:
        if attempt == 3: raise
        time.sleep(5)
    for m in msgs:
      w = m.which()
      if t0 is None:
        if w != "initData": continue
        t0 = m.logMonoTime
      t = (m.logMonoTime - t0) / 1e9
      last_t = max(last_t, t)
      if w == "can" or w == "sendcan":
        kind = 0 if w == "can" else 1
        for f in getattr(m, w):
          a = f.address
          if a in WANT:
            d = bytes(f.dat)[:8].ljust(8, b"\0")
            can["t"].append(t); can["src"].append(f.src); can["addr"].append(a); can["kind"].append(kind); can["dat"].append(d)
      elif w == "carParams" and meta["op_long"] is None:
        meta["op_long"] = bool(m.carParams.openpilotLongitudinalControl); meta["fingerprint"] = str(m.carParams.carFingerprint)
      elif w == "carState":
        c = m.carState
        S["cs"].append((t, c.vEgo, c.aEgo, c.brakePressed, c.gasPressed, c.cruiseState.enabled, c.cruiseState.speed))
      elif w == "radarState":
        l = m.radarState.leadOne
        S["rs"].append((t, l.status, l.dRel, l.vRel, l.aLeadK, l.yRel))
      elif w == "modelV2":
        ls = m.modelV2.leadsV3
        if len(ls): S["md"].append((t, ls[0].prob, ls[0].x[0], ls[0].v[0], ls[0].a[0]))
      elif w == "carControl":
        S["cc"].append((t, m.carControl.actuators.accel, m.carControl.longActive))
      elif w in ("controlsState", "selfdriveState"):
        try: S["ctl"].append((t, m.controlsState.enabled if w == "controlsState" else m.selfdriveState.enabled, 0 if w == "controlsState" else 1))
        except Exception: pass
      elif w == "liveTracks":
        try:
          lt = m.liveTracks
          n = len(lt.points) if hasattr(lt, "points") else len(lt)
          S["lt"].append((t, n))
        except Exception: pass
    del msgs
  if t0 is None: raise RuntimeError("no initData")
  meta.update(n_segments=len(segs), duration=last_t)
  rx1df = [s_ for s_, a_, k_ in zip(can["src"], can["addr"], can["kind"]) if a_ == 0x1DF and k_ == 0 and s_ < 128]
  meta["acc_bus"] = int(np.bincount(rx1df).argmax()) if rx1df else None  # bus where received ACC_CONTROL is heaviest
  out = {}
  out["can_t"] = np.array(can["t"], np.float64)
  src = np.array(can["src"], np.int16)
  out["can_src"] = src; out["can_bus"] = (src % 128).astype(np.int8)
  out["can_addr"] = np.array(can["addr"], np.uint16); out["can_is_sendcan"] = np.array(can["kind"], np.int8)
  out["can_dat"] = np.frombuffer(b"".join(can["dat"]), np.uint8).reshape(-1, 8) if can["dat"] else np.zeros((0,8), np.uint8)
  names = {"cs": ["t","vEgo","aEgo","brakePressed","gasPressed","cruiseEnabled","cruiseSpeed"],
           "rs": ["t","status","dRel","vRel","aLeadK","yRel"], "md": ["t","prob","x0","v0","a0"],
           "cc": ["t","accel","longActive"], "ctl": ["t","enabled","is_selfdriveState"], "lt": ["t","n"]}
  for k, cols in names.items():
    arr = np.array(S[k], np.float64).reshape(-1, len(cols))
    for i, c in enumerate(cols): out[f"{k}_{c}"] = arr[:, i]
  out["meta"] = np.array(json.dumps(meta))
  np.savez_compressed(OUT / f"{route_dir.name}.npz", **out)
  return meta, len(out["can_t"])

if __name__ == "__main__":
  dirs = [Path(a) for a in sys.argv[1:]] or sorted(p for p in ROOT.iterdir() if p.is_dir() and '--' in p.name)
  for d in dirs:
    if (OUT / f"{d.name}.npz").exists(): print("skip", d.name); continue
    try:
      t = time.time(); meta, n = extract(d); print("OK", d.name, meta, n, f"{time.time()-t:.0f}s", flush=True)
    except Exception as e:
      print("FAIL", d.name, repr(e), flush=True)
