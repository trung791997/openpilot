"""Automatic labels for TRAIN crops, calibrated on the owner's validation labels. Never writes labels.json.

  python tools/vasm_finetune/autolabel.py --crops crops12 --model ft/run12p/soup_ep3.onnx [--model ...]
  python tools/vasm_finetune/train.py ... --owner-only --auto-labels crops12/autolabels.json

The owner asked on 2026-10-16 for labelling to be fully automated, learning from the earlier labelling mistakes:
- Agent labels put the adjacent lane one dashed line too far out (107 of 201 owner-2 test crops were agent 1; run1,
  trained on them, failed the gate). So agent labels are not used here at all, and class 1 is judged with owner 2
  counted as an error, the confusion that failed run1. Train with --owner-only so auto labels replace agent ones.
- Car slivers at a car's first and last frames are 1 to the owner and were often 0 to agents. Crops within --edge-gap s
  of a change of decided class on the same side get no label: the model's boundary is where it is least reliable.
- Single-frame flickers (reflections, walls): probabilities are median-filtered over --smooth frames each side along
  the segment side before deciding.

Per class c the threshold is the lowest confidence at which, on owner-labelled crops in train.py's validation groups,
the decided-c crops reach --precision with at least --min-support of them; a class that never reaches it is not
auto-labelled. Validation groups, queue segments and test crops are never labelled, and owner labels are never
replaced, so validation, queue selection and the benchmark stay on owner labels. The precision and recall reported
on validation are measured after the edge rule, exactly as applied to the labelled crops.
Writes <out> = {file: {label, by, how, conf}} and <out>.report.json. Model scores are cached next to <out>.

v1 failed its independent check (2026-10-16, route 329, owner labels): class 1 precision 0.70, 23 of the 76 errors
owner class 2, with whole wrong events at p1 0.999. One run12-family model labelling its own training data copies its
own class-2-as-1 mistakes, and the validation groups share routes with its training, so validation looked fine. So:
- --agree: each --model gets its own validation thresholds and a crop is labelled only when every model decides the
  same class. Use models trained differently (e.g. ul_scratch.py from ImageNet next to a run12 soup); averaging
  (the default) lets one confident model outvote the other.
- --classes: emit only these classes, so a class that fails the check can be left out.
- No labels are usable for training until autolabel_check.py passes them on an owner-labelled route outside both the
  labeller's calibration and the models' training; pipeline.py refuses a recipe whose auto labels lack a passing
  check for that exact file (sha256). Radar lateral position was measured as a third signal and does not separate
  class 1 from class 2 (diag/radar_lat.py, 2026-10-16: validation |yRel| median 3.53 m for owner 1 vs 3.13 m for owner 2;
  leadLeft/leadRight are already adjacent-lane picks), so it is not used.
- --min-speed: v2's check on route 327 (2026-10-17) failed class 1 almost entirely at a standstill: 1,158 of its
  1,159 owner-0-as-1 errors are in segments 6-13, a drive-through line at v_ego 0.0 where landscaping grass beside the
  car read as an adjacent car to both models. Stopped crops can then be left to the owner.
"""

import argparse
import csv
import hashlib
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from benchmark import infer  # noqa: E402
from train import in_segments, is_val, queue_segments  # noqa: E402

GRID = np.round(np.arange(0.50, 1.0, 0.005), 3)


def scores(models, crops, rows, cache_dir):
  # per-model cache keyed by crop file, so a crop set that grows (new routes) only scores the new crops
  acc = np.zeros((len(rows), 3), np.float32)
  os.makedirs(cache_dir, exist_ok=True)
  for m in models:
    c = os.path.join(cache_dir, f"autolabel_{hashlib.sha1(open(m, 'rb').read()).hexdigest()[:8]}.npz")
    have = dict(zip(*(lambda z: (list(z["files"]), z["p"]))(np.load(c)))) if os.path.exists(c) else {}
    todo = [r for r in rows if r["file"] not in have]
    if todo:
      print("scoring", len(todo), "crops with", m, flush=True)
      have.update(zip([r["file"] for r in todo], infer(m, crops, todo)))
      np.savez(c, files=np.array(list(have)), p=np.stack(list(have.values())))
    acc += np.stack([have[r["file"]] for r in rows])
  return acc / len(models)


def sides(rows):
  by = {}
  for i, r in enumerate(rows):
    by.setdefault((r["route"], r["seg"], r["image_side"]), []).append(i)
  return [sorted(v, key=lambda i: float(rows[i]["t"])) for v in by.values()]


def smooth(P, rows, seqs, k):
  S = P.copy()
  for idx in seqs:
    t = np.array([float(rows[i]["t"]) for i in idx])
    for n in range(len(idx)):
      lo, hi = max(0, n - k), min(len(idx), n + k + 1)
      win = [m for m in range(lo, hi) if abs(t[m] - t[n]) <= 0.26 * k + 0.01]
      S[idx[n]] = np.median(P[[idx[m] for m in win]], 0)
  return S


def decide(S, thr):
  y = S.argmax(1)
  conf = S.max(1)
  ok = np.array([thr.get(int(c)) is not None and conf[i] >= thr[int(c)] for i, c in enumerate(y)])
  return np.where(ok, y, -1), conf


def edge_abstain(dec, rows, seqs, gap):
  out = dec.copy()
  for idx in seqs:
    t = np.array([float(rows[i]["t"]) for i in idx])
    d = dec[idx]
    known = np.where(d >= 0)[0]
    for a, b in zip(known[:-1], known[1:]):
      if d[a] != d[b]:
        for n in range(len(idx)):
          if min(abs(t[n] - t[a]), abs(t[n] - t[b])) <= gap or t[a] <= t[n] <= t[b]:
            out[idx[n]] = -1
  return out


def decide_agree(Ss, thrs):
  # a crop is decided only where every model decides, and all decide the same class
  ds = [decide(S, t)[0] for S, t in zip(Ss, thrs)]
  out = ds[0].copy()
  for d in ds[1:]:
    out = np.where(out == d, out, -1)
  return out


def apply(S, Ss, thr, agree, keep, rows, seqs, gap, min_speed=0.0):
  """The whole decision after scoring and smoothing; autolabel_check.py calls this too, so the check judges exactly
  what was labelled. thr: {class: t} or, with agree, {class: [t per model]} (JSON string keys accepted).
  min_speed > 0: crops with v_ego below it get no label, applied after the edge rule so it moves no other decision."""
  thr = {int(c): t for c, t in thr.items()}
  if agree:
    raw = decide_agree(Ss, [{c: thr[c][j] for c in thr if thr[c][j] is not None} for j in range(len(Ss))])
  else:
    raw = decide(S, {c: t for c, t in thr.items() if t is not None})[0]
  raw = np.where(np.isin(raw, list(keep)), raw, -1)
  out = edge_abstain(raw, rows, seqs, gap)
  if min_speed > 0:
    out = np.where(np.array([float(r["v_ego"]) >= min_speed for r in rows]), out, -1)
  return out


def calibrate(S, y, prec, support):
  thr = {}
  pred, conf = S.argmax(1), S.max(1)
  for c in (0, 1, 2):
    for g in GRID:
      sel = (pred == c) & (conf >= g)
      if sel.sum() >= support and (y[sel] == c).mean() >= prec:
        thr[c] = float(g)
        break
  return thr


def report(dec, y):
  res = {}
  for c in (0, 1, 2):
    sel = dec == c
    res[c] = {"labelled": int(sel.sum()), "precision": round(float((y[sel] == c).mean()), 4) if sel.any() else None,
              "recall": round(float((dec[y == c] == c).mean()), 4) if (y == c).any() else None,
              "wrong_as": {int(w): int(((y == w) & sel).sum()) for w in (0, 1, 2) if w != c}}
  res["abstained"] = round(float((dec < 0).mean()), 4)
  return res


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("--crops", required=True)
  ap.add_argument("--model", action="append", required=True, help="ONNX; repeat for an ensemble (probabilities averaged)")
  ap.add_argument("--out", default=None, help="default <crops>/autolabels.json")
  ap.add_argument("--val-frac", type=float, default=0.1)
  ap.add_argument("--precision", type=float, default=0.98)
  ap.add_argument("--min-support", type=int, default=30)
  ap.add_argument("--smooth", type=int, default=1, help="median filter half-width in 4 Hz frames")
  ap.add_argument("--edge-gap", type=float, default=0.5)
  ap.add_argument("--exclude-queue", default=None, help="default <crops>/queue.json if present")
  ap.add_argument("--agree", action="store_true", help="per-model thresholds; label only where all models agree")
  ap.add_argument("--classes", default="0,1,2", help="emit only these classes")
  ap.add_argument("--version", default="auto-v2")
  ap.add_argument("--min-speed", type=float, default=0.0, help="no label below this v_ego (m/s); 0 = off")
  args = ap.parse_args()
  out = args.out or os.path.join(args.crops, "autolabels.json")
  queue = args.exclude_queue or os.path.join(args.crops, "queue.json")
  segs = queue_segments(queue) if os.path.exists(queue) else set()

  with open(os.path.join(args.crops, "manifest.csv")) as f:
    rows = [r for r in csv.DictReader(f) if r["split"] == "train"]  # test is never read
  with open(os.path.join(args.crops, "labels.json")) as f:
    labels = json.load(f)
  owner = {f: e["label"] for f, e in labels.items() if "by" not in e and e.get("label") in (0, 1, 2)}

  cache = os.path.dirname(os.path.abspath(out))
  seqs = sides(rows)
  Ss = [smooth(scores([m], args.crops, rows, cache), rows, seqs, args.smooth) for m in args.model] if args.agree else None
  S = smooth(scores(args.model, args.crops, rows, cache), rows, seqs, args.smooth)
  val = np.array([is_val(f"{r['route']}/{r['seg']}", args.val_frac) for r in rows])
  yv = np.array([owner.get(r["file"], -1) for r in rows])
  cal = val & (yv >= 0)

  keep = {int(c) for c in args.classes.split(",") if c != ""}
  if args.agree:
    thrs = [calibrate(x[cal], yv[cal], args.precision, args.min_support) for x in Ss]
    thr = {c: [t.get(c) for t in thrs] for c in (0, 1, 2)}
  else:
    thr = calibrate(S[cal], yv[cal], args.precision, args.min_support)
  dec = apply(S, Ss, thr, args.agree, keep, rows, seqs, args.edge_gap, args.min_speed)
  conf = S.max(1)
  rep = {"version": args.version, "models": args.model,
         "model_sha256": [hashlib.sha256(open(m, "rb").read()).hexdigest() for m in args.model],
         "args": vars(args), "thresholds": thr,
         "val_owner_crops": int(cal.sum()), "val": report(dec[cal], yv[cal]),
         "val_shipped_argmax": report(np.array([int(np.argmax([float(r["p_nocar"]), float(r["p_car"]), float(r["p_distant"])]))
                                               for r in rows])[cal], yv[cal])}

  target = np.array([not val[i] and r["file"] not in owner and not in_segments(
      {"route": r["route"], "seg": r["seg"]}, segs) for i, r in enumerate(rows)])
  how = f"{'agree' if args.agree else 'ens'}{len(args.model)}+med{args.smooth}+edge{args.edge_gap}"
  auto = {rows[i]["file"]: {"label": int(dec[i]), "by": args.version, "how": how,
                            "conf": round(float(conf[i]), 4)} for i in np.where(target & (dec >= 0))[0]}
  agent = {f: e["label"] for f, e in labels.items() if "by" in e and e.get("label") in (0, 1, 2)}
  both = [f for f in auto if f in agent]
  rep["labelled"] = {"target_crops": int(target.sum()), "auto": len(auto),
                     "classes": np.bincount([e["label"] for e in auto.values()], minlength=3).tolist(),
                     "vs_agent": {f"{a}->{b}": sum(agent[f] == a and auto[f]["label"] == b for f in both)
                                  for a in (0, 1, 2) for b in (0, 1, 2)}}
  # the rules above, asserted on the output: never an owner-labelled, validation, queue or test crop
  byfile = {r["file"]: (i, r) for i, r in enumerate(rows)}
  for f, e in auto.items():
    i, r = byfile[f]
    assert f not in owner and not val[i] and r["split"] == "train" and target[i] and e["label"] in keep, f
  with open(out, "w") as f:
    json.dump(auto, f)
  rep["sha256"] = hashlib.sha256(open(out, "rb").read()).hexdigest()
  with open(out + ".report.json", "w") as f:
    json.dump(rep, f, indent=1)
  print(json.dumps({k: rep[k] for k in ("thresholds", "val", "labelled")}, indent=1))


if __name__ == "__main__":
  main()
