"""Class-2 (car past the first dashed line) proposals from a front mask, upstream vasm-op's two-mask idea, on our crops.

  python tools/vasm_finetune/class2_proposals.py --crops crops12 --model ft/run12/soup_ep3.onnx --out ft/class2

Upstream trains one model on a front mask and one on the full mask and calls "seen in full, not in front" class 2. We
have owner class-2 labels, so the front mask is MEASURED instead of drawn: on owner-labelled train crops outside
train.py's validation groups, per (route, seg, side) the background is the median of class-0 crops, and the mean
|crop - background| of class-1 vs class-2 crops gives each pixel's class-2 share. The front mask is the polygon pixels
whose smoothed share is below an Otsu split (the band only adjacent-lane cars reach). It is applied to the existing
daemon crops (same letterbox), so no re-extraction.

Scores on a crop, with the given model: s_p2 = p_distant on the full crop; s_mask = (p_car + p_distant) on the full
crop minus the same on the front-masked crop. Both are judged on the validation groups' owner labels (AUC for class 2
against class 1, and against all labelled crops) before anything is proposed. Proposals are crops without an owner
label, ranked by the better score, written to <out>/proposals.json for labeler.py. They are never written as labels.
"""

import argparse
import collections
import csv
import json
import os
import random
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from train import is_val  # noqa: E402

N_BG, N_PER = 15, 40


def owner(e):
  return "by" not in e and e.get("label") in (0, 1, 2)


def fit_masks(crops, rows, labels, frac):
  random.seed(0)
  grp = collections.defaultdict(lambda: {0: [], 1: [], 2: []})
  for r in rows:
    e = labels.get(r["file"], {})
    if owner(e) and not is_val(f"{r['route']}/{r['seg']}", frac):
      grp[(r["route"], r["seg"], r["image_side"])][e["label"]].append(r["file"])
  gray = lambda f: cv2.cvtColor(cv2.imread(os.path.join(crops, f)), cv2.COLOR_BGR2GRAY).astype(np.float32)  # noqa: E731
  masks, info = {}, {}
  for side in ("left", "right"):
    acc, n, poly = {1: 0.0, 2: 0.0}, {1: 0, 2: 0}, None
    for k, g in grp.items():
      if k[2] != side or len(g[0]) < 5 or not (g[1] or g[2]):
        continue
      bgs = np.stack([gray(f) for f in random.sample(g[0], min(N_BG, len(g[0])))])
      poly = (bgs.max(0) > 0) if poly is None else poly | (bgs.max(0) > 0)
      bg = np.median(bgs, 0)
      for y in (1, 2):
        for f in random.sample(g[y], min(N_PER, len(g[y]))):
          acc[y] = acc[y] + np.abs(gray(f) - bg)
          n[y] += 1
    m1, m2 = cv2.GaussianBlur(acc[1] / n[1], (0, 0), 6), cv2.GaussianBlur(acc[2] / n[2], (0, 0), 6)
    share = m2 / (m1 + m2 + 1e-6)
    inside = cv2.erode(poly.astype(np.uint8), np.ones((9, 9), np.uint8)) > 0
    split, _ = cv2.threshold((share[inside] * 255).astype(np.uint8), 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    front = poly & (share * 255 <= split)
    masks[side] = front
    info[side] = {"class1_crops": n[1], "class2_crops": n[2], "otsu_share": round(split / 255, 3),
                  "front_frac_of_polygon": round(float(front.sum() / poly.sum()), 3)}
  return masks, info


def scores(net, crops, files, masks, sides):
  out = np.zeros((len(files), 2), np.float32)
  for k, (f, side) in enumerate(zip(files, sides)):
    rgb = cv2.cvtColor(cv2.imread(os.path.join(crops, f)), cv2.COLOR_BGR2RGB)
    x = np.stack([rgb, rgb * masks[side][..., None]]).astype(np.float32).transpose(0, 3, 1, 2) / 255.0
    net.setInput(x)
    p = net.forward()
    out[k] = (p[0, 2], (p[0, 1] + p[0, 2]) - (p[1, 1] + p[1, 2]))
  return out


def auc(pos, neg):
  if not len(pos) or not len(neg):
    return float("nan")
  s = np.concatenate([pos, neg])
  r = s.argsort().argsort() + 1.0
  return float((r[: len(pos)].sum() - len(pos) * (len(pos) + 1) / 2) / (len(pos) * len(neg)))


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("--crops", required=True)
  ap.add_argument("--model", required=True)
  ap.add_argument("--out", required=True)
  ap.add_argument("--val-frac", type=float, default=0.1)
  ap.add_argument("--prefilter", type=float, default=0.3, help="propose only where the manifest's shipped p_car + p_distant >= this")
  ap.add_argument("--top", type=int, default=600)
  args = ap.parse_args()
  os.makedirs(args.out, exist_ok=True)

  with open(os.path.join(args.crops, "manifest.csv")) as f:
    rows = [r for r in csv.DictReader(f) if r["split"] == "train"]  # test is never read
  with open(os.path.join(args.crops, "labels.json")) as f:
    labels = json.load(f)

  masks, info = fit_masks(args.crops, rows, labels, args.val_frac)
  for side, m in masks.items():
    cv2.imwrite(os.path.join(args.out, f"front_mask_{side}.png"), m.astype(np.uint8) * 255)
  print("front masks:", json.dumps(info))

  # batch of 2 (full, front): export with a fixed batch would break this; the daemon's ONNX takes a dynamic batch
  net = cv2.dnn.readNetFromONNX(args.model)
  val = [r for r in rows if owner(labels.get(r["file"], {})) and is_val(f"{r['route']}/{r['seg']}", args.val_frac)]
  y = np.array([labels[r["file"]]["label"] for r in val])
  s = scores(net, args.crops, [r["file"] for r in val], masks, [r["image_side"] for r in val])
  res = {"masks": info, "model": args.model, "val": {"n": len(val), "class2": int((y == 2).sum()), "class1": int((y == 1).sum())}}
  for j, name in enumerate(("s_p2", "s_mask")):
    res["val"][name] = {"auc_2_vs_1": round(auc(s[y == 2, j], s[y == 1, j]), 4), "auc_2_vs_rest": round(auc(s[y == 2, j], s[y != 2, j]), 4)}
  print("validation:", json.dumps(res["val"]))
  best = max((0, 1), key=lambda j: res["val"][("s_p2", "s_mask")[j]]["auc_2_vs_rest"])
  res["proposal_score"] = ("s_p2", "s_mask")[best]

  cand = [r for r in rows if not owner(labels.get(r["file"], {})) and not is_val(f"{r['route']}/{r['seg']}", args.val_frac)
          and float(r["p_car"]) + float(r["p_distant"]) >= args.prefilter]
  print(len(cand), "unlabelled train crops pass the prefilter; scoring")
  sc = scores(net, args.crops, [r["file"] for r in cand], masks, [r["image_side"] for r in cand])[:, best]
  order = np.argsort(-sc)[: args.top]
  res["proposals"] = [{"file": cand[i]["file"], "score": round(float(sc[i]), 4)} for i in order]
  with open(os.path.join(args.out, "proposals.json"), "w") as f:
    json.dump(res, f, indent=1)
  print(len(res["proposals"]), "proposals by", res["proposal_score"], "->", os.path.join(args.out, "proposals.json"))


if __name__ == "__main__":
  main()
