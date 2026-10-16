"""Unit tests for autolabel.py, autolabel_check.py and pipeline.py's auto-label gate (synthetic data, no models).

  venv/bin/python -m pytest <repo>/tools/vasm_finetune/test_autolabel.py
"""

import argparse
import hashlib
import json
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import autolabel as A  # noqa: E402
import autolabel_check as C  # noqa: E402
import pipeline as P  # noqa: E402


def rows(n, side="left"):
  return [{"route": "r", "seg": "0", "image_side": side, "t": f"{0.25 * i:.3f}", "file": f"r/0/{i:04d}_{side}.png"} for i in range(n)]


def onehot(cls, conf=0.99):
  S = np.full((len(cls), 3), (1 - conf) / 2, np.float32)
  S[np.arange(len(cls)), cls] = conf
  return S


def test_decide_threshold_and_missing_class():
  S = onehot([0, 1, 2], 0.9)
  d, _ = A.decide(S, {0: 0.85, 1: 0.95})
  assert d.tolist() == [0, -1, -1]  # 1 under its threshold, 2 has none


def test_agree_needs_every_model():
  a, b = onehot([0, 1, 1, 2]), onehot([0, 1, 0, 2])
  thr = {0: 0.5, 1: 0.5, 2: 0.5}
  assert A.decide_agree([a, b], [thr, thr]).tolist() == [0, 1, -1, 2]
  # a model with no threshold for a class vetoes it
  assert A.decide_agree([a, b], [thr, {0: 0.5, 1: 0.5}]).tolist() == [0, 1, -1, -1]


def test_edge_abstain_clears_the_change_and_its_gap():
  r = rows(12)
  dec = np.array([0] * 6 + [1] * 6)
  out = A.edge_abstain(dec, r, A.sides(r), 0.5)
  # change between t=1.25 and t=1.5: everything within 0.5 s of either side is dropped
  assert out.tolist() == [0, 0, 0, -1, -1, -1, -1, -1, -1, 1, 1, 1]


def test_smooth_removes_single_frame_flicker():
  r = rows(5)
  P_ = onehot([0, 0, 1, 0, 0])
  assert A.smooth(P_, r, A.sides(r), 1).argmax(1).tolist() == [0, 0, 0, 0, 0]


def test_calibrate_reaches_precision_with_support():
  y = np.array([1] * 99 + [2] * 1 + [1] * 25 + [2] * 25)
  S = np.concatenate([onehot([1] * 100, 0.9), onehot([1] * 50, 0.6)])
  # 125/150 at >= 0.6 fail; the confident 100 hold 99/100, so the first grid step above 0.6
  assert A.calibrate(S, y, 0.98, 30)[1] == pytest.approx(0.605)
  assert 1 not in A.calibrate(S, y, 0.999, 30)
  assert 1 not in A.calibrate(S, y, 0.98, 101)  # not enough support anywhere it is precise


def test_apply_keep_and_json_keys():
  r = rows(4)
  S = onehot([0, 0, 0, 0])
  assert A.apply(S, None, {"0": 0.5}, False, {1}, r, A.sides(r), 0.0).tolist() == [-1] * 4
  assert A.apply(S, None, {"0": 0.5}, False, {0}, r, A.sides(r), 0.0).tolist() == [0] * 4
  assert A.apply(S, [S, S], {"0": [0.5, None]}, True, {0}, r, A.sides(r), 0.0).tolist() == [-1] * 4


def test_wilson_lower_bound():
  assert C.wilson_lo(0, 0) == 0.0
  assert C.wilson_lo(190, 190) >= 0.98 > C.wilson_lo(180, 180)  # 0.98 needs ~190 clean labels
  assert C.wilson_lo(53, 76) == pytest.approx(0.5867, abs=1e-4)  # v1 class 1 on route 329


def test_owner_y_ignores_agent_and_unsure():
  r = rows(4)
  labels = {r[0]["file"]: {"label": 1}, r[1]["file"]: {"label": 1, "by": "agent"}, r[2]["file"]: {"label": "x"}}
  assert C.owner_y(r, labels).tolist() == [1, -1, -1, -1]


def _pipe(tmp_path, recipe):
  w = tmp_path
  (w / "crops").mkdir(exist_ok=True)
  (w / "ft").mkdir(exist_ok=True)
  (w / "crops" / "manifest.csv").write_text("file\n")
  (w / "crops" / "labels.json").write_text("{}")
  (w / "ft" / "vasm_converted.pt").write_text("x")
  (w / "recipe.json").write_text(json.dumps(recipe))
  a = argparse.Namespace(work=str(w), crops="crops", run="t", hardneg="none.json", recipe=str(w / "recipe.json"),
                         dry_run=False, until=None, road_crops="cropsroad", status=False)
  return P.Pipeline(a)


def test_pipeline_refuses_unchecked_or_changed_auto_labels(tmp_path):
  recipe = {"seeds": [1], "rule": "highest", "train": {"epochs": 1, "auto-labels": "crops/auto.json"}}
  auto = tmp_path / "crops" / "auto.json"
  p = _pipe(tmp_path, recipe)
  auto.write_text('{"a": {"label": 0}}')
  with pytest.raises(SystemExit, match="no passing"):
    p.check()
  chk = {"passed": False, "labels_sha256": hashlib.sha256(auto.read_bytes()).hexdigest()}
  (tmp_path / "crops" / "auto.json.check.json").write_text(json.dumps(chk))
  with pytest.raises(SystemExit, match="no passing"):
    p.check()
  chk["passed"] = True
  (tmp_path / "crops" / "auto.json.check.json").write_text(json.dumps(chk))
  p.check()
  assert p.state["data"]["auto"] == chk["labels_sha256"]
  auto.write_text('{"a": {"label": 1}}')  # edited after the check
  with pytest.raises(SystemExit, match="no passing"):
    _pipe(tmp_path, recipe).check()


def test_pipeline_without_auto_keeps_old_data_keys(tmp_path):
  p = _pipe(tmp_path, {"seeds": [1], "rule": "highest", "train": {"epochs": 1}})
  p.check()
  assert set(p.state["data"]) == {"manifest", "labels", "hardneg", "queue"}  # runs started before v2 still resume
