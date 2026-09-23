"""Tests for tools/bosch_a_vrel_shadow_report.py."""

from __future__ import annotations

import importlib.util
import os
import pytest

from cereal import log

_HERE = os.path.dirname(__file__)
_SCRIPT = os.path.normpath(os.path.join(_HERE, "..", "..", "bosch_a_vrel_shadow_report.py"))


def _load():
  spec = importlib.util.spec_from_file_location("bosch_a_vrel_shadow_report", _SCRIPT)
  module = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(module)
  return module


report = _load()


class TestStatistics:
  def test_percentiles_and_pearson_against_hand_computed_values(self):
    values = list(range(1, 101))
    p = report.percentiles(values)
    assert p["n"] == 100
    assert p["min"] == 1
    assert p["max"] == 100
    assert p["mean"] == pytest.approx(50.5)
    assert p["p5"] == pytest.approx(5.95)
    assert p["p25"] == pytest.approx(25.75)
    assert p["p50"] == pytest.approx(50.5)
    assert p["p75"] == pytest.approx(75.25)
    assert p["p95"] == pytest.approx(95.05)
    assert p["p99"] == pytest.approx(99.01)

    assert report.percentiles([]) == {"n": 0}

    single = report.percentiles([42.0])
    assert single["n"] == 1
    for k in ("min", "max", "mean", "p5", "p25", "p50", "p75", "p95", "p99"):
      assert single[k] == pytest.approx(42.0)

    # Pearson correlation
    assert report.pearson([1.0, 2.0, 3.0, 4.0], [2.0, 4.0, 6.0, 8.0]) == pytest.approx(1.0)
    assert report.pearson([1.0, 2.0, 3.0, 4.0], [8.0, 6.0, 4.0, 2.0]) == pytest.approx(-1.0)
    # Zero variance returns None
    assert report.pearson([5.0, 5.0, 5.0, 5.0], [1.0, 2.0, 3.0, 4.0]) is None
    assert report.pearson([1.0, 2.0, 3.0, 4.0], [5.0, 5.0, 5.0, 5.0]) is None
    # n < 3 returns None
    assert report.pearson([1.0, 2.0], [3.0, 4.0]) is None
    assert report.pearson([], []) is None
    # Mismatched length returns None
    assert report.pearson([1.0, 2.0, 3.0], [1.0, 2.0]) is None

  def test_diff_stats_excludes_nan_and_counts_it(self):
    samples = [
      {"vRel": -4.0, "vRelRangeDerived": -4.0, "measuredRadar": True},    # diff = 0.0
      {"vRel": -5.0, "vRelRangeDerived": -8.0, "measuredRadar": True},    # diff = 3.0 (>2)
      {"vRel": -6.0, "vRelRangeDerived": -12.0, "measuredRadar": False},  # diff = 6.0 (>5)
      {"vRel": -4.0, "vRelRangeDerived": float("nan"), "measuredRadar": True},
      {"vRel": -5.0, "vRelRangeDerived": float("nan"), "measuredRadar": False},
    ]
    st = report.diff_stats(samples)
    assert st["n"] == 3
    assert st["nanCount"] == 2
    assert st["diff"]["n"] == 3
    assert st["diff"]["min"] == pytest.approx(0.0)
    assert st["diff"]["max"] == pytest.approx(6.0)
    assert st["diff"]["mean"] == pytest.approx(3.0)
    assert st["diff"]["p50"] == pytest.approx(3.0)
    assert st["absDiff"]["p50"] == pytest.approx(3.0)
    assert st["fracAbsOver2"] == pytest.approx(2.0 / 3.0)
    assert st["fracAbsOver5"] == pytest.approx(1.0 / 3.0)
    assert st["r"] == pytest.approx(1.0)

  def test_rail_stats_sign_and_filtering(self):
    # vRel=-13.5, vRelRangeDerived=-18.7 gives understatement p50 == 5.2 (approx)
    samples = [
      {"vRel": -13.5, "vRelRangeDerived": -18.7, "measuredRadar": True},
      {"vRel": -13.5, "vRelRangeDerived": -18.7, "measuredRadar": False},
      {"vRel": -10.0, "vRelRangeDerived": -15.0, "measuredRadar": True},        # not <= rail
      {"vRel": -13.5, "vRelRangeDerived": float("nan"), "measuredRadar": True},  # NaN
    ]
    rail = report.rail_stats(samples, rail=-13.49)
    assert rail["n"] == 2
    assert rail["understatement"]["p50"] == pytest.approx(5.2)

  def test_onset_lags_and_negative_control(self):
    # Synthetic run where vRelRangeDerived crosses -2.0 at t=1.0 and vRel at t=1.3 returns [0.3]
    # Consecutive samples must have dt <= 0.2 s to remain in the same run.
    samples = []
    for i in range(25):
      t = round(i * 0.1, 1)
      v_rel = -2.5 if t >= 1.3 else -0.5
      v_range = -2.5 if t >= 1.0 else -0.5
      samples.append({"t": t, "trackId": 1, "vRel": v_rel, "vRelRangeDerived": v_range})

    lags = report.onset_lags(samples, threshold=-2.0)
    assert len(lags) == 1
    assert lags[0] == pytest.approx(0.3)

    # NEGATIVE CONTROL: a single noisy range sample below threshold is not an onset. With it, a
    # first-sample rule would report 0.3 - (-0.8) = 1.1 s; the sustained rule must still say 0.3.
    noisy = [dict(s) for s in samples]
    noisy[2]["vRelRangeDerived"] = -2.4
    assert report.onset_lags(noisy, threshold=-2.0) == [pytest.approx(0.3)]

    # NEGATIVE CONTROL: crossings 10 s apart in one run are two events, not a 10 s onset lag.
    far = []
    for i in range(160):
      t = round(i * 0.1, 1)
      far.append({"t": t, "trackId": 1, "vRel": -2.5 if t >= 12.0 else -0.5,
                  "vRelRangeDerived": -2.5 if t >= 2.0 else -0.5})
    assert report.onset_lags(far, threshold=-2.0) == []

    # NEGATIVE CONTROL: a run starting already below -1.0 returns []
    neg_samples = [
      {"t": 0.0, "trackId": 1, "vRel": -1.5, "vRelRangeDerived": -1.5},
      {"t": 0.1, "trackId": 1, "vRel": -2.5, "vRelRangeDerived": -2.5},
    ]
    assert report.onset_lags(neg_samples, threshold=-2.0) == []

    # Track ID change splits runs
    split_samples = [
      {"t": 0.0, "trackId": 1, "vRel": 0.0, "vRelRangeDerived": 0.0},
      {"t": 1.0, "trackId": 1, "vRel": -1.5, "vRelRangeDerived": -2.2},  # crosses on track 1
      {"t": 1.1, "trackId": 2, "vRel": 0.0, "vRelRangeDerived": 0.0},    # track ID switches
      {"t": 1.3, "trackId": 2, "vRel": -2.1, "vRelRangeDerived": -1.5},  # crosses on track 2
    ]
    assert report.onset_lags(split_samples, threshold=-2.0) == []

    # Time gap > 0.2 s splits runs
    gap_samples = [
      {"t": 0.0, "trackId": 1, "vRel": 0.0, "vRelRangeDerived": 0.0},
      {"t": 1.0, "trackId": 1, "vRel": -1.5, "vRelRangeDerived": -2.2},
      {"t": 1.3, "trackId": 1, "vRel": -2.1, "vRelRangeDerived": -2.8},  # gap 0.3 s > 0.2 s
    ]
    assert report.onset_lags(gap_samples, threshold=-2.0) == []


class TestExtractionAndPooling:
  def test_extraction_synthetic_rlog(self, tmp_path):
    rlog_path = tmp_path / "rlog"
    with open(rlog_path, "wb") as f:
      # carState setting latest vEgo
      ev_cs = log.Event.new_message()
      ev_cs.logMonoTime = 1000
      ev_cs.init("carState")
      ev_cs.carState.vEgo = 22.5
      ev_cs.write(f)

      # radarState 1: valid lead, measured=True
      ev1 = log.Event.new_message()
      ev1.logMonoTime = 2000
      ev1.init("radarState")
      lead1 = ev1.radarState.leadOne
      lead1.status = True
      lead1.radar = True
      lead1.radarTrackId = 4
      lead1.dRel = 45.0
      lead1.vRel = -3.0
      lead1.vRelRangeDerived = -3.2
      lead1.measuredRadar = True
      ev1.write(f)

      # radarState 2: radar=False (must be EXCLUDED)
      ev2 = log.Event.new_message()
      ev2.logMonoTime = 3000
      ev2.init("radarState")
      lead2 = ev2.radarState.leadOne
      lead2.status = True
      lead2.radar = False
      lead2.radarTrackId = 4
      lead2.dRel = 44.0
      lead2.vRel = -3.0
      lead2.vRelRangeDerived = -3.2
      lead2.measuredRadar = False
      ev2.write(f)

      # radarState 3: valid lead, measured=False
      ev3 = log.Event.new_message()
      ev3.logMonoTime = 4000
      ev3.init("radarState")
      lead3 = ev3.radarState.leadOne
      lead3.status = True
      lead3.radar = True
      lead3.radarTrackId = 4
      lead3.dRel = 43.0
      lead3.vRel = -3.0
      lead3.vRelRangeDerived = -3.1
      lead3.measuredRadar = False
      ev3.write(f)

    res = report.analyse_segment(str(rlog_path), onset_threshold=-2.0)
    assert res["radarLeadFrames"] == 2
    assert res["all"]["n"] == 2
    assert res["measured"]["n"] == 1
    assert res["unmeasured"]["n"] == 1
    assert len(res["_samples"]) == 2
    assert res["_samples"][0]["vEgo"] == pytest.approx(22.5)
    assert res["_samples"][0]["trackId"] == 4
    assert res["_samples"][0]["measuredRadar"] is True
    assert res["_samples"][1]["measuredRadar"] is False

  def test_pool_and_render(self, tmp_path):
    seg1 = {
      "segment": 0,
      "radarLeadFrames": 2,
      "onsetLags": [0.3],
      "_samples": [
        {"t": 0.1, "trackId": 1, "dRel": 30.0, "vRel": -2.0, "vRelRangeDerived": -2.1, "measuredRadar": True, "vEgo": 20.0},
        {"t": 0.2, "trackId": 1, "dRel": 29.8, "vRel": -2.0, "vRelRangeDerived": -2.2, "measuredRadar": False, "vEgo": 20.0},
      ],
    }
    seg2 = {
      "segment": 1,
      "radarLeadFrames": 1,
      "onsetLags": [0.4],
      "_samples": [
        {"t": 0.1, "trackId": 2, "dRel": 40.0, "vRel": -13.5, "vRelRangeDerived": -18.7, "measuredRadar": True, "vEgo": 15.0},
      ],
    }
    pooled = report.pool([seg1, seg2])
    assert pooled["segments"] == 2
    assert pooled["radarLeadFrames"] == 3
    assert pooled["all"]["n"] == 3
    assert pooled["measured"]["n"] == 2
    assert pooled["unmeasured"]["n"] == 1
    assert pooled["rail"]["n"] == 1
    assert pooled["rail"]["understatement"]["p50"] == pytest.approx(5.2)
    assert pooled["onsetLag"]["n"] == 2
    assert pooled["onsetLag"]["mean"] == pytest.approx(0.35)

    rendered = report.render([seg1, seg2], pooled)
    assert "Bosch-A vRel (U11) vs vRelRangeDerived (D-044) Report" in rendered
    assert "Segments processed: 2" in rendered
    assert "all" in rendered
    assert "measured" in rendered
    assert "unmeasured" in rendered
    assert "Saturation rail" in rendered
    assert "Braking onset lag" in rendered
    assert "NOT validated against a real route" in rendered
