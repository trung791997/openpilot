"""Tests for tools/bosch_a_route_report.py.

The report's one job is to tell a route with REAL radar targets apart from one carrying only
firmware no-target sentinels. Per STATUS.md every clean Bosch-A replay so far has been
sentinels, so a report that cannot make that distinction would quietly turn "we still have no
target data" into "the parser ran fine".

So these tests write two synthetic rlog segments through the real parser's own frame builders
-- one with a tracked object, one all sentinels -- and assert the report separates them. The
sentinel segment is the negative control D-009 asks for: without it, "found targets" and
"always says targets" look identical.
"""

import importlib.util
import os
import zipfile

import pytest

from cereal import log
from opendbc.car.honda.radar_interface import BOSCH_A_DIRECT_VREL_INVALID, BOSCH_A_STATUS_INVALID
from opendbc.car.honda.tests.test_bosch_a_radar import sweep

_HERE = os.path.dirname(__file__)
_SCRIPT = os.path.normpath(os.path.join(_HERE, "..", "..", "bosch_a_route_report.py"))


def _load():
  spec = importlib.util.spec_from_file_location("bosch_a_route_report", _SCRIPT)
  module = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(module)
  return module


report = _load()


def _write_segment(path, n_sweeps, real_targets):
  """One synthetic rlog. Frames come from the parser's own test builders, never hand-rolled."""
  with open(path, "wb") as f:
    for i in range(n_sweeps):
      t = int(i * (1.0 / 14.35) * 1e9)
      if real_targets:
        frames = sweep(0, i % 16, 0x7, 1000 - i * 3, 1024, 1 + 2 * i, t,
                       with_aux=True, direct_vrel_raw=800, direct_vrel_uncertainty_raw=40)
      else:
        frames = sweep(0, i % 16, BOSCH_A_STATUS_INVALID, 0xFFF, 0x7FF, 0xFFF, t,
                       with_aux=True, track_id=0xFF,
                       direct_vrel_raw=BOSCH_A_DIRECT_VREL_INVALID,
                       direct_vrel_uncertainty_raw=0x3FF)
      for mono, cans in frames:
        ev = log.Event.new_message()
        ev.logMonoTime = mono
        can_list = ev.init("can", len(cans))
        for j, c in enumerate(cans):
          can_list[j].address = c.address
          can_list[j].dat = c.dat
          can_list[j].src = c.src
        ev.write(f)


@pytest.fixture
def bosch_ids():
  from opendbc.car.honda.radar_interface import BOSCH_A_ALL_IDS
  return set(BOSCH_A_ALL_IDS)


class TestTargetDiscrimination:
  def test_real_targets_are_reported(self, tmp_path, bosch_ids):
    seg = tmp_path / "rlog"
    _write_segment(seg, 40, real_targets=True)
    r = report.analyse_segment(str(seg), bosch_ids, "HONDA_CRV_5G")

    assert r["error"] is None, r["error"]
    assert r["bosch_frames"] > 0, "frames must be counted by the raw census"
    assert r["published_points"] > 0, "the parser should publish points for a tracked object"
    assert len(r["track_ids"]) >= 1
    assert r["min_dRel"] is not None and r["min_dRel"] > 0

  def test_sentinels_publish_nothing(self, tmp_path, bosch_ids):
    """The negative control. Frames present, targets absent -- the distinction that matters."""
    seg = tmp_path / "rlog"
    _write_segment(seg, 30, real_targets=False)
    r = report.analyse_segment(str(seg), bosch_ids, "HONDA_CRV_5G")

    assert r["error"] is None, r["error"]
    assert r["bosch_frames"] > 0, "sentinel frames are still frames and must be counted"
    assert r["published_points"] == 0, "no-target sentinels must not yield radar points"
    assert r["track_ids"] == set()

  def test_frames_are_attributed_to_the_camera_bus(self, tmp_path, bosch_ids):
    seg = tmp_path / "rlog"
    _write_segment(seg, 10, real_targets=True)
    r = report.analyse_segment(str(seg), bosch_ids, "HONDA_CRV_5G")
    assert sum(r["by_bus"].values()) == r["bosch_frames"]
    assert set(r["by_bus"]) == {2}, f"expected the camera bus, got {dict(r['by_bus'])}"


class TestInputHandling:
  def test_reads_a_route_zip_and_orders_segments(self, tmp_path, bosch_ids):
    for seg in (0, 1):
      d = tmp_path / f"00000231--5782493b00--{seg}"
      d.mkdir()
      _write_segment(d / "rlog", 5, real_targets=(seg == 0))
    zpath = tmp_path / "route.zip"
    with zipfile.ZipFile(zpath, "w") as z:
      for seg in (0, 1):
        z.write(tmp_path / f"00000231--5782493b00--{seg}" / "rlog",
                f"00000231--5782493b00--{seg}/rlog")

    inputs = report.collect_inputs(str(zpath), str(tmp_path / "extract"))
    assert [n for n, _ in inputs] == [0, 1]

  def test_qlog_only_archive_is_refused_with_the_reason(self, tmp_path):
    """qlogs drop the CAN data, so this has to fail loudly rather than report zero frames."""
    zpath = tmp_path / "qlogs.zip"
    with zipfile.ZipFile(zpath, "w") as z:
      z.writestr("00000231--5782493b00--0/qlog.zst", b"not a real qlog")
    with pytest.raises(SystemExit) as e:
      report.collect_inputs(str(zpath), str(tmp_path / "x"))
    assert "qlog" in str(e.value).lower()
    assert "rlog" in str(e.value).lower()

  @pytest.mark.parametrize("spec,expected", [
    ("0-4", {0, 1, 2, 3, 4}),
    ("0,3,5", {0, 3, 5}),
    ("2-3,7", {2, 3, 7}),
    (None, None),
  ])
  def test_segment_spec_parsing(self, spec, expected):
    assert report._parse_segments(spec) == expected

  @pytest.mark.parametrize("path,expected", [
    ("00000231--5782493b00--7/rlog.zst", 7),
    ("/a/b/00000231--5782493b00--12/rlog", 12),
    ("rlog", -1),
  ])
  def test_segment_number_extraction(self, path, expected):
    assert report._segment_number(path) == expected
