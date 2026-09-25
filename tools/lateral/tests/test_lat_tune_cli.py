import json
from pathlib import Path

from openpilot.tools.lateral import lat_tune_cli as cli


def _mk(root, route, segs):
  for s in segs:
    d = root / f"{route}--{s}"
    d.mkdir(parents=True)
    (d / "rlog.zst").write_bytes(b"")


def test_discover_routes_latest_first_and_capped(tmp_path):
  _mk(tmp_path, "2026-09-20--10-00-00", [0, 1])
  _mk(tmp_path, "2026-09-22--08-30-00", [0])
  _mk(tmp_path, "2026-09-21--12-00-00", [0, 1, 2])
  routes = cli.discover_routes(tmp_path, latest=2)
  assert [r for r, _ in routes] == ["2026-09-21--12-00-00", "2026-09-22--08-30-00"]   # oldest first, latest 2
  assert [len(segs) for _, segs in routes] == [3, 1]
  assert routes[0][1][0].name == "rlog.zst"


def test_discover_routes_handles_dongle_prefixed_konik_dirs(tmp_path):
  _mk(tmp_path / "abc123", "2026-09-22--08-30-00", [0])
  routes = cli.discover_routes(tmp_path, latest=8)
  assert [r for r, _ in routes] == ["abc123|2026-09-22--08-30-00"]


def test_discover_routes_handles_konik_fetch_layout(tmp_path):
  for route, segs in (("00000268--4bc9811934", [0, 10, 2]), ("0000026b--92b1979afa", [0]), ("00000262--864cc3c6db", [0])):
    for s in segs:
      d = tmp_path / route / str(s)
      d.mkdir(parents=True)
      (d / "rlog.zst").write_bytes(b"")
  (tmp_path / "notes" / "3").mkdir(parents=True)
  routes = cli.discover_routes(tmp_path, latest=2)
  assert [r for r, _ in routes] == ["00000268--4bc9811934", "0000026b--92b1979afa"]
  assert [p.parent.name for p in routes[0][1]] == ["0", "2", "10"]


def test_main_prints_band_params(tmp_path, monkeypatch, capsys):
  _mk(tmp_path, "2026-09-22--08-30-00", [0])
  cur = [{"p": 100, "i": 100, "f": 50}, {"p": 100, "i": 75, "f": 100}, {"p": 105, "i": 100, "f": 100}]
  new_p = [100, 105, 105]
  bands = [{"name": n, "lowMph": lo, "highMph": hi, "pKey": f"LatPScale{n}", "minutes": 4.0, "ready": True, "factor": 1.0,
            "decision": "hold", "reason": f"{n}: hold", "signRate": 0.1, "curveRatio": None, "pressRate": 0.0,
            "current": c, "proposed": dict(c, p=p)}
           for (n, lo, hi), c, p in zip(cli.lat.BANDS, cur, new_p, strict=True)]
  fake_trial = {"bandNames": list(cli.lat.BAND_NAMES), "routeNames": ["2026-09-22--08-30-00"], "warnings": [], "bands": bands,
                "baseline": {"fingerprint": "x", "gains": cur, "raw": {}, "scheduleTerms": []},
                "factors": [1.0, 1.05, 1.0], "perRoute": [], "applied": None, "schemaVersion": 2}
  monkeypatch.setattr(cli.lat, "analyze_sources", lambda sources, **kw: fake_trial)
  rc = cli.main(["--routes-root", str(tmp_path), "--latest", "1", "--json", str(tmp_path / "trial.json")])
  out = capsys.readouterr().out
  assert rc == 0
  assert "LatPScaleLowSpeed = 100\n" in out
  assert "LatPScaleStandard = 105   (was 100)" in out
  assert "LatPScaleHighway = 105\n" in out
  assert "100/75/100" in out and "50+" in out and "25-50" in out
  assert "LatGainSchedule =" not in out
  assert json.loads((tmp_path / "trial.json").read_text())["routeNames"] == ["2026-09-22--08-30-00"]


def test_main_passes_baseline_overrides_and_rejects_bad_keys(tmp_path, monkeypatch, capsys):
  _mk(tmp_path, "2026-09-22--08-30-00", [0])
  seen = {}
  monkeypatch.setattr(cli.lat, "analyze_sources", lambda sources, **kw: seen.update(kw) or (_ for _ in ()).throw(SystemExit(0)))
  import pytest
  with pytest.raises(SystemExit):
    cli.main(["--routes-root", str(tmp_path), "--latest", "1", "--baseline", "LatPScaleStandard=115"])
  assert seen["baseline_overrides"] == {"LatPScaleStandard": "115"}
  assert cli.main(["--routes-root", str(tmp_path), "--baseline", "HondaCenterScale=5"]) == 2
  assert cli.main(["--routes-root", str(tmp_path), "--baseline", "LatPScaleStandard=abc"]) == 2


def test_main_rejects_more_than_eight(tmp_path, capsys):
  assert cli.main(["--routes-root", str(tmp_path), "--latest", "9"]) == 2
  assert "at most 8" in capsys.readouterr().err


def test_main_passes_mixed_tuning_flag(tmp_path, monkeypatch):
  _mk(tmp_path, "2026-09-22--08-30-00", [0])
  seen = {}
  monkeypatch.setattr(cli.lat, "analyze_sources", lambda sources, **kw: seen.update(kw) or (_ for _ in ()).throw(SystemExit(0)))
  import pytest
  for argv, want in (([], False), (["--mixed-tuning"], True)):
    with pytest.raises(SystemExit):
      cli.main(["--routes-root", str(tmp_path), "--latest", "1", *argv])
    assert seen["mixed_tuning"] is want
