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


def test_main_prints_schedule_line(tmp_path, monkeypatch, capsys):
  _mk(tmp_path, "2026-09-22--08-30-00", [0])
  fake_trial = {"knotsMph": [20.0, 30.0, 40.0, 50.0], "routeNames": ["2026-09-22--08-30-00"], "warnings": [],
                "knots": [{"mph": m, "minutes": 4.0, "ready": True, "factor": 1.0, "decision": "hold", "reason": "hold",
                           "signRate": 0.1, "curveRatio": None, "pressRate": 0.0} for m in (20.0, 30.0, 40.0, 50.0)],
                "proposedPPct": [100.0, 100.0, 100.0, 100.0], "baseline": {"fingerprint": "x", "pPct": [100.0] * 4, "raw": {}},
                "factors": [1.0] * 4, "perRoute": [], "applied": None, "schemaVersion": 1}
  monkeypatch.setattr(cli.lat, "analyze_sources", lambda sources, **kw: fake_trial)
  rc = cli.main(["--routes-root", str(tmp_path), "--latest", "1", "--json", str(tmp_path / "trial.json")])
  out = capsys.readouterr().out
  assert rc == 0
  assert 'LatGainSchedule = {"v_mph":[20.0,30.0,40.0,50.0],"p":[100.0,100.0,100.0,100.0]}' in out
  assert json.loads((tmp_path / "trial.json").read_text())["routeNames"] == ["2026-09-22--08-30-00"]


def test_main_rejects_more_than_eight(tmp_path, capsys):
  assert cli.main(["--routes-root", str(tmp_path), "--latest", "9"]) == 2
  assert "at most 8" in capsys.readouterr().err
