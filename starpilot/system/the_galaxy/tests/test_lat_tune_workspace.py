import importlib.util
import json
import sys
import types
from pathlib import Path

MODULE_PATH = Path(__file__).resolve().parents[1] / "lat_tune_workspace.py"


class FakeParams:
  _store = {}
  _memory_store = {}

  def __init__(self, memory=False):
    self._s = FakeParams._memory_store if memory else FakeParams._store

  def get(self, key, block=False, return_default=False, encoding=None, default=None):
    v = self._s.get(key)
    if v is None:
      return default
    if isinstance(v, bytes) and encoding:
      return v.decode(encoding)
    return v

  def get_bool(self, key, block=False):
    return bool(self._s.get(key, False))

  def put(self, key, value):
    self._s[key] = value

  def put_bool(self, key, value):
    self._s[key] = bool(value)

  def put_nonblocking(self, key, value):
    self._s[key] = value

  def remove(self, key):
    self._s.pop(key, None)


def _stub(name, **attrs):
  m = types.ModuleType(name)
  m.__dict__.update(attrs)
  sys.modules[name] = m
  return m


def _load(tmp_path):
  FakeParams._store = {}
  FakeParams._memory_store = {}
  _stub("openpilot.common.params", Params=FakeParams)
  _stub("openpilot.system.hardware", PC=True)
  _stub("openpilot.system.hardware.hw", Paths=types.SimpleNamespace(comma_home=lambda: str(tmp_path)))
  _stub("openpilot.starpilot.system.the_galaxy.utilities", get_segments_in_route=lambda route, footage_path: [])
  spec = importlib.util.spec_from_file_location(f"test_lat_tune_ws_{abs(hash(tmp_path))}", MODULE_PATH)
  module = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(module)
  module.STATUS_PATH = tmp_path / "status.json"
  return module


def _trial(module, trial_id="lt-1", fingerprint="fp1", p=(100.0, 105.0, 100.0, 100.0)):
  t = {"schemaVersion": 1, "trialId": trial_id, "createdAt": 1.0, "knotsMph": [20.0, 30.0, 40.0, 50.0], "routeNames": ["r"],
       "warnings": [], "baseline": {"fingerprint": fingerprint, "pPct": [100.0] * 4, "raw": {}}, "factors": [1.0, 1.05, 1.0, 1.0],
       "proposedPPct": list(p), "perRoute": [], "applied": None,
       "knots": [{"mph": m, "minutes": 4.0, "ready": True, "factor": 1.0, "decision": "hold", "reason": "hold",
                  "signRate": 0.1, "curveRatio": None, "straightRms": 0.2, "pressRate": 0.0} for m in (20.0, 30.0, 40.0, 50.0)]}
  module.ensure_workspace()
  module._write_json(module.get_workspace_root() / "trials" / f"{trial_id}.json", t)
  return t


def test_workspace_root_on_pc_is_under_comma_home(tmp_path):
  module = _load(tmp_path)
  assert module.get_workspace_root() == tmp_path / "starpilot" / "data" / "galaxy" / "lat_tune"


def test_start_is_refused_onroad_and_limits_routes(tmp_path, monkeypatch):
  module = _load(tmp_path)
  FakeParams._store = {"IsOnroad": True}
  assert module.start_background_analysis(["r1"], ["/x"]) is False
  FakeParams._store = {}
  import pytest
  with pytest.raises(ValueError):
    module.start_background_analysis([f"r{i}" for i in range(9)], ["/x"])
  with pytest.raises(ValueError):
    module.start_background_analysis([], ["/x"])


def test_start_launches_detached_worker_and_writes_queued_status(tmp_path, monkeypatch):
  module = _load(tmp_path)
  calls = {}

  class P:
    pid = 4242
    def poll(self):
      return None
  monkeypatch.setattr(module.subprocess, "Popen", lambda cmd, **kw: calls.setdefault("cmd", cmd) and P())
  monkeypatch.setattr(module.threading, "Thread", lambda *a, **k: types.SimpleNamespace(start=lambda: None))
  assert module.start_background_analysis(["r1", "r2"], ["/x"]) is True
  assert calls["cmd"][:3] == ["nice", "-n", "19"] and calls["cmd"][-2] == "worker"
  assert json.loads(calls["cmd"][-1])["routes"] == ["r1", "r2"]
  st = module.read_status()
  assert st["state"] == "queued" and st["running"] is True and st["pid"] == 4242


def test_resolve_route_sources_prefers_rlog_and_warns_on_qlog(tmp_path, monkeypatch):
  module = _load(tmp_path)
  foot = tmp_path / "foot"
  (foot / "r--0").mkdir(parents=True)
  (foot / "r--0" / "rlog.zst").write_bytes(b"")
  (foot / "r--1").mkdir()
  (foot / "r--1" / "qlog.zst").write_bytes(b"")
  monkeypatch.setattr(module.utilities, "get_segments_in_route", lambda route, footage_path: ["r--0", "r--1"])
  sources, warnings = module.resolve_route_sources(["r"], [str(foot)])
  assert [Path(s.log_path).name for s in sources] == ["rlog.zst", "qlog.zst"]
  assert warnings and "qlog" in warnings[0]


def test_run_worker_writes_trial_and_complete_status(tmp_path, monkeypatch):
  module = _load(tmp_path)
  monkeypatch.setattr(module, "resolve_route_sources", lambda routes, paths: ([module.lat.RouteLog("r", "0", "/x/rlog.zst")], []))
  monkeypatch.setattr(module.lat, "analyze_sources", lambda sources, **kw: dict(_trial(module, "tmp"), routeNames=["r"]))
  monkeypatch.setattr(module.threading, "Thread", lambda *a, **k: types.SimpleNamespace(start=lambda: None))
  module.run_worker(json.dumps({"routes": ["r"], "footagePaths": ["/x"]}))
  st = module.read_status()
  assert st["state"] == "complete" and st["trialId"].startswith("lt-")
  saved = json.loads((module.get_workspace_root() / "trials" / f"{st['trialId']}.json").read_text())
  assert saved["trialId"] == st["trialId"] and saved["routeNames"] == ["r"]


def test_apply_and_revert_round_trip(tmp_path):
  module = _load(tmp_path)
  _trial(module, "lt-1", fingerprint=module.current_fingerprint())
  FakeParams._store["LatGainSchedule"] = '{"v_mph":[10,60],"p":[100,100],"i":[40,90]}'
  _trial(module, "lt-1", fingerprint=module.current_fingerprint())   # re-write with the fingerprint that includes the schedule
  result = module.apply_trial("lt-1")
  written = json.loads(FakeParams._store["LatGainSchedule"])
  assert written["p"] == [100.0, 105.0, 100.0, 100.0] and "i" in written
  assert FakeParams._memory_store["StarPilotTogglesUpdated"] is True
  assert module.load_trial("lt-1")["applied"]["priorSchedule"] == '{"v_mph":[10,60],"p":[100,100],"i":[40,90]}'
  assert module.list_workspace()["activeStack"] == ["lt-1"]
  assert result["trial"]["trialId"] == "lt-1"
  module.revert_trial("lt-1")
  assert FakeParams._store["LatGainSchedule"] == '{"v_mph":[10,60],"p":[100,100],"i":[40,90]}'
  assert module.load_trial("lt-1")["applied"] is None
  assert module.list_workspace()["activeStack"] == []


def test_revert_removes_key_when_prior_was_empty(tmp_path):
  module = _load(tmp_path)
  _trial(module, "lt-1", fingerprint=module.current_fingerprint())
  module.apply_trial("lt-1")
  assert "LatGainSchedule" in FakeParams._store
  module.revert_trial("lt-1")
  assert "LatGainSchedule" not in FakeParams._store


def test_stack_only_top_can_be_reverted_and_second_apply_chains(tmp_path):
  import pytest
  module = _load(tmp_path)
  _trial(module, "lt-1", fingerprint=module.current_fingerprint())
  module.apply_trial("lt-1")
  _trial(module, "lt-2", fingerprint=module.current_fingerprint(), p=(100.0, 110.0, 100.0, 100.0))
  module.apply_trial("lt-2")
  assert module.list_workspace()["activeStack"] == ["lt-1", "lt-2"]
  with pytest.raises(RuntimeError):
    module.revert_trial("lt-1")
  with pytest.raises(RuntimeError):
    module.delete_trial("lt-2")
  module.revert_trial("lt-2")
  assert json.loads(FakeParams._store["LatGainSchedule"])["p"] == [100.0, 105.0, 100.0, 100.0]
  module.revert_trial("lt-1")
  assert module.list_workspace()["activeStack"] == []
  module.delete_trial("lt-2")
  assert not (module.get_workspace_root() / "trials" / "lt-2.json").exists()


def test_apply_is_gated_by_fingerprint_onroad_and_double_apply(tmp_path):
  import pytest
  module = _load(tmp_path)
  _trial(module, "lt-1", fingerprint="stale")
  with pytest.raises(RuntimeError, match="tuning changed"):
    module.apply_trial("lt-1")
  module.apply_trial("lt-1", force=True)
  with pytest.raises(RuntimeError, match="already applied"):
    module.apply_trial("lt-1", force=True)
  FakeParams._store["IsOnroad"] = True
  with pytest.raises(RuntimeError, match="offroad"):
    module.revert_trial("lt-1")
  _trial(module, "lt-3", fingerprint="x")
  with pytest.raises(RuntimeError, match="offroad"):
    module.apply_trial("lt-3", force=True)


def test_list_workspace_summaries_newest_first(tmp_path):
  module = _load(tmp_path)
  for i, ts in enumerate((5.0, 9.0, 1.0)):
    t = _trial(module, f"lt-{i}")
    t["createdAt"] = ts
    module._write_json(module.get_workspace_root() / "trials" / f"lt-{i}.json", t)
  ws = module.list_workspace()
  assert [t["trialId"] for t in ws["trials"]] == ["lt-1", "lt-0", "lt-2"]
  assert set(ws["trials"][0]) >= {"trialId", "createdAt", "routeNames", "factors", "proposedPPct", "readyKnots", "applied"}
  assert "currentSchedule" in ws and "currentFingerprint" in ws and "status" in ws
