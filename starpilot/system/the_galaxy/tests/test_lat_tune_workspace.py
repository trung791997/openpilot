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


def _trial(module, trial_id="lt-1", fingerprint="fp1", p=(100, 105, 105)):
  cur = [{"p": 100, "i": 100, "f": 50}, {"p": 100, "i": 75, "f": 100}, {"p": 105, "i": 100, "f": 100}]
  bands = []
  for (name, lo, hi), c, pk, newp in zip(module.lat.BANDS, cur, module.lat.P_KEYS, p, strict=True):
    bands.append({"name": name, "lowMph": lo, "highMph": hi, "pKey": pk, "minutes": 4.0, "ready": True, "factor": newp / c["p"],
                  "decision": "hold", "reason": f"{name}: hold", "signRate": 0.1, "curveRatio": None, "straightRms": 0.2,
                  "pressRate": 0.0, "current": dict(c), "proposed": dict(c, p=newp)})
  t = {"schemaVersion": 2, "trialId": trial_id, "createdAt": 1.0, "bandNames": list(module.lat.BAND_NAMES), "routeNames": ["r"],
       "warnings": [], "baseline": {"fingerprint": fingerprint, "gains": cur, "raw": {}, "scheduleTerms": []},
       "factors": [1.0, 1.05, 1.0], "perRoute": [], "applied": None, "bands": bands}
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


def test_apply_writes_p_band_params_strips_schedule_p_and_reverts_exactly(tmp_path):
  module = _load(tmp_path)
  sched = '{"v_mph":[10,60],"p":[100,100],"i":[40,90]}'
  FakeParams._store.update({"LatPScaleLowSpeed": 100, "LatPScaleStandard": 100, "LatPScaleHighway": 105,
                            "LatIScaleStandard": 75, "LatGainSchedule": sched})
  _trial(module, "lt-1", fingerprint=module.current_fingerprint())
  result = module.apply_trial("lt-1")
  assert result["written"] == {"LatPScaleStandard": 105}                          # held bands are not written
  assert FakeParams._store["LatPScaleStandard"] == 105 and isinstance(FakeParams._store["LatPScaleStandard"], int)
  assert FakeParams._store["LatIScaleStandard"] == 75                               # I/F never written
  assert json.loads(FakeParams._store["LatGainSchedule"]) == {"v_mph": [10, 60], "i": [40, 90]}  # p dropped, i kept
  assert FakeParams._memory_store["StarPilotTogglesUpdated"] is True
  applied = module.load_trial("lt-1")["applied"]
  assert applied["priorParams"] == {"LatPScaleStandard": "100"}
  assert applied["priorSchedule"] == sched
  assert module.list_workspace()["activeStack"] == ["lt-1"]
  module.revert_trial("lt-1")
  assert (FakeParams._store["LatPScaleLowSpeed"], FakeParams._store["LatPScaleStandard"], FakeParams._store["LatPScaleHighway"]) == (100, 100, 105)
  assert FakeParams._store["LatGainSchedule"] == sched
  assert module.load_trial("lt-1")["applied"] is None
  assert module.list_workspace()["activeStack"] == []


def test_apply_without_schedule_leaves_it_absent_and_revert_removes_unset_bands(tmp_path):
  module = _load(tmp_path)
  _trial(module, "lt-1", fingerprint=module.current_fingerprint())
  module.apply_trial("lt-1")
  assert "LatGainSchedule" not in FakeParams._store
  assert FakeParams._store["LatPScaleStandard"] == 105
  module.revert_trial("lt-1")
  assert not any(k in FakeParams._store for k in module.lat.P_KEYS)   # they were unset before the apply
  assert "LatGainSchedule" not in FakeParams._store


def test_apply_removes_schedule_that_only_had_p(tmp_path):
  module = _load(tmp_path)
  FakeParams._store["LatGainSchedule"] = '{"v_mph":[20,50],"p":[100,120]}'
  _trial(module, "lt-1", fingerprint=module.current_fingerprint())
  module.apply_trial("lt-1")
  assert "LatGainSchedule" not in FakeParams._store
  module.revert_trial("lt-1")
  assert FakeParams._store["LatGainSchedule"] == '{"v_mph":[20,50],"p":[100,120]}'


def test_apply_refuses_pre_band_trials(tmp_path):
  import pytest
  module = _load(tmp_path)
  t = _trial(module, "lt-old", fingerprint=module.current_fingerprint())
  t["schemaVersion"] = 1
  module._write_json(module.get_workspace_root() / "trials" / "lt-old.json", t)
  with pytest.raises(RuntimeError, match="speed bands"):
    module.apply_trial("lt-old")


def test_stack_only_top_can_be_reverted_and_second_apply_chains(tmp_path):
  import pytest
  module = _load(tmp_path)
  _trial(module, "lt-1", fingerprint=module.current_fingerprint())
  module.apply_trial("lt-1")
  _trial(module, "lt-2", fingerprint=module.current_fingerprint(), p=(100, 110, 105))
  module.apply_trial("lt-2")
  assert module.list_workspace()["activeStack"] == ["lt-1", "lt-2"]
  with pytest.raises(RuntimeError):
    module.revert_trial("lt-1")
  with pytest.raises(RuntimeError):
    module.delete_trial("lt-2")
  module.revert_trial("lt-2")
  assert FakeParams._store["LatPScaleStandard"] == 105
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
  assert set(ws["trials"][0]) >= {"trialId", "createdAt", "routeNames", "factors", "currentP", "proposedP", "readyBands", "applied"}
  assert ws["trials"][0]["proposedP"] == [100, 105, 105] and ws["trials"][0]["readyBands"] == ["LowSpeed", "Standard", "Highway"]
  assert "currentSchedule" in ws and "currentFingerprint" in ws and "status" in ws
  assert [(b["name"], b["p"], b["i"]) for b in ws["currentBands"]] == [("LowSpeed", 100, 20), ("Standard", 100, 100), ("Highway", 100, 0)]


def test_removals_are_mirrored_into_the_params_cache(tmp_path):
  # manager_init restores unset keys from the params cache at boot; a remove() not mirrored there comes back.
  module = _load(tmp_path)
  removed = []
  module._cache_params = lambda: types.SimpleNamespace(remove=removed.append)
  FakeParams._store["LatGainSchedule"] = '{"v_mph":[20,50],"p":[100,110]}'
  _trial(module, "lt-1", fingerprint=module.current_fingerprint())
  module.apply_trial("lt-1")
  assert "LatGainSchedule" not in FakeParams._store and removed == ["LatGainSchedule"]
  module.revert_trial("lt-1")
  assert "LatPScaleStandard" in removed and "LatPScaleLowSpeed" not in removed   # only the moved band was written


def test_stop_never_signals_a_dead_or_foreign_pid(tmp_path, monkeypatch):
  module = _load(tmp_path)
  killed = []
  monkeypatch.setattr(module.os, "killpg", lambda *a: killed.append(a))
  module._write_status({"pid": 999999, "running": False, "state": "complete"})
  assert module.stop_background_analysis() is False and killed == []
  module._write_status({"pid": 999999, "running": True, "state": "analyzing"})   # pid is not alive
  assert module.stop_background_analysis() is False and killed == []
  assert module.read_status()["state"] == "cancelled"


def test_public_status_reports_a_dead_worker_as_failed(tmp_path):
  module = _load(tmp_path)
  module._write_status({"pid": 999999, "running": True, "state": "analyzing"})
  st = module.public_status()
  assert st["running"] is False and st["state"] == "failed"


def test_start_refuses_while_an_orphaned_worker_runs(tmp_path, monkeypatch):
  import os
  import pytest
  module = _load(tmp_path)
  module._write_status({"pid": os.getpid(), "running": True, "state": "analyzing"})
  with pytest.raises(RuntimeError, match="already running"):
    module.start_background_analysis(["r1"], ["/x"])


def test_applied_trial_stays_listed_past_the_newest_20(tmp_path):
  module = _load(tmp_path)
  t = _trial(module, "lt-old", fingerprint=module.current_fingerprint())
  module.apply_trial("lt-old")
  for i in range(25):
    x = _trial(module, f"lt-n{i}")
    x["createdAt"] = 100.0 + i
    module._write_json(module.get_workspace_root() / "trials" / f"lt-n{i}.json", x)
  ids = [s["trialId"] for s in module.list_workspace()["trials"]]
  assert len(ids) == 21 and "lt-old" in ids and t["createdAt"] == 1.0


def test_forced_apply_steps_from_the_device_value_and_leaves_held_bands_alone(tmp_path):
  module = _load(tmp_path)
  _trial(module, "lt-1", fingerprint="driven-on-an-older-tune")       # logged Standard P 100 -> 105
  FakeParams._store.update({"LatPScaleLowSpeed": 90, "LatPScaleStandard": 120, "LatPScaleHighway": 130})  # manual changes since
  result = module.apply_trial("lt-1", force=True)
  assert result["written"] == {"LatPScaleStandard": 125}              # 120 x 1.05, not the logged 105
  assert FakeParams._store["LatPScaleLowSpeed"] == 90 and FakeParams._store["LatPScaleHighway"] == 130
  module.revert_trial("lt-1")
  assert FakeParams._store["LatPScaleStandard"] == 120


def test_apply_refuses_a_trial_with_nothing_to_change(tmp_path):
  import pytest
  module = _load(tmp_path)
  _trial(module, "lt-1", fingerprint=module.current_fingerprint(), p=(100, 100, 105))
  with pytest.raises(RuntimeError, match="no P or I change"):
    module.apply_trial("lt-1")
  assert module.list_workspace()["activeStack"] == []


def test_apply_leaves_a_rejected_schedule_untouched(tmp_path):
  module = _load(tmp_path)
  rejected = '{"v_mph":[20,50],"p":[500,500],"i":[40,90]}'             # p > 300: controller ignores the whole thing
  FakeParams._store["LatGainSchedule"] = rejected
  _trial(module, "lt-1", fingerprint=module.current_fingerprint())
  module.apply_trial("lt-1")
  assert FakeParams._store["LatGainSchedule"] == rejected
