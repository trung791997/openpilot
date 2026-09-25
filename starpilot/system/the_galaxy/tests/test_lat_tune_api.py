import ast
import json
import types
from pathlib import Path

from flask import Flask, jsonify, request

GALAXY = Path(__file__).resolve().parents[1] / "the_galaxy.py"
HANDLERS = ["_lat_tune_error", "get_lat_tune_workspace", "get_lat_tune_status", "start_lat_tune_analysis",
            "stop_lat_tune_analysis", "get_lat_tune_trial", "delete_lat_tune_trial", "apply_lat_tune_trial",
            "revert_lat_tune_trial"]


class FakeParams:
  store = {}
  def get_bool(self, key, block=False):
    return bool(FakeParams.store.get(key, False))


def _app(ws):
  tree = ast.parse(GALAXY.read_text())
  setup = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "setup")
  wanted = [n for n in setup.body if isinstance(n, ast.FunctionDef) and n.name in HANDLERS]
  assert len(wanted) == len(HANDLERS), f"missing handlers: {set(HANDLERS) - {n.name for n in wanted}}"
  app = Flask(__name__)
  env = {"app": app, "request": request, "jsonify": jsonify, "params": FakeParams(), "lat_tune_workspace": ws,
         "FOOTAGE_PATHS": ["/x"]}
  exec(compile(ast.Module(body=wanted, type_ignores=[]), str(GALAXY), "exec"), env)
  return app.test_client()


def _ws(**over):
  ws = types.SimpleNamespace(
    list_workspace=lambda: {"trials": [], "activeStack": [], "status": {}},
    read_status=lambda: {"state": "idle"},
    cancel_if_onroad=lambda: None,
    start_background_analysis=lambda routes, paths: True,
    stop_background_analysis=lambda: True,
    load_trial=lambda tid: {"trialId": tid},
    delete_trial=lambda tid: {"trials": []},
    apply_trial=lambda tid, force=False: {"trial": {"trialId": tid}, "written": "{}", "activeStack": [tid]},
    revert_trial=lambda tid: {"trial": {"trialId": tid}, "restored": "", "activeStack": []},
  )
  for k, v in over.items():
    setattr(ws, k, v)
  return ws


def test_workspace_and_status():
  FakeParams.store = {}
  c = _app(_ws())
  assert c.get("/api/lat_tune/workspace").get_json()["trials"] == []
  st = c.get("/api/lat_tune/status").get_json()
  assert st["isOnroad"] is False and st["status"]["state"] == "idle"


def test_analyze_is_offroad_only_and_validates():
  FakeParams.store = {"IsOnroad": True}
  c = _app(_ws())
  r = c.post("/api/lat_tune/analyze", json={"routes": ["r"]})
  assert r.status_code == 409 and "offroad" in r.get_json()["error"]
  FakeParams.store = {}
  def bad(routes, paths):
    raise ValueError("too many")
  assert _app(_ws(start_background_analysis=bad)).post("/api/lat_tune/analyze", json={"routes": ["r"]}).status_code == 400
  assert c.post("/api/lat_tune/analyze", json={}).status_code == 400
  ok = c.post("/api/lat_tune/analyze", json={"routes": ["r1", "r2"]})
  assert ok.status_code == 200 and "2 route" in ok.get_json()["message"]


def test_trial_get_delete_apply_revert_error_mapping():
  FakeParams.store = {}
  def missing(tid):
    raise FileNotFoundError(tid)
  def busy(tid, force=False):
    raise RuntimeError("already applied")
  c = _app(_ws(load_trial=missing, apply_trial=busy))
  assert c.get("/api/lat_tune/trial/nope").status_code == 404
  assert c.post("/api/lat_tune/trial/lt-1/apply", json={}).status_code == 409
  c = _app(_ws())
  assert c.get("/api/lat_tune/trial/lt-1").get_json()["trialId"] == "lt-1"
  assert c.delete("/api/lat_tune/trial/lt-1").status_code == 200
  a = c.post("/api/lat_tune/trial/lt-1/apply", json={"force": True}).get_json()
  assert a["activeStack"] == ["lt-1"] and "Applied" in a["message"]
  r = c.post("/api/lat_tune/trial/lt-1/revert").get_json()
  assert r["activeStack"] == [] and "Reverted" in r["message"]
  assert c.post("/api/lat_tune/analyze/stop").get_json()["stopped"] is True
