"""Lateral Tune workspace (FLM-style trials for the modified-EPS Honda P trim, item 117).

Storage: <galaxy dir>/lat_tune/trials/<trialId>.json, active.json (applied stack), status in /tmp.
Analysis runs in a detached worker (`python lat_tune_workspace.py worker <json>`), offroad only.
Apply writes StarPilot's PID band params LatPScaleLowSpeed/Standard/Highway (P only; I/F are never touched) and
drops a "p" term from LatGainSchedule so the bands drive P. Revert restores all four exactly. Offroad only.
Unit-test/replay evidence only, not driven.
"""
import json
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

from openpilot.common.params import Params
from openpilot.system.hardware import PC
from openpilot.system.hardware.hw import Paths
from openpilot.selfdrive.controls.lib import lat_tune_analyzer as lat
from openpilot.starpilot.system.the_galaxy import utilities

STATUS_PATH = Path("/tmp/galaxy_lat_tune_status.json")
LOG_PATH = Path("/tmp/galaxy_lat_tune.log")
STATUS_MAX_AGE_SECONDS = 3600.0
ROUTE_LIMIT = 8
ONROAD_POLL_INTERVAL_SECONDS = 0.25
SCHEDULE_KEY = lat.SCHEDULE_KEY
_PROCESS = None
_LOCK = threading.Lock()


class AnalysisCancelled(RuntimeError):
  pass


# ---------------------------------------------------------------- storage

def _galaxy_dir():
  return Path(Paths.comma_home()) / "starpilot" / "data" / "galaxy" if PC else Path("/data/galaxy")


def get_workspace_root():
  return _galaxy_dir() / "lat_tune"


def ensure_workspace():
  root = get_workspace_root()
  (root / "trials").mkdir(parents=True, exist_ok=True)
  return root


def _read_json(path, default):
  try:
    return json.loads(Path(path).read_text())
  except (OSError, ValueError):
    return default


def _write_json(path, payload):
  path = Path(path)
  tmp = path.with_suffix(path.suffix + ".tmp")
  tmp.write_text(json.dumps(payload, indent=2, sort_keys=True))
  tmp.replace(path)


def _active_path():
  return get_workspace_root() / "active.json"


def _read_stack():
  return list(_read_json(_active_path(), {}).get("stack", []))


def _write_stack(stack):
  if stack:
    _write_json(_active_path(), {"stack": stack, "updatedAt": time.time()})
  else:
    _active_path().unlink(missing_ok=True)


def _trial_path(trial_id):
  if not trial_id or "/" in trial_id or trial_id.startswith("."):
    raise ValueError("bad trial id")
  return get_workspace_root() / "trials" / f"{trial_id}.json"


def load_trial(trial_id):
  ensure_workspace()
  trial = _read_json(_trial_path(trial_id), None)
  if trial is None:
    raise FileNotFoundError(f"trial {trial_id} not found")
  return trial


def save_trial(trial):
  ensure_workspace()
  _write_json(_trial_path(trial["trialId"]), trial)
  return trial


def delete_trial(trial_id):
  trial = load_trial(trial_id)
  if trial.get("applied"):
    raise RuntimeError("revert the trial before deleting it")
  _trial_path(trial_id).unlink(missing_ok=True)
  return list_workspace()


def _summary(trial):
  bands = trial.get("bands", [])
  return {"trialId": trial["trialId"], "createdAt": trial.get("createdAt"), "routeNames": trial.get("routeNames", []),
          "factors": trial.get("factors"), "schemaVersion": trial.get("schemaVersion"),
          "currentP": [b["current"]["p"] for b in bands], "proposedP": [b["proposed"]["p"] for b in bands],
          "readyBands": [b["name"] for b in bands if b.get("ready")],
          "applied": trial.get("applied"), "warnings": trial.get("warnings", [])}


# ---------------------------------------------------------------- params

def _params():
  return Params()


def current_schedule(params=None):
  raw = (params or _params()).get(SCHEDULE_KEY)
  return lat._param_str(raw)


def current_tuning(params=None):
  p = params or _params()
  return {k: lat._param_str(p.get(k)) for k in lat.TUNING_KEYS}


def current_band_gains(params=None):
  return lat.band_gains(current_tuning(params))


def current_fingerprint(params=None):
  return lat.tuning_fingerprint(current_tuning(params))


def _require_offroad(params=None, what="This action"):
  if (params or _params()).get_bool("IsOnroad"):
    raise RuntimeError(f"{what} is offroad only; park the car first.")


# ---------------------------------------------------------------- status

def read_status():
  return _read_json(STATUS_PATH, {})


def _write_status(payload):
  payload = dict(payload)
  payload["updatedAt"] = time.time()
  _write_json(STATUS_PATH, payload)


def clear_status():
  STATUS_PATH.unlink(missing_ok=True)


def analyzer_running():
  global _PROCESS
  with _LOCK:
    if _PROCESS is not None and _PROCESS.poll() is None:
      return True
    _PROCESS = None
  st = read_status()
  if not st.get("running"):
    return False
  if time.time() - float(st.get("updatedAt", 0)) > STATUS_MAX_AGE_SECONDS:
    return False
  try:
    os.kill(int(st.get("pid", 0)), 0)
    return True
  except (OSError, ValueError):
    return False


def stop_background_analysis(reason="cancelled"):
  st = read_status()
  pid = int(st.get("pid") or 0)
  stopped = False
  if pid:
    try:
      os.killpg(os.getpgid(pid), signal.SIGTERM)
      stopped = True
    except (OSError, ProcessLookupError):
      pass
  if st.get("running"):
    _write_status({**st, "running": False, "state": reason})
  return stopped


def cancel_if_onroad(params=None):
  if (params or _params()).get_bool("IsOnroad") and analyzer_running():
    stop_background_analysis("cancelled_onroad")


# ---------------------------------------------------------------- routes -> logs

LOG_CANDIDATES = ("rlog.zst", "rlog.bz2", "rlog")
QLOG_CANDIDATES = ("qlog.zst", "qlog.bz2", "qlog")


def resolve_route_sources(route_names, footage_paths):
  """RouteLog list (route order, segment order) and warnings. rlog preferred, qlog fallback with a warning."""
  sources, warnings = [], []
  for route in route_names:
    segments, footage = [], None
    for fp in footage_paths:
      segments = utilities.get_segments_in_route(route, fp)
      if segments:
        footage = fp
        break
    if not segments:
      warnings.append(f"{route}: no segments found on device")
      continue
    for seg in segments:
      seg_dir = Path(footage) / seg
      log = next((seg_dir / n for n in LOG_CANDIDATES if (seg_dir / n).is_file()), None)
      if log is None:
        log = next((seg_dir / n for n in QLOG_CANDIDATES if (seg_dir / n).is_file()), None)
        if log is not None:
          warnings.append(f"{seg}: rlog missing, used qlog (lower rate; minutes are approximate)")
      if log is None:
        warnings.append(f"{seg}: no log file")
        continue
      sources.append(lat.RouteLog(route, seg.rsplit("--", 1)[-1], str(log)))
  return sources, warnings


# ---------------------------------------------------------------- worker

def _repo_root():
  return Path(__file__).resolve().parents[3]


def _worker_env():
  env = dict(os.environ)
  root = str(_repo_root())
  extra = ["/usr/local/venv/lib/python3.12/site-packages", str(_repo_root() / "starpilot" / "third_party"), root]
  env["PYTHONPATH"] = ":".join(extra + [p for p in env.get("PYTHONPATH", "").split(":") if p])
  for k in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"):
    env[k] = "1"
  return env


def start_background_analysis(route_names, footage_paths):
  global _PROCESS
  params = _params()
  if params.get_bool("IsOnroad"):
    return False
  route_names = [r for r in route_names if r]
  if not route_names:
    raise ValueError("select at least one route")
  if len(route_names) > ROUTE_LIMIT:
    raise ValueError(f"Lateral Tune analysis is limited to {ROUTE_LIMIT} routes at a time (requested {len(route_names)}).")
  with _LOCK:
    if _PROCESS is not None and _PROCESS.poll() is None:
      return True
    payload = json.dumps({"routes": route_names, "footagePaths": [str(p) for p in footage_paths]})
    command = ["nice", "-n", "19", sys.executable or "python3", str(Path(__file__).resolve()), "worker", payload]
    log_file = open(LOG_PATH, "ab")
    try:
      _PROCESS = subprocess.Popen(command, cwd=str(_repo_root()), env=_worker_env(), stdout=log_file, stderr=log_file,
                                  start_new_session=True)
    except OSError:
      return False
    finally:
      log_file.close()   # the child holds its own copy of the fd
    _write_status({"pid": _PROCESS.pid, "startedAt": time.time(), "running": True, "state": "queued",
                   "routes": route_names, "progress": 0, "total": len(route_names)})
    proc = _PROCESS
  threading.Thread(target=_watch_process_for_onroad, args=(proc,), daemon=True).start()
  return True


def _watch_process_for_onroad(proc):
  params = _params()
  while proc.poll() is None:
    if params.get_bool("IsOnroad"):
      stop_background_analysis("cancelled_onroad")
      return
    time.sleep(ONROAD_POLL_INTERVAL_SECONDS)


def _watch_worker_for_onroad():
  params = _params()
  while True:
    if params.get_bool("IsOnroad"):
      _write_status({**read_status(), "running": False, "state": "cancelled_onroad"})
      os.killpg(os.getpgrp(), signal.SIGTERM)
      os._exit(0)
    time.sleep(ONROAD_POLL_INTERVAL_SECONDS)


def run_worker(payload_json):
  payload = json.loads(payload_json)
  routes, footage = payload["routes"], payload["footagePaths"]
  base = {"pid": os.getpid(), "startedAt": time.time(), "running": True, "routes": routes}
  _write_status({**base, "state": "starting", "progress": 0, "total": 0})
  threading.Thread(target=_watch_worker_for_onroad, daemon=True).start()
  params = _params()
  try:
    sources, warnings = resolve_route_sources(routes, footage)
    if not sources:
      raise ValueError("no readable logs for the selected routes")

    def on_progress(idx, total, src):
      _write_status({**base, "state": "analyzing", "progress": idx, "total": total, "currentSegment": f"{src.route}--{src.segment}"})

    def should_continue():
      return not params.get_bool("IsOnroad")

    trial = lat.analyze_sources(sources, should_continue=should_continue, on_progress=on_progress)
    if params.get_bool("IsOnroad"):
      raise AnalysisCancelled("vehicle went onroad")
    trial["trialId"] = f"lt-{int(time.time())}"
    trial["createdAt"] = time.time()
    trial["warnings"] = list(warnings) + list(trial.get("warnings", []))
    trial["segmentCount"] = len(sources)
    save_trial(trial)
    _write_status({**base, "running": False, "state": "complete", "progress": len(sources), "total": len(sources),
                   "trialId": trial["trialId"]})
  except AnalysisCancelled:
    _write_status({**base, "running": False, "state": "cancelled_onroad"})
  except Exception as e:  # noqa: BLE001 - the state file is the only channel back to Galaxy
    _write_status({**base, "running": False, "state": "failed", "error": f"{type(e).__name__}: {e}"})
    raise


# ---------------------------------------------------------------- apply / revert

def apply_trial(trial_id, force=False):
  params = _params()
  _require_offroad(params, "Applying a trial")
  trial = load_trial(trial_id)
  if trial.get("schemaVersion") != lat.SCHEMA_VERSION or not trial.get("bands"):
    raise RuntimeError("this trial predates the StarPilot speed bands; re-analyze the routes")
  if trial.get("applied"):
    raise RuntimeError("trial is already applied")
  fp_now = current_fingerprint(params)
  if not force and trial["baseline"].get("fingerprint") != fp_now:
    raise RuntimeError("manual lateral tuning changed since these routes were driven (fingerprint mismatch); "
                       "re-analyze, or apply with force")
  prior = {k: lat._param_str(params.get(k)) for k in lat.P_KEYS}
  prior_schedule = current_schedule(params)
  written = lat.build_band_params(trial)
  written_schedule = lat.strip_schedule_p(prior_schedule)
  trial["applied"] = {"at": time.time(), "priorParams": prior, "writtenParams": written, "priorSchedule": prior_schedule,
                      "writtenSchedule": written_schedule, "priorFingerprint": fp_now, "forced": bool(force)}
  save_trial(trial)
  stack = _read_stack()
  stack.append(trial_id)
  _write_stack(stack)
  for k, v in written.items():
    params.put(k, int(v))
  if written_schedule != prior_schedule:
    if written_schedule:
      params.put(SCHEDULE_KEY, written_schedule)
    else:
      params.remove(SCHEDULE_KEY)
  Params(memory=True).put_bool("StarPilotTogglesUpdated", True)
  return {"trial": trial, "written": written, "writtenSchedule": written_schedule, "activeStack": stack}


def revert_trial(trial_id):
  params = _params()
  _require_offroad(params, "Reverting a trial")
  trial = load_trial(trial_id)
  stack = _read_stack()
  if not trial.get("applied") or not stack or stack[-1] != trial_id:
    raise RuntimeError("only the most recently applied trial can be reverted" if stack else "trial is not applied")
  applied = trial["applied"]
  for k, v in (applied.get("priorParams") or {}).items():
    if str(v).strip():
      params.put(k, int(round(float(v))))
    else:
      params.remove(k)
  prior = applied.get("priorSchedule") or ""
  if prior.strip():
    params.put(SCHEDULE_KEY, prior)
  else:
    params.remove(SCHEDULE_KEY)
  Params(memory=True).put_bool("StarPilotTogglesUpdated", True)
  trial["applied"] = None
  save_trial(trial)
  stack.pop()
  _write_stack(stack)
  return {"trial": trial, "restored": applied.get("priorParams") or {}, "restoredSchedule": prior, "activeStack": stack}


def list_workspace():
  ensure_workspace()
  trials = [t for t in (_read_json(p, None) for p in (get_workspace_root() / "trials").glob("*.json")) if t]
  trials.sort(key=lambda t: float(t.get("createdAt") or 0), reverse=True)
  params = _params()
  return {"trials": [_summary(t) for t in trials[:20]], "activeStack": _read_stack(), "status": read_status(),
          "currentSchedule": current_schedule(params), "currentFingerprint": current_fingerprint(params),
          "currentBands": [{"name": n, "lowMph": lo, "highMph": hi, **g}
                           for (n, lo, hi), g in zip(lat.BANDS, current_band_gains(params), strict=True)],
          "routeLimit": ROUTE_LIMIT}


def main(argv):
  if len(argv) >= 3 and argv[1] == "worker":
    run_worker(argv[2])
    return 0
  print("usage: lat_tune_workspace.py worker '<json>'", file=sys.stderr)
  return 2


if __name__ == "__main__":
  sys.exit(main(sys.argv))
