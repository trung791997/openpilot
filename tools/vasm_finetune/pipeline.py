"""One-command, resumable V-ASM fine-tune: the run12 recipe as stages, the way upstream's train_pipeline.py chains its own.

  cd ~/oprad-scratch/vasm
  PYTHONPATH=/tmp/oppath venv/bin/python <repo>/tools/vasm_finetune/pipeline.py --crops crops12 --run run13
  ... --dry-run          print the commands only
  ... --status           print what is done, and the labelling backlog

Stages, each skipped when its output already exists (re-running the same command resumes after a crash):
  check     hash manifest.csv, labels.json, the hard-negative list and the queue. A run is tied to that data: if any
            of it changed since the run started, the pipeline refuses and asks for a new --run.
  train     train.py once per seed (owner labels only, so agent proposals never become training labels)
  soup      soup.py: average the seeds' state dicts per epoch
  select    select_queue.py --with-val: which (soup, on) qualify on the queue + validation groups
  pick      pick.py: the held-out road-route rule; NONE stops the run before test
  baseline  benchmark.py on test with the shipped model, cached per test-set hash (shared by runs)
  test      benchmark.py on test with the pick, ONCE. An existing result is never overwritten, so test is judged once.
  gate      benchmark.py --compare baseline test: missed events, false-occupied s/h, latency
  package   only when the gate passes: <run>/deliver/vasm.onnx + deploy.json (sha256, on, off, data hashes, recipe)

Not automated, on purpose: extracting crops (Docker, see README), labelling (labeler.py; a human confirms every
label), and deploying to the car (owner's call). Thresholds come from validation and the road route, never from test.
"""

import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
STAGES = ("check", "train", "soup", "select", "pick", "baseline", "test", "gate", "package")

# run12 (ft/run12.sh, 2026-10-14). Keys are train.py flags; True = a bare flag. A recipe with "models" (ONNX paths
# under --work, e.g. ul_scratch.py exports) skips train and soup and judges those models instead.
RECIPE = {
  "seeds": [9, 10, 11],
  "rule": "highest",
  "train": {"epochs": 4, "lr": 3e-5, "truck-weight": 1, "balance": 0.5, "owner-only": True, "hardneg-gap": 1.0,
            "onset-weight": 3},
}


def sha(path):
  if not os.path.exists(path):
    return None
  h = hashlib.sha256()
  with open(path, "rb") as f:
    for b in iter(lambda: f.read(1 << 20), b""):
      h.update(b)
  return h.hexdigest()


class Pipeline:
  def __init__(self, args):
    self.a = args
    self.work = os.path.abspath(args.work)
    self.crops = os.path.join(self.work, args.crops)
    self.ft = os.path.join(self.work, "ft")
    self.run = os.path.join(self.ft, args.run)
    self.hardneg = os.path.join(self.work, args.hardneg)
    self.queue = os.path.join(self.crops, "queue.json")
    self.state_path = os.path.join(self.run, "pipeline.json")
    self.state = json.load(open(self.state_path)) if os.path.exists(self.state_path) else {}
    if args.recipe:
      recipe = json.load(open(args.recipe))
    else:
      recipe = self.state.get("recipe", RECIPE)
    if "recipe" in self.state and self.state["recipe"] != recipe:
      sys.exit(f"{args.run} was started with a different recipe; start a new --run")
    self.recipe = recipe
    self.base = os.path.join(self.ft, "vasm_converted.pt")
    self.env = {**os.environ, "PYTORCH_ENABLE_MPS_FALLBACK": "1"}

  def save(self):
    if self.a.dry_run:
      return
    os.makedirs(self.run, exist_ok=True)
    tmp = self.state_path + ".tmp"
    with open(tmp, "w") as f:
      json.dump(self.state, f, indent=1)
    os.replace(tmp, self.state_path)

  def sh(self, stage, cmd):
    print("$", " ".join(cmd), flush=True)
    if self.a.dry_run:
      return ""
    os.makedirs(os.path.join(self.run, "logs"), exist_ok=True)
    out = []
    with open(os.path.join(self.run, "logs", f"{stage}.log"), "a") as log:
      log.write(f"\n# {time.strftime('%Y-%m-%d %H:%M:%S')} $ {' '.join(cmd)}\n")
      p = subprocess.Popen(cmd, cwd=self.work, env=self.env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
      for line in p.stdout:
        sys.stdout.write(line)
        log.write(line)
        out.append(line)
      rc = p.wait()
    if rc not in (0,) and stage != "gate":
      sys.exit(f"stage {stage} failed (exit {rc}); fix and re-run the same command to resume")
    self.rc = rc
    return "".join(out)

  def py(self, script, *args):
    return [sys.executable, os.path.join(HERE, script), *map(str, args)]

  def done(self, stage, **info):
    self.state.setdefault("stages", {})[stage] = {"at": time.strftime("%Y-%m-%d %H:%M:%S"), **info}
    self.save()

  # --- stages ---------------------------------------------------------------------------------------------------
  def check(self):
    data = {"manifest": sha(os.path.join(self.crops, "manifest.csv")), "labels": sha(os.path.join(self.crops, "labels.json")),
            "hardneg": sha(self.hardneg), "queue": sha(self.queue)}
    for k in ("manifest", "labels"):
      if data[k] is None:
        sys.exit(f"{self.crops} has no {k}")
    if not os.path.exists(self.base):
      sys.exit(f"missing {self.base} (convert.py)")
    if "data" in self.state and self.state["data"] != data:
      changed = [k for k in data if data[k] != self.state["data"].get(k)]
      sys.exit(f"{', '.join(changed)} changed since {self.a.run} started; start a new --run (results would mix data)")
    labels = json.load(open(os.path.join(self.crops, "labels.json")))
    owner = sum("by" not in e and e.get("label") in (0, 1, 2) for e in labels.values())
    print(f"data hashes ok; {owner} owner labels")
    self.state.update({"crops": self.a.crops, "recipe": self.recipe, "data": data})
    self.done("check", owner_labels=owner)

  def epochs(self):
    return range(self.recipe["train"]["epochs"])

  def train(self):
    if "models" in self.recipe:
      print("recipe gives models: no training")
      return self.done("train")
    last = self.recipe["train"]["epochs"] - 1
    for s in self.recipe["seeds"]:
      out = os.path.join(self.run, f"s{s}")
      if os.path.exists(os.path.join(out, f"ep{last}.pt")) and os.path.exists(os.path.join(out, "vasm_ft.onnx")):
        print(f"seed {s}: done")
        continue
      flags = []
      for k, v in self.recipe["train"].items():
        flags += [f"--{k}"] if v is True else [f"--{k}", v]
      if os.path.exists(self.hardneg):
        flags += ["--hardneg", self.hardneg]
      if os.path.exists(self.queue):
        flags += ["--exclude-queue", self.queue]
      self.sh("train", self.py("train.py", "--crops", self.crops, "--base", self.base, "--out", out, *flags,
                               "--save-epochs", "--seed", s))
    self.done("train")

  def soups(self):
    if "models" in self.recipe:
      return [os.path.join(self.work, m) for m in self.recipe["models"]]
    return [os.path.join(self.run, f"soup_ep{e}.onnx") for e in self.epochs()]

  def soup(self):
    if "models" in self.recipe:
      return self.done("soup")
    for e, out in zip(self.epochs(), self.soups()):
      if os.path.exists(out):
        continue
      states = [os.path.join(self.run, f"s{s}", f"ep{e}.pt") for s in self.recipe["seeds"]]
      self.sh("soup", self.py("soup.py", "--base", self.base, "--out", out, *states))
    self.done("soup")

  def select(self):
    out = os.path.join(self.run, "select.json")
    if not os.path.exists(out):
      self.sh("select", self.py("select_queue.py", "--crops", self.crops, "--with-val", "--models", *self.soups(), "--out", out))
    self.done("select")

  def pick(self):
    out = os.path.join(self.run, "pick.json")
    if not os.path.exists(out):
      self.sh("pick", self.py("pick.py", os.path.join(self.run, "select.json"), "--road-crops", os.path.join(self.work, self.a.road_crops),
                              "--rule", self.recipe["rule"], "--out", out))
    if self.a.dry_run:
      return True
    p = json.load(open(out))["pick"]
    self.done("pick", pick=p and {"model": p["model"], "on": p["on"], "off": p["off"]})
    if not p:
      print("NONE: no candidate passes the road route; not testing")
    return bool(p)

  def baseline_path(self):
    d = self.state.get("data", {})
    return os.path.join(self.ft, f"base_{(d.get('manifest') or 'x')[:8]}_{(d.get('labels') or 'x')[:8]}.json")

  def baseline(self):
    out = self.baseline_path()
    if not os.path.exists(out):
      self.sh("baseline", self.py("benchmark.py", "--crops", self.crops, "--out", out))
    self.done("baseline", path=out)

  def test(self):
    out = os.path.join(self.run, "test.json")
    if os.path.exists(out):
      print("test already judged once:", out)
    else:
      p = self.state["stages"]["pick"]["pick"] if not self.a.dry_run else {"model": "<pick>", "on": "<on>", "off": "<off>"}
      self.sh("test", self.py("benchmark.py", "--crops", self.crops, "--model", p["model"], "--on", p["on"], "--off", p["off"], "--out", out))
    self.done("test")

  def gate(self):
    self.sh("gate", self.py("benchmark.py", "--compare", self.baseline_path(), os.path.join(self.run, "test.json")))
    if self.a.dry_run:
      return True
    ok = self.rc == 0
    self.done("gate", passed=ok)
    print("GATE", "passed" if ok else "FAILED: nothing packaged")
    return ok

  def package(self):
    if self.a.dry_run:
      print("(copy the pick to deliver/vasm.onnx, write deploy.json)")
      return
    p = self.state["stages"]["pick"]["pick"]
    d = os.path.join(self.run, "deliver")
    os.makedirs(d, exist_ok=True)
    dst = os.path.join(d, "vasm.onnx")
    shutil.copyfile(os.path.join(self.work, p["model"]) if not os.path.isabs(p["model"]) else p["model"], dst)
    info = {"run": self.a.run, "model_from": p["model"], "sha256": sha(dst), "on": p["on"], "off": p["off"],
            "data": self.state["data"], "recipe": self.recipe, "test": os.path.join(self.run, "test.json"),
            "note": "replay/static evidence only; deploying to the car is the owner's call"}
    with open(os.path.join(d, "deploy.json"), "w") as f:
      json.dump(info, f, indent=1)
    self.done("package", path=d, sha256=info["sha256"])
    print("packaged", d, "(not deployed)")

  def go(self):
    order = STAGES[: STAGES.index(self.a.until) + 1] if self.a.until else STAGES
    for st in order:
      print(f"\n== {st}", flush=True)
      if st == "check":
        self.check()
      elif st == "pick":
        if not self.pick():
          return
      elif st == "gate":
        if not self.gate():
          return
      else:
        getattr(self, st)()

  def status(self):
    for st in STAGES:
      s = self.state.get("stages", {}).get(st)
      print(f"{st:9s}", "-" if not s else json.dumps(s))
    labels = json.load(open(os.path.join(self.crops, "labels.json")))
    n = sum(1 for _ in open(os.path.join(self.crops, "manifest.csv"))) - 1
    own = sum("by" not in e and e.get("label") in (0, 1, 2) for e in labels.values())
    agent = sum("by" in e for e in labels.values())
    print(f"\n{self.a.crops}: {n} crops, {own} owner labels, {agent} agent proposals awaiting the owner")


def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("--work", default=".", help="scratch root holding <crops>/, ft/vasm_converted.pt, hardneg/")
  ap.add_argument("--crops", required=True, help="crops dir under --work (manifest.csv + labels.json)")
  ap.add_argument("--run", required=True, help="run name: outputs go to ft/<run>/")
  ap.add_argument("--recipe", default=None, help="JSON recipe (default: run12's); fixed for the life of a run")
  ap.add_argument("--hardneg", default="hardneg/accepted.json")
  ap.add_argument("--road-crops", default="cropsroad", help="held-out road route crops for pick.py")
  ap.add_argument("--until", choices=STAGES, default=None, help="stop after this stage")
  ap.add_argument("--dry-run", action="store_true")
  ap.add_argument("--status", action="store_true")
  args = ap.parse_args()
  p = Pipeline(args)
  if args.status:
    p.status()
    return
  lock = os.path.join(p.run, ".lock")
  if not args.dry_run:
    os.makedirs(p.run, exist_ok=True)
    try:
      fd = os.open(lock, os.O_CREAT | os.O_EXCL | os.O_WRONLY)
      os.write(fd, str(os.getpid()).encode())
      os.close(fd)
    except FileExistsError:
      sys.exit(f"{lock} exists: another pipeline is running this run (delete it if that process is gone)")
  try:
    p.go()
  finally:
    if not args.dry_run and os.path.exists(lock):
      os.remove(lock)


if __name__ == "__main__":
  main()
