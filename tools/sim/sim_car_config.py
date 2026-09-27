#!/usr/bin/env python3
"""Carry a real car's configuration into the Mac sim: its CarParams and its settings toggles.

  tools/sim/sim_car_config.py extract --log QLOG_OR_RLOG --out DIR   # once, from any segment of the car
  tools/sim/sim_car_config.py apply DIR                               # before the manager starts

extract writes DIR/carParams.bin (the logged CarParams, carFw and VIN included) and DIR/params.json
(the car's PERSISTENT BOOL/INT/FLOAT params and any param with a default value, from initData).
apply writes those params. The sim bridge then seeds CarParamsCache from carParams.bin, so card
takes the firmware from the cache and picks the same tune as the car (the modified-EPS branch in the
Honda interface keys off the EPS firmware). Keys the sim cannot honour are forced by SIM_OVERRIDES.
"""
import argparse
import json
import os

from openpilot.common.params import Params, ParamKeyFlag, ParamKeyType

# MetaDrive has no radar: with the Bosch-A radar parsed, radard would wait on radar CAN that never comes.
SIM_OVERRIDES = {"BoschARadar": False}
# Device identity, calibration and learned state stay the sim's own.
SKIP_PREFIXES = ("Calibration", "Live", "CarParams", "Dongle", "Api", "Github", "Git", "Hardware", "Update", "Version")


def restorable(params: Params, key: str) -> bool:
  if key.startswith(SKIP_PREFIXES):
    return False
  try:
    kind, flags = params.get_type(key), params.get_key_flag(key)
  except Exception:
    return False  # not a key in this tree
  if kind in (ParamKeyType.BYTES, ParamKeyType.JSON):
    return False
  return (kind in (ParamKeyType.BOOL, ParamKeyType.INT, ParamKeyType.FLOAT) and bool(flags & ParamKeyFlag.PERSISTENT)) or \
         params.get_default_value(key) is not None


def extract(log: str, out: str) -> None:
  from openpilot.tools.lib.logreader import LogReader
  params = Params()
  car_params, entries = None, {}
  for m in LogReader(log):
    if m.which() == "carParams" and car_params is None:
      car_params = m.carParams.as_builder().to_bytes()
    elif m.which() == "initData" and not entries:
      entries = {e.key: e.value.decode("utf-8", "replace") for e in m.initData.params.entries}
  assert car_params is not None and entries, "log has no carParams or initData params"
  kept = {k: v for k, v in sorted(entries.items()) if restorable(params, k)}
  os.makedirs(out, exist_ok=True)
  with open(os.path.join(out, "carParams.bin"), "wb") as f:
    f.write(car_params)
  with open(os.path.join(out, "params.json"), "w") as f:
    json.dump(kept, f, indent=1)
  print(f"wrote {out}: carParams.bin, params.json ({len(kept)} of {len(entries)} params)")


def apply(config_dir: str) -> None:
  params = Params()
  with open(os.path.join(config_dir, "params.json")) as f:
    kept = json.load(f)
  for key, value in kept.items():
    try:
      params.put(key, params.cpp2python(key, value.encode()))
    except Exception as e:
      print(f"skipped {key}: {e}")
  for key, value in SIM_OVERRIDES.items():
    params.put_bool(key, value)
  print(f"applied {len(kept)} params from {config_dir}; sim overrides {SIM_OVERRIDES}")
  seed_learned(params, config_dir)


def seed_learned(params: Params, config_dir: str) -> None:
  # DIR/learned.json (optional, hand-written from the car's own route): {"steerRatio", "angleOffsetAverageDeg",
  # "stiffnessFactor", "rpyCalib": [r, p, y],
  # "wideFromDeviceEuler", "height": [m], "lateralDelay": s}. Seeds paramsd and calibrationd the way the car starts a drive, from its
  # last learned values, instead of from CarParams and an uncalibrated camera. Pair it with SIM_PLANT_SR /
  # SIM_PLANT_OFFSET_DEG / SIM_CAM_RPY so the plant and the camera are the car the learners describe.
  path = os.path.join(config_dir, "learned.json")
  if not os.path.exists(path):
    return
  import cereal.messaging as messaging
  with open(path) as f:
    learned = json.load(f)
  with open(os.path.join(config_dir, "carParams.bin"), "rb") as f:
    cp = f.read()
  params.put("CarParamsPersistent", cp)
  params.put("CarParamsPrevRoute", cp)
  msg = messaging.new_message("liveParameters")
  lp = msg.liveParameters
  lp.valid = True
  lp.steerRatio = float(learned["steerRatio"])
  lp.stiffnessFactor = float(learned.get("stiffnessFactor", 1.0))
  lp.angleOffsetAverageDeg = float(learned.get("angleOffsetAverageDeg", 0.0))
  params.put("LiveParametersV2", msg.to_bytes())
  if "rpyCalib" in learned:
    msg = messaging.new_message("liveCalibration")
    msg.liveCalibration.rpyCalib = [float(x) for x in learned["rpyCalib"]]
    msg.liveCalibration.calStatus = 1  # calibrated
    msg.liveCalibration.validBlocks = 20
    msg.liveCalibration.wideFromDeviceEuler = [float(x) for x in learned.get("wideFromDeviceEuler", [0.0, 0.0, 0.0])]
    msg.liveCalibration.height = [float(x) for x in learned.get("height", [1.22])]
    params.put("CalibrationParams", msg.to_bytes())
  if "lateralDelay" in learned:
    msg = messaging.new_message("liveDelay")
    ld = msg.liveDelay
    ld.lateralDelay = ld.lateralDelayEstimate = float(learned["lateralDelay"])
    ld.status = "estimated"
    ld.validBlocks = 20
    params.put("LiveDelay", msg.to_bytes())
  print(f"seeded learned state from {path}: {learned}")


def main():
  ap = argparse.ArgumentParser()
  sub = ap.add_subparsers(dest="cmd", required=True)
  e = sub.add_parser("extract")
  e.add_argument("--log", required=True)
  e.add_argument("--out", required=True)
  a = sub.add_parser("apply")
  a.add_argument("dir")
  args = ap.parse_args()
  extract(args.log, args.out) if args.cmd == "extract" else apply(args.dir)


if __name__ == "__main__":
  main()
