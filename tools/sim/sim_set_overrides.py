#!/usr/bin/env python3
"""Applies key=value param overrides to openpilot params under OPENPILOT_PREFIX."""
from __future__ import annotations

import argparse
import os


def set_overrides(prefix: str, overrides: list[str]) -> list[str]:
  os.environ["OPENPILOT_PREFIX"] = prefix
  from openpilot.common.params import ParamKeyType, Params

  params = Params()
  set_keys = []
  for pair in overrides:
    key, sep, val = pair.partition("=")
    if not sep:
      continue
    key = key.strip()
    val_clean = val.strip()
    if val_clean.lower() == "true":
      params.put_bool(key, True)
    elif val_clean.lower() == "false":
      params.put_bool(key, False)
    else:
      # put() requires the value's Python type to match the key's declared ParamKeyType
      # (see params_pyx.pyx PYTHON_2_CPP) -- a bare str only satisfies STRING-typed keys, so
      # int/float/bool/bytes-typed keys (e.g. LatIScaleStandard is INT) need coercion first.
      key_type = params.get_type(key)
      if key_type == ParamKeyType.INT:
        params.put(key, int(float(val_clean)))
      elif key_type == ParamKeyType.FLOAT:
        params.put(key, float(val_clean))
      elif key_type == ParamKeyType.BOOL:
        params.put_bool(key, val_clean.lower() not in ("", "0", "false", "none"))
      elif key_type == ParamKeyType.BYTES:
        params.put(key, val_clean.encode())
      else:
        params.put(key, val_clean)
    print(key)
    set_keys.append(key)
  return set_keys


def main() -> None:
  parser = argparse.ArgumentParser(description="Set openpilot param overrides for simulation")
  parser.add_argument("--prefix", required=True, help="OPENPILOT_PREFIX environment value")
  parser.add_argument("overrides", nargs="+", help="KEY=VALUE override pairs")
  args = parser.parse_args()

  set_overrides(args.prefix, args.overrides)


if __name__ == "__main__":
  main()
