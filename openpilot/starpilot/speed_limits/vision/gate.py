"""Whether the vision speed-limit reader should run. Light imports only: the manager calls this every cycle."""

from __future__ import annotations

import os


def wanted(params, cp, environment=None) -> bool:
  """Speed Limit Controller with the Vision source, on a car whose display can show the limit."""
  from openpilot.starpilot.feature_runtime import enabled
  environment = os.environ if environment is None else environment
  return enabled(params, cp, 'slc', environment) and enabled(params, cp, 'vision', environment)
