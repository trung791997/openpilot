"""D-117: the nearest Mapbox-flagged traffic control (signal, stop sign, railway crossing) along the route."""
from __future__ import annotations


def control_ahead(route, along_m: float, lookahead_m: float) -> dict | None:
  """D-117: the nearest Mapbox-flagged traffic control within lookahead_m: {distance, kind}."""
  if route is None:
    return None
  for step in route.steps:
    for junction_m, kind in getattr(step, "junction_controls", ()):
      distance = junction_m - along_m
      if distance <= 0.0:
        continue
      if distance > lookahead_m:
        return None
      return {"distance": distance, "kind": kind}
  return None
