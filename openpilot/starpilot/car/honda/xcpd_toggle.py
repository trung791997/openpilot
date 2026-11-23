"""Galaxy toggle constants and capability for running the Bosch-A XCP radar logger as its own process (xcpd)."""

from openpilot.starpilot.car.honda.xcp_radar_assist import capability as _assist_capability

KEY = "AlphaHondaXcpDaemon"
LABEL = "Alpha: XCP Radar Daemon (Bosch-A)"
REASON = (
  "Runs the XCP radar reader as its own process with a dedicated send path, so its requests no longer wait on the "
  "car-controls loop. Off keeps it inside card. Same reads, faster cycle; logging-only unless XCP Radar Lag Removal is also on."
)


def capability(cp, params=None) -> tuple | None:
  """Same Bosch-A car set as XCP Radar Lag Removal, additionally excluding radarless and CAN FD Honda Bosch."""
  binding = _assist_capability(cp, params)
  if binding is None:
    return None
  try:
    from opendbc.car.honda.values import HondaFlags
    if int(cp.flags) & int(HondaFlags.BOSCH_RADARLESS | HondaFlags.BOSCH_CANFD):
      return None
  except (AttributeError, TypeError, ValueError, OverflowError):
    return None
  return binding
