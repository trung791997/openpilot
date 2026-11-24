"""Galaxy toggle constants and capability for running the Bosch-A XCP radar logger as its own process (xcpd)."""

from openpilot.starpilot.car.honda.xcp_radar_assist import capability as _assist_capability

KEY = "AlphaHondaXcpDaemon"
LABEL = "Alpha: XCP Radar Reader (Bosch-A)"
REASON = (
  "Reads extra radar data over XCP in its own process. Off = no XCP reads. Logging only unless XCP Radar Lag Removal "
  "is also on; Lag Removal needs this on. Applies on the next drive."
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
