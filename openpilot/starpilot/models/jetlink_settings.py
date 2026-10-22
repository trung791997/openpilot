"""Parked Jetlink selection and its read-only provisioning status."""
from __future__ import annotations

from openpilot.starpilot.models.jetlink_adapter import UNSUPPORTED

MODES = ("off", "usb", "ios")

# jetlink's Status.phase in Galaxy's words, where they differ
PHASES = {"waiting": "waiting for disengagement", "reconnecting": "fallback / reconnecting"}


class JetlinkSettings:
  def __init__(self, *, params=None, link=None, device_type=None, chestnut=None):
    self._params = params
    self._link = link
    self._device_type = device_type
    self._chestnut = chestnut

  def params(self):
    if self._params is None:
      from openpilot.common.params import Params
      self._params = Params()
    return self._params

  def snapshot(self):
    try:
      return self._snapshot()
    except (OSError, RuntimeError, ValueError, ImportError):
      return {"mode": "off", "supported": False, "chestnut": False, "enabled": False,
              "present": False, "prepared": False, "active": False, "runtimeState": "unavailable", "state": "unavailable",
              "reason": "Jetlink status is unavailable; the local model remains in use.",
              "progress": "", "progressFrac": None, "model": None, "defaultModel": None, "selectedModel": None,
              "models": [], "transport": "USB", "chargePhone": False}

  def _snapshot(self):
    from openpilot.starpilot.models.jetlink_adapter import BIG_MODEL_KEY, adapter, hooks, models, supported_device
    supported = supported_device(self._device_type() if self._device_type is not None else None)
    chestnut = self._chestnut() if self._chestnut is not None else adapter().chestnut_present()
    params = self.params()
    raw = params.get("JetlinkMode")
    mode = MODES[raw] if type(raw) is int and 0 <= raw < len(MODES) else "off"
    link = self._link() if self._link is not None else hooks()
    status = link.status()
    reason = "" if supported else UNSUPPORTED + "."
    if mode != "off" and status is None:
      reason = reason or "Jetlink is unavailable; the local model remains in use."
    if status is not None and status.reason:
      reason = str(status.reason)
    usable = mode != "off" and supported and status is not None
    # the big model drives modeld's frames now, by modeld's runtime record
    active = usable and not chestnut and status.active
    state = "off" if mode == "off" else "unavailable" if reason else PHASES.get(status.phase, status.phase)
    progress = status.progress if status is not None and isinstance(status.progress, dict) else {}
    frac = progress.get("frac")
    # the owner's pick, by ref; None is jetlink's default (Status.default_model names it)
    pick = params.get(BIG_MODEL_KEY)
    selected = pick.get("ref") if isinstance(pick, dict) and pick.get("ref") else None
    return {"mode": mode, "supported": supported, "chestnut": chestnut, "enabled": bool(usable and status.enabled),
            "present": bool(status and status.present), "prepared": bool(usable and status.ready),
            "active": active, "runtimeState": (status.running if status is not None else None) or "unavailable",
            "activeArtifactSha256": status.running_sha256 if active else None,
            "activeCheckpoint": status.running_checkpoint if active else None,
            "state": state, "reason": reason, "progress": str(progress.get("msg", "")),
            "progressFrac": float(frac) if isinstance(frac, (int, float)) and not isinstance(frac, bool) else None,
            # Status.model: the pick, else jetlink's default; active_model what drives once ready, else the standin
            "model": status.model if status is not None else None,
            "activeModel": status.active_model if status is not None else None,
            "defaultModel": status.default_model if status is not None else None,
            "selectedModel": selected, "models": models(params, link),
            "transport": status.transport if status is not None else "USB",
            "chargePhone": params.get_bool("JetlinkChargePhone")}

  @staticmethod
  def _wired_session_running():
    """Wired Android Auto holds the comma's USB-C port while a session runs."""
    try:
      from openpilot.starpilot.system.android_auto.protocol import AndroidAutoClient
      status = AndroidAutoClient().status()
      return status.get("connection") == "wired" and bool(status.get("running"))
    except Exception:
      return False

  def configure(self, payload):
    if not payload or set(payload) - {"mode", "chargePhone", "bigModel"}:
      raise ValueError("Invalid Jetlink settings")
    if "mode" in payload and payload["mode"] not in MODES:
      raise ValueError("Choose Off, USB, or iPhone")
    if "chargePhone" in payload and type(payload["chargePhone"]) is not bool:
      raise ValueError("Invalid phone charging setting")
    if "bigModel" in payload and payload["bigModel"] is not None and type(payload["bigModel"]) is not str:
      raise ValueError("Invalid Jetlink model")
    view = self.snapshot()
    if payload.get("mode", "off") != "off" and not view["supported"]:
      raise ValueError("Jetlink requires a comma 3X or comma 4")
    if payload.get("mode", "off") != "off" and view["chestnut"]:
      raise ValueError("Chestnut is connected; Jetlink cannot run alongside it")
    if payload.get("mode", "off") != "off" and self._wired_session_running():
      raise ValueError("Disconnect wired Android Auto before enabling Jetlink")
    params = self.params()
    if "bigModel" in payload:
      from openpilot.starpilot.models.jetlink_adapter import hooks, select_model
      link = self._link() if self._link is not None else hooks()
      if not select_model(payload["bigModel"] or None, params, link):
        raise ValueError("That model is not in the Jetlink catalog")
    if "chargePhone" in payload:
      params.put_bool("JetlinkChargePhone", payload["chargePhone"], block=True)
    if "mode" in payload:
      params.put("JetlinkMode", MODES.index(payload["mode"]), block=True)
    return {"message": "Jetlink settings saved. Check connection status before driving."}
