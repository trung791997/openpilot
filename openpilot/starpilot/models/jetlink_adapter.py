"""StarPilot's adapter to jetlink's openpilot layer, and the guard every Jetlink hook line calls.

The top level imports the standard library only: manager builds its process list from this module and the
gadget owner (jetlinkd) runs it.
"""
from __future__ import annotations

import functools
import os
import time

API = 3  # the jetlink.openpilot API this adapter is written to
UNSUPPORTED = "Jetlink requires a comma 3X or comma 4"
# AOL steers on its own axes, and a paused AOL is still armed: only its intent says so
AOL_SERVICES = ("aolIntentWire", "aolAxisState")
AOL_AXES = ("desiredLateral", "desiredLongitudinal", "lateralActive", "longitudinalActive")
# how long after AOL's reader is built an intent not yet heard counts as armed: its first polls run before the
# subscription connects, though card publishes the intent at 100 Hz
AOL_GRACE_S = 1.
# CatalogModelState's per-frame camera input, not history
FRAME_INPUTS = ("mapped_gpu_image",)


def supported_device(device_type=None) -> bool:
  if device_type is None:
    from openpilot.common.hardware import HARDWARE
    device_type = HARDWARE.get_device_type()
  return device_type in ("tizi", "mici")


def aol_in_control(intent, alive, axes, waiting=False) -> bool:
  """AOL armed (paused or not) or on its axes: the half of the swap's in-control rule that is not openpilot's,
  shared by engaged() and selfdrived. `intent` is aolIntentWire's message, None until heard; `alive` that it still
  is; `axes` the AOL_AXES flags, empty when unknown; `waiting` that its reader is younger than AOL_GRACE_S, when an
  intent not heard yet may be armed."""
  if intent is None:
    if waiting:
      return True
  elif not alive:
    return True
  else:
    from openpilot.starpilot.aol.wire import decode_intent
    decoded = decode_intent(intent)
    if decoded is None or decoded.allowedLatch or decoded.lateralArmed:
      return True
  return any(axes)


BIG_MODEL_KEY = "JetlinkBigModel"
CATALOG_KEY = "JetlinkCatalog"


def default_bundle() -> dict:
  """jetlink's default big model as a catalog bundle, at the selector version Models.catalog() keeps."""
  from jetlink.registry.catalog import DEFAULT_BIG_MODEL_NAME, DEFAULT_BIG_MODEL_REF, REQUIRED_SELECTOR_VERSION
  return {"ref": DEFAULT_BIG_MODEL_REF, "display_name": DEFAULT_BIG_MODEL_NAME,
          "minimum_selector_version": REQUIRED_SELECTOR_VERSION, "index": 0}


def with_default_bundle(catalog):
  """The stored catalog, or one listing only jetlink's default while none with bundles is stored."""
  if isinstance(catalog, dict) and catalog.get("bundles"):
    return catalog
  return {**(catalog if isinstance(catalog, dict) else {}), "bundles": [default_bundle()]}


@functools.cache
def _adapter_class():
  from jetlink.openpilot.interface import Keys
  from jetlink.openpilot.stock import Stock
  from jetlink.registry.catalog import REQUIRED_SELECTOR_VERSION

  class StarPilotAdapter(Stock):
    # The big-model slot and catalog are StarPilot's own JSON params (params_keys.h): the slot is the owner's pick,
    # {ref, displayName}, unset for jetlink's default; the catalog is refresh_catalog()'s {bundles}
    KEYS = Keys(link="JetlinkMode", offroad="IsOffroad", progress="JetlinkProgress", spec="JetlinkSpec",
                pointers="JetlinkPointers", big_model=BIG_MODEL_KEY, catalog=CATALOG_KEY,
                charge_phone="JetlinkChargePhone")
    # Models.catalog() keeps only bundles at this selector version; merge_catalogs() retypes newer ones to it
    CATALOG_SELECTOR = REQUIRED_SELECTOR_VERSION
    IN_CONTROL_SERVICES = ("carControl", "selfdriveState")
    _aol = None
    _aol_since = 0.

    def get(self, key):
      # With Keys.catalog set, Models.default_model() answers None from an empty catalog and nothing provisions
      # until refresh_catalog() has fetched one (jetlink/openpilot/models.py default_model). Until then the
      # catalog reads as jetlink's own default, the model it ran with no catalog key at all
      value = super().get(key)
      return with_default_bundle(value) if key == CATALOG_KEY else value

    def model_root(self):
      from openpilot.starpilot.models.manager import ROOT
      return ROOT

    def engaged(self, sm) -> bool:
      # openpilot, or aol_in_control(); Stock.in_control() answers True if this raises. AOL's reader is built and
      # polled on every call, openpilot engaged or not, so it has heard AOL by the time an offer or a fall asks,
      # rather than answering from its grace
      if self._aol is None:
        from openpilot.cereal import messaging
        self._aol, self._aol_since = messaging.SubMaster(list(AOL_SERVICES)), time.monotonic()
      aol = self._aol
      aol.update(0)
      if super().engaged(sm):
        return True
      heard = aol.seen["aolIntentWire"]
      axes = aol["aolAxisState"] if aol.seen["aolAxisState"] and aol.alive["aolAxisState"] else None
      return aol_in_control(aol["aolIntentWire"] if heard else None, aol.alive["aolIntentWire"],
                            () if axes is None else [getattr(axes, axis) for axis in AOL_AXES],
                            waiting=time.monotonic() - self._aol_since < AOL_GRACE_S)

    def model_warp(self, small, cam_w, cam_h, model_w, model_h):
      # None under legacy9d, whose tinygrad is the one Jetlink's stock pickle and its own warp are built for; any
      # other runtime cannot load that pickle, so it sends the small model's own warp (jetlink_warp). A failure
      # raises, and Jetlink logs it and keeps the small model
      from openpilot.starpilot.models.runner import CatalogModelState
      from openpilot.starpilot.models.runtime_bridge.runtime import selected_legacy
      if selected_legacy():
        return None
      if not isinstance(small, CatalogModelState):
        raise RuntimeError(f"the small model is a {type(small).__name__}, not a catalog model")
      from openpilot.starpilot.models.jetlink_warp import CatalogWarp
      warp = CatalogWarp(small, (model_w, model_h))
      self.log.warning("jetlink: the link sends the small model's warp (%s, %d bytes a frame, %s)",
                       small.camera_warp_descriptor.output_shape, warp.output.nbytes, warp.sending)
      return warp

    def small_reset(self, small):
      # CatalogModelState's history zeroed in place: GPU queues, numpy inputs (prev_feat, packed inputs) and
      # the desire edge. Never _reset_state(), which makes tensors the run JIT was not captured with.
      # Not jetlink's reset_queues either: its TinyJit clear() compiles a fresh QCOM kernel, which the driving
      # fresh-compile guard refuses ("Unexpected fresh QCOM submission"), so every attach failed. The queues
      # are CPU-mapped, so they are zeroed through their host views with no compile and no submission.
      from openpilot.starpilot.models.runner import CatalogModelState
      if not isinstance(small, CatalogModelState):
        return None
      from tinygrad import Device
      import numpy as np

      queues = [q for name, q in small.input_queues.items() if q.device != "NPY" and name not in FRAME_INPUTS]
      buffers = [q.uop.base.buffer for q in queues]
      views = []
      for buffer in buffers:
        if not buffer.is_allocated():
          raise ValueError("Small model history has no allocated host mapping")
        if hasattr(buffer, "get_storage"):
          if not hasattr(buffer.get_storage().host, "mv"):
            raise ValueError("Small model history has no allocated host mapping")
          view = buffer.as_memoryview(allow_zero_copy=True)
        else:  # legacyf6's Buffer has no get_storage; its QCOM allocator maps the buffer directly
          view = buffer.allocator._as_buffer(buffer._buf) if hasattr(buffer.allocator, "_as_buffer") else None
          if view is None:
            raise ValueError("Small model history has no allocated host mapping")
        if view.readonly or view.nbytes != buffer.nbytes:
          raise ValueError("Small model history mapping is not writable or complete")
        views.append(np.frombuffer(view, dtype=np.uint8))
      devices = tuple(Device[name] for name in dict.fromkeys(q.device for q in queues))
      arrays = tuple(small.npy.values())

      def reset():
        # The local history is idle at a handover. Keep the existing buffers
        # and synchronize their device before zeroing their mapped storage.
        for device in devices:
          device.synchronize()
        for view in views:
          view.fill(0)
        small.prev_desire.fill(0)
        for array in arrays:
          array.fill(0)

      return reset

  return StarPilotAdapter


def _host_view(q):
  # the zero-copy host bytes of a realized queue buffer; refuse rather than fall back to a kernel or a copy
  import numpy as np
  buf = q.uop.base.buffer
  if not buf.is_allocated():
    raise RuntimeError(f"small_reset: {q.device} queue buffer is not allocated")
  if hasattr(buf, "_host_mv"):  # 9d runtime
    mv = buf._host_mv()
  elif hasattr(buf.allocator, "_as_buffer"):  # f6 runtime
    mv = buf.allocator._as_buffer(buf._buf)
  else:
    mv = None
  if mv is None:
    raise RuntimeError(f"small_reset: {q.device} queue buffer has no host mapping; refusing to compile a reset")
  view = np.frombuffer(mv, dtype=np.uint8)
  if view.nbytes != buf.nbytes:
    raise RuntimeError(f"small_reset: host view is {view.nbytes} bytes, buffer is {buf.nbytes}")
  if not view.flags.writeable:
    raise RuntimeError(f"small_reset: {q.device} host view is read-only")
  return buf, view


def host_zero_reset(queues, arrays, prev_desire):
  # captured once, at prepare; the returned reset() compiles, allocates and submits nothing
  from tinygrad import Device
  views = [_host_view(q) for q in queues]
  devices = sorted({buf.device for buf, _ in views})

  def reset():
    for device in devices:  # no GPU work may still be writing the history being cleared
      Device[device].synchronize()
    for _, view in views:
      view.fill(0)
    for array in arrays:
      array.fill(0)
    prev_desire.fill(0)
  return reset


def adapter():
  return _adapter_class()()


def owner_config():
  return _adapter_class().owner_config()


def main():
  _adapter_class().main()


def link_on() -> bool:
  """The Jetlink setting, read as a file, so it works with no jetlink at all."""
  prefix = os.environ.get("OPENPILOT_PREFIX", "")
  root = os.environ.get("PARAMS_ROOT") or ("/data/params" if os.path.isfile("/AGNOS") else
                                           os.path.expanduser(f"~/.comma{prefix}/params"))
  try:
    with open(os.path.join(root, os.environ.get("OPENPILOT_PREFIX", "d"), "JetlinkMode")) as f:
      return f.read().strip() in ("1", "2")
  except OSError:
    return False


def coherent_warp():
  """What modeld's small model moves its warp's output with before warmup links the warp: Jetlink's
  make_coherent, with Jetlink on and off legacy9d, where the link sends that output (jetlink_warp). Else None."""
  if not link_on():
    return None
  from openpilot.starpilot.models.runtime_bridge.runtime import selected_legacy
  if selected_legacy():
    return None
  try:
    from jetlink.openpilot.warp import make_coherent
  except Exception:
    return None
  return make_coherent


def _cloudlog():
  from openpilot.common.swaglog import cloudlog
  return cloudlog


@functools.cache
def hooks():
  """What every hook line calls, one per process. Never raises."""
  try:
    from jetlink.openpilot.hooks import Hooks, Off
  except Exception as e:
    return _NoJetlink(f"jetlink is missing: {e}")
  try:
    if not supported_device():
      return Off(UNSUPPORTED, link_on=link_on)
    return Hooks(API, adapter, link_on=link_on, log=_cloudlog)
  except Exception as e:
    return Off(str(e), link_on=link_on)


class _NoJetlink:
  """The hooks with no usable jetlink checkout, each answering link off. Only these are reached: modeld stops at
  prepare()."""
  def __init__(self, why):
    self._why = why

  def should_run(self, started, params, CP):
    return False

  def status(self):
    return None

  def reason(self):
    return self._why if link_on() else None

  def prepare(self, chestnut):
    return False

  def shutdown_step(self, should_shutdown, reason=""):
    return should_shutdown

  def runtime(self):
    return None

  def swap_events(self, tick_s):
    return None

  def model_state(self, ref):
    return None

  def should_extend_catalog(self):
    return False

  def extend_catalog(self, catalog):
    return catalog


def _guarded(default):
  """The model picker's calls never raise into the model manager or Galaxy: logged, then `default`."""
  def wrap(fn):
    @functools.wraps(fn)
    def call(*args, **kwargs):
      try:
        return fn(*args, **kwargs)
      except Exception:
        try:
          _cloudlog().exception("jetlink: %s() failed", fn.__name__)
        except Exception:
          pass
        return default() if callable(default) else default
    return call
  return wrap


def _params(store):
  if store is not None:
    return store
  from openpilot.common.params import Params
  return Params()


@_guarded(False)
def refresh_catalog(store=None, link=None) -> bool:
  """The model manager's refresh: fetch the big-model catalogs and keep them in CATALOG_KEY. Network; a failed
  fetch keeps the last catalog (Models.big_catalog). Skipped with a chestnut fitted (should_extend_catalog). True
  when the stored catalog changed. Reads the raw param, never the default-bundle stand-in."""
  link = hooks() if link is None else link
  if not link.should_extend_catalog():
    return False
  store = _params(store)
  cached = store.get(CATALOG_KEY)
  cached = cached if isinstance(cached, dict) else {}
  merged = link.extend_catalog(cached)
  if not isinstance(merged, dict) or not merged.get("bundles") or merged == cached:
    return False
  store.put(CATALOG_KEY, merged, block=True)
  return True


@_guarded(list)
def models(store=None, link=None) -> list[dict]:
  """The picker's rows, newest first: {ref, name, state, selected}. The same filter as Models.catalog() (40-hex
  ref, selector version REQUIRED_SELECTOR_VERSION, index descending), deduplicated. state is jetlink's
  model_state(): 'ready' (built on the Jetson), 'downloaded' (on the comma) or None. Records only, no network."""
  from jetlink.registry.catalog import REQUIRED_SELECTOR_VERSION, is_ref
  store = _params(store)
  link = hooks() if link is None else link
  bundles = with_default_bundle(store.get(CATALOG_KEY))["bundles"]
  pick = store.get(BIG_MODEL_KEY)
  picked = pick.get("ref") if isinstance(pick, dict) else None
  rows: list[dict] = []
  for b in sorted((b for b in bundles if isinstance(b, dict)), key=lambda b: int(b.get("index", 0) or 0), reverse=True):
    ref = b.get("ref")
    try:
      selector = int(b.get("minimum_selector_version", 0))
    except (TypeError, ValueError):
      continue
    if not is_ref(ref) or selector != REQUIRED_SELECTOR_VERSION or any(r["ref"] == ref for r in rows):
      continue
    rows.append({"ref": ref, "name": str(b.get("display_name") or ref[:10]), "state": link.model_state(ref),
                 "selected": ref == picked})
  return rows


@_guarded(False)
def select_model(ref, store=None, link=None) -> bool:
  """Pick the big model jetlink runs, by catalog ref; None or '' goes back to jetlink's default (the slot
  removed). Offroad only: the caller checks. False for a ref the catalog does not list. jetlink's settings watcher
  re-provisions on the slot's change (jetlink/openpilot/settings.py, watched keys)."""
  store = _params(store)
  if not ref:
    store.remove(BIG_MODEL_KEY)
    return True
  row = next((r for r in models(store, link) if r["ref"] == ref), None)
  if row is None:
    return False
  store.put(BIG_MODEL_KEY, {"ref": ref, "displayName": row["name"]}, block=True)
  return True
