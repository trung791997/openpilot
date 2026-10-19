"""StarPilot's adapter to jetlink's openpilot layer, and the guard every Jetlink hook line calls.

Mirrors firestar5683/StarPilot#231 (Zeph Leggett, MIT): a Stock subclass defined lazily, hooks() over
jetlink.openpilot.hooks (Hooks, or Off, or _NoJetlink without a usable checkout), link_on() as a file read, and
the in-control rule openpilot engaged OR always-on lateral armed (paused included). jetlink_repo is pinned in
jetlink_repo/VENDORED_FROM.

Where this fork differs from the PR (each is this fork's layout, not a change of rule):
  - the module is openpilot.starpilot.jetlink_adapter (the PR's is starpilot.models.jetlink_adapter);
  - the params keep this fork's names (JetlinkLink, AcceleratorProgress, JetlinkModelPointers), and the
    model manager's pick and catalog (JetlinkBigModel, JetlinkCatalog) stay, for the Galaxy model picker,
    which the PR's StarPilot does not have;
  - AOL is starpilotCarState's alwaysOnLateralAllowed (armed, paused included) or alwaysOnLateralEnabled,
    not the PR's aolIntentWire/aolAxisState, which this fork's cereal does not have;
  - openpilot.system.hardware, not openpilot.common.hardware, and the top-level cereal package, not
    openpilot.cereal; Params.put has no block argument;
  - the driving warp is upstream openpilot's pickle under starpilot/jetlink_adapter/models, captured on the
    comma by selfdrive/modeld/compile_driving_warp.py (this fork's modeld compile_warp is the older format);
  - the small model is selfdrive/modeld/jetlink_join.SmallModel, whose state_pairs/packed_input layout
    Stock.small_reset already resets (jetlink.openpilot.warp.reset_for_stock).

The top level imports the standard library only: manager builds its process list from this module and the
gadget owner (jetlinkd) runs it.
"""
from __future__ import annotations

import functools
import os
import time
from collections import namedtuple
from pathlib import Path

API = 3  # the jetlink.openpilot API this adapter is written to
UNSUPPORTED = "Jetlink requires a comma 3X or comma 4"

# the gadget owner, as manager names the process and selfdrived lists it
OWNER = 'jetlinkd'

# the Jetlink setting, stored as an index: written out so the panels build the setting without a jetlink checkout
MODES = ('off', 'usb', 'ios')

# the params jetlink reads and writes, all declared in params_keys.h. big_model is the Galaxy picker's pick,
# {ref, displayName}, and catalog jetlink's big-model catalog ({bundles: [...]}); unset, jetlink runs its default
_Keys = namedtuple('_Keys', 'link offroad progress spec pointers big_model catalog charge_phone')
KEYS = _Keys(link='JetlinkLink', offroad='IsOffroad', progress='AcceleratorProgress', spec='JetlinkSpec',
             pointers='JetlinkModelPointers', big_model='JetlinkBigModel', catalog='JetlinkCatalog',
             charge_phone='JetlinkChargePhone')

# AOL armed (paused included) or engaged: starpilot_card publishes both at 100 Hz
AOL_SERVICE = 'starpilotCarState'
AOL_FIELDS = ('alwaysOnLateralAllowed', 'alwaysOnLateralEnabled')
# how long after AOL's reader is built a state not yet heard counts as armed: its first polls run before the
# subscription connects
AOL_GRACE_S = 1.

# where the driving warp for each camera is kept and modeld loads it from: upstream openpilot's format and name,
# captured on the comma's GPU by selfdrive/modeld/compile_driving_warp.py. In the fork's tree, never in
# jetlink_repo, and not Paths.comma_home(), which on AGNOS is a tmpfs overlay. No file here, no large model:
# the drive stays on the small model (warp.Warps.built)
WARP_DIR = Path(__file__).resolve().parent / 'models'


def warp_path(cam_w: int, cam_h: int, model_w: int, model_h: int) -> Path:
  """The driving warp for one camera, what modeld opens. Upstream names it by the camera only; model_w x model_h
  is the 512x256 every driving model reads, and warp.Warps.load checks the pickle's output shape against it."""
  return WARP_DIR / f'driving_warp_{cam_w}x{cam_h}_tinygrad.pkl'


def supported_device(device_type=None) -> bool:
  if device_type is None:
    from openpilot.system.hardware import HARDWARE
    device_type = HARDWARE.get_device_type()
  return device_type in ("tizi", "mici")


def aol_in_control(state, alive, waiting=False) -> bool:
  """AOL armed (paused or not) or engaged: the half of the swap's in-control rule that is not openpilot's, shared
  by engaged() and selfdrived. `state` is starpilotCarState's message, None until heard; `alive` that it still is;
  `waiting` that its reader is younger than AOL_GRACE_S, when a state not heard yet may be armed."""
  if state is None:
    return bool(waiting)
  if not alive:
    return True
  return any(bool(getattr(state, field, False)) for field in AOL_FIELDS)


@functools.cache
def _adapter_class():
  from jetlink.openpilot.interface import Keys
  from jetlink.openpilot.stock import Stock
  from jetlink.registry.catalog import REQUIRED_SELECTOR_VERSION

  class StarPilotAdapter(Stock):
    KEYS = Keys(**KEYS._asdict())
    # the catalog is jetlink's own fetch (refresh_catalog), kept at the selector it merges to
    CATALOG_SELECTOR = REQUIRED_SELECTOR_VERSION
    IN_CONTROL_SERVICES = ("carControl", "selfdriveState")
    _aol = None
    _aol_since = 0.

    def camera(self):
      # Stock's, over this fork's openpilot.system.hardware
      from openpilot.system.hardware import HARDWARE
      from openpilot.common.transformations.camera import _ar_ox_fisheye, _os_fisheye
      from openpilot.common.transformations.model import MEDMODEL_INPUT_SIZE
      camera = _os_fisheye if HARDWARE.get_device_type() == 'mici' else _ar_ox_fisheye
      return camera.width, camera.height, *MEDMODEL_INPUT_SIZE

    def warp_path(self, cam_w, cam_h, model_w, model_h):
      return warp_path(cam_w, cam_h, model_w, model_h)

    def put(self, key, value, *, block=False):
      # this fork's Params.put has no block argument and always writes before it returns, which is what
      # block=True asks for and never less than False
      self._params().put(key, value)

    def model_root(self):
      # StarPilot's model root: its cleanup deletes stray files there, never directories, and jetlink keeps its
      # downloads in jetlink/ under it
      from openpilot.starpilot.common.starpilot_variables import MODELS_PATH
      return Path(MODELS_PATH)

    def in_control(self) -> bool:
      # Stock's, with its SubMaster made from this fork's top-level cereal package (Stock imports
      # openpilot.cereal, a second copy of the schema here). Stock's own retry and fail-closed rules apply: a
      # SubMaster that cannot be made, a service late or invalid, or any failure answers True (no swap)
      if self._sm is None:
        if time.monotonic() < self._sm_after:
          return True
        self._sm_after = time.monotonic() + 5.
        try:
          import cereal.messaging as messaging
          self._sm = messaging.SubMaster(list(self.IN_CONTROL_SERVICES))
        except Exception as e:
          try:
            self.log.error("jetlink: in_control() failed: %s", f"{type(e).__name__}: {e}")
          except Exception:
            pass
          return True
      return super().in_control()

    def engaged(self, sm) -> bool:
      # openpilot, or aol_in_control(); Stock.in_control() answers True if this raises. AOL's reader is built and
      # polled on every call, openpilot engaged or not, so it has heard AOL by the time an offer or a fall asks,
      # rather than answering from its grace
      if self._aol is None:
        import cereal.messaging as messaging
        self._aol, self._aol_since = messaging.SubMaster([AOL_SERVICE]), time.monotonic()
      aol = self._aol
      aol.update(0)
      if super().engaged(sm):
        return True
      heard = aol.seen[AOL_SERVICE]
      return aol_in_control(aol[AOL_SERVICE] if heard else None, aol.alive[AOL_SERVICE],
                            waiting=time.monotonic() - self._aol_since < AOL_GRACE_S)

  return StarPilotAdapter


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
    with open(os.path.join(root, os.environ.get("OPENPILOT_PREFIX", "d"), KEYS.link)) as f:
      return f.read().strip() in ("1", "2")
  except OSError:
    return False


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
  prepare(). The model manager's (model_state, should_extend_catalog, extend_catalog) are this fork's."""
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


# -- the hook lines, by name, for the callers that predate hooks() ------------

def should_run(started: bool, params, CP) -> bool:
  """manager's rule for jetlinkd: the link is on and no chestnut is fitted. jetlinkd runs onroad too: a gadget
  whose owner exits leaves the bus."""
  return hooks().should_run(started, params, CP)


def status():
  """One snapshot for the UI and the panels (jetlink.openpilot.Status), or None when there is no jetlink here."""
  return hooks().status()


def reason() -> str | None:
  """Why the link the user turned on cannot run: hardwared's offroad alert. Files only."""
  return hooks().reason()


# -- StarPilot's model manager and the Galaxy model picker (this fork's; not in the PR) --------------------------

def _guarded(default):
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


@_guarded(False)
def refresh_catalog() -> bool:
  """The model manager's refresh: fetch the big-model catalogs and keep them in KEYS.catalog. Network; a failed
  fetch keeps the last catalog. Skipped with a chestnut fitted, which runs StarPilot's own big models. True when
  the stored catalog changed."""
  link = hooks()
  if not link.should_extend_catalog():
    return False
  store = adapter()
  cached = store.get(KEYS.catalog)
  cached = cached if isinstance(cached, dict) else {}
  merged = link.extend_catalog(cached)
  if not isinstance(merged, dict) or not merged.get('bundles') or merged == cached:
    return False
  store.put(KEYS.catalog, merged, block=True)
  return True


@_guarded(list)
def models() -> list[dict]:
  """The picker's rows, newest first: {ref, name, state, selected}. state is 'ready' (built on the host),
  'downloaded' (on the comma) or None. Records only, no network."""
  from jetlink.registry.catalog import REQUIRED_SELECTOR_VERSION, is_ref
  store = adapter()
  catalog = store.get(KEYS.catalog)
  bundles = catalog.get('bundles', []) if isinstance(catalog, dict) else []
  pick = store.get(KEYS.big_model)
  picked = pick.get('ref') if isinstance(pick, dict) else None
  rows = []
  for b in sorted((b for b in bundles if isinstance(b, dict)), key=lambda b: int(b.get('index', 0) or 0), reverse=True):
    ref = b.get('ref')
    try:
      selector = int(b.get('minimum_selector_version', 0))
    except (TypeError, ValueError):
      continue
    if not is_ref(ref) or selector != REQUIRED_SELECTOR_VERSION or any(r['ref'] == ref for r in rows):
      continue
    rows.append({'ref': ref, 'name': str(b.get('display_name') or ref[:10]), 'state': hooks().model_state(ref),
                 'selected': ref == picked})
  return rows


@_guarded(False)
def select_model(ref: str | None) -> bool:
  """Pick the big model jetlink runs, by catalog ref; None or '' goes back to jetlink's default. Offroad only:
  the caller checks. False for a ref the catalog does not list."""
  store = adapter()
  if not ref:
    store.remove(KEYS.big_model)
    return True
  row = next((r for r in models() if r['ref'] == ref), None)
  if row is None:
    return False
  store.put(KEYS.big_model, {'ref': ref, 'displayName': row['name']}, block=True)
  return True
