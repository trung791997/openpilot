"""
Jetlink on StarPilot: the one module that holds every openpilot import jetlink
needs, and the functions the hooks call. Ported from zoompilot's
openpilot/sunnypilot/jetlink_adapter (develop, jetlink API 2; jetlink_repo
pinned in jetlink_repo/VENDORED_FROM). Original copyright Zeph Leggett, MIT.

Differences from zoompilot: no sunnypilot model manager. StarPilot's manifest
names no comma commits, so jetlink keeps its own catalog (JetlinkCatalog,
sunnypilot's big-model catalogs as jetlink fetches them, refreshed by
StarPilot's model manager) and its own pick (JetlinkBigModel, set from the
Galaxy model manager; unset runs jetlink's default big model). No MADS (in_control reads
carControl), StarPilot's MODELS_PATH as the model root, and this fork's
make_warp signature.

manager imports this module to build the process list and runs it as jetlinkd,
the resident gadget owner, so the top level is the standard library only.
"""
from __future__ import annotations

import functools
import os
import threading
from collections import namedtuple
from pathlib import Path

# the version of jetlink.openpilot's API this adapter is written to; any other
# is treated as jetlink being absent, with the reason as the offroad alert
API = 2

# the gadget owner, as manager names the process and selfdrived lists it
OWNER = 'jetlinkd'

# the Jetlink setting, stored as an index: jetlink.openpilot.MODES,
# written out so the panels build the setting without a jetlink checkout
MODES = ('off', 'usb', 'ios')

# the params jetlink reads and writes, all declared in params_keys.h. big_model
# is the pick, {ref, displayName}, and catalog jetlink's catalog JSON, in the
# shape sunnypilot's ModelFetcher caches ({bundles: [...]})
_Keys = namedtuple('_Keys', 'link offroad progress spec pointers big_model catalog charge_phone')
KEYS = _Keys(link='JetlinkLink', offroad='IsOffroad', progress='AcceleratorProgress', spec='JetlinkSpec',
             pointers='JetlinkModelPointers', big_model='JetlinkBigModel', catalog='JetlinkCatalog',
             charge_phone='JetlinkChargePhone')

# comma's chestnut, running and in its ROM (system.hardware.usb): the comma's
# USB-C port hosts one and is never held as a jetlink device beside it. The
# hardware package is too heavy for the owner, so they are written out here
CHESTNUT_IDS = frozenset({(0xADD1, 0x0001), (0x3801, 0x0001), (0x174C, 0x2464), (0x174C, 0x2463)})

# where the build puts the warp for each camera (SConscript) and modeld loads
# it from: in the fork's tree, never in the jetlink submodule (a file there
# leaves it dirty for the updater), and under the *.pkl ignore, which the
# release scripts add past. Not Paths.comma_home(), which on AGNOS is a tmpfs
# overlay: the pickle was gone every boot
WARP_DIR = Path(__file__).resolve().parent / 'models'

OWNER_LOG = Path('/data/log/jetlink-owner.log')

_AGNOS = os.path.isfile('/AGNOS')


def _params_dir() -> Path:
  """The params store's directory, by params.cc and hw.h's rule: PARAMS_ROOT,
  else /data/params on a device and ~/.comma<OPENPILOT_PREFIX>/params
  elsewhere, then /<OPENPILOT_PREFIX, or d>. Per call, so it follows a prefix
  as Params does; the environment only, so it never raises."""
  prefix = os.environ.get('OPENPILOT_PREFIX', '')
  root = os.environ.get('PARAMS_ROOT')
  if root is None:
    root = '/data/params' if _AGNOS else os.path.join(os.environ.get('HOME', ''), '.comma' + prefix, 'params')
  return Path(root) / os.environ.get('OPENPILOT_PREFIX', 'd')


def warp_path(cam_w: int, cam_h: int, model_w: int, model_h: int) -> Path:
  """The warp for one geometry: the build's target and what modeld opens."""
  return WARP_DIR / f'warp_{cam_w}x{cam_h}_{model_w}x{model_h}_tinygrad.pkl'


def owner_config():
  """What jetlinkd needs: data only, so the owner imports nothing heavy."""
  from jetlink.openpilot.interface import Keys, OwnerConfig

  from openpilot.common.basedir import BASEDIR
  # the provisioning run starts in the checkout with the checkout on its path,
  # where launch_chffrplus.sh links jetlink_repo/jetlink in as jetlink
  return OwnerConfig(params_dir=_params_dir(), keys=Keys(**KEYS._asdict()), chestnut_ids=CHESTNUT_IDS,
                     adapter=__name__, cwd=Path(BASEDIR), env={'PYTHONPATH': BASEDIR}, log_file=OWNER_LOG)


def main() -> None:
  """jetlinkd: hold the USB gadget until manager stops this process."""
  from jetlink.openpilot.owner import main as run_owner
  run_owner(owner_config())


def adapter() -> Adapter:
  """The adapter, for jetlink's entry points that run as their own process:
  the provisioning run and the warp build."""
  return Adapter()


class Adapter:
  """jetlink.openpilot.interface.Openpilot over this fork."""

  def __init__(self):
    from jetlink.openpilot.interface import Keys

    from openpilot.common.basedir import BASEDIR
    from openpilot.common.swaglog import cloudlog
    self.keys = Keys(**KEYS._asdict())
    self.log = cloudlog
    self.basedir = Path(BASEDIR)
    # one Params per store: constructing one costs 144 us on the comma against
    # 110 us for the read, and the UI reads several five times a second. By
    # store, since a test or a bench runs under its own prefix
    self._stores: dict[Path, object] = {}

  # -- params ---------------------------------------------------------------

  def params_dir(self) -> Path:
    return _params_dir()

  def _params(self):
    where = _params_dir()
    store = self._stores.get(where)
    if store is None:
      from openpilot.common.params import Params
      store = self._stores[where] = Params()
    return store

  def get(self, key: str):
    # read from hardwared and the UI's threads, which UnknownKeyName (a
    # params library older than the key) must not take down
    try:
      return self._params().get(key)
    except Exception:
      return None

  def put(self, key: str, value, *, block: bool = False) -> None:
    # this fork's Params.put has no block argument and always writes before
    # it returns, which is what block=True asks for and never less than False
    self._params().put(key, value)

  def remove(self, key: str) -> None:
    self._params().remove(key)

  # -- the device -------------------------------------------------------------

  def chestnut_present(self) -> bool:
    from openpilot.system.hardware.usb import is_chestnut_usb_id, read_int, usb_devices
    return any(is_chestnut_usb_id(read_int(d / 'idVendor', 16), read_int(d / 'idProduct', 16), True) for d in usb_devices())

  def camera(self) -> tuple[int, int, int, int]:
    # the choice modeld/SConscript makes for a source build
    from openpilot.system.hardware import HARDWARE
    from openpilot.common.transformations.camera import _ar_ox_fisheye, _os_fisheye
    from openpilot.common.transformations.model import MEDMODEL_INPUT_SIZE
    camera = _os_fisheye if HARDWARE.get_device_type() == "mici" else _ar_ox_fisheye
    return camera.width, camera.height, *MEDMODEL_INPUT_SIZE

  def warp_path(self, cam_w: int, cam_h: int, model_w: int, model_h: int) -> Path:
    return warp_path(cam_w, cam_h, model_w, model_h)

  def model_root(self) -> Path:
    # StarPilot's model root: its cleanup deletes stray files there, never
    # directories, and jetlink keeps its downloads in jetlink/ under it
    from openpilot.starpilot.common.starpilot_variables import MODELS_PATH
    return Path(MODELS_PATH)

  @property
  def catalog_selector(self) -> int:
    # the catalog is jetlink's own fetch, kept at the selector it merges to
    from jetlink.registry.catalog import REQUIRED_SELECTOR_VERSION
    return REQUIRED_SELECTOR_VERSION

  # -- modeld -----------------------------------------------------------------

  def model_face(self):
    """comma's large model's face: stock modeld's Parser, constants and action
    function, and modeld_v2's ModelConstants, which modeld_tinygrad reads off
    the model."""
    from jetlink.openpilot.interface import ModelFace

    from openpilot.selfdrive.modeld.constants import ModelConstants
    from openpilot.selfdrive.modeld.modeld import LAT_SMOOTH_SECONDS, LONG_SMOOTH_SECONDS, get_action_from_model
    from openpilot.selfdrive.modeld.parse_model_outputs import Parser
    from openpilot.system.camerad.cameras.nv12_info import get_nv12_info
    return ModelFace(parser=Parser, frame_size=lambda w, h: get_nv12_info(w, h)[3], desire_len=ModelConstants.DESIRE_LEN,
                     constants=ModelConstants, lat_smooth_seconds=LAT_SMOOTH_SECONDS,
                     long_smooth_seconds=LONG_SMOOTH_SECONDS, get_action_from_model=get_action_from_model)

  def event(self, name: str, **fields) -> None:
    self.log.event(name, **fields)

  # -- the build --------------------------------------------------------------

  def make_warp(self, cam_w: int, cam_h: int, model_w: int, model_h: int):
    # compile_modeld first: it patches tinygrad's firmware fetch as it loads
    # In policy-history mode on the default device this fork's make_warp is
    # upstream's graph (frame_skip is unused there)
    from openpilot.selfdrive.modeld.compile_modeld import IMAGE_HISTORY_IN_POLICY, NV12Frame, make_warp
    from openpilot.system.camerad.cameras.nv12_info import get_nv12_info
    from tinygrad import Device
    nv12 = NV12Frame(cam_w, cam_h, *get_nv12_info(cam_w, cam_h))
    return make_warp(nv12, model_w, model_h, 1, IMAGE_HISTORY_IN_POLICY, Device.DEFAULT), nv12.size


# -- what the hooks call ------------------------------------------------------

class _Absent:
  """jetlink's answers when it cannot run here: not checked out (why is
  None), or a package this build cannot use (why says so, as the offroad
  alert, to someone who turned the link on)."""

  def __init__(self, why: str | None):
    self.why = why

  def enabled(self) -> bool:
    return False

  def status(self):
    return None

  def reason(self) -> str | None:
    if self.why is None:
      return None
    # the setting as jetlink reads it, a file: hardwared asks twice a second
    try:
      on = 0 < int((_params_dir() / KEYS.link).read_bytes()) < len(MODES)
    except (OSError, ValueError):
      on = False
    return self.why if on else None

  def prepare(self) -> bool:
    return False

  def attach(self, small, cam_w: int, cam_h: int):
    return None

  def request_shutdown(self, reason: str = '') -> bool:
    return False

  def shutdown_pending(self) -> bool:
    return False

  def should_extend_catalog(self) -> bool:
    return False

  def extend_catalog(self, catalog: dict) -> dict:
    return catalog


_bound = None
_binding = threading.Lock()


def _api():
  """jetlink for this process, bound to the adapter on first use. Kept, the
  null answers included: Python does not cache a failed import, and searching
  the path again on every UI and hardwared call costs more than the call. One
  per process: prepare() and attach() have to reach the same one."""
  global _bound
  if _bound is None:
    with _binding:
      if _bound is None:
        _bound = _bind()
  return _bound


def _bind():
  try:
    import jetlink
    if getattr(jetlink, '__file__', None) is None:
      return _Absent(None)   # an empty jetlink_repo, which Python takes for a namespace package
    import jetlink.openpilot as jl
  except ModuleNotFoundError as e:
    if e.name == 'jetlink':
      return _Absent(None)   # no checkout: the link does not exist on this device
    if (e.name or '').startswith('jetlink.'):
      return _unusable("jetlink package too old for this build", e)
    return _unusable(f"jetlink failed to load: {e}", e)
  except Exception as e:
    return _unusable(f"jetlink failed to load: {type(e).__name__}: {e}", e)
  api = getattr(jl, 'API', None)
  if api != API:
    return _unusable(f"jetlink package API {api}, this build expects {API}")
  try:
    return jl.bind(Adapter())
  except Exception as e:
    return _unusable(f"jetlink failed to start: {type(e).__name__}: {e}", e)


def _unusable(why: str, error: Exception | None = None) -> _Absent:
  _log_failure(why, error)
  return _Absent(why)


# manager, hardwared, the model manager and the UI call in here on every
# device, link on or off, and modeld on every drive: whatever jetlink does
# wrong turns the link off and is logged, and never takes one of them down.
# jetlink's own readers never raise; this is the net under that promise.
# Hook -> the failure last logged for it, cleared by a call that works
_failed_hooks: dict[str, str] = {}


def _log_failure(what: str, error: Exception | None) -> None:
  try:
    from openpilot.common.swaglog import cloudlog
    cloudlog.error("jetlink: %s", what, exc_info=error)
  except Exception:
    pass


def _guarded(default):
  def wrap(hook):
    @functools.wraps(hook)
    def call(*args, **kwargs):
      try:
        result = hook(*args, **kwargs)
      except Exception as e:
        # once per distinct error, as jetlink's readers log: the UI would log
        # a failing status five times a second
        error = f"{type(e).__name__}: {e}"
        if _failed_hooks.get(hook.__name__) != error:
          _failed_hooks[hook.__name__] = error
          _log_failure(f"{hook.__name__}() failed", e)
        return default(*args, **kwargs) if callable(default) else default
      _failed_hooks.pop(hook.__name__, None)
      return result
    return call
  return wrap


@_guarded(False)
def should_run(started: bool, params, CP) -> bool:
  """manager's rule for jetlinkd: the link is on and no chestnut is fitted.
  jetlinkd runs onroad too: a gadget whose owner exits leaves the bus."""
  return _api().enabled()


@_guarded(None)
def status():
  """One snapshot for the UI and the panels (jetlink.openpilot.Status), or
  None when there is no jetlink here."""
  return _api().status()


@_guarded(None)
def reason() -> str | None:
  """Why the link the user turned on cannot run: hardwared's offroad alert.
  Files only, so hardwared can ask twice a second."""
  return _api().reason()


@_guarded(False)
def prepare() -> bool:
  """modeld, before config_realtime_process: will the link join this modeld?
  The GPU's setup has to happen now, or its threads inherit the frame loop's
  realtime priority and core."""
  return _api().prepare()


# what in_control() reads; modeld subscribes to both
IN_CONTROL = ('carState', 'carControl')


@_guarded(True)
def in_control(sm) -> bool:
  """modeld, before every frame, onto the model: is openpilot steering or
  holding the car? jetlink's large model swaps in only while it is not.
  StarPilot's always-on lateral sets latActive without enabled. A service
  late or invalid counts as in control."""
  if not (sm.all_alive(IN_CONTROL) and sm.all_valid(IN_CONTROL)):
    return True
  cc = sm['carControl']
  return bool(cc.enabled or cc.latActive or cc.longActive)


@_guarded(None)
def attach(small, cam_w: int, cam_h: int):
  """modeld, once the camera is up and `small` is built: the model to run,
  `small` driving until the link has joined; None unless prepare() said yes."""
  return _api().attach(small, cam_w, cam_h)


@_guarded(False)
def request_shutdown(reason: str = '') -> bool:
  """hardwared, once, when the comma is about to power off for good: ask for
  the far end to go down with it. Returns at once: True when the request now
  waits for jetlinkd, which shutdown_pending() follows.

  A jetlink of API 1 from before the non-blocking power-off has no
  request_shutdown: it is asked the old way, blocking up to 25 s, so the
  Jetson is never left on for want of a method."""
  api = _api()
  ask = getattr(api, 'request_shutdown', None)
  if ask is None:
    api.shutdown(reason, 25.0)
    return False
  return ask(reason)


@_guarded(False)
def shutdown_pending() -> bool:
  """hardwared, every loop after request_shutdown(), until it puts DoShutdown:
  has jetlinkd still to take the request? A stat."""
  return getattr(_api(), 'shutdown_pending', lambda: False)()


@_guarded(False)
def should_extend_catalog() -> bool:
  """Should the big-model catalog carry the models newer catalogs list?
  Hardware, not the link setting: the model manager drops a pick its catalog
  does not list."""
  return _api().should_extend_catalog()


@_guarded(lambda catalog: catalog)
def extend_catalog(catalog: dict) -> dict:
  """The big-model catalog with those models folded in."""
  return _api().extend_catalog(catalog)


# -- StarPilot's model manager and the Galaxy model picker -------------------

@_guarded(False)
def refresh_catalog() -> bool:
  """The model manager's refresh: fetch the big-model catalogs and keep them in
  KEYS.catalog. Network; a failed fetch keeps the last catalog (jetlink's
  big_catalog never raises). Skipped with a chestnut fitted, which runs
  StarPilot's own big models. True when the stored catalog changed."""
  if not should_extend_catalog():
    return False
  store = adapter()
  cached = store.get(KEYS.catalog)
  cached = cached if isinstance(cached, dict) else {}
  merged = extend_catalog(cached)
  if not merged.get('bundles') or merged == cached:
    return False
  store.put(KEYS.catalog, merged, block=True)
  return True


@_guarded(list)
def models() -> list[dict]:
  """The picker's rows, newest first: {ref, name, state, selected}. state is
  'ready' (built on the host), 'downloaded' (on the comma) or None. Records
  only, no network."""
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
    state = getattr(_api(), 'model_state', lambda ref: None)(ref)
    rows.append({'ref': ref, 'name': str(b.get('display_name') or ref[:10]), 'state': state, 'selected': ref == picked})
  return rows


@_guarded(False)
def select_model(ref: str | None) -> bool:
  """Pick the big model jetlink runs, by catalog ref; None or '' goes back to
  jetlink's default. Offroad only: the caller checks. False for a ref the
  catalog does not list."""
  store = adapter()
  if not ref:
    store.remove(KEYS.big_model)
    return True
  row = next((r for r in models() if r['ref'] == ref), None)
  if row is None:
    return False
  store.put(KEYS.big_model, {'ref': ref, 'displayName': row['name']}, block=True)
  return True
