import ast
import logging
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
import pytest

from jetlink.openpilot.hooks import Hooks, Off
from jetlink.openpilot.stock import Stock
from jetlink.openpilot.testing import FakeSubMaster, check_adapter, check_fallback, check_seam, install_guard
from openpilot.starpilot.models import jetlink_adapter as adapter
from openpilot.starpilot.models.tests.jetlink_fixture import ARMED_PAUSED, AXES_IDLE, rule

ROOT = Path(__file__).resolve().parents[4]
GUARD = install_guard()


@pytest.fixture(autouse=True)
def no_machine_files():
  # on a comma, a test that reaches the live params would clear JetlinkSpec
  GUARD.start()
  yield
  assert GUARD.stop() == []


def test_the_adapter_module():
  assert check_adapter(adapter, heavy=("openpilot.common.params", "openpilot.common.swaglog", "openpilot.cereal")) == []


def test_the_modeld_seam():
  assert check_seam(ROOT / "openpilot/selfdrive/modeld/modeld.py") == []


def test_interface_keys():
  # conformance is check_adapter's; the params directory is Stock's, held against Params in test_jetlink_lifecycle
  op = adapter.adapter()
  assert adapter.owner_config().adapter == adapter.__name__
  assert op.keys.link == "JetlinkMode"
  # the owner's pick and refresh_catalog()'s catalog, both StarPilot JSON params (params_keys.h)
  assert op.keys.big_model == "JetlinkBigModel" and op.keys.catalog == "JetlinkCatalog"
  from jetlink.registry.catalog import REQUIRED_SELECTOR_VERSION
  assert op.catalog_selector == REQUIRED_SELECTOR_VERSION


def test_guard_answers_link_off_without_a_usable_jetlink():
  adapter.hooks.cache_clear()
  try:
    with patch.object(adapter, "supported_device", return_value=False):
      assert isinstance(adapter.hooks(), Off)
    adapter.hooks.cache_clear()
    with patch.object(adapter, "supported_device", return_value=True), patch.object(adapter, "API", 2):
      assert isinstance(adapter.hooks(), Off)
    adapter.hooks.cache_clear()
    with patch.object(adapter, "supported_device", return_value=True):
      assert isinstance(adapter.hooks(), Hooks)
    adapter.hooks.cache_clear()
    with patch.dict("sys.modules", {"jetlink.openpilot.hooks": None}):
      hooks = adapter.hooks()
    assert not hooks.prepare(False) and not hooks.should_run(False, None, None) and hooks.status() is None
    assert hooks.shutdown_step(True) and not hooks.shutdown_step(False)
  finally:
    adapter.hooks.cache_clear()


@pytest.mark.parametrize("mode", [None, "0", "1"])
def test_the_fallback_answers_link_off(monkeypatch, tmp_path, mode):
  # the hooks a missing jetlink reaches: modeld stops at prepare(), selfdrived builds no swap events
  monkeypatch.setenv("PARAMS_ROOT", str(tmp_path))
  monkeypatch.setenv("OPENPILOT_PREFIX", "jetlink-test")
  (tmp_path / "jetlink-test").mkdir()
  if mode is not None:
    (tmp_path / "jetlink-test/JetlinkMode").write_text(mode)
  hooks = ("should_run", "status", "reason", "prepare", "shutdown_step", "runtime", "swap_events")
  assert check_fallback(adapter._NoJetlink("jetlink is missing"), hooks, link_on=adapter.link_on) == []


def test_one_action_call_for_every_small_model():
  from openpilot.selfdrive.modeld import modeld
  from openpilot.starpilot.models import runner

  small = runner.CatalogModelState.__new__(runner.CatalogModelState)
  small.behavior_version = "v9"
  outputs: dict[str, np.ndarray] = {}
  previous = object()
  with patch.object(runner, "action_from_outputs", return_value="catalog") as catalog:
    assert small.get_action_from_model(outputs, previous, .4, .7, 12.) == "catalog"
  catalog.assert_called_once_with(outputs, "v9", previous, .4, .7, 12., modeld.LAT_SMOOTH_SECONDS, modeld.LONG_SMOOTH_SECONDS)
  assert catalog.call_args.args[0] is outputs and catalog.call_args.args[2] is previous
  assert modeld.ModelState.get_action_from_model is modeld.get_action_from_model


def test_small_reset_only_for_the_catalog_runner():
  assert adapter.adapter().small_reset(SimpleNamespace(input_queues={}, npy={}, prev_desire=np.zeros(8))) is None


def test_unsupported_device():
  assert adapter.supported_device("tizi")
  assert adapter.supported_device("mici")
  for device in ("tici", "pc", "unknown", "neo"):
    assert not adapter.supported_device(device)


@pytest.mark.parametrize("device,camera", [("tizi", (1928, 1208)), ("mici", (1344, 760))])
def test_jetlink_runs_the_driving_warp_modeld_ships(device, camera):
  op = adapter.adapter()
  with patch("openpilot.common.hardware.HARDWARE.get_device_type", return_value=device):
    geometry = op.camera()
  assert geometry == (*camera, 512, 256)
  assert op.warp_path(*geometry).is_file()


def test_chestnut_ids_are_openpilots():
  from openpilot.common.hardware.usb import CHESTNUT_ROM_USB_IDS, CHESTNUT_USB_IDS
  assert adapter.adapter().CHESTNUT_IDS == frozenset(CHESTNUT_USB_IDS + CHESTNUT_ROM_USB_IDS) == Stock.CHESTNUT_IDS


DISARMED = SimpleNamespace(allowedLatch=False, lateralArmed=False)


def in_control(core=None, aol=None, intent=DISARMED):
  with patch("openpilot.starpilot.aol.wire.decode_intent", return_value=intent):
    return rule(core, aol).in_control()


# openpilot's own services are Stock's rule, tested in jetlink: one row here, then AOL's
@pytest.mark.parametrize("case,expected", [
  ({}, False),
  ({"core": {"selfdriveState": SimpleNamespace(enabled=True, active=True)}}, True),
  # AOL armed but paused: its axes are idle and only the intent says so
  ({"aol": {"aolIntentWire": b"intent", "aolAxisState": AXES_IDLE}, "intent": ARMED_PAUSED}, True),
  ({"aol": {"aolIntentWire": b"intent"}, "intent": SimpleNamespace(allowedLatch=True, lateralArmed=False)}, True),
  ({"aol": {"aolIntentWire": b"intent"}, "intent": None}, True),
  ({"aol": {"aolIntentWire": b"intent", "aolAxisState": SimpleNamespace(**{**vars(AXES_IDLE), "desiredLateral": True})}}, True),
  ({"aol": {"aolIntentWire": b"intent", "aolAxisState": AXES_IDLE}}, False),
])
def test_in_control(case, expected):
  assert in_control(**case) is expected


def test_in_control_holds_the_swap_off_when_aol_cannot_be_read():
  op = rule(aol={"aolIntentWire": b"intent"})
  op._aol.alive["aolIntentWire"] = False
  assert op.in_control() is True
  op._aol = SimpleNamespace(update=lambda timeout: (_ for _ in ()).throw(RuntimeError("no msgq")))
  assert op.in_control() is True


def test_an_aol_intent_not_heard_from_yet_counts_as_armed():
  # the first frames after the AOL subscription starts have seen nothing yet
  op = rule()
  op._aol = None
  with patch("openpilot.cereal.messaging.SubMaster", side_effect=lambda services: FakeSubMaster(services)), \
       patch("openpilot.starpilot.models.jetlink_adapter.time.monotonic", return_value=100.):
    assert op.in_control() is True
  with patch("openpilot.starpilot.models.jetlink_adapter.time.monotonic", return_value=100. + adapter.AOL_GRACE_S):
    assert op.in_control() is False


def test_aols_reader_is_built_while_openpilot_is_engaged():
  # so it has heard AOL by the time openpilot disengages and an offer or a fall asks
  op = rule(core={"selfdriveState": SimpleNamespace(enabled=True, active=True)})
  op._aol = None
  with patch("openpilot.cereal.messaging.SubMaster", side_effect=lambda services: FakeSubMaster(services)) as built:
    assert op.in_control() is True
    assert op.in_control() is True
  built.assert_called_once_with(list(adapter.AOL_SERVICES))
  assert op._aol.timeouts == [0, 0]


STATEFUL = {"input_shapes": {"new_img": (2, 6, 4, 8), "desire_pulse": (1, 8), "traffic_convention": (1, 2), "state_feat": (1, 3, 4)},
            "input_dtypes": {"state_feat": "float32"}, "state_pairs": ["state_feat"], "mapped_image_input": True}
VISION = {"img": (1, 12, 4, 8), "big_img": (1, 12, 4, 8)}
POLICY = {"features_buffer": (1, 3, 4), "desire_pulse": (1, 3, 8), "traffic_convention": (1, 2)}


@pytest.mark.parametrize("layout", ["split", "supercombo", "stateful"])
def test_the_small_models_history_clears_in_place(layout):
  # after the big model drove, the catalog runner starts from a zero history on the buffers its JIT captured
  from tinygrad import Device, Tensor
  from tinygrad.helpers import Context
  from jetlink.openpilot import warp
  from openpilot.starpilot.models import compile as catalog
  from openpilot.starpilot.models.runner import CatalogModelState

  # Host conformance uses real CPU buffers; never discover hardware under the guard.
  with Context(DEV="CPU"):
    queues, npy = {"split": lambda: catalog.make_split_input_queues(VISION, POLICY, 2, Device.DEFAULT),
                   "supercombo": lambda: catalog.make_supercombo_input_queues({**VISION, **POLICY}, 2, Device.DEFAULT),
                   "stateful": lambda: catalog.make_stateful_input_queues(STATEFUL, Device.DEFAULT)}[layout]()
    small = CatalogModelState.__new__(CatalogModelState)
    small.input_queues, small.npy, small.prev_desire = queues, npy, np.ones(8, dtype=np.float32)
    buffers = {name: q.uop.base.buffer for name, q in queues.items()}
    warp.init_device(logging.getLogger(__name__))  # as modeld: the reset's kernels compile with no compile pool
    with patch.object(Tensor, "assign", side_effect=AssertionError("reset must not compile a new GPU assignment")):
      reset = adapter.adapter().small_reset(small)
    history = [name for name, q in queues.items() if q.device != "NPY" and name not in adapter.FRAME_INPUTS]
    for _ in range(3):
      for name in history:
        queues[name].assign(Tensor.full(queues[name].shape, 7, dtype=queues[name].dtype, device=queues[name].device)).realize()
      for array in npy.values():
        array.fill(3)
      small.prev_desire.fill(1)
      with patch.object(Tensor, "assign", side_effect=AssertionError("reset must not submit a new GPU assignment")):
        reset()
      assert {name: q.uop.base.buffer for name, q in queues.items()} == buffers
      for name in history:
        assert not queues[name].numpy().any(), name
      assert not any(array.any() for array in npy.values())
      assert not queues["packed_npy_inputs"].numpy().any()
      assert not small.prev_desire.any()


class _FreshCompileGuard:
  # stands in for the driving role's USBProtection fresh guard: any compile or kernel launch while it is
  # installed is an "Unexpected fresh QCOM submission" on the car. Every module binding is replaced, since
  # tinygrad imports these by name
  NAMES = ("lower_and_compile", "compile_linear", "run_linear", "hcq_compile")

  def __init__(self, monkeypatch):
    import sys
    from tinygrad.engine import jit, realize
    from tinygrad.runtime.support import hcq2
    self.calls = []
    originals = {getattr(realize, n, None) for n in self.NAMES} | {getattr(hcq2, n, None) for n in self.NAMES}
    originals.discard(None)
    for module in list(sys.modules.values()):
      if module is None or not getattr(module, "__name__", "").startswith("tinygrad"):
        continue
      for name in self.NAMES:
        if getattr(module, name, None) in originals:
          monkeypatch.setattr(module, name, self._refuse(f"{module.__name__}.{name}"))
    monkeypatch.setattr(jit.TinyJit, "__call__", self._refuse("TinyJit.__call__"))

  def _refuse(self, where):
    def refuse(*args, **kwargs):
      self.calls.append(where)
      raise ValueError(f"Unexpected fresh QCOM submission ({where})")
    return refuse


def _history_layout(layout):
  from tinygrad import Device
  from openpilot.starpilot.models import compile as catalog
  return {"split": lambda: catalog.make_split_input_queues(VISION, POLICY, 2, Device.DEFAULT),
          "supercombo": lambda: catalog.make_supercombo_input_queues({**VISION, **POLICY}, 2, Device.DEFAULT),
          "stateful": lambda: catalog.make_stateful_input_queues(STATEFUL, Device.DEFAULT)}[layout]()


@pytest.mark.parametrize("layout", ["split", "supercombo", "stateful"])
def test_the_attach_reset_compiles_nothing_under_the_driving_guard(layout, monkeypatch):
  # Every boot since route 00000004: small_reset -> reset_queues -> TinyJit clear() compiled a fresh kernel
  # inside the driving guard, the attach failed and Jetlink never joined. Capture and reset must both
  # run with the guard installed, keep the buffers the run JIT captured, and leave every history at zero
  from tinygrad.helpers import Context
  from openpilot.starpilot.models.runner import CatalogModelState

  with Context(DEV="CPU"):
    queues, npy = _history_layout(layout)
    small = CatalogModelState.__new__(CatalogModelState)
    small.input_queues, small.npy, small.prev_desire = queues, npy, np.ones(8, dtype=np.float32)
    history = [name for name, q in queues.items() if q.device != "NPY" and name not in adapter.FRAME_INPUTS]
    assert history
    buffers = {name: q.uop.base.buffer for name, q in queues.items()}
    frames = {name: bytes(buffers[name].as_memoryview()) for name in adapter.FRAME_INPUTS if name in buffers}

    guard = _FreshCompileGuard(monkeypatch)
    reset = adapter.adapter().small_reset(small)  # captured inside prepare(), under the guard
    assert callable(reset)
    for _ in range(3):
      for name in history:  # what the big model's drive left behind, written without a kernel
        buffers[name].allocator._copyin(buffers[name]._buf, memoryview(np.full(buffers[name].nbytes, 7, dtype=np.uint8)))
      for array in npy.values():
        array.fill(3)
      small.prev_desire.fill(1)
      reset()
      assert guard.calls == []
      assert {name: q.uop.base.buffer for name, q in queues.items()} == buffers
      for name in history:
        assert not any(buffers[name].as_memoryview()), name
      assert not any(array.any() for array in npy.values())
      assert not small.prev_desire.any()
      for name, before in frames.items():
        assert bytes(buffers[name].as_memoryview()) == before, name


def test_the_old_jit_reset_trips_the_driving_guard(monkeypatch):
  # positive control: the guard above does catch jetlink's TinyJit reset, the cause of the attach failure
  from tinygrad.helpers import Context
  from jetlink.openpilot.warp import reset_queues

  with Context(DEV="CPU"):
    queues, npy = _history_layout("split")
    history = [q for name, q in queues.items() if q.device != "NPY" and name not in adapter.FRAME_INPUTS]
    guard = _FreshCompileGuard(monkeypatch)
    with pytest.raises(ValueError, match="Unexpected fresh QCOM submission"):
      reset = reset_queues(history, npy.values(), np.ones(8, dtype=np.float32))
      if reset is not None:
        reset()
    assert guard.calls


def test_the_reset_refuses_a_queue_with_no_host_mapping():
  # no host view must raise, never fall back to a kernel, and never return None (None means prepare_reset)
  class Allocator: pass
  class Buffer:
    device, nbytes, allocator, _buf = "QCOM", 4, Allocator(), None
    def is_allocated(self): return True
    def _host_mv(self): return None
  class Base: buffer = Buffer()
  class UOp: base = Base()
  class Queue: uop, device = UOp(), "QCOM"
  with pytest.raises(RuntimeError, match="no host mapping"):
    adapter.host_zero_reset([Queue()], [], np.zeros(8, dtype=np.float32))


def test_the_reset_reads_what_the_catalog_runner_keeps():
  # small_reset reads these, and jetlink_warp the warp's; a rename would leave every fallback on a stale history
  tree = ast.parse((ROOT / "openpilot/starpilot/models/runner.py").read_text())
  runner = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "CatalogModelState")
  assigned = {n.attr for n in ast.walk(runner) if isinstance(n, ast.Attribute) and isinstance(n.ctx, ast.Store)
              and isinstance(n.value, ast.Name) and n.value.id == "self"}
  assert {"input_queues", "npy", "prev_desire", "_blob_cache", "camera_warp_descriptor", "frame_buf_size"} <= assigned


@pytest.mark.parametrize("mapping", ["missing", "readonly", "short"])
def test_small_reset_refuses_unusable_mapped_history(mapping):
  from tinygrad import Tensor
  from tinygrad.helpers import Context
  from openpilot.starpilot.models.runner import CatalogModelState

  with Context(DEV="CPU"):
    q = Tensor.ones(4, device="CPU").realize()
    small = CatalogModelState.__new__(CatalogModelState)
    small.input_queues, small.npy, small.prev_desire = {"history": q}, {}, np.ones(8, np.float32)
    buffer = q.uop.base.buffer
    if mapping == "missing":
      mocked = patch.object(buffer, "get_storage", return_value=SimpleNamespace(host=None))
    else:
      view = memoryview(bytes(buffer.nbytes)) if mapping == "readonly" else memoryview(bytearray(1))
      mocked = patch.object(buffer, "as_memoryview", return_value=view)
    with mocked, pytest.raises(ValueError, match="history"):
      adapter.adapter().small_reset(small)
    assert np.all(small.prev_desire == 1)
    assert np.all(q.numpy() == 1)


def test_small_reset_zeroes_legacyf6_history_without_get_storage():
  # legacyf6's Buffer has no get_storage; its QCOM allocator's _as_buffer is the host view
  from openpilot.starpilot.models.runner import CatalogModelState
  history = bytearray(b"\x01" * 8)
  class Allocator:
    def _as_buffer(self, buf): return memoryview(buf)
  class Buffer:
    device, nbytes, allocator, _buf = "CPU", 8, Allocator(), history
    def is_allocated(self): return True
  class Base: buffer = Buffer()
  class UOp: base = Base()
  class Queue: uop, device = UOp(), "CPU"
  small = CatalogModelState.__new__(CatalogModelState)
  small.input_queues, small.npy, small.prev_desire = {"history": Queue()}, {}, np.ones(8, np.float32)
  with patch("tinygrad.Device", {"CPU": SimpleNamespace(synchronize=lambda: None)}):
    reset = adapter.adapter().small_reset(small)
    reset()
  assert history == bytearray(8)
  assert np.all(small.prev_desire == 0)
