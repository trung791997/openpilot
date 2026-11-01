"""The Jetlink big-model picker: the catalog refresh, its rows, the pick, and jetlink's default with no catalog."""
import logging
from types import SimpleNamespace

from jetlink.openpilot.models import Models
from jetlink.registry.catalog import DEFAULT_BIG_MODEL_NAME, DEFAULT_BIG_MODEL_REF, REQUIRED_SELECTOR_VERSION
from openpilot.starpilot.models import jetlink_adapter as adapter

OLD, NEW, NEXT = "a" * 40, "b" * 40, "c" * 40


def bundle(ref, index, selector=REQUIRED_SELECTOR_VERSION, name=None):
  return {"ref": ref, "display_name": name or f"model {index}", "minimum_selector_version": selector, "index": index}


class Store:
  def __init__(self, **values):
    self.values = dict(values)
    self.puts = []

  def get(self, key):
    return self.values.get(key)

  def put(self, key, value, *, block):
    assert block
    self.puts.append(key)
    self.values[key] = value

  def remove(self, key):
    self.values.pop(key, None)


class Link:
  def __init__(self, extend=True, catalog=None, states=None):
    self.extend, self.catalog, self.states = extend, catalog, states or {}
    self.extended = []

  def should_extend_catalog(self):
    return self.extend

  def extend_catalog(self, catalog):
    self.extended.append(catalog)
    return self.catalog

  def model_state(self, ref):
    return self.states.get(ref)


def jetlink_op(store):
  """What jetlink's Models reads through: the adapter's keys and selector, and its get(), which stands jetlink's
  default bundle in for an empty catalog."""
  cls = adapter._adapter_class()

  def get(key):
    value = store.get(key)
    return adapter.with_default_bundle(value) if key == adapter.CATALOG_KEY else value

  # params_dir() names a missing directory: the store is in memory, so the pick has no file time.
  return SimpleNamespace(keys=cls.KEYS, catalog_selector=cls.CATALOG_SELECTOR, get=get, params_dir=lambda: "/nonexistent-params",
                         log=logging.getLogger("jetlink-test"), put=store.put)


def test_selector_is_jetlinks():
  cls = adapter._adapter_class()
  assert cls.CATALOG_SELECTOR == REQUIRED_SELECTOR_VERSION == 19
  assert (cls.KEYS.catalog, cls.KEYS.big_model) == (adapter.CATALOG_KEY, adapter.BIG_MODEL_KEY)


def test_refresh_stores_a_changed_catalog():
  store = Store()
  catalog = {"bundles": [bundle(NEW, 2)]}
  link = Link(catalog=catalog)
  assert adapter.refresh_catalog(store, link) is True
  assert store.values[adapter.CATALOG_KEY] == catalog
  # the raw param, never the default-bundle stand-in
  assert link.extended == [{}]
  # unchanged: no write
  assert adapter.refresh_catalog(store, link) is False
  assert store.puts == [adapter.CATALOG_KEY]


def test_refresh_keeps_the_last_catalog():
  kept = {"bundles": [bundle(OLD, 1)]}
  for link in (Link(extend=False, catalog={"bundles": [bundle(NEW, 2)]}), Link(catalog={"bundles": []}),
               Link(catalog=None), Link(catalog="junk")):
    store = Store(JetlinkCatalog=kept)
    assert adapter.refresh_catalog(store, link) is False
    assert store.values[adapter.CATALOG_KEY] == kept and store.puts == []


def test_refresh_never_raises():
  class Broken(Link):
    def extend_catalog(self, catalog):
      raise RuntimeError("network")

  assert adapter.refresh_catalog(Store(), Broken()) is False


def test_rows_newest_first_at_the_selector():
  store = Store(JetlinkCatalog={"bundles": [bundle(OLD, 1), bundle(NEW, 3), bundle(NEXT, 5, selector=20),
                                            bundle(NEW, 2), bundle("not-a-ref", 4), "junk",
                                            {**bundle(OLD, 6), "minimum_selector_version": "x"}]},
                JetlinkBigModel={"ref": OLD, "displayName": "model 1"})
  rows = adapter.models(store, Link(states={NEW: "ready"}))
  assert rows == [{"ref": NEW, "name": "model 3", "state": "ready", "selected": False},
                  {"ref": OLD, "name": "model 1", "state": None, "selected": True}]


def test_empty_catalog_lists_jetlinks_default():
  for catalog in (None, {}, {"bundles": []}, "junk"):
    rows = adapter.models(Store(JetlinkCatalog=catalog), Link())
    assert rows == [{"ref": DEFAULT_BIG_MODEL_REF, "name": DEFAULT_BIG_MODEL_NAME, "state": None, "selected": False}]


def test_jetlink_models_read_through_the_adapter():
  # an empty catalog: jetlink's default, not None (Models.default_model waits for a catalog otherwise)
  models = Models(jetlink_op(Store()))
  assert models.catalog() == [{"name": DEFAULT_BIG_MODEL_NAME, "ref": DEFAULT_BIG_MODEL_REF}]
  default = models.default_model()
  assert default is not None and default["ref"] == DEFAULT_BIG_MODEL_REF
  # a selector-20 bundle is not one jetlink runs
  store = Store(JetlinkCatalog={"bundles": [bundle(OLD, 1), bundle(NEXT, 5, selector=20)]})
  models = Models(jetlink_op(store))
  assert models.catalog() == [{"name": "model 1", "ref": OLD}]
  default = models.default_model()
  assert default is not None and default["ref"] == OLD
  # the picker and jetlink agree on the rows
  assert [r["ref"] for r in adapter.models(store, Link())] == [m["ref"] for m in models.catalog()]


def test_select_model():
  store = Store(JetlinkCatalog={"bundles": [bundle(OLD, 1), bundle(NEXT, 5, selector=20)]})
  assert adapter.select_model(OLD, store, Link()) is True
  assert store.values[adapter.BIG_MODEL_KEY] == {"ref": OLD, "displayName": "model 1"}
  selected = Models(jetlink_op(store)).selected_model()
  assert selected is not None and selected["ref"] == OLD
  for ref in (NEW, NEXT, "not-a-ref"):
    assert adapter.select_model(ref, store, Link()) is False
  assert store.values[adapter.BIG_MODEL_KEY]["ref"] == OLD
  for empty in ("", None):
    store.values[adapter.BIG_MODEL_KEY] = {"ref": OLD, "displayName": "model 1"}
    assert adapter.select_model(empty, store, Link()) is True
    assert adapter.BIG_MODEL_KEY not in store.values


def test_select_jetlinks_default_from_an_empty_catalog():
  store = Store()
  assert adapter.select_model(DEFAULT_BIG_MODEL_REF, store, Link()) is True
  assert store.values[adapter.BIG_MODEL_KEY] == {"ref": DEFAULT_BIG_MODEL_REF, "displayName": DEFAULT_BIG_MODEL_NAME}
