"""Exercise Jetlink's model choices at the comma/host handshake."""
import json
import logging
from types import SimpleNamespace
from unittest.mock import patch

import pytest

from jetlink.comma import gadget
from jetlink.openpilot.interface import Keys
from jetlink.openpilot.link import Link, open_link
from jetlink.openpilot.models import Models
from jetlink.registry.index import DEFAULT_BIG_MODEL_REF, NetworkError
from openpilot.starpilot.models.jetlink_adapter import adapter
from openpilot.starpilot.models.jetlink_settings import JetlinkSettings


MODEL = "a" * 64
SIZE = 768749892


@pytest.fixture
def comma(tmp_path, monkeypatch):
  params = tmp_path / "params"
  params.mkdir()

  def get(key):
    path = params / key
    return json.loads(path.read_text()) if path.exists() else None

  def put(key, value, *, block):
    assert block
    (params / key).write_text(json.dumps(value))

  monkeypatch.setattr(gadget, "SERVER", tmp_path / "server.json")
  monkeypatch.setattr(gadget, "wait_for_host", lambda *args, **kwargs: True)
  op = SimpleNamespace(keys=adapter().keys, log=logging.getLogger(__name__), params_dir=lambda: params,
                       model_root=lambda: tmp_path, get=get, put=put, remove=lambda key: (params / key).unlink(missing_ok=True))
  return op, Models(op)


def join(models, hello):
  def ensure_engine(sha256, nbytes, **kwargs):
    # The fake host can reuse exactly one already-built engine.
    assert (sha256, nbytes, kwargs["onnx_path"]) == (MODEL, SIZE, None)
    return SimpleNamespace(sha256=sha256, nbytes=nbytes)

  def ignore(*args, **kwargs):
    pass

  client = SimpleNamespace(hello=lambda **kwargs: hello, rebind=ignore, ensure_engine=ensure_engine)
  link = Link(models.op.log)
  parts = SimpleNamespace(models=models, log=models.op.log, spec=SimpleNamespace(ready_spec=lambda: None, store=ignore),
                          progress=SimpleNamespace(report=ignore, report_with_eta=ignore))
  with patch.object(link, "open", return_value=client):
    return open_link(parts, link)[1]


@pytest.fixture
def offline_index(monkeypatch):
  def unreachable():
    raise NetworkError("index is unreachable on this link")

  monkeypatch.setattr("jetlink.registry.index.fetch_index", unreachable)


def test_prebuilt_engine_joins_offline_after_upgrade(comma, offline_index):
  op, models = comma
  op.put(op.keys.big_model, {"sha256": MODEL}, block=True)
  op.put(op.keys.pointers, {MODEL: {"oid": MODEL, "size": SIZE}}, block=True)

  spec = join(models, {"cached_models": [MODEL], "loaded": MODEL})

  assert (spec.sha256, spec.nbytes) == (MODEL, SIZE)


def test_prebuilt_engine_joins_after_adopting_a_pick_missing_from_the_index(comma, offline_index):
  op, models = comma
  gadget.write_record(models.index_path(), {"schema": 1, "models": [{"ref": "c" * 40, "oid": "d" * 64, "size": 1}]})
  pick = {"ref": "b" * 40, "sha256": MODEL, "nbytes": SIZE, "name": "Host's pick", "age_s": 60}

  spec = join(models, {"pick": pick, "cached_models": [MODEL], "loaded": MODEL})

  assert op.get(op.keys.big_model)["sha256"] == MODEL
  assert (spec.sha256, spec.nbytes) == (MODEL, SIZE)


def test_clearing_active_big_delegates_to_the_jetlink_app(comma, offline_index):
  op, models = comma
  pick = {"ref": "b" * 40, "sha256": MODEL, "nbytes": SIZE, "name": "Host's pick", "age_s": 60}
  op.put(op.keys.big_model, {"sha256": MODEL}, block=True)
  assert models.hello_pick()["sha256"] == MODEL

  JetlinkSettings(params=op).choose(None)
  models = Models(op)

  assert models.hello_pick() is None
  assert join(models, {"pick": pick, "cached_models": [MODEL], "loaded": MODEL}).sha256 == MODEL


def test_a_picker_that_uses_clear_for_default_still_sends_default(comma):
  op, models = comma
  op.keys = Keys(link="Mode", offroad="Offroad", progress="Progress", spec="Spec", big_model="Big", catalog="Catalog")
  op.put(op.keys.big_model, {"sha256": MODEL}, block=True)
  assert models.hello_pick()["sha256"] == MODEL

  op.remove(op.keys.big_model)

  pick = Models(op).hello_pick()
  assert pick is not None
  assert pick["ref"] == DEFAULT_BIG_MODEL_REF


def test_metadata_for_a_different_hash_does_not_name_the_engine(comma, offline_index):
  op, models = comma
  ref, other = "b" * 40, "c" * 64
  op.put(op.keys.big_model, {"ref": ref, "sha256": MODEL}, block=True)
  op.put(op.keys.pointers, {MODEL: {"oid": other, "size": SIZE}}, block=True)
  pick = {"ref": ref, "sha256": other, "nbytes": SIZE, "age_s": 60}

  with pytest.raises(NetworkError):
    join(models, {"pick": pick, "cached_models": [other], "loaded": other})
