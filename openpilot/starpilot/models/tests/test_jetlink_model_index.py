import pytest

from jetlink.registry.index import parse_index


@pytest.mark.parametrize("malformed_default", [[], {}], ids=["array", "object"])
def test_model_index_keeps_valid_entries_with_a_malformed_default(malformed_default):
  ref = "a" * 40
  oid = "b" * 64
  index = parse_index({"schema": 1, "default": malformed_default,
                       "models": [{"ref": ref, "oid": oid, "size": 4096, "name": "Big model"}]})

  assert index.default is None
  assert [(model.ref, model.oid, model.size) for model in index.models] == [(ref, oid, 4096)]
  assert index.size(oid) == 4096
