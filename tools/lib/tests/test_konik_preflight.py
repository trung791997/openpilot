"""Regression tests for tools/konik_preflight.py.

The point of these is narrow and specific. The preflight script has to run on a machine
with no openpilot build -- your laptop, a fresh comma -- so it carries its own copy of the
Bosch-A CAN ID table. A mirrored constant that drifts from the thing it mirrors is exactly
the failure AGENTS.md §3 is about: the script would keep reporting "0 Bosch-A frames" on a
route that is full of them, and the number would look authoritative.

So: assert the mirror against the real parser, and assert that the JWT expiry check
actually rejects an expired token rather than passing everything through.
"""

import base64
import importlib.util
import json
import os
from datetime import UTC, datetime, timedelta

import pytest

_HERE = os.path.dirname(__file__)
_SCRIPT = os.path.normpath(os.path.join(_HERE, "..", "..", "konik_preflight.py"))


def _load():
  spec = importlib.util.spec_from_file_location("konik_preflight", _SCRIPT)
  module = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(module)
  return module


preflight = _load()


def _fake_jwt(exp: datetime | None) -> str:
  payload = {"identity": "test"}
  if exp is not None:
    payload["exp"] = int(exp.timestamp())
  body = base64.urlsafe_b64encode(json.dumps(payload).encode()).rstrip(b"=").decode()
  return f"header.{body}.signature"


class TestBoschAIdMirror:
  def test_fallback_table_matches_the_parser_exactly(self):
    """The mirrored table must equal opendbc's, or the radar check silently under-reports."""
    from opendbc.car.honda.radar_interface import BOSCH_A_ALL_IDS

    fallback = set(preflight._FALLBACK_MAIN) | set(preflight._FALLBACK_AUX)
    assert fallback == set(BOSCH_A_ALL_IDS), (
      "konik_preflight's mirrored Bosch-A ID table has drifted from "
      + "opendbc/car/honda/radar_interface.py. Re-mirror it."
    )

  def test_table_is_the_full_16_slot_bank(self):
    """16 slots x 4 main frames + 16 aux frames = 80 distinct IDs (D-039)."""
    fallback = set(preflight._FALLBACK_MAIN) | set(preflight._FALLBACK_AUX)
    assert len(preflight._FALLBACK_MAIN) == 64
    assert len(preflight._FALLBACK_AUX) == 16
    assert len(fallback) == 80, "main and aux ranges must not overlap"

  def test_prefers_the_parser_when_importable(self):
    ids, source = preflight._bosch_a_ids()
    assert source == "parser"
    assert len(ids) == 80


class TestJwtExpiry:
  def test_expired_token_is_detected(self):
    token = _fake_jwt(datetime.now(UTC) - timedelta(hours=1))
    exp = preflight._jwt_expiry(token)
    assert exp is not None and exp < datetime.now(UTC)

  def test_valid_token_is_not_flagged(self):
    token = _fake_jwt(datetime.now(UTC) + timedelta(hours=1))
    exp = preflight._jwt_expiry(token)
    assert exp is not None and exp > datetime.now(UTC)

  @pytest.mark.parametrize("token", ["", "not-a-jwt", "a.b", "a.!!!.c"])
  def test_malformed_tokens_return_none_rather_than_raising(self, token):
    """A malformed token must not crash the preflight before it can report anything."""
    assert preflight._jwt_expiry(token) is None

  def test_token_without_exp_returns_none(self):
    assert preflight._jwt_expiry(_fake_jwt(None)) is None


class TestTokenLookup:
  def test_env_var_takes_precedence(self, monkeypatch):
    monkeypatch.setenv("KONIK_TOKEN", "from-env")
    assert preflight._token("https://api.konik.ai") == "from-env"

  def test_comma_token_is_a_fallback_env(self, monkeypatch):
    monkeypatch.delenv("KONIK_TOKEN", raising=False)
    monkeypatch.setenv("COMMA_TOKEN", "from-comma-env")
    assert preflight._token("https://api.konik.ai") == "from-comma-env"
