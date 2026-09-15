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


class TestRlogScanBack:
  """The newest route is the one that has NOT finished uploading.

  A comma sends qlogs eagerly and holds rlogs until it sees WiFi, so the newest route is
  routinely qlog-only. The user's first real run hit exactly that: 551 routes on the account
  and the preflight reported "qlogs only" on the newest one and stopped, which is true and
  useless -- qlogs drop the CAN data, so that reads as "this path cannot deliver radar" when
  in fact it can, a few routes back.

  These tests drive main() with a stubbed transport and pin the walk-back: it must find the
  rlog route, say how far back it was, download from THAT route, and bound its own cost.
  """

  HOST = "https://api.konik.ai"
  DONGLE = "11c8fa231c0499ed"

  def _routes(self, n):
    return [{"fullname": f"{self.DONGLE}|{i:08d}--deadbeef{i:02d}",
             "start_time_utc_millis": 1_700_000_000_000 - i * 3_600_000} for i in range(n)]

  def _install(self, monkeypatch, files_by_route, n_routes=6, argv=None):
    """Stub the network under main(). Returns the list of route names whose files were read."""
    import socket as _socket
    pre = _load()
    asked = []

    def fake_request(method, url, data=None, headers=None, timeout=30.0, **kw):
      path = url[len(self.HOST):].lstrip("/")
      if path == "":
        return 401, "", "stub"
      if path == "v1/me":
        return 200, json.dumps({"id": "uid", "email": "u@example.com"}), "stub"
      if path.startswith("v1/devices/") and "routes_segments" in path:
        return 200, json.dumps(self._routes(n_routes)), "stub"
      if path.startswith("v1/route/") and path.endswith("/files"):
        name = path[len("v1/route/"):-len("/files")]
        asked.append(name)
        return 200, json.dumps(files_by_route.get(name, {"logs": [], "qlogs": []})), "stub"
      raise AssertionError(f"unexpected request to {path}")

    downloaded = []

    def fake_download(url, max_bytes, timeout=60.0):
      downloaded.append(url)
      return b"x" * 1024, "stub"

    monkeypatch.setattr(pre.plain_http, "request", fake_request)
    monkeypatch.setattr(pre.plain_http, "download", fake_download)
    monkeypatch.setattr(pre, "_token", lambda host: _fake_jwt(datetime.now(UTC) + timedelta(days=30)))
    monkeypatch.setattr(_socket, "getaddrinfo", lambda *a, **k: [(2, 1, 6, "", ("1.2.3.4", 443))])
    monkeypatch.setattr(pre.sys, "argv",
                        ["konik_preflight.py", "--dongle-id", self.DONGLE] + (argv or []))
    return pre, asked, downloaded

  def test_walks_back_to_a_route_that_has_rlogs(self, monkeypatch, capsys):
    newest = f"{self.DONGLE}|00000000--deadbeef00"
    third = f"{self.DONGLE}|00000002--deadbeef02"
    pre, asked, downloaded = self._install(monkeypatch, {
      newest: {"logs": [], "qlogs": ["https://api.konik.ai/q0"]},
      f"{self.DONGLE}|00000001--deadbeef01": {"logs": [], "qlogs": ["https://api.konik.ai/q1"]},
      third: {"logs": ["https://api.konik.ai/r2"], "qlogs": ["https://api.konik.ai/q2"]},
    })
    rc = pre.main()
    out = capsys.readouterr().out
    assert third in out, "the route that actually has rlogs must be named"
    assert "2 route(s) back" in out, "say how far back it was, so --route can pin it"
    assert downloaded == ["https://api.konik.ai/r2"], "must prove the path on an rlog, not a qlog"
    assert rc == 0

  def test_a_route_that_fails_to_list_does_not_abort_the_scan(self, monkeypatch, capsys):
    newest = f"{self.DONGLE}|00000000--deadbeef00"
    good = f"{self.DONGLE}|00000002--deadbeef02"
    files = {newest: {"logs": [], "qlogs": ["https://api.konik.ai/q0"]},
             good: {"logs": ["https://api.konik.ai/r2"], "qlogs": []}}
    pre, asked, downloaded = self._install(monkeypatch, files)
    real = pre.plain_http.request

    def flaky(method, url, *a, **kw):
      if url.endswith("00000001--deadbeef01/files"):
        raise pre.plain_http.HttpError("500 on that one")
      return real(method, url, *a, **kw)

    monkeypatch.setattr(pre.plain_http, "request", flaky)
    assert pre.main() == 0
    assert good in capsys.readouterr().out
    assert downloaded == ["https://api.konik.ai/r2"]

  def test_scan_is_bounded_by_the_flag(self, monkeypatch, capsys):
    all_qlog = {f"{self.DONGLE}|{i:08d}--deadbeef{i:02d}": {"logs": [], "qlogs": ["https://api.konik.ai/q"]}
                for i in range(20)}
    pre, asked, downloaded = self._install(monkeypatch, all_qlog, n_routes=20,
                                           argv=["--scan-routes", "3"])
    pre.main()
    assert len(asked) == 1 + 3, f"scanned {len(asked)-1} routes, --scan-routes said 3"
    out = capsys.readouterr().out
    assert "scanned 3 route(s) back" in out
    assert "WiFi" in out, "tell the user WHY there are no rlogs and what to do"

  def test_no_rlogs_anywhere_is_a_warning_not_a_failure(self, monkeypatch, capsys):
    """The transport is still proven. Downgrading this to a failure would hide that."""
    all_qlog = {f"{self.DONGLE}|{i:08d}--deadbeef{i:02d}": {"logs": [], "qlogs": ["https://api.konik.ai/q"]}
                for i in range(6)}
    pre, _, downloaded = self._install(monkeypatch, all_qlog)
    assert pre.main() == 0
    assert downloaded, "a qlog download still proves the data path"

  def test_newest_route_with_rlogs_does_not_scan_at_all(self, monkeypatch, capsys):
    newest = f"{self.DONGLE}|00000000--deadbeef00"
    pre, asked, downloaded = self._install(monkeypatch, {
      newest: {"logs": ["https://api.konik.ai/r0"], "qlogs": []}})
    assert pre.main() == 0
    assert asked == [newest], "no extra API calls when the newest route already has rlogs"
    assert "route(s) back" not in capsys.readouterr().out

  def test_explicit_route_is_never_second_guessed(self, monkeypatch, capsys):
    """--route means the user picked it. Silently downloading a different one would be wrong."""
    pinned = f"{self.DONGLE}|00000009--pinned"
    pre, asked, downloaded = self._install(
      monkeypatch, {pinned: {"logs": [], "qlogs": ["https://api.konik.ai/qp"]}},
      argv=["--route", pinned])
    assert pre.main() == 0
    assert asked == [pinned]
    assert downloaded == ["https://api.konik.ai/qp"]
    assert "route(s) back" not in capsys.readouterr().out
