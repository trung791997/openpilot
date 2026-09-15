"""Tests for tools/plain_http.py.

This module exists so the Konik tools survive a Python linked against an obsolete TLS stack
(macOS CLT Python 3.9 ships LibreSSL 2.8.3, which cannot negotiate with a modern endpoint and
raises SSLV3_ALERT_HANDSHAKE_FAILURE). It shells out to the system curl in that case.

Two properties are load-bearing and are what these tests pin:

  * a credential passed to curl must NEVER appear in argv, or it is visible to every other
    user on the machine via `ps`. It goes on stdin through `--config -`.
  * the fallback must be narrow. A TLS failure means the local stack cannot talk to the host
    at all; anything else (DNS, refused, HTTP 500) is a real condition and retrying it through
    a second client would just report the same thing more slowly.
"""

import importlib.util
import os
import subprocess

import pytest

_HERE = os.path.dirname(__file__)
_SCRIPT = os.path.normpath(os.path.join(_HERE, "..", "..", "plain_http.py"))


def _load():
  spec = importlib.util.spec_from_file_location("plain_http", _SCRIPT)
  module = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(module)
  return module


ph = _load()
SECRET = "eyJhbGciOiJSUzI1NiJ9.SUPERSECRETTOKEN.sig"


class TestCredentialsStayOffArgv:
  def test_token_is_not_in_the_curl_command_line(self, monkeypatch):
    seen = {}

    def fake_run(argv, **kw):
      seen["argv"] = argv
      seen["stdin"] = kw.get("input")
      return subprocess.CompletedProcess(argv, 0, stdout="{}\n200", stderr="")

    monkeypatch.setattr(ph.subprocess, "run", fake_run)
    ph.curl_request("GET", "https://api.konik.ai/v1/me",
                    headers={"Authorization": f"JWT {SECRET}"})

    joined = " ".join(seen["argv"])
    assert SECRET not in joined, "the token must never reach argv -- ps would show it"
    assert seen["argv"][1:] == ["--config", "-"], "config must come from stdin"
    assert SECRET in seen["stdin"], "the token should be on stdin instead"

  def test_post_body_is_not_in_argv(self, monkeypatch):
    seen = {}

    def fake_run(argv, **kw):
      seen["argv"] = argv
      seen["stdin"] = kw.get("input")
      return subprocess.CompletedProcess(argv, 0, stdout="{}\n200", stderr="")

    monkeypatch.setattr(ph.subprocess, "run", fake_run)
    ph.curl_request("POST", "https://api.konik.ai/v2/auth/", data={"code": "SECRETCODE"})
    assert "SECRETCODE" not in " ".join(seen["argv"])
    assert "SECRETCODE" in seen["stdin"]


class TestCurlConfigEscaping:
  @pytest.mark.parametrize("value", ['has"quote', "has\\backslash", 'both"and\\'])
  def test_quotes_and_backslashes_are_escaped(self, value):
    """An unescaped quote would truncate the config line and silently drop a header."""
    escaped = ph._curl_escape(value)
    assert escaped.count('"') == escaped.count('\\"'), "every quote must be escaped"
    # round-trip: unescaping returns the original
    assert escaped.replace('\\"', '"').replace("\\\\", "\\") == value


class TestFallbackIsNarrow:
  def test_ssl_error_falls_back_to_curl(self, monkeypatch):
    import requests
    monkeypatch.setattr(ph, "curl_request", lambda *a, **k: (200, "ok"))

    def boom(*a, **k):
      raise requests.exceptions.SSLError("handshake failure")

    monkeypatch.setattr(requests, "request", boom)
    code, text, transport = ph.request("GET", "https://api.konik.ai/v1/me")
    assert (code, text, transport) == (200, "ok", "curl")

  def test_non_ssl_errors_are_not_retried_through_curl(self, monkeypatch):
    import requests
    called = {"curl": False}

    def mark(*a, **k):
      called["curl"] = True
      return (200, "ok")

    monkeypatch.setattr(ph, "curl_request", mark)

    def boom(*a, **k):
      raise requests.exceptions.ConnectionError("name resolution failed")

    monkeypatch.setattr(requests, "request", boom)
    with pytest.raises(requests.exceptions.ConnectionError):
      ph.request("GET", "https://api.konik.ai/v1/me")
    assert not called["curl"], "a DNS/connection failure is real; curl would only repeat it"

  def test_http_errors_are_returned_not_retried(self, monkeypatch):
    called = {"curl": False}
    monkeypatch.setattr(ph, "curl_request",
                        lambda *a, **k: (called.__setitem__("curl", True), (500, "x"))[1])
    code, _, transport = ph.request("GET", "https://pypi.org/pypi/requests/json")
    assert transport == "requests" and code == 200
    assert not called["curl"]


class TestStatusParsing:
  def test_status_is_split_off_the_body(self, monkeypatch):
    monkeypatch.setattr(ph.subprocess, "run",
                        lambda a, **k: subprocess.CompletedProcess(a, 0, stdout='{"a":1}\n404', stderr=""))
    code, body = ph.curl_request("GET", "https://example.invalid/")
    assert code == 404
    assert body == '{"a":1}'

  def test_body_containing_newlines_survives(self, monkeypatch):
    monkeypatch.setattr(ph.subprocess, "run",
                        lambda a, **k: subprocess.CompletedProcess(a, 0, stdout="line1\nline2\n200", stderr=""))
    code, body = ph.curl_request("GET", "https://example.invalid/")
    assert code == 200 and body == "line1\nline2"

  def test_curl_failure_raises_with_stderr(self, monkeypatch):
    monkeypatch.setattr(ph.subprocess, "run",
                        lambda a, **k: subprocess.CompletedProcess(a, 6, stdout="", stderr="could not resolve host"))
    with pytest.raises(ph.HttpError, match="could not resolve host"):
      ph.curl_request("GET", "https://example.invalid/")

  def test_missing_status_raises(self, monkeypatch):
    monkeypatch.setattr(ph.subprocess, "run",
                        lambda a, **k: subprocess.CompletedProcess(a, 0, stdout="no status here", stderr=""))
    with pytest.raises(ph.HttpError):
      ph.curl_request("GET", "https://example.invalid/")


class TestDownloadRange:
  def test_range_header_bounds_the_fetch(self, monkeypatch):
    seen = {}

    def fake_run(argv, **kw):
      seen["stdin"] = kw.get("input")
      return subprocess.CompletedProcess(argv, 0, stdout=b"x" * 10, stderr=b"")

    monkeypatch.setattr(ph.subprocess, "run", fake_run)
    ph.curl_download("https://example.invalid/rlog.zst", 4096)
    assert 'range = "0-4095"' in seen["stdin"].decode()
    assert "location" in seen["stdin"].decode(), "signed URLs redirect; curl must follow"


@pytest.mark.skipif(not os.environ.get("PLAIN_HTTP_NETWORK_TESTS"),
                    reason="set PLAIN_HTTP_NETWORK_TESTS=1 to run the live curl check")
class TestLive:
  def test_real_curl_round_trip(self):
    code, body = ph.curl_request("GET", "https://pypi.org/pypi/requests/json")
    assert code == 200 and "info" in body

  def test_real_curl_ranged_download(self):
    assert len(ph.curl_download("https://pypi.org/pypi/requests/json", 1024)) == 1024
