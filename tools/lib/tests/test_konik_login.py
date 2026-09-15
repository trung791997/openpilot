"""Pin tools/konik_login.py against tools/lib/auth.py.

konik_login.py deliberately re-implements the OAuth flow with no openpilot imports, so it
works on a laptop with only `requests` installed — the import chain in auth.py reaches cereal
and dies on `No module named 'capnp'` before the flow can run.

That duplication is the thing AGENTS.md §6 warns about, and these tests are the price of it:
they run on a checkout that CAN import auth.py and assert every mirrored constant and every
generated URL still matches. If one fails, konik_login.py is stale — fix it there.
"""

import importlib.util
import json
import os

import pytest

_HERE = os.path.dirname(__file__)
_SCRIPT = os.path.normpath(os.path.join(_HERE, "..", "..", "konik_login.py"))


def _load():
  spec = importlib.util.spec_from_file_location("konik_login", _SCRIPT)
  module = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(module)
  return module


login = _load()


class TestMirroredConstants:
  def test_api_hosts_match_auth_config(self):
    from openpilot.tools.lib import auth_config
    assert login.DEFAULT_API_HOST == auth_config.DEFAULT_API_HOST
    assert login.KONIK_API_HOST == auth_config.KONIK_API_HOST
    assert login.COMMA_API_HOST_ALIASES == auth_config.COMMA_API_HOST_ALIASES

  def test_port_matches_auth(self):
    from openpilot.tools.lib import auth
    assert login.PORT == auth.PORT, "the redirect listener port is baked into the OAuth state"

  @pytest.mark.parametrize("host,expected", [
    ("konik", "https://api.konik.ai"),
    ("comma", "https://api.commadotai.com"),
    ("commaai", "https://api.commadotai.com"),
    ("https://api.comma.ai", "https://api.commadotai.com"),
    ("https://api.example.com", "https://api.example.com"),
  ])
  def test_host_resolution_matches_auth(self, host, expected):
    from openpilot.tools.lib import auth
    assert login.resolve_api_host(host) == expected
    assert login.resolve_api_host(host) == auth.resolve_api_host(host)

  @pytest.mark.parametrize("host", [
    "https://api.konik.ai", "https://api.commadotai.com", "api.konik.ai",
    "https://API.Konik.AI/", "https://api.comma.ai",
  ])
  def test_normalisation_matches_auth_config(self, host):
    from openpilot.tools.lib.auth_config import normalize_api_host
    assert login.normalize_api_host(host) == normalize_api_host(host)


class TestRedirectUrls:
  """The OAuth URL is where a stale client ID would silently break login."""

  @pytest.mark.parametrize("method", ["github", "google", "apple"])
  @pytest.mark.parametrize("host", ["https://api.konik.ai", "https://api.commadotai.com"])
  def test_redirect_link_matches_auth(self, method, host):
    from openpilot.tools.lib import auth
    assert login.auth_redirect_link(method, host) == auth.auth_redirect_link(method, host)

  def test_konik_uses_its_own_github_client_id(self):
    url = login.auth_redirect_link("github", "https://api.konik.ai")
    assert login.GITHUB_CLIENT_ID_KONIK in url
    assert login.GITHUB_CLIENT_ID_COMMA not in url

  def test_comma_uses_the_comma_github_client_id(self):
    url = login.auth_redirect_link("github", "https://api.commadotai.com")
    assert login.GITHUB_CLIENT_ID_COMMA in url
    assert login.GITHUB_CLIENT_ID_KONIK not in url

  def test_comma_redirect_uses_api_comma_ai(self):
    from openpilot.tools.lib import auth
    assert login.auth_redirect_api_host("https://api.commadotai.com") == \
           auth.auth_redirect_api_host("https://api.commadotai.com")


class TestPortSelection:
  """3000 is commonly taken. The port travels in `state`, so it must reach the URL."""

  @pytest.mark.parametrize("port", [3000, 8976, 49152])
  def test_port_appears_in_the_oauth_state(self, port):
    url = login.auth_redirect_link("github", "https://api.konik.ai", port)
    assert f"localhost%3A{port}" in url or f"localhost:{port}" in url

  def test_default_port_still_matches_auth(self):
    """The default must stay byte-identical to auth.py; only an explicit port may differ."""
    from openpilot.tools.lib import auth
    assert login.auth_redirect_link("github", "https://api.konik.ai") == \
           auth.auth_redirect_link("github", "https://api.konik.ai")

  def test_non_default_port_changes_only_the_state(self):
    from urllib.parse import parse_qs, urlparse
    a = parse_qs(urlparse(login.auth_redirect_link("github", "https://api.konik.ai", 3000)).query)
    b = parse_qs(urlparse(login.auth_redirect_link("github", "https://api.konik.ai", 8976)).query)
    assert a["state"] != b["state"]
    for k in a:
      if k != "state":
        assert a[k] == b[k], f"changing the port must not disturb {k}"

  def test_busy_port_reports_the_alternative_rather_than_traceback(self, capsys):
    """A taken port is the expected case here, so it must fail with advice, not a stack."""
    import socket
    s = socket.socket()
    s.bind(("localhost", 0))
    s.listen(1)
    taken = s.getsockname()[1]
    try:
      assert login.login("github", "https://api.konik.ai", 0.1, taken) is None
      err = capsys.readouterr().err
      assert "--port" in err and str(taken) in err
    finally:
      s.close()

  def test_port_zero_binds_a_real_ephemeral_port(self):
    """With --port 0 the kernel picks it, and the REAL port must go into state."""
    import socket
    probe = socket.socket()
    probe.bind(("localhost", 0))
    free = probe.getsockname()[1]
    probe.close()
    url = login.auth_redirect_link("github", "https://api.konik.ai", free)
    assert "localhost%3A0" not in url and "localhost:0" not in url
    assert str(free) in url


class TestTokenFile:
  """The file this writes is the file konik_preflight.py and auth_config.get_token read."""

  def test_written_shape_is_readable_by_auth_config(self, tmp_path, monkeypatch):
    monkeypatch.setattr(login, "auth_path", lambda: str(tmp_path / "auth.json"))
    login.write_token("konik-tok", "https://api.konik.ai")
    data = json.loads((tmp_path / "auth.json").read_text())
    assert data["tokens"]["https://api.konik.ai"] == "konik-tok"

  def test_konik_token_does_not_clobber_an_existing_comma_token(self, tmp_path, monkeypatch):
    """Mirrors set_token's comment: adding a Konik token must not replace a comma one."""
    p = tmp_path / "auth.json"
    monkeypatch.setattr(login, "auth_path", lambda: str(p))
    p.write_text(json.dumps({"tokens": {login.DEFAULT_API_HOST: "comma-tok"},
                             "access_token": "comma-tok"}))
    login.write_token("konik-tok", "https://api.konik.ai")
    data = json.loads(p.read_text())
    assert data["tokens"][login.DEFAULT_API_HOST] == "comma-tok"
    assert data["tokens"]["https://api.konik.ai"] == "konik-tok"
    assert data["access_token"] == "comma-tok", "the legacy field must stay on comma"

  def test_comma_token_sets_the_legacy_field(self, tmp_path, monkeypatch):
    p = tmp_path / "auth.json"
    monkeypatch.setattr(login, "auth_path", lambda: str(p))
    login.write_token("comma-tok", "https://api.commadotai.com")
    assert json.loads(p.read_text())["access_token"] == "comma-tok"

  def test_file_is_not_world_readable(self, tmp_path, monkeypatch):
    p = tmp_path / "auth.json"
    monkeypatch.setattr(login, "auth_path", lambda: str(p))
    login.write_token("tok", "https://api.konik.ai")
    assert (os.stat(p).st_mode & 0o077) == 0, "auth.json holds a credential"

  def test_preflight_reads_what_login_writes(self, tmp_path, monkeypatch):
    """End to end across the two tools: the preflight must find this token."""
    spec = importlib.util.spec_from_file_location(
      "konik_preflight", os.path.normpath(os.path.join(_HERE, "..", "..", "konik_preflight.py")))
    pre = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(pre)

    p = tmp_path / "auth.json"
    monkeypatch.setattr(login, "auth_path", lambda: str(p))
    login.write_token("round-trip-tok", "https://api.konik.ai")
    monkeypatch.delenv("KONIK_TOKEN", raising=False)
    monkeypatch.delenv("COMMA_TOKEN", raising=False)
    monkeypatch.setattr(os.path, "expanduser", lambda s: str(tmp_path) if s == "~/.comma" else s)
    monkeypatch.setattr(pre, "_token", pre._token)  # keep the real implementation

    with open(p) as f:
      auth = json.load(f)
    assert auth.get("tokens", {}).get("https://api.konik.ai") == "round-trip-tok"


class TestNoHeavyImports:
  def test_module_does_not_import_openpilot_or_capnp(self):
    """The entire point: this must load with only requests + stdlib available."""
    src = open(_SCRIPT).read()
    for banned in ("from openpilot", "import openpilot", "import capnp", "from cereal"):
      assert banned not in src, f"konik_login.py must not contain '{banned}'"
