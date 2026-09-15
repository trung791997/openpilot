#!/usr/bin/env python3
"""Log in to Konik (or comma) and write auth.json, using only requests + the stdlib.

Why this exists alongside tools/lib/auth.py
-------------------------------------------
`auth.py` imports `Paths` from `openpilot.system.hardware.hw`, which pulls in `cereal`, which
needs `pycapnp`. On a machine without an openpilot build that chain fails at

    ModuleNotFoundError: No module named 'capnp'

before the OAuth flow gets a chance to run. Obtaining a token needs none of that: it is a
browser redirect and two HTTP calls. This script does exactly that and nothing else, so it
works on a bare laptop with `pip install requests`.

This is a deliberate second implementation of one flow, which AGENTS.md §6 otherwise warns
against. It is justified only because the duplication is what removes the dependency, and it
is held in place by tools/lib/tests/test_konik_login.py, which asserts every constant here
still matches `tools/lib/auth.py` on a checkout that can import it. If that test fails, this
file is stale — fix it here, do not relax the test.

Usage
-----
    python3 tools/konik_login.py                    # Konik, GitHub
    python3 tools/konik_login.py --host comma       # comma servers
    python3 tools/konik_login.py --method google
    python3 tools/konik_login.py --print-token      # print instead of writing auth.json
    python3 tools/konik_login.py --port 8976        # if 3000 is already in use
    python3 tools/konik_login.py --port 0           # let the OS pick a free port

Then run the preflight, which reads the file this writes:

    python3 tools/konik_preflight.py

Use `github` on Konik: it is the only provider with a Konik-specific OAuth client ID. The
token is written to ~/.comma/auth.json (0600) and is never printed unless you ask for it.
"""

from __future__ import annotations

import argparse
import json
import os
import stat
import sys
import webbrowser
from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import parse_qs, urlencode, urlparse

try:
  import requests
except ImportError:
  print("FAIL  requests is not installed.  python3 -m pip install requests", file=sys.stderr)
  sys.exit(2)

# Mirrored from tools/lib/auth_config.py and tools/lib/auth.py. test_konik_login.py asserts
# these still agree with the originals.
DEFAULT_API_HOST = "https://api.commadotai.com"
KONIK_API_HOST = "https://api.konik.ai"
COMMA_API_HOST_ALIASES = {"https://api.comma.ai", DEFAULT_API_HOST}
PORT = 3000

GITHUB_CLIENT_ID_KONIK = "Ov23liy0AI1YCd15pypf"
GITHUB_CLIENT_ID_COMMA = "28c4ecb54bb7272cb5a4"
GOOGLE_CLIENT_ID = "45471411055-ornt4svd2miog6dnopve7qtmh5mnu6id.apps.googleusercontent.com"
APPLE_CLIENT_ID = "ai.comma.login"
PROVIDER_ID = {"google": "g", "apple": "a", "github": "h"}


def normalize_api_host(host: str | None) -> str:
  if not host:
    host = DEFAULT_API_HOST
  parsed = urlparse(host if "//" in host else f"https://{host}")
  normalized = f"{parsed.scheme.lower()}://{parsed.netloc.lower()}".rstrip("/")
  return DEFAULT_API_HOST if normalized in COMMA_API_HOST_ALIASES else normalized


def resolve_api_host(host: str) -> str:
  if host in ("comma", "commaai", "commadotai"):
    return DEFAULT_API_HOST
  if host == "konik":
    return KONIK_API_HOST
  return normalize_api_host(host)


def auth_redirect_api_host(api_host: str) -> str:
  # comma's redirect uses api.comma.ai even though the API host is api.commadotai.com.
  return "https://api.comma.ai" if normalize_api_host(api_host) == DEFAULT_API_HOST else normalize_api_host(api_host)


def auth_redirect_link(method: str, api_host: str, port: int = PORT) -> str:
  # The listener port travels in `state`, not in redirect_uri: the provider redirects to the
  # API host, which then bounces to localhost:<port> using this value. So changing the port
  # here is sufficient and needs no OAuth app re-registration.
  params = {
    "redirect_uri": f"{auth_redirect_api_host(api_host)}/v2/auth/{PROVIDER_ID[method]}/redirect/",
    "state": f"service,localhost:{port}",
  }
  if method == "google":
    params.update({"type": "web_server", "client_id": GOOGLE_CLIENT_ID,
                   "response_type": "code",
                   "scope": "https://www.googleapis.com/auth/userinfo.email",
                   "prompt": "select_account"})
    return "https://accounts.google.com/o/oauth2/auth?" + urlencode(params)
  if method == "github":
    client = GITHUB_CLIENT_ID_KONIK if normalize_api_host(api_host) == KONIK_API_HOST else GITHUB_CLIENT_ID_COMMA
    params.update({"client_id": client, "scope": "read:user"})
    return "https://github.com/login/oauth/authorize?" + urlencode(params)
  if method == "apple":
    params.update({"client_id": APPLE_CLIENT_ID, "response_type": "code",
                   "response_mode": "form_post", "scope": "name email"})
    return "https://appleid.apple.com/auth/authorize?" + urlencode(params)
  raise NotImplementedError(f"no redirect implemented for {method}")


class _Redirect(HTTPServer):
  query_params: dict = {}


class _Handler(BaseHTTPRequestHandler):
  def do_GET(self):
    if not self.path.startswith("/auth"):
      self.send_response(204)
      self.end_headers()
      return
    self.server.query_params = parse_qs(self.path.split("?", 1)[-1], keep_blank_values=True)
    self.send_response(200)
    self.send_header("Content-type", "text/plain")
    self.end_headers()
    self.wfile.write(b"Signed in. Return to the terminal.")

  def log_message(self, *args):
    pass


def auth_path() -> str:
  # Paths.config_root() is ~/.comma on a PC; hardcoded here so this file imports nothing.
  return os.path.join(os.path.expanduser("~/.comma"), "auth.json")


def write_token(token: str, api_host: str) -> str:
  """Same on-disk shape as tools/lib/auth_config.set_token, including the legacy field."""
  path = auth_path()
  try:
    with open(path) as f:
      auth = json.load(f)
  except Exception:
    auth = {}
  normalized = normalize_api_host(api_host)
  auth.setdefault("tokens", {})[normalized] = token
  if normalized == DEFAULT_API_HOST or "access_token" not in auth:
    auth["access_token"] = token
  os.makedirs(os.path.dirname(path), exist_ok=True)
  with open(path, "w") as f:
    json.dump(auth, f)
  try:
    os.chmod(path, stat.S_IRUSR | stat.S_IWUSR)   # it is a credential; keep it to this user
  except OSError:
    pass
  return path


def login(method: str, api_host: str, timeout: float, port: int = PORT) -> str | None:
  # Bind first, THEN build the URL: with --port 0 the kernel picks the port and it has to be
  # the real one that goes into `state`, or the redirect lands nowhere.
  try:
    server = _Redirect(("localhost", port), _Handler)
  except OSError as e:
    print(f"Could not listen on localhost:{port}: {e}", file=sys.stderr)
    print("Pick another with --port N, or --port 0 to let the OS choose a free one.",
          file=sys.stderr)
    return None
  port = server.server_address[1]
  url = auth_redirect_link(method, api_host, port)
  print(f"Opening your browser to sign in with {method}.")
  print(f"If it does not open, paste this into a browser:\n\n{url}\n")
  server.timeout = timeout
  try:
    webbrowser.open(url, new=2)
  except Exception:
    pass
  print(f"Waiting for the redirect on localhost:{port} ...")
  while True:
    server.handle_request()
    q = server.query_params
    if "code" in q:
      break
    if "error" in q:
      print(f"Authentication error: {q['error']} {q.get('error_description', '')}", file=sys.stderr)
      return None
    if not q:
      print("Timed out waiting for the browser redirect.", file=sys.stderr)
      return None
  resp = requests.post(f"{normalize_api_host(api_host)}/v2/auth/",
                       data={"code": q["code"][0], "provider": q.get("provider", [PROVIDER_ID[method]])[0]},
                       timeout=30)
  if resp.status_code >= 400:
    print(f"Token exchange failed: {resp.status_code} {resp.text[:300]}", file=sys.stderr)
    return None
  return resp.json().get("access_token")


def main() -> int:
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("--host", default="konik", help="konik, comma, or an API URL")
  ap.add_argument("--method", default="github", choices=sorted(PROVIDER_ID))
  ap.add_argument("--port", type=int, default=PORT,
                  help=f"local port for the OAuth redirect (default {PORT}; 0 picks a free one)")
  ap.add_argument("--timeout", type=float, default=300.0, help="seconds to wait for the redirect")
  ap.add_argument("--print-token", action="store_true",
                  help="print the token instead of writing auth.json (for $KONIK_TOKEN)")
  args = ap.parse_args()

  api_host = resolve_api_host(args.host)
  if normalize_api_host(api_host) == KONIK_API_HOST and args.method != "github":
    print(f"note: '{args.method}' uses comma's OAuth client on Konik and may be rejected; "
          + "github is the provider Konik has its own client ID for.", file=sys.stderr)

  token = login(args.method, api_host, args.timeout, args.port)
  if not token:
    return 1

  sess = requests.Session()
  sess.headers.update({"Authorization": f"JWT {token}", "User-Agent": "konik-login"})
  try:
    r = sess.get(f"{normalize_api_host(api_host)}/v1/me", timeout=30)
    if r.status_code in (401, 403):
      print(f"The server rejected the new token ({r.status_code}).", file=sys.stderr)
      return 1
    r.raise_for_status()
    me = r.json()
  except Exception as e:
    print(f"Could not verify the token against /v1/me: {e}", file=sys.stderr)
    return 1

  print(f"\nAuthenticated to {normalize_api_host(api_host)} as id={me.get('id')} "
        + f"email={me.get('email', '(hidden)')}")
  if args.print_token:
    print("\nexport KONIK_TOKEN='" + token + "'")
  else:
    print(f"Token written to {write_token(token, api_host)}")
    print("\nNext:  python3 tools/konik_preflight.py")
  return 0


if __name__ == "__main__":
  sys.exit(main())
