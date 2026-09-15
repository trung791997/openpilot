#!/usr/bin/env python3
"""Minimal HTTPS with a curl fallback, for machines whose Python has an obsolete TLS stack.

Why this exists
---------------
macOS ships a Command Line Tools Python linked against **LibreSSL 2.8.3** (2018), which has
no TLS 1.3. Against a modern endpoint that produces

    ssl.SSLError: [SSL: SSLV3_ALERT_HANDSHAKE_FAILURE] sslv3 alert handshake failure

with `urllib3` warning `NotOpenSSLWarning: urllib3 v2 only supports OpenSSL 1.1.1+`. No
version of `requests` or `urllib3` fixes that: the failure is below them, in the TLS library
Python itself was linked against. The usual answers are "install Homebrew Python" or "use
python.org's build", both of which mean modifying a machine the user may not want to modify.

The system `curl` on the same machine is maintained separately and negotiates fine. So: try
`requests`, and on a TLS failure specifically, retry the same request through `curl`.

Credentials never reach the process list. curl is invoked with `--config -` and the URL,
headers and body are fed on **stdin**, so a token is not visible in `ps` output. That is the
whole reason this does not just shell out with `-H` and `-d` on the command line.

Scope: this is a fallback for a handful of small JSON calls, not a general HTTP client. It
returns (status_code, text) and nothing else.
"""

from __future__ import annotations

import json as _json
import shutil
import subprocess
from urllib.parse import urlencode


class HttpError(Exception):
  pass


def _curl_escape(value: str) -> str:
  """curl config values are double-quoted; backslash and quote need escaping."""
  return value.replace("\\", "\\\\").replace('"', '\\"')


def curl_request(method: str, url: str, data: dict | None = None,
                 headers: dict | None = None, timeout: float = 30.0) -> tuple[int, str]:
  """One request via the system curl. Secrets go on stdin, never argv."""
  curl = shutil.which("curl")
  if curl is None:
    raise HttpError("curl not found; cannot work around the local TLS stack")

  config = [f'url = "{_curl_escape(url)}"',
            f'request = "{method.upper()}"',
            f'max-time = {int(timeout)}',
            'silent', 'show-error',
            'write-out = "\\n%{http_code}"']
  for k, v in (headers or {}).items():
    config.append(f'header = "{_curl_escape(f"{k}: {v}")}"')
  if data:
    config.append(f'data = "{_curl_escape(urlencode(data))}"')

  proc = subprocess.run([curl, "--config", "-"], input="\n".join(config) + "\n",
                        capture_output=True, text=True, timeout=timeout + 10)
  if proc.returncode != 0:
    raise HttpError(f"curl exited {proc.returncode}: {proc.stderr.strip()[:300]}")

  out = proc.stdout
  # write-out appended "\n<code>" after the body; split it back off.
  idx = out.rfind("\n")
  if idx < 0:
    raise HttpError(f"could not parse curl output: {out[:200]!r}")
  body, code = out[:idx], out[idx + 1:].strip()
  if not code.isdigit():
    raise HttpError(f"curl returned no status code: {out[-200:]!r}")
  return int(code), body


def request(method: str, url: str, data: dict | None = None, headers: dict | None = None,
            timeout: float = 30.0, allow_curl_fallback: bool = True) -> tuple[int, str, str]:
  """(status, text, transport). Tries requests; falls back to curl on a TLS failure only.

  The fallback is deliberately narrow: a TLS handshake failure means the local stack cannot
  talk to this host at all, which curl may still manage. Any other error (DNS, connection
  refused, HTTP 500) is a real condition and is raised rather than retried through a second
  client that would only report the same thing.
  """
  try:
    import requests
  except ImportError:
    if not allow_curl_fallback:
      raise HttpError("requests is not installed") from None
    code, text = curl_request(method, url, data, headers, timeout)
    return code, text, "curl"

  try:
    resp = requests.request(method, url, data=data, headers=headers, timeout=timeout)
    return resp.status_code, resp.text, "requests"
  except requests.exceptions.SSLError as e:
    if not allow_curl_fallback:
      raise
    # LibreSSL 2.8.3 and friends. Say so once, then use curl.
    print(f"  note: local TLS stack could not reach {url.split('/')[2]} ({type(e).__name__}); "
          + "retrying through the system curl", flush=True)
    code, text = curl_request(method, url, data, headers, timeout)
    return code, text, "curl"


def curl_download(url: str, max_bytes: int, timeout: float = 60.0) -> bytes:
  """Fetch up to max_bytes of a binary body via curl. Used for rlog segments."""
  curl = shutil.which("curl")
  if curl is None:
    raise HttpError("curl not found; cannot work around the local TLS stack")
  config = [f'url = "{_curl_escape(url)}"', f'max-time = {int(timeout)}',
            'silent', 'show-error', 'location',
            f'range = "0-{max(max_bytes - 1, 0)}"']
  proc = subprocess.run([curl, "--config", "-"], input=("\n".join(config) + "\n").encode(),
                        capture_output=True, timeout=timeout + 15)
  if proc.returncode != 0:
    raise HttpError(f"curl exited {proc.returncode}: {proc.stderr.decode(errors='replace')[:300]}")
  return proc.stdout


def download(url: str, max_bytes: int, timeout: float = 60.0) -> tuple[bytes, str]:
  """(bytes, transport). Streams via requests; falls back to curl on a TLS failure."""
  try:
    import requests
  except ImportError:
    return curl_download(url, max_bytes, timeout), "curl"
  try:
    blob = b""
    with requests.get(url, timeout=timeout, stream=True) as r:
      r.raise_for_status()
      for chunk in r.iter_content(1 << 20):
        blob += chunk
        if len(blob) >= max_bytes:
          break
    return blob, "requests"
  except requests.exceptions.SSLError:
    print(f"  note: local TLS stack could not reach {url.split('/')[2]}; using the system curl",
          flush=True)
    return curl_download(url, max_bytes, timeout), "curl"


def get_json(url: str, headers: dict | None = None, timeout: float = 30.0):
  """GET returning parsed JSON, or raise HttpError with the status."""
  code, text, _ = request("GET", url, None, headers, timeout)
  if code >= 400:
    raise HttpError(f"{code} on {url}: {text.strip()[:300]}")
  try:
    return _json.loads(text)
  except ValueError as e:
    raise HttpError(f"non-JSON response from {url}: {text[:200]!r}") from e
