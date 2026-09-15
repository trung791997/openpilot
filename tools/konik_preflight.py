#!/usr/bin/env python3
"""Verify a live Konik server end to end, from auth through to real radar bytes.

Why this exists
---------------
Claude Code's remote containers cannot reach konik.ai -- the environment's network policy
answers 403 to CONNECT for konik.ai, api.konik.ai and api.commadotai.com -- so the server
side of the Konik path cannot be verified from an agent session. This script is the part
that has to run somewhere with real network access: your laptop, or the comma itself.

Run it, paste the output back into the agent session, and the agent has evidence rather
than an assumption.

What it checks, in order, failing fast with a specific reason
-------------------------------------------------------------
  1. reachability   DNS + TLS + HTTP to the API host
  2. auth           a token exists for THIS host and has not expired
  3. identity       GET /v1/me                     -- the token is accepted
  4. devices        GET /v1/me/devices/            -- at least one device is visible
  5. routes         GET /v1/devices/<id>/routes_segments
  6. files          GET /v1/route/<route>/files    -- rlog URLs are issued
  7. download       range-GET the first rlog       -- the DATA path works, not just metadata
  8. radar          decode it and report whether Bosch-A radar is actually present

Step 8 is the one that matters for this repo. A route that uploads cleanly but contains no
Bosch-A object frames is not a radar route, and finding that out here costs seconds instead
of a round trip through an agent session.

Usage
-----
    python tools/konik_preflight.py                      # full check, newest route
    python tools/konik_preflight.py --route <name>       # a specific route
    python tools/konik_preflight.py --host https://api.commadotai.com
    python tools/konik_preflight.py --no-download        # metadata only, no bytes
    python tools/konik_preflight.py --json               # machine-readable summary

Auth: uses the token stored by `python tools/lib/auth.py` for the selected host, or
$KONIK_TOKEN / $COMMA_TOKEN if set. Nothing here writes or transmits credentials; the token
is sent only to the host you name, and is never printed.
"""

from __future__ import annotations

import argparse
import base64
import json
import os
import socket
import sys
import time
from datetime import UTC, datetime
from urllib.parse import urlparse

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import plain_http  # gives us a curl fallback on an obsolete local TLS stack

try:
  import requests
except ImportError:
  print("FAIL  requests is not installed.  python3 -m pip install requests", file=sys.stderr)
  sys.exit(2)

KONIK_API_HOST = "https://api.konik.ai"

# Bosch-A object-bank CAN IDs, mirrored from opendbc/car/honda/radar_interface.py so this
# script still works from a checkout that has not been built. Verified against the parser at
# import time when the parser is importable (see _bosch_a_ids).
_FALLBACK_MAIN = [0x280 + 4 * s + i for s in range(4) for i in range(4)] + \
                 [0x2D0 + 4 * (s - 4) + i for s in range(4, 16) for i in range(4)]
_FALLBACK_AUX = [0x2C8 + s for s in range(8)] + [0x290 + (s - 8) for s in range(8, 16)]

PASS, FAIL, WARN, SKIP = "PASS", "FAIL", "WARN", "SKIP"

# User-facing diagnostics. Kept here as single strings: the repo's ruff config bans
# implicitly concatenated literals (ISC002), and these read better in one place anyway.
MSG_UNREACHABLE = (
  "If this is an agent container, the network policy likely denies this host."
)
MSG_NO_TOKEN = (
  "Run `python tools/lib/auth.py` or set $KONIK_TOKEN."
  + " (The token is never printed by this script.)"
)
MSG_NO_DEVICES = (
  "no devices on this account. Has the comma registered against this server?"
  + " Check UseKonikServer=1 and KonikDongleId on the device."
)
MSG_NO_ROUTES = (
  "Has the device uploaded anything? Check the upload queue on the comma."
)
MSG_QLOGS_ONLY = (
  "qlogs only. qlogs are decimated and DROP the CAN data this repo needs --"
  + " Bosch-A radar analysis requires rlogs."
)
MSG_DOWNLOAD_FAIL = (
  "Metadata worked but the data path did not -- often an expired signed URL,"
  + " or a storage host the network policy does not allow. The host is named above;"
  + " allow it alongside the API host and re-run."
)
MSG_NO_DECODE = (
  "run from a built checkout to decode. Transport is verified regardless."
)
MSG_NO_BOSCH = (
  "No Bosch-A object frames in this segment. Either this is not a Bosch-A car, the camera"
  + " bus was not logged, or the radar was quiet. This route will NOT advance the radar work"
  + " -- check before sending it on."
)
MSG_SENTINELS = (
  "Bosch-A frames present but radarState never reported a lead. Likely all no-target"
  + " sentinels -- see STATUS.md; every clean replay so far has been sentinels. A route with"
  + " real targets is the open gap."
)


class Check:
  def __init__(self):
    self.rows: list[tuple[str, str, str]] = []
    self.failed = False

  def add(self, status: str, name: str, detail: str = "") -> None:
    self.rows.append((status, name, detail))
    marker = {PASS: "  ok ", FAIL: "FAIL ", WARN: "warn ", SKIP: "skip "}[status]
    print(f"{marker} {name}" + (f"\n         {detail}" if detail else ""), flush=True)
    if status == FAIL:
      self.failed = True

  def bail(self, name: str, detail: str) -> None:
    self.add(FAIL, name, detail)
    self.finish()
    sys.exit(1)

  def finish(self) -> None:
    n_pass = sum(1 for s, _, _ in self.rows if s == PASS)
    n_fail = sum(1 for s, _, _ in self.rows if s == FAIL)
    n_warn = sum(1 for s, _, _ in self.rows if s == WARN)
    print("\n" + "-" * 72)
    print(f"{n_pass} passed, {n_fail} failed, {n_warn} warnings")
    if n_fail:
      print("Konik path is NOT verified. Fix the first FAIL above and re-run.")
    else:
      print("Konik path verified. Paste this output into the agent session.")


def _bosch_a_ids() -> tuple[set[int], str]:
  """Bosch-A CAN IDs, preferring the real parser over the mirrored fallback."""
  try:
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "opendbc_repo"))
    from opendbc.car.honda.radar_interface import BOSCH_A_ALL_IDS  # type: ignore
    return set(BOSCH_A_ALL_IDS), "parser"
  except Exception:
    return set(_FALLBACK_MAIN) | set(_FALLBACK_AUX), "fallback table"


def _token(host: str) -> str | None:
  for env in ("KONIK_TOKEN", "COMMA_TOKEN"):
    if os.environ.get(env):
      return os.environ[env]
  try:
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    from openpilot.tools.lib.auth_config import get_token  # type: ignore
    return get_token(host)
  except Exception:
    pass
  # Fall back to reading auth.json directly, so this runs on a machine with no build.
  # Paths.config_root() is ~/.comma on a PC and /tmp/.comma on the device (system/hardware/hw.py).
  for path in (os.path.expanduser("~/.comma/auth.json"), "/tmp/.comma/auth.json"):
    try:
      with open(path) as f:
        auth = json.load(f)
      norm = host.rstrip("/").lower()
      return auth.get("tokens", {}).get(norm) or auth.get("access_token")
    except Exception:
      continue
  return None


def _jwt_expiry(token: str) -> datetime | None:
  try:
    payload = token.split(".")[1]
    payload += "=" * (-len(payload) % 4)
    exp = json.loads(base64.urlsafe_b64decode(payload)).get("exp")
    return datetime.fromtimestamp(exp, UTC) if exp else None
  except Exception:
    return None


def main() -> int:
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("--host", default=os.environ.get("API_HOST", KONIK_API_HOST))
  ap.add_argument("--route", help="route name, e.g. <dongle_id>|00000001--abcdef1234")
  ap.add_argument("--dongle-id", help="skip device discovery and use this device")
  ap.add_argument("--no-download", action="store_true", help="metadata only; do not fetch rlog bytes")
  ap.add_argument("--max-bytes", type=int, default=8 << 20, help="cap on rlog bytes fetched (default 8 MiB)")
  ap.add_argument("--timeout", type=float, default=30.0)
  ap.add_argument("--json", action="store_true", help="emit a machine-readable summary at the end")
  args = ap.parse_args()

  host = args.host.rstrip("/")
  netloc = urlparse(host).netloc or host
  c = Check()
  summary: dict = {"host": host, "checked_at": datetime.now(UTC).isoformat()}

  print(f"Konik preflight against {host}\n" + "=" * 72)

  # 1. reachability ---------------------------------------------------------
  try:
    socket.getaddrinfo(netloc.split(":")[0], 443, proto=socket.IPPROTO_TCP)
  except OSError as e:
    c.bail("reachability: DNS", f"cannot resolve {netloc}: {e}")
  try:
    t0 = time.monotonic()
    code, _, transport = plain_http.request("GET", f"{host}/", timeout=args.timeout)
    c.add(PASS, "reachability",
          f"{netloc} answered HTTP {code} in {1000*(time.monotonic()-t0):.0f} ms via {transport}")
  except requests.exceptions.SSLError as e:
    c.bail("reachability: TLS", f"TLS failed for {netloc}: {e}")
  except requests.exceptions.RequestException as e:
    c.bail("reachability", f"cannot reach {netloc}: {e}\n         " + MSG_UNREACHABLE)

  # 2. auth -----------------------------------------------------------------
  token = _token(host)
  if not token:
    c.bail("auth: token", f"no token for {host}. " + MSG_NO_TOKEN)
  exp = _jwt_expiry(token)
  if exp and exp < datetime.now(UTC):
    c.bail("auth: token", f"token expired at {exp.isoformat()}. Re-authenticate.")
  c.add(PASS, "auth: token", f"present, {len(token)} chars" + (f", expires {exp.isoformat()}" if exp else ""))

  sess = requests.Session()
  sess.headers.update({"Authorization": f"JWT {token}", "User-Agent": "konik-preflight"})

  auth_headers = {"Authorization": f"JWT {token}", "User-Agent": "konik-preflight"}

  def api(path: str):
    code, text, _ = plain_http.request("GET", f"{host}/{path.lstrip('/')}",
                                       headers=auth_headers, timeout=args.timeout)
    if code in (401, 403):
      raise PermissionError(f"{code} on /{path.lstrip('/')} -- token rejected for this host")
    if code >= 400:
      raise plain_http.HttpError(f"{code} on /{path.lstrip('/')}: {text.strip()[:200]}")
    return json.loads(text)

  # 3. identity -------------------------------------------------------------
  try:
    me = api("v1/me")
    c.add(PASS, "identity", f"GET /v1/me -> id={me.get('id')} email={me.get('email', '(hidden)')}")
    summary["identity"] = me.get("id")
  except PermissionError as e:
    c.bail("identity", str(e))
  except Exception as e:
    c.bail("identity", f"GET /v1/me failed: {e}")

  # 4. devices --------------------------------------------------------------
  dongle_id = args.dongle_id
  if dongle_id:
    c.add(SKIP, "devices", f"using --dongle-id {dongle_id}")
  else:
    try:
      devices = api("v1/me/devices/") or []
    except Exception as e:
      c.bail("devices", f"GET /v1/me/devices/ failed: {e}")
    if not devices:
      c.bail("devices", MSG_NO_DEVICES)
    dongle_id = devices[0].get("dongle_id")
    names = ", ".join(f"{d.get('dongle_id')}({d.get('alias') or 'no alias'})" for d in devices[:4])
    c.add(PASS, "devices", f"{len(devices)} device(s): {names}")
  summary["dongle_id"] = dongle_id

  # 5. routes ---------------------------------------------------------------
  route_name = args.route
  if not route_name:
    try:
      end = int(datetime.now(UTC).timestamp() * 1000)
      start = end - 90 * 24 * 3600 * 1000
      routes = api(f"v1/devices/{dongle_id}/routes_segments?start={start}&end={end}") or []
    except Exception as e:
      c.bail("routes", f"route listing failed: {e}")
    if not routes:
      c.bail("routes", f"no routes in the last 90 days for {dongle_id}. " + MSG_NO_ROUTES)
    routes.sort(key=lambda r: r.get("start_time_utc_millis", 0), reverse=True)
    route_name = routes[0].get("fullname") or routes[0].get("canonical_name")
    when = routes[0].get("start_time_utc_millis")
    when_s = datetime.fromtimestamp(when / 1000, UTC).isoformat() if when else "unknown"
    c.add(PASS, "routes", f"{len(routes)} route(s); newest {route_name} at {when_s}")
  else:
    c.add(SKIP, "routes", f"using --route {route_name}")
  summary["route"] = route_name

  # 6. files ----------------------------------------------------------------
  try:
    files = api(f"v1/route/{route_name.replace('/', '|')}/files")
  except Exception as e:
    c.bail("files", f"file listing failed for {route_name}: {e}")
  rlogs = files.get("logs") or []
  qlogs = files.get("qlogs") or []
  if not rlogs and not qlogs:
    c.bail("files", f"{route_name} has no logs or qlogs. Upload may be incomplete.")
  c.add(PASS, "files", f"{len(rlogs)} rlog segment(s), {len(qlogs)} qlog segment(s)")
  summary["n_rlogs"], summary["n_qlogs"] = len(rlogs), len(qlogs)
  if not rlogs:
    c.add(WARN, "files: rlogs", MSG_QLOGS_ONLY)

  # 7. download -------------------------------------------------------------
  if args.no_download:
    c.add(SKIP, "download", "--no-download")
    c.finish()
    return 1 if c.failed else 0
  target = (rlogs or qlogs)[0]
  # Name the storage host BEFORE trying it. Log files are served from signed URLs that
  # usually point at object storage or a CDN, NOT at the API host -- so an environment that
  # allows only api.konik.ai gets metadata working and the data path blocked. Printing the
  # host means a network policy that needs widening is fixed in one round trip, not two.
  dl_host = urlparse(target).netloc or "(unknown)"
  c.add(PASS, "download: host", f"segments are served from {dl_host}"
        + ("" if dl_host == netloc else f" -- NOT the API host ({netloc}). Both must be reachable."))
  summary["download_host"] = dl_host
  try:
    t0 = time.monotonic()
    blob, dl_transport = plain_http.download(target, args.max_bytes, args.timeout)
    dt = time.monotonic() - t0
  except Exception as e:
    c.bail("download", f"could not fetch from {dl_host}: {e}\n         " + MSG_DOWNLOAD_FAIL)
  c.add(PASS, "download",
        f"{len(blob)/1e6:.1f} MB in {dt:.1f}s ({len(blob)/1e6/max(dt,1e-3):.1f} MB/s) via {dl_transport}")
  summary["downloaded_bytes"] = len(blob)

  # 8. radar ----------------------------------------------------------------
  # Decoding needs a built cereal. Degrade to SKIP rather than failing the whole run: the
  # transport is already proven by step 7, and that is this script's primary job.
  try:
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    from openpilot.tools.lib.logreader import LogReader  # type: ignore
  except Exception as e:
    c.add(SKIP, "radar content",
          f"cereal/logreader not importable here ({type(e).__name__}); " + MSG_NO_DECODE)
    c.finish()
    return 1 if c.failed else 0

  bosch_ids, ids_src = _bosch_a_ids()
  counts = {"can_bosch_a": 0, "liveTracks": 0, "radarState": 0, "radar_leads": 0, "buses": set()}
  try:
    for msg in LogReader(target):
      w = msg.which()
      if w == "can":
        for f in msg.can:
          if f.address in bosch_ids:
            counts["can_bosch_a"] += 1
            counts["buses"].add(f.src)
      elif w == "liveTracks":
        counts["liveTracks"] += 1
      elif w == "radarState":
        counts["radarState"] += 1
        if msg.radarState.leadOne.status:
          counts["radar_leads"] += 1
  except Exception as e:
    c.add(WARN, "radar content", f"decode stopped early: {type(e).__name__}: {e}")

  buses = sorted(counts["buses"])
  detail = (f"Bosch-A CAN frames: {counts['can_bosch_a']} on bus(es) {buses or 'none'}"
            + f" (IDs from {ids_src}); liveTracks: {counts['liveTracks']};"
            + f" radarState: {counts['radarState']} ({counts['radar_leads']} with a lead)")
  summary["radar"] = {k: (sorted(v) if isinstance(v, set) else v) for k, v in counts.items()}

  if counts["can_bosch_a"] == 0:
    c.add(WARN, "radar content", detail + "\n         " + MSG_NO_BOSCH)
  else:
    c.add(PASS, "radar content", detail)
    if counts["radar_leads"] == 0:
      c.add(WARN, "radar content: targets", MSG_SENTINELS)

  c.finish()
  if args.json:
    print("\n--- json ---")
    print(json.dumps(summary, indent=2, default=str))
  return 1 if c.failed else 0


if __name__ == "__main__":
  sys.exit(main())
