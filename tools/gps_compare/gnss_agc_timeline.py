#!/usr/bin/env python3
"""When does the comma's GPS front end see interference? Prints each jump of the u-blox AGC count
(MON-HW agcCnt, a drop of ~800 means more in-band noise) next to log lines matching a pattern, plus the
4th-best satellite signal per AGC level.

Found 2026-11-01: AGC sits ~4600 with no Jetson, and drops to ~3800 (satellite signal ~8 dB weaker) from the
moment the Jetson USB link carries traffic ("jetlink: Orin ... loaded" ~1 s before the drop).

Usage (on the comma, offroad): /usr/local/venv/bin/python gnss_agc_timeline.py [--pattern REGEX] RLOG [RLOG ...]
  default pattern: "jetlink|gadget|udc"
"""
import argparse
import collections
import os
import re
import sys

import zstandard

sys.path.insert(0, os.environ.get("OPENPILOT_ROOT", "/data/openpilot"))
try:
  from openpilot.cereal import log
except ImportError:
  from cereal import log

MSG = re.compile(r'"msg": ("(?:[^"\\]|\\.)*"|\{[^}]*\})')


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("--pattern", default="jetlink|gadget|udc")
  ap.add_argument("rlogs", nargs="+")
  a = ap.parse_args()
  pat = re.compile(a.pattern, re.I)
  for p in a.rlogs:
    with open(p, "rb") as f:
      dat = zstandard.ZstdDecompressor().decompressobj().decompress(f.read())
    try:
      ev = list(log.Event.read_multiple_bytes(dat))
    except Exception as ex:  # truncated last segment
      print(p, "error", repr(ex)[:80])
      continue
    t0 = ev[0].logMonoTime
    agc = None
    by_agc = collections.defaultdict(list)
    print(p)
    for e in ev:
      t = (e.logMonoTime - t0) / 1e9
      w = e.which()
      if w == "ubloxGnss" and e.ubloxGnss.which() == "hwStatus":
        new = e.ubloxGnss.hwStatus.agcCnt
        if agc is None or abs(new - agc) > 300:
          print(f"  {t:7.1f}  AGC {agc} -> {new}")
        agc = new
      elif w == "ubloxGnss" and e.ubloxGnss.which() == "satReport" and agc is not None:
        c = sorted((s.cno for s in e.ubloxGnss.satReport.svs if s.cno > 0), reverse=True)
        by_agc[agc // 200 * 200].append(c[3] if len(c) > 3 else 0)
      elif w in ("logMessage", "errorLogMessage"):
        s = getattr(e, w)
        if pat.search(s) and "32mloggerd" not in s:
          m = MSG.search(s)
          print(f"  {t:7.1f}  {(m.group(1) if m else s)[:150]}")
    for k in sorted(by_agc):
      v = sorted(by_agc[k])
      print(f"  AGC ~{k}: {len(v)} reports, 4th-best satellite {v[len(v) // 2]} dB-Hz")


if __name__ == "__main__":
  main()
