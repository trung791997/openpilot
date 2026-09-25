#!/usr/bin/env python3
"""Device-side radar lead-loss census: how often did the car drop its radar lead while the camera
still saw one, and was the lost track still being published by the parser?

What it counts
--------------
A *loss* starts on the first `radarState` frame where `leadOne` is no longer a radar lead, if:
the car was engaged (`carControl.enabled`), the previous radar lead had been held for at least
`--min-held` seconds, and the vision lead (`modelV2.leadsV3[0]`) had prob > `--vision-prob` within
`--vision-max-d` m. It ends when a radar lead returns (any track id), the vision lead goes away, or
the car disengages. Only losses of at least `--min-loss` seconds are reported.

Each loss is split by whether the lost track id was still in `liveTracks` on at least half of its
frames. If it was, `radard` chose not to use it (selection). If it was not, the parser stopped
publishing it: the D-052/D-054/D-055 failure family.

What it is NOT
--------------
Not a parser replay (that is `bosch_a_route_report.py` / `bosch_a_lifecycle_report.py`). It reads
what the DEVICE published, from whatever commit was on the car, so comparing routes compares builds
AND traffic. It is also not the D-054 "followed lead lost" replay metric, which ran two parsers on
the same CAN; do not put the two in one table.

Usage
-----
    python tools/bosch_a_lead_loss.py <route dir> [<route dir> ...] [--json out.json]

A route dir holds numbered segment dirs with rlog.zst / rlog.bz2 / rlog (the tools/konik_fetch.py
layout, e.g. ~/routes/0000026c--10bec2e200/5/rlog.zst).
"""
import argparse
import glob
import json
import os
import sys

MIN_HELD_S = 0.5
MIN_LOSS_S = 1.0
VISION_PROB = 0.5
VISION_MAX_D = 80.0
MAX_DT_S = 0.2   # a gap in radarState (segment boundary, dropped log) never counts as more than this
LOG_NAMES = ("rlog.zst", "rlog.bz2", "rlog")


class LeadLossCensus:
  """Feed it the four message kinds in log order; read `episodes` and the time totals."""

  def __init__(self, min_held_s=MIN_HELD_S, min_loss_s=MIN_LOSS_S, vision_prob=VISION_PROB, vision_max_d=VISION_MAX_D):
    self.min_held_s, self.min_loss_s = min_held_s, min_loss_s
    self.vision_prob, self.vision_max_d = vision_prob, vision_max_d
    self.engaged = False
    self.long_active = False
    self.vision = None          # (prob, x) of the first vision lead
    self.live_ids = set()
    self.cur_id = None          # radar lead track id being held
    self.cur_since = None
    self.loss = None            # open episode
    self.last_t = None
    self.seg = None
    self.episodes = []
    self.engaged_s = self.long_s = self.vision_lead_s = self.radar_lead_s = 0.0

  def car_control(self, enabled, long_active):
    self.engaged, self.long_active = enabled, long_active

  def model(self, prob, x):
    self.vision = None if prob is None else (prob, x)

  def live_tracks(self, ids):
    self.live_ids = set(ids)

  def _close(self, t, why, returned_id):
    self.loss.update(end=t, why=why, returned_id=returned_id)
    if t - self.loss["start"] >= self.min_loss_s:
      self.episodes.append(self.loss)
    self.loss = None

  def radar_state(self, t, status, radar, track_id):
    dt = 0.05 if self.last_t is None else min(t - self.last_t, MAX_DT_S)
    self.last_t = t
    vis_ok = self.vision is not None and self.vision[0] > self.vision_prob and self.vision[1] < self.vision_max_d
    if self.engaged:
      self.engaged_s += dt
      self.long_s += dt if self.long_active else 0.0
      self.vision_lead_s += dt if vis_ok else 0.0
    radar_now = bool(status and radar)

    if self.loss is not None and not radar_now and (not self.engaged or not vis_ok):
      self._close(t, "vision-gone" if self.engaged else "disengaged", None)
      self.cur_id = None

    if radar_now:
      self.radar_lead_s += dt if self.engaged else 0.0
      if self.loss is not None:
        self._close(t, "radar-back", track_id)
      if track_id != self.cur_id:
        self.cur_id, self.cur_since = track_id, t
      return

    if self.loss is None and self.cur_id is not None and self.engaged and vis_ok and t - self.cur_since >= self.min_held_s:
      self.loss = {"seg": self.seg, "start": t, "lost_id": self.cur_id, "frames": 0, "in_tracks": 0,
                   "vision_d": self.vision[1]}
    if self.loss is not None:
      self.loss["frames"] += 1
      self.loss["in_tracks"] += self.cur_id in self.live_ids
    else:
      self.cur_id = None

  def summary(self):
    eps = self.episodes
    parser = [e for e in eps if e["frames"] and e["in_tracks"] / e["frames"] < 0.5]

    def dur(es):
      return sum(e["end"] - e["start"] for e in es)
    return {"engaged_s": self.engaged_s, "long_s": self.long_s, "vision_lead_s": self.vision_lead_s,
            "radar_lead_s": self.radar_lead_s, "losses": len(eps), "loss_s": dur(eps),
            "parser_losses": len(parser), "parser_loss_s": dur(parser),
            "longest_parser_loss_s": max((e["end"] - e["start"] for e in parser), default=0.0)}


def route_logs(route_dir):
  segs = sorted((int(os.path.basename(p)), p) for p in glob.glob(os.path.join(route_dir, "*"))
                if os.path.basename(p).isdigit())
  for seg, p in segs:
    log = next((os.path.join(p, n) for n in LOG_NAMES if os.path.exists(os.path.join(p, n))), None)
    if log is not None:
      yield seg, log


def run_route(route_dir, census=None):
  from openpilot.tools.lib.logreader import LogReader
  c = census or LeadLossCensus()
  for seg, log in route_logs(route_dir):
    c.seg = seg
    for m in LogReader(log):
      w = m.which()
      if w == "carControl":
        c.car_control(m.carControl.enabled, m.carControl.longActive)
      elif w == "modelV2":
        ld = m.modelV2.leadsV3
        c.model(ld[0].prob, ld[0].x[0]) if len(ld) else c.model(None, None)
      elif w == "liveTracks":
        c.live_tracks(pt.trackId for pt in m.liveTracks.points)
      elif w == "radarState":
        l1 = m.radarState.leadOne
        c.radar_state(m.logMonoTime * 1e-9, l1.status, l1.radar, l1.radarTrackId)
  return c


def main(argv=None):
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("routes", nargs="+", help="route directories")
  ap.add_argument("--json", help="write per-route summaries and episodes here")
  ap.add_argument("--min-held", type=float, default=MIN_HELD_S)
  ap.add_argument("--min-loss", type=float, default=MIN_LOSS_S)
  ap.add_argument("--vision-prob", type=float, default=VISION_PROB)
  ap.add_argument("--vision-max-d", type=float, default=VISION_MAX_D)
  args = ap.parse_args(argv)
  out = {}
  for rd in args.routes:
    name = os.path.basename(rd.rstrip("/"))
    c = run_route(rd, LeadLossCensus(args.min_held, args.min_loss, args.vision_prob, args.vision_max_d))
    s = c.summary()
    out[name] = {"summary": s, "episodes": c.episodes}
    h = max(s["engaged_s"] / 3600, 1e-9)
    print("  ".join((
      f"{name}",
      f"engaged {s['engaged_s'] / 60:5.1f} min (long {s['long_s'] / 60:5.1f})",
      f"radar lead {100 * s['radar_lead_s'] / max(s['engaged_s'], 1e-9):5.1f}%",
      f"losses {s['losses']:3d} / {s['loss_s']:6.1f} s ({s['losses'] / h:5.1f}/h, {s['loss_s'] / h:6.1f} s/h)",
      f"parser {s['parser_losses']:3d} / {s['parser_loss_s']:5.1f} s")))
    for e in c.episodes:
      print(f"   seg {e['seg']:>3}  id {e['lost_id']:>3}  {e['end'] - e['start']:4.1f} s  vision d {e['vision_d']:5.1f}" +
            f"  in liveTracks {e['in_tracks']}/{e['frames']}  end {e['why']} {e['returned_id']}  t {e['start']:.2f}")
  if args.json:
    with open(args.json, "w") as f:
      json.dump(out, f, indent=1)
  return 0


if __name__ == "__main__":
  sys.exit(main())
