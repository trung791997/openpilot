#!/usr/bin/env python3
"""Summarize Sentry's parked shake log and replay candidate sensitivity / warning-time pairs.

Copy these off the comma first (read-only):
  /data/starpilot/sentry/shake/*.jsonl      the shake log (openpilot/starpilot/sentry_mode/shake_log.py)
  /data/starpilot/sentry/events/*.json      recorded events (kind, monoTimeNs, wallTimeNs)
Never copy sentry/notifications/: it holds webhook secrets.

  tools/sentry_shake_report.py --shake shake/ --events events/ --labels episodes.csv
  tools/sentry_shake_report.py --shake shake/ --pairs 0.04:1,0.06:2,0.08:3

Episode CSV (header start,end,label): times are ISO 8601 (local to --tz unless they carry an offset) or epoch
seconds; labels are free text such as rumble, getin, door.

Candidate pairs run through the real SentryPolicy from openpilot/starpilot/sentry_mode/policy.py, not a copy of its
rules. The log stores every signed magnitude delta of each 1 s window above 0.02, so the replay feeds the policy a
10 Hz magnitude stream with those exact deltas and zero deltas for the quiet seconds the log skipped (all of which
were <= 0.02, so they cannot count for any sensitivity >= 0.02). Stretches more than 61 s apart are replayed with a
fresh armed policy: the policy's 60 s motion window has reset across such a gap anyway. What the replay does not
model: real disarms (sensor gaps, settings changes, ignition), and sub-tick timing (deltas are spaced evenly in their
second). By default only windows where the real policy was armed are replayed (--include-unarmed for all).
"""

from __future__ import annotations

import argparse
import copy
import csv
import json
import math
import sys
from collections import defaultdict
from datetime import UTC, datetime
from pathlib import Path
from zoneinfo import ZoneInfo

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from openpilot.starpilot.sentry_mode.policy import (
  ARM_DELAY_SECONDS, RESET_TIME_SECONDS, Inputs, MotionSample, SentryPolicy, Settings,
)
from openpilot.starpilot.sentry_mode.shake_log import THRESHOLDS

DEFAULT_PAIRS = [(s, w) for s in (0.03, 0.04, 0.05, 0.06, 0.08, 0.10, 0.12) for w in (0.5, 1.0, 2.0, 3.0)]
SEGMENT_GAP_NS = int((RESET_TIME_SECONDS + 1) * 1e9)
BASE_MAGNITUDE = 100.0  # Far from zero so reconstructed magnitudes never fold through hypot's absolute value.


def load_jsonl(paths: list[Path]) -> tuple[list[dict], list[dict]]:
  windows, heartbeats = [], []
  files = []
  for path in paths:
    files += sorted(path.glob("*.jsonl")) if path.is_dir() else [path]
  for file in files:
    with open(file) as handle:
      for number, raw in enumerate(handle, 1):
        raw = raw.strip()
        if not raw:
          continue
        try:
          line = json.loads(raw)
        except ValueError:
          print(f"skip {file.name}:{number}: not JSON", file=sys.stderr)
          continue
        (heartbeats if line.get("hb") else windows).append(line)
  windows.sort(key=lambda w: w["monoNs"])
  heartbeats.sort(key=lambda h: h["wallNs"])
  return windows, heartbeats


def load_events(path: Path | None) -> list[dict]:
  if path is None:
    return []
  events = []
  for file in sorted(path.glob("*.json")) if path.is_dir() else [path]:
    try:
      value = json.loads(file.read_text())
      events.append({"kind": value["kind"], "wallNs": int(value["wallTimeNs"]), "monoNs": int(value.get("monoTimeNs", -1)),
                     "simulated": bool(value.get("simulated"))})
    except (OSError, ValueError, KeyError, TypeError):
      print(f"skip event {file.name}", file=sys.stderr)
  return [e for e in events if not e["simulated"]]


def parse_time(text: str, tz) -> int:
  text = text.strip()
  try:
    return int(float(text) * 1e9)
  except ValueError:
    pass
  moment = datetime.fromisoformat(text)
  if moment.tzinfo is None:
    moment = moment.replace(tzinfo=tz)
  return int(moment.timestamp() * 1e9)


def load_labels(path: Path | None, tz) -> list[dict]:
  if path is None:
    return []
  with open(path, newline="") as handle:
    return [{"start": parse_time(row["start"], tz), "end": parse_time(row["end"], tz), "label": row["label"].strip(),
             "name": f"{row['start'].strip()}"} for row in csv.DictReader(handle)]


def window_wall(window: dict) -> int:
  # wallNs is stamped when the window closes, about a second after it starts.
  return int(window["wallNs"]) - 1_000_000_000


def local(ns: int, tz) -> datetime:
  return datetime.fromtimestamp(ns / 1e9, tz=UTC).astimezone(tz)


def percentile(values: list[float], q: float) -> float:
  if not values:
    return float("nan")
  ordered = sorted(values)
  return ordered[min(len(ordered) - 1, int(math.ceil(q * len(ordered))) - 1 if q > 0 else 0)]


def fmt(value: float) -> str:
  return "   -  " if value != value else f"{value:6.3f}"


def threshold_header() -> str:
  return " ".join(f">{t:<4g}" for t in THRESHOLDS)


def threshold_counts(windows: list[dict]) -> str:
  totals = [sum(w["over"][i] for w in windows) for i in range(len(THRESHOLDS))]
  return " ".join(f"{t:5d}" for t in totals)


def summary_row(name: str, windows: list[dict], events: list[dict]) -> str:
  maxima = [w["max"] for w in windows]
  kinds = defaultdict(int)
  for event in events:
    kinds[event["kind"]] += 1
  kind_text = ",".join(f"{k}:{v}" for k, v in sorted(kinds.items())) or "-"
  return (f"{name:<22} {len(windows):5d} {fmt(percentile(maxima, 0.5))} {fmt(percentile(maxima, 0.9))} "+
          f"{fmt(max(maxima) if maxima else float('nan'))}  {threshold_counts(windows)}  {kind_text}")


def summary_header(title: str) -> str:
  return f"{title:<22} {'win':>5} {'p50':>6} {'p90':>6} {'max':>6}  {threshold_header()}  events"


def segments(windows: list[dict]) -> list[list[dict]]:
  result: list[list[dict]] = []
  for window in windows:
    if result and window["monoNs"] - result[-1][-1]["monoNs"] <= SEGMENT_GAP_NS:
      result[-1].append(window)
    else:
      result.append([window])
  return result


ARMED: dict[Settings, tuple[SentryPolicy, int, float]] = {}


def armed_policy(settings: Settings) -> tuple[SentryPolicy, int, float]:
  """A real SentryPolicy taken through its 90 s arming on a still magnitude. Returns policy, next tick, magnitude."""
  if settings not in ARMED:
    ARMED[settings] = _arm(settings)
  policy, tick, magnitude = ARMED[settings]
  return copy.copy(policy), tick, magnitude


def _arm(settings: Settings) -> tuple[SentryPolicy, int, float]:
  policy = SentryPolicy(settings)
  tick = 0
  while True:
    now = tick / 10
    decision = policy.update(now, Inputs(True, True, True, True, True, MotionSample(now, (0.0, 0.0, BASE_MAGNITUDE))))
    tick += 1
    if decision.state == "armed" and now >= ARM_DELAY_SECONDS:
      return policy, tick, BASE_MAGNITUDE


def replay(segment: list[dict], settings: Settings) -> list[tuple[str, dict]]:
  """Feed one stretch of logged windows to a fresh armed policy; return (event kind, window that produced it)."""
  policy, tick, magnitude = armed_policy(settings)
  origin = tick / 10  # Policy seconds at the segment's first window.
  start_ns = segment[0]["monoNs"]
  now = origin - 0.1
  fired = []

  def step(at: float, value: float, window: dict | None):
    decision = policy.update(at, Inputs(True, True, True, True, True, MotionSample(at, (0.0, 0.0, value))))
    if decision.event is not None:
      fired.append((decision.event, window))

  for window in segment:
    window_start = origin + (window["monoNs"] - start_ns) / 1e9
    # Quiet seconds the log skipped: still magnitude at 10 Hz (policy needs updates <= 0.5 s apart).
    while now + 0.1 < window_start - 1e-9:
      now += 0.1
      step(now, magnitude, None)
    deltas = window.get("d") or []
    spacing = min(0.1, 1.0 / max(1, len(deltas)))
    at = max(window_start, now + 1e-6)
    for delta in deltas:
      magnitude += delta
      step(at, magnitude, window)
      now = at
      at += spacing
  return fired


def in_span(ns: int, episode: dict) -> bool:
  return episode["start"] <= ns <= episode["end"]


def main() -> int:
  parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  parser.add_argument("--shake", type=Path, nargs="+", required=True, help="shake/ directory or .jsonl files")
  parser.add_argument("--events", type=Path, help="events/ directory copied off the comma")
  parser.add_argument("--labels", type=Path, help="episode CSV: start,end,label")
  parser.add_argument("--tz", default="America/Chicago", help="time zone for hours and naive label times")
  parser.add_argument("--pairs", help="sensitivity:warningSeconds,... (default: a grid)")
  parser.add_argument("--include-unarmed", action="store_true", help="also replay windows logged while not armed")
  args = parser.parse_args()
  tz = ZoneInfo(args.tz)

  windows, heartbeats = load_jsonl(args.shake)
  events = load_events(args.events)
  episodes = load_labels(args.labels, tz)
  if not windows and not heartbeats:
    print("no shake log lines")
    return 1

  print(f"shake log: {len(windows)} windows above 0.02, {len(heartbeats)} heartbeats, {len(events)} recorded events")
  if heartbeats:
    first, last = local(heartbeats[0]["wallNs"], tz), local(heartbeats[-1]["wallNs"], tz)
    dropped = sum(h.get("dropped", 0) for h in heartbeats)
    armed = sum(1 for h in heartbeats if h.get("armed"))
    settings = sorted({(h.get("sens"), h.get("warnN")) for h in heartbeats if "sens" in h})
    print(f"heartbeats {first:%Y-%m-%d %H:%M} .. {last:%Y-%m-%d %H:%M} {args.tz}; armed at {armed}/{len(heartbeats)}; "+
          f"dropped lines {dropped}; live settings (sensitivity, warning samples) {settings}")
  logged_events = [e for w in windows for e in w.get("events", [])]
  if logged_events:
    print(f"events seen by the shake log: {len(logged_events)} "+
          f"({sum(1 for e in logged_events if e['queued'])} queued for recording)")

  print("\nPer hour (" + args.tz + "); counts are samples with |delta| above each threshold")
  print(summary_header("hour"))
  hours: dict[str, list[dict]] = defaultdict(list)
  for window in windows:
    hours[f"{local(window_wall(window), tz):%Y-%m-%d %H}:00"].append(window)
  event_hours: dict[str, list[dict]] = defaultdict(list)
  for event in events:
    event_hours[f"{local(event['wallNs'], tz):%Y-%m-%d %H}:00"].append(event)
  for hour in sorted(set(hours) | set(event_hours)):
    print(summary_row(hour, hours.get(hour, []), event_hours.get(hour, [])))

  if episodes:
    print("\nPer labeled episode")
    print(summary_header("episode"))
    by_label: dict[str, list[dict]] = defaultdict(list)
    for episode in episodes:
      inside = [w for w in windows if in_span(window_wall(w), episode)]
      episode_events = [e for e in events if in_span(e["wallNs"], episode)]
      by_label[episode["label"]].append(episode)
      print(summary_row(f"{episode['label'][:8]} {local(episode['start'], tz):%m-%d %H:%M:%S}", inside, episode_events))
    print("\nPer label: episode peak |delta|")
    for label, group in sorted(by_label.items()):
      peaks = [max((w["max"] for w in windows if in_span(window_wall(w), e)), default=0.0) for e in group]
      print(f"{label:<10} episodes {len(group):3d}  peak min {min(peaks):.3f} p50 {percentile(peaks, 0.5):.3f} "+
            f"max {max(peaks):.3f}")

  pairs = DEFAULT_PAIRS
  if args.pairs:
    pairs = [(float(s), float(w)) for s, w in (item.split(":") for item in args.pairs.split(","))]
  replayed = windows if args.include_unarmed else [w for w in windows if w.get("armed")]
  stretches = segments(replayed)
  labels = sorted({e["label"] for e in episodes})
  print(f"\nCandidate pairs replayed through SentryPolicy ({len(replayed)} {'logged' if args.include_unarmed else 'armed'} "+
        f"windows in {len(stretches)} stretches)")
  label_columns = "".join(f" {label[:12]:>12}" for label in labels)
  print(f"{'sens':>5} {'warn s':>6} {'samples':>7} {'warnings':>8} {'alarms':>6} {'unlabeled':>9}{label_columns}")
  for sensitivity, warning in pairs:
    settings = Settings(sensitivity, warning)
    fired = [item for stretch in stretches for item in replay(stretch, settings)]
    warned_episodes = defaultdict(set)
    unlabeled = 0
    for _kind, window in fired:
      if window is None:
        continue
      hit = False
      for index, episode in enumerate(episodes):
        if in_span(window_wall(window), episode):
          warned_episodes[episode["label"]].add(index)
          hit = True
      unlabeled += not hit
    totals = {label: sum(1 for e in episodes if e["label"] == label) for label in labels}
    columns = "".join(" " + f"{len(warned_episodes[label])}/{totals[label]}".rjust(12) for label in labels)
    print(f"{sensitivity:5.3f} {warning:6.1f} {settings.warning_trigger_count:7d} "+
          f"{sum(1 for k, _ in fired if k == 'warning'):8d} {sum(1 for k, _ in fired if k == 'alarm'):6d} {unlabeled:9d}{columns}")
  if labels:
    print("label columns: episodes with at least one replayed warning or alarm / labeled episodes")
  return 0


if __name__ == "__main__":
  sys.exit(main())
