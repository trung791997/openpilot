#!/usr/bin/env python3
"""Offline CAN -> RadarInterface -> model-cycle RadarD comparison against a source snapshot.

Runs with BoschARadarShadow on in the new parser and radard (D-127/D-128); the baseline is a radard
source without the shadow (e.g. `git show origin/iq-nav-on-pr10:selfdrive/controls/radard.py`).
Explicit inputs only; no fetching or route storage. Fixed lead probability threshold .35,
adjacent/human-lane toggles off, newborn semantics on. Preserves logged model/car messages,
uses the last received radar sweep (no lookahead). Summaries are not physical target labels.
"""
import argparse
import hashlib
import json
from pathlib import Path
from time import perf_counter
from types import ModuleType, SimpleNamespace

import numpy as np

from cereal import car
from openpilot.common.params import Params
from openpilot.selfdrive.controls import radard
from openpilot.tools.lib.logreader import _LogFileReader
from opendbc.car.can_definitions import CanData
from opendbc.car.honda.interface import CarInterface
from opendbc.car.honda.radar_interface import BOSCH_A_ALL_IDS
from opendbc.car.honda.values import CAR


class ReplaySM:
  def __init__(self):
    self.seen = {'modelV2': False}
    self.recv_frame = dict(carState=0, liveTracks=0)
    self.logMonoTime = dict(carState=0, liveTracks=0, modelV2=0)
    self.data = {'starpilotPlan': SimpleNamespace(increasedStoppedDistance=0.0)}

  def __getitem__(self, key):
    return self.data[key]

  def all_checks(self):
    return True  # availability test only; not real SubMaster frequency/validity validation


def sha(path):
  return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def replay(route, path, baseline):
  sm = ReplaySM()
  new, old = radard.RadarD(honda_bosch_a_radar=True, bosch_a_shadow=True), baseline.RadarD(honda_bosch_a_radar=True)
  ri = None
  rr = car.RadarData.new_message()
  rr.init('points', 0)
  frames, pending_time = [], 0
  counts = dict(model_cycles=0, radar_sweeps=0, can_error_sweeps=0, selected_state_mismatches=0,
                path_rows=0, valid_path_rows=0, extra_corridor_overlaps=0, evaluated_matches=0,
                changed_matches=0, changed_matches_fresh=0, coast_or_duplicate_valid_edges=0)
  examples, sizes, times = [], [], []
  original = radard.bosch_a_overlap_shadow

  def timed(*args):
    start = perf_counter()
    result = original(*args)
    times.append(1000 * (perf_counter() - start))
    return result

  def flush():
    nonlocal rr
    if ri is not None and frames:
      result = ri.update([(pending_time, frames)])
      if result is not None:
        rr = result
        sm.logMonoTime['liveTracks'] = pending_time
        sm.recv_frame['liveTracks'] += 1
        counts['radar_sweeps'] += 1
        counts['can_error_sweeps'] += int(result.errors.canError)
    frames.clear()

  radard.bosch_a_overlap_shadow = timed
  try:
    for msg in _LogFileReader(str(path)):
      t, kind = msg.logMonoTime, msg.which()
      if pending_time != t:
        flush()
        pending_time = t
      if kind == 'carParams' and ri is None:
        fingerprint = str(msg.carParams.carFingerprint)
        cp = CarInterface.get_non_essential_params(getattr(CAR, fingerprint))
        ri = CarInterface.RadarInterface(cp)
        ri.shadow_telemetry = True  # BoschARadarShadow on for the replay (D-127)
      elif kind == 'can':
        frames.extend(CanData(f.address, bytes(f.dat), f.src) for f in msg.can if f.address in BOSCH_A_ALL_IDS)
      elif kind == 'carState':
        sm.data[kind] = msg.carState
        sm.recv_frame[kind] += 1
        sm.logMonoTime[kind] = t
      elif kind == 'modelV2' and 'carState' in sm.data and ri is not None:
        sm.data[kind] = msg.modelV2
        sm.seen[kind] = True
        sm.logMonoTime[kind] = t
        new.update(sm, rr)
        old.update(sm, rr)
        current = new.radar_state.to_dict()
        current.pop('boschAOverlapShadow', None)
        if (repr(current) != repr(old.radar_state.to_dict()) or
            repr(new.starpilot_radar_state.to_dict()) != repr(old.starpilot_radar_state.to_dict()) or
            new.prev_lead_track_ids != old.prev_lead_track_ids):
          counts['selected_state_mismatches'] += 1
        counts['model_cycles'] += 1
        shadow = new.radar_state.boschAOverlapShadow
        sizes.append(len(new.radar_state.to_bytes()) - len(old.radar_state.to_bytes()))
        for row in shadow.tracks:
          counts['path_rows'] += 1
          counts['valid_path_rows'] += int(row.pathValid)
          counts['extra_corridor_overlaps'] += int(row.pathValid and row.overlapM > 0 and abs(row.centerOffset) >= shadow.pathHalfWidth)
          counts['coast_or_duplicate_valid_edges'] += int(row.edgeValid and not shadow.radarFresh)
        for match in shadow.matches:
          counts['evaluated_matches'] += int(match.evaluated)
          counts['changed_matches'] += int(match.changed)
          counts['changed_matches_fresh'] += int(match.changed and shadow.radarFresh)
          if match.changed and shadow.radarFresh and len(examples) < 20:
            point = next((p for p in rr.points if p.trackId == match.shadowTrackId), None)
            overlap = next((r for r in shadow.tracks if r.radarTrackId == match.shadowTrackId), None)
            examples.append(dict(logMonoTime=t, leadIndex=match.leadIndex, existing=match.existingTrackId,
                                 shadow=match.shadowTrackId, selected=match.selectedTrackId,
                                 yawRate=float(sm.data['carState'].yawRate),
                                 point=None if point is None else dict(dRel=point.dRel, yRel=point.yRel,
                                                                      yLeft=point.yLeft, yRight=point.yRight),
                                 pathOverlapM=None if overlap is None or not overlap.pathValid else overlap.overlapM))
    flush()
  finally:
    radard.bosch_a_overlap_shadow = original
  if ri is None or not counts['model_cycles']:
    raise RuntimeError(f'No carParams/model cycles for {route}: {path}')
  return dict(route=route, segment=path.parent.name, input_sha256=sha(path), fingerprint=fingerprint,
              counts=counts, first_changed_examples=examples,
              shadow_compute_ms=dict(zip(('p50', 'p95', 'max'), np.percentile(times, [50, 95, 100]).tolist(), strict=True)),
              added_serialized_bytes=dict(zip(('p50', 'p95', 'max'), np.percentile(sizes, [50, 95, 100]).tolist(), strict=True)))


def main():
  ap = argparse.ArgumentParser(description=__doc__)
  ap.add_argument('--log', nargs=2, action='append', required=True, metavar=('ROUTE', 'PATH'))
  ap.add_argument('--baseline-source', type=Path, required=True)
  ap.add_argument('--output', type=Path, required=True)
  args = ap.parse_args()
  baseline = ModuleType('baseline_radard')
  exec(compile(args.baseline_source.read_text(), str(args.baseline_source), 'exec'), baseline.__dict__)
  toggles = SimpleNamespace(lead_detection_probability=0.35, adjacent_lead_tracking=False, human_lane_changes=False)
  radard.get_starpilot_toggles = baseline.get_starpilot_toggles = lambda *args: toggles
  radard.set_bosch_a_newborn_leads(True)
  baseline.set_bosch_a_newborn_leads(True)
  Params().put_bool('BoschARadar', True)
  root = Path(__file__).resolve().parents[1]
  sources = ['selfdrive/controls/radard.py', 'cereal/log.capnp',
             'opendbc_repo/opendbc/car/honda/radar_interface.py', 'tools/bosch_a_overlap_replay.py']
  result = dict(source_sha256={p: sha(root / p) for p in sources},
                baseline_sha256=sha(args.baseline_source), segments=[])
  args.output.parent.mkdir(parents=True, exist_ok=True)
  for route, path in args.log:
    row = replay(route, Path(path), baseline)
    result['segments'].append(row)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    print(json.dumps(dict(route=route, segment=row['segment'], **row['counts'])), flush=True)
  assert all(r['counts']['selected_state_mismatches'] == r['counts']['coast_or_duplicate_valid_edges'] == 0
             for r in result['segments'])


if __name__ == '__main__':
  main()
