#!/usr/bin/env python3
"""Build openpilot/starpilot/timezone_map.bin from timezone-boundary-builder's GeoJSON.

The comma looks up its IANA zone from GPS offline with this grid. Input: timezones-with-oceans-now.geojson
(https://github.com/evansiroky/timezone-boundary-builder/releases), so the open sea has an Etc/GMT zone and there
are no gaps along coasts; "now" merges zones whose clocks have agreed since the release, which is all a display needs.

Needs pillow and numpy (development only):
  python3 tools/scripts/build_timezone_map.py combined-with-oceans-now.json openpilot/starpilot/timezone_map.bin
"""
import argparse
import json
import struct
import zlib

import numpy as np
from PIL import Image, ImageDraw

from openpilot.starpilot.timezone_map import MAGIC, HEADER


def rasterize(features, step):
  width, height = round(360 / step), round(180 / step)
  grid = np.zeros((height, width), dtype=np.uint16)
  names = sorted({f['properties']['tzid'] for f in features})
  index = {name: n + 1 for n, name in enumerate(names)}

  def pixels(ring):
    return [((lon + 180.) / step, (90. - lat) / step) for lon, lat in ring]

  for feature in features:
    geometry = feature['geometry']
    polygons = geometry['coordinates'] if geometry['type'] == 'MultiPolygon' else [geometry['coordinates']]
    for polygon in polygons:
      outer = pixels(polygon[0])
      xs, ys = [p[0] for p in outer], [p[1] for p in outer]
      x0, y0 = max(int(min(xs)) - 1, 0), max(int(min(ys)) - 1, 0)
      x1, y1 = min(int(max(xs)) + 2, width), min(int(max(ys)) + 2, height)
      if x1 <= x0 or y1 <= y0:
        continue
      # Per polygon on its own mask: a hole clears only this polygon, never an enclave drawn by another feature.
      mask = Image.new('1', (x1 - x0, y1 - y0), 0)
      draw = ImageDraw.Draw(mask)
      draw.polygon([(x - x0, y - y0) for x, y in outer], fill=1)
      for hole in polygon[1:]:
        draw.polygon([(x - x0, y - y0) for x, y in pixels(hole)], fill=0)
      grid[y0:y1, x0:x1][np.asarray(mask, dtype=bool)] = index[feature['properties']['tzid']]
  return grid, names


def encode(grid, names, step):
  height, width = grid.shape
  runs, offsets = bytearray(), [0]
  for row in grid:
    changes = np.flatnonzero(np.diff(row)) + 1
    starts = np.concatenate(([0], changes))
    lengths = np.diff(np.concatenate((starts, [width])))
    for start, length in zip(starts, lengths, strict=True):
      runs += struct.pack('<HH', int(length), int(row[start]))
    offsets.append(len(runs))
  name_blob = '\n'.join(names).encode()
  payload = (MAGIC + struct.pack(HEADER, width, height, -180., 90., step, len(name_blob)) + name_blob +
             struct.pack(f'<{height + 1}I', *offsets) + bytes(runs))
  return zlib.compress(payload, 9)


def main():
  parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  parser.add_argument('geojson')
  parser.add_argument('output')
  parser.add_argument('--step', type=float, default=0.05, help='cell size in degrees')
  args = parser.parse_args()
  with open(args.geojson) as f:
    features = json.load(f)['features']
  grid, names = rasterize(features, args.step)
  data = encode(grid, names, args.step)
  with open(args.output, 'wb') as f:
    f.write(data)
  print(f'{len(names)} zones, {grid.shape[1]}x{grid.shape[0]} cells, unassigned {int((grid == 0).sum())}, {len(data):,} bytes')


if __name__ == '__main__':
  main()
