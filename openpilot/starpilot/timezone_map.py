"""Offline GPS position -> IANA timezone, from the grid tools/scripts/build_timezone_map.py builds.

A zlib-compressed file: header, zone names, then one run-length-encoded row of zone indexes per latitude band.
Lookups decode a single row, so they need no map service, data plan or third-party package.
"""

from functools import lru_cache
from pathlib import Path
import struct
import zlib

MAGIC = b'TZM1'
HEADER = '<HHdddI'  # width, height, west longitude, north latitude, cell degrees, zone-name bytes
DEFAULT_PATH = Path(__file__).with_name('timezone_map.bin')


class TimezoneMap:
  def __init__(self, data: bytes):
    payload = zlib.decompress(data)
    if payload[:4] != MAGIC:
      raise ValueError('Not a timezone map')
    offset = 4 + struct.calcsize(HEADER)
    self.width, self.height, self.west, self.north, self.step, name_bytes = struct.unpack_from(HEADER, payload, 4)
    self.names = [''] + payload[offset:offset + name_bytes].decode().split('\n')
    offset += name_bytes
    self.rows = struct.unpack_from(f'<{self.height + 1}I', payload, offset)
    self.runs = memoryview(payload)[offset + 4 * (self.height + 1):]

  @classmethod
  def load(cls, path: Path = DEFAULT_PATH) -> 'TimezoneMap':
    return cls(Path(path).read_bytes())

  @lru_cache(maxsize=8)  # noqa: B019 -- a handful of rows, for the life of the one loaded map
  def _row(self, row: int) -> tuple[tuple[int, int], ...]:
    return tuple(struct.iter_unpack('<HH', self.runs[self.rows[row]:self.rows[row + 1]]))

  def _index(self, row: int, col: int) -> int:
    row, col = min(max(row, 0), self.height - 1), col % self.width
    start = 0
    for length, index in self._row(row):
      start += length
      if col < start:
        return index
    return 0

  def _cell(self, latitude: float, longitude: float) -> tuple[int, int]:
    return int((self.north - latitude) / self.step), int((longitude - self.west) / self.step)

  def zone_at(self, latitude: float, longitude: float) -> str | None:
    row, col = self._cell(latitude, longitude)
    return self.names[self._index(row, col)] or None

  def clear_zone_at(self, latitude: float, longitude: float) -> str | None:
    """The zone only when it covers this cell and all eight around it, so a border never flips the clock."""
    row, col = self._cell(latitude, longitude)
    indexes = {self._index(row + dr, col + dc) for dr in (-1, 0, 1) for dc in (-1, 0, 1)}
    return self.names[indexes.pop()] or None if len(indexes) == 1 else None
