"""Short H.264 clips to MP4 without re-encoding.

The device has no ffmpeg or PyAV. The livestream encoder already emits H.264 (Annex-B, codec header on keyframes), so
a clip only needs repacking: length-prefixed NAL units in one mdat, sample tables in a moov placed first so players
can start before the whole file arrives. No B-frames (the encoder uses none), so no composition offsets.
"""
import struct

TIMESCALE = 90_000  # video ticks per second
MOVIE_TIMESCALE = 1000
FALLBACK_DURATION = TIMESCALE // 20  # 20 Hz livestream frame
PARAMETER_SETS = (7, 8, 9)  # SPS, PPS, access unit delimiter: kept in avcC or dropped
MATRIX = struct.pack(">9I", 0x00010000, 0, 0, 0, 0x00010000, 0, 0, 0, 0x40000000)


def nal_units(data: bytes) -> list[bytes]:
  units = []
  start = data.find(b"\x00\x00\x01")
  while start >= 0:
    start += 3
    end = data.find(b"\x00\x00\x01", start)
    # A NAL unit never ends in a zero byte; trailing zeros belong to the next 4-byte start code.
    unit = data[start:end if end >= 0 else len(data)].rstrip(b"\x00")
    if unit:
      units.append(unit)
    start = end
  return units


def parameter_sets(*sources: bytes) -> tuple[bytes, bytes] | None:
  sps = pps = None
  for source in sources:
    for unit in nal_units(source):
      if unit[0] & 0x1F == 7 and sps is None and len(unit) >= 4:
        sps = unit
      elif unit[0] & 0x1F == 8 and pps is None:
        pps = unit
  return (sps, pps) if sps is not None and pps is not None else None


def _box(kind: bytes, *parts: bytes) -> bytes:
  body = b"".join(parts)
  return struct.pack(">I", 8 + len(body)) + kind + body


def _full(kind: bytes, version: int, flags: int, *parts: bytes) -> bytes:
  return _box(kind, struct.pack(">I", version << 24 | flags), *parts)


def _runs(values: list[int]) -> list[tuple[int, int]]:
  runs: list[tuple[int, int]] = []
  for value in values:
    if runs and runs[-1][1] == value:
      runs[-1] = (runs[-1][0] + 1, value)
    else:
      runs.append((1, value))
  return runs


def mux(width: int, height: int, sps: bytes, pps: bytes, frames: list[tuple[int, bool, bytes]]) -> bytes:
  """frames: (timestamp ns, keyframe, Annex-B data), in order. Starts at the first keyframe."""
  while frames and not frames[0][1]:
    frames = frames[1:]
  samples, keys, stamps = [], [], []
  for stamp, key, data in frames:
    body = b"".join(struct.pack(">I", len(u)) + u for u in nal_units(data) if u[0] & 0x1F not in PARAMETER_SETS)
    if not body:
      continue
    samples.append(body)
    stamps.append(stamp)
    if key:
      keys.append(len(samples))
  if not samples:
    raise ValueError("No video frames")
  durations = [round((b - a) * TIMESCALE / 1e9) for a, b in zip(stamps, stamps[1:], strict=False)]
  durations = [d if 0 < d < TIMESCALE else FALLBACK_DURATION for d in durations]
  durations.append(durations[-1] if durations else FALLBACK_DURATION)
  duration = sum(durations)
  movie_duration = duration * MOVIE_TIMESCALE // TIMESCALE

  avcc = _box(b"avcC", bytes((1, sps[1], sps[2], sps[3], 0xFF, 0xE1)), struct.pack(">H", len(sps)), sps,
              b"\x01", struct.pack(">H", len(pps)), pps)
  avc1 = _box(b"avc1", bytes(6), struct.pack(">H", 1), bytes(16), struct.pack(">HH", width, height),
              struct.pack(">IIIH", 0x00480000, 0x00480000, 0, 1), bytes(32), struct.pack(">Hh", 0x18, -1), avcc)

  def moov(offset: int) -> bytes:
    stbl = _box(b"stbl",
                _full(b"stsd", 0, 0, struct.pack(">I", 1), avc1),
                _full(b"stts", 0, 0, struct.pack(">I", len(runs)), b"".join(struct.pack(">II", *r) for r in runs)),
                _full(b"stss", 0, 0, struct.pack(">I", len(keys)), b"".join(struct.pack(">I", k) for k in keys)),
                _full(b"stsc", 0, 0, struct.pack(">IIII", 1, 1, len(samples), 1)),
                _full(b"stsz", 0, 0, struct.pack(">II", 0, len(samples)), b"".join(struct.pack(">I", len(s)) for s in samples)),
                _full(b"stco", 0, 0, struct.pack(">II", 1, offset)))
    minf = _box(b"minf", _full(b"vmhd", 0, 1, bytes(8)),
                _box(b"dinf", _full(b"dref", 0, 0, struct.pack(">I", 1), _full(b"url ", 0, 1))), stbl)
    mdia = _box(b"mdia", _full(b"mdhd", 0, 0, struct.pack(">IIIIHH", 0, 0, TIMESCALE, duration, 0x55C4, 0)),
                _full(b"hdlr", 0, 0, struct.pack(">I", 0), b"vide", bytes(12), b"VideoHandler\x00"), minf)
    tkhd = _full(b"tkhd", 0, 3, struct.pack(">IIIII", 0, 0, 1, 0, movie_duration), bytes(8),
                 struct.pack(">hhhH", 0, 0, 0, 0), MATRIX, struct.pack(">II", width << 16, height << 16))
    mvhd = _full(b"mvhd", 0, 0, struct.pack(">IIIIIH", 0, 0, MOVIE_TIMESCALE, movie_duration, 0x00010000, 0x0100),
                 bytes(10), MATRIX, bytes(24), struct.pack(">I", 2))
    return _box(b"moov", mvhd, _box(b"trak", tkhd, mdia))

  runs = _runs(durations)
  ftyp = _box(b"ftyp", b"isom", struct.pack(">I", 0x200), b"isomiso2avc1mp41")
  header = moov(0)
  mdat = b"".join(samples)
  return ftyp + moov(len(ftyp) + len(header) + 8) + struct.pack(">I", 8 + len(mdat)) + b"mdat" + mdat
