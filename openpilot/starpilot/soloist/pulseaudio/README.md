# Vendored PulseAudio server (AArch64)

Spotify Soloist plays only through PulseAudio or PipeWire. AGNOS up to 19.8.1 shipped PulseAudio's
client library without a server; AGNOS 20.2.1 ships neither, so the client libraries are vendored too. This directory is the minimal server needed to carry Soloist's audio to one output:
the `pulseaudio` daemon, `pactl` (to switch between the speaker and the silent sink without
restarting Soloist), their private libraries, and four modules (native protocol, ALSA sink for
BlueALSA, null sink, and the real-time pipe sink used for Android Auto).

- Source: Ubuntu 24.04 (noble) arm64 binary packages, `pulseaudio` and `pulseaudio-utils`
  `1:16.1+dfsg1-2ubuntu10.1`, and the
  libraries listed in `PACKAGES.sha512` (SHA-512 of each `.deb` as published by Ubuntu).
- Client side (AGNOS 20.2.1 has none of it): `libpulse.so.0`, `libpulse-simple.so.0` and
  `libpulsecommon-16.1.so` from `libpulse0`, plus `libsndfile.so.1` and its codecs (`libFLAC`, `libvorbis`,
  `libvorbisenc`, `libogg`, `libmpg123`, `libmp3lame`) and `libasyncns`. Soloist and the phone-audio worker
  load libpulse at run time, so every launch puts `lib/` on `LD_LIBRARY_PATH` (`backend.library_path`). Regenerate with `./update.sh`; `SHA256SUMS` lists the extracted files.
- Licenses: PulseAudio and libltdl are LGPL-2.1-or-later, libtdb LGPL-3.0-or-later, libsoxr
  LGPL-2.1-or-later, liborc and speexdsp BSD-style; libpulse, libsndfile, libasyncns and libmpg123
  LGPL-2.1-or-later, libmp3lame LGPL-2.0-or-later, libFLAC, libvorbis and libogg BSD-3-Clause. Full notices are in `licenses/`. Corresponding source is
  available from Ubuntu's archive (`apt-get source pulseaudio=1:16.1+dfsg1-2ubuntu10.1`, and likewise
  for each package above). The libraries are unmodified and dynamically linked.
