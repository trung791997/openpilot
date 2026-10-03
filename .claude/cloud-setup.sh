#!/bin/bash
# Setup script for the Claude Code cloud environment (paste into the environment's
# "Setup script" box; it is not run from the repo).
#
# It only does the slow, repo-independent work, so every new session starts with it done:
# the apt packages and uv's managed Python 3.12. Everything else -- the .venv, the pip
# dependency set, the scons build of the x86_64 extensions, and masking the tracked aarch64
# binaries -- stays in .claude/hooks/session-start.sh, which runs on every remote session and
# skips whatever is already present. Keep the package list in step with that hook.
set -euo pipefail

SUDO=""
[ "$(id -u)" -ne 0 ] && SUDO="sudo"

export DEBIAN_FRONTEND=noninteractive
# `apt-get update` can fail on unrelated third-party sources in this image; the install gates.
$SUDO apt-get update -qq || echo "[setup] apt-get update reported errors; continuing."
$SUDO apt-get install -y -qq \
  clang build-essential xvfb \
  capnproto libcapnp-dev libzmq3-dev \
  opencl-headers ocl-icd-opencl-dev \
  libeigen3-dev libusb-1.0-0-dev

# uv, then its own CPython 3.12 (ships Python.h, matches the device's 3.12 for the .so files).
if ! command -v uv >/dev/null && [ ! -x "$HOME/.local/bin/uv" ]; then
  curl -LsSf https://astral.sh/uv/install.sh | sh
fi
export PATH="$HOME/.local/bin:$PATH"
uv python install 3.12

echo "[setup] done"
