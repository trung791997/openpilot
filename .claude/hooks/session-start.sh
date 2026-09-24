#!/bin/bash
# SessionStart hook for openpilot-radar.
#
# Two jobs:
#   1. ALWAYS print the working contract, so every agent session -- Claude Code on any
#      account, or any other agent that surfaces hook output -- starts having been told to
#      read AGENTS.md / STATUS.md / DECISIONS.md / CLAUDE.md before touching anything.
#   2. On a remote (Claude Code on the web) container, install deps and build the native
#      extensions, so tests actually run. The checked-in .so files are aarch64 and will not
#      load on the x86_64 web container.
#
# Safe to run by hand:  ./.claude/hooks/session-start.sh
set -euo pipefail

PROJECT_DIR="${CLAUDE_PROJECT_DIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
cd "$PROJECT_DIR"

# ---------------------------------------------------------------------------
# 1. The working contract. Printed in every session, remote or local.
# ---------------------------------------------------------------------------
cat <<'CONTRACT'
================================================================================
openpilot-radar — READ THE CONTRACT BEFORE DOING ANY WORK
================================================================================
This repo has a binding working contract in four files at the repo root. Read them
in this order before your first edit, and do not skip them because a task "looks
like a one-liner":

  1. AGENTS.md    — the contract itself. Applies to EVERY agent, every account.
  2. STATUS.md    — what is true right now, and what is NOT verified.
  3. DECISIONS.md — settled decisions (D-009, D-010, D-027..D-047). Do not
                    relitigate these without new evidence.
  4. CLAUDE.md    — the short version, priority-ordered.

The three rules that cost the most when broken:

  * This code drives and brakes a car. Nothing on this branch is road-validated
    with real radar targets. Never call a change safe/working/verified without
    saying "static", "replay", or "limited road evidence".
  * Deleting a radar point is more dangerous than publishing a degraded one.
    When a gate is uncertain, publish a bound; do not coast. (D-041, D-042)
  * The constants in opendbc_repo/opendbc/car/honda/radar_interface.py each carry
    the evidence that set them. READ THE COMMENT BLOCK BEFORE CHANGING A NUMBER.
    Two of them were re-tuned on offline statistics and caused measured on-road
    regressions.

More than one agent works this repo. One working directory, no side clones, and
commit before you hand off (AGENTS.md §7).
================================================================================
CONTRACT

# ---------------------------------------------------------------------------
# 2. Remote-only environment setup.
# ---------------------------------------------------------------------------
if [ "${CLAUDE_CODE_REMOTE:-}" != "true" ]; then
  echo "[session-start] Local session: skipping dependency install and build."
  echo "[session-start] To set up a local tree, run this script with CLAUDE_CODE_REMOTE=true,"
  echo "[session-start] or follow STATUS.md -> 'Build and test environment'."
  exit 0
fi

VENV="$PROJECT_DIR/.venv"
UV_BIN="$(command -v uv || echo /root/.local/bin/uv)"

echo "[session-start] Remote container detected. Preparing build + test environment."

# --- native libraries ------------------------------------------------------
# openpilot needs capnp, zmq, OpenCL headers, eigen and libusb to build.
# `apt-get update` can fail on unrelated third-party PPAs in this image; that must not
# abort setup, so it is tolerated and the install is what actually gates.
if ! [ -f /usr/include/capnp/common.h ] || ! [ -f /usr/include/zmq.h ] \
   || ! [ -f /usr/include/eigen3/Eigen/Dense ] || ! [ -f /usr/include/libusb-1.0/libusb.h ]; then
  echo "[session-start] Installing native libraries via apt..."
  export DEBIAN_FRONTEND=noninteractive
  sudo apt-get update -qq || echo "[session-start] apt-get update reported errors; continuing."
  sudo apt-get install -y -qq \
    capnproto libcapnp-dev libzmq3-dev \
    opencl-headers ocl-icd-opencl-dev \
    libeigen3-dev libusb-1.0-0-dev
else
  echo "[session-start] Native libraries already present."
fi

# --- python environment ----------------------------------------------------
# NOTE: `uv sync --frozen` does not complete in this image (pyaudio fails to build and
# json-rpc needs a raised HTTP timeout), so the dependency set needed to build and to run
# the radar/longitudinal suites is installed explicitly. Keep this list in sync with
# STATUS.md -> "Build and test environment".
# Python 3.12 matches the device (3.12.3): the checked-in aarch64 .so files need 3.12
# (msgq/ipc_pyx.so imports PyType_FromMetaclass) and fail to import under 3.11.
# An existing venv on another version is rebuilt, unless .venv is tracked by git
# (it once was a committed symlink -- AGENTS.md §10 says not to delete it in passing).
PY_VERSION=3.12
if [ -x "$VENV/bin/python" ] \
   && [ "$("$VENV/bin/python" -c 'import sys; print("%d.%d" % sys.version_info[:2])')" != "$PY_VERSION" ]; then
  if [ -n "$(git ls-files -- .venv)" ]; then
    echo "[session-start] WARNING: .venv is not Python $PY_VERSION and is tracked by git; leaving it alone."
  else
    echo "[session-start] Rebuilding .venv on Python $PY_VERSION ..."
    rm -rf "$VENV"
  fi
fi
if [ ! -x "$VENV/bin/python" ]; then
  echo "[session-start] Creating virtualenv at .venv (Python $PY_VERSION) ..."
  "$UV_BIN" venv --python "$PY_VERSION" "$VENV"
fi

echo "[session-start] Installing Python dependencies (idempotent)..."
UV_HTTP_TIMEOUT=300 "$UV_BIN" pip install --quiet --python "$VENV/bin/python" \
  numpy pycapnp cython scons setuptools \
  pytest pytest-xdist pytest-asyncio pytest-cpp pytest-mock parameterized hypothesis ruff \
  pyzmq smbus2 sentry-sdk requests psutil pyserial tqdm zstandard crcmod \
  setproctitle pyjwt libusb1 python-dateutil pycryptodome cffi sympy casadi \
  future-fstrings

# --- build the native extensions ------------------------------------------
# The tracked .so files are aarch64 (built for the comma). They must be rebuilt for x86_64
# or every import of cereal/msgq/params/acados fails with "cannot open shared object file",
# which reads like a missing file rather than a wrong architecture.
echo "[session-start] Building native extensions (scons)..."
export PYTHONPATH="$PROJECT_DIR"
PATH="$VENV/bin:$PATH" "$VENV/bin/scons" -j"$(nproc)" \
  msgq_repo/ cereal/ opendbc_repo/ common/ selfdrive/

# --- protect the tracked device binaries ----------------------------------
# The build overwrites 26 tracked artifacts in place. Marking them skip-worktree keeps
# `git status` clean so a stray `git commit -a` cannot ship x86_64 binaries to a branch
# that boots on an aarch64 comma.
#
# TO INTENTIONALLY REBUILD DEVICE ARTIFACTS (rare, deliberate -- cf. commit a972d15):
#   git update-index --no-skip-worktree <path>   # then build, stage, and commit it
#
# Marked from the tracked file list, NOT from `git diff` -- a file rebuilt later in this
# same run would otherwise be missed, which is exactly what happened the first time.
echo "[session-start] Marking tracked build artifacts skip-worktree..."
mapfile -t _artifacts < <(git ls-files -- \
  '*.so' '*.a' '*.os' '*.o' \
  'selfdrive/pandad/pandad' 'cereal/messaging/bridge' \
  'panda/board/obj/gitversion.h' 'panda/board/obj/version' \
  'common/params_pyx.cpp' \
  'selfdrive/locationd/models/generated/*.cpp' \
  'selfdrive/locationd/models/generated/*.h')
if [ "${#_artifacts[@]}" -gt 0 ]; then
  git update-index --skip-worktree "${_artifacts[@]}" 2>/dev/null || true
fi
echo "[session-start] skip-worktree set on $(git ls-files -v | grep -c '^S') tracked artifacts."

_dirty="$(git status --porcelain --untracked-files=no | wc -l)"
if [ "$_dirty" -ne 0 ]; then
  echo "[session-start] WARNING: $_dirty tracked file(s) still modified after the build:"
  git status --porcelain --untracked-files=no | sed 's/^/[session-start]   /'
  echo "[session-start] Do NOT 'git commit -a'. Stage only the sources you changed."
fi

# --- persist environment for the session ----------------------------------
if [ -n "${CLAUDE_ENV_FILE:-}" ]; then
  {
    echo "export PYTHONPATH=\"$PROJECT_DIR\""
    echo "export PATH=\"$VENV/bin:\$PATH\""
    echo "export UV_PROJECT_ENVIRONMENT=\"$VENV\""
  } >> "$CLAUDE_ENV_FILE"
fi

cat <<'READY'
[session-start] Environment ready.

  Run the radar + longitudinal suites (366 tests) with:
    python -m pytest opendbc_repo/opendbc/car/honda/tests/ -q -o addopts=""
    python -m pytest selfdrive/controls/tests/test_radard_bosch.py \
                     selfdrive/controls/tests/test_lead_behavior.py \
                     selfdrive/controls/tests/test_lead_follow_policy.py \
                     selfdrive/controls/tests/test_following_distance.py \
                     selfdrive/controls/tests/test_turn_lead.py -q -o addopts=""

  Lint from the tree that owns the file -- opendbc_repo has its own ruff config:
    ruff check <changed files>

  NOTE: selfdrive/controls/tests/test_leads.py needs a real process-replay
  environment and hangs here. It is unrun, not failing.
READY
