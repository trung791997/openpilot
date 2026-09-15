#!/bin/bash
# Remove tracked build artifacts so scons rebuilds them.
#
# Why this exists: the .so/.a files tracked in this repo are aarch64 (built for the comma).
# On any other host they must be rebuilt, but scons considers them up to date and skips them,
# so the link fails with "Relocations in generic ELF (EM: 183)" or "file in wrong format".
# Clearing them forces a real rebuild.
#
# Everything it deletes is tracked in git, so `git restore <path>` brings any of it back.
#
# Safety: it only ever touches files that are ALL of:
#   - inside this repository (it cds to the repo root and uses git's own file list)
#   - tracked by git (so recoverable)
#   - a build artifact by extension or a known generated binary
# It will not follow a path outside the repo and takes no path arguments.
#
# Usage:
#   ./tools/clean_build_artifacts.sh            # only the wrong-architecture ones (default)
#   ./tools/clean_build_artifacts.sh --all      # every tracked build artifact
#   ./tools/clean_build_artifacts.sh --dry-run  # list what would go, delete nothing
#   ./tools/clean_build_artifacts.sh --sconsign # also drop .sconsign.dblite (scons's cache)
set -euo pipefail

cd "$(git rev-parse --show-toplevel)"

ALL=0; DRY=0; SCONSIGN=0
for arg in "$@"; do
  case "$arg" in
    --all) ALL=1 ;;
    --dry-run|-n) DRY=1 ;;
    --sconsign) SCONSIGN=1 ;;
    -h|--help) sed -n '2,25p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "unknown option: $arg (see --help)" >&2; exit 2 ;;
  esac
done

HOST_ARCH="$(uname -m)"

# Only the trees scons builds in this checkout. third_party/ is deliberately absent: it holds
# VENDORED DEVICE libraries (third_party/acados/larch64/, the aarch64 h3 wheels under
# starpilot/third_party/) which are legitimately aarch64 and are not outputs of this build.
# An earlier version of this script keyed purely on "wrong architecture" and would have
# deleted 12 of them.
mapfile -t CANDIDATES < <(git ls-files -- \
  'cereal/*.so'   'cereal/*.a'   'cereal/*.os'   'cereal/*.o' \
  'common/*.so'   'common/*.a'   'common/*.os'   'common/*.o' \
  'msgq_repo/*.so' 'msgq_repo/*.a' 'msgq_repo/*.os' 'msgq_repo/*.o' \
  'opendbc_repo/*.so' 'opendbc_repo/*.a' 'opendbc_repo/*.os' 'opendbc_repo/*.o' \
  'rednose_repo/*.so' 'rednose_repo/*.a' 'rednose_repo/*.os' 'rednose_repo/*.o' \
  'selfdrive/*.so' 'selfdrive/*.a' 'selfdrive/*.os' 'selfdrive/*.o' \
  'selfdrive/pandad/pandad' 'cereal/messaging/bridge')

# Belt and braces: never touch anything under a third_party directory, at any depth.
FILTERED=()
for f in "${CANDIDATES[@]}"; do
  case "$f" in
    third_party/*|*/third_party/*) continue ;;
  esac
  FILTERED+=("$f")
done
CANDIDATES=("${FILTERED[@]}")

TARGETS=()
for f in "${CANDIDATES[@]}"; do
  [ -f "$f" ] || continue
  if [ "$ALL" -eq 1 ]; then
    TARGETS+=("$f"); continue
  fi
  # Default: only artifacts built for a different architecture than this host.
  desc="$(file -b "$f" 2>/dev/null || true)"
  case "$HOST_ARCH:$desc" in
    x86_64:*aarch64*|x86_64:*"ARM aarch64"*) TARGETS+=("$f") ;;
    aarch64:*x86-64*)                        TARGETS+=("$f") ;;
  esac
done

if [ "${#TARGETS[@]}" -eq 0 ]; then
  echo "No ${ALL:+tracked }build artifacts to clear (host $HOST_ARCH)."
else
  printf '%s\n' "${TARGETS[@]}"
  echo "-- ${#TARGETS[@]} tracked build artifact(s)"
  if [ "$DRY" -eq 1 ]; then
    echo "(dry run: nothing deleted)"
  else
    # skip-worktree hides deletions from git status, which would make these look absent
    # rather than pending. Clear the flag first so the tree state stays honest.
    git update-index --no-skip-worktree "${TARGETS[@]}" 2>/dev/null || true
    rm -f -- "${TARGETS[@]}"
    echo "Deleted. Rebuild with: scons -j\$(nproc) msgq_repo/ cereal/ opendbc_repo/ common/ selfdrive/"
    echo "Restore instead with:  git restore ${TARGETS[0]} ..."
  fi
fi

if [ "$SCONSIGN" -eq 1 ] && [ -f .sconsign.dblite ]; then
  if [ "$DRY" -eq 1 ]; then
    echo ".sconsign.dblite would be removed"
  else
    rm -f .sconsign.dblite
    echo "Removed .sconsign.dblite (untracked scons cache; forces a full re-evaluation)."
  fi
fi
