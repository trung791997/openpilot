#!/usr/bin/env bash
# Run a vendored auto-compact hook in cloud sessions only. On the owner's Mac
# the sure-scale plugin already runs these hooks; running both would double
# every <context-usage> reminder. Stdin (the hook payload) passes through.
[ "${CLAUDE_CODE_REMOTE:-}" = "true" ] || exit 0
export CLAUDE_COMPACT_SOFT="${CLAUDE_COMPACT_SOFT:-120000}"
export CLAUDE_COMPACT_HARD="${CLAUDE_COMPACT_HARD:-180000}"
export CLAUDE_COMPACT_WINDOW="${CLAUDE_COMPACT_WINDOW:-218000}"
exec python3 "$(dirname "$0")/$1"
