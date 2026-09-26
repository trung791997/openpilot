#!/usr/bin/env python3
"""SessionStart(compact) hook: point the fresh context at this session's manifest.

Adapted from the sure-scale auto-compact plugin's post-compact hook; runs in
cloud sessions only, via cloud-only.sh. Built-in auto-compact writes its own
summary; the manifest Claude wrote beforehand is richer. Manifests are per
session (CLAUDE.md), so only .claude/compact-state-<session_id>.md is named.
Silent no-op on any failure.
"""
from __future__ import annotations

import json
import os
import sys
from pathlib import Path


def main() -> int:
    try:
        payload = json.loads(sys.stdin.read() or "{}")
    except ValueError:
        return 0
    if not isinstance(payload, dict) or payload.get("source") != "compact":
        return 0
    session_id = payload.get("session_id") or os.environ.get("CLAUDE_CODE_SESSION_ID")
    if not session_id:
        return 0
    cwd = payload.get("cwd") or os.getcwd()
    manifest = Path(cwd) / ".claude" / f"compact-state-{session_id}.md"
    if not manifest.is_file():
        return 0
    context = (
        "<post-compact>The conversation was just compacted. Your manifest, "
        f"written before compaction: {manifest}\nRead it before your next "
        "action and continue from its Immediate Next Step.</post-compact>"
    )
    sys.stdout.write(json.dumps({"hookSpecificOutput": {
        "hookEventName": "SessionStart", "additionalContext": context}}))
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception:
        sys.exit(0)
