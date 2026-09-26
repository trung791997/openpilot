#!/usr/bin/env python3
"""Claude Code context-usage sensor.

Vendored from the sure-scale auto-compact plugin (patched copy on the
owner's Mac); runs in cloud sessions only, via cloud-only.sh.

Reads a hook payload on stdin, inspects the transcript referenced there,
and emits a hookSpecificOutput.additionalContext JSON blob on stdout when
the last-assistant token usage exceeds configured thresholds.

Silent no-op on any failure: the sensor must never break Claude's turn.
"""
from __future__ import annotations

import json
import os
import sys
from pathlib import Path
from typing import Optional


def parse_last_assistant_usage(transcript_path: str) -> Optional[int]:
    """Return total input tokens on the last assistant entry, or None.

    Total = input_tokens + cache_creation_input_tokens + cache_read_input_tokens.
    Returns None when the file is missing, empty, has no assistant entry with
    usage, or cannot be parsed at all.
    """
    path = Path(transcript_path)
    if not path.is_file():
        return None
    last_total: Optional[int] = None
    try:
        with path.open("r", encoding="utf-8") as fh:
            for line in fh:
                line = line.strip()
                if not line:
                    continue
                try:
                    entry = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if entry.get("type") != "assistant":
                    continue
                usage = (entry.get("message") or {}).get("usage")
                if not isinstance(usage, dict):
                    continue
                total = (
                    usage.get("input_tokens", 0)
                    + usage.get("cache_creation_input_tokens", 0)
                    + usage.get("cache_read_input_tokens", 0)
                )
                last_total = total
    except OSError:
        return None
    return last_total


def compute_tier(total: int, soft: int, hard: int) -> str:
    """Classify total against soft/hard thresholds.

    Boundaries are inclusive on the higher tier: total == soft returns "soft",
    total == hard returns "hard".
    """
    if total >= hard:
        return "hard"
    if total >= soft:
        return "soft"
    return "under"


_SOFT_TEMPLATE = (
    "<context-usage>{total} tokens ({pct}% of {window_k}k) — approaching "
    "limit. Compact at the next natural stop point (end of user turn, "
    "after current sub-task completes).</context-usage>"
)

_HARD_TEMPLATE = (
    "<context-usage status=\"critical\">{total} tokens ({pct}% of "
    "{window_k}k) — hard threshold exceeded. If main session: {main_action} "
    "If subagent: stop, emit <handoff reason=\"context-limit\"> and return "
    "immediately — do NOT compact.</context-usage>"
)

_MAIN_ACTION_TMUX = (
    "call ~/.claude/bin/auto-compact.sh now, before the next tool call."
)

# Headless sessions (Remote Control, SDK, Desktop) have no TUI prompt in
# $TMUX_PANE: `claude rc` spawns each session as a `--print --sdk-url` child
# that inherits the host's TMUX vars, so send-keys would type /compact into
# the rc host log instead of the session. There, Claude Code's built-in
# auto-compact does the compacting once an auto-compact window is set
# (autoCompactWindow / CLAUDE_CODE_AUTO_COMPACT_WINDOW); Claude's job is to
# have a checkpoint on disk before it fires.
_CHECKPOINT = (
    "write or refresh a checkpoint (files touched, test state, open "
    "decisions, exact next step) to your own manifest, "
    ".claude/compact-state-$CLAUDE_CODE_SESSION_ID.md"
)

_SOFT_TEMPLATE_AUTO = (
    "<context-usage>{total} tokens ({pct}% of {window_k}k) — approaching "
    "limit. This session compacts automatically (built-in auto-compact); at "
    "the next natural stop point, " + _CHECKPOINT + ".</context-usage>"
)

_MAIN_ACTION_AUTO = (
    "do NOT call auto-compact.sh (it cannot reach this session). Before the "
    "next tool call, " + _CHECKPOINT + ", then keep working — Claude Code's "
    "built-in auto-compact will compact this session automatically."
)

_MAIN_ACTION_MANUAL = (
    "this session cannot receive /compact from auto-compact.sh and has no "
    "auto-compact window set. Stop before the next tool call, "
    + _CHECKPOINT + ", then ask the user to run /compact and end the turn. "
    "(Setting autoCompactWindow makes this automatic.)"
)

DEFAULT_SOFT = 100_000
DEFAULT_HARD = 160_000
DEFAULT_WINDOW = 300_000
DEFAULT_DEBUG_LOG = "/tmp/claude-compact-sensor.log"


def _debug_log(message: str) -> None:
    if os.environ.get("CLAUDE_COMPACT_DEBUG") != "1":
        return
    path = os.environ.get("CLAUDE_COMPACT_DEBUG_FILE", DEFAULT_DEBUG_LOG)
    try:
        with open(path, "a", encoding="utf-8") as fh:
            fh.write(message + "\n")
    except OSError:
        pass


def render_reminder(
    tier: str, total: int, window: int, cwd: Optional[str] = None
) -> str:
    """Build the reminder XML for a given tier. Returns '' for 'under'."""
    if tier == "under":
        return ""
    pct = round(100 * total / window) if window > 0 else 0
    window_k = window // 1000
    deliverable = can_deliver_compact()
    auto = not deliverable and builtin_autocompact_window_set(cwd)
    if tier == "soft":
        fmt = _SOFT_TEMPLATE_AUTO if auto else _SOFT_TEMPLATE
        return fmt.format(total=total, pct=pct, window_k=window_k)
    if deliverable:
        main_action = _MAIN_ACTION_TMUX
    elif auto:
        main_action = _MAIN_ACTION_AUTO
    else:
        main_action = _MAIN_ACTION_MANUAL
    return _HARD_TEMPLATE.format(
        total=total, pct=pct, window_k=window_k, main_action=main_action
    )


def builtin_autocompact_window_set(cwd: Optional[str] = None) -> bool:
    """True when Claude Code's own auto-compact has an explicit window.

    Without one, models with a native 1M window only auto-compact near the
    end of it, far past the sensor's thresholds.
    """
    if os.environ.get("DISABLE_AUTO_COMPACT") == "1" or os.environ.get(
        "DISABLE_COMPACT"
    ) == "1":
        return False
    if os.environ.get("CLAUDE_CODE_AUTO_COMPACT_WINDOW"):
        return True
    paths = [Path.home() / ".claude" / "settings.json"]
    if cwd:
        paths += [
            Path(cwd) / ".claude" / "settings.json",
            Path(cwd) / ".claude" / "settings.local.json",
        ]
    for path in paths:
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            continue
        if not isinstance(data, dict) or data.get("autoCompactEnabled") is False:
            continue
        window = data.get("autoCompactWindow")
        if window not in (None, "auto"):
            return True
    return False


def can_deliver_compact() -> bool:
    """True when auto-compact.sh can type /compact into this session.

    Requires tmux, and an interactive session: SDK-driven sessions (including
    Remote Control children) report CLAUDE_CODE_ENTRYPOINT=sdk-*.
    """
    if not (os.environ.get("TMUX") and os.environ.get("TMUX_PANE")):
        return False
    return not os.environ.get("CLAUDE_CODE_ENTRYPOINT", "").startswith("sdk")


def _env_int(name: str, default: int) -> int:
    """Read an integer from an environment variable, or return default."""
    raw = os.environ.get(name)
    if raw is None:
        return default
    try:
        return int(raw)
    except (TypeError, ValueError):
        return default


def main() -> int:
    try:
        raw = sys.stdin.read()
    except Exception:
        return 0
    if not raw.strip():
        return 0
    try:
        payload = json.loads(raw)
    except json.JSONDecodeError:
        return 0
    if not isinstance(payload, dict):
        return 0

    transcript_path = payload.get("transcript_path")
    event_name = payload.get("hook_event_name", "")
    if not transcript_path:
        return 0

    total = parse_last_assistant_usage(transcript_path)
    if total is None:
        return 0

    soft = _env_int("CLAUDE_COMPACT_SOFT", DEFAULT_SOFT)
    hard = _env_int("CLAUDE_COMPACT_HARD", DEFAULT_HARD)
    window = _env_int("CLAUDE_COMPACT_WINDOW", DEFAULT_WINDOW)

    tier = compute_tier(total, soft, hard)
    _debug_log(
        f"event={event_name} total={total} soft={soft} hard={hard} tier={tier}"
    )
    if tier == "under":
        return 0

    reminder = render_reminder(tier, total, window, payload.get("cwd"))
    output = {
        "hookSpecificOutput": {
            "hookEventName": event_name,
            "additionalContext": reminder,
        }
    }
    sys.stdout.write(json.dumps(output))
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception:
        sys.exit(0)
