---
name: auto-compact
description: Checkpoint state and execute a lossless compaction in Claude Code Desktop or CLI. Use when context usage is high (>120k tokens), before /compact, or after completing major phases.
argument-hint: "[optional next-phase focus]"
---

# Auto-compact Protocol for Claude Code Desktop & CLI

When context window usage grows large (~120k–160k tokens) or a major milestone is reached, execute this protocol to guarantee zero loss of uncommitted work, test state, and architectural context during compaction.

## Step 1: Audit Current State

Gather the current operational context without making code edits:
1. Run `git status -s` and `git diff --stat` to identify all touched, modified, and untracked files.
2. Identify the active branch, worktree path, and target PR or issue if applicable.
3. Determine the current RED/GREEN status of all recent tests and the exact command lines used to run them.
4. Identify all settled invariants, architectural constraints, and open decisions that must NOT be relitigated.
5. Identify the exact next atomic action (file path, function name, command).

## Step 2: Write Checkpoint Manifest

**The manifest is per session.** `<session-id>` is `$CLAUDE_CODE_SESSION_ID` (read it with
`echo "$CLAUDE_CODE_SESSION_ID"`). Several sessions and agents share this checkout, so a fixed name
like `.claude/compact-state.md` gets picked up by the wrong session. Never write, read, restore from or
delete another session's manifest, and never commit one (`.claude/compact-state*.md` is gitignored).
If `$CLAUDE_CODE_SESSION_ID` is empty, use `.claude/compact-state-<branch>-<UTC yyyymmddThhmmss>.md`
and pass that exact path in the compact summary.

Write the collected state to `.claude/compact-state-<session-id>.md` at the project or worktree root:

```markdown
# Checkpoint Manifest

**Timestamp:** <ISO timestamp>
**Branch/Worktree:** <current branch and worktree path>
**User Goal:** <one-sentence description of the active task>

## 1. Files Touched & Key Symbols
- `<path/to/file>`: lines <start-end>, functions/symbols `<name>` — <one-line summary of change>

## 2. Test Verification Matrix
- **Command:** `<exact command>`
- **Status:** [PASS | FAIL | IN_PROGRESS]
- **Key Output:** <concise error or passing assertion details>

## 3. Settled Decisions & Invariants
- <Invariant 1: e.g. Do not modify public API / do not coast on radar timeout>
- <Settled decision: why approach X was chosen over Y>

## 4. Immediate Next Step
- **Action:** <exact single action to execute next>
- **Target:** `<file or command>`
- **Expected Outcome:** <what success looks like>
```

Ensure `.claude/compact-state-<session-id>.md` is successfully written to disk.

## Step 3: Trigger Compaction

### In Terminal / Tmux:
If running inside tmux (`$TMUX` set) **and not a Remote Control session**
(`$CLAUDE_CODE_CHILD_SESSION` is not `1` and `$CLAUDE_CODE_ENVIRONMENT_KIND` is not `bridge`).
A Remote Control session inherits the rc host's `TMUX_PANE`, so the script types into the rc
host screen and nothing compacts; follow the Desktop path below instead.
Invoke `~/.claude/bin/auto-compact.sh "Restoring from .claude/compact-state-<session-id>.md" "<optional continuation>"`.

### In Claude Code Desktop:
If running inside Claude Code Desktop (no tmux), or in a Remote Control session:
1. Confirm that `.claude/compact-state-<session-id>.md` has been saved to disk.
2. Present a clear, actionable directive to the user:
   ```text
   📦 **State Checkpoint Saved to `.claude/compact-state-<session-id>.md`**

   Full context, modified files, test states, and the next steps are safely anchored to disk.
   
   To compact without losing context, please type:
   /compact Resume from .claude/compact-state-<session-id>.md and execute the Immediate Next Step
   ```
3. Conclude the turn immediately without making further tool calls.

## Step 4: Post-Compaction Recovery Contract

Immediately after any `/compact` or fresh session resume:
1. Check only for `.claude/compact-state-<session-id>.md` with **your own** session id (or the exact path named in the compact summary). Ignore every other `compact-state*` file, even if it is newer.
2. If it exists, read it immediately on turn 1.
3. Announce that context has been restored from `.claude/compact-state-<session-id>.md` and immediately execute the **Immediate Next Step** without asking the user for recap.
4. Once the immediate task is confirmed verified, remove or update `.claude/compact-state-<session-id>.md`.
