# CLAUDE.md

See **[AGENTS.md](AGENTS.md)** — it is the working contract for this repo and applies to
Claude Code, Cursor, Codex, and any other agent.

Short version, in priority order:

1. Read [`STATUS.md`](STATUS.md) first. It states what is verified and what is not.
2. Read [`DECISIONS.md`](DECISIONS.md) before proposing a design change.
3. This is **longitudinal and lateral control on a car that gets driven.** Nothing on this
   branch has been road-validated with real radar targets. Never call a change safe or
   working without the word *static*, *replay*, or *limited road evidence*.
4. **Deleting a radar point is more dangerous than publishing a degraded one.** When a gate
   is uncertain, publish a bound and coast nothing — see D-041 and D-042.
5. The constants in `opendbc_repo/opendbc/car/honda/radar_interface.py` each carry the
   evidence that set them. **Read the comment block before changing the number.** Several
   of them were re-tuned on offline statistics and caused measured on-road regressions.
6. **One working directory: this repo.** Do not create a parallel copy elsewhere. If you are
   handed a file or folder from outside (attached, `@`-mentioned, or a path), **fold it into
   the repo as part of the work** rather than reading it in place — see AGENTS.md §7.
   Vendored subtrees (`opendbc_repo/`, `panda/`, …) are edited here, in place. Route data is
   cited by route ID and never committed.
7. The checked-in `.so` files are **aarch64**. If a test "cannot run on this machine," it is
   almost certainly that — see AGENTS.md §10 and the build recipe in `STATUS.md`. The
   SessionStart hook (`.claude/hooks/session-start.sh`) builds the tree for you on remote
   containers and masks the build-dirtied binaries.
8. Commit before handing off: `YYYY-MM-DD <what changed>`.
9. **Only where `~/.local/bin/agy-flash` and `agy-pro` exist (the owner's Mac); elsewhere, including
   cloud sessions and other accounts, do the work directly.**
   **Delegate menial and automated tasks to save tokens:** Claude serves as the
   **Architect & Supervisor**. Do not burn high-tier reasoning tokens on repetitive or
   isolated automated tasks:
   - **Delegate to Gemini 3.8 Flash** (`agy-flash`): Purely deterministic/mechanical tasks (running
     `pytest`, `ruff check/format`, AST/Python 3.9 checks in `test_py39_compat.py`, CAN ID
     mirror validation, skip-worktree binary checks, mechanical symbol refactoring).
   - **Delegate to Gemini 3.1 Pro** (`agy-pro`): Automated tasks requiring reasoning or data processing
     (running and diagnosing `tools/bosch_a_corpus_report.py`, telemetry residual analysis
     for `vRelRange` vs U11 per D-044, track-ID lifecycle log parsing per D-049, building
     mock fixtures for `test_leads.py`, synthesizing parameterized negative-control tests).
   - **Retain for Claude**: Architecture, lead arbitration laws in `radard.py` (D-048),
     Alpha Long P061B control invariants, gate threshold safety tuning, and final diff audits.

> **More than one agent works this repo, on separate accounts.** Everything above applies to
> all of them equally. Before you start, run `git log --oneline -10` and `git status` — if
> another agent has pushed since your branch point, read their commits before editing the
> same files, and diff rather than assuming which version is newer (AGENTS.md §7).
