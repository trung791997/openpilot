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
   almost certainly that — see AGENTS.md §10 and the build recipe in `STATUS.md`.
8. Commit before handing off: `YYYY-MM-DD <what changed>`.
