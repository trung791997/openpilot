# CLAUDE.md (car branch)

This is a **car branch**: `night-star7-longradar` and `night-star7-vasm-jetson` are what Peter's
car runs. Its working contract is in this branch: `AGENTS.md`, `STATUS.md` and `DECISIONS.md`,
started fresh on 2026-11-06. Read them before editing control code here.
For past fixes and D-numbers Peter mentions, see `STATUS-archive.md` and `DECISIONS-archive.md`
(frozen copies from `ns-bosch-radar-testing` @ 84af40f039; read-only history, not this branch's status).

Short version, in priority order:

1. This is **longitudinal and lateral control on a car that gets driven.** Never call a change
   safe or working without the word *static*, *replay*, or *limited road evidence*.
2. **Deleting a radar point is more dangerous than publishing a degraded one.** When a gate is
   uncertain, publish a bound and coast nothing.
3. The constants in `opendbc_repo/opendbc/car/honda/radar_interface.py` each carry the evidence
   that set them. **Read the comment block before changing a number.**
4. **Nothing goes to the car without Peter.** Do not push to this branch unless Peter asked for it.
5. The checked-in `.so` files are **aarch64**; a test that "cannot run here" is usually that.
6. More than one agent works this repo. Run `git log --oneline -10` and `git status` before
   editing. Commit as `YYYY-MM-DD <what changed>`. Never commit `.claude/compact-state*.md`.

# Opus-orchestrator workflow (when the session model is Opus)

If your session runs on Opus, Opus plans, splits the work and verifies it; the agents in
`.claude/agents/` do the implementation: `implementer` (Sonnet, bounded code and scripts),
`runner` (Haiku, commands, tests, inventories), `verifier` (Sonnet, read-only re-checks of a
worker's claims). One bounded question per brief; never accept a report without checking it.
Opus keeps architecture, safety judgment, final review, git, pushes, GitHub posts and any device
action. Do tiny tasks directly (each worker costs ~90k tokens of start-up context). Do not start
Opus or Fable workers unless a Sonnet worker has already failed. Other models: ignore this section.
