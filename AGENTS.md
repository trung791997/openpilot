# AGENTS.md (night-star7-longradar)

Working contract for every agent on this branch (Claude Code, Cursor, Codex, any account).
This branch started its own docs on 2026-11-06. They are not copied from any other branch.

## 1. What this branch is
`night-star7-longradar` is a branch Peter's car runs. Changes here can reach a car that gets driven.

## 2. Read first
1. `STATUS.md`: what is true on this branch now, and what is not verified.
2. `DECISIONS.md`: settled decisions for this branch. Do not reverse one without new evidence.
3. `CLAUDE.md`: the short, priority-ordered version.
4. `STATUS-archive.md` and `DECISIONS-archive.md`: frozen copies from `ns-bosch-radar-testing`
   @ 84af40f039. Look there when Peter refers to a past fix or a D-number. They describe that branch, not
   this one; never edit them, and record anything new in the live files.

## 3. Safety rules
- Never call a change safe or working without saying *static*, *replay* or *limited road evidence*.
- Deleting a radar point is more dangerous than publishing a degraded one. When a gate is
  uncertain, publish a bound and coast nothing.
- Each constant in `opendbc_repo/opendbc/car/honda/radar_interface.py` carries the evidence that
  set it. Read the comment block before changing the number.
- Nothing goes to the car without Peter. Push only when Peter asks.

## 4. Working rules
- More than one agent works this repo. Run `git log --oneline -10` and `git status` before
  editing. If someone pushed since you started, read their commits before touching the same files.
- One working directory. Do not make side clones.
- Commit before handing off, as `YYYY-MM-DD <what changed>`.
- Never commit `.claude/compact-state*.md`.
- The checked-in `.so` files are aarch64. A test that "cannot run here" usually fails for that reason.
- Route data is cited by route ID and never committed.

## 5. Keeping the docs current
- When you verify something on this branch, add it to `STATUS.md` with its evidence (route ID,
  commit, command) and its level (*static*, *replay* or *limited road evidence*).
- When a choice is settled, add it to `DECISIONS.md` with the next number.
- Entries cover this branch only. To cite work from another branch, give its branch name and commit.
