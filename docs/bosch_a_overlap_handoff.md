# Claude handoff: Bosch-A shadow overlap

> **Port note (2026-10-20, STATUS 259).** This work was ported from Codex's
> `codex/bosch-a-gated-lateral-velocity` (1dc044833, f60e19d54, 4425d1404, where it was numbered
> D-089/D-090) onto `bosch-a-shadow-telemetry` from `iq-nav-on-pr10`, renumbered D-127/D-128. It now
> sits behind the `BoschARadarShadow` toggle (default off, Bosch-A only). With the toggle off the
> parser and radard skip the shadow entirely and publish bytes identical to iq-nav (static test).
> The body below is Codex's original text with only the D-numbers changed; its branch names, test
> counts and baseline commit refer to the Codex branch.

Peter requested path-overlap scoring beside the current matcher and a review handoff. Work is on
`codex/bosch-a-gated-lateral-velocity`, continuing f60e19d543 (D-127). Read AGENTS.md, STATUS.md,
D-127/D-128, then the comments and `bosch_a_overlap_shadow()` next to the matcher in radard.py.
This branch is a review artifact; it has not been merged into the separately evolving active branch
or deployed. Firmware-analysis-kit's `radar-re/parser-shadow/CLAUDE_HANDOFF.md` covers the earlier
azimuth-rate/edge investigation and links the exact source commit and retained replay hashes.

## Implemented

- D-127 remains: exact Cartesian yvRel derivative with unavailable values on direct-U11 rails,
  ratio fallback, unusable rate/quality and coasts. Azimuth rate cannot replace absolute azimuth.
- Clamped edges now reconstruct boundary angles before projection. Existing off-axis tests that
  asserted the approximate Cartesian expression were replaced by inverse-angle acceptance tests.
- New capnp telemetry: `radarState.boschAOverlapShadow.tracks` reports each current track's edge
  availability and model-path overlap. Positive y is radar-left; model path right-positive y is
  negated. Sample the model path at dRel + RADAR_TO_CAMERA; no extrapolation. The +/-1 m corridor is
  an explicit diagnostic scale, not a calibrated footprint. Scores are exp(-centre/edge distance).
- `matches` reports each model lead's current matcher ID, edge-counterfactual ID, actual selected
  ID, preference, candidate probabilities/gates and changed marker. Counterfactual replaces lateral
  distance with distance to the extent, retaining all other matcher checks and preferred fallback.
  Path overlap is a separate observation, not an extra gate. Wider returns receive more favorable
  interval scores; structures, adjacent traffic and ghosts remain potential wrong matches.
- Invalid/coasted/duplicate edges use centre matching in the counterfactual. Invalid paths produce
  unavailable path scores; invalid model-lead arrays are not scored. No stale extent is stored.
  `edgeValid`, `pathValid`, `radarFresh`, `mdMonoTime` and `liveTracksMonoTime` qualify the records.

The actual selector, lead outputs and planner never consume these fields. No cut-in predictor or
velocity/yaw projection was added. The old onpath adopter remains unchanged, including its unshifted
path interpolation. Confirm the extrinsic/mounting and path corridor before any active use.

## Evidence and interpretation

Four existing cached segments were replayed through real CAN parsing and model-cycle radard:
`00000323--c20feb1f4b/4`, `0000033c--476e8185eb/14`, `0000033c--476e8185eb/24`,
`00000325--da2c095247/54`. The comparator executes radard from f60e19d543 against identical parser
outputs. New and old established radar/StarPilot fields and preferred IDs match on all 3,187 model
cycles. This is observational noninterference, not road or changed-closed-loop validation.

There are 2,373 sweeps, one CAN-error sweep, 6,577 point/cycle rows and 3,386 valid path rows. 32
brackets overlap the diagnostic corridor while their centres lie outside it. This does not mean
32 recovered leads. Seven counterfactual lead-slot matches (five model cycles, not seven objects)
would choose track 25 on 33c/24. Three replace no match; four replace track 55. The chosen track has
zero or unavailable model-path overlap in every case. Target identity and correctness are unproved.
No active-policy promotion follows from these results.

184 consumer tests pass, including 26 new tests of published telemetry, curves/sign/origin,
availability, probability/distance gates, preferred fallback, competing tracks, actual publication,
and state/lead equivalence. 191 parser tests pass. Full Honda: 412 pass and the known eps-121
FakeParams.get_bool failure. Mutation checks fail as intended for a broken interval distance,
diagnostic mutation of a Track, and old clamped projection (all three angular cases). Ruff passes.
No native build or device deployment occurred. Container timing is not device timing.

## Reproduce and continue

Use the AGENTS/STATUS arm64 container test recipe and add
`selfdrive/controls/tests/test_radar_overlap_shadow.py` to the consumer suites. Parser tests run
on the Mac's working Python too. Tests assert the capnp field on the actual radarState topic.

`tools/bosch_a_overlap_replay.py` takes explicit `--log ROUTE PATH` arguments, an explicit
`--baseline-source` (export `git show f60e19d543:selfdrive/controls/radard.py`) and `--output`.
It stores summaries, input/source hashes and changed-match examples, never raw route files. Run
in the supported arm64 test container. Replay fixes .35 probability threshold, adjacent/human-lane
toggles off and newborn semantics on; it does not reproduce persisted settings or real SubMaster
frequency validity. Segments restart filter/parser history, and only last-received messages are used.
No Konik downloads were needed; existing caches were mounted read-only and left intact.

Retained small evidence: firmware-analysis-kit `radar-re/parser-shadow/overlap-shadow/` and
`SHADOW_PATH_OVERLAP.md`. Inspect changed cases with calibrated overlays/physical object identity,
test curved/adjacent/stopped/oncoming/merge cases, then replay protected braking and measure device
CPU/logging overhead before proposing an active consumer. These follow-ups remain eps-120.
