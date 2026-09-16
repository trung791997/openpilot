# Decisions

Settled decisions and the reasoning behind them. **Do not relitigate these without new
evidence** — if you have new evidence, add a superseding entry rather than editing the old
one.

Format: what was decided, what was rejected, why.

---

### On the numbering

Numbers are **inherited from the EPS knowledge-base repo** so cross-references between the
two projects stay valid. **D-001 … D-026 are EPS firmware decisions** (hook placement, the
code cave, RWD build/verify, UART/UDS flashing, telemetry v1–v5, the gain bench) and live in
that repo, not here. They are intentionally absent rather than renumbered.

Carried into this repo: **D-009, D-010, D-027 … D-038**. New decisions made in this repo
start at **D-039** and continue upward.

---

## D-009 — Every assertion suite needs a negative control
**Decided 2026-07-28. Applies here unchanged.** A suite of passing assertions is
unfalsifiable until one of them is shown to fail when the mechanism is broken on purpose.

Concretely in this repo: if a test asserts that a gate suppresses a bad radar point, it must
be paired with a case proving the point *is* published when the gate is removed. Without it,
the passing assertions look identical to a harness that simply observes nothing. This repo
has already paid for the general version of this — see `d8604bd`, where the test evidence for
four defects was measuring a stale mirror of the value under test and passed either way.

## D-010 — Git is the timestamp; artifacts are content-addressed
**Decided 2026-07-28. Applies here unchanged.** No dates in filenames. Commit per session as
`YYYY-MM-DD <what changed>`; tag milestones; `STATUS.md` carries one `As of:` date.

In this repo, the content address of a claim is the pair **(route ID, commit SHA)**. A replay
number without both is not reproducible, because a replay result inherits the code it ran on.

## D-027 — The James night-star experiment keeps James's SR and adds observability only
**Decided 2026-08-21.** The alternate OpenPilot experiment starts from JamesL787
`night-star-bosch-radar` and retains its CR-V road-measured direct six-point steering-ratio
curve unchanged. Runtime telemetry identifies this as profile 5. Profile 4 remains reserved
for arc-dev's firmware-extracted 30-point curve, so replay cannot silently substitute one
implementation for the other.

Ported scope is additive OpenPilot PID/mechanism telemetry and the coherent nine-frame EPS
extractor. Rejected: transplanting arc-dev's firmware VGR, current-curvature hook, or
learned/fixed hybrid behaviour into this comparison branch. Those changes would prevent an
A/B evaluation of James's implementation and would violate the requested scope.

## D-028 — Bosch-A radar support remains an explicit RX-only experiment
**Decided 2026-08-22.** `HONDA_CRV_5G` may use the hand-written Bosch-A object decoder
because two content-retained vehicle logs prove the complete 16-slot object family and the
`0x2FF` sweep trigger are present on the same camera-side bus the Civic implementation
parses. The platform allowlist is **one named set shared by `CarInterface` and `radard`**, so
parser availability and the radar-rate fusion behaviour cannot silently diverge.

The tester toggle remains **required and default-off**. Rejected: enabling the decoder for
every Honda Bosch fingerprint; making it default-on; or treating stationary no-target
sentinels as proof that real target geometry, relative velocity, association, planner
consumption, or vehicle behaviour is correct. Those require a comma build followed by
controlled live-target and road validation. This change is RX-only and does not alter the
steering-ratio implementation.

> **Naming, as of this repo:** the toggle is `BoschARadar` (param `BoschARadar`; UI
> Longitudinal → "Bosch A Radar"), and the car set / prefixes are `HONDA_BOSCH_A` /
> `HONDA_BOSCH_A_*`. The earlier `NrdrBoschARadar` and `civic_bosch_radar` / `CIVIC_BOSCH_*`
> names were renamed in `af4ea03`/`cdfde65` and `fa0552c`, and the non-functional
> `BoschLong` toggle was removed at the same time. The decision above is unchanged; only the
> identifiers moved. `common/params_keys.h` also still carries a separate legacy
> `HondaBoschARadar` key — do not add a third.

## D-029 — StarPilot keeps Konik selection but adopts bounded durable registration
**Decided 2026-08-22.** The alternate branch retains StarPilot's `UseKonikServer` setting and
defaults it to Konik, while adopting arc-dev's durable identity separation under
`/data/community/identity`. A saved Konik identity is authoritative across params resets and
is never written over the factory comma identity. Explicit server switching remains
supported.

Remote registration failure must not block manager indefinitely or manufacture a plausible
random dongle ID. After a bounded attempt (60 s) it records the explicit unregistered state
so setup can continue and report the failure. Rejected: switching this branch to comma
servers, removing the independent background API, or importing arc-dev steering/controller
behaviour as part of the registration repair.

## D-030 — Night-star CR-V starts from the effective arc/nrdr lateral profile
**Decided 2026-08-22.** The modified-EPS `HONDA_CRV_5G` profile follows James
`84f1b7ec789b`: retain StarPilot's DBC-aligned `[0,3840]` identity request mapping while
importing the effective arc/nrdr four-point P/I and feedforward schedules. StarPilot's schema
lacks `kfBP/kfV`, so the feedforward curve remains an explicit runtime selector with `3.6e-6`
as the `CarParams` fallback.

The P/I fine-trim Params default to neutral 100% so the declared `CarParams` tune is the
effective baseline. A migration rewrites **only** the exact untouched legacy `100/135/200`
tuple; custom or partial profiles are preserved. Rejected: keeping the stale flat CR-V tune;
stacking hidden speed multipliers on the explicit schedule; treating the EPS-internal 30,000
command-map endpoint as an OpenPilot CAN command; or copying nrdr's 4096 request endpoint —
the source commit keeps 3840, which matches the Honda DBC's documented range. The new gain
schedules still require a new build and controlled road validation.

## D-031 — CR-V Alpha Long stays disabled until gas and brake use one command domain
**Decided 2026-09-03.** Route `00000002--aa8501ddcb` captures P061B beginning at
951.918310 s during a continuous positive-gas / negative-acceleration / brake-bit
contradiction. The installed source selects gas from adjusted `gas_force` and braking from
raw `accel`, permitting both.

Rejected: treating the fault as a device memory leak — post-engagement RAM and control-process
RSS remain stable, with no OOM or restart. Also rejected: continuing road reproduction on the
defective command path, or attributing the event to MAF/air-filter hardware without new
evidence, given the clean stock-ACC / manual-pedal A/B and the exact software-command/DTC
timing captured here.

## D-032 — Publish the P061B fix as an isolated test branch, not a merge
**Decided 2026-09-03.** `fix-honda-alpha-long-p061b` at `6e583e1c43c5…` is based exactly on
the faulting `7f2bab7b…` commit. The captured failure and the general mutual-exclusion
invariant are both regression-tested.

Rejected: merging directly into the owner's normal driving branch, or calling the change
verified from static tests. The branch stays isolated until a controlled vehicle A/B shows
that P061B does not recur and longitudinal behaviour remains acceptable.

## D-033 — Withdraw the raw-zero P061B gate after its first vehicle test
**Decided 2026-09-04.** Do not continue testing `6e583e1c43c5…` in traffic. Route
`00000003--1423cb6de2` proves it suppresses a positive calculated gas value for 23.96 of
87.3 engaged seconds and immediately reapplies hundreds of gas units across raw-acceleration
zero, creating a severe speed sawtooth. The static gas/brake mutual-exclusion tests were
necessary but insufficient.

Rejected: reading the absence of P061B in this route as validation — the engagement lasted
87.3 s against ~585 s before the original fault. Also rejected: reverting to the unmodified
`gas_force` gate, which restores the proven gas-plus-brake conflict. A future candidate must
preserve road-load compensation **and** explicit gas/brake mutual exclusion together.

## D-034 — Replace the raw-zero gate with compensated gas plus brake inhibition
**Decided 2026-09-04.** Candidate `57d77a7e1918…` restores the positive-`gas_force` decision
so freeway drag/grade compensation survives raw-acceleration zero, and adds `not braking` as
a hard gas condition. Open-loop replay restores all 1,198 gas frames removed by the defective
candidate and suppresses all 1,584 original gas-plus-brake frames.

Rejected: merely reverting `6e583e1c…`, which recreates the contradiction; or treating
open-loop command replay as proof of closed-loop behaviour. The candidate still sends
positive gas with negative non-braking `ACCEL_COMMAND`; whether Honda accepts that over a
long engagement remains a vehicle-validation question.

## D-035 — Do not tune the lead planner around the compensated-gate regression
**Decided 2026-09-04.** Route `00000004--dcc6a5b59e` shows `57d77a7e1918…` resolves the
simultaneous transmitted gas/brake command but still selects those modes from **different
physical quantities** — braking from raw `accel < −0.2`, propulsion from acceleration plus
grade and aero compensation. Of 3,916 logged brake frames, 2,336 have positive reconstructed
adjusted force. The next candidate arbitrates gas, coast and brake from that same
adjusted-force quantity with stateful hysteresis near zero, retaining explicit stop/hold and
urgent-deceleration tests.

Rejected: retuning lead-follow acceleration to conceal the downstream mode discontinuity.
The described natural behaviour is observational evidence, not a policy, and the pre-fix
controller already felt smooth before its simultaneous gas/brake defect was exposed. Also
rejected: treating the illustrative `−0.12/−0.02 m/s²` thresholds as final — they reduce
logged brake runs 39 → 11 in open-loop replay only, and this route retains just 3.70 s of
decline activity, which cannot establish a two-sided tune.

## D-036 — Test same-force-domain Bosch arbitration with a stateful coast band
**Decided 2026-09-04.** Publish experimental candidate `694046f33f58…` on the existing
isolated branch. It enters braking when adjusted tractive force falls below `−0.12 m/s²`,
releases above `−0.02 m/s²`, forces braking in the stopping state, and retains the
independent CAN-boundary rule that gas requires **both** positive adjusted force and no
brake. Propulsion, coast and ordinary braking now use one physical domain without weakening
the P061B mutual-exclusion invariant.

The thresholds are an experimental selection, not a final tune. Rejected: keeping D-034's raw
`−0.2 m/s²` brake bit, which route 4 proves taps the brake during positive tractive effort;
using adjusted force with no hysteresis, which leaves a 50 Hz sign boundary; removing the
stopping override; or claiming the candidate verified from replay — a changed command
invalidates the recorded future closed loop.

## D-037 — Base the Alpha Long PR on the current James target tip
**Decided 2026-09-05.** Merge the current JamesL787 `ns-bosch-radar-testing` tip into the
isolated candidate and open the result as PR #9. The branch contains the complete target
history through `3f27c6faaf5b…` via merge `1ddce1708420…`; the comparison is clean and
confined to the three intended Honda Alpha Long files.

Rejected: opening review from the stale `7f2bab7b…` base, which would show eight intervening
radar/planner commits as divergence; rebasing away the experimental history after an explicit
request to merge into the existing test branch; or treating `MERGEABLE` as road validation.

> **Superseded in part by D-047.** This entry recorded the planner/`radard` tests as
> "locally uncollected because the checked-in `msgq/ipc_pyx.so` is not valid Mach-O." That
> was an architecture mismatch, not a corrupt artifact, and it is solvable. Re-run those
> suites before citing this decision's coverage claims.

## D-038 — Port the final force-domain fix to StarPilot without retuning MVL
**Decided 2026-09-06.** Base the StarPilot port exactly on Firestar `Dom` `9d1043ad0114…`
and adapt the ordinary Honda Bosch path to StarPilot's `CarParams` CAN-packer API. Ordinary
Bosch passes the same adjusted force used to calculate gas magnitude, applies stateful
`−0.12/−0.02 m/s²` brake hysteresis plus stopping override, and repeats mutual exclusion at
the CAN boundary. Preserve the separate Accord 11G MVL force crossover through the optional
legacy selector; there is **no route evidence authorizing an MVL retune**.

Rejected: cherry-picking the James commit without adaptation, which would discard StarPilot's
`CarParams` interface and MVL behaviour; keeping raw acceleration as ordinary Bosch mode
selection; or describing the user-reported absence of new DTCs as proof.

---

## D-039 — Bosch-A is one 16-slot bank, and its DBC is hand-written and enforcing
**Decided in this repo; recorded 2026-09-15 from the implementation.** The wire format is a
single 16-slot object bank: four main frames per slot (`f0..f3`, one CAN ID each — **not**
sub-frames muxed onto a shared ID) plus one synchronized auxiliary frame.

This supersedes two earlier models that were wrong: `0x280/0x284/0x288/0x28C` as pieces of
one object, and `0x2C8/0x2C9` as a separate "coarse" object list. Both are retired.

`opendbc/dbc/honda_bosch_a_radar.dbc` is written by hand and is deliberately **not** produced
by the opendbc generator pipeline. It declares the real Honda rolling `COUNTER` (2-bit, last
byte bits 5:4) and `CHECKSUM` (4-bit, bits 3:0) so opendbc enforces them — verified
361,360/361,360 checksum passes and 214,400/214,400 sequential counter transitions on route
`000001df`. `FRAME_IDX` and `LIFECYCLE_RAW` are intentionally *not* named `COUNTER`.

Rejected: generating this DBC, which cannot express the bank geometry; and naming the
per-slot frame index `COUNTER`, which would make opendbc enforce the wrong field.

## D-040 — Decode constants come from the firmware, not from fitting against vision
**Decided in this repo; recorded 2026-09-15.** Range scale is `1/16` m/count and azimuth is
`(raw − 1024)/2048` rad, both derived from the firmware's own arithmetic and independently
corroborated (see `STATUS.md`). The range **offset** is a per-unit calibration value
(`−n/128`, `n` from a config word plus a runtime addend); `−3.0` is a retained plausible
value, and the right way to improve it is to **read the radar's configuration**.

Rejected: the previous `0.05712` scale, which was `16 × 0.00357` where the `0.00357` had been
solved from a **single tape point with the offset assumed** — it read ~9% short, −9 m at 60 m
against vision. Also rejected: re-fitting either constant against the vision lead. Vision is
not a range reference, and a two-parameter fit against it will silently trade scale for
offset and look good in the middle of the range.

## D-041 — A saturation rail is a bound, and it must still be published
**Decided in this repo; recorded 2026-09-15.** U11 raw `0` and `1728` mean `|vRel| ≥ 13.5 m/s`
with the exact value unrecoverable. Publish the rail. Do **not** treat it as a missing
measurement.

This was briefly routed to the coast path, and it was a safety regression. A stationary car
approached above 13.5 m/s (30 mph) rails on *every* sweep, so the coast never ended, it
outlived `BOSCH_A_STALE_S`, and the radar point was **deleted**. Route `000001f9` at 29:52:
two stopped cars, 88 of 88 active frames on the low rail with healthy u10 (78–94), range
closing smoothly at −19.4 m/s. `radard` fell back to a vision lead reporting only −11.4 m/s;
the planner commanded **0.00 while closing on stopped traffic at 76 m with a 6.6 s TTC**, and
the driver intervened. Restored in `6126e51`.

Publishing the rail understates closing (a stopped car reads as `vLead = vEgo − 13.5`) and
that understatement is why it looked worth "fixing". **Understating closing still brakes;
deleting the object does not.** Recovering the true value past the rail needs the range
channel and is deliberately left to a separate, validated change.

Generalise this: **when a radar gate is uncertain, the safe direction is to publish a
degraded bound, not to coast.** Every coast has a deadline, and the deadline deletes the car.

## D-042 — The u10 threshold is 511, validated by replay; do not re-tune it from statistics
**Decided in this repo; recorded 2026-09-15.** `BOSCH_A_DIRECT_VREL_MAX_UNCERTAINTY_RAW =
511`, validated in `d5000fe344` by replaying 20 bookmarks through the real `RadarInterface`
and `radard`'s Kalman filter, where it collapsed recorded `aLeadK` spikes up to −26.5 m/s².

It was lowered to 128 on error statistics alone. That undid the validated result and caused a
measured regression: route `000001f3` at 19:27, u10 sat above 128 for 0.81 s during a real
~8 m/s² lead decel, the coast outlived `BOSCH_A_STALE_S`, the lead was deleted, and the
vision fallback injected a 5 m / 6 m/s step driving a −3.51 m/s² brake. Restored in `39c0f09`.

The reason the statistics mislead: **u10 is confounded with dynamics.** Across 16,834 frames,
median |err| rises 0.26 → 0.88 → 1.44 → 1.95 m/s over u10 bins 0-64/64-128/128-192/192-256,
but median |a_rel| rises 0.73 → 2.13 → 3.51 → 4.59 m/s² alongside it
(`corr(u10,|err|) = +0.40`, `corr(u10,|a_rel|) = +0.43`). Holding dynamics out, the quality
signal is real (calm-frame corr +0.52) — but u10 climbs during genuine hard braking just as
reliably, so a lower threshold **preferentially rejects real manoeuvring**.

Rejected: tuning this number from offline error statistics at all. What must stay safe is the
coast path (D-041), not this threshold.

## D-043 — The multi-sweep velocity/range consistency check is one-sided
**Decided in this repo; recorded 2026-09-15.** Compare U11 against a range rate fitted over
several sweeps, and reject **only** "U11 claims more closing than the geometry supports."

Two reasons it cannot be symmetric. U11 *lags* true closure at a deceleration onset — at
`000001f3` 19:27 the fitted range rate was −6.7 m/s while U11 still read −2.08 — so a
symmetric `|U11 − rate|` test fires during genuine hard braking, exactly when the velocity
matters most. And lag can only make U11 *under*-report closing while a lead brakes, so only
the other direction is evidence of a fault. In the mirror case (a lead accelerating away) the
one-sided test coasts a more-conservative velocity, so it fails safe.

Scope, measured on nine flagged events: this catches **gross** disagreement — `000001eb` 6:59
reported `vRel −10.58 m/s` while the range was *opening* at +0.46 m/s, an 11 m/s
contradiction across a track-identity change. It deliberately does **not** chase the milder
0.6–2.5 m/s overshoots at deceleration onset; a trailing window legitimately lags during real
braking, and those are the u10 gate's job.

Rejected: relying on the per-sweep innovation gate to catch velocity error. Over one ~70 ms
sweep, even a 3 m/s error moves the range 0.2 m — far under its 2–5 m thresholds. That gate
has no power here at all.

## D-044 — The range-derived vRel is shadow telemetry; nothing consumes it
**Decided in this repo; recorded 2026-09-15.** `Track.vRelRange` publishes a 5-sample LSQ
range rate alongside the radar's own `vRel`, for **whichever** radar lead is selected — not
only Bosch-A — so the two can be compared on real drives. It is wired to **no** control path.

It is plain LSQ with **no outlier rejection**, so it inherits the range channel's ~1% gross
outliers: read it as a diagnostic, never as a validated velocity. Timestamps come from the
message clock rather than wall clock, because wall-clock time made every accelerated replay
of this field meaningless. Duplicate payloads are excluded (`measurement_update=False`) so a
repeated frame cannot forge a zero-`dt` sample, and a stale or gappy history is rejected
outside `[0.12, 0.60] s`.

Why it exists: on `000001fe/fb/fd`, U11 detects a closing onset 0.88–1.28 s late while a
4-sample range LSQ lands within 0.07–0.14 s; on `00000141` R141-8, U11 diverged from the
range by 13.7 m/s while the range moved 1.9 m.

Rejected: wiring it into control before a drive confirms or refutes it. Also rejected:
diverging `RANGE_VREL_SAMPLES` from the minimum fit length — the deque length and the minimum
are deliberately the same constant.

## D-045 — Publish RadarPoints on the last main frame, never on the auxiliary frame
**Decided in this repo; recorded 2026-09-15.** `BOSCH_A_TRIGGER_MSG` is slot 15's fourth main
frame (`0x2FF`). Passive captures prove slot 15's companion frame `0x297` follows it and is
the final observed object-family frame in a full sweep.

Rejected: making `0x297` the trigger. One dropped auxiliary frame would then suppress an
otherwise-valid point update, contradicting the parser's contract that **aux never gates
validity**. The companion data stays optional/debug-only, and that is the whole point.

## D-046 — The lead Kalman dt comes from the measured cadence, not a round number
**Decided in this repo; recorded 2026-09-15.** `BOSCH_A_FREQ_HZ = 14.35`, the median of
`0x280` inter-arrival across the retained routes (p5 12.5, p95 16.9). `radard` derives
`HONDA_BOSCH_A_RADAR_TS` from it, so the lead Kalman filter runs at the rate the radar
actually measures at.

Rejected: the previous round `15`, which ran the filter ~4.5% fast. Note this is **not** the
rate `radard` itself is driven at — that is the ~20 Hz model rate — which is why the duplicate
payload path exists at all: a Bosch-A measurement must not be absorbed twice.

## D-047 — The checked-in device binaries are aarch64; rebuild rather than reporting "cannot run"
**Decided 2026-09-15.** The `.so` files tracked in this repo (`msgq/ipc_pyx.so`,
`common/params_pyx.so`, `common/transformations/transformations.so`, the acados solvers) are
built for the comma device and are **ARM aarch64**. On any other host they fail to load with
`cannot open shared object file`, which reads like a missing file rather than a wrong
architecture.

This has been recorded at least three times across prior status writeups as "the
radard/planner tests cannot run on this checkout" — on macOS as "not a valid Mach-O binary",
on Linux as "is a Linux binary". In every case the tests were runnable after a local
`scons` build. **366 radar and longitudinal tests pass** once the tree is rebuilt; the exact
recipe is in `STATUS.md` → *Build and test environment*.

Standing rule: **do not report a test as unrunnable until you have tried that recipe**, and
do not cite a coverage claim that was skipped for this reason without re-running it.

Corollary, and it cuts the other way: **a local build overwrites 26 tracked device binaries
in place**, so after building, `git status` shows them modified. Committing them would ship
x86_64 artifacts to a branch that boots on an aarch64 comma. Stage explicitly; never
`git commit -a`. Rebuilding a device artifact on purpose is its own commit that says so
(`a972d15` is the pattern), never a side effect of running tests.

Also settled here: the repo-root **`.venv` is a tracked symlink to another machine's home
directory** (`/Users/jameslichtenstiger/nrdr/openpilot/.venv`, mode `120000`, from `82796af`)
and breaks `uv sync`. Work around it with `UV_PROJECT_ENVIRONMENT`. Rejected: deleting the
tracked link as a side effect of unrelated work — it is a one-line commit of its own, and it
belongs in a change that is about the build environment and nothing else.

## D-048 — Corroboration bounds a radar lead's braking authority; it never deletes the point
**Proposed 2026-09-15 from a six-segment corpus. NOT IMPLEMENTED — see the validation gate
at the end before writing any code.**

### The failure this addresses

On `00000231--5782493b00` t=9.1–11.35 s, radar track 6 (the selected lead, own lane) walked
its range from 71.8 to 61.6 m with U11 ramping to −6.02 m/s while the vision lead held
75–78 m at `prob` ≥ 0.93. The planner commanded −2.89 m/s², the driver felt −4.28 and
overrode with the accelerator. Track 6 then coasted, froze, and disappeared for 11.02 s.

**Every existing gate passed it, and two of them structurally cannot ever catch it:**

- The per-sweep range innovation gate saw ~0.25 m per sweep against a 2.0 m limit.
- D-043's one-sided U11-vs-range check compares two channels that **both come from the same
  radar track**. Here they agreed with each other (−6.0 vs −4.5…−6.4). When a track migrates
  onto the wrong scatterer, its range and its velocity stay mutually consistent while both
  describe the wrong object. No radar-internal cross-check can see this.
- D-044's shadow `vRelRange` is an LSQ fit of that same range, so it agrees too. **Record
  this against D-044: it can cross-check U11, but it can never cross-check the range.**
- `HONDA_BOSCH_A_GROSS_DISTANCE_M = 25.0` saw a 15.8 m peak disagreement.

### What was rejected first, on evidence

**Tightening `HONDA_BOSCH_A_GROSS_DISTANCE_M`.** Refuted by the corpus: pooled radar/vision
gap is median +4.0, p95 +14.9, max +27.7 m, and segment `9e21cac9` sustains a **median 13.8 m
gap with no hard braking at all**. A ~12 m threshold fires continuously where nothing is
wrong.

**Gating on d(gap)/dt.** Refuted: |d(gap)/dt| is p95 31 m/s, max 130 m/s, dominated by the
vision model's frame-to-frame `x` jitter; `9e21cac9` reaches p95 53.7 m/s while braking
normally.

**Switching to the vision lead when radar looks wrong.** Rejected on existing evidence, not
new work: D-042 records that exactly this hard switch injected a 5 m / 6 m/s step and drove a
−3.51 m/s² brake. A fix that hard-switches recreates a regression already paid for.

**Deleting or coasting the suspect point.** Rejected outright — that is D-041, and doing it
cost a driver intervention on stopped traffic.

### The decision

Give each radar track a **corroboration score**: a slow EMA of how well its range has agreed
with the vision lead while the two were matched, plus how much of its recent history was
`measured` rather than coasted. The score bounds **how much deceleration authority that track
may command** — it never gates publication.

Three properties are load-bearing:

1. **The point is always published.** Corroboration changes authority, not existence
   (D-041). A poorly-corroborated lead still brakes; it brakes *less hard*.
2. **Authority is blended, never switched.** Low corroboration moves the commanded
   deceleration toward what the vision-implied kinematics support, continuously. No
   discontinuity, so the D-042 step-injection cannot recur.
3. **Corroboration is slow; urgency overrides it.** The score must not react to
   frame-to-frame vision jitter (see the refuted gap-rate candidate), so it is an EMA over
   seconds. Any genuinely urgent geometry — small range, short TTC — **bypasses the bound
   entirely**. A fix that softens real emergency braking is worse than the fault it fixes.

### Where it lives

**`selfdrive/controls/radard.py`, not the parser.** `opendbc`'s `radar_interface.py` has no
access to `modelV2` and never should; the corroborating signal is vision, so this must sit
above the parser, after `match_vision_to_track` and before the lead is published. It is a new
layer on top of the existing chain (sentinels → innovation → u10 → D-043 rate check → rail
publish → staleness → Kalman → vision match → preferred-track staleness), and it changes none
of them.

### Validation gate — do not implement before this is satisfied

The corpus has **four** hard-brake events, one of them the fault. That cannot validate a
threshold, and D-042 is explicit about what happens when a constant is tuned on partial
evidence. Before any code:

1. Enough routes to see the radar/vision residual distribution **separately for tracks that
   later prove good versus tracks that later die** — track 6's 11 s disappearance suggests
   mortality is the label to train against.
2. A replay harness that re-runs the corpus and asserts **zero reduction in authority on the
   five benign segments** — the negative control D-009 requires.
3. The parameters chosen from that distribution, not from the single fault.

Until then this is a design, and the repo carries it as one.

## D-049 — The incarnation boundary is not published; the lead KF reset rides on one message
**Recorded 2026-09-15 from static analysis and tests against `bdf98de`. NOT IMPLEMENTED — the
fix is not the obvious one; see below.**

### What is settled

`radard`'s lead Kalman filter **is** reset across a Bosch-A track-ID reuse, but only because
the object goes absent, never because the identity changed:

- `radar_interface.py` detects the identity change (`life_delta != 2 × frame_delta`), clears
  the incarnation's range history and pops the point. The replacement needs a second coherent
  sample to mature, so the CAN identity is missing from `RadarData` for **exactly one sweep**
  and returns under the **same `trackId`**.
- `RadarD.update` pops a `Track` — and with it the `KF1D` — only when an ID is absent from the
  `liveTracks` it is looking at. **`RadarPoint` carries no incarnation field**, so the parser
  knows the identity changed and `radard` cannot.
- `card.py` publishes `liveTracks` once per sweep (~14.35 Hz); `radard` polls `modelV2` at
  `DT_MDL` (20 Hz) and reads the latest message through `SubMaster`, which keeps no queue.

So the reset is carried by **one message with nothing behind it**. Measured through the real
`RadarD` with both objects at constant velocity (true lead acceleration 0 throughout), losing
that message fabricates ≈ **0.92 m/s² of lead acceleration per m/s of identity step**, peaking
at 0.49 s and taking ~2.2 s to fall under 0.5 m/s²: a 15 m/s step yields ±13.73 m/s². D-042's
comparable 6 m/s step drove a measured −3.51 m/s² brake on `000001f3`.

Both signs are harmful: closing→opening fabricates a departing lead and **suppresses** braking;
opening→closing fabricates an approaching one and **brakes for nothing**.

Tests: `test_civic_bosch_incarnation_gap_resets_lead_kalman` and
`test_civic_bosch_coalesced_incarnation_gap_injects_phantom_lead_accel`. The first fails when
the `Track` pop is removed (D-009 negative control, checked).

### What is NOT settled, and it is the part that matters

All of the above assumes the radar **signals** the reuse. Whether Bosch-A can hand a track ID
to a new object with `life` still advancing by exactly `2 × frame_delta` — a seamless reuse —
is **not established**. If it can, nothing resets: not the parser's range history, not the lead
filter, and there is no absence for anything downstream to notice. That is D-048's "track
migrates onto the wrong scatterer" seen from the identity side, and it needs route evidence.

### Rejected: widening the gap for redundancy

The reflex fix — withhold the replacement for two sweeps instead of one so the absence cannot
be coalesced away — is **rejected outright**. It buys redundancy by deleting a radar point for
longer, which is precisely D-041 and D-042: on `000001f9` a deleted point left the planner
commanding 0.00 while closing on stopped traffic. **Never buy a reset by deleting geometry.**

The direction that does not fight D-041 is to publish the boundary as *information* — an
incarnation counter on `RadarPoint`, so `radard` can reseed the filter on an identity change
while the point keeps being published continuously. That changes a published struct and the
parser/`radard` contract, so it is a design, not a patch.

### Validation gate — do not implement before this is satisfied

1. **How often lifecycle breaks actually occur on real routes**, and whether any of them
   coincide with a hard brake. Zero observed breaks would make this a latent hazard, not a
   live one, and would change the priority.
2. **Whether seamless ID reuse exists at all** (the open question above). It decides whether an
   incarnation counter is sufficient or merely necessary.
3. A negative control per D-009: the corpus replays with **no** change in published leads on
   segments containing no incarnation break.

Until then this is a characterisation, and the repo carries it as one.

---

## D-050 — The standalone laptop tools target Python 3.9, and lint must not be allowed to break that

**Status:** settled, 2026-09-15. **Evidence:** a user's own run of `tools/konik_preflight.py`.

`tools/konik_login.py`, `tools/plain_http.py` and `tools/konik_preflight.py` exist so that a
route can be checked from a machine where the network works and the tree cannot be built. The
Python they will actually meet there is the macOS Command Line Tools build, which is **3.9** —
older than the 3.11 the rest of this repo targets.

This is not hypothetical. `pyproject.toml` sets ruff `target-version = "py311"`, so `UP017`
rewrote `datetime.now(timezone.utc)` to `datetime.now(datetime.UTC)` in the preflight. It
passed lint and every test in the container, and then died on the user's laptop before the
first check ran:

    from datetime import UTC, datetime
    ImportError: cannot import name 'UTC' from 'datetime'

The rule this settles: **the repo's Python target does not apply to these three files.**

1. `UP017` is disabled for them in `pyproject.toml`, with the reason written at the ignore.
2. `tools/lib/tests/test_py39_compat.py` enforces the constraint from two sides — it AST-parses
   each file at `feature_version=(3, 9)` (catching syntax) and greps for a list of runtime names
   that parse fine but do not exist until 3.10/3.11/3.12 (catching imports and attributes).
3. That name list is a **ratchet, not a specification**. It cannot be exhaustive. When a
   too-new name gets through and bites someone, the fix includes adding it to the list.
4. Anything new added to `STANDALONE` in that test takes on the same constraint.

**Known limit, stated per AGENTS.md §4:** there is no 3.9 interpreter in the agent container,
so this is a *static* check. It would not catch a 3.10+ behaviour change in a function that
exists in both versions. The only real verification is a user running the tool on 3.9.

## D-051 — Where the corpus harness and the recorded analysis disagree on a definition, the harness's definition is canonical and both are stated

**Settled 2026-09-15**, when `tools/bosch_a_corpus_report.py` first ran on
`00000231--5782493b00` and was compared against the 6- and 16-segment figures recorded in
`STATUS.md`. Those figures came from code that was never committed, so a disagreement had no
tie-breaker; three of them turned out to be **definitions**, not measurements.

1. **The radar/vision range gap carries `RADAR_TO_CAMERA`.** The harness computes
   `dRel - (lead.x - 1.52)`; the recorded figures computed `lead.x - dRel`. Every recorded gap
   percentile is reproduced to within 0.9–1.7 m by that 1.52 m offset (6 segments, n=4,872 here
   against 4,868 recorded). **The harness's definition wins** because it is the one
   `radard.track_matches_vision` actually applies when deciding whether a track matches a lead —
   a gap statistic that does not match the matcher cannot inform a matcher threshold.
2. **Vision-lead occupancy and residuals are cut at `prob >= 0.5`; the gap distribution at
   `>= 0.9`.** At 0.9, `ed6257ef` (segment 25) reads 1.4% vision-lead occupancy against the
   recorded 94.2%; at 0.5 it reads 94.2% exactly, its lateral residual reproduces at +4.40 m
   (recorded 4.30) and `7b76edd7` (segment 24) at +4.36 (recorded 4.33). Both cuts are now
   reported side by side rather than one being silently chosen.
3. **U11-versus-range-rate pairs use MEASURED points only, inside contiguous runs.** Coasted
   points carry a held velocity, not U11, and differentiating across a dropout or an ID reuse
   manufactures range rates — 55.8 m/s at the extreme. This one does **not** reconcile: the
   harness reports 9.16% of samples disagreeing by more than 5 m/s against a recorded 2.26%, and
   no lag at the cross-correlation peak against a recorded −4 samples. **Neither side wins**; see
   `STATUS.md`.

Rejected: quietly adopting the recorded definition to make the numbers match. The offset and the
probability cut are each a *decision about what the statistic means*, and the recorded analysis
cannot be re-run to defend its choice.

**Consequence for anyone reading older figures:** a gap quoted before this date is ~1.5 m larger
than the same gap quoted after it, and an occupancy quoted before this date is the 0.5 cut.

---

## D-052 — A saturated lifecycle counter is not an identity change; keep publishing the object

**Settled 2026-09-15 from two real drives.** `LIFECYCLE_RAW` is 12 bits and advances by 2 per frame
index, so it pins at **0xFFE (4094) after 2,047 frames — ~137 s of continuous tracking** at the
~14.9 Hz sweep rate, and never advances again. It is **not** the invalid sentinel (0xFFF): STATUS,
range, azimuth and track id all stay valid and the object is still there.

`life_delta == 2 * frame_delta` therefore failed on every subsequent sweep, clearing the range
history and popping the point each time, so no point could ever mature again.

### What that cost, measured

* `00000232--fc8dad0d18`: track 37 saturated at an age of **137.4 s** and was then observed on every
  sweep for **121.8 s (1,815 sweeps) and published on none of them**. Its last published geometry
  was dRel 38.9 m, yRel −0.1 m — the followed lead. Track 34 did the same at 137.5 s.
* `0000020a--1fd2b58eda`: track 38 was suppressed across segments 5→8 (~3.7 min, starting at route
  time 5:05.17) and track 51 across segments 11→12.
* **Corroborated by the device, not only by replay.** On the same CAN, the car (running `0083ffa`,
  without this fix) published id 38 on **0/893, 0/894, 0/893** sweeps of segments 6, 7 and 8, while
  this tree's parser publishes 93.1%, 100.0% and 82.3%. On `00000232` the recorded `liveTracks` drop
  id 37 entirely across segments 10–12 and the radar lead goes 1200/1200 → 0/1200 frames.
* The driver felt it: the 5:05 brake on `0000020a` handed from radar to vision mid-manoeuvre, and
  the 7:43 stop-and-creep ran with no radar lead at all.

### The decision

A counter pinned at its maximum **cannot testify to identity either way**, so it must not be read as
evidence of a new incarnation. `BOSCH_A_LIFE_SATURATED` makes a saturated→saturated step a
continuation. This follows D-041 and D-042: publishing a bounded/uncertain point is safer than
deleting a real object, and the range-innovation gate still judges every sweep while
`_bosch_a_retire_stale_tracks` still retires a genuine disappearance.

Verified after the change: id 37 publishes **1,788/1,815 (98.5%)** over the same window, id 34
180/184, and the five-route lifecycle census reports **zero chronic runs**.

**Rejected:** treating saturation as a death and letting the track re-birth. That is the same
"delete the geometry to buy a reset" move D-049 already rejected, and here it deletes a lead that is
still in front of the car.

### The cost, stated plainly

An ID reuse that happens **while the counter is saturated** is now undetectable at the parser: there
is no counter movement left to break. That narrows D-049's gate item 2 rather than closing it, and
it is the price of not deleting a live lead. A tracked object old enough to saturate has been held
for over two minutes, which makes a reuse at that moment less likely but not impossible.

**Negative control (D-009):** a counter frozen at any non-saturated value is still a lifecycle break
— `test_a_stuck_but_unsaturated_counter_is_still_a_lifecycle_break`.
