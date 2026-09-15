# Status

**As of: 2026-09-15**

Update the date above whenever this file changes. If it is stale, trust `git log` over this
file.

Repo: StarPilot / openpilot fork `openpilot-radar`. Working branch
`ns-bosch-radar-testing`, tip `0083ffa` ("fix: define raw lead safety distance",
2026-09-11, JamesL787). `claude/radar-testing-state-88vt2t` is identical to it.

**Scope note.** This file covers the **radar and longitudinal** work in this repo. The EPS
firmware programme (RWD tunes, the `0x6A0..0x6A8` telemetry stub, the gain bench, UART/UDS
flashing, D-001..D-026) lives in the separate EPS knowledge-base repo and is **not** tracked
here. Where the two touch — the CR-V lateral profile, the steering-ratio curves, the
`extract_drives.py` lineage — that is recorded below as a cross-reference only.

---

## What the Bosch-A radar work is

Honda's Bosch-A platform exposes a **16-slot object bank** on the camera bus that openpilot
upstream does not parse at all. This branch reverse-engineers it and feeds it to `radard`
as a real radar source, replacing vision-only lead tracking on those cars.

`[CONFIRMED static]` The wire format is one 16-slot bank: each slot has four main frames
(`f0..f3`, one CAN ID each — **not** sub-frames muxed onto a shared ID) plus one
synchronized auxiliary frame. This supersedes the earlier model of `0x280/0x284/0x288/0x28C`
as pieces of a single object, and of `0x2C8/0x2C9` as a separate "coarse" list.

| piece | where |
|---|---|
| Hand-written DBC, 896 signals | `opendbc_repo/opendbc/dbc/honda_bosch_a_radar.dbc` |
| Parser, decode constants, all gates | `opendbc_repo/opendbc/car/honda/radar_interface.py` |
| Lead selection, staleness, shadow telemetry | `selfdrive/controls/radard.py` |
| Enable toggle | Param `BoschARadar` (UI: Longitudinal → "Bosch A Radar") |
| Car set | `HONDA_BOSCH_A` = `HONDA_BOSCH` − radarless − CANFD − alt-radar |

The DBC is **hand-written and not produced by the opendbc generator pipeline.** Its
`VERSION` string records the validation: **361,360/361,360 checksum passes and
214,400/214,400 sequential counter transitions** on route `000001df`. `FRAME_IDX` and
`LIFECYCLE_RAW` are deliberately not named `COUNTER` — the real Honda rolling counter is the
2-bit field at last-byte bits 5:4, alongside the 4-bit checksum at bits 3:0, and both are
declared so opendbc enforces them.

### Decode constants that are firmware-exact, not fitted

`[CONFIRMED static, from firmware]` — do **not** re-fit these against vision:

- **Range scale = 1/16 m per count.** AC004 converts with `(q16 − n)/128` where
  `q16 = sat16(round(8 × raw_range))`, so `range_m = raw_range/16 − n/128`. Corroborated
  three ways: `8 × 4095 = 32760` fits int16 with 7 counts to spare (the `×8` exists to make
  the 12-bit field fill the internal word), full scale is `4095/16 = 255.9 m`, and `/128`
  (Q7) is this firmware's unit for physical quantities throughout. The previous `0.05712`
  was solved from a **single tape point with the offset assumed** and read ~9% short —
  −9 m at 60 m against vision.
- **Range offset is a per-unit calibration value, not a constant.** The firmware term is
  `−n/128`, with `n` assembled from a config word plus a runtime addend; `335` (−2.617 m) is
  only the fallback when the config word reads zero. `−3.0` is retained because it sits
  inside the plausible calibration range. Read it from the radar's own configuration rather
  than re-fitting it.
- **Azimuth = `(raw_angle − 1024)/2048` rad**, closed independently by the `f3` angular-edge
  pair. Supersedes the older empirical 0.032 deg/count fit.
- **Sweep cadence = 14.35 Hz** (median of `0x280` inter-arrival across Peter's routes; p5
  12.5, p95 16.9). This also sets the lead-Kalman `dt` in `radard` — the previous round 15
  ran the filter ~4.5% fast.

### The U11 / u10 velocity channel — the expensive part

`[CONFIRMED, replay + route evidence]` The auxiliary frame carries an 11-bit offset-binary
velocity field (**U11**, `1/64` m/s per count, centre 864) and a 10-bit companion
uncertainty (**u10**). Two hard-won rules govern it, and both were learned by breaking them:

1. **A saturation rail is a bound, not a missing measurement, and it must still be
   published.** Raw 0 and 1728 mean `|vRel| ≥ 13.5 m/s` with the exact value unrecoverable.
   This was briefly routed to the coast path. That was a **safety regression**: a stationary
   car approached above 13.5 m/s (30 mph) rails on *every* sweep, so the coast never ended,
   it outlived `BOSCH_A_STALE_S`, and the point was deleted. Measured on route `000001f9` at
   29:52 — two stopped cars, **88 of 88 active frames on the low rail** with healthy u10
   (78–94), range closing smoothly at −19.4 m/s. The radar lead was dropped, `radard` fell
   back to a vision lead reporting only −11.4 m/s, and the planner commanded **0.00 while
   closing on stopped traffic at 76 m with a 6.6 s TTC. The driver intervened.** See D-041.
2. **u10 is confounded with dynamics; do not re-tune it from offline error statistics.**
   Over 16,834 frames against an event-local reference, median |err| rises 0.26 → 0.88 →
   1.44 → 1.95 m/s across u10 bins 0-64/64-128/128-192/192-256 — but median |a_rel| rises
   0.73 → 2.13 → 3.51 → 4.59 m/s² alongside it (`corr(u10,|err|) = +0.40`,
   `corr(u10,|a_rel|) = +0.43`). The threshold **511** was validated by replaying 20
   bookmarks through the real `RadarInterface` and `radard`'s Kalman filter. Lowering it to
   128 on error statistics alone undid that and caused a measured regression: on route
   `000001f3` at 19:27 u10 sat above 128 for 0.81 s during a real ~8 m/s² lead decel, the
   coast outlived `BOSCH_A_STALE_S`, the lead was deleted, and the vision fallback injected
   a 5 m / 6 m/s step that drove a −3.51 m/s² brake. Restored. See D-042.

`[CONFIRMED]` The **multi-sweep velocity/range consistency check is one-sided by design.**
The per-sweep innovation gate cannot see a velocity error at all — over one ~70 ms sweep even
a 3 m/s error moves the range 0.2 m, far under its 2–5 m thresholds. U11 *lags* true closure
at a deceleration onset (at `000001f3` 19:27 the fitted range rate was −6.7 m/s while U11
still read −2.08), so a symmetric test fires during genuine hard braking — exactly when the
velocity is most needed. Only "U11 claims more closing than the geometry supports" is
evidence of a fault. Checked both ways: `000001eb` 6:59 (U11 −10.58 while the range *opened*
at +0.46, an 11 m/s contradiction across a track-identity change) is rejected; the
`000001f3` onset is not. See D-043.

### Shadow range-derived vRel — telemetry only

`[CONFIRMED static]` `Track.vRelRange` publishes a 5-sample LSQ range rate alongside the
radar's own `vRel`, for **whichever** radar lead is selected — not only Bosch-A. **Nothing
consumes it.** It is plain LSQ with no outlier rejection, so it inherits the range channel's
~1% gross outliers: a diagnostic, not a validated velocity. Timestamps come from the message
clock, not wall clock, so accelerated replay stays faithful.

Motivating measurements: on `000001fe/fb/fd`, U11 detects a closing onset 0.88–1.28 s late
while a 4-sample range LSQ lands within 0.07–0.14 s; on `00000141` R141-8, U11 diverged from
the range by 13.7 m/s while the range moved 1.9 m. It exists to confirm or refute that on a
real drive **before any of it is wired into control.** See D-044.

`[CONFIRMED]` Two defects in this telemetry have already been fixed and are worth
remembering as a pattern, not just as history: `05930e2` — the shadow fields were written to
the **wrong capnp struct**, crashing `radard` on every Bosch-A drive; `d8604bd` — four
further defects where the **test evidence was measuring a stale mirror** of the value under
test. Both passed review first. See AGENTS.md §4.

---

## Build and test environment — solved 2026-09-15, record the recipe

`[CONFIRMED this session]` The repeated note in prior status writeups — "the radard tests
cannot run on this checkout because `msgq/ipc_pyx.so` is not a valid Mach-O binary" / "is a
Linux binary" — was **not** a platform limitation. The checked-in `.so` files are
**aarch64**, built for the comma device:

```
msgq_repo/msgq/ipc_pyx.so: ELF 64-bit LSB shared object, ARM aarch64
```

On any other host they fail to load with `cannot open shared object file`, which reads like
a missing file rather than a wrong architecture. **Rebuild locally and the tests run.**

Two traps to know before you start:

1. **`.venv` at the repo root is a tracked symlink to another machine's home directory**
   (`/Users/jameslichtenstiger/nrdr/openpilot/.venv`, mode `120000`, introduced in
   `82796af`). `uv sync` fails with `failed to create directory .venv: File exists`. Point
   `UV_PROJECT_ENVIRONMENT` elsewhere. Do not delete the tracked link as a side effect of
   unrelated work — retire it deliberately, as its own commit.
2. `uv sync --frozen` does not complete cleanly here (it fails on `pyaudio`, and
   `json-rpc` needs `UV_HTTP_TIMEOUT` raised). The targeted install below is sufficient for
   the radar and longitudinal suites.
3. **The build overwrites 26 tracked device binaries in place.** `git status` after a build
   shows `msgq/ipc_pyx.so`, `common/params_pyx.so`, both acados solvers, `libcereal.a`,
   `pandad` and others as modified. Committing them would ship **x86_64 artifacts to a
   branch that boots on an aarch64 comma.** Stage explicitly; never `git commit -a`. See
   AGENTS.md §10 for the restore command.

### Working recipe (x86_64 Linux, verified end to end this session)

```bash
sudo apt-get update
sudo apt-get install -y capnproto libcapnp-dev libzmq3-dev opencl-headers \
                        ocl-icd-opencl-dev libeigen3-dev libusb-1.0-0-dev

export UV_PROJECT_ENVIRONMENT=/tmp/opvenv       # NOT the repo's broken .venv
uv venv --python 3.11 "$UV_PROJECT_ENVIRONMENT"
uv pip install --python "$UV_PROJECT_ENVIRONMENT/bin/python" \
  numpy pycapnp pytest pytest-xdist pytest-asyncio pytest-cpp cython scons setuptools \
  smbus2 pyzmq sentry-sdk requests psutil pyserial tqdm zstandard crcmod setproctitle \
  pyjwt libusb1 python-dateutil pycryptodome cffi sympy casadi future-fstrings \
  parameterized hypothesis ruff

export PYTHONPATH=$(dirname "$PWD"):$PWD
scons -j8 msgq_repo/ cereal/ opendbc_repo/ common/ selfdrive/
```

`pytest-asyncio`, `pytest-cpp` and `pytest-xdist` are **not optional** — `pyproject.toml`
sets `--strict-config` plus `-n auto --dist=loadgroup`, and without them pytest exits
`Unknown config option` and reports *"no tests ran"*, which looks like a collection problem
rather than a missing plugin.

### Test results, this tree, this session

| suite | result |
|---|---|
| `opendbc_repo/opendbc/car/honda/tests/test_bosch_a_radar.py` | **91 passed** |
| `opendbc_repo/opendbc/car/honda/tests/test_honda.py` | **116 passed** |
| `opendbc_repo/opendbc/car/honda/tests/test_carcontroller_learners.py` | **31 passed** |
| `selfdrive/controls/tests/test_radard_bosch.py` | **24 passed** |
| `selfdrive/controls/tests/test_lead_behavior.py` | **38 passed** |
| `selfdrive/controls/tests/test_lead_follow_policy.py` | **14 passed** |
| `selfdrive/controls/tests/test_following_distance.py` | **18 passed** |
| `selfdrive/controls/tests/test_turn_lead.py` | **34 passed** |

**366 passed, 0 failed.** `test_lead_follow_policy` and `test_following_distance` require
the acados MPC solvers, so `scons selfdrive/` must have completed — collecting them against
an unbuilt tree raises `acados_ocp_solver_pyx.so: cannot open shared object file`, which is
the same architecture trap wearing a different hat.

**`selfdrive/controls/tests/test_leads.py` does not run here** — it drives
`replay_process_with_name` from `selfdrive/test/process_replay`, which spawns real processes
and hangs in this sandbox. Untested, not failing. Running it needs a proper process-replay
environment.

**Lint:** `ruff check` is clean on `selfdrive/controls/radard.py`,
`test_radard_bosch.py`, `lead_behavior.py`, `lead_follow_policy.py`. It reports **2 findings
in `opendbc_repo/opendbc/car/honda/radar_interface.py`** under opendbc's *own* config —
`E303` too many blank lines at line 534, and `B905` `zip()` without explicit `strict=` at
line 601. Both are real and unfixed. Run ruff from the tree that owns the file; opendbc has
its own `pyproject.toml`.

---

## Alpha Long P061B — exact onset captured; carried over, not re-verified here

> The material in this section was established in the EPS knowledge-base repo against
> routes and forks not present in this checkout. It is carried forward because it governs
> the Honda longitudinal code on this branch. **Nothing in it was re-verified in this
> repo this session** — treat the route claims as prior evidence, and re-derive before
> relying on any of them for a new change.

Route `00000002--aa8501ddcb` (JamesL787/openpilot `ns-bosch-radar-testing`, commit
`7f2bab7b…`) contains the exact P061B onset. At 951.918310 s, PCM-side CAN `0x400` changes
from zero to `06 1b 01 02 …` and keeps broadcasting the code. For **3.681 s immediately
beforehand**, Alpha Long continuously sent positive `GAS_COMMAND` together with negative
`ACCEL_COMMAND`, `BRAKE_REQUEST=1` and `BRAKE_LIGHTS=1` on Honda `0x1DF`. The route contains
**1,584 such contradictory frames**.

`[CONFIRMED static/log analysis]` The installed source gates gas on drag/grade-adjusted
`gas_force > 0` but gates braking independently on **raw** `accel < −0.2`, allowing both
paths at once. `[INFERRED, high confidence]` This is the P061B trigger: the exact onset
during a sustained contradiction matches Honda's time/integrated torque-monitor semantics
and explains the cross-fork/stock-ACC A/B. `[CONFIRMED]` It is **not** memory exhaustion —
control processes are flat after engagement, available RAM never falls below 554.64 MiB, and
there is no OOM, process restart, CAN invalidity, or openpilot `accFaulted` sample.

**Do not use** the faulting Alpha Long build, or test commit `6e583e1c43c5…`, on this
vehicle. Route `00000003--1423cb6de2` proves the test commit's raw-zero gas gate causes
severe closed-loop acceleration/coast oscillation: in its only 87.3 s longitudinal
engagement the gate suppressed a still-positive calculated gas value for ~23.96 s and
repeatedly reapplied hundreds of gas units as requested acceleration crossed zero. It did
prevent gas-plus-brake in that route, but 87 s cannot validate a fault that took ~585 s of
engagement to appear. See D-033.

**Current candidate `694046f33f58…`** derives brake entry (`adjusted force < −0.12 m/s²`)
and release (`> −0.02 m/s²`) from the **same** grade/aero-adjusted quantity used for gas,
with an explicit stopping override and a CAN-boundary gas inhibitor. Open-loop replay cuts
route-4 brake runs 39 → 11; on the complete fault route it preserves gas while removing
braking from all 1,584 conflict frames and cuts brake runs 22 → 11, with **zero replayed
gas-plus-brake frames**. See D-034, D-036.

Its predecessor `57d77a7e1918…` was first vehicle-tested on route `00000004--dcc6a5b59e`:
substantially better than the raw-zero candidate, but it did not restore pre-fix smoothness.
In 291.24 retained active seconds, 2,336 brake frames (46.72 s) occurred while reconstructed
grade/aero-adjusted tractive force was positive; 1,987 of those suppressed a positive
calculated gas output, concentrated in lead-following on positive-pitch inclines, including
561 frames just below the raw −0.2 m/s² threshold.

**This is command replay, not closed-loop validation.** It cannot predict the changed future
speed response or prove PCM acceptance. No P061B appears in the route-4 windows, but their
total active duration is **shorter than the original time-to-fault**, so the fault correction
remains unverified.

### Upstream PRs (state as last recorded — re-check before citing)

- `JamesL787/openpilot` **PR #9** — candidate merged through target tip `3f27c6faaf5b…` as
  merge commit `1ddce1708420…`. Reported `MERGEABLE`/`CLEAN`, diff confined to the three
  intended Honda controller / CAN packer / regression-test files. Post-merge: 232 Honda
  tests and 393 applicable Panda Honda safety tests pass (271 expected skips).
- `firestar5683/StarPilot` **PR #124** — the same final arbitration ported from StarPilot
  `Dom` tip `9d1043ad0114…` as `3a811e9622d9…`, adapted to StarPilot's `CarParams` CAN-packer
  interface and preserving its separate Accord 11G MVL crossover. Reported `MERGEABLE`/
  `CLEAN`, same three-file diff. StarPilot Honda suite 210 tests; Panda Honda safety 369
  tests (267 expected skips); Ruff clean. The owner reports **no fault recurrence so far and
  smoother longitudinal behaviour, especially in full E2E mode** — that is limited road
  evidence, not proof of long-duration P061B prevention or universal thresholds. See D-038.

In both records, the planner/`radard` tests "could not load locally" for the architecture
reason now solved above. Re-run them.

---

## Related lanes (cross-reference only — not this repo's branch)

`[carried over]` An alternate OpenPilot lane at `RiskyBiscuit-arc/openpilot` `arc-dev`
carries the firmware-extracted 30-point CR-V steering-ratio curve, additive lateral-control
telemetry, and the EPS `0x6A0..0x6A8` extractor; PR #3 (`70e0773f…`) corrects VGR ordering so
measured `currentCurvature` uses the direct measured-angle rack ratio before
`VehicleModel.calc_curvature`. Branch `codex/night-star-bosch-radar-telemetry` (`c62d4034c4`)
is the tester-gated CR-V Bosch-A parsing lane built from JamesL787 `f96cfdb8f7…`; both
retained CR-V rlogs contain the complete object family on `CanBus.camera` (bus 2), with 136
and 352 `0x2FF` sweep triggers, offline parser replay completing every trigger with no CAN
decode error — **all objects were firmware no-target sentinels.** See D-027, D-028, D-030.

---

## NOT verified — do not treat as safe

- **No road validation with real radar targets on this branch.** Every Bosch-A replay to date
  that completed cleanly did so against **no-target sentinels**. Parsing a sweep correctly is
  not the same as tracking a car.
- **The P061B correction is unproven.** Route 4's active duration is shorter than the
  original time-to-fault. Replay cannot predict the changed closed loop or PCM acceptance.
- **Fusion timing is untested here.** `radard` runs at the ~20 Hz model rate while Bosch-A
  measures at ~14.35 Hz; the duplicate-payload path (`measurement_update=False`) is covered
  by unit tests only, never on a live bus.
- **`test_leads.py` has not been run** in this environment (process-replay harness).
- **The two ruff findings in `radar_interface.py` are unfixed.**
- **`.venv` remains a broken tracked symlink**, and the checked-in `.so` files remain
  aarch64. Both will bite the next person on a non-comma host.

## Next

1. Fix the two `radar_interface.py` ruff findings and retire the `.venv` symlink — both
   small, both isolated, neither touching a gate.
2. Get `test_leads.py` running in a real process-replay environment; it is the only radar
   suite still unexercised.
3. Re-run the `radard`/planner suites against the Alpha Long PR branches now that the
   architecture blocker is understood — both PR records list them as skipped for that reason.
4. Capture a Bosch-A route with **real targets**. Every parser claim above is still resting
   on no-target sentinels, and the shadow `vRelRange` channel (D-044) exists precisely to be
   confirmed or refuted on such a drive before anything is wired into control.
