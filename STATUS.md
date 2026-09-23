# Status

**As of: 2026-09-23**

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

**Open topics to revisit** (parked by decision, not closed):
- **Brake over-delivery and the low-speed stop-and-go jolt: item 72.** Parked 2026-09-23. Reopen
  when there are about 10 or more low-speed gas-to-brake onsets (below 10 m/s) in rlogs. Today there
  are 3. Also reopen if a drive reports the jolt again.

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
| `selfdrive/controls/tests/test_radard_bosch.py` | **26 passed** |
| `selfdrive/controls/tests/test_lead_behavior.py` | **38 passed** |
| `selfdrive/controls/tests/test_lead_follow_policy.py` | **14 passed** |
| `selfdrive/controls/tests/test_following_distance.py` | **18 passed** |
| `selfdrive/controls/tests/test_turn_lead.py` | **34 passed** |

**368 passed, 0 failed.** `test_lead_follow_policy` and `test_following_distance` require
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

## ✅ First real-target Bosch-A route — 2026-09-15

`[CONFIRMED, one 60 s segment replayed through the real parser]` **The sentinel-only era is
over.** Segment `00000231--5782493b00` (device `11c8fa231c0499ed`, car **`HONDA_CIVIC_BOSCH`**,
`radarUnavailable=False`) is the first Bosch-A capture in this project with real tracked
objects. This supersedes the standing caveat that every clean replay had contained only
firmware no-target sentinels.

| | |
|---|---|
| Bosch-A frames | 144,380 — **all 80 addresses**, buses 1 / 2 / 128 |
| Sweep triggers (`0x2FF`) | 1,786 |
| Parser output | **2,878 points across 892 sweeps, 33 distinct track IDs** |
| Range / velocity | dRel 10.9–111.2 m, vRel −13.50 … +4.52 m/s |
| Device-side | `liveTracks` 893, `radarState` 1200 (a lead on **all** 1200) |

Reproduce: `python tools/bosch_a_route_report.py <rlog.zst> --fingerprint HONDA_CIVIC_BOSCH`

### D-041 confirmed on the road, not just in replay

`[CONFIRMED]` The U11 saturation rail is **not** an edge case: **77 samples across 12
distinct tracks hit exactly −13.50 m/s in this single 60-second segment**, and every one of
them was published as a radar point rather than coasted.

The clearest instance is track 58 at t=5.79–6.73 s (y ≈ +10.2…+11.5 m, d 66.9 → 49.3 m):
U11 read the rail on all 14 samples while the **range itself closed 17.6 m in 0.94 s =
−18.69 m/s**, with ego at 16.40 m/s. So the rail understated true closing by ~5.2 m/s —
exactly the documented trade in D-041 — and the point survived. Under the pre-`6126e51`
behaviour those 77 samples would have fed the coast path.

This is the first road evidence that publishing the bound is the correct call, and it
strengthens rather than revisits D-041. It is **not** a licence to re-tune the rail or u10
from one route (D-042).

### Vision and radar disagreed on lead range by ~11 m

`[CONFIRMED]` Around t=37.7 s the model put its lead at `x` 54–57 m (`prob` 1.00,
`y` ≈ −0.1 m) while radar track 6 had it at dRel ≈ 44 m. Allowing for `RADAR_TO_CAMERA`
(1.52 m) that is an ~11 m disagreement. `radard` published the radar range (43.5 m,
`leadOne.radar=1`), which is the right choice — but the match only survives because
`track_matches_vision` uses `dist_scale=0.25`, i.e. a ~13.75 m tolerance at that range. The
margin here is thinner than it looks; worth watching before anyone tightens that scale.

### Bosch-A track IDs are reused within a segment

`[CONFIRMED]` Track ID 58 covers **two unrelated objects 27 s apart**: y ≈ +10.2…+11.5 m
(t 5.8–6.7) and then y ≈ −3.5…−2.5 m (t 33.8–60.0). Bosch-A IDs are 6-bit
(`BOSCH_A_TRACK_ID_MIN=1`, `MAX=0x3F`), so reuse is expected. The second life opens at
vRel +3.03 then reads −2.16 within 0.55 s, which reads like a **fresh** Kalman filter
converging rather than stale state carrying over.

**Settled 2026-09-15 — the filter does reset, but on a one-message margin.** See the section
below; the open item is closed and replaced by a narrower one.

### Track-ID reuse and the lead Kalman filter — settled, with a fragility

`[CONFIRMED static]` Synthetic input driven through the real `RadarInterface` and the real
`RadarD`, against commit `bdf98de`. The question left open above was whether a Bosch-A
track-ID reuse resets `radard`'s lead Kalman filter. **It does — but by object absence, never
by the incarnation boundary the parser actually computes.**

The chain, end to end:

1. **The parser signals the identity change but does not publish it.** A lifecycle
   discontinuity (`life_delta != 2 × frame_delta`) clears the incarnation's range history and
   pops the point. The replacement cannot be republished until a second coherent sample gives
   it a finite derivative (`matured`), so the CAN identity is **absent from `RadarData` for
   exactly one sweep** and then returns *under the same `trackId`*. Pinned by the existing
   `test_in_place_replacement_no_invalid_gap_resets_history_but_keeps_can_id`.
2. **That one-sweep absence is the entire reset signal `radard` receives.** `RadarD.update`
   pops a `Track` — and with it the `KF1D` — only when an ID is missing from the `liveTracks`
   it is looking at. There is no incarnation field on `RadarPoint`: the parser knows the
   identity changed and `radard` cannot.
3. **Observing the gap resets the filter cleanly** (new
   `test_civic_bosch_incarnation_gap_resets_lead_kalman`): the `Track` is reconstructed and the
   new filter is seeded from the new object's own `vLead`, reporting it with **zero** phantom
   acceleration.

So track 58 is reset — though by staleness retirement, not by the lifecycle path, since a 27 s
absence expires `BOSCH_A_STALE_S` many times over. **The lifecycle path is the interesting one,
because it is one message wide.**

**The fragility.** `card.py` publishes `liveTracks` once per sweep (`RadarInterface.update`
returns `None` without a `0x2FF` trigger), i.e. at ~14.35 Hz. `radard` polls `modelV2` at
`DT_MDL` (20 Hz) and reads the **latest** `liveTracks` through `SubMaster`, which keeps no
queue. The gap survives only while the sweep interval stays longer than the model period —
nominally true (median 14.35 Hz, p95 16.9 Hz on `000001df`), but carried by a **single message
with nothing behind it**. Coalesce or drop that one message and the settled filter absorbs the
identity change as a step.

`[CONFIRMED static]` What that costs, measured through the real `RadarD` (new
`test_civic_bosch_coalesced_incarnation_gap_injects_phantom_lead_accel`), with both objects at
constant velocity so the **true lead acceleration is 0 throughout**:

| identity step | peak phantom `aLeadK` | at | back under 0.5 m/s² |
|---|---|---|---|
| −12 → +3 m/s (15 m/s) | **+13.73 m/s²** | 0.49 s | 2.16 s |
| +3 → −12 m/s (15 m/s) | **−13.73 m/s²** | 0.49 s | 2.16 s |
| −0.5 → −6 m/s (5.5 m/s) | **−5.03 m/s²** | 0.49 s | 1.88 s |

≈ **0.92 m/s² of fabricated lead acceleration per m/s of identity step**, in whichever
direction the step runs. For scale, D-042 records a vision fallback injecting a 6 m/s step that
drove a measured **−3.51 m/s²** brake on `000001f3`. The sign matters both ways: a closing→
opening reuse fabricates a *departing* lead and suppresses braking; opening→closing fabricates
an *approaching* one and brakes for nothing.

**This is a characterisation, not a demonstrated on-road defect.** No route evidence shows a
`liveTracks` message being coalesced at an incarnation boundary, and under nominal timing the
reset works. What is established is that the margin is one message and the consequence of
losing it is a multi-m/s² phantom acceleration lasting ~2 s.

**The narrower open item that replaces the old one.** All of the above assumes the radar
*signals* the reuse. Whether Bosch-A can hand an ID to a new object **without** a lifecycle
discontinuity — `life` continuing to advance by exactly `2 × frame_delta` across the change —
is **not established**, and if it can, nothing resets: not the parser's range history, not the
lead filter. That is D-048's “track migrates onto the wrong scatterer” seen from the identity
side, and it needs route evidence to answer. See D-049.

### 🔴 FALSE BRAKE at t≈11 s — radar range drift, and it defeats all three gates

`[CONFIRMED, one segment]` **This is what the driver flagged** (they bookmarked ~26 s after
the event, reporting "a sharp slowdown as if locking on to a much closer target, even though
the lead in front still has plenty of room"). It is a genuine radar fault with a real
actuation consequence, and it is the most important finding in this segment.

**What happened.** Between t=9.1 and 11.35 s, radar track 6 — the selected lead, own lane,
y ≈ 0.0 m — reported its range falling 71.8 → 61.6 m and its `vRel` ramping
−0.59 → **−6.02 m/s**. Over the same window the **vision lead held x ≈ 75–78 m**, `prob`
0.93–1.00, implying `vRel` of only −0.16 to −2.00 m/s. The radar-versus-vision range gap grew
**monotonically: +7.4 → +10.4 → +15.8 → +15.3 → +14.5 m**.

`radard` published the radar lead (`leadOne.radar=1`). The planner commanded **−2.89 m/s²**
and measured **aEgo −4.28 m/s²**. The driver overrode with the accelerator at t≈11.5
(`gasPressedOverride`, `gasPressed`). Track 6's final sample was `measured=0` — coasting,
`vRel` frozen at −4.36 for three samples — and the track then **disappeared for 11.02 s**
(11.35 → 22.36 s), the only dropout over 0.3 s anywhere in the segment. It returned at
53.1 m with a vision gap back down to +7.1 m.

`[INFERRED, high confidence]` The radar **range** on track 6 drifted ~6 m toward the car
while the lead vehicle did not approach. Supporting: vision was stable and confident
throughout; the gap grew monotonically rather than jittering; the track coasted and then died
immediately after; and the driver reports the gap was never closing.

**Why every existing gate passed it — this is the part that matters:**

| gate | why it did not fire |
|---|---|
| Per-sweep range innovation (`..._INNOVATION_MAX_M = 2.0`, hard 5.0) | drift was ~6 m over 1.7 s ≈ **0.25 m per sweep**, far inside 2.0 m |
| One-sided multi-sweep vRel/range check (D-043) | U11 (−6.0) and the fitted range rate (≈ −4.5 to −6.4 m/s) **agreed with each other**. The check only fires when U11 claims *more* closing than the geometry supports, so it **structurally cannot** catch this |
| Gross-distance staleness (`HONDA_BOSCH_A_GROSS_DISTANCE_M = 25.0`) | peak disagreement 15.8 m, **under** the 25 m threshold |

`[CONFIRMED by construction]` The shadow `vRelRange` channel (D-044) is **also blind to
this**: it is an LSQ fit of the same range that drifted, so it would have agreed with U11.
The diagnostic published specifically to cross-check U11 cannot catch a *range* error, and
this event is a range error. That is a real limit on D-044's usefulness and should be
recorded against it.

**The signal that would have caught it is radar-versus-vision range disagreement** — 15.8 m
at a 78 m vision range is ~20%. `HONDA_BOSCH_A_GROSS_DISTANCE_M = 25.0` is too loose to see
it. **This is a candidate, not a change.** Per D-042, one route is not grounds for moving a
threshold, and the two constants previously re-tuned on partial evidence both caused measured
regressions. What is needed first: how often a 10–20 m radar/vision gap occurs on routes
where the radar is *right* (vision range at 70–80 m is itself weak), which cannot be answered
from one segment. **Do not tighten this constant until that distribution is known.**

### Six-segment corpus — two candidate fixes refuted, and a correction

`[CONFIRMED, 6 segments from one drive, ~6 min]` Five further segments
(`f66399a5`, `1be0aa43`, `97566dde`, `9e21cac9`, `bf574f16`) were analysed alongside the
flagged one. **The corpus refutes the threshold candidate this file proposed above**, which is
recorded rather than quietly dropped.

**Radar-versus-vision range gap, pooled** (radar lead selected, vision `prob` ≥ 0.9,
n = 4,868):

| p5 | p25 | p50 | p75 | p90 | p95 | p99 | max |
|---|---|---|---|---|---|---|---|
| −0.2 | +1.4 | **+4.0** | +7.2 | +11.0 | **+14.9** | +21.7 | +27.7 m |

Vision reads **systematically longer** than radar — expected for monocular depth. The
false-brake event's 15.8 m gap is therefore only ~p95, **not an outlier**. Worse, segment
`9e21cac9` runs at a **median gap of 13.8 m with a maximum of 27.7 m and commands no hard
braking at all**. Tightening `HONDA_BOSCH_A_GROSS_DISTANCE_M` toward ~12 m, as suggested
above, would fire continuously on a segment where nothing is wrong. **Candidate dead.**

**Gap *rate* was tested as an alternative and is also refuted.** |d(gap)/dt| pooled: p50 4.7,
p90 22.3, p95 31.1, max 130.2 m/s — vision's frame-to-frame `x` jitter dominates. `9e21cac9`
reaches p95 = 53.7 m/s while braking normally. **Candidate dead.**

`[CORRECTION]` The entry above states the vision evidence showed the radar was wrong. Given
this distribution that was **over-claimed**: a 15.8 m gap at 78 m is within normal vision
spread and proves nothing on its own. The genuine evidence that track 6 was spurious is
(a) it **vanished for 11.02 s** immediately afterwards — the only dropout over 0.3 s in that
segment — and (b) the driver overrode and reports the gap never closed. Treat the gap as
corroborating, not as proof.

**Hard-brake events are too few to tune anything.** Across all six segments there are only
**four** runs with commanded accel < −1.5 m/s² while engaged:

| segment | t | accel | dRel | vRel | vEgo | headway | gap |
|---|---|---|---|---|---|---|---|
| `3a4e0842` **(false)** | 11.21 | −2.89 | 62.4 | −4.36 | 17.6 | **3.56 s** | 14.8 |
| `97566dde` | 43.28 | −2.00 | 40.0 | −2.20 | 19.6 | 2.04 s | 10.5 |
| `f66399a5` | 5.21 | −2.17 | 33.9 | −0.62 | 13.8 | 2.46 s | 6.0 |
| `f66399a5` | 15.15 | −2.40 | 18.5 | −3.53 | 4.5 | 4.09 s | 0.5 |

At highway speed the false event has both the largest headway and the largest gap, but the
low-speed row (4.5 m/s, stop-and-go) has a larger headway still, so headway alone does not
separate them either. **Four events cannot validate a threshold** — this is exactly the
situation D-042 was written about.

### Corpus grown to 16 segments — experimental mode ruled out, and a second failure mode

`[CONFIRMED, 16 segments, ~16 min from one drive]`

**Experimental mode is not implicated in the flagged brake, and does not govern radar-lead
use.** The driver asked, and the data answers cleanly:

- At the flagged brake, `experimentalMode` was **False**. That segment is the only one with a
  sustained False block (False from segment start until t=17.31 s; the brake is at
  t=10.6–11.5 s, and the switch to True comes 5.8 s *after* it ends). So the e2e model was not
  driving longitudinal — the chill/ACC path was, which is what makes the lead attribution
  meaningful.
- Across the corpus, `corr(experimental%, radarLead%) = **−0.084**` — no relationship.
  `corr(visionLead%, radarLead%) = **+0.817**`. Two segments are 100% experimental mode with
  100% and 98.4% radar-lead use.

**The brake began on the radar lead.** `longitudinalPlanSource` runs `lead0` from t=10.2 to
10.8 s as the command ramps −0.52 → −2.70, then hands to `cruise`, which carries the peak to
−2.94. StarPilot's own speed controllers were **flat through the entire window** and are not
involved. The onset is therefore attributable to the radar lead; *why `cruise` binds at the
peak is not explained by these fields* and is left open rather than guessed at.

**A second override event is a CORRECT brake — so override is not a fault label.** On
`3053f5a6` at t=12.57 the command reaches **−3.50 m/s²** (harder than the flagged event) with
`lead0`, a real vehicle at 61 m closing at −12.14 m/s, **vision agreeing at prob 1.00 with a
5.2 m gap**, TTC 5.0 s. The driver overrode anyway. Any future classifier must not treat this
as a false positive.

All six hard-brake events across 16 segments:

| segment | accel | dRel | vRel | headway | TTC | gap | vprob | source | override |
|---|---|---|---|---|---|---|---|---|---|
| `3053f5a6` | −3.50 | 61.3 | −12.14 | 3.20 | **5.0** | 5.2 | 1.00 | lead0 | yes — correct |
| `4b66cbf6` | −3.25 | 72.4 | −13.28 | 6.28 | **5.4** | 21.9 | 0.79 | cruise | no |
| `3a4e0842` **(false)** | −2.89 | 62.4 | −4.36 | 3.56 | **14.3** | 14.8 | 0.98 | lead0→cruise | yes |
| `f66399a5` | −2.40 | 18.5 | −3.53 | 4.09 | 5.3 | 0.5 | 1.00 | lead0 | no |
| `f66399a5` | −2.17 | 33.9 | −0.62 | 2.46 | 54.2 | 6.0 | 1.00 | lead0 | no |
| `97566dde` | −2.00 | 40.0 | −2.20 | 2.04 | 18.1 | 10.5 | 1.00 | lead0 | no |

Note `4b66cbf6`: gap **21.9 m**, larger than the false event's 14.8 — and the brake was
correct (TTC 5.4 s, a near-stationary object at 72 m). **Gap magnitude is decisively not the
discriminator.** The false event is the only one combining hard braking with *both* a long
TTC and a long headway, but with one positive example that is an observation, not a rule.

### The second failure mode: good radar tracks rejected on lateral

`[CONFIRMED]` Radar-lead use tracks vision-lead availability closely (r = +0.82) — four
segments with ~0% radar lead simply have **no lead at all** (`visLead%` of 0.0, 0.0, 0.6,
25.0 on empty road). That is benign, and an earlier framing of it here as a concern was
wrong.

**One segment is a genuine outlier.** `ed6257ef` has a confident vision lead **94.2%** of the
time and **1.84 radar tracks per frame**, yet uses a radar lead **0.8%** of the time. Taking
the nearest-in-range track to the vision lead on every such frame:

| | range residual (radar+1.52 − vision x) | lateral residual |
|---|---|---|
| `ed6257ef` | median **−1.1 m** (p25 −1.6, p75 +0.5) | median **4.30 m** |
| `7b76edd7` | median **−1.2 m** (p25 −1.6, p75 −0.6) | median **4.33 m** |

The tracks agree with vision on **range to about a metre** and are rejected on **lateral**,
where `track_matches_vision` uses `y_std_scale=1.0, y_floor=1.0` — roughly a 1 m tolerance.

**This is the same root cause as the false brake, seen from the other side.** The matcher's
tolerances are inverted relative to the sensors' actual error characteristics: range is gated
loosely (`dist_scale=0.25` → ~15–19 m at 60–78 m) and lateral tightly (~1 m), whereas
Bosch-A is *accurate in range* and *poor in azimuth at distance* (4.3 m lateral at ~70 m is
about 3.5°), and monocular vision is the reverse. So a track whose **range** drifted 15 m kept
matching, while tracks whose range is right to a metre are thrown away on **lateral**.

`[INFERRED]` A lateral tolerance expressed as an **angle** rather than a fixed metric
distance, paired with a tighter range gate, would address both directions. **Not implemented
— see D-048's validation gate.** The tension is real and unresolved: tightening range would
also reject `9e21cac9`, which sustains a median 13.8 m gap while behaving correctly.

### CEM mode chattering — unrelated observation, worth not losing

`[CONFIRMED]` `78270494` shows `experimentalMode` flipping at t=25.17 → 25.25 → 25.36 s —
**80 ms and 110 ms dwell times** — and again at 26.05 → 26.62. Nothing in this corpus ties it
to a control fault, but sub-100 ms longitudinal mode switching is worth a look on its own.

### U11 is a real, independent velocity measurement — not a range derivative

`[CONFIRMED, 9,390 measured track samples across 6 segments]` Worth settling, because the
project has been asked whether Bosch-A has a native vRel at all. It does: U11 in the AUX
frame, and it is **not** simply a differentiated range.

- Correlation with a centred range derivative at zero lag: **0.807** — not ~1.0.
- Cross-correlation peaks at **lag −4 samples (−0.28 s), r = 0.847**: U11 **lags** the range
  channel by roughly 0.28 s on average, consistent with the 0.88–1.28 s onset lag recorded
  earlier for deceleration onsets.
- `|U11 − range_rate|`: median 0.78, p90 2.77, p99 7.12, **max 12.22 m/s**.
- **2.26%** of samples disagree by more than 5 m/s, overwhelmingly with U11 **under-reporting
  opening** (e.g. U11 +0.8 while the range opened at +9.1).

A range-derived field would show r ≈ 1.0 at zero lag with no gross disagreements. So U11 and
the range are two genuinely independent channels — which is what makes D-043's cross-check
meaningful. The honest summary is **not** "there is no native vRel" but "there are two
imperfect vRel sources: U11 (independent, railed at ±13.5, lagged) and the range derivative
(prompt, but only as good as the range)."

**And in the false brake, vRel was not the failure.** U11 (−6.0 m/s) and the fitted range
rate (−4.5 to −6.4 m/s) agreed with each other throughout. Both channels consistently
described an object that was genuinely approaching — the object simply **was not the lead
vehicle**. The defect is in **track-to-object association**, not velocity derivation, and no
amount of vRel cross-checking can catch it because both sources come from the same radar
track. See the proposed direction in `DECISIONS.md` D-048.

### Adjacent-lane closer was never a lead candidate — NOT the flagged event

`[CONFIRMED mechanism, significance not established]` Recorded because it was found while
locating the flag, but the driver's delay means the bookmark points at the t≈11 s false brake
above, not here. At the bookmark instant (t=37.69 s, `userBookmark`) track 58 sat at y ≈ −2.6 m closing at −2.6 to
−3.9 m/s while the lead (id6, y ≈ +0.3 m) closed at only −0.5 m/s. At t=37.8 s track 58 was
**nearer than the lead** (42.6 m vs 44.1 m) and closing roughly 8× faster. It was tracked
continuously and promoted to neither `leadOne` nor `leadTwo`.

The mechanism is not a Bosch-A bug: `match_vision_to_track` only promotes a radar track that
matches a **model** lead, and the model reported one lead at y ≈ −0.1 m. A radar-only target
in an adjacent lane therefore cannot become a lead however fast it closes. Whether that is
the right policy for a cut-in is a longitudinal-planner question, not a parser one, and
nothing here shows the car behaved wrongly — `laneChangeState` stayed `off` and the vehicle
never crossed into own-lane (|y| < 1.8 m).

---

## ✅ Routes now reach an agent session, and the recorded corpus is partly reproduced — 2026-09-15

`[CONFIRMED]` Two routes were fetched from Konik **inside an agent session** and analysed:
`00000231--5782493b00` (32 of 32 rlog segments) and `00000232--fc8dad0d18` (27 rlog segments,
32 qlogs). Both were recorded by device `11c8fa231c0499ed` on **`0083ffa`**
(`ns-bosch-radar-testing`, v0.11.2), car `HONDA_CIVIC_BOSCH`. The standing entry "no Konik route
has been fetched or analysed from an agent session" is retired.

### The laptop can run the whole suite — in an arm64 container, without building

`[CONFIRMED]` The checked-in `.so` files are aarch64 **built against Python 3.12**, and this Mac
runs Colima with an **aarch64** Linux VM. So they load as-is: no `scons`, no rebuild, no
architecture trap. The recipe is `ubuntu:24.04` + `python3-venv` + `libzmq5` (the only missing
shared library) + the pip list from the x86_64 recipe above:

```bash
docker run --rm --platform linux/arm64 -e PYTHONDONTWRITEBYTECODE=1 \
  -v "$PWD":/src/openpilot:ro -v oprad-routes:/routes:ro oprad-test:py312 \
  python -m pytest -p no:cacheprovider -o addopts="" -q selfdrive/controls/tests/test_leads.py
```

Four things bite, all of them recorded because each cost time here:

- **Colima shares only `$HOME` with its VM.** A bind mount of anything under `/tmp` or
  `/private/tmp` silently mounts an **empty** directory inside the container — the report reads
  "Not found", not "permission denied". Route data goes in a **docker volume** (`docker cp`), or
  under `$HOME`.
- **Mount the repo read-only** (`:ro`) and pass `PYTHONDONTWRITEBYTECODE=1`, `-p no:cacheprovider`
  and `ruff --no-cache`. A read-write mount let a crashed replay drop a 32 MB `core` file into the
  working tree.
- **`/usr/bin/time` does not exist** in `ubuntu:24.04`; wrapping a run in it exits 127.
- **Do not point pytest at `tools/lib/tests/` as a directory.** It collects upstream openpilot
  tests that fork and download into `/tmp/comma_download_cache…`; name this branch's test files.

### Test results on this tree, in that container

| suite | result |
|---|---|
| AGENTS.md §8 suites + this branch's tool tests | **492 passed, 2 skipped, 1 failed** (510 s) — that one failure is fixed by `b9612b2a`, see below |
| `selfdrive/controls/tests/test_leads.py` | **6 passed** (5.2 s) — including `test_radar_fault` |
| tool tests after this session's changes | **147 passed, 2 skipped** (452 s) |

**`test_leads.py` has now run** — the process-replay suite that every prior writeup listed as
unexercised. `test_radar_fault` drives `replay_process_with_name("card", …)` and passes.

**The one failure was real, and it is now fixed by `b9612b2a`.**
`test_far_lead_brake_limit.py::TestDefaultOff` failed with `UnknownKeyName: FarLeadBrakeLimit`,
because the key existed in `common/params_keys.h` (`ebf4c20`) but **not** in the checked-in
aarch64 `common/params_pyx.so`, last rebuilt at `a972d15` — **16 commits earlier**. That file now
reports **17 passed**.

**Say this precisely: the artifacts were not "stale", they were exactly one key behind.** Before
`b9612b2a`, loading the committed `.so` and calling `all_keys()` returned **821 keys** while the
header declared **822**; the only missing one was `FarLeadBrakeLimit`. `BoschARadar`, `BlotV2` and
`HondaBoschARadar` were all present and working throughout.

⚠️ **Do not diagnose these artifacts with `strings`** — that is how this was first (wrongly) argued
here. The `.so` is linked with tail-merged string literals, so a key that is the suffix of a longer
key has no standalone copy: `strings` "proves" `BoschARadar` missing because it only appears inside
`HondaBoschARadar`. Load the `.so` and call `all_keys()`:

```bash
docker run --rm --platform linux/arm64 -v "$PWD":/openpilot -e PYTHONPATH=/:/openpilot \
  oprad-test:py312 python -c \
  "from openpilot.common.params_pyx import Params; print(len(Params(memory=True).all_keys()))"
``` What this cost, before the rebuild:

1. The repo ships a `prebuilt` marker and `launch_chffrplus.sh` only builds when it is absent, so
   **the device ran that one-key-behind artifact** rather than compiling its own.
2. `Params().put_bool("FarLeadBrakeLimit", True)` — the documented way to enable the TEST feature —
   **raised `UnknownKeyName` on the device**, and the Galaxy toggle's write path
   (`the_galaxy.py:461`, a bare `params.put`) has no handler for it, so the row failed to save.
3. Nothing crashed: `longitudinal_planner.far_lead_brake_limit_enabled()` wraps the read in
   `try/except → False` and re-reads every 100 frames, so the exception was caught ~every 5 s and
   the feature was simply inert.

**Resolved in `b9612b2a`** (artifacts only, AGENTS.md §10, pattern `a972d15`). Rebuilt with the
toolchain that produced `a972d15` — Ubuntu 24.04, clang 18.1.3, Python 3.12.3, **Cython pinned to
3.1.4** — and `SP_FORCE_TICI=1` for `arch=larch64`. `params_pyx.cpp` regenerated byte-identical.
Two builds mounted at `/src` and `/work` produced byte-identical binaries. Loading the old and new
`.so` and diffing `all_keys()` shows **821 → 822 keys, the sole difference being
`FarLeadBrakeLimit`, nothing dropped**; `put`/`get`/`remove` round-trip on it.

⚠️ **`scons` will lie to you here.** A rebuild after `git restore` printed
``scons: `common/libcommon.a' is up to date`` and relinked the `.so` against the **old 821-key
archive** — deleting the `.o` files is not enough, because scons trusts `.sconsign.dblite`. Delete
the signature DB and the archive itself (or use `./tools/clean_build_artifacts.sh --sconsign`), then
**check the hash**, or you ship a `.so` that looks freshly built and is not.

This is static verification only — no replay, no road evidence. It makes the toggle writable; it
does not validate the feature, which stays TEST and default OFF.

### Upstream PR state, re-checked

`JamesL787/openpilot` **PR #9 is MERGED** (2026-09-05) and its merge commit `10e4705` is an
ancestor of this branch, so the §8 run above *is* the re-run those records asked for.
`firestar5683/StarPilot` **PR #124 is CLOSED, unmerged** — it was not re-run.

### The recorded corpus: what reproduced, what did not

`[CONFIRMED]` `tools/bosch_a_corpus_report.py` ran on all 32 segments in 38 s. The recorded
hash-labelled segments are identified beyond reasonable doubt, and **the 6-segment corpus is
segments 10–15 and the 16-segment corpus is segments 10–25** of `00000231--5782493b00`:

| recorded label | segment | matched on |
|---|---|---|
| `3a4e0842` (the false brake) | **10** | −2.89 m/s² at dRel 62.4, vRel −4.36, headway 3.57, TTC 14.3; `experimentalMode` False until t=17.3 s (71.1%) |
| `f66399a5` | **11** | both its brakes: −2.40 at 18.5 m and −2.17 at 33.9 m |
| `97566dde` | **13** | −2.00 at 40.0 m, headway 2.04, TTC 18.1 |
| `9e21cac9` | **14** | gap p50 +12.33, max +26.76 (recorded 13.8 / 27.7 before the 1.52 m offset) |
| `4b66cbf6` | **17** | −3.25 at 71.7 m, vRel −13.28, TTC 5.40, `cruise` |
| `3053f5a6` | **19** | −3.50 at 61.3 m, vRel −12.14, TTC 5.05, vprob 1.00, override |
| `7b76edd7` | **24** | lateral residual p50 +4.36, range −1.22 (p25 −1.62, p75 −0.80) |
| `ed6257ef` | **25** | vision lead 94.2% at prob ≥ 0.5, 1.84 tracks/frame, radar lead 0.8%, lateral +4.40 |
| `78270494` | **26** | `experimentalMode` dwells of 78 ms and 112 ms at t=25.25 and 25.37 |

`1be0aa43` and `bf574f16` are segments 12 and 15 in some order; nothing recorded separates them.

**Reproduced** (see D-051 for the two definition differences):

- Pooled radar/vision gap, segments 10–15: **n=4,872** against a recorded 4,868, every percentile
  within 0.9–1.7 m of the recorded value — the `RADAR_TO_CAMERA` offset.
- `corr(visionLead%@0.5, radarLead%)` over segments 10–25: **+0.818** against a recorded +0.817.
- The four "≈0% radar lead" segments: 15, 16, 20, 24 at vision occupancy **0.0 / 0.0 / 0.6 / 25.0**
  — the recorded list exactly.
- The two "100% experimental mode" segments with ~100% radar-lead use: 11 and 12.
- Every hard-brake row's accel, dRel, vRel, headway, TTC and vision probability.
- `tools/bosch_a_route_report.py` on segment 10 reproduces the first-route census **exactly**:
  144,380 Bosch-A frames, 1,786 sweep triggers, 2,878 points over 892 sweeps, 33 track ids,
  dRel 10.9–111.2 m, vRel −13.50…+4.52, liveTracks 893, radarState 1200.

**NOT reproduced, and unresolved in both directions:**

- **U11 versus the range derivative.** Harness (measured points only, runs split at 0.1 s):
  n=10,444, r=0.882 at zero lag, peak at **lag 0**, **9.16%** of samples disagreeing by >5 m/s,
  max 55.05 m/s. Recorded: n=9,390, r=0.807, peak at **lag −4** (r=0.847), **2.26%**, max 12.22.
  The recorded numbers came from code that no longer exists; the harness's own filtering is
  documented but unvalidated. **Do not cite either as settled.**
- **`|d(gap)/dt|`**: harness p50 6.0 / p90 30.5 / p95 43.3 / max 214.6 against recorded
  4.7 / 22.3 / 31.1 / 130.2. Both refute the gap-rate gate; they disagree on how badly.
- **`corr(experimental%, radarLead%)`** over 10–25: **−0.197** against a recorded −0.084. Sign
  agrees, magnitude does not; both are far from explaining anything.

**Two corrections to the recorded hard-brake table**, from the same run:

- The flagged false brake **was** overridden by the driver, and `3053f5a6` too. The earlier harness
  read the override flag at the peak frame only, where the gas press lands ~0.3 s later.
- `3053f5a6` contains **two** runs over −1.5 m/s² (−3.50 at t=12.56, then −3.09 at t=13.41), not
  one. The recorded table lists one.

Whole-drive, three hard brakes fall outside the recorded 16-segment window, including
**segment 7 at −3.50 m/s² with vRel at the −13.50 rail and vision prob 0.58**, driver override.

### `experimentalMode` chattering is real, and not confined to one segment

`[CONFIRMED]` The recorded `78270494` observation reproduces (78 ms, 112 ms) and is not unique.
`00000231` segment 9 switches **10 times**, four of them with dwells of **151, 89, 59 and 54 ms**
within 0.2 s at t≈46.2–46.4. On `00000232`, segment 12 shows 38–196 ms dwells and segment 22 shows
115 ms and 138 ms. Still nothing ties it to a control fault; it is now measured rather than
anecdotal.

### D-044's shadow channel, measured on both drives

`[CONFIRMED, device-side telemetry]` `tools/bosch_a_vrel_shadow_report.py` compares
`radarState.leadOne.vRel` (U11) with `vRelRangeDerived` on the selected radar lead:

| route | frames | r (measured) | `|diff|`>2 m/s | rail samples | rail understatement p50 / max |
|---|---|---|---|---|---|
| `00000231` | 14,446 finite (203 NaN) | **+0.877** | 13.2% | 150 | **+2.19** / +7.37 m/s |
| `00000232` | 15,091 finite (170 NaN) | **+0.854** | 8.9% | 60 | **+2.12** / +9.95 m/s |

So on the rail the range channel says the object is closing **~2 m/s faster** than the published
bound, with a worst case near 10 m/s — the direction D-041 predicted, now quantified on real
drives. **Onset lag is not established**: requiring a crossing to hold 5 samples and pairing it
within 3 s leaves **2 events** (0.19 s and 1.16 s, U11 late) on `00000232` and none on
`00000231`. D-044's recorded 0.88–1.28 s claim is neither confirmed nor refuted by n=2. Note also
that `radarState` repeats at 20 Hz over a 14.35 Hz radar, so these n are inflated by duplicates.

### D-049 gate item 1, answered on one drive and refuted on the other

`[CONFIRMED]` `tools/bosch_a_lifecycle_report.py` replays CAN through the real `RadarInterface`
and counts what the parser itself decided. On **`00000231`: zero lifecycle breaks** in 27,899
sweeps (41,238 continuations, 740 fresh identities, 0 parser errors). An independent check of
segment 10 agrees: 0 range-history clears on a same track object, and the recorded "track 58
reuse" is a **retirement then rebirth** (new objects at t=5.56 s and t=33.34 s), not a lifecycle
break. So the one-message reset D-049 characterises never fired on that drive — a latent hazard,
not a live one.

`00000232` is the opposite, and it exposed a parser defect rather than a reset: see the section
below.

**Seamless reuse (gate item 2) remains open.** Of 18 lateral steps >1.5 m with no break on
`00000231`, 14 are consecutive-sweep runs — a smooth sweep of a real target across the car frame
(track 32 walked y +10.3 → −1.4 m over seven sweeps at a steady ~51 m), not an identity swap.
Four isolated steps on `00000231` and three on `00000232` remain as candidates; none is confirmed.

---

## 🔴 The lifecycle counter saturates after ~137 s and the parser deleted the lead — fixed 2026-09-15

`[CONFIRMED on two drives, offline replay + the device's own recording]` **This is the most
consequential finding in this session, and it explains symptoms the driver reported in chat.**

`LIFECYCLE_RAW` is a 12-bit counter that advances by 2 per frame index, so it reaches its highest
even value **0xFFE = 4094 after 2,047 frames — about 137 s of continuous tracking** at the ~14.9 Hz
sweep rate — and then stops advancing. It is *not* the invalid sentinel (0xFFF): STATUS, range,
azimuth and track id all stay valid and the object is still physically there.

The parser's continuity rule `life_delta == 2 × frame_delta` therefore fails on **every** sweep once
the counter pins. Each failure cleared the range history and popped the point, so no point ever
matured again: **the object was observed every sweep and published on none of them**, for as long as
it stayed visible.

| route | track | age at saturation | suppressed for | published during |
|---|---|---|---|---|
| `00000232--fc8dad0d18` | id 37 | **137.4 s** | **121.8 s** (1,815 sweeps) | **0** |
| `00000232--fc8dad0d18` | id 34 | 137.5 s | 12.3 s (184 sweeps) | 0 |
| `0000020a--1fd2b58eda` | id 38 | — | segments 5→8, ~3.7 min | 0 after t=5:05 |
| `0000020a--1fd2b58eda` | id 51 | — | segments 11→12, ~95 s | 0 |

Track 37's last published geometry before it vanished was **dRel 38.9 m, yRel −0.1 m** — the lead we
were following.

**The device's own recording corroborates it independently of any replay.** On `00000232`, recorded
`liveTracks` carry id 37 in 894/894 frames of segment 8 and 569/893 of segment 9, then **0** in
segments 10, 11 and 12, while the radar lead goes from 1200/1200 frames to **0/1200**. On
`0000020a`, same CAN, device (pre-fix `0083ffa`) versus this tree's parser:

| segment | device published id 38 | with the fix |
|---|---|---|
| 5 | 76/894 (8.5%) | 890/894 (99.6%) |
| 6 | **0/893 (0.0%)** | 831/893 (93.1%) |
| 7 | **0/894 (0.0%)** | 894/894 (100.0%) |
| 8 | **0/893 (0.0%)** | 735/893 (82.3%) |

**The fix** (`BOSCH_A_LIFE_SATURATED`, in `radar_interface.py` with its evidence block): a
saturated→saturated step is treated as a continuation, because a counter pinned at its maximum
cannot testify to identity either way. Per D-041/D-042 the safe direction is to keep publishing
geometry — the range-innovation gate still judges every sweep and staleness still retires a genuine
disappearance. **Cost, recorded against D-049: an ID reuse that happens *while* the counter is
saturated cannot be detected at all.**

Verified after the fix: id 37 publishes **1,788/1,815 sweeps (98.5%)** across the same window, id 34
180/184 (97.8%). The five-route census now reports **0 chronic runs everywhere** (`0000020a` 2 single
one-sweep breaks, `0000020b` 2, `00000213` 1, `00000231` 0, `00000232` 2 — all of them the ordinary
D-049 one-sweep case). Negative control per D-009: a counter
frozen at any *non*-saturated value is still a lifecycle break, pinned by
`test_a_stuck_but_unsaturated_counter_is_still_a_lifecycle_break`.

`[NOT VERIFIED]` Offline replay only. Nothing here shows how the car behaves with the lead restored.

---

## The five flagged drives — what each event actually was, 2026-09-15

`[CONFIRMED, device-side telemetry + offline replay]` Three further routes were fetched and analysed
against timestamps the driver flagged in chat: `0000020a--1fd2b58eda` (17 rlog segments),
`0000020b--60b34177d9` (38) and `00000213--0b61770c26` (23). The driver's own `userBookmark` presses
land 5–25 s *after* each quoted time, confirming the chat timestamps are the events themselves.

| flagged | what the data shows |
|---|---|
| **5:05** semi-hard stop | **Correct brake** — lead genuinely decelerating (vLeadK 18.7 → 5.7, aLeadK −5.7), vision agreeing at prob 1.00, command −3.45. But the radar lead **vanished mid-manoeuvre at t=5:05.17** — the saturation defect above — and the lead handed to vision for the next ~3.7 min. |
| **7:43** stop then creep | Stop at only **−1.2 m/s²** on a **vision** lead (radar still suppressed), ending with the driver on the brake. During the creep the radar briefly offered tracks *further away* than vision (id 26 at 16.8 m vs vision 14.0; id 24 at 7.4 vs 6.6) with negative `vLeadK`. |
| **9:53** "random brake" | **Not a range-drift false brake.** At full rate the radar range moves smoothly (50.8 → 45.9 m over 2.3 s), per-sweep innovation 0.00–0.32 m, `measured` throughout; the apparent radar/vision divergence is **vision x jitter of ±3 m** sweep-to-sweep. The lead really did decelerate mildly (vLeadK 23.5 → 21.4) and the planner commanded **−1.67 m/s² for a lead 48 m away closing at <1 m/s** (TTC ≈ 56 s). That is braking authority, not a sensor fault. **`FarLeadBrakeLimit` would not have caught it**: its regime needs headway ≥ 3.0 s and this was **2.07 s**. |
| **6:06** | Braking −1.1…−1.41 with **no lead at all** (vision prob 0.15–0.50 at ~130 m), then −2.60 against radar id 16 at **94.4 m, y +2.3 m, vLeadK 2.3** — a near-stationary, off-lane object at 94 m, vision prob 0.52. |
| **12:18**, **28:44** | Ordinary lead-deceleration braking on an in-lane radar lead; −1.6 and −1.9 peak. Nothing anomalous. |
| **29:52** | **Not openpilot.** From t=47.1 the car is **disengaged with the brake pedal pressed** (`en0`, `brk1`, commanded accel 0.00) while the driver slows 22.0 → 10.4 m/s. |
| **15:12** curve braking | Lead id 56 at 21–28 m through a curve (steering to +12°, curvature −0.0045). `aLeadK` swings **−1.7 to −3.0 while `vRel` is positive and the range is opening**; commands −1.4…−1.73. The lead filter is manufacturing deceleration out of curve geometry. |
| **16:04** cut-out | The real event is at **t=16:15**. Radar id 44 **keeps its track id** while its motion steps discontinuously in one 0.28 s sweep: dRel 61.6 → 59.3 but `vRel` **+0.45 → −3.62**, `aLeadK` −0.74 → −3.39 → −5.11. Command ramps to −3.45, measured **aEgo −4.81 m/s²**, driver overrides with the gas. The parser recorded **no lifecycle break** on that route at that time. |

**On "FCW".** There are **no FCW or collision `onroadEvents` anywhere** in the sampled segments — the
instrument works (311 event messages, 10 distinct names: `cruiseMismatch`, `gasPressedOverride`,
`pedalPressed`, `laneChange`, …). The `radarState.leadOne.fcw` bit is set on **70.4%** of frames on
`0000020b` and **100%** on `0000020a` segment 9, so that bit is not an alert indicator. The four
"FCW" timestamps are brake events, listed above.

`[OPEN]` **16:04 is the best seamless-reuse candidate yet found** (D-049 gate item 2): an identity
that keeps its id while its motion changes discontinuously, with no lifecycle break for the parser
to see. It is equally consistent with the radar genuinely re-measuring onto the revealed slower car.
Distinguishing the two needs the raw per-slot data at that instant, which has not been done.

---

## 🟠 The far-lead limit was inert on the fault it exists for — fixed and re-replayed 2026-09-16

`[CONFIRMED in replay, 39 segments]` `FarLeadBrakeLimit` as shipped in `ebf4c20` did essentially
nothing on the **t≈11 s false brake** (seg 10 of `00000231--5782493b00`) it was written for.
Replayed at its own toggle: **2 frames fired, 6 altered, peak `aTarget` −2.9075 → −2.9012 m/s²**.
It clipped 0.25 s of tail *after* the peak had already been commanded. The earlier "8 frames
altered, max reduction 0.89 m/s²" figure recorded above was measured the same way and is not in
conflict with this — what was missing was the observation that **none of it lands on the event**.

**Root cause.** The call site stood down entirely whenever `get_close_lead_brake_cap` had returned
a cap. Instrumenting the planner shows frames 203–224 — the fault window — **never reached the
function**, because that cap was non-empty across exactly those frames. `get_close_lead_brake_cap`
has **no distance gate**, fires out to ~68 m, and on this segment it is itself the mechanism
issuing the false brake. The feature deferred to the thing it was meant to bound.

**Why removing the stand-down alone is not enough.** Dropping the deference fixes seg 10 — peak
−2.901 → −1.950 m/s², 21 frames — but produces a **false positive on a correct stop-and-go**:
`0000022e--2c6875ff90` seg 8, t 37.6–41.9 s, 88 altered frames, up to 0.631 m/s² of relief on a
brake where radar (27.1 m) and vision (29.9 m) **agree**, `modelProb` 0.99–1.00, `aLeadK`
−0.45 → −2.47, ego 13.4 → 2.7 m/s, ending in `brakePressed` at 41.94 and a disengage at 42.24.
That is a real deceleration and the limit must not touch it.

The headway gate does not separate the two, because **`dRel / v_ego` rises as the car slows**: an
ordinary brake satisfies a 3.0 s headway on its way down. What separates them is absolute distance
— the fault sits at 61–64 m, the correct brake at 27 m.

**The change: drop the blanket stand-down, add `FAR_LEAD_BRAKE_LIMIT_MIN_DIST = 40.0` m.** Measured
across the variants tried:

| variant | seg 10 peak | seg 10 fires | 15 corpus controls | 23 road segments |
|---|---|---|---|---|
| as shipped (`ebf4c20`) | −2.901 | 2 | 0 | 0 |
| drop the stand-down only | −1.950 | 21 | 0 | **6 fires — false positive** |
| `MIN_TTC` 10.0 → 8.0 | −1.950 | 21 | 0 | bit-identical to the row above |
| fix the ramp anchor (below) | −2.818 | 18 | 0 | — |
| **shipped: stand-down dropped + `dRel ≥ 40 m`** | **−1.950** | **21** | **0** | **0** |

**Acceptance evidence.** Frame-by-frame diff of replayed `longitudinalPlan.aTarget`, toggle OFF vs
ON, against the patched tree through the **stock** `plannerd` process config, over **39 segments** —
`00000231--5782493b00` segs 10–25, all 10 of `0000022e--2c6875ff90`, all 13 of
`0000022f--8acb9d34ac`:

```
segments compared: 39
  CHANGED  231 seg10: 21 altered frames, maxD 1.4807 m/s2, 21 fires
segments with any altered frame: 1
```

Exactly one segment in the set changes, and it is the fault. `test_far_lead_brake_limit.py`:
**21 passed**.

**Margin on the 40 m threshold — it is not wide everywhere.** Counting frames that pass every gate
*except* distance: on `0000022e--2c6875ff90` 3,351 pass and **none** sit in the 30–40 m band; on
`00000231--5782493b00` 8,010 pass and 294 sit in that band; on `0000022f--8acb9d34ac` 1,736 pass
and **376** sit in it. The threshold is doing real work on `0000022f`, and a route carrying a
genuine far-lead range-drift fault at 35 m would land on the wrong side of it.

**This is replay evidence only, on one fault. There is no road evidence for the amended logic.**
The feature stays TEST and default OFF. Its positive evidence base is still **n=1**; what the
39-segment diff buys is a bound on the false-positive side, not a second fault.

**More routes would move this, and these specific shapes are what is missing:** highway cut-ins and
merges settling at 40–70 m (the regime the limit now owns, with no example of a *correct* hard brake
in it); downhill or trailing-throttle following at 45–60 m; and any second instance of long-range
radar range walk, which is the only thing that can take the positive evidence past n=1.

### KNOWN DEFECT, pinned and deliberately not fixed: the ramp anchor

`[CONFIRMED]` `get_far_lead_brake_limit` computes
`floor = accel_min + ramp * (FAR_LEAD_BRAKE_LIMIT_ACCEL - accel_min)`. The unit tests pass
`ACCEL_MIN` (−3.5) as `accel_min`, for which ramp 0 means "no restriction" and ramp 1 means −2.0 —
correct. **The call site passes `output_accel_min`**, which is −0.5 on most of the fault frames, so
intermediate ramp values land between −0.5 and −2.0: the limit **clamps hardest when it is least
confident**. Observed floors on the fault: −1.20 to −1.37.

This is worse than a sign error, because the confidence signal is self-defeating: `ttc = dRel /
-vRel` is derived from the **same corrupted `vRel`** that causes the fault, so TTC collapses
1153 → 10.14 s and the ramp collapses 1.00 → 0.04 exactly when braking is hardest.

Fixing the anchor makes the feature **less** effective on the fault (−2.818 vs −1.950), because the
inversion is accidentally compensating for the collapsing ramp. It is left in place, pinned by
`TestKnownRampAnchorDefect` in `test_far_lead_brake_limit.py` (marked *PINNED, NOT ENDORSED*), so
the behaviour cannot drift silently. **Do not "tidy" this anchor without re-running the 39-segment
diff** — the number it produces today is load-bearing and is not the number the docstring implies.

### `process_replay` could not build `CarParams` at all

`[CONFIRMED]` Separately, and the reason none of the above could be measured at first:
`selfdrive/test/process_replay/process_replay.py:357` called `get_car()` with the pre-StarPilot
signature and died with `TypeError: get_car() missing 1 required positional argument: 'params'` on
any replay that fingerprints. `params` is now passed positionally and
`starpilot_toggles=get_starpilot_toggles()` explicitly — that argument's `None` default is a lie,
`opendbc_repo/opendbc/car/car_helpers.py:321` reads `starpilot_toggles.force_fingerprint`
unconditionally. The import is done inside the function on purpose: `get_starpilot_toggles()` has a
`SubMaster` as a default argument and would open a socket at import time in every process that
merely imports this module. Verified by a stock-config replay that now fingerprints for real:
`HONDA_CIVIC_BOSCH`, source 1, fuzzy True, cached True, fw_count 22.

⚠️ **The same break still exists at `opendbc_repo/opendbc/car/panda_runner.py:17`**, which calls
`get_car(self._can_recv, self.p.can_send_many, self.p.set_obd, True, False)`. It is in a vendored
subtree and is a standalone runner, so it was left alone deliberately rather than fixed in passing.
Anyone using that runner will hit the same `TypeError`.

---

## 🟡 The shadow `vRelRange` channel is now wired into control behind a toggle — 2026-09-16

**TEST feature, default OFF, param `RangeDerivedVrel`, Bosch-A only. Never run on a car.**
Decision and rejected alternatives: **D-053**. Constants and their evidence: the
`RANGE_VREL_ASSIST_*` block in `selfdrive/controls/radard.py`.

D-044 published `vRelRange` — a 5-sample LSQ of d(dRel)/dt — as telemetry and said nothing consumed
it. That is no longer true. With the toggle on, for the lead track only, the range rate may push
the published closing speed **more closing, never less**.

### What it does, precisely

| | |
|---|---|
| Scope | Bosch-A cars only; `RadarD.prev_lead_track_ids` only (leadOne/leadTwo) |
| Direction | one-sided — `disagreement = vRel - vRelRange`, acted on only when **positive** |
| Arming | 2.0 m/s sustained across 5 consecutive **measured** updates (0.35 s) |
| Cap | 8.0 m/s of extra closing |
| Geometry | dRel ≥ 8.0 m and \|yRel\| ≤ 1.5 m → azimuth ≤ 10.8°, cos 0.982 |
| Touches | `radarState.leadOne/leadTwo` `vRel` and `vLead`, and the lead KF (so `aLeadK`) |
| Does **not** touch | `Track.vRel` / `Track.vLead` — so `track_matches_vision`, `vision_track_probability` and the adjacent-lane detectors are unchanged |
| Disarms | on a coast, a range dropout, leaving the geometry gate, the flag going off, or the disagreement reaching zero |
| Holds | on a **duplicate** radard cycle — no new `liveTracks`, so nothing to re-decide. Not the same thing as a coast; see below |

Because it never edits the track's own velocity, this changes what is reported **about** the chosen
lead and never **which** track is chosen. That separation is deliberate — see D-053 rejected
alternative 2.

### Why it exists

* **The rail.** D-041, `000001f9` at 29:52: U11 pinned at −13.5 m/s on 88/88 active frames with
  healthy u10 while the range closed at −19.4 m/s. D-041 published the rail as a bound and left
  recovering the true value "to a separate, validated change". This is that change, unvalidated.
* **The lag.** D-043/D-044, `000001fe/fb/fd`: U11 is **0.88–1.28 s late** to a closing onset where
  a 4-sample range LSQ lands within **0.07–0.14 s**. `000001f3` at 19:27: fitted −6.7 vs U11 −2.08.

### 🔴 The hazard — read this before touching a constant

**This is a velocity check that reads the range channel, so a range error is invisible to it.** The
t≈11 s false brake above is a range error: track 6 walked 71.8 → 61.6 m while vision held 75–78 m
at prob 0.93–1.00 and the real gap grew. U11 (−6.02) and the range fit (−4.5 to −6.4) **agreed**.
This file already records that the shadow channel is blind to that fault; the assist inherits the
blindness exactly.

`[INFERRED from the recorded figures — driven through the real Track object in a unit test,
NOT replayed]` on those numbers the worst disagreement is **+0.38 m/s against the 2.0 m/s
threshold**, so the assist stays inert there. The margin is **1.62 m/s**. That is the only thing between this
feature and amplifying the one false brake this branch has recorded.
`TestTheRangeWalkFault` pins it. **Do not "fix" that test** — if it starts failing, the feature has
become the amplifier, not the test.

### What was verified, and by what method

* `[CONFIRMED — static unit test]` `selfdrive/controls/tests/test_range_vrel_assist.py`, **47
  passed**: default-OFF, one-sidedness across the whole disagreement axis, the arming count,
  single- and double-gross-outlier rejection, the correction cap, both geometry gates, coast and
  dropout disarm, continuous decay as U11 catches up, the KF path, and the `000001f9` rail and
  `000001f3` onset cases recovering toward the range rate.
* `[CONFIRMED — static unit test]` `TestRadardLoopCadence` drives the **real** radard interleave —
  a 20 Hz loop over the 14.35 Hz radar, so ~1 cycle in 4 carries no new `liveTracks` message — and
  pins that the assist arms there, that a duplicate cycle changes nothing, and that a coast still
  clears. See the subsection below; this is where the first implementation was wrong.
* `[CONFIRMED — static]` D-009 negative controls: `TestNegativeControlOfTheTestsThemselves` breaks
  the arm count, the disagreement threshold, the lateral gate and the duplicate-cycle hold in turn
  and asserts each guarded property actually fails.
* `[CONFIRMED — static]` No regression in the §8 suites: honda **240 passed**; radard / lead
  behaviour / lead follow policy / following distance / turn lead / far-lead brake limit
  **152 passed**; `test_leads.py` (process replay, including `test_radar_fault`) **6 passed**.
  `ruff check` clean on `radard.py` and the new test file.
* `[CONFIRMED — static]` The x86_64 `common/params_pyx.so` was rebuilt in this container and
  `Params().get_bool("RangeDerivedVrel")` returns `False` by default.

### A duplicate radard cycle is not a coast — the defect this nearly shipped with

`[CONFIRMED — static]` `RadarD` runs at the 20 Hz model rate over a 14.35 Hz radar and collapses
two conditions into one bit (`measured = pt.measured and radar_fresh`), so `Track.update` sees
`measurement_update = False` both when **no new message arrived** (a duplicate, ~1 cycle in 4,
`t_now` unchanged, point data unchanged, lead KF not stepped) and when a **coast** arrived (new
message, `t_now` advanced, parser's measured bit clear).

The first implementation cleared on both. That resets the arm count roughly every fourth cycle,
so five consecutive qualifying fits are unreachable and **the feature would have been permanently
inert on the car** — while all 41 unit tests passed, because every one of them fed exactly one
update per radar sweep. It was caught by re-reading the call site before committing, not by the
tests. The two are now separated by whether `t_now` advanced, which needs no new constant.

The general lesson is worth more than the fix: **a Bosch-A unit test that feeds one
`Track.update` per sweep is not testing what radard does.** Anything downstream of
`measurement_update` needs `TestRadardLoopCadence`'s interleave, or it is testing a cadence the
car never runs at.

### What is NOT verified

* **No replay and no road evidence for the feature ON.** The `test_leads.py` process replay
  above ran with the param at its default, so it shows the **OFF** path is unchanged and nothing
  more. Unlike `FarLeadBrakeLimit`, there is no OFF-vs-ON segment diff for this. The next agent
  should run one before anything else — the 39-segment harness used for the far-lead limit is the
  right tool, and the expected result is **zero altered frames on all 39**, because no segment in
  the corpus contains a sustained 2 m/s U11-vs-range disagreement on a lead track.
* The `000001f9` rail and `000001f3` onset cases are driven here as **synthetic constant-rate range
  series with the recorded velocities**, not as log replays. They show the arithmetic works; they
  do not show the feature would have helped on those drives.
* The 8.0 m/s cap is sized from **n = 2** recorded events (5.9 and 4.6 m/s). It is a bound on
  damage, not a fitted value.

### 🔴 CONFIRMED ON THE CAR: the toggle renders but cannot be switched on — 2026-09-16

**Observed on a real device** (Galaxy at `/device_settings`): the row renders in the right place,
directly under Far-Lead Brake Limit, and toggling it returns a red banner —

> Parameter 'RangeDerivedVrel' is not editable.

**This is open item 12 firing, not a UI bug.** The row placement and gating are correct; the key
does not exist in the registry the device is running.

The chain, end to end:

1. `common/params_keys.h` declares `RangeDerivedVrel` — but that header is **compiled into**
   `common/params_pyx.so`, and the checked-in `.so` is the aarch64 one built by `b9612b2a`,
   which predates this key.
2. `the_galaxy.py:601` `_build_default_params()` enumerates **`_params_raw.all_keys()`** — the
   `.so` registry. The key is not in it.
3. `_get_param_type_info()` builds `allowed_keys` from that list, so the key is absent.
4. `PUT /api/params` hits `if key not in allowed_keys` and returns **403** with the banner above,
   before any `put` is attempted.

**Verified [CONFIRMED, static]** against the committed blob, not the working tree — the working
copy is this container's x86-64 scons rebuild and is `skip-worktree`, so it proves nothing about
the device:

```bash
git show HEAD:common/params_pyx.so > /tmp/committed.so
file /tmp/committed.so                                  # ELF aarch64
strings -a /tmp/committed.so | grep -x FarLeadBrakeLimit   # PRESENT -> its toggle works
strings -a /tmp/committed.so | grep -x RangeDerivedVrel    # ABSENT  -> 403
```

`strings` is trustworthy **for these two keys specifically**: the tail-merging caveat in open
item 4 only bites keys that are a suffix of a longer key, and no declared key ends with either of
these. It does still lie about `BoschARadar` (a suffix of `HondaBoschARadar`).

**Do not work around this in Galaxy.** Adding the key to an allowlist by hand would let the PUT
through to `params.put`, which raises `UnknownKeyName` on the same missing registry entry; and
even if it stored, `_range_vrel_assist_enabled()` reads through the same `.so` and returns
`False`. The result would be a toggle that looks like it saved and does nothing — strictly worse
than the honest 403. The 403 is the gate working correctly.

**The fix is the larch64 rebuild, and it is the only fix.** It is a **deliberate, separate
commit** (AGENTS.md §10); follow the recipe and the scons-lies warning in open item 4 verbatim,
and expect the key count to go 822 → **823**.

**It cannot be done in this container** — checked 2026-09-16: no docker daemon, no `qemu-user`,
no aarch64 cross-compiler, and the local toolchain is Python 3.11.15 / Cython 3.3.0 against the
pinned Python 3.12.3 / **Cython 3.1.4** that `b9612b2a` reproduced byte-identically. It needs a
machine that can run `--platform linux/arm64`.

**A native on-device build unblocks the toggle but does NOT close open item 12.**
`[CONFIRMED — run on the car, 2026-09-16]` Building on the comma itself produces a working
larch64 `params_pyx.so` with the key present, which is enough to make the Galaxy row editable and
`_range_vrel_assist_enabled()` return the stored value. It is **not** the reproducible
pinned-toolchain artifact, so it must never be staged or pushed, and the committed `.so` stays at
822 keys until item 12's docker recipe is run properly.

Two traps found doing this, both worth knowing before anyone repeats it:

- **`scons common/` fails on the device**, at `common/transformations/coordinates.cc`:
  `fatal error: 'eigen3/Eigen/Dense' file not found`. AGNOS ships no eigen headers. This is a
  sibling SConscript (`common/SConscript:31`) and is unrelated to the params binary, but with
  `-j4` scons aborts **before linking `common/params_pyx.so`**, so the build looks like it ran
  and the key is still missing. Build the single target instead:
  `scons -j4 common/params_pyx.so`. Do not install eigen or touch `common/transformations/` —
  its committed aarch64 `.so` is correct and unchanged by this branch.
- **Do not remove the `prebuilt` marker** to force a rebuild. `launch_chffrplus.sh:153` runs
  `./build.py` when it is absent, and **this branch has no `build.py` at the repo root**, so
  removing it breaks the launch.

Verify the result with `all_keys()`, never `strings` (open item 4's tail-merging caveat).
`PYTHONPATH` must be the **parent** of the openpilot dir, i.e. `/data` on the device:

```bash
PYTHONPATH=/data python3 -c "from openpilot.common.params import Params; \
k=Params(memory=True).all_keys(); print(len(k), b'RangeDerivedVrel' in k)"   # expect: 823 True
```

`RangeDerivedVrel` alone is not sufficient to make the assist act. `is_bosch_a_radar_car(CP)`
(`selfdrive/controls/radard.py:107`) requires `CP.radarUnavailable == False`, which
`opendbc_repo/opendbc/car/honda/interface.py:56-59` sets from the **`BoschARadar`** param when
CarParams is built. So `BoschARadar` must be on too, and it only takes effect after an
offroad→onroad transition; `RangeDerivedVrel` is re-read every 100 radard frames and can be
flipped onroad.

An openpilot update that resets the working tree drops the device back to the committed 822-key
binary, at which point the key silently reads `False` while the UI still shows it on. Re-run the
`all_keys()` check before trusting the toggle after any update.

### The Galaxy row: same location as the far-lead brake limit — 2026-09-16

Both surfaces now carry the row, and **both gate it the same way the far-lead brake limit is
gated**. The two rows are deliberately siblings: same section, same parent, adjacent, same gate.

| Surface | File | State |
|---|---|---|
| On-device (raylib) | `selfdrive/ui/layouts/settings/starpilot/longitudinal.py` | In `_bosch_a_radar_rows`, immediately after `FarLeadBrakeLimit`; inherits that section's advanced + Bosch-A gating |
| Layout metadata | `starpilot/common/assets/device_settings_layout.json` | Immediately after `FarLeadBrakeLimit`; `parent_key: AdvancedLongitudinalTune`, `settings_tier: advanced`, `requires_offroad: true` |
| Galaxy (web) | `starpilot/system/the_galaxy/assets/components/tools/device_settings.js` | `BOSCH_A_REQUIRED_KEYS`, hidden unless `BoschARadarAvailable` |

**The Galaxy gate was the one real gap.** The layout entry and the raylib row were already in
place, but Galaxy's `isSettingVisible` carried a hand-written `param.key === "FarLeadBrakeLimit"`
check and nothing for `RangeDerivedVrel`, so on the web UI the new row would have rendered on
**every** car — including cars with no Bosch-A radar, where the feature cannot do anything. That
single-key check is now a two-key set, `BOSCH_A_REQUIRED_KEYS`, so adding a third Bosch-A TEST row
means adding a string rather than another branch.

**Correction — Galaxy's write path is NOT generic.** An earlier revision of this section said
it was. `PUT /api/params` checks `key in allowed_keys` from `_get_param_type_info()`, and that
set is built by `_build_default_params()` from **`_params_raw.all_keys()`** — the compiled
`params_pyx.so` registry, not `params_keys.h`. A key absent from the `.so` is rejected with
**403 `Parameter '<key>' is not editable.`** before any `put` runs. See the blocker below: this
is the path that actually fires on the car.

**Verified [CONFIRMED, static]:** `node --check` on the frontend, `json.loads` on the layout, and a
new test — `test_bosch_a_test_toggles_share_one_galaxy_location_and_gate` in
`starpilot/system/the_galaxy/tests/test_device_settings_layout.py` — pinning section, parent,
tier, offroad flag, default, adjacency and the Galaxy gate across all three surfaces at once.
Negative-controlled per D-009: dropping the key from `BOSCH_A_REQUIRED_KEYS` fails it, and moving
the row away from its sibling fails it. 21 passed in that file, 116 passed across it plus
`test_range_vrel_assist.py`, `test_radard_bosch.py` and `test_far_lead_brake_limit.py`; ruff clean.

**Not covered:** nothing here renders the page. These are source-level assertions that the two
rows agree; they do not prove Galaxy draws the toggle, and no car has been looked at. Note also
that `starpilot/system/the_galaxy/tests/test_device_settings_frontend.py` has **5 failures at
HEAD**, unrelated to this work (a developer-mode notice missing from the JS and CSS) — confirmed
pre-existing by re-running them against a stashed tree.

### Where the native value went

Nothing was removed and the capnp schema is unchanged. When the assist is active, `radarState`'s
`vRel`/`vLead` carry the correction and `leadOne.vRelRangeDerived` still carries the raw fit; the
**native U11 `vRel` for the same track is recoverable from `liveTracks`** by matching
`rr.points[].trackId` against `radarTrackId`. So a log still contains all three numbers and the
comparison the D-044 channel exists for stays auditable.

---

## 🟡 D-053 reworked after replay — 2026-09-16

`[REPLAY, open loop — not road evidence]` The original D-053 was replayed through the real
`radard.Track` on every Bosch-A sweep of `00000232`, `00000236`, `00000237`, `00000239` and
`0000023a` (script `d053dump.py`; instrument check against logged `vRelRangeDerived`/`aLeadK`
matched: median |diff| 0.000, ≥93% of frames under 0.01 m/s). It **helped** on real rail/onset
closings (236 12:51, 18:55) but **hurt** on two recorded range faults: the `00000232` 3:02.8 range
walk (59.0 → 56.1 → 59.1 m while U11 read +2.1 → +0.1 → +1.1) and the `00000236` 22:13.6
new-track settle (77.2 → 59.5 m in 1.8 s, hit the 8.0 cap). Because it fed the corrected vLead
into the lead KF, `aLeadK` — which the MPC brakes on — dipped to −3.8 / −7.2 / −6.0 / −4.5 / −5.9
m/s² on replay.

**The rework (in `radard.py`, same toggle):** a 15-sample (~1 s) long range fit must agree with
the 5-sample fit for 5 consecutive updates; clear-only guards on long-fit residual (0.6 m), span
(0.6–1.5 s), ego speed (≥5 m/s), a lead that would read as driving backwards (>5 m/s), and a rail
rule (on the U11 −13.5 rail the long fit alone decides); the correction never takes the lead
below 0 m/s; and **the KF stays on native U11**, so `aLeadK` is unchanged by construction. Every
number carries its evidence in the comment block. 74 unit test functions in
`test_range_vrel_assist.py` (was 43), each guard with a negative control.

Open-loop gain = reduction in |published vRel − centered ±0.7 s range fit|, integrated (m/s·s):

| Route | Original: active / gain+ / gain− / min aLeadK shift | Rework: active / gain+ / gain− / aLeadK |
|---|---|---|
| 232 | 21.8 s, hurt at 3:02.8 / min aLeadK shift −3.8 | 1.1 s / +0.4 / −0.2 / native |
| 236 | 27.5 s, hurt at 22:13.6 / −7.2 | 7.4 s / +12.7 / −0.5 / native |
| 237 | 33.5 s / −6.0 | 18.8 s / +35.0 / −0.0 / native |
| 239 | 16.4 s / +15.9 / −1.3 / −4.5 | 5.9 s / +7.1 / −0.0 / native |
| 23a | 9.3 s / +6.6 / −0.1 / −5.9 | 0.7 s / +0.5 / −0.0 / native |

**What bites:**
- **Both recorded phantoms are held off by ONE arm update.** At `ARM_UPDATES = 4` the 232 walk lets
  1.0 m/s·s through and the 236 settle 0.4. Do not lower it.
- **Onset latency is 0.77 s** from a closing kink to the first correction (original: 0.42 s),
  against a measured U11 onset lag of 0.88–1.28 s. The margin is ~0.1 s.
- **It is blind to a range error that U11 agrees with** (the t≈11 s false brake) — unchanged.

**Bookmarks on 239/23a (feature not running on either):** `00000239` 10:33.7, phantom hard brake
then driver gas: lead track's yRel went −0.9 → −3.7 while range fell 74 → 61.5 m and U11 went
+1.5 → −7.5, radar lead dropped at 10:31.5, vision held 69–75 m. Looks like an **association
fault**; the rework's |y| ≤ 1.5 m gate keeps it inert there and it cannot fix it — needs a raw-track
look. `0000023a` 6:23.3, stop-and-go hard brake closing to 17.8 m after a ~1 s radar lead dropout at
6:20.5; disagreement 1.7 < 2.0, rework inert. Neither is a parser lockout.

**Lockout census, D-054 implemented `[REPLAY]`:** on 232 / 236 / 237 / 239 / 23a, old parser → D-054:
- Locked object-time: 6.5 / 7.3 / 14.1 / 6.7 / 9.6 % → 1.9 / 3.2 / 7.4 / 3.6 / 7.8 %.
- Followed-lead-lost ≥1 s episodes: 8 / 8 / 9 / 1 / 2 → 2 / 3 / 4 / 0 / 0.
- Published sweeps went up on every route.
- No new lead-lost episode appeared. Full table and the per-sweep A/B are in D-054.

## Handoff — what is live, what is untested, what bites

**The four contract files are the handoff.** `AGENTS.md` → `STATUS.md` → `DECISIONS.md` →
`CLAUDE.md`, in that order. `.claude/hooks/session-start.sh` prints them at the top of every
session; on a non-remote machine it prints the contract and then skips the dependency install
and build, which is correct — do not "fix" that by making it build on a laptop.

**More than one agent writes this branch.** As of 2026-09-15 there are three: two Claude Code
sessions on separate accounts and a local one. `git pull` before you start and before you
push; a rejected push means someone else moved, so read their commits before merging (a real
example is `7344dc4`, where the other session had landed `D-049` and the corpus harness).

### Awaiting a road test

**`FarLeadBrakeLimit` (`ebf4c20`, amended 2026-09-16) is a TEST feature, default OFF.** It bounds
braking demanded for a lead far away in both time and distance, and it is the only behavioural
change in this work. **As originally shipped it was inert on the fault it was written for** — see
*The far-lead limit was inert on the fault it exists for* above for why, and for the fix. Current
replay evidence: 39 segments, toggle OFF vs ON, **exactly one segment changes** — seg 10 of
`00000231--5782493b00`, the flagged false brake: 21 altered frames, peak `aTarget`
−2.901 → −1.950 m/s². Zero altered frames across the other 38 segments. Its positive evidence base
is still **one example**, which is why it is off by default and labelled TEST. Enabling it is a
deliberate act: `Params().put_bool("FarLeadBrakeLimit", True)`. It carries a **known, pinned defect
in its ramp anchor** — read that subsection before touching the function.

**`RangeDerivedVrel` (2026-09-16, REWORKED the same day) is a TEST feature, default OFF.** It lets
the range-derived closing rate correct the Bosch-A native U11 velocity for the lead, one-sided and
bounded — see *D-053 reworked after replay* below and **D-053**. It has **open-loop replay evidence
on five routes and no road evidence**: the reworked version never ran on a car. It **is now
reachable on the car** once the device is on `b2baba87` or later (open item 12 closed). Routes
`00000239` and `0000023a` were driven on `fa262e0c`, before that fix, and `RangeDerivedVrel` is
absent from their `initData`: **the feature did not run on either**. Enabling it is a deliberate act:
`Params().put_bool("RangeDerivedVrel", True)`, offroad, then cycle offroad→onroad.

Everything else committed in this work is tooling, tests or documentation. No default
behaviour has changed.

### Known traps on a laptop

- **You cannot build this tree on macOS.** The checked-in `.so` files are aarch64 and the
  build needs capnp/zmq/eigen/OpenCL. Anything importing `cereal`, `opendbc.can` or
  `params_pyx` will not run there. The pure-Python tools are written to degrade instead:
  `konik_login.py`, `plain_http.py` and `konik_preflight.py` need only `requests`.
- **The skip-worktree protection is remote-only.** The hook that marks the 53 tracked device
  binaries runs under `CLAUDE_CODE_REMOTE`. On a laptop those flags are not set — which is
  harmless while you cannot build, but never `git commit -a` regardless.
- **macOS CLT Python 3.9 is linked against LibreSSL 2.8.3** and cannot complete a TLS
  handshake with `api.konik.ai`. `plain_http.py` falls back to the system `curl`; the tools
  report which transport they used. If you see the curl note, that is expected, not a fault.
- **`python`/`pip` do not exist on macOS** — `python3`/`pip3` only.
- **The standalone tools must stay Python 3.9-compatible.** That macOS CLT Python is 3.9;
  `pyproject.toml` sets ruff's `target-version = "py311"`, so lint here will happily push a
  3.11-only construct into a file a user runs on 3.9. That already happened once:
  `from datetime import UTC` (3.11+) was written by ruff's `UP017` autofix and the preflight
  died at import on the user's machine. `tools/lib/tests/test_py39_compat.py` now AST-parses
  the three standalone tools at `feature_version=(3, 9)` and greps for a ratchet list of
  too-new runtime names; `UP017` is disabled for those three files in `pyproject.toml`.
  Note the limit: no 3.9 interpreter exists in the agent container, so this is a static check,
  not a real 3.9 run.

### Still open, in priority order

See **Next** at the end of this file. The short version: the Konik transport is verified
from the user's laptop (7 passed / 1 warning, 2026-09-15) but nothing has yet decoded a
Konik-fetched rlog, and no route has reached an agent session by any path other than a manual
file upload; `test_leads.py` has never been run; and D-048's validation gate is unmet, which
is why the far-lead limit ships off.

---

## Tooling added 2026-09-15

**`tools/konik_preflight.py`** — verifies a Konik (or comma) server end to end from a machine
where the network actually works. Agent containers cannot: this environment's policy answers
**403 to CONNECT** for `konik.ai`, `api.konik.ai` and `api.commadotai.com`, so no agent session
can reach a route. Run it on your laptop or the comma and paste the output back.

It walks reachability → token → `/v1/me` → devices → routes → files → an actual ranged rlog
fetch → **Bosch-A content**. That last step is the one specific to this repo: it counts Bosch-A
object frames by CAN ID and bus, `liveTracks`, and `radarState` leads, and warns explicitly when
frames are present but no lead was ever reported — which per this file is what every clean
replay so far has actually contained. A route that uploads cleanly but carries no Bosch-A frames
will not advance the radar work, and finding that out locally costs seconds.

The script mirrors the 80 Bosch-A CAN IDs so it runs on an unbuilt checkout;
`tools/lib/tests/test_konik_preflight.py` asserts that mirror against opendbc exactly, because
a drifted copy would report "0 Bosch-A frames" on a route full of them.

**It does not trust the newest route.** A comma uploads qlogs eagerly and holds rlogs for
WiFi, so the route you just drove is routinely qlog-only — which is what the first live run
hit. Reporting "qlogs only" there and stopping is true and useless, because it reads as *this
path cannot deliver radar data* when the account may hold rlogs a few routes back. So when the
newest route has no rlogs the script walks back up to `--scan-routes` (default 10) older
routes, names the first one that does, and **downloads from that route**, proving the data
path on an rlog rather than on a qlog. `--route` disables the walk-back entirely: an explicit
route is the user's choice and is never second-guessed. When nothing in range has rlogs it
warns and says why (WiFi, or request the segments), rather than failing — the transport is
still proven at that point, and hiding that would be the worse error.

**`tools/bosch_a_corpus_report.py`** — the missing corpus harness. Pools **device-side**
behaviour across the segments of a drive: per-segment and pooled radar/vision range-gap
distributions, `|d(gap)/dt|`, range and lateral residuals against the vision lead, radar- and
vision-lead occupancy, `experimentalMode` occupancy and its correlation with radar-lead use,
U11 versus a centred range derivative with the lag of the cross-correlation peak, and
hard-brake events with headway, TTC, `longitudinalPlanSource` and override.

It is the offline twin of `bosch_a_route_report.py` in the same sense `konik_preflight` is:
the route report replays CAN through the real `RadarInterface` and answers *what the parser
would publish*; this one reads `radarState`, `modelV2`, `carControl` and `longitudinalPlan`
and answers *what the device actually did*. Segment collection is imported from the route
report rather than copied, so the two cannot drift.

```bash
python tools/bosch_a_corpus_report.py <route.zip|dir|rlog> --json corpus.json
```

Two things it gets right that are easy to get wrong, and both are pinned by tests:

- **The lateral residual is a SUM** (`yRel + lead.y[0]`), because radar `yRel` is car-frame
  left-positive and model `y` is device-frame. `track_matches_vision` adds them; differencing
  would report ~2× the residual on any off-centre track and ~0 on a genuinely mismatched one.
  The tests locate the real matcher's accept/reject boundary and assert the report reads the
  tolerance exactly there, so a drift in `radard` breaks them.
- **Hard brakes are grouped into runs**, reported at each run's peak. STATUS.md's counts are
  runs ("only **four** runs with commanded accel < −1.5"); a 1.5 s brake is ~30 frames, so a
  per-frame table would inflate the event count by an order of magnitude — exactly the
  false sample size D-042 warns about.

`[NOT VALIDATED AGAINST A ROUTE]` Its geometry, arithmetic and extraction are tested — the
last against a synthetic segment of real capnp Events with hand-computed answers — but it has
**never been run on a real drive**, so message rates, dropouts and clock skew are unexercised.
Until it re-derives the recorded figures on `00000231--5782493b00`, its output is a fresh
measurement, and a disagreement with the numbers above is **unresolved in both directions**,
not a correction to either.

**`tools/bosch_a_scenarios.py` + `tools/bosch_a_viewer.html`** — drives synthetic Bosch-A CAN
frames through the **real** `RadarInterface` and records what the parser decided on every sweep,
then renders it. Not a simulation: every published number came out of the parser. Frame builders
are imported from the Bosch-A test module rather than copied, so they cannot drift.

Five scenarios, each tied to a decision: `closing_lead` (baseline), `saturation_rail` (D-041),
`high_u10_decel` (D-042), `vrel_contradiction` (D-043), `no_targets` (the honest baseline —
renders empty, because that is what the data we have actually contains).

```bash
python tools/bosch_a_scenarios.py --html viewer.html   # rebuild the page
python tools/bosch_a_scenarios.py --scenario saturation_rail   # JSON for one scenario
```

Published viewer: <https://claude.ai/artifact/PAWLymcsRvobb9waW87KuT>

### What the scenarios show

`[CONFIRMED, synthetic input through the real parser]` **D-041 holds.** A stopped car approached
at 19.4 m/s rails U11 on all 60 sweeps, and the parser publishes a point on 59 of them, pinned at
the −13.5 m/s bound. Longest coast 0.07 s, well inside `BOSCH_A_STALE_S`.

`[INFERRED, needs a real route]` **Two scenarios coast far past the stale limit.**
`high_u10_decel` publishes nothing for 14 consecutive sweeps (0.98 s) and `vrel_contradiction`
for 34 (2.37 s), against `BOSCH_A_STALE_S = 0.20 s`. That is the shape of the failure D-041 and
D-042 describe — a coast that outlives the stale limit, after which the point is deleted and
`radard` falls back to vision. **These inputs are synthetic and may not be representative**: the
`high_u10_decel` gap begins where the scenario's own range clamps, so part of it is likely an
artifact of the construction rather than parser behaviour. This is a question to put to a real
route, **not a demonstrated defect**, and it must not be treated as one.

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

- **Still no closed-loop road validation.** One segment now proves the parser tracks real
  objects (see the section above), but that is offline replay of a recorded drive: it says
  nothing about how the car behaves when this radar drives the planner.
- **The P061B correction is unproven.** Route 4's active duration is shorter than the
  original time-to-fault. Replay cannot predict the changed closed loop or PCM acceptance.
- **Fusion timing is untested here.** `radard` runs at the ~20 Hz model rate while Bosch-A
  measures at ~14.35 Hz; the duplicate-payload path (`measurement_update=False`) is covered
  by unit tests only, never on a live bus.
- *Resolved 2026-09-15 (evening).* **`test_leads.py` has now run** — 6 passed, including
  `test_radar_fault`, which drives `replay_process_with_name("card", …)`. It runs in the arm64
  container described above; it still cannot run on macOS directly (`SocketEventHandle` needs
  eventfd, and that test is skipped on Darwin by its own marker).
- *Resolved 2026-09-15 (evening): routes now reach an agent session end to end. The walk-back
  preflight passed **8/8** live, `tools/konik_fetch.py` pulled **five routes in full**, and all
  five were decoded and analysed in the arm64 container — so step 8's payload check is done, on
  a different machine than the fetch. What remains true is only the last line below: a remote/
  cloud agent container still cannot reach `konik.ai`. Original entry kept for the record.*
- **The Konik transport is verified from the user's laptop; no route has reached an agent
  session over it.** On 2026-09-15 the user ran `tools/konik_preflight.py --dongle-id
  11c8fa231c0499ed` on macOS and it returned **7 passed, 0 failed, 1 warning**: DNS,
  HTTP 401 on `/`, a valid token (expires 2026-12-14), `GET /v1/me`, **551 routes**,
  a file listing, and a ranged download of 0.5 MB at 0.3 MB/s. Every HTTPS call fell back to
  the system `curl` (LibreSSL 2.8.3), as designed. Two limits stand:
  * **Segments are served from `api.konik.ai` itself**, not a separate CDN — so one host
    covers both the metadata and the data path. Good for a network policy, but it means the
    storage host has *not* been exercised as a distinct allow-list entry.
  * **Step 8 was SKIPped** — `cereal` is not importable on that laptop, so nothing has
    decoded a Konik-fetched rlog and confirmed Bosch-A content end to end. The transport is
    proven; the payload is not.
  The agent container's own policy still answers **403 to CONNECT** for `konik.ai`, so the
  fetch must happen on the laptop or the comma either way.
- **Fresh routes are qlog-only, and qlogs are useless here.** The newest route on the account
  (`11c8fa231c0499ed|00000233--e03fb98cc3`, 2026-09-15T21:06Z) listed **0 rlog segments and
  4 qlog segments**. The device sends qlogs eagerly and holds rlogs for WiFi. qlogs are
  decimated and drop the CAN data, so a qlog-only route cannot support any radar claim.
  `konik_preflight.py` now walks back up to `--scan-routes` (default 10) older routes to find
  one that has rlogs, names it, and downloads from *that* route. **Resolved 2026-09-15
  (evening):** the walk-back was run and found `11c8fa231c0499ed|00000232--fc8dad0d18` with
  **27 rlog segments**, one route back from the qlog-only newest. Five routes have since been
  fetched in full — `0000020a` (17 segments), `0000020b` (38), `00000213` (23), `00000231` (32),
  `00000232` (27).
- The checked-in `.so` files remain **aarch64**, and a local build still overwrites 53 of
  them. The SessionStart hook rebuilds and masks them, but a session that skips the hook
  will hit both.
- 🟡 **The recorded corpus numbers are PARTLY reproduced — see the reproduction section above
  and D-051.** Reproduced: the pooled gap distribution (n=4,872 against a recorded 4,868, every
  percentile within the 1.52 m `RADAR_TO_CAMERA` offset), `corr(visionLead%@0.5, radarLead%)`
  (+0.818 against +0.817), the per-segment identities, and every hard-brake row's kinematics.
  **Still NOT reproduced, and unresolved in both directions:** the U11-vs-range-derivative
  statistics (9.16% gross disagreement against a recorded 2.26%; peak at lag 0 against −4),
  `|d(gap)/dt|`, and `corr(experimental%, radarLead%)` (−0.197 against −0.084). The original
  defect that caused this is unchanged and is why it cost a session to sort out: The original defect: the 6- and 16-segment results
  above (pooled radar/vision gap percentiles, the
  `|d(gap)/dt|` distribution, `corr(experimental%, radarLead%)`, the U11-vs-range-derivative
  cross-correlation, the hard-brake tables, the per-segment lateral residuals) were recorded
  in commits `5edbd59`, `69e1683` and `bdf98de`, **all three of which touch only `STATUS.md`
  and `DECISIONS.md`**. `tools/bosch_a_route_report.py` is the only committed route tool and
  it computes none of those quantities — it counts frames, replays the parser, and reports
  points/tracks. So the numbers cannot be re-derived, re-checked, or re-run on a new route,
  and they are unbound to a commit in the sense AGENTS.md §3 requires. They are recorded
  above as prior evidence and not as reproducible until the new harness has actually re-derived
  them. That run is also **D-048's validation-gate item 2** and D-049's item 3, so it still
  blocks both designs — writing the harness did not clear them, running it will.

*Resolved 2026-09-15, previously listed here:* the two `radar_interface.py` ruff findings
(`edef432`) and the dangling `.venv` symlink (`edef432`).

## Next

1. **Done 2026-09-15 (evening)** — five routes fetched and analysed in-session. Fetch any
   further route with:
   ```bash
   python3 tools/konik_fetch.py --route '11c8fa231c0499ed|<route>' --out <dir outside the repo>
   ```
2. **Done 2026-09-15 (evening)** — `test_leads.py` runs: 6 passed, including the
   process-replay `test_radar_fault`.
3. **Done 2026-09-15 (evening)** — `JamesL787/openpilot` PR #9 is **merged** and its merge
   commit `10e4705` is an ancestor of this branch, so the §8 run (492 passed / 1 failed /
   2 skipped) *is* that re-run. `firestar5683/StarPilot` PR #124 is **closed unmerged** and was
   not re-run.
4. **Done 2026-09-15 (`b9612b2a`)** — `common/params_pyx.so` and `common/libcommon.a` rebuilt;
   `FarLeadBrakeLimit` is now writable on the car. The `.so` went from 821 keys to **822**,
   matching the header. Verified by loading the old and the new `.so` and diffing `all_keys()`:
   the sole difference is `FarLeadBrakeLimit` and nothing was dropped; `put`/`get`/`remove`
   round-trip on it; `test_far_lead_brake_limit.py` goes 16 passed / 1 failed → **17 passed**.
   Nothing else had drifted — this was never a general "the artifacts are stale" problem, and it
   must **not** be diagnosed with `strings` (see the test-results section).

   The rebuild used the toolchain that produced the committed artifacts: Ubuntu 24.04,
   clang 18.1.3, Python 3.12.3 and **Cython pinned to 3.1.4** (3.3.0 regenerates
   `params_pyx.cpp` with ~8k lines of churn and a different `.so`), with `SP_FORCE_TICI=1` to
   select `arch=larch64` and `SP_SCONS_CACHE_DIR` set because the larch64 path hardcodes
   `/data/scons_cache`. Locally that is the `oprad-build:cy314` image:
   ```bash
   docker run --rm --platform linux/arm64 -v "$PWD":/work -w /work \
     -e SP_FORCE_TICI=1 -e SP_SCONS_CACHE_DIR=/tmp/sconscache oprad-build:cy314 \
     bash -lc 'export PATH=/opt/venv/bin:$PATH; scons --cache-disable -j$(nproc) \
       common/libcommon.a common/params_pyx.so'
   ```
   Mount at **`/work`**: the generated `params_pyx.cpp` embeds its source path in a metadata
   comment, so mounting elsewhere adds a spurious one-line diff. Builds at `/src` and `/work`
   were otherwise byte-identical, so this is reproducible.

   ⚠️ **Verify the hash afterwards — scons will claim success without rebuilding.** A second
   pass after `git restore` printed ``scons: `common/libcommon.a' is up to date`` and relinked
   the `.so` against the **old 821-key archive**. Removing the `.o` files is not enough; scons
   trusts `.sconsign.dblite`. Delete the signature DB and the archive
   (`./tools/clean_build_artifacts.sh --sconsign`), then confirm the hash changed.

   Shipping rebuilt device binaries stays a deliberate, separate commit (AGENTS.md §10,
   pattern `a972d15`); `b9612b2a` contains those two binaries and nothing else.

   **This is static verification only — no replay, no road evidence.** It makes the toggle
   reachable; it does not validate the feature, which remains TEST and default OFF. Note also
   that the Galaxy row carries `requires_offroad: true`, `settings_tier: "advanced"` and
   `parent_key: AdvancedLongitudinalTune`, so it is editable only while parked, with developer
   mode on, under an enabled parent — and only on cars whose fingerprint is in `HONDA_BOSCH_A`.
5. **Done 2026-09-15** — a real-target route exists; see the section above. The follow-on
   is the shadow `vRelRange` channel (D-044): this segment carries 77 railed samples and an
   ~11 m vision/radar range disagreement, which is exactly the material that channel was
   published to be judged against. Compare `vRelRange` with U11 on this route before anything
   is wired into control.
6. **Done 2026-09-15** — a track-ID reuse does reset the lead Kalman filter, by object
   absence rather than by the incarnation boundary the parser computes, and the reset is
   carried by a single `liveTracks` message. See *Track-ID reuse and the lead Kalman filter*
   above and D-049. Two follow-ons, both needing route data: how often lifecycle breaks
   actually occur, and whether a **seamless** ID reuse (no lifecycle break at all) is possible
   — if it is, nothing resets anywhere.
7. **Partly done 2026-09-16 — the event is now characterised and a mitigation is replayed;
   the constant it argues for is still not justified.** See *The far-lead limit was inert on the
   fault it exists for* above: the amended `FarLeadBrakeLimit` reduces the commanded peak from
   −2.901 to −1.950 m/s² on this event and alters nothing across the other 38 segments replayed.
   That bounds the *output*; it does not fix the *sensor* error, and it does not supply the
   distribution below. Original wording:
   **Still open: characterise the t≈11 s range-drift false brake.** (A second candidate was
   examined this session on `0000020a` at 9:53 and **refuted** — that one is vision jitter plus
   braking authority, not range drift. So the original event stands alone, still n=1.) It produced a
   −2.89 m/s² command and a driver override, and it passed the innovation gate, the
   one-sided rate check and the gross-distance gate. Needed before any fix: the distribution
   of radar-versus-vision range disagreement across routes where the radar is right, so a
   tightened `HONDA_BOSCH_A_GROSS_DISTANCE_M` can be justified rather than guessed (D-042).
   More real-target routes are the blocker.
8. **Done 2026-09-15 (evening)** — the harness ran; see the reproduction section and D-051 for
   what matched and what did not. Original wording:
   **Run `tools/bosch_a_corpus_report.py` against `00000231--5782493b00` and compare it with
   the figures recorded above.** The harness was written 2026-09-15 and covers every quantity
   the 6- and 16-segment sections quote, but it has never seen a route, so the recorded numbers
   are still unreproduced. This is D-048's validation-gate item 2 and D-049's item 3; both
   designs stay blocked until it runs. Three outcomes, and the third is the valuable one:
   it reproduces them (the corpus becomes evidence again), it disagrees (**neither side wins
   automatically** — the recorded numbers came from code that no longer exists, and the harness
   has never been validated, so the disagreement is the finding), or it will not run on real
   data at all, which is itself worth knowing before anyone depends on it.

9. **Open: the far-lead limit's ramp anchor is inverted at the call site.** `[CONFIRMED]` The unit
   tests anchor the ramp at `ACCEL_MIN` (−3.5); the call site passes `output_accel_min` (−0.5 on
   most fault frames), so partial confidence produces a *tighter* floor than full confidence. It is
   pinned by `TestKnownRampAnchorDefect`, not endorsed. Fixing it in isolation makes the feature
   worse on the only fault we have (−2.818 vs −1.950), because it is compensating for a ramp that
   collapses on the same corrupted `vRel` that causes the fault. What is needed before changing it:
   a second fault example, so the ramp's behaviour can be judged on more than n=1. Re-run the
   39-segment OFF-vs-ON diff on any change here.

10. **Open: more routes, in three specific shapes.** The 40 m distance floor separating the fault
    (61–64 m) from a correct stop-and-go brake (27 m) has **376 frames sitting in the 30–40 m band
    on `0000022f--8acb9d34ac`** that pass every other gate, so the margin is not wide. Wanted:
    highway cut-ins/merges settling at 40–70 m; downhill or trailing-throttle following at 45–60 m;
    and a second long-range radar range-walk fault. The third is the one that takes the positive
    evidence past n=1.

11. **Open: `opendbc_repo/opendbc/car/panda_runner.py:17` calls `get_car()` with the old
    signature** and will raise `TypeError: get_car() missing 1 required positional argument:
    'params'`. This is the same break fixed in `process_replay.py` on 2026-09-16, but it is in a
    vendored subtree and a standalone runner, so it was left alone rather than fixed in passing.

12. **CLOSED 2026-09-16 by `b2baba87`:** the larch64 `common/params_pyx.so` and `libcommon.a` were
    rebuilt with item 4's pinned recipe, 822 → 823 keys, `RangeDerivedVrel` the only addition. The
    Galaxy layout test's strict xfail on this is removed, so it now guards the regression. Device
    check: `PYTHONPATH=/data python3 -c "from openpilot.common.params import Params; k=Params(memory=True).all_keys(); print(len(k), b'RangeDerivedVrel' in k)"`
    must print `823 True`. The original entry is kept below for the record.
    **Was: `RangeDerivedVrel` is unreachable on the car until `common/params_pyx.so` is rebuilt
    for larch64. THIS IS NOW CONFIRMED ON A REAL DEVICE, not just inferred — 2026-09-16.**
    `[CONFIRMED — observed on the car]` Toggling the row in Galaxy returns
    **403 `Parameter 'RangeDerivedVrel' is not editable.`** The key is in `common/params_keys.h`,
    but the checked-in `.so` is aarch64 and carries the **old 822-key list**. Two separate paths
    fail on it: Galaxy's `PUT /api/params` rejects the write up front, because `allowed_keys`
    comes from `_build_default_params()` → `_params_raw.all_keys()` → the `.so`; and
    `get_bool("RangeDerivedVrel")` raises `UnknownKeyName` in `_range_vrel_assist_enabled()`,
    which swallows it. The feature stays off no matter what the UI shows. **Do not patch the
    Galaxy allowlist to get past the 403** — see the section above for why that makes it worse.
    **This container cannot do the rebuild** (no docker daemon, no qemu, no aarch64
    cross-compiler, Python 3.11.15 / Cython 3.3.0 vs the pinned 3.12.3 / 3.1.4). This is open item 4 repeating itself with a
    different key. A **native on-device build** makes the toggle usable in the field (see the
    section above for the two traps: `scons common/` dies on missing eigen headers, so build
    `common/params_pyx.so` as a single target; and never remove the `prebuilt` marker), but it
    does **not** close this item — that binary is not the reproducible pinned-toolchain artifact
    and must not be staged. Use item 4's recipe **verbatim** — the `oprad-build:cy314` image, Cython pinned
    to 3.1.4, mounted at `/work`, `SP_FORCE_TICI=1` — and heed its warning that **scons will report
    success without rebuilding**: delete `.sconsign.dblite` and the archive
    (`./tools/clean_build_artifacts.sh --sconsign`) and confirm the hash actually changed. Expected
    key count **822 → 823**, with `RangeDerivedVrel` the sole difference and nothing dropped, proved
    by diffing `all_keys()` between the old and new `.so` — **not** with `strings`. Ship it as a
    deliberate, separate commit containing only the binaries (AGENTS.md §10, pattern `b9612b2a`).
    Until then, `test_param_default_is_off` passes here only because this container's **x86_64**
    `.so` was rebuilt in-session; that proves nothing about the device.

13. **Open: the OFF-vs-ON replay diff for `RangeDerivedVrel` has never been run.** This is the
    single most valuable next step and it needs no new route data — the 39-segment corpus and the
    harness that produced the `FarLeadBrakeLimit` diff (39 segments, one changed) already exist.
    Run it with the param OFF and then ON and diff `aTarget` frame by frame. **The expected result
    is zero altered frames on all 39**, because nothing in the corpus should hold a 2.0 m/s
    U11-versus-range disagreement on a lead track for 5 consecutive measured updates. Both outcomes
    are informative and the second is the important one:
    * Zero altered frames → the arming gate is doing what it was designed to do on ordinary
      driving, and the feature is inert until the conditions it targets actually occur.
    * **Any** altered frames → do **not** treat that as the feature working. Read those frames
      directly: it means the corpus contains a sustained disagreement that nobody has
      characterised, and until it is known whether U11 or the range channel was right there, an
      altered frame is as likely to be the hazard subsection above as a fix.

14. **Open: two of the `RANGE_VREL_ASSIST_*` constants rest on almost nothing.** Both are bounds
    chosen to limit damage, not values fitted to data, and the comment blocks say so — read them
    before changing a number (CLAUDE.md §5).
    * `RANGE_VREL_ASSIST_MAX_CORRECTION_MPS = 8.0` is sized from **n = 2** recorded events
      (the `000001f9` rail, ~5.9 m/s, and the `000001f3` onset, ~4.6 m/s). A third event is what
      would turn it into a justified number.
    * `RANGE_VREL_ASSIST_MIN_DISAGREEMENT_MPS = 2.0` currently clears the one recorded false brake
      by **1.62 m/s** (worst disagreement there is +0.38). That margin is the whole safety
      argument and it is measured against a **single** fault. Lowering it without a second
      range-walk fault to test against is how this feature becomes an amplifier — the same
      blocker as open items 7 and 10.

15. **CLOSED 2026-09-16 by the D-053 rework:** the lead KF now stays on native U11, so `aLeadK` is
    never corrected and cannot be left frozen at a corrected value across a coast. Kept for the record:
    **Was: the correction is inconsistent across a Bosch-A coast, in a bounded way.** `[CONFIRMED
    — static]` On a coast the assist clears (D-052: a coast refreshes `last_seen_nanos`, so it is
    **not** bounded by `BOSCH_A_STALE_S`, and 121.8 s of continuous suppression has been measured —
    holding a correction across one would be unbounded staleness). But the lead KF is not stepped
    on a coast either, so published `vRel`/`vLead` revert to native U11 immediately while `aLeadK`
    stays frozen at its last corrected value until the next measured update. Both sides are
    bounded and both sit on the conservative side, but for the length of the coast the two
    disagree. It is documented in D-053 rather than papered over. Whether it matters at all is a
    replay question, and open item 13 is the run that would show it.

16. **Open: road-test the reworked `RangeDerivedVrel`.** Needs the device on this commit, the
    `823 True` check from item 12, and the toggle stored (confirm it in `initData`, not the UI). The
    replay evidence rests on two one-update margins (the 232 walk and the 236 settle, D-053
    revision): a road event that arms in 4 updates would pass. Onset lead over U11 is only ~0.1 s.
    Item 14's `MIN_DISAGREEMENT` argument now also has the two recorded phantoms behind it.

17. **D-054 lockout fix: implemented 2026-09-17, replay and static only.** The gate now rejects a
    range only if it contradicts both the last accepted sample and the last gated range (the
    `range_anchor` that coasts advance). Evidence is in D-054.
    - **Open, road:** needs the device on this commit. Watch for a lead that pulls away or closes
      while the rate check coasts: it should stay published, with measured=False.
    - **Residuals, investigated 2026-09-17 (replay and static only):** the `ratio_vrel` timing has no
      effect in replay. Lasting steps and stale rate history are real causes. The proposals are in
      D-055 / D-056 / D-057, on branch `proposal/d054-residuals`, which is not for the car.

18. **CHARACTERISED 2026-09-17 (see item 27): `00000239` 10:33.7 phantom hard brake is a same-identity range walk at 10:30.3, not an association fault.** Original note: Looks like a lead association fault (lead
    track yRel −0.9 → −3.7 m while range fell 74 → 61.5 m, U11 +1.5 → −7.5, vision held 69–75 m).
    Neither the rework nor D-054 touches it. Needs a raw-track look at 625–635 s.

19. **ON THE CAR BRANCH 2026-09-17 (replay and static only): D-055 invalid-slot hide guard.**
    See item 26 for the fresh-route evidence that promoted it.
    - Replay vs D-054: 0 lost point-sweeps, 2,252 restored as coasts (504 lead), including
      237 track 9 (21.2 s).
    - Static: 4 tests.
    - Candidate for the car once fresh routes on 0756f810 confirm D-054 on the road.

20. **Rejected in replay: D-056 rate check fit over gated ranges.** Its re-admitted vRel over-closes
    by >3 m/s about 4× as often as reference, and it loses 14 lead point-sweeps on 0000023a. See D-056.

21. **D-057 re-anchor on a lasting clean step — ON THE CAR BRANCH 2026-09-17 (replay and static
    only; see item 28).** Earlier prototype record: on top of D-056 the replay is clean:
    0 lost, and newly measured vRel is at or better than reference. It restores 236 track 38
    (17.1 s) and 237 track 31 (16.2 s). It is stacked on D-056, so it is not shippable as-is.

22. **Next-session tasks (radar residuals).**
    1. **Done 2026-09-17 (item 26).** Lock census run on 0000023b / 0000023e (both on 0756f810).
    2. **Done 2026-09-17 (item 28).** D-057 re-based onto D-055 alone, 4 tests rerun, `ab5.py`
       replay D-055 vs D-055+D-057 on all seven routes with the future-slope metric.
    3. **D-055 part done 2026-09-17 (item 26):** lost = 0 on both fresh routes, no new measured
       vRel; committed to the car branch on its own. D-057 still needs the re-base and replay.
    4. **Done 2026-09-17 (item 28):** 237 track 63 is absent 51.4 s under D-055 and published
       measured under D-057 (23.4→20.5 m, vRel −1.1→0, non-lead).
    5. Decide whether a join should be allowed to publish measured when the joined range contradicts
       U11 (see the D-054 census). That needs its own test, like the D-057 contradiction control
       with an 8.5 m step.
    6. 236 track 21 (20.4 s, fully degraded lead lockout) is not addressed by any proposal.
    7. Item 18 (239 phantom brake) is still open.
    Tooling: `/routes/an2` and `/routes/an2/parsers` in the `oprad-routes` volume.

23. **Hard brakes on 0000023b / 0000023e are not RangeDerivedVrel; an aLeadK limiter was tried and
    NOT proposed (replay only, 2026-09-17).**
    - Toggle read from initData: 23b RangeDerivedVrel=0, 23e =1 (all 42 rlog segments), both on
      0756f810. On 23e the assist was active (correction > 1 m/s) for 21 s of 42 min. At 0 of 11 engaged
      hard-brake onsets (aTarget < -2.5) was it active. The hard-brake rate was 22/h, against 23/h and
      19/h on toggle-off 237 and 239.
    - Mechanism. A short closing transient drives the lead KF's aLeadK to -3..-7 with aLeadTau ~0 in
      about 0.5 s. Past aLeadK < -`MODEL_LEAD_TRAJECTORY_MAX_LEAD_BRAKE`, `build_model_lead_trajectory`
      falls back to `extrapolate_lead`. Above 35 mph that holds aLeadK **constant for the whole
      horizon**, so a -3 dip reads as "lead stops in ~4 s". Over-reactions: 23e 4:55 (future 1 s
      range slope +1.3, driver took over) and 30:20 (U11 -9.1, future -2.8). The real closings at
      9:41, 11:10, 12:51, 16:09, 25:02 and 39:04 look the same at onset.
    - Tried, and why each fails (`alk.py`, `alk3.py`, `alk4.py` in `/routes/an2`):
      1. *Clamp when the short range fit says U11 over-closes*: 0 of 22 onsets touched on 23e, 237 and
         239. At the onset the range fit is MORE closing than U11 (4:55: range -4.4, U11 -2.8). The
         transient is real for about 0.3 s; it just does not last.
      2. *Clamp when vision disagrees* (mlV − vLeadK > 2, mlA > -0.6, TTC > 4): catches 4:55 and
         30:20, and is right on median (the future slope is 2.4–3.9 m/s less closing than U11). But
         it clamps 239 10:30.5 (U11 -3.4, **future -12.7** at 68 m; vision said 25 m/s, mlA 0).
         Rejected: vision is not closing-speed truth.
      3. *Decay young decelerations* (exp decay with tau ≥ 0.6 at all speeds while aLeadK < -0.5
         for < 1 s). Scored against the true lead position 2–3 s later over 1130 engaged
         aLeadK < -2 rows on 7 routes. Current projection error at 3 s for young decels: p50
         -12.6 m (predicts the lead too close). Limited: p50 -10.0 m. Under-predictions > 3 m go
         31 → 44, and the new ones under 50 m are the **real** brakes 23e 11:10 (25 m, up to +5.8 m)
         and 12:51, plus 239 7:41. With tau ≥ 1.5 it is worse (31 → 55). It softens the brakes that
         were right.
    - What the data does say: projection error is biased toward over-braking even for old
      decelerations (all rows p50 -3.0 m at 2 s, -6.4 m at 3 s). aLeadK stays negative after the lead
      stops decelerating, which is KF lag on the release side and not only onset gain.
    - Next candidate, not built: **release-side only**. When aLeadK < -1 but the range slope over the
      last ~0.5 s shows closing has stopped growing, pull aLeadK toward 0 faster. Never soften onset.
      Score it with `alk4.py` (a projection-vs-truth metric, zero new close-range under-predictions
      as the gate) before any code. It needs a radard/long_mpc change, so it gets a DECISIONS entry
      and user OK first.

24. **BLoTv3 supervisor (experimental): two BLoTv3 behavior fixes ported and the supervisor renamed
    from BLoTv2, 2026-09-17, unit evidence (D-058).**
    Upstream `SpysyWeeb/Spysypilot` restructured BLoTv2 into BLoTv3 (`combo-blotv3`, `7aed876`,
    `docs/BLoTv3.md`). Two of those changes are supervisor behavior and are now in
    `selfdrive/controls/lib/blotv3.py`: the `t_follow` pads saturate at their ceilings instead of
    vanishing above `ONSET_MAX_A_REQ`, and the crawl hold latches on "was necessity-braking"
    instead of on the exact `JERK_SCALE_MIN` floor (released by the emergency bypass and by lead
    loss). The module, class, tests and toggle are renamed (`BlotV2` → `BlotV3`, toggle starts off).
    The BLoTv3 module split (`force_stops.py`, `stop_helpers.py`,
    `conditional_experimental_mode.py`, the model-lead anchor) is **not** ported — see D-058.
    - **Replay A/B done 2026-09-17 (rlog replay of the supervisor only, not the MPC; agy-pro
      dispatch, Claude-audited).** BLoTv2 (b38e933e) and BLoTv3 (current) were stepped side by side
      on every `modelV2` frame of 00000232 / 236 / 237 / 239 / 23a / 23b / 23e from the logged
      radar lead, `vEgo`, `aTarget` and `tFollow` (script `blotab.py` in the `oprad-routes` volume,
      `/routes/an2`). Over 115,536 engaged lead frames: `jerk_scale` differed on **0** frames and the
      crawl hold differed in **0** runs, so on these drives the two versions differ only through
      the `t_follow` pad. V3 pad exceeded V2 by > 0.2 s in 68 runs totalling 90.4 s (2.1 % of engaged
      lead frames; longest run 2.9 s, pad saturating at 0.45–0.75 s, closest lead 7.7 m on 23e),
      every one a hard-braking approach where V2 had collapsed its pad to zero.
      **It does not help the radar fault classes and slightly works against them:** during the 239
      10:30 phantom (item 27) V3 raised the pad to 0.45 s for 1.1 s (630.46–631.56 s) while V2 held
      0, i.e. V3 asks for a wider gap from a lead that is not braking; both versions softened
      `jerk_scale` identically (0.70 at 631.6 s). BLoTv3 reads `aLeadK` from the radar lead, the
      signal item 23 shows over-reacting on a range walk, so a phantom reaches it unfiltered.
      Braking *force* is unchanged by V3 on these routes (no jerk-scale difference); braking
      *onset* on a real decelerating lead moves earlier by up to the pad. Verdict: a conservative
      comfort change relative to V2, reasonable to enable for road evaluation, not a radar fix.
      Every route so far was driven with `BlotV2` ON; the renamed `BlotV3` key starts OFF, so it
      must be switched on explicitly for the next drive. Not road-validated.

25. **Release-side aLeadK rule and FarLeadBrakeLimit review. Neither shipped (replay and limited
    road evidence, 2026-09-17).**
    - *Release-side aLeadK* (`alk5.py`). For a deceleration at least 1 s old, fit
      d = d0 + v·t + ½a·t² over the last 1.0–1.5 s of range. If the fit says the lead is decelerating
      less, raise the published aLeadK (optionally capped at half, optionally only when the short range
      fit shows closing is not growing). Scored like item 23 (1130+ rows, 7 routes). Every setting
      improves the bias (changed rows at 2 s: p50 -2.6 → -1.7 m), but none meets the gate "zero new
      close-range under-predictions". The strictest setting still adds 4 cases at 2 s (> 3 m, d < 50 m)
      and 10 at 3 s (> 5 m). They sit on steady -1 to -1.5 m/s² braking where the 1.5 s quadratic fit is
      too noisy (237 6:20 at 34 m, 237 20:02 at 25 m, 23a 5:35). Not implemented.
    - *FarLeadBrakeLimit* was ON on every route 232–23e (initData). `farev.py`: 12 fire clusters. Truth
      is the 1 s future range slope.
      - Not engaged, so no effect on the car: 23a 16:36 and 23e 17:18 / 34:12. In the last two the driver
        was already braking. 23e 34:12 would have cut -5.9 → -0.7 m/s² on a lead truly closing at
        20.5 m/s.
      - Engaged, and helped (lead not really closing, relief > 2 m/s²): 237 15:37 and 15:42.
      - Engaged, negligible relief (< 0.35): 232 19:08, 237 14:24, 14:30 and 19:53, 23e 1:40.
      - Engaged, real closing, relief given: 236 14:18 (closing 5.3 m/s at 60 m, -3.2 → -2.0, no
        intervention), and **237 5:22** (slow car, mlV ~7 at 104 m, truly closing 13.5 m/s, capped
        -3.8 → -1.9; driver braked 4 s later at ~47 m).
      - Verdict: two helpful, two cut real braking, and the capping logic trusts the same vRel it is meant
        to distrust. **Not merged; the toggle stays.** Recommend OFF until the 5:22 / 34:12 shape is
        excluded.

26. **D-055 promoted to the car branch — 2026-09-17 (replay and static only; awaiting road
    evidence).** Item 22.1 and the D-055 half of 22.3, done on the two fresh routes driven on
    D-054 (`gitCommit 0756f810` in `initData`, both routes):
    - **Lock census (`lockcensus.py` / `locksumm.py`, the parser the car ran):** 0000023b (5 seg)
      76 locked sweeps of 5,828 (1.3 %, 5 s), 10 episodes, 0 ≥1 s in path, 0 ≥1 s while the
      device lead was vision-matched to the locked object. 0000023e (42 seg) 2,933 of 77,560
      (3.8 %, 204 s), 231 episodes, 62 ≥1 s, 1 ≥1 s in path, **0 lead-matched**. The longest
      is id5 at 37:22.4 for 6.9 s. Compare 232/236/237 before D-054: 2, 3 and 4 lead-matched
      episodes (5, 26, 42 s). Limited road evidence, not a road test of the fix in isolation.
    - **D-055 vs main (`ab3.py`, pair MG):** 23b lost 0 / lost_lead 0, gained 9 (0 lead), 0 runs
      ≥1 s. 23e lost 0 / lost_lead 0, gained 331 (33 lead), 4 runs ≥1 s, none lead. Every
      restored sweep is a coast (`gained_measured` 0), so no new measured vRel enters control and
      `new_measured_vrel_vs_future_slope` is n = 0. That is the item 22.3 gate, met.
    - **Static:** `test_bosch_a_radar.py` 104 passed, `test_far_lead_brake_limit.py` 22 passed
      (arm64 container). The change is the one-line hide-set exclusion from `4ab261ac` plus its
      4 tests; the comment now records the fresh-route numbers.
    - **What this is not:** D-055 has never run on the car. It only ever re-publishes coasts the
      old parser hid, so its failure mode is a stale coasted point living up to the coast limit,
      not a missing one (D-041/D-042). Watch the next route's `ab3`-style diff for lead runs.
    - Still open from item 22: re-base D-057 onto D-055 (22.2), 237 track 63 (22.4), the
      join-vs-U11 decision (22.5), 236 track 21 (22.6), item 18.

27. **Item 18 characterised — the 239 phantom brake is a same-identity radar range walk, and the
    far-lead limit was live but gated out (replay of recorded data, 2026-09-17).** Raw-slot
    replay (`rawslots.py`, `tracks.py`, `tl2.py`, seg 10, 622–640 s; first pass by an agy-pro
    worker, audited against the raw output, deliverables in the job scratch dir, worktree
    aborted for a `scratch/` path breach):
    - Track 21 is the in-lane lead at 10:22 (59 m, y 0.1, vision 66 m). Through a bend (vision
      y +1.4 → +5.2, radar y −1.9 → −3.8, opposite sign conventions) its range walks
      **72.6 → 61.9 m in 1.0 s (−10.7 m/s)** while wire U11 ramps 0 → **−7.7**, `u10` climbs
      93 → 329, then the range **parks at 61.5 m** for 2 s while U11 decays to 0 and later turns
      +2 as the range grows back to 67 m. Vision holds 73–81 m at vRel ≈ 0 throughout. No other
      valid slot within |y| < 4 m, 50–90 m; the identity never changed and slot migrations
      (s0 ↔ s1) were tracked. The worker's verdict "(b) genuine wire step, same identity" is
      correct as far as it goes; the shape is the 000001f9 range-walk class, which makes this
      the **second recorded range-walk fault** item 10 asked for (n = 2, not yet 3).
    - Command: aTarget −3.5 from 10:30.7 to 10:31.4 (0.8 s), then the lead flip-flopped between
      radar (62 m) and vision-only (76–80 m) sweep to sweep until 10:32.2. The driver's bookmark
      is 3 s after onset.
    - **FarLeadBrakeLimit was ON (`initData` seg 10) and the 2026-09-16 fix `9984de88` is in the
      route's commit `fa262e0c`, yet it fired nowhere on 239.** Both gates excluded it: headway
      68.5 / 24.5 = 2.8 s < `MIN_HEADWAY` 3.0, and TTC 63.5 / 6.6 = 9.6 s < `MIN_TTC` 10. This
      event sits just under both constants of D-042. Not re-tuned: item 25 already shows the
      limit cutting real braking inside its current envelope (236 14:18 at TTC ≈ 11), so
      widening it is a safety-tuning decision that needs the third event, not a replay of one.
    - RangeDerivedVrel would not help here: the range slope (−10.7) closes *faster* than U11
      (−7.7), so the assist direction is the wrong one. What separates this from a real closer is
      vision (13–18 m further, vRel ≈ 0) and the park-and-decay after the walk — the same
      discriminator item 25 lacked for 237 5:22. Proposal work should start from that pair.

28. **D-057 promoted to the car branch (replay and static only, 2026-09-17).** Re-based onto D-055
    without D-056 (`rejected_run` on the track state, `_bosch_a_lasting_clean_step`, constants
    `BOSCH_A_REANCHOR_*`). Static: 108 passed (4 D-057 tests: positive publishes at 1.5–1.65 s, the
    degraded and U11-contradicting controls never re-anchor). Replay D-055 vs D-055+D-057 (`ab5.py`,
    agy-flash dispatch for the mechanical run, Claude-audited): lost 0 on every route; restores 236
    track 38 (17.1 s, lead), 237 track 31 (16.2 s, lead), 237 track 63 (51.4 s, non-lead, item 22.4);
    23b/23e unchanged apart from 13 sweeps. Newly measured vRel over-closes the next-1 s range slope
    by >3 m/s on 2.2 / 2.2 / 0 % (236 / 237 / 23a) vs reference 5.4 / 5.9 / 2.6 %. **Unlike D-055,
    this admits new measured points into control** on a re-anchored identity. Not road-validated:
    the next drive is the first road evidence; check lead lockouts (lock census) and any brake event
    on a re-anchored identity. Still open from item 22: 22.6 (236 track 21); 22.5 closed by item 29.

29. **D-059: a join holds measured vRel until a post-join rate fit agrees (replay and static only,
    2026-09-17).** Closes item 22.5. After the range gate passes again following a rejection run, the
    point publishes unmeasured on its last trusted vRel until a D-043-style fit over post-join ranges
    only agrees with U11; `samples` is untouched. Census: 210 joins (9 lead); post-join measured vRel
    over-closed >3 m/s on 21.0 % vs reference 4.6 %. Static: 110 passed (negative control, 8.5 m step
    with U11 −4, fails on a8370b3b). Replay vs car branch (`ab7.py`): lost 0, new measured 0, 612
    unmeasured flips (30 lead); withdrawn vRel over-closed 29.2 % but also under-closed 18 % (ref
    8.6 %). J1 (clear `samples`) rejected: 16 % over-close on 106 new measured sweeps, 65 lost. Next
    drive: lead lock census, brake events near joins, lead rejoin-hold duration.

30. **Route `0000023f--66ddbb900a` analysed (2026-09-17): first road evidence for
    `FarLeadBrakeLimit`, and the radar/vision gap discriminator is refuted on 8 routes.** 32
    segments, 31:41, 21.3 min engaged, fetched with `konik_fetch.py` and cached in `oprad-routes`.
    The device ran commit `7106fdf4` with `FarLeadBrakeLimit = 1`, `BoschARadar = 1`,
    `RangeDerivedVrel = 1`, `BlotV3 = 0` (from initData — the toggles were actually stored).

    * **`FarLeadBrakeLimit` fired on the road for the first time: 43 caps in 5 clusters**
      (`an2/farev.py`). One cluster is the phantom the feature exists for (12:25, d 46 m, U11
      −2.31 m/s while the next-1 s range slope is **+0.75**, relief 1.11 m/s²). **The other four
      capped leads that were genuinely closing** (future slope −4.0 to −10.0 m/s): 18:10 (relief
      0.05), 20:41 (relief **2.82** at d 83 m, min gap 78.5 m, min TTC 10.7 s), 23:15 (relief 1.28
      at d 57 m, closing −9.97 m/s, min gap 39.1 m, min TTC 4.6 s), 25:51 (relief 0.38). No driver
      brake followed any cluster, min gap over the route stayed 10.4 m and min TTC 2.7 s, and route
      `aEgo` min is −5.90 m/s² with 4 brake presses while enabled, none within 6 s of a cap. So on
      one drive the cap was 1-for-5 on phantoms and never left the car short — but it is bounding
      real closing leads, and 23:15 is the shape to watch. Still default OFF, still a TEST label,
      ramp-anchor defect still pinned, D-048 validation gate still unmet.
    * **D-048 gate item 2 / D-049 item 3, the measurement that was blocking them, is done for the
      absolute-gap candidate** (`an2/visgap.py`, 8 routes, 121 min, 140k radar-lead frames with a
      confident model lead). At d >= 60 m the radar/vision range gap is >= 10 m on 31.5 % and
      >= 15 m on 13.1 % of frames, and those frames are *not* followed by harder braking than
      baseline (lower on 6 of 7 routes). The 15.8 m gap in the 231 fault is therefore ordinary. A
      range-*slope* disagreement variant is refuted too: p90 of normal driving at 60–80 m is
      5.1–9.5 m/s and the fault's worst frame is 7.9 m/s. Written into D-048's rejected list. This
      does **not** clear the gates: it answers the distribution question in the negative, and the
      unreproduced statistics still need the harness re-run.
    * **D-059 out-of-sample on this unseen route** (`an2/ab7.py`): lost 0, lost_lead 0, new measured
      0, 108 unmeasured flips (16 lead); withdrawn measured vRel over-closed the next-1 s slope on
      19/67 (28 %) vs reference 33/702 (4.7 %), and under-closed on 21/67 vs 83/702 — the same
      shape, including the same caveat, as the 7 routes it was built on.
    * **Join census now 8 routes** (`an2/joinsum.py`): 249 joins, 13 on the lead, post-join measured
      vRel over-closes on 20.4 % (1,084 scored) vs reference 4.6 % (5,442). 39 joins and 2 D-057
      re-anchors on 23f alone.
    * Route 231 segments 9–11 are cached now too, so the fault episode can be re-derived against the
      current tree (the other agent's point 4: the 0.25 m/sweep vs 2.0 m gate arithmetic predates
      D-054 and has **not** been re-run yet).

31. **Route `00000241--7948e97423` (2026-09-17): the user reports phantom brakes and one stopped-lead
    miss they had to intervene on. Both are road evidence, and the lateral estimate is implicated in
    both directions.** 12 segments, 11:41, 5.1 min engaged, 4.5 km. Same device build as 23f
    (`FarLeadBrakeLimit = 1`, `BoschARadar = 1`, `RangeDerivedVrel = 1`, `BlotV3 = 0`). 5 brake
    presses and 12 `longActive` drops while enabled; route `aEgo` min −4.85.

    **The stopped-lead misses — three separate shapes, and our parser caused none of them.**
    * **9:36–9:40, the radar itself went blank.** `an2/rawslots.py` on segment 9: at 9:36.1 the
      sweep contains **no valid objects at all** (16 invalid slots) and the near lead (track 20,
      d 25.6 m, existence 68) is simply gone; it does not come back for ~3.7 s. The model lead's
      probability decays 0.93 → 0.62 → 0.38 → 0.06 with `xStd` 8 m, so `radarState` has no lead
      either. The car, just out of a near-stop, was commanding **+1.67 m/s²** into a lead that had
      stopped ~20 m ahead when the driver braked at 9:37.7. Nothing in the Bosch-A gate is involved:
      the radar published nothing to gate.
    * **6:50–6:52, lead selection walked away from a good in-lane point.** The raw sweeps keep
      track 8 at d ≈ 21 m, y ≈ 0.2, existence **126**, published every sweep. `leadOne` nonetheless
      drops it at 6:50.5 (`rad=0 tid=-1 d=27.2`), follows the model lead out to 32 m, then binds to
      track 33 at **43 m, y 4.2**. The genuinely stopping car (track 33) had been visible since 6:49
      at d 48 m with U11 −6.6 m/s, but its `y` read 5–6 m — out of lane — until 1.5 s before the
      intervention. Driver braked at 6:52.1.
    * **3:37–3:47, enough braking, then released.** Lead track 38, probability 1.00, both sensors
      agreeing, a slow roller (`vLead` 1.6–3.4 m/s) approached from 105 m. `FarLeadBrakeLimit` fired
      6 times at 3:37 (planner −2.61 → commanded −1.34), the car then braked to −3.32 at 3:39 and
      **released to −0.80 … −1.24 m/s² for the next 7 s** while still closing 5–8 m/s inside 60 m
      (`aLeadK` had gone positive). Driver braked at 3:47 with 13.9 m left.

    **The phantom brakes are the same coin.** `an2/phantom.py` lists all 8 engaged episodes at
    `aTarget <= -1.5`: **4:55.9** (−2.32, lead **y −8.1 m**), **4:59.4** (−3.24, aEgo −4.15, y −5.3),
    **9:09.5** (−3.20, aEgo −4.25, y +3.7, closing only 2.0 m/s with a −1.27 m/s future slope) and
    **1:51.9** (−2.86, aEgo −3.51, **no radar lead published at all** — cause not yet identified).
    So the car brakes hard for targets 4–8 m off centre while ignoring an in-path stopped car whose
    `y` reads 5 m. The Bosch-A lateral estimate is being trusted symmetrically in both directions,
    and `leadOne` accepted a lead at y −8.1 m.

    **Answered in item 38 (2026-09-21):** the census ran on all 13 routes and found 5 near in-lane
    dropouts on 4 routes — but four of the five lose the target at the lateral edge of the in-lane
    box, so the evidence points at the target leaving the beam rather than at a failing sensor. The
    "16 invalid slots for 3.7 s" premise is also corrected there: the blank condition lasted 0.14 s.
    Original note: whether a stationary-target dropout like 9:36 is visible in the other 9 routes.
    The 1:51.9 no-lead brake and the y −8.1 m lead acceptance are both answered in item 32. `FarLeadBrakeLimit`'s
    road record is now 2 routes: 1 phantom cap and **6 caps on genuinely closing leads**, two of them
    (23f 23:15, 241 3:37) followed by a driver brake. That argues for tightening its gate before it
    ever leaves TEST, not for widening it.

## 32. Route 241 follow-up: the 1:51.9 brake is not a radar event, and there is no absolute lateral bound on a Bosch-A lead — 2026-09-17

Two of the three items left open by item 31 are now answered. Replay and log analysis only; nothing
changed in the tree.

**1:51.9 was the end-to-end longitudinal plan, with no lead of any kind.** Route `00000241--7948e97423`,
seg 1. The raw sweeps carry no object anywhere near the path (`rawslots.py … 1 106 118 1.0`: the only
valid slots are track 23 at d 6–20 m, y −2.5 to −3.2, and from 1:52.0 to 1:55.0 *no valid objects at
all*), the model lead probability is **0.00–0.03** for the whole episode, and `hasLead = 0`,
`shouldStop = 0`, `src = cruise` in `scan_00000241--7948e97423.csv` throughout. `expMode = 1`
(Experimental Mode) the entire time, so longitudinal came from the model's own plan.

What moved was the plan, not a target. Between 1:49.8 and 1:52.6 the last point of `modelV2.position.x`
collapsed from **210 m to 99 m**, then to 69 m at 1:54 and 41 m at 1:57, while `steeringAngleDeg` held
−1.1° and yaw rate 0.007 rad/s — i.e. a straight, clear road. `spVCruise` stepped from 22.36 m/s to
14.06 at 1:50.4 and decayed to 5.09 m/s by 1:59.6, where `longActive` went 1 → 0 as the driver pressed
the gas. So the model planned a stop on an empty straight, the planner followed it down from 22.1 to
5.6 m/s, and the driver overrode.

> **Time-base note.** `scan_<route>.csv`'s `t` is raw `logMonoTime`; route time is
> `(t - min(seg_t0))/1e9` from the JSON, which is what `load.py`, `tl2.py`, `rawslots.py`, `phantom.py`
> and `curv.py` all use. An earlier revision of this item quoted the `spVCruise` times from a hand
> dump that normalised against the CSV's own first row instead, which on this route runs **5.13 s
> late**. The figures above are corrected; the plan-collapse, no-lead and no-object findings came from
> the correctly-based tools and are unaffected.

**This one is outside the Bosch-A path entirely.** No change to `radar_interface.py` could have caused
or prevented it, and it should not be counted against the radar work. It is the clearest phantom the
driver felt on this route, and its cause is the e2e longitudinal plan in Experimental Mode.

**Why `radard` accepted a lead at y −8.1 m: there is no absolute lateral bound on this car.**
`selfdrive/controls/radard.py`:

* The only absolute |`yRel`| gate in the lead path is `g90_radar_lead_lateral_sane` (line 588),
  `abs(yRel) <= min(6.0, 1.5 + 0.08 * dRel)`. At d 45 m that is 5.1 m, which **would** have rejected
  the 4:55.9 track. But it is applied only when `g90_radar_filter` is set, and line 1066 sets that to
  `CP.brand == "hyundai" and CP.carFingerprint == "GENESIS_G90"`. On this Honda it is off.
* The only lateral test that does run is `lat_sane` inside `track_matches_vision` (line 621):
  `abs(track.yRel + lead.y[0]) < max(y_floor, y_std_scale * max(lead.yStd[0], 0.2))`. That is a
  *relative* check — the radar's `y` only has to agree with **vision's** `y`. With `y_std_scale = 1.0,
  y_floor = 1.0` on the strict path and `2.0 / 1.5` on the preferred-track fallback (line 657), and
  vision `yStd` running 1.8–4.8 m at 40–100 m on this route, the tolerance reaches ~10 m. A radar
  track at y −8.1 passes whenever vision's own lead `y` is also off-centre and uncertain, which is
  exactly the case on a bend.

**Do not turn this into a threshold yet.** The same route contains the symmetric failure: at 6:49 the
car that really was stopping read `y` 5–6 m for 3 s (item 31), so an absolute |`yRel`| bound tight
enough to reject the 4:55.9 phantom would also have rejected that real lead and delayed the brake
further. The Bosch-A lateral estimate is too coarse to gate on in *either* direction on one route's
evidence, and D-048's rejected list is already full of thresholds that looked good on one route. The
next step is a measurement, not a constant: a census of published `yRel` against the vision `y` and
against the outcome, over all 10 cached routes, before any bound is proposed.

**Still open from items 31 and 32:** the stationary-target dropout census (does a 9:36-style
all-slots-invalid gap occur on the other 9 routes?), the `yRel` census above, and item 22.6
(236 track 21).

## 33. The 13-route lateral census: adjacent-lane locks are real, rare, and they brake — 2026-09-21

**Tool:** `an2/ycen.py` writes one row per `radarState` frame; `an2/ysum.py` buckets it. The measured
quantity is **`dyPath` = the lead's lateral offset against `modelV2.position` interpolated at the
lead's own range**, not against the car's centreline:

    y1 = -leadOne.yRel            # world frame, left-positive
    dyPath = y1 - pathY(d1)       # blank, never extrapolated, beyond position.x[-1]

`dyPath` is ~0 for an in-lane lead at *any* curvature and ~±3.7 for a genuine next-lane lock. Raw
`|yRel|` cannot separate those on a bend, which is why item 32 declined to bound it.

> **Retraction.** An earlier hand analysis in this session claimed `dyPath` was ~0 for all seven of
> 241's known episodes, and concluded a lateral gate was therefore not the fix. **That was wrong**,
> and wrong for the same reason item 32 was: the hand dump was read at the CSV's own time base, so it
> sampled windows several seconds off the episodes. Re-read at the correct base, 241 at 4:55.9–5:02.7
> shows `dyPath` growing smoothly 0 → **−4.2 m** while vision's own lead stays on path
> (`dyPath_ml` +0.08). The census below replaces that claim.

**Coverage:** 13 routes, 231/232/236/237/239/23a/23b/23e/23f/241/245/246/248. 248 is 8 of 12
segments — **3, 5, 7 and 8 never uploaded**, so its 2170 accepted frames are a partial sample.

**Distribution — lead selection is overwhelmingly in-path, as it should be:** `|dyPath| < 1 m` covers
**88.5–99.0%** of accepted-lead frames on every route. The measure is well behaved.

**Braking rises with off-path offset.** Share of accepted-lead frames whose next 3 s contains an
`aTarget < -1.5 m/s²`, by `|dyPath|` band:

| route | <1 | 1–2.5 | 2.5–5 | >5 |
|---|---|---|---|---|
| 232 | 4.9% | 21.5% | **33.8%** | 25.0% |
| 237 | 7.6% | 26.3% | **29.0%** | 23.5% |
| 245 | 7.9% | 33.7% | **27.7%** | 70.0% |
| 239 | 5.1% | 20.0% | 25.3% | 2.8% |
| 236 | 6.5% | 14.1% | 13.2% | 58.3% |
| 23f | 9.0% | 18.9% | 5.0% | 45.5% |
| 241 | 12.4% | 11.8% | 8.1% | 32.6% |
| 23b | 9.1% | 25.6% | 1.3% | 0.0% |

Nine of thirteen routes show a monotone rise from the `<1` band into `1–2.5`; a 3–5× jump is typical.
241 and 23b do not, so this is an association across the fleet, not a law. **`>5 m` is contaminated**
by junction geometry (ego turning across its own plan, `|curv|` 0.02–0.14, ranges under 20 m) and is
not the interesting band. The band that matters is **2.5–5 m at 30–115 m on a gentle bend.**

**The sharpest case, 245 at 5:52.8–5:58.5** (`ycen_00000245--1356bb0355.csv`), v≈20 m/s, gentle right
bend `curv ≈ -0.0027` (R≈370 m). Radar tracks 26, 32 and 35 are bound as `leadOne` at d = 75–108 m
with `dyPath` **+2.9 to +5.2** — left of the ego's own plan — while vision's lead sits on the plan at
d≈75 m (`dyPath_ml` −0.5…−1.3, `mlProb` 0.35–0.77). `aTarget` reaches the **−3.50 m/s² floor** with
`src = lead0`/`lead1` twice, at 5:53.5 and 5:56.5. 245's `1–2.5` band brakes in 33.7% of frames
against a 7.9% baseline.

Comparable radar-backed, vision-on-path episodes: 237 at 6837–6843 s (d 82–95 m, `dyPath` +2.5…+3.7,
`aTarget` −2.00) and 6888.5 s (d 74 m, −3.50); 236 at 3461–3466 s (d 86–112 m, −1.58); 23e at
23110.5 s and 23376.1 s (both −3.50); 241 at 4:55.9–5:02.7 (d 45–49 m).

**Observation to verify, not yet a finding:** across those 245 frames `leadOne.vRel` reads exactly
**−13.50 m/s** on four different track ids for dozens of consecutive frames. On track 29 the range
slope confirms it (86.3 → 65.5 m in 1.4 s ≈ −14.8 m/s), so it is not simply a clamp — but an
identical constant across distinct tracks needs explaining before anyone reasons from it. Check it
against `rawslots.py` before using `vRel` in any gate.

**What this does and does not license.** It is replay evidence that a lateral disagreement measure
would fire on the frames the driver complains about, and item 32's counter-example is answered:
241's 6:49 real missed stopper reads `dyPath` −0.50…+0.62, so a `dyPath` bound keeps it where a raw
`|yRel|` bound would have dropped it — safe in the D-041 direction. It is **not** a validated gate.
Nothing here has been replayed through `radard` with a candidate threshold, and the 2.5–5 m band
still holds 0.6–3.4% of accepted frames on routes where the association is weak. Before proposing
anything: replay a candidate bound over all 13 routes and count leads *lost*, not leads rejected.

**Still open:** the longitudinal over-reaction from item 32 (−3 m/s² for a 2.0–2.3 m/s closure at
42–45 m) is unexplained and is a separate defect from lead selection; the stationary-target dropout
census; item 22.6 (236 track 21). *(Item 34 resolves the first of those: it is not a defect.)*

## 34. There is no longitudinal over-reaction: the −3.50 floor is correct arithmetic on a railed off-path lead — 2026-09-21

Item 33 left two things open and called them separate: an unexplained `vRel` constant, and an
unexplained longitudinal over-reaction. They are the same thing, and neither is a longitudinal
defect. **The MPC's response is arithmetically correct for the input it was handed. The input is
wrong twice over, and both errors push the same way.**

**The `−13.50 m/s` constant is the Bosch-A saturation rail, already documented in this repo.**
`opendbc_repo/opendbc/car/honda/radar_interface.py:133` states it outright: the U11 domain endpoints
are saturation rails, "at raw 0 or 1728 the true |vRel| is >= 13.5 m/s", and line 145 names the
consequence — "a stopped car reads as vLead = vEgo − 13.5". Four distinct track ids reading exactly
the same value is the rail, not a coincidence and not a clamp bug. Item 33's `vRel` observation is
resolved; nothing new needs measuring. Cross-checking `rawslots.py` is no longer required.

**Why that is expensive: the rail makes any off-path radar return look like a near-stopped car.**
`long_mpc.py` builds the obstacle as `dRel + get_stopped_equivalence_factor(vLead)` =
`dRel + vLead²/(2·2.5)`, and compares it against
`get_safe_obstacle_distance(vEgo, t_follow)` = `vEgo²/5 + 1.45·vEgo + 6`. At the moment the rail
holds, `vLead` reads `vEgo − 13.5` whatever the object is actually doing, so its stopped-equivalence
contribution nearly vanishes. Worked from the 245 CSV rows, with `ACCEL_MIN = −3.5`:

| frame (route time) | tid | vEgo | dRel | vLead published | obstacle | safe distance | deficit |
|---|---|---|---|---|---|---|---|
| 353.6 s | 26 | 20.72 | 89.79 | 7.22 (rail) | 100.2 | 121.9 | **−21.7 m** |
| 356.5 s | 35 | 19.55 | 77.92 | 6.05 (rail) | 85.2 | 110.8 | **−25.5 m** |
| 357.4 s | 29 | 19.35 | 77.92 | 5.85 (rail) | 84.8 | 108.9 | **−24.2 m** |
| 354.4 s | vision lead, same scene | 20.18 | 74.04 | 16.68 | 129.7 | 116.7 | **+13.0 m** |

The last row is the finding. At the same instant, on the same road, the vision lead carries **+13 m
of margin** — the planner wants no brake at all — while the railed radar track 4 m off the predicted
path reads **−25 m** and the solver goes to the floor. −3.50 m/s² is not an over-reaction to that
input; it is the only answer to "a near-stopped car 78 m ahead at 19.5 m/s". A deficit of −20 to −26 m
is not marginal, which is why these events hit `ACCEL_MIN` exactly rather than something in between.

**The reach of the mechanism, from the same two formulas.** A railed radar lead triggers braking
below `safe(vEgo) − (vEgo−13.5)²/5`:

| vEgo (m/s) | 13.5 | 16.0 | 19.5 | 22.0 | 25.0 | 30.0 |
|---|---|---|---|---|---|---|
| brake-triggering range | 62 m | 79 m | 103 m | 120 m | 141 m | 175 m |

So at highway speed *any* accepted radar lead inside ~120–175 m with U11 on its rail commands heavy
braking. This is what makes lateral mis-binding so costly: the rail guarantees the bogus lead looks
nearly stopped, and the range band where that matters is exactly the 30–115 m band item 33 measured.

**The rail correction cannot help here, by design.** `radard.py`'s range-derived `vRel` assist
(D-053, default OFF) is **one-sided** — it may only make the published closing speed *more* closing,
never less — and `RANGE_VREL_ASSIST_MAX_ABS_Y_REL_M = 1.5` excludes adjacent-lane tracks outright.
Both properties are correct for what it was built for (D-041, recovering closing the rail hides) and
both mean it is irrelevant to a false brake. Nothing in the longitudinal stack is mistuned here.

**What the track geometry says about these objects.** Per-track range slope over the contiguous run,
against ego speed (radar frames only, duplicate ranges dropped):

- tid 26, 352.8–353.9 s: 108.3 → 79.3 m, slope **−26.3** vs vEgo 20.8 → ratio **1.26**. Closing
  *faster* than ego travels, so not a stationary object: either an oncoming vehicle (`dyPath` +3.9…
  +5.2, i.e. one to one-and-a-half lanes left) or a track whose range is drifting across objects.
- tid 35, 355.7–356.9 s: 93.0 → 72.9 m, slope **−16.8** vs 19.6 → ratio **0.86**. Apparent object
  speed +2.7 m/s: near-stationary, `dyPath` +2.9…+3.8 — adjacent lane or roadside.
- tid 29, 356.9–358.9 s: 86.3 → 57.8 m, slope **−14.2** vs 19.4 → ratio **0.73**. Apparent speed
  +5.1 m/s, and `dyPath` ≈ **0** — this one is *in* the path. Its brake is defensible, and its true
  closing (−14.2) slightly exceeds the rail, so here the rail under-reads rather than invents.

tid 29 matters as a negative control: not every railed track in the window is a lane lock, and the
one that is on-path is one the car should brake for. Only tids 26/32/35 are the complaint.

**Consequence for the roadmap.** Item 33's "still open: the longitudinal over-reaction" is
**withdrawn** — there is no such defect to chase, on either of its two cases. Item 32's 241 case
(−2.3…−3.2 m/s² for a 2.0–2.3 m/s closure at 42–45 m) computes to a deficit of −8.3 m at vEgo 20 and
is likewise the tune answering honestly: the comfort-brake asymmetry alone demands ~50 m behind a
same-speed lead at 20 m/s, so 43 m is genuinely short by the MPC's own model. Disagreeing with that
is a `COMFORT_BRAKE`/`T_FOLLOW` argument, not a bug report.

All of the remaining leverage on the user's reported symptom is therefore **in lead selection**, which
is where item 33 already pointed. Static/replay evidence only; nothing here was driven.

**Still open:** replay a candidate `dyPath` bound over all 13 routes and score leads *lost*, not
leads rejected (D-041) — 241's 6:49 real stopper at `dyPath` −0.50…+0.62 must survive, and so must
245 tid 29; the stationary-target dropout census; item 22.6 (236 track 21).
*(Item 35 does the bound scoring: 3 of 113 hard brakes touched, and 245 tid 29 sets a 2.5 m floor.)*

## 35. A `dyPath` bound scored by leads *lost*: 3 of 113 hard brakes touched, and 2 of them are the reported symptom — 2026-09-21

Item 34 concluded that all remaining leverage on the reported adjacent-lane lock-on is in lead
selection. This scores the candidate gate the census pointed at, on the metric D-041 demands: not
how many leads it rejects, but **how many frames it would leave with no lead at all.**

**The gate, stated exactly as measured.** Reject the radar-backed lead when `|dyPath| >= 2.5` **and**
vision has its own lead that is *on* the predicted path (`|dyPath_ml| < 1.5` and `mlProb >= 0.5`).
The second clause is the whole safety argument and is not decoration: the gate is only permitted to
fire when a replacement lead demonstrably already exists, so it degrades which object is followed
rather than deleting a point. Where vision disagrees, nothing is rejected.

**Cost, measured over all 13 routes** (`ybound.py`, host Python over the `ycen_*.csv` frames):

| `|dyPath|` bound | frames fired (fallback exists) | frames left alone (no fallback) |
|---|---|---|
| 2.0 | 773 | 617 |
| **2.5** | **449** | **398** |
| 3.0 | 252 | 246 |
| 3.5 | 125 | 180 |

Roughly **half of all off-path radar leads have no on-path vision lead to fall back to**, and at every
bound the gate leaves those untouched. That is the D-041 half of the result, and it is the reason to
require the vision clause rather than gate on `dyPath` alone.

**The number that matters: hard-brake episodes touched.** Classifying every episode with
`aTarget < −2.5` fleet-wide against the 2.5 m bound:

| route | episodes | kept | gated |
|---|---|---|---|
| all 13 routes | **113** | **110** | **3** |

Only **2.7% of hard-brake episodes** are touched at all, and the three are:

- **245 @ 353.5 s** — 8 of 12 frames, `|dyPath|` up to 5.21 at 81 m, floor −3.50.
- **245 @ 356.3 s** — 11 of 33 frames, `|dyPath|` up to 3.44 at 73 m, floor −3.50.
- **23e @ 23109.7 s** — **1 of 48 frames** (`dyPath` +6.09). Not a lane lock. Direct inspection shows
  the ego yawing hard (`curv` → −0.005) during a radar→vision lead handoff: track 25's `dyPath` walks
  +0.2 → +7.75 over 1.5 s while `dyPath_ml` stays under 0.8, then `radar1` goes to 0 and vision holds
  the lead at 15–25 m with a real −3.5 m/s closing rate. **The brake proceeds regardless**; the gate
  would only bring forward a handoff that happened on its own one frame later. This also confirms
  item 33's reading of the `>5 m` band as ego-yaw/junction contamination, now by inspection and not
  just by bucket share.

So the two episodes the gate actually suppresses are **exactly the event the driver reported** — the
adjacent-lane lock on the curve at 352.8–358.5 s on 245, which item 34 showed reaches the `ACCEL_MIN`
floor honestly because the Bosch-A rail makes an off-path return look nearly stopped.

**Both negative controls survive, and one of them sets the bound.**

- **241, every hard brake.** All 8 episodes (128 frames) sit at `|dyPath| < 1.1` except one at
  502.7 s, which is `radar1 == 0` — vision-sourced, so a radar-only gate cannot reach it. Every real
  241 deceleration survives any bound ≥ 2.5. (Correction to item 34's framing: the 6:49 frames carry
  `aTarget` ≈ +0.09, so that window is not itself a hard brake; the route's real stoppers are at
  257.4, 338.1, 356.0, 588.4 and 606.2 s. The conclusion is unchanged and now rests on all of them.)
- **245 tid 29** (near-stationary, in the path, brake correct — item 34's control) peaks at
  `|dyPath| = 2.22` at 358.59 s. **It is gated at a 2.0 m bound and survives at 2.5 m.** This is an
  empirical floor, not a round number: **do not set the bound below 2.5 m.**

**Method limits, stated plainly.** This is a frame-level scoring of `radarState` output, not a
`radard` replay — it measures which frames a gate would reject and what fallback existed at that
instant, not the closed-loop trajectory that would result. Two of the three gated episodes are
*partially* gated (8/12 and 11/33 frames), so the honest claim is that the brake would be reduced or
delayed, not that it would be cleanly prevented. A closed-loop replay is still the thing that would
settle the magnitude. Static/replay evidence only; nothing here was driven.

**Still open:** the closed-loop `radard` replay to get the trajectory effect rather than the frame
counts; the stationary-target dropout census; item 22.6 (236 track 21).

## 36. The closed-loop `radard` replay: the gate never deletes a lead, and the times in items 33–35 were wrong — 2026-09-21

Item 35 scored the 2.5 m `dyPath` gate at **frame level** — which published `radarState.leadOne`
frames a gate *would* reject, and whether vision offered an on-path replacement at that instant. It
could not say what lead would actually be published instead, so two of its three gated hard-brake
episodes came out only *partially* gated (8 of 12 frames, 11 of 33) and "the brake is delayed" was
indistinguishable from "the brake is prevented". This item runs the selection.

**Tool:** `/routes/an2/yrep.py` + `yrepsum.py`. Two `RadarD` instances are driven off the same rlog
stream, cycle for cycle, from the logged `liveTracks` / `carState` / `modelV2` / `starpilotPlan`. A is
untouched. B applies the frozen gate and then **re-runs `get_lead` with the offending radar track
removed from the candidate set**, repeating while the winner keeps tripping the gate;
`prev_lead_track_ids` is then set from B's *own* choice, so the Bosch-A preferred-track hysteresis
follows the replacement on every later cycle instead of re-preferring the track it just rejected.
Track Kalman states do not depend on selection, so they stay shared by construction.

**Validity first.** Restricted to cycles where the log and the baseline replay both published a
radar-backed lead, the baseline picks the **same `trackId` on 99.86–100.00% of cycles on all 13
routes** (100.00% on six of them). Status agreement is 99.35–100.00%. The harness reproduces the
drive; without that nothing below would mean anything.

**Fleet result, 310,403 cycles.** The gate fires on **163** cycles. The published lead differs on
**559** cycles — **0.180%** — across **49 divergence episodes**. The fire count is much smaller than
the divergence count because the gate fires *once* to break a lock and the hysteresis then holds the
replacement; frame-level scoring could not see that, and it is why item 35's partial-gating worry was
the wrong worry.

**The D-041 question is now answered by observation, not by argument.** At the start of the 49
episodes the replacement lead is:

| replacement | episodes |
|---|---|
| the **vision** lead | 48 |
| another radar track | 1 |
| **no lead at all** | **0** |

The gate never once deleted a lead. It is a change of *which* object is followed, in every episode
the fleet contains. That also locates the residual risk exactly: the gate's safety rests entirely on
vision's own lead being right, because vision is what it always falls back to.

**Three episodes contain a hard brake** (`aTarget < −2.5`), independently reproducing item 35's
"3 of 113" by a completely different method:

- **245 @236.6 s** — A follows tid 26 at **101.7 m, `vRel` −13.50** (the U11 saturation rail),
  `dyPath` +4.17. B follows vision at **74.3 m, `vRel` −7.34**, `dyPath` −0.98.
- **245 @239.3 s** — A follows tid 35 at **93.0 m, `vRel` −13.50**, `dyPath` +3.53. B follows vision
  at **84.8 m, `vRel` −3.64**, `dyPath` −0.52.
- **23e @1787.0 s** — A follows tid 25 at 36.0 m, `vRel` −2.30, `dyPath` +2.77. B follows vision at
  **33.4 m, `vRel` −4.51** — *closer, and closing faster.* The gate would make this brake **harder,
  not softer**. Item 35 called this outlier a single frame of 48 and said the brake proceeds
  regardless; closed-loop it is 11 cycles, and the brake does not merely proceed, it strengthens.

On the two 245 episodes B swaps a **railed** lead (`vRel` pinned at the rail, so `long_mpc` loses its
stopped-equivalence term — item 34) for an unrailed one with a real closing rate. That is the
mechanism by which the reported symptom would go away. **The magnitude is still not measured:** that
needs the two lead streams pushed through the MPC, which is the next step and is now unblocked
(`LongitudinalMpc` instantiates in the analysis image).

### A correction that affects every time cited in items 33, 34 and 35

**`ycen.py` writes raw `logMonoTime`, not route-relative ns.** The checkpoint invariant that said
otherwise was wrong, and `ysum.py` / `ybound.py` divide that column by 1e9 with no `T0` subtraction,
so **every time printed by items 33–35 is high by that route's `min(seg_t0)`**:

| route | `T0` (s) | route | `T0` (s) | route | `T0` (s) |
|---|---|---|---|---|---|
| 231 | 77919.70 | 237 | 5901.74 | 23f | 10567.81 |
| 232 | 17367.82 | 239 | 11865.87 | 241 | **38.63** |
| 236 | 3119.15 | 23a | 15469.67 | 245 | **116.46** |
| 23b | 20796.82 | 23e | 21323.47 | 246 | 1641.77 |
| 248 | 31.76 | | | | |

So item 35's "245 @353.5 s / @356.3 s" are route **236.6 s / 239.3 s**, its "23e @23109.7 s" is route
**1787.0 s**, its "245 tid 29 @358.59 s" is route **242.1 s**, and item 33's "245 at 352.8–358.5 s" is
route **236.3–242.0 s**. This replay, which subtracts `T0`, lands on exactly those corrected times —
the two harnesses agree and only the labels were wrong. **No physics, count or conclusion in items
33–35 changes**, because each was computed in one internally consistent base. This is the third time
the time base has cost a correction.

**One control was genuinely measured in the wrong place.** `ybound.py` pinned the driver-reported 241
6:49 window at `t` 409.0–410.5, which on that route is route **370.4–371.9 s — about 6:10, 39 s
early**. Re-examined at the correct route 400–420 s: `vEgo` 1.1–8.5 m/s in slow traffic, min `aTarget`
−0.84, **max `|dyPath|` 0.86, and zero frames the gate could even consider**. The 241 control passes
at the right window, and now passes *where the driver actually reported the event* — a stronger
result than item 35 had. Item 35's claim that the 6:49 window is not itself a hard brake also
survives, but it had been measured at the wrong window.

Two further cautions from the corrected read: from route 411.7 s that window has `aTarget` exactly
`0.0000`, and **56% of route 241's frames carry `longActive == 0`** — openpilot longitudinal is not
engaged, so `aTarget` there is not evidence about what openpilot would have done. Any `aTarget`-based
scoring on 241 must be read with that in mind.

Replay/static evidence only; nothing here was driven.

**Still open:** the MPC pass for the braking magnitude on the two 245 episodes; the stationary-target
dropout census; item 22.6 (236 track 21).

## 37. The planner-level pass: the gate belongs on both lead slots, it is not mergeable, and `FarLeadBrakeLimit` is structurally inert — 2026-09-21

This scores the `dyPath` gate one level higher than item 36: **through the real
`LongitudinalPlanner`**, not at `radarState`. `$T/an2/ympc.py` (in the `oprad-routes` volume at
`/routes/an2/`) builds N planner instances off one rlog stream and swaps only `radarState` between
them, so every variant sees identical frames, identical `carState`, identical `modelV2`. Nothing
longitudinal is reimplemented — it is the shipped planner, including arbitration, `t_follow`,
`jerk_scale` and the BLoTv3 supervisor. Variants: `A` baseline, `B` gate on `leadOne`, `C` gate on
both slots, `Apre`/`Bpre` the pre-`084a9d56` BLoTv3 feedback read, `Aoff` BLoTv3 off, `Aflbl`
`FarLeadBrakeLimit` forced on.

**This corrects item 36's expectation.** Item 36 predicted the two 245 hard brakes would "soften a
lot" once the gate ran closed-loop. Measured, they do not: every variant still reaches min `aTarget`
−3.50, and the `leadOne` gate only cuts time below −2.5 from 2.30 s to 2.00 s.

**Why: `radarState` has two published lead slots and items 35–36 scored only one.**
`LongitudinalMpc.update` takes `min()` over `lead_0_obstacle` and `lead_1_obstacle`, and
`longitudinalPlanSource` at the 245 237.2 s peak reads **`lead1`** — the brake was sourced from
`radarState.leadTwo`, which the frozen gate never touches. The railed off-path track simply
reappears in the second slot. Gating both slots (variant C) on 245 234–243 s:

| variant | min `aTarget` | s below −2.5 | s below −1.5 |
|---|---|---|---|
| A baseline | −3.50 | 2.30 | 2.50 |
| B gate on `leadOne` | −3.50 | 2.00 | 2.45 |
| C gate on both slots | −3.50 | **1.00** | **1.10** |

57% less time in hard braking against 13%. It shortens the brake; it does not prevent it. The
residual is 245 tid 29 at `|dyPath|` 2.22, below the frozen 2.5 floor by design (item 35).

**And the gate is NOT mergeable, because the cost is now quantified.** On 23e 1783–1792 s the gate
makes the brake *worse*, and variant C is identical to B there (11 differing cycles, same numbers):

| variant | min `aTarget` | s below −2.5 |
|---|---|---|
| A baseline | −1.55 in the divergence run | 2.35 |
| B / C | **−3.50** | **2.90** |

Mechanism, from the run's own lead columns: the gate drops radar tid 25 at `dRel` 36.0,
`vRel` −2.30, and selection falls through to the **vision** lead at `dRel` 33.4, `vRel` −4.51 —
nearer and closing twice as fast. The gate fired on **1 cycle** and produced 11 divergent cycles,
because dropping the track also reset `prev_lead_track_ids` and handed the slot to vision for the
rest of the episode. This is D-041 measured rather than argued: deleting a radar point cost 2.32
m/s² of extra braking on an episode where the radar lead was the gentler answer. **Do not ship the
`dyPath` gate in this form.** What it needs is a replacement that is *bounded*, not a deletion — the
gate may only stand a lead down when the object taking its place is not more urgent.

**`FarLeadBrakeLimit` is structurally inert, and this is why it is removed in this commit.** Variant
`Aflbl` (cap forced ON) differs from `A` on **0 cycles in every window measured** — 245 234–243 s,
245 180–270 s (1800 cycles), 23e 1783–1792 s, 241 380–470 s (1798 cycles) — including both of the
worst far-lead hard brakes in the fleet, 90 m leads at −3.50. The cap never engaged once.
`get_far_lead_brake_limit` stands itself down when `ttc = dRel / closing < FAR_LEAD_BRAKE_LIMIT_MIN_TTC
= 10.0`; on the U11 saturation rail (item 34) `closing` is pinned at 13.5 m/s, so a 90 m lead reads
TTC 6.7 s. **The rail that causes the false brake is what makes the cap blind to it.** It can only
fire on leads whose `vRel` is not railed — genuine closers — which is the whole of its 1-good-vs-6-bad
record, and it cannot be tuned out: raising `MIN_TTC` past the rail's own TTC parks the cap directly
on top of real emergency braking. See D-060.

**James's `084a9d56` (BLoTv3 supervisor reads the MPC target, not the arbitrated output) is correct
but not exercised by any window measured.** `Apre` vs `A` = **0 differing cycles** on all four
windows above, 241 included. The fix only bites where the arbitrated output is much more negative
than the MPC's own lead solve (curve limiter, red light, force-decel), and none of these windows
contains that divergence. Report it as unexercised, **not** as ineffective. `Aoff` (BLoTv3 off)
differs on 97 cycles over 245's 90 s span, with BLoTv3 slightly *lengthening* hard braking
(2.70 s on vs 2.55 s off) and on 23e shortening it (2.15 s off vs 2.35 s on) — small, and not
the subject of this item.

**Toggle census from `initData`:** routes 245, 246 and 248 all ran `FarLeadBrakeLimit=0` and
`BlotV3=1`. The driver reports the last two drives did not over-react with the cap off. That is
limited road evidence consistent with the cap being inert, not with the cap having helped.

Replay/static evidence only; nothing in this item was driven.

**Still open:** a bounded (non-deleting) replacement for the `dyPath` gate on both slots — now
designed as **D-061 (proposed, not implemented)**, which bounds the obstacle in the planner instead
of gating selection, and whose first prerequisite is that **`dyPath` does not exist online at all**;
a window that actually exercises `084a9d56`; item 22.6 (236 track 21). **The stationary-target
dropout census is done — see item 38**, which also retracts three intermediate claims made while it
was being built.

## 38. The near in-lane target-dropout census: 5 events on 4 routes, and the evidence points at geometry, not a failing sensor — 2026-09-21

> Extended to 17 routes by **item 39**, which adds 248 (full), 249, 24b and 24d. The conclusion is unchanged.

Closes the "stationary-target dropout census" left open in items 31 and 37. Replay evidence only,
decoded at the CAN/wire level (pre-parser). Nothing here was driven.

### The premise was wrong, and correcting it changed the metric

The starting report was that on 241 at 9:36 *"the radar returned 16 invalid slots — completely
blank — for 3.7 seconds"*. Measured over all 894 sweeps of 241 segment 9, the longest all-16-invalid
run is **0.14 s** (2 sweeps). Only 8 of 894 sweeps have zero valid objects; the `n_valid` histogram
is `{0:8, 1:42, 2:480, 3:308, 4:47, 5:9}`. What lasted ~3.7 s was **track 20, the near in-lane lead,
being absent** while the radar kept reporting other objects — which is what item 31 always said.

**"All 16 slots invalid" is therefore the wrong metric.** On 241 the longest such run is 44.98 s, at
`longActive=False` and `vEgo` ~0 — a parked car. On an empty road the Bosch-A legitimately reports
nothing. The right metric is a **near in-lane target dropout**: an object with `d < 40 m`,
`|y| < 2.0 m`, existence `>= 32`, held `>= 0.5 s`, that then leaves the box, where the track id does
not reappear anywhere in the object list.

Every other number in the original report checks out: commanded `accel` 0.76 -> **1.67** -> 1.80 m/s²
with `longActive=True`, `leadOne` degrading to vision-only then `None`, model lead probability
0.95 -> 0.24, and driver intervention at 9:37.74. The slot was invalid at the wire level, so the
radar genuinely stopped reporting it — **our parser caused none of it.**

### Result: 13 routes, 264 segments, 5 events on 4 routes

```
route  route-time  dur     tid  d      y     exFade   drate  accel@loss  vEgo         mlprobMin
241     9:35.73   71.36s   20   26.0  -1.9   67/126   +1.6    +0.72      3.0 -> 14.2   0.00
241     4:06.92   13.77s   22   28.3  -1.4   68/68    +2.2    +0.78     13.0 -> 17.5   0.66
236    28:16.84   29.49s   37   11.8  -2.0  126/126   -1.4    +0.86     24.1 -> 30.3   0.84
23f     3:57.19    2.67s   24   15.4  -1.4   74/109   -5.3    +0.49      8.1 ->  9.5   0.21
248    10:33.39    3.50s    2   39.2  +0.5   78/126   -0.5    +0.17     12.0 -> 13.4   1.00
```
Zero on 231, 232, 237, 239, 23a, 23b, 23e, 245, 246.

### The finding: four of the five died at the edge of the box

`y` at loss is **−1.9, −1.4, −2.0, −1.4** against a box half-width of 2.0 m. On 241's track 20 the
azimuth is *still drifting outward* as the existence decays 126 -> 94 -> 68 -> 67. That is as
consistent with the target leaving the radar's beam as with the radar failing to hold it, and
nothing measured here separates the two.

The fifth, 248 10:33, is the only one lost near boresight (`y +0.5`) — and there the **camera still
held the lead at probability 1.00**, so no lead was lost at all.

**This census does not establish a radar dropout fault.** It establishes that near in-lane tracks
are lost at the lateral edge of the in-lane box while the sensor keeps reporting. The discriminating
observation — not yet found in any route — would be a **fade-then-die with the target centred**,
`|y|` small throughout while existence halves. Until that exists there is no signal to gate on, and
**D-041 forbids a gate that deletes a point on a suspicion this weak.** No code change is proposed.

### Four detector defects, each of which produced a confidently wrong answer first

Recorded because each one passed quietly and the census looked clean while wrong.

1. **Closing gaps at segment boundaries missed the event that prompted the census.** 241's 9:36
   dropout runs to the end of segment 9. `logMonoTime` is continuous across segments, so a real
   dropout may span one. Exclude recording holes with a sweep-to-sweep `MAX_DT = 2.0 s` check
   instead. *Any census that cannot find the event you already know about is not measuring it.*
2. **Geometry cannot classify an exit.** A `|y| >= 1.6 m` test labelled 241 track 20 a clean lateral
   exit, because its azimuth wandered to the edge as its return weakened. Use **track survival**.
   A "seen at all within 1.5 s" survival test *also* mislabelled it — it was in the object list for
   0.07 s more. Require the re-sighting at **0.75 s or later**. Ground truth from a sweep-by-sweep
   trace: track 20 held 235 consecutive sweeps, rt 560.09..575.80, no gap > 0.5 s, never returned.
3. **Engagement and acceleration were read from different moments.** `longActive` was an OR over the
   whole gap and peak accel a max over its first 5 s, so a row could report "engaged and
   accelerating" when the car was engaged at one instant and accelerating at another. On **23e
   23:46.08** it did: the driver disengaged at 23:43.68, the track died 2.5 s later with the car
   under manual control, and the +1.14 m/s² came from after a re-engagement at 23:48.03. That row
   was reported as a worse instance of 241 before this was caught; **it is not an openpilot event at
   all.** Record engagement and accel **at the moment of loss** and gate on that. Two rows demote
   (23e 23:46.08, 236 32:04.95, both `accel@loss = 0.00`).
4. **Oncoming traffic is not a dropout.** An oncoming car crosses the boresight, sweeps the in-lane
   box for ~1 s and leaves the near field at ~10 m, **dying at full existence with no fade** — the
   exact "no warning" signature. Two such crossings (232 10:05 track 1, 23e 13:38 track 9) were
   reported as a distinct fault population that a degradation-based gate would miss. Both are
   oncoming: range collapsing at ~29 m/s while `vEgo` was 14–16 m/s, `y` sweeping −4.6 -> +2.8.
   Nothing ahead can close faster than ego speed. Classify `d_rate < -(vEgo + 3)` as **oncoming**
   (15 such gaps fleet-wide), and compute `d_rate` over **one track id**, never over "nearest
   in-lane object", which switches tracks and yields rates no object had (−29.7, −30.9 m/s).

### Caveats on these numbers

- **The 248 row is from a partial route.** Only 8 of 23 segments were cached, and not contiguous
  (0,1,2,4,6,9,10,11). The full route arrived 2026-09-21 and has not been re-censused.
- Routes `00000249--d481c5de77`, `0000024b--02cabe206c` and `0000024d--f80e13b850` arrived the same
  day and are **not** in this census.
- `seg_t0` in `/routes/an2/scan_<route>.json` **cannot be used to locate a segment**: every entry
  reads the same value because each segment's first `logMonoTime` is `initData` (all 42 entries of
  23e read offset 0.0). The global `T0 = min(seg_t0)` is correct; segment index comes from
  `route_time // 60`.

## 39. Four more routes censused (248 full, 249, 24b, 24d): the geometry finding holds, and a separate range-rate latency shows up on 24d — 2026-09-21

Extends item 38 from 13 routes to **17 routes / 343 segments**.
> **Corrected by 39.5:** that is 17 route-*entries* over **16 routes / 335 segments** — 248 is
> counted twice, partial then full. The events and the conclusion are unaffected. Routes added, all fetched complete
into the `oprad-routes` volume and rescanned with `/routes/an2/scan.py`:

| route | segs | in-lane gaps | >=1.0s | dangerous |
|---|---|---|---|---|
| `00000248--4f275f0bb6` | 23 (was 8, non-contiguous) | 10 | 5 | **1** |
| `00000249--d481c5de77` | 6 | 19 | 9 | 0 (1 demoted, not engaged at loss) |
| `0000024b--02cabe206c` | 18 | 83 | 19 | **0** — all 19 classified `exited` |
| `0000024d--f80e13b850` | 32 | 57 | 33 | **1** |

**248's row is confirmed, not corrected.** Item 38 scored 248 on 8 of 23 segments. The full route
returns the *same single event* at 10:33.39 (tid 2, d=39.2, y=+0.5, exFade 78/126, camera held the
lead at mlprob 1.00). The partial-route row was right by luck, but it was right.

**The one new event, 24d 18:47.38**, fits the item 38 pattern and does not challenge it:
`tid=21 d=37.4 y=-2.0 ex=126 held=2.5s drate=+6.1 exFade=126/126 blank=0.0% mlprobMin=0.00`.
`y = -2.0` is exactly the edge of the +/-2.0 m box, and `drate = +6.1` means the target was
**receding at 6 m/s** while ego accelerated 14.5 -> 21.9 m/s. A receding target drifting off the
lane edge is a vehicle being left behind, not a sensor failure. Existence never faded (126/126).

**Fleet total: 6 events on 5 routes out of 17 routes. Five of the six died at |y| >= 1.4 m against
a 2.0 m box.** The discriminating observation named in item 38 -- a fade-then-die with the target
**centred** -- still does not occur anywhere in the fleet. Item 38's conclusion stands: no deletion
gate is justified on this evidence, and D-041 forbids one on evidence this weak.

### 39.1 A separate finding on 24d: the lead range rate lags the true range slope by ~1 s

This is NOT a dropout and is NOT part of the census. It came from investigating the reported
"strange early braking / it insists on keeping a certain distance with lead" on 24d.

24d has **23 braking episodes** with `longActive` and `aTarget < -1.0`; **ten of them pin at
aTarget ~ -3.5**, which is a rail, not a computed demand. Tracing the 5:18 episode against the
route CSV:

| route-time | d1 (m) | measured range slope | `leadOne.vRel` | `liveTracks` vRel (U11) |
|---|---|---|---|---|
| 5:17.14 | 54.2 | **-4.0** | +0.09 | +0.09 |
| 5:17.35 | 53.5 | **-6.1** | +0.02 | +0.02 |
| 5:17.55 | 52.5 | -6.4 | -3.42 | -0.20 |
| 5:18.15 | 49.0 | -6.6 | -5.44 | -1.36 |
| 5:18.35 | 47.8 | -5.7 | **-5.77** | -2.09 |
| 5:19.14 | 43.7 | -5.3 | -3.48 | -3.48 |

Two things, in order of confidence:

1. **Both estimators read ~0.0 m/s while the range was already closing at 4-6 m/s**, for roughly
   one second (5:17.1 -> 5:17.5). The radar did not report the lead's deceleration until the gap
   had already closed ~8 m. When `vRel` finally catches up, the planner has to make up the deficit
   at once -- which is why the demand goes straight to the -3.5 rail and holds it 1.5 s. The brake
   is **late in detection and therefore hard**, not early. From the seat it reads as "early"
   because there is still 47 m of gap.
2. **`leadOne.vRel` converges much faster than the raw U11 track vRel** (error ~1.1-1.5 m/s vs
   ~4.7-6.2 m/s through the transient). An earlier reading of this table had it backwards -- that
   our estimate was overshooting and driving the brake. It is not: checked against the measured
   range slope, ours is the closer of the two and the native U11 lags ~1.5 s. Recorded because the
   wrong version is the intuitive one.

**Instrument corrected (centred difference).** 39.1's table above used a 0.5 s *forward*
difference, which leads the truth by ~0.25 s and inflated the slope. Re-run with a centred
+/-0.5 s difference, the finding holds but is smaller:

| route-time | d1 (m) | centred slope | fwd slope (old) | `leadOne.vRel` | U11 vRel |
|---|---|---|---|---|---|
| 5:17.10 | 54.5 | -3.45 | -4.30 | +0.16 | +0.09 |
| 5:17.26 | 53.7 | -4.21 | -4.85 | +0.05 | +0.05 |
| 5:17.40 | 53.2 | -4.61 | -5.49 | **-2.96** | -0.08 |
| 5:18.15 | 49.0 | -6.27 | -6.62 | -5.44 | -1.36 |
| 5:19.05 | 44.5 | -5.14 | -6.62 | -3.20 | -3.20 |

Corrected magnitudes: the blind window is **~0.8 s** (5:16.6 -> 5:17.40), not ~1 s, and the peak
under-read is **~4.3 m/s**, not 6. `leadOne.vRel` breaks away at 5:17.40 and tracks within
0.6-2.0 m/s; the raw U11 track vRel does not converge until ~5:19.0, so it lags by **~1.6 s**.
Both conclusions in 39.1 survive the correction; only the numbers move.

### 39.2 RETRACTED: the tFollow/desiredFollowDistance zeros are a disengagement, not a defect

39.1 closed by calling the simultaneous zeroing of `tFollow`, `desiredFollowDistance` and
`minAcceleration` at 5:25.24 "not yet explained" and pointing at longitudinalPlanSP. **That was
wrong.** At that same sample `longActive`, `enabled` and `active` all go 1 -> 0 together: the
driver disengaged. `StarPilotFollowing.update()` sets `t_follow = 0` in its `else`
(`not long_control_active`) branch and `desired_follow_distance = 0` alongside it, so the zeros
are the documented behaviour of a disengaged system.

Confirmed fleet-style on the route itself: **0 of 22,038 `longActive` samples on 24d have
`tFollow == 0`.** The zeros never occur while engaged.

Consequence for the complaint: the slow crawl from 5:25 to 5:36 (6.3 -> 0 m/s, gap 39 -> 10 m)
was **the driver**, not openpilot. Only the ~5 engaged seconds before the disengagement belong to
the car.

One real oddity noted but NOT established: `carState.cruiseState.speed` reads 0.0 through this
window while the planner's own `vCruise` reads 22.22 m/s. I did not determine whether that is
normal for this Honda or a reporting gap, and nothing here rests on it.

### 39.3 The padding behaviour is real, rare, and still without a mechanism

The engaged window 5:21 -> 5:25 is genuine and matches the reported complaint: `vEgo` 6.3 m/s flat,
gap 38-40 m, `desiredFollowDistance` 13-14 m, set speed 22.2 m/s, `maxAcceleration` 1.77,
`disableThrottle` 0, `shouldStop` 0 -- and `aTarget` sitting at **+0.05**. The car wants a 13 m gap,
has 39 m, has 16 m/s of speed deficit and full throttle headroom, and commands nothing.

Scored across the route with a deliberate signature (tracking a lead, gap > 2.5x desired,
|aTarget| < 0.25, `maxAcceleration` > 0.8, set speed at least 5 m/s above `vEgo`, throttle not
disabled, not stopping):

- **93 samples = 4.7 s out of 1102 s engaged (0.4%).**
- `longitudinalPlanSource` is `cruise` on **all 93**.
- median `vEgo` 6.3 m/s (range 4.2-16.5) -- it is a low-speed, post-deceleration behaviour.

**Two hypotheses tested and both rejected.** (a) A `t_follow` defect: rejected by 39.2. (b)
Experimental mode / e2e longitudinal: rejected -- the 93 samples split 49/44 between `expMode` 1
and 0, while the route runs 5081/16922 in favour of `expMode` 0, so the behaviour is not tied to
it. The common thread is only `src == cruise` at low speed after a deceleration.

**Open:** why the cruise cost yields ~0 m/s^2 with a 16 m/s speed deficit and 1.77 m/s^2 of
headroom. That is a `longitudinal_mpc_lib` / cruise-cost question. 4.7 s on one route is too
little to tune against -- the next step is to run this same signature across all 17 routes before
anyone touches a cost weight.

### 39.4 The padding signature run fleet-wide: 24d is unremarkable, and there is nothing to tune

39.3's closing step is done. The same signature (tracking a lead, gap > 2.5x desired,
`|aTarget| < 0.25`, `maxAcceleration > 0.8`, set speed at least 5 m/s above `vEgo`, throttle not
disabled, not stopping) was scored over all **16 cached routes, 11,221 s of engaged time**, from
the `scan_<route>.csv` whole-route dumps. Script: `$CLAUDE_JOB_DIR/tmp/padfleet.py` (job-local).

**Fleet: 673 samples = 58.7 s of 11,221 s engaged (0.52%).** Per route, as a share of that route's
engaged time: 23f 1.14%, 236 1.01%, **24d 0.63%**, 232 0.55%, 237 0.51%, 245 0.35%, 248 0.22%,
23e 0.14%, 241 0.13%, 24b 0.05%, 23a 0.00%; and **zero samples on 231, 239, 23b, 246**.

- **24d is not an outlier.** At 0.63% it sits mid-pack, below 23f and 236. Whatever the drive felt
  like, the route is not measurably more padded than the rest of the fleet.
- **`longitudinalPlanSource` is `cruise` on 669 of 673 samples** (the other 4 are `lead0`/`lead1`
  on 236). This is the one property that holds fleet-wide and it confirms 39.3's read.
- **`expMode` does not track it,** again: 232 splits 0/70, 237 splits 78/11, 23f 135/44. The
  behaviour appears on both sides of the toggle on different routes, so it is not e2e longitudinal.

**The signature does not describe a sustained behaviour, which is the finding that matters.**
The longest contiguous episode anywhere in 11,221 s is **3.51 s** (23f 23:00.60); 24d's 5:21.60
window is **2.39 s**, the second longest. Sweeping the `aTarget` cap to see whether a slow-approach
mode was being excluded by the tight threshold:

```
 cap   samples   secs  % eng  eps>=1s  longest
0.25       550   51.5  0.41%        6    3.51s
0.40       887   75.0  0.60%       10    6.15s
0.60      1263  105.2  0.85%       17    6.15s
0.80      1761  142.7  1.15%       29    6.15s
1.00      2408  185.9  1.50%       46    6.15s
```

The sample count and the engaged share grow roughly linearly with the cap while the longest episode
**stops moving at 6.15 s**. That is the shape of ordinary distribution mass being swept up by a
widening threshold, not of a distinct behaviour mode with its own duration scale. A real
"insists on keeping a certain distance" fault would show sustained episodes that appear once the cap
clears their operating point; none do.

**Conclusion: no cruise-cost change is justified on this evidence, and 39.3's open question is
closed as not actionable.** The brief near-zero-`aTarget` samples are real and they are concentrated
in `src == cruise` at low-to-mid speed after a deceleration, but they are transient (median episode
well under a second, maximum 6.15 s across the whole corpus) and no worse on the route that
prompted the complaint. Touching a `longitudinal_mpc_lib` cost weight to chase 0.5% of engaged time
would be tuning against noise. **What would reopen this is a drive where the driver marks the
timestamp of the padding as it happens** -- the complaint may be about a window this signature does
not capture at all, and 4.7 s buried in 1102 s cannot be matched to a memory of a drive.

Replay/offline evidence only. Nothing here is road-validated.

### 39.5 The census re-run as one uniform pass over all 16 routes: every event reproduces, and the route/segment count was double-counted

Items 38 and 39 spliced **two different runs** — 13 routes, then 4 more — and 248 was scored twice
(8 non-contiguous segments in item 38, the full 23 in item 39). That splice is the reason to re-run,
not a doubt about the events. This is one consistent pass: the durable
`tools/bosch_a_dropout_census.py` with `DUMPALL=1` and `min_gap 1.0` over every route that has a
`scan_*.json`, run route-by-route from a single detached driver.

**The six events reproduce exactly — same routes, same route-times, same track ids, same `y`,
`exFade`, `drate`, `accel@loss` and `mlprobMin`.** So do all three demotions
(23e 23:46.08, 236 32:04.95, 249 2:13.91 — each `accelAt0 = 0.00`, the defect-3 gate from item 38).
Nothing in items 38, 39 or their conclusion changes.

#### The one correction: 16 routes / 335 segments, not 17 / 343

Item 39's header says "17 routes / 343 segments". There are **16 distinct routes and 335 segments**.
The extra route-entry and the extra 8 segments are 248 counted twice — item 38's partial 8-segment
copy plus item 39's full 23. The arithmetic closes: 264 − 8 = 256 over 12 routes, plus item 39's
79 (23 + 6 + 18 + 32) = **335**. Read item 38's "17 routes" as *17 route-entries over 16 routes*.

#### The full pre-filter picture, which items 38 and 39 only ever reported as zeros

| route | segs | in-lane gaps | ≥1.0s | `exited` | `oncoming` | `DROPOUT` | strict |
|---|---|---|---|---|---|---|---|
| `00000231--5782493b00` | 3 | 22 | 3 | 3 | 0 | 0 | 0 |
| `00000232--3a01619ce5` | 29 | 43 | 30 | 26 | 1 | 3 | 0 |
| `00000236--60bfb34cb1` | 42 | 78 | 53 | 39 | 3 | 11 | **1** |
| `00000237--77313c5a66` | 30 | 69 | 26 | 21 | 4 | 1 | 0 |
| `00000239--d1cf55daa7` | 16 | 35 | 14 | 12 | 0 | 2 | 0 |
| `0000023a--5c3a439dfc` | 18 | 34 | 10 | 5 | 0 | 5 | 0 |
| `0000023b--7f6d4c1ba9` | 5 | 11 | 6 | 4 | 0 | 2 | 0 |
| `0000023e--9a40b07f55` | 42 | 111 | 37 | 28 | 3 | 6 | 0 (1 demoted) |
| `0000023f--66ddbb900a` | 32 | 70 | 27 | 22 | 1 | 4 | **1** |
| `00000241--7948e97423` | 12 | 37 | 11 | 5 | 1 | 5 | **2** |
| `00000245--1356bb0355` | 19 | 63 | 17 | 12 | 1 | 4 | 0 |
| `00000246--52b04ed170` | 8 | 17 | 9 | 5 | 2 | 2 | 0 |
| `00000248--4f275f0bb6` | 23 | 10 | 5 | 3 | 0 | 2 | **1** |
| `00000249--d481c5de77` | 6 | 19 | 9 | 4 | 0 | 5 | 0 (1 demoted) |
| `0000024b--02cabe206c` | 18 | 83 | 19 | 19 | 0 | 0 | 0 |
| `0000024d--f80e13b850` | 32 | 57 | 33 | 29 | 0 | 4 | **1** |
| **TOTAL** | **335** | **759** | **309** | 237 | **16** | 56 | **6** |

Two things this table says that the earlier write-ups did not:

1. **56 gaps carry `kind=DROPOUT` at the pre-filter stage; 6 survive the strict gate, 3 more are
   demoted at the engagement check.** The other 47 fail on engagement, acceleration, hold time or
   the box — i.e. the strict gate is doing almost all of the work, and **`kind=DROPOUT` on its own
   is not a fault count.** Anyone reading a raw census log should not quote the 56.
2. **16 oncoming crossings, not 15** as item 38's defect-4 note says. The extra one is bookkeeping
   from the same splice; the classifier (`d_rate < -(vEgo + 3)`) is unchanged and correct.

`0000024b--02cabe206c` remains the clean control: **19 gaps ≥ 1 s, all 19 `exited`, no `DROPOUT`
kind at all.** A route with that many long in-lane gaps and zero dropout-shaped ones is the strongest
single argument that the shape is geometric.

#### Still absent, and it is the thing that would change the answer

**No fade-then-die with the target centred, anywhere in 335 segments.** Of the six strict events the
`y` at loss is −1.9, −1.4, −2.0, −1.4, −2.0 and +0.5, and the one near boresight (248 10:33) kept the
camera lead at probability 1.00. Item 38's conclusion is unchanged and now rests on a single
uniform pass: **no deletion gate is justified, and D-041 forbids one on evidence this weak.**

Replay evidence only. Nothing here was driven. Benign stderr on every route: the `CANParser ...
not valid (timeout or missing)` lines are the parser's cold start before the first radar frame.

## 40. James's `3a257065d` reconciled against our `longitudinal_planner.py` — DO NOT MERGE IT

The long-carried open item. James's commit `3a257065d` ("long: feed the BLoT supervisor the MPC
target, not the arbitrated output") was never merged here because we already carry the same fix
twice: `552798ab0` and `084a9d56e`, de-duplicated by `95ce224f2`. This is the deliberate diff that
was owed, done as a diff and not a merge.

### The fix is carried in full, exactly once

Every line James's commit adds, normalised for the BLoTv2 -> BLoTv3 rename, appears **exactly once**
in our tree; both lines it removes appear **zero** times:

| element | ours |
|---|---|
| `JERK_SCALE_MIN` added to the supervisor import | `longitudinal_planner.py:23` |
| `self.last_mpc_a_target = 0.0` in `__init__` | `:635` |
| `self.last_mpc_a_target = float(self.a_desired)` on reset, after the aEgo clip | `:2150` |
| supervisor fed `float(self.last_mpc_a_target)` instead of `output_a_target` | `:2246` |
| `blotv3_jerk_scale` clipped to `[JERK_SCALE_MIN, 1.0]` at `set_weights` | `:2433` |
| `output_a_target_mpc = None` before the model-path branch | `:2555` |
| write-back after arbitration, before the caps | `:2603` |

`output_a_target_mpc` is assigned on the tinygrad path at `:2561` and stays `None` on the classic and
default paths, so the write-back's `is not None` fallback is what carries those two. The write-back
sits after the experimental speed handoff (`:2591`) and before `comfort_output_accel_min` (`:2605`),
which is the point the fix requires: the MPC solution is captured before the vision caps, the curve
limiter, the force-decel floor and the stop-go target can touch it. `blotv3.py` exports
`JERK_SCALE_MIN = 0.3`, and `a_mpc` is the same third positional parameter of
`BLoTv3Supervisor.update()`.

### The entire remaining divergence in this file is the rename plus one blank line

`git diff 3a257065d HEAD -- selfdrive/controls/lib/longitudinal_planner.py` is **27 insertions,
26 deletions**, and every one of them is either a `blotv2`/`BLoTv2`/`BlotV2` -> v3 identifier,
comment or param-key rename, or a single added blank line at `:158`. **There is nothing in James's
version of this file that we lack**, and nothing of ours that his fix would improve. A merge or
cherry-pick would apply the fix a third time; `git apply -3` reports conflicts on it. **Do not merge
`3a257065d`. The item is closed.**

### What the reconciliation did turn up: the rename silently dropped the toggle for one drive

The rename changed the Params key from `BlotV2` to `BlotV3`. `common/params_keys.h:373` registers
`{"BlotV3", {PERSISTENT, BOOL, "0", ...}}` — default **off** — and nothing migrates the old key. No
`BlotV2`/`blotv2`/`BLoTv2` reference survives anywhere in the tree, so the rename itself is clean,
but the stored value did not carry. Read from `initData` on all 16 cached routes:

```
231 232 236 237 239 23a 23b 23e    BlotV2=1   BlotV3 absent   (pre-rename builds)
23f                                BlotV2 absent   BlotV3=0    <-- BLoT ran DISABLED
241 245 246 248 249 24b 24d        BlotV2 absent   BlotV3=1
```

**23f is a drive with the supervisor off**, between the rename and the toggle being re-set. This is
the same failure mode as D-053 (believed on, never stored) and it argues for checking `initData`
before crediting any toggle, which the project memory already says.

**Tempting but not supported:** 23f is also the highest row in the 39.4 padding table (1.14% of
engaged time) and the one route where BLoT was off. But 236 sits at 1.01% with `BlotV2=1`, so the
supervisor being off does not separate the populations. n=1 either way. **Do not read 23f's 39.4 row
as a BLoT effect.**

### Verification

`selfdrive/controls/tests/test_longitudinal_planner.py` + `selfdrive/controls/lib/tests/test_blotv3.py`:
**519 passed** (`-W ignore::DeprecationWarning`, required or the planner file fails collection under
NumPy 2.5). Static and unit-test evidence only. No replay and no road validation of the BLoT path.

## 41. Two changes carried in from the upstream review: the pursuit tail, and a renamed-toggle migration

Both land on `ns-bosch-radar-testing`. Both are **unit-test evidence only — no replay, no road
validation.** Neither is a fix for anything observed on our own routes.

### Half A — the pursuit tail (`f7a85a35a`)

Ported from upstream `SpysyWeeb/Spysypilot` `cdf8e54c9` (2026-09-02), `necessity_supervisor.py`.
Our port baseline was `7aed8763c`, so this was the **one real behavioural gap** in the upstream
supervisor. The other two supervisor commits since that baseline are not gaps: `7dfe70610` removes
dead `stand_down` code we never carried, and `7f0a9ad37` is naming only
(`STOPPED_LEAD_FULL_DECEL = 1.2`, already the literal `1.2` in `blotv3.py`).

**The problem.** The recovery trigger disarms at the plan's zero crossing — which is exactly the
frame where the MPC has to swing from braking to acceleration. So the pickup behind a lead that has
driven off is made with the **stock stiff jerk cost**, the one the supervisor had just spent the
whole braking episode softening.

**The mechanism.** `PURSUIT_TAIL_S = 3.0` in `selfdrive/controls/lib/blotv3.py`:

- arms `self._pursuit_s = PURSUIT_TAIL_S` while `recovery_active and lead.acceleration > LAUNCH_ALEAD_ON`;
- clears outright on `lead.acceleration <= 0.0` (the lead stopped pulling — not a pursuit any more);
- otherwise decays by `self.dt`;
- feeds one disjunct:
  `if recovery_active or model_active or launch_active or self._pursuit_s > 0.0: scale_target = JERK_SCALE_MIN`.

Cleared in `reset()`, on the emergency bypass, and on the not-present/crawl branch. A pull-away that
ends in a crawl is not a pursuit, so the tail is deliberately **not** held through the low-speed latch.

**It touches `jerk_scale` ONLY — never `t_follow`.** Following distance is unchanged by this commit,
and there is a test that asserts exactly that.

**One deliberate divergence from upstream.** `_pursuit_s` is initialised in `__init__` as well as in
`reset()`. Upstream carries it only in `reset()` because upstream's `__init__` calls `reset()`; ours
does not. Without the extra initialiser the first frame that reaches the decay branch raises
`AttributeError`. Pinned by `test_pursuit_tail_state_exists_before_the_first_update` so a future
re-sync cannot quietly drop it.

**Evidence discipline.** The figure in the code comment — `+0.26 m/s²` early in the pickup, `0.15 s`
over the ramp, upstream route `0x3b` t=390–405, a truck that drove off — is **upstream's replay
measurement on upstream's tree.** It is not ours. We have no route in the cache that reproduces it.
On this tree the tail is supported by unit tests and by the mechanism argument above, nothing more.

**Do NOT read this as a fix for the 24d padding complaint.** Item 39.4 found those padding samples
are `src == cruise`, not a lead-departure recovery. The two are adjacent in subject matter and
unconnected in evidence. Recorded as a lead only.

### Half B — migrating a renamed PERSISTENT BOOL (`38d5635a9`)

Item 40 found that renaming the Params key `BlotV2` → `BlotV3` silently dropped the stored value,
and route `0000023f` therefore drove with the supervisor **off**. This commit adds the migration that
should have shipped with the rename.

`migrate_starpilot_bool_param_renames(params, params_cache)` in `system/manager/manager.py`, table
`LEGACY_STARPILOT_BOOL_RENAMES = {"BlotV2": "BlotV3"}`, guarded by its own one-shot flag
`/data/starpilot_bool_rename_v1`, called from `manager_init` **ahead of the `clear_all()` calls**.

Three design points worth keeping:

1. **It needs its own flag.** The existing `migrate_starpilot_param_renames` looks like the right
   home, but its flag (`starpilot_param_rename_v1`) is already written on every device that went
   through the FrogPilot→StarPilot rename. An entry added to that table would **never run**. Any
   future rename migration needs a fresh flag for the same reason.
2. **It reads the old key as raw bytes off disk** (`_read_raw_param_bytes` → `get_param_path`), not
   through the schema. So **no `common/params_keys.h` change, and therefore no aarch64 artifact
   rebuild.** The orphaned key is unreadable through the normal API precisely because the schema no
   longer knows it.
3. **An explicit value on the new key wins**, in either store. For a toggle that gates longitudinal
   control, turning something **on** unasked is the worse error.

**The key finding, and why this is a guard rather than a repair.** `clear_all()` **deletes params
keys the schema does not know** (see the comment near `manager.py:1132`). `BlotV2` was therefore
already gone from disk after the *first* boot of the renamed build. **This migration cannot repair
route `0000023f`.** A rename has exactly **one** boot in which the old value is still recoverable,
which means the migration entry has to ship in the **same release** as the rename — added afterwards
it is always too late. That constraint is written into the comment above the table so the next
person renaming a toggle hits it before repeating the mistake.

### Verification

| | |
|---|---|
| `test_blotv3.py` (8 new tests) | 26 passed |
| negative control, pursuit tail | **2 failed / 24 passed** on revert |
| `test_manager.py` (7 new tests) | 42 passed, 1 skipped |
| negative control, migration | **2 failed / 40 passed** on revert |
| planner + blotv3 | 527 passed |
| manager + planner + blotv3 | **569 passed, 1 skipped** |

The negative controls are partial by design: of the 8 pursuit-tail tests only two assert the tail
*holds*; the rest assert it **ends** (on its own, when the lead stops pulling, on lead loss, on
reset) and so pass under revert. A test that passes with the feature removed is not evidence for the
feature — recorded here so the count is not mistaken for a weak control.

`-W ignore::DeprecationWarning` is required or `test_longitudinal_planner.py` fails **collection**
under NumPy 2.5 and the run silently shrinks. `manager.py` carries 5 pre-existing ruff findings
(ISC002, E501, PIE810); none are on these lines and none were touched.

Static and unit-test evidence only. **No replay and no road validation of either change.**

## 42. Two bookmarked drives (24f, 251): the semi-hard brake is the −3.5 rail, and the driver's marks land on it — 2026-09-21

Routes `0000024f--8c147bae2e` (19 segments, 1082 s) and `00000251--0f743e380e` (15 segments, 888 s),
both fetched on 2026-09-21. **First drives in this project with usable driver bookmarks.**

### Configuration (read from `initData`, per the D-053 lesson)

Both routes: `gitBranch = ns-bosch-radar-testing`, commit `350570847`, `dirty = False`,
**`BlotV3 = 1`** (supervisor ON, unlike 23f), `LongitudinalPersonality = 1` (Standard),
`StandardFollow = 1.6`, `StandardFollowHigh = 1.2`, `CustomPersonalities = 0`.
**Neither route contains the pursuit tail** (`f7a85a35a`, committed after these drives).

### The bookmarks are good instruments

`userBookmark` presses: 24f at 2:53.24, 4:37.36, 6:09.52, 7:34.11, 9:57.02, 10:25.57; 251 at
3:57.13, 6:07.46, 8:27.71. **Every mark follows its braking episode by 1–8 s**, against ~26 s on 24d.
Search a window *backwards* from the mark, not at it, but the lag is now small enough that
attribution is unambiguous.

### The marks land on rail-pinned episodes

Hard-brake episodes = `longActive`, `aTarget < −1.0`, held ≥ 0.2 s. "Rail" = `aTarget <= −3.40`.

| route | episodes | rail-pinned | rail rate |
|---|---|---|---|
| `0000024f` | 14 | 3 | 21% |
| `00000251` | 4 | 1 | 25% |
| `0000024d` (item 39.1) | 23 | 10 | 43% |

Mapping marks to episodes:

| mark | episode | duration | `aTarget` min | time at rail |
|---|---|---|---|---|
| 24f 2:53.24 | 2:49.37 | 2.3 s | **−3.47** | 0.30 s |
| 24f 4:37.36 | 4:31.57 | 3.1 s | **−3.45** | 0.90 s |
| 24f 6:09.52 | 6:01.88 | 6.5 s | **−3.47** | 0.45 s |
| 24f 7:34.11 | 7:28.33 | 1.6 s | −2.09 | — |
| 24f 9:57.02 | 9:54.54 | 1.9 s | −2.46 | — |
| 24f 10:25.57 | 10:25.35 | 0.4 s | −1.08 | — |
| 251 3:57.13 | 3:55.50 | 3.3 s | **−3.47** | 0.85 s |
| 251 6:07.46 | 6:05.01 | 1.0 s | −3.20 | — |
| 251 8:27.71 | 8:22.51 | 0.4 s | −1.29 | — |

**All four rail-pinned episodes across both routes are bookmarked**, and 24f's three rail events are
its only three. The unbookmarked 24f 12:11.29 touched the rail for one frame (0.05 s). So the
sensation the driver reports **is the rail**, and the driver's discrimination is sharper than the
statistics: the four worst episodes by this metric are exactly the four they marked.

### The mechanism is NOT 24d's estimator divergence

Traced 24f 2:49.37 frame by frame (centred ±0.5 s difference on `d1`):

| t−onset | `vEgo` | `d1` | `y1` | centred slope | `leadOne.vRel` | U11 `vRel` | `aTarget` |
|---|---|---|---|---|---|---|---|
| −2.78 | 17.57 | 47.3 | −0.68 | −0.49 | −0.06 | −0.06 | +0.46 |
| −2.33 | 17.85 | 46.8 | −0.62 | **−2.32** | −0.64 | −0.64 | +0.26 |
| −2.03 | 18.03 | 46.0 | −0.57 | −2.51 | −1.23 | −1.23 | −0.41 |
| −1.43 | 17.92 | 44.0 | −0.43 | −2.29 | −2.12 | −2.12 | −0.50 |
| −0.68 | 17.14 | 42.8 | −0.23 | **−3.04** | −1.69 | −1.69 | −0.64 |
| −0.23 | 16.63 | 40.8 | −0.14 | **−4.68** | −2.95 | −2.95 | −0.81 |
| −0.08 | 16.46 | 39.4 | −0.14 | −5.22 | −4.00 | −4.00 | −0.94 |
| +0.52 | 15.21 | 37.3 | −0.11 | −2.90 | −3.59 | −3.59 | **−3.47** |

Three things, in order of confidence:

1. **`leadOne.vRel` and the raw U11 `vRel` are identical to two decimals through the whole episode.**
   There is **no estimator divergence on this route.** 39.1's finding — ours converging ~1.6 s ahead
   of raw U11 — does **not** reproduce here, so it is not a general property of the Bosch-A path. Do
   not cite 39.1's convergence advantage as a fleet-wide claim; on 24f the two are the same signal.
2. **The radar under-read still happens, but it is half the size of 24d's.** Blind window ≈ **1.3 s**
   (−2.8 to −1.5) with a peak under-read of ≈ **1.7 m/s**, and a second divergence at −0.7 to −0.2
   reaching ≈ 2.4 m/s. 24d's corrected figures were ~0.8 s and ~4.3 m/s. So latency is present and
   smaller, yet the outcome (rail) is the same — **latency alone does not explain the rail.**
3. **The planner under-reacts to data it already has, then catches up all at once.** At t = −2.03 the
   measured slope is −2.51 and `vRel` is already −1.23, and `aTarget` is only −0.41. It then moves
   −0.94 → **−3.47 in 0.6 s**. The lead is a clean, continuously-tracked, dead-centre target the
   whole time (`tid1 = 48` unbroken, `mProb1 = 0.999`, `y1` −0.8 → 0.0, `d1` 47 → 36 m). This is not
   a dropout, not a cut-in, and not a track change.

**Consequence: the tuning target moves.** 24d pointed at sensor latency. 24f points at the planner's
*response shape* — it sits on a developing closure, then demands everything in one ramp and clips the
limit. That is a jerk-cost / ramp-rate question, and it is the first evidence on this branch that
argues for touching one.

### Not established, and deliberately not acted on

- **No cut-in has been isolated yet.** The driver reports the brake sometimes fires when being cut
  off, inconsistently. All four rail episodes examined here are continuous-track cases, so the
  cut-in population is still unsampled. It is a different code path (track initialisation, not
  `vRel` convergence) and needs its own pass.
- **Why `aTarget` lags its own `vRel` input is not explained.** Item 39.1 assumed late detection was
  sufficient; on 24f it is not. Candidates not yet separated: the jerk cost, `danger`/`accJerk`
  weighting, or a `t_follow` interaction. **Do not tune until the term is identified.**
- **Driving personality and safety-gap bias were deliberately left unchanged** for these drives, so
  24f/251 remain comparable with the other 16 routes. Both act on base `t_follow` and would confound
  the BLoTv3 pads. Changing either would mask this finding rather than fix it.
- **No claim that BLoTv3 caused or worsened this.** `BlotV3 = 1` on both routes, but there is no
  matched-configuration comparison, and the supervisor only touches `jerk_scale` and `t_follow`.

Route data cited by ID only. Offline log analysis; no replay and no road validation of any change.

## 43. The −0.50 clamp ahead of the rail: `DecelerationProfile` is ECO on both bookmarked routes

Item 42 ended with "why `aTarget` lags its own `vRel` input is not explained" and told the next
pass to find which term moves. It is not a jerk weight. The jerk columns are **constant** across
every rail episode on both routes: `accJerk = 250.00`, `spdJerk = 5.50`, `dngJerk = 100.00`,
`danger = 0.75`, `tFollow = 1.45`. Nothing in the jerk/danger family moves as `aTarget` ramps.

What moves is nothing — because `aTarget` is **clamped**, and the clamp is a setting.

### The measurement

`minAcc` in the scan CSV is `starpilotPlan.minAcceleration`. On both routes, on every sampled
frame, it reads exactly **−0.50**. `starpilotPlan.minAcceleration` is
`starpilot_acceleration.min_accel`, which for a plain deceleration profile is
`get_profile_min_accel_floor(deceleration_profile)`
(`starpilot/controls/lib/starpilot_acceleration.py:122`). That returns:

- `A_CRUISE_MIN_ECO` = `A_CRUISE_MIN / 2` = **−0.5**   (`starpilot_acceleration.py:61`)
- `A_CRUISE_MIN` = **−1.0**                            (`longitudinal_planner.py:429`)
- `A_CRUISE_MIN_SPORT` = `A_CRUISE_MIN * 2` = −2.0

`DECELERATION_PROFILES` is `{"STANDARD": 0, "ECO": 1, "SPORT": 2}`
(`starpilot/common/accel_profile.py:15`). The stored param on **both** routes is
**`DecelerationProfile = 1`, i.e. ECO** — read from `init_by_seg[<first seg>]['params']`, not
guessed. So the observed −0.50 is `A_CRUISE_MIN_ECO` and matches the code exactly.

That value is fed straight in as the lower accel limit:

```
longitudinal_planner.py:2137-2141
    if self.mpc.mode == 'acc':
      accel_limits = [sm['starpilotPlan'].minAcceleration, sm['starpilotPlan'].maxAcceleration]
      ...
      accel_limits_turns[0] = max(get_vehicle_min_accel(self.CP, v_ego), accel_limits_turns[0])
    else:
      accel_limits = [ACCEL_MIN, ACCEL_MAX]
```

### `aTarget` sits on that number, including with a closing lead tracked

24f 6:01.88, sampled at 5 Hz, `dt` relative to onset. `d1` is lead range, `vRel1` closing rate:

| dt | vEgo | aTarget | src | d1 | vRel1 | minAcc |
|---|---|---|---|---|---|---|
| −1.35 | 24.17 | **−0.50** | lead1 | 78.6 | −4.50 | −0.50 |
| −1.15 | 24.19 | **−0.50** | lead1 | 75.9 | −3.97 | −0.50 |
| −0.95 | 24.19 | **−0.50** | lead1 | 61.7 | −5.41 | −0.50 |
| −0.75 | 24.20 | **−0.50** | lead1 | 60.5 | −5.58 | −0.50 |
| −0.55 | 24.15 | **−0.50** | lead1 | 58.2 | −6.20 | −0.50 |
| −0.35 | 24.09 | −0.55 | lead1 | 56.8 | −6.30 | −0.50 |
| +0.05 | 23.91 | −1.24 | lead1 | 54.3 | −6.33 | −0.50 |
| +0.65 | 23.14 | −2.09 | lead1 | 52.2 | −5.11 | −0.50 |

**`aTarget` holds at exactly −0.50 for 1.2 s while a tracked lead closes at 6.3 m/s.** That is not
a solver choosing to be gentle; −0.50 to two decimals, held flat, equal to the configured floor,
is a clamp. 24f 4:31.57 shows the same pin for 1.6 s (`src = cruise`, `aTarget` −0.50 for eight
consecutive sampled frames) before ramping to −3.45.

The other side is unchanged from item 42: once released, `planAmin` and `planA1s` go to **−3.50**
exactly and `aTarget` reaches −3.47 in 0.6 s.

### What this does and does not establish

**Established.** ECO is set on both routes. The floor it selects is −0.5 rather than STANDARD's
−1.0. `aTarget` measurably rests on that value for over a second at a time, in at least one case
with a continuously tracked lead closing at 6.3 m/s at 55–78 m. No jerk or danger term varies
across any rail episode on either route.

**NOT established — and one code comment argues against it.**
`starpilot_acceleration.py:63` says of the traffic floor: *"cruise-decel floor only; MPC lead
braking keeps full ACCEL_MIN authority."* If lead braking really keeps full authority, the ECO
floor should not bind while `src = lead1` — yet the table above is `src = lead1` at −0.50. Either
the comment does not hold for the ECO/STANDARD floors, or something else produces exactly −0.50.
**That is not resolved here, and it is the next thing to settle.** `aTarget` does later exceed
−0.50, so the clamp is not unconditional; the release path has not been traced.

**No causal claim that ECO produces the −3.5 slam.** The sequence (pinned at −0.50, then −3.47 in
0.6 s) is consistent with a suppressed early response forcing a late large one, but consistency is
not evidence. There is no matched-configuration comparison: every route in the set was driven on
ECO, so the profile is a constant across all 16 and cannot be attributed against itself.

### Why this matters for the settings question

Item 42 answered "should I change driving personality or safety gap bias?" with no, because both
scale `t_follow` and the complaint is brake *shape*. That answer stands — and `tFollow = 1.45`
constant through every episode confirms the gap was never the moving part.

`DecelerationProfile` is a different lever and was not considered: it does not touch `t_follow`,
it sets the braking authority floor directly. ECO → STANDARD doubles it (−0.5 → −1.0). **This is
the first setting on this branch with a measured mechanism behind it.** It is still not a
recommendation: the release path is untraced, the causal link is unproven, and changing it breaks
the one-configuration assumption that makes 24f/251 comparable with the other 14 routes. If it is
changed, it must be changed as a deliberate A/B with the profile recorded, not quietly.

Offline log analysis of two routes. No replay, no road validation, no change made.

## 44. The 251 6:06 semi-hard brake: a real but small lead brake-tap, answered 1 s late at full authority

User-identified event (not the cut-in earlier assumed): route `11c8fa231c0499ed/00000251--0f743e380e`,
route time 364.4-366.2 s (~6:04-6:06). Peak measured `aEgo` **-3.58 m/s2 at t=365.86**. Traced frame by
frame from the device's own log (`radarState` 20 Hz, `liveTracks` ~15 Hz, segment 6 rlog).

### The stimulus is real. It is not a phantom and not a tracker fabrication.

`liveTracks` carries exactly one point in the ego lane through the whole event (y = -0.3 to -0.4 m), and
that point's own range and Doppler agree with each other:

```
  t        dRel   yRel   vRel      (raw liveTracks, ego-lane point only)
  363.90   45.6   -0.4   +2.5   <- lead receding
  364.44   43.7   -0.4   -0.9   <- closing begins
  364.77   41.0   -0.4   -4.2   <- peak apparent closing
  365.11   40.0   -0.4   -3.4   <- minimum range 39.5 m (radarState)
  365.18   40.6   -0.4   -1.1   <- +0.6 m / +2.3 m/s step in one 66 ms frame (the one discontinuity)
  365.58   40.6   -0.3   -0.2   <- closing over
  366.59   43.4   +0.1   +3.5   <- receding again
```

Range fell 45.6 -> 40.0 m in 1.0 s with `vEgo` flat at 26.45 m/s, so the lead really lost about 5 m/s
and regained most of it: a brake tap. `tid1 = 57` throughout, `mProb1 = 1.00`, `radar1 = 1`, no track
change. **Minimum range was 39.5 m at 26.3 m/s = 1.50 s headway, against `tFollow = 1.45`.** The gap
closed to roughly the configured following distance and no further. There was no collision threat at
any point in this event.

Two caveats recorded, not resolved: the +0.6 m / +2.3 m/s step at 365.18 is not physical for a rigid
object, and the recovery limb implies about +4.8 m/s2 of lead acceleration, which is also not physical
for a car. Some of the recovery limb is filter, not vehicle. The descending limb is smooth and
coherent in both range and Doppler and is taken as real.

### Vision saw nothing.

`mlV` stayed 26.3-27.2 m/s and `mlA` stayed about +0.03 m/s2 for the entire event; `mlX` stayed 47-51 m
and never dipped. The model contributed nothing to this brake and, read literally, argued against it.
Consistent with the standing finding that vision is not closing-speed truth - but note the direction:
here vision's silence was the *correct* magnitude call and the radar's was the alarming one.

### The control response is late and then unlimited. That is the defect shape.

```
  t        aTarget  minAcc  planAmin  src      vRel1   d1
  364.41    -0.04   -0.50    -0.51    lead1    -0.28   43.67
  364.65    -0.50   -0.50    -1.72    lead1    -3.38   41.48   <- aTarget == minAcc exactly
  364.75    -0.50   -0.50    -2.25    lead1    -3.80   40.98   <- still exactly -0.50
  364.85    -0.50   -0.50    -3.35    lead1    -4.14   40.23   <- still exactly -0.50
  364.91    -0.65   -0.50    -3.50    lead1    -4.14   40.23   <- clamp released; minAcc unchanged
  365.15    -1.55   -0.50    -3.50    lead1    -3.41   39.54   <- minimum range
  365.50    -3.20   -0.50    -3.43    lead1    -0.38   40.23   <- aTarget minimum; closing already over
  365.60    -3.14   -0.50    -3.32    lead1    -0.17   40.17
  365.86    -2.58   -0.50    -2.95    cruise   +0.98   40.67   <- aEgo minimum -3.58; lead pulling away
```

Three facts, each read directly off the log:

1. **The ECO floor binds again, with `src = lead1`.** Five CSV rows (about 0.2 s, 2-3 distinct plan
   frames) sit at exactly -0.50 = `minAcc` = `A_CRUISE_MIN_ECO` while `planAmin` is already diving
   through -3.35. This is the second independent instance of the STATUS 43 signature, and the second
   one that contradicts the `starpilot_acceleration.py:63` comment ("MPC lead braking keeps full
   ACCEL_MIN authority"). The 24f 6:01 instance held 1.2 s; this one holds 0.2 s. Weaker, same shape.
2. **The clamp delays the onset without capping the peak.** `minAcc` never changes value, yet `aTarget`
   passes through it at 364.91 and continues to -3.20. So the floor is not an authority limit on this
   path - it is a 0.2 s dead band at the *start* of the response, i.e. exactly the interval in which a
   small early correction would have been sufficient. Late and then full-authority is the worst of the
   two behaviours.
3. **The hardest braking is applied after the stimulus has gone.** Closing is over by 365.58 and the
   lead is pulling away at +1.0 m/s by 365.86, which is where measured `aEgo` bottoms out at -3.58.
   The peak of the response trails the peak of the stimulus by about 1.0 s and trails the end of the
   stimulus by about 0.3-0.7 s. The range was already growing when the car braked hardest.

### What this changes and what it does not

Established: a real 5 m/s lead brake-tap that never took the gap below the configured following
distance produced a -3.58 m/s2 brake whose peak landed after the gap had reopened, with the ECO floor
holding the first 0.2 s of the response at exactly -0.50 while `planAmin` was already on the -3.50 rail.

NOT established, and specifically not to be tuned on: that removing or raising the ECO floor would
reduce the peak. It is plausible - an earlier, gentler response to a stimulus this small should not
need -3.5 - but the counterfactual has not been run. **That is a replay experiment, not an edit.**
`ACCEL_MIN = -3.5` as the rail source is still unverified, and the release path at 364.91 is still
untraced (`longitudinal_planner.py:2137-2141` versus `get_mpc_mode()` at :2448-2450).

Also noted for later, not part of this event: at 365.65-365.98 a third `liveTracks` point appears at
y = +9.2 to +10.4 m with a constant reported `vRel = -13.5 m/s` while its own `dRel` walks 40.6 -> 29.2 m
in 0.33 s, a range rate of about -34 m/s. Its Doppler contradicts its own range rate by roughly 20 m/s.
It is well outside the ego lane and did not drive this brake, but it is a parser-side inconsistency
worth a census pass.

Offline log analysis of one route, one event. No replay, no road validation, no change made.

## 45. The ECO decel floor is a one-sided leaky clamp on the planner OUTPUT, not on the MPC

Static code read of `selfdrive/controls/lib/longitudinal_planner.py` and
`selfdrive/controls/lib/longitudinal_mpc_lib/long_mpc.py` on this tree. No replay, no road
validation, no change made. This resolves the release path left untraced in items 43 and 44.

**`ACCEL_MIN == -3.5` is confirmed** at `opendbc_repo/opendbc/car/interfaces.py:40`
(`ACCEL_MAX = 2.0` at :39). It is the rail `planAmin` sits on in both the 24f 6:01 and the
251 6:06 episodes.

**The ECO floor never reaches the solver.** `long_mpc.py:956` sets

    self.params[:,0] = ACCEL_MIN

unconditionally, before and outside the `if self.mode == 'acc'` branch. `set_accel_limits()`
(:919-923) stores its `min_a` in `self.cruise_min_a`, and `cruise_min_a` is read in exactly one
place, `:965`:

    v_lower = v_ego + (T_IDXS * self.cruise_min_a * 1.05)

which clips `v_cruise` to build the *cruise* obstacle. Upstream's own comment on `set_accel_limits`
says so: "TODO this sets a max accel limit, but the minimum limit is only for cruise decel /
needs refactor." So the MPC really does keep full `ACCEL_MIN` authority for lead braking, and
the claim in `starpilot/controls/lib/starpilot_acceleration.py:63` is **correct about the MPC**.
Items 43 and 44 called that comment contradicted. That was wrong, and this supersedes it.

**The clamp is on the published output instead.** The path is:

  1. `longitudinal_planner.py:2137-2139` - in `acc` mode, `accel_limits[0] = starpilotPlan.minAcceleration`
     (the ECO floor, -0.50), carried into `accel_limits_turns[0]` by `limit_accel_in_turns()`.
  2. `:2141` - `accel_limits_turns[0] = max(get_vehicle_min_accel(self.CP, v_ego), accel_limits_turns[0])`.
     For this car `get_vehicle_min_accel` falls through to `float(ACCEL_MIN)` at `:425`, so
     `max(-3.5, -0.50) = -0.50`. The floor survives.
  3. `:2605-2607` - `comfort_output_accel_min = accel_limits_turns[0]`, then `output_accel_min = comfort_output_accel_min`.
  4. `:3115` - `output_a_target = float(np.clip(output_a_target, output_accel_min, output_accel_max))`.

That `np.clip` is what emits exactly -0.50. It explains the exact equality `aTarget == minAcc` with
`src = lead1` while `planAmin` was already through -3.35: the MPC had solved for the harder number
and the planner truncated it on the way out.

**What releases it is `:2202`:**

    # clip limits, cannot init MPC outside of bounds
    accel_limits_turns[0] = min(accel_limits_turns[0], self.a_desired + 0.05)

`self.a_desired` is the planner's own trajectory state, interpolated from the MPC solution at
`:2526`, and it is **not** subject to the `:3115` clip. So `a_desired` follows the MPC down
freely while the published `aTarget` is pinned at -0.50; as soon as `a_desired + 0.05` falls
below -0.50 the bound follows `a_desired` down, and from then on the clip is inactive.

**So the ECO deceleration profile is a pure phase lag on lead braking, with no attenuation
afterwards.** The hold lasts however long `a_desired` needs to fall through the floor - about
0.2 s on 251 (2-3 plan frames), about 1.2 s on 24f - and once released the output tracks the
unclamped MPC solution all the way to the rail. It is late-then-unlimited by construction, not
by accident. This is the mechanism behind the shape measured in item 44: the -3.58 m/s2 peak
landed about 1.0 s after peak closing.

**What this does NOT establish.** That the clamp is the dominant cause of the late peak - the
MPC's own solution was already through -3.35 before the clamp released, so the solver was
asking for a hard number on its own and part of the phase error is upstream of `:3115`.
Whether removing the clamp reduces or merely re-times the peak is still a replay counterfactual,
not an edit, and it has not been run. `:2202`'s comment says its purpose is MPC initialisation,
so it is not obviously the right place to govern output authority either; that is a design
question for DECISIONS.md, not a tuning change.

## 46. The phase hypothesis is dead. The radar-vs-no-radar difference is brake ENTRY RATE, not lag and not peak magnitude

Item 44 proposed that the 251 6:06 brake felt wrong because it arrived ~1.0 s after peak
closing with the gap already reopening, and that radar brakes are systematically later than
vision-lead brakes. **Measured across the corpus, that is false.** The lag distributions are
indistinguishable.

Method: `$CLAUDE_JOB_DIR/tmp/phase.py`. Episode = `aEgo < -1.5` held > 0.15 s on engaged
frames with `vEgo > 5`. For each episode `t_a = argmin(aEgo)`; the closing signal is searched
over `[episode_start - 4.0, t_a]` using two independent measures, the published `vRel1` and the
geometric range rate `d(d1)/dt` differenced over 0.5 s. `lag = t_a - t_closing_peak`.

| | 242 NO RADAR | 251 RADAR | 24f RADAR |
|---|---|---|---|
| episodes / span | 25 / 1385 s | 3 / 448 s | 11 / 831 s |
| lag `vRel1` p50 | +0.70 s | +0.95 s | +0.70 s |
| lag range-rate p50 | +2.60 s | +1.05 s | +1.10 s |
| gap already opening at peak brake | 20% (5/25) | 33% (1/3) | 36% (4/11) |

The `vRel1` medians are the same to within one plan frame. On the range-rate measure the
**no-radar** route is the later one (+2.60 s vs +1.05/+1.10), the opposite of the hypothesis.
The opening-at-peak fractions do differ in the predicted direction (20% vs 33/36%), but with
n=5 and n=14 that is not a result. **Item 44's phase reading is withdrawn as a general claim.**
It still stands as a description of the single 251 6:06 episode, which item 46 does not revisit.

### What does separate: how fast the brake is applied

`$CLAUDE_JOB_DIR/tmp/onset.py`. Onset = walking back from the episode to the last frame with
`aEgo >= -0.5` (max 4 s). `entry_rate = (a_min - a_onset) / (t_a - t_onset)`.

| | n | entry rate p50 | mean | p25 | entry duration p50 |
|---|---|---|---|---|---|
| 242 NO RADAR | 25 | **-0.74** | -1.01 | -1.48 | **2.40 s** |
| 251 RADAR | 3 | -1.95 | -2.86 | -5.10 | 0.61 s |
| 24f RADAR | 11 | -1.45 | -1.44 | -2.33 | 1.30 s |
| pooled RADAR | 14 | **-1.54** | -1.74 | | |

Mann-Whitney U (normal approximation, tie-corrected) on pooled radar vs no-radar entry rate:
**U=99.0, z=-2.23, p=0.026.** The radar routes reach their peak deceleration about twice as
fast. The no-radar route spends a median 2.40 s getting from -0.5 to its peak; 24f takes
1.30 s and 251 takes 0.61 s.

This **reconciles** the contradiction in the checkpoint. Item 45's comparison established that
no-radar 242 is harsher on every magnitude and jerk metric (worst -4.57 vs -3.92, 1.08
episodes/min vs 0.40, 4.42% of frames below -1.5 vs 2.68%, |j| p99 7.72 vs 6.29). Both are
true at once: the no-radar brakes are **deeper but entered gradually**, the radar brakes are
**shallower but entered abruptly**. "Too reactive" is a statement about the entry, not the
peak, and the entry is where the two configurations actually differ.

The narrower 0.3 s onset-jerk window does NOT reach significance (no-radar p50 -0.63, radar
p50 -1.86, U=132.0, z=-1.26, **p=0.208**) — the difference lives in the whole ramp from -0.5 to
the peak, not in the first three frames. Do not quote the 0.3 s figure as the result.

### What this does NOT establish

- **No mechanism.** This is an output-side statistic on published `aEgo`. It does not say
  whether the fast entry comes from the radar lead's `vRel` driving a larger `x_obstacle` step,
  from `d1` discontinuities at track handover, from the ECO clamp release traced in item 45, or
  from BLoTv3's jerk-cost softening firing more often with a radar lead. Each is a separate
  measurement.
- **n=14 radar episodes, two routes, one of them contributing three.** 251's 0.61 s median is
  three episodes. The pooled p=0.026 rests mostly on 24f.
- **Confounds carried from item 45 are unchanged:** mean `vEgo` 18.0 (242) vs 21.1 (251) vs
  18.2 (24f), different traffic, 1385 s vs 448 s vs 831 s, and 242's `src` histogram is
  lead1-heavy where the radar routes are lead0-heavy. A no-radar drive on the same road at the
  same speed does not exist in the corpus.
- **No road evidence of any kind for a change.** Nothing here has been replayed and no code has
  been touched. The next step is a replay that reproduces the entry-rate difference on 251 with
  the radar path instrumented, not an edit.

## 47. The abrupt radar brake entry is generated in the PLANNER, not downstream, and it is not track handover

Item 46 established that radar brake episodes reach their peak roughly twice as fast as no-radar
ones, but measured only the published `aEgo` — an output-side statistic that cannot tell a planner
cause from an actuator/brake-response one. This item measures the plan itself.

**Method.** `entry_dump.py` (job-local, not committed). Same episode definition as item 46
(`aEgo < -1.5` held > 0.15 s, engaged, `vEgo > 5`). For each episode, the entry rate is computed
independently for `aEgo` and for `aTarget` (the planner's published target), each with its own
onset: peak = the signal's own minimum in the episode, onset = walking back from that peak while
the signal stays below -0.5, at most 4.0 s. **Note the definitional refinement:** item 46 walked
back from the episode start, this walks back from the peak. That shifts the no-radar `aEgo` p50
from -0.74 to -0.65 and the pooled radar p50 from -1.54 to -1.55; the separation is unchanged.

| | n | `aEgo` entry p50 | `aTarget` entry p50 | ratio |
|---|---|---|---|---|
| 242 NO RADAR | 25 | -0.65 | -0.41 | 0.63 |
| 251 RADAR | 3 | -1.66 | -1.79 | 1.08 |
| 24f RADAR | 11 | -1.40 | -1.04 | 0.75 |
| pooled RADAR | 14 | **-1.55** | **-1.25** | 0.81 |

Mann-Whitney U (tie-corrected normal approximation), pooled radar vs no-radar:
- `aEgo`: U=106.0, z=-2.02, **p=0.043**
- `aTarget`: U=85.0, z=-2.64, **p=0.008**

**The plan separates more strongly than the output.** `aTarget` already carries the full entry
asymmetry — about 0.8 of the `aEgo` rate on radar routes and 0.63 on the no-radar route, so the
ratio is if anything *lower* where the entry is slow. There is no configuration-dependent
downstream amplification: whatever makes a radar brake abrupt is present in the planner's own
commanded target before the actuator sees it. The planner investigation is pointed the right way.

**Track handover is ruled out as the driver.** Over each entry ramp the dump counted distinct
`tid1` values and the largest frame-to-frame step in `d1`, `vRel1`, `nat_d1`, `nat_vRel1`. Of the
14 radar episodes, 11 ran on a single track id with steps under 2 m and 2 m/s. The three that did
show handover are the three with the *slowest* entries: 24f t=99.01 and t=100.01 (4 ids, 28.50 m
`d1` step, entry -0.80 and -0.62) and 24f t=367.38 (5 ids, 8.62 m and 7.44 m/s steps, entry
-0.37). Every fast entry (-1.40 or steeper) occurred with `nTid=1`. A discontinuity at handover
does exist in the data and is worth its own item, but it is anti-correlated with abrupt entry.

**What this does NOT establish.** No mechanism inside the planner is identified — this narrows the
search to the planner and eliminates one of item 46's four candidates, nothing more. Two candidates
remain untested: radar `vRel` driving a larger `x_obstacle` step (`long_mpc.py:1087`) and the
BLoTv3 jerk-cost softening firing more often. The item-45 ECO clamp remains a phase-lag-only
effect and cannot attenuate, so it cannot be the cause of a *faster* entry. n is still 14 radar
episodes over two routes with 251 contributing three, the speed/traffic/duration/`src` confounds
of item 45 are unchanged, and there is no road evidence and no replay of any kind.

## 48. BLoTv3 is exonerated: neither supervisor dial is active on a fast brake entry. Only the x_obstacle path remains.

Item 47 narrowed the abrupt radar brake entry to the planner. Item 46 named two surviving
candidate causes inside it: the BLoTv3 supervisor softening its jerk cost more often, or radar
`vRel` driving a larger `x_obstacle` step. This item tests the supervisor and clears it.

**First, a correction to the method I had planned.** The checkpointed plan was to read the
supervisor's `t_follow_pad` as `desFollow - tFollow`. **That is wrong: `desFollow` is a distance in
metres, not a following time.** Across all three routes `desFollow - vEgo*tFollow` has p50 +3.1 to
+7.4 m and p10/p90 of roughly -9/+19 m, and `desFollow` is exactly 0.00 on every `src=cruise`
frame — it is the desired follow *gap*. Any figure of the form `desFollow - tFollow` is
meaningless; the first run of `pad_entry.py` produced such figures (pad "p50 41.55", U tests at
p=0.965 and p=0.176) and they are **withdrawn, not evidence of anything.**

**The pad is directly observable as `tFollow` itself**, whose base is 1.45 s on these routes.

| | engaged frames | `tFollow` = 1.45 | other values | entry ramps with `tFollow` > 1.45 |
|---|---|---|---|---|
| 242 NO RADAR | 17834 | 98.55% | 26 distinct values, 0.79–1.42, 1–5 frames each | **0 / 25** |
| 251 RADAR | 2721 | 100.00% | none | **0 / 3** |
| 24f RADAR | 9660 | 84.93% | 1.75 (14.93%), 1.25 (9 fr), 1.65 (1 fr), 0.00 (4 fr) | **1 / 11** |

The one hit is 24f t=449.08 (entry -2.69, pad +0.30). It is almost certainly **not** a supervisor
pad: 24f's 1.75 s frames form a single 1442-frame block with one intermediate frame at 1.65 in the
whole route, which is a step change in the gap setting, not a pad slewed at `ONSET_RATE_UP` = 0.8/s
(that would leave roughly six intermediate frames per transition). The scatter of sub-1.45 values
on 242 is below the base and so cannot be a pad at all. **On no route does the t_follow pad
participate in a brake entry.**

**The jerk cost is observable too, and it clears the supervisor on the other dial.** `accJerk` is
250 and `spdJerk` 5.50 for 98–99% of engaged frames; the softened values are `accJerk` 50 (0.2x) on
235 frames of 242 and 148 of 24f, 125 (0.5x) on 9 frames of 24f, and `spdJerk` 2.75 on those same 9.
**251 — the route with the fastest entries in the corpus — never softens at all.** Counting
softened frames inside each entry ramp:

- 242: 4 ramps, at t=137.72 / 138.22 / 138.82 / 139.71, entry rates **-0.33 / -0.28 / -0.24 / -0.20**
- 24f: 2 ramps, at t=99.01 / 100.01, entry rates **-0.80 / -0.62**
- 251: **none**

Those six are the six slowest entries in the whole 39-episode corpus. As with track handover in
item 47, softening is not merely absent from the fast entries — it is **anti-correlated** with
abruptness, and it fires slightly more often on the no-radar route (1.34% of frames vs 1.56% on 24f
and 0% on 251). BLoTv3 does not cause the fast entry on either of its dials.

**What this leaves.** One candidate from item 46 survives: radar `vRel1` driving a larger or
steppier `x_obstacle` (`long_mpc.py:1087`) than vision `vRel1` does. That is now the whole
remaining hypothesis and it has not been tested.

**What this does NOT establish.** The softened-frame counts are an association over 39 episodes,
not a causal test, and `jerk_scale` itself is not logged — this reads it through `accJerk`, which
also carries whatever else scales the MPC cost. `0.2x` does not correspond to any bound in
`blotv3.py` (`JERK_SCALE_MIN` is 0.3, which would give 75), so the mapping from `jerk_scale` to
`accJerk` is assumed, not verified. n is still 14 radar episodes over two routes with 251
contributing three; the speed/traffic/duration/`src` confounds of item 45 are unchanged; and there
is no road evidence and no replay of any kind.

## 49. The x_obstacle path is exonerated too: the radar obstacle is SMOOTHER, not steppier. All four candidates from item 46 are dead.

Item 48 left one candidate standing: radar `vRel1` driving a larger or steppier `x_obstacle`
(`long_mpc.py:1087`) than vision `vRel1` does. It does not. The radar obstacle is measurably
*less* jagged than the vision obstacle, in the opposite direction to the hypothesis.

**What the solver actually sees.** Traced from the source, not assumed. At horizon index 0 the
lead cost residual (`long_mpc.py:374`) reduces to

    e = (x_obstacle - x_ego) - desired_dist_comfort
      = dRel + v_lead^2/(2*CB) - ( v_ego^2/(2*CB) + t_follow*v_ego + STOP_DISTANCE )
      = dRel - desired_follow_distance(v_ego, v_lead, t_follow)
      = d1 - desFollow          <- both already logged, both in metres

with `COMFORT_BRAKE = 2.5` and `STOP_DISTANCE = 6.0` (`:155-156`), the obstacle built at `:951`
as `lead_xv_0[:,0] + get_stopped_equivalence_factor(lead_xv_0[:,1])`, and `process_lead` feeding
it `lead.dRel` and `lead.vLead`. So `d(x_obstacle)/d(v_lead) = v_lead/2.5` metres per m/s -- **10 m
of obstacle motion per 1 m/s of lead-speed error at 25 m/s.** That gain is real and is why the
hypothesis was worth testing: radar `vRel` is a Doppler measurement, vision `vRel` a regression.

The reconstruction was validated before use, the check item 48's premise did not get:
`desFollow - recon` p50 -0.74 m (242) / -0.50 (251) / -0.45 (24f), p10/p90 within about +-2 m of
gaps of 40-100 m. The small systematic offset is the `lead_v_filter` that `process_lead` applies
to `v_lead` and the logged `desFollow` does not see.

**Measurement** (`obst_entry.py`), over the same `aEgo` entry ramp `[k0,ka]` as items 47-48, per
episode: the slope of `e`, of its geometric part `d1`, and of its lead-speed part `v_lead^2/5`;
the slope of `vRel1` itself; and `jag`, the mean absolute second difference per frame, which
measures steppiness independently of slope.

|  | n | entry p50 | de/dt | d(d1)/dt | dV/dt | d(vRel)/dt | jag V | jag e |
|---|---|---|---|---|---|---|---|---|
| 242 NO RADAR | 25 | -0.65 | -1.51 | -2.44 | -7.75 | -0.38 | 1.743 | 2.400 |
| 251 RADAR | 3 | -1.66 | +0.79 | -3.07 | -12.84 | -0.31 | 1.387 | 1.548 |
| 24f RADAR | 11 | -1.40 | +0.33 | -2.91 | -1.24 | +0.57 | 0.582 | 1.195 |
| pooled RADAR | 14 | -1.55 | **+0.33** | -3.07 | -5.33 | +0.35 | **0.867** | 1.244 |

Mann-Whitney, pooled radar vs no-radar:

- `de/dt`  U=230.0 z=+1.61 **p=0.107** -- not significant, and the sign is backwards: the residual
  *grows* on radar entries while it shrinks on no-radar ones.
- `d(d1)/dt` U=203.0 z=+0.82 p=0.412. `dV/dt` U=212.0 z=+1.08 p=0.279. `d(vRel)/dt` U=230.0
  z=+1.61 p=0.107. None significant.
- `jag V` U=100.0 z=-2.20 **p=0.028** -- significant, **radar SMOOTHER** (0.867 vs 1.743).
- `jag e` U=124.0 z=-1.49 p=0.135, same direction.

**The horizon, not just frame 0.** The residual above is only the first horizon point; the solver
sees the whole trajectory that `extrapolate_lead` builds from `aLeadK` and `aLeadTau`. So the
extrapolation input was checked the same way: `jag aLeadK1` p50 0.0871 radar vs 0.1116 no-radar,
U=172.0 z=-0.09 **p=0.930**; `jag mProb1` p=0.128; `d(aLeadK)/dt` p=0.380. Nothing steppier on the
radar side there either.

**This is the third consecutive anti-correlation.** Track handover (item 47), jerk-cost softening
(item 48) and now obstacle steppiness all appear *less* on the fast radar entries than on the slow
no-radar ones. Three independent mechanisms have now been eliminated by the same shape of evidence.
All four candidates enumerated in item 46 are dead: the ECO clamp (45: phase-lag only, cannot
attenuate, so cannot produce a faster entry), track handover, the BLoTv3 dials, and the obstacle path.

**What this does NOT establish.**
- It does not explain the abruptness. Item 47's result stands -- the entry asymmetry is in
  `aTarget`, so it is generated inside the planner -- but no mechanism inside the planner has been
  found, and CSV-level inference has now exhausted its candidate list.
- `de/dt` at n=14 vs 25 is underpowered; p=0.107 on a backwards sign is "no evidence", not
  "evidence of no effect". The `jag` result is the one that carries a direction.
- The reconstruction is horizon index 0 only. A mechanism that reshapes the *interior* of the
  extrapolated trajectory without changing `aLeadK`, `aLeadTau`, `dRel` or `vLead` would be
  invisible to this measurement. Reading the MPC's own solved trajectory requires replay.
- Confounds unchanged from 43-48: different drives, traffic, and 251 contributing only 3 episodes.
- No road evidence and no replay. Nothing here was validated against a driven car with real radar
  targets, and no code has been changed.

**Next move is therefore replay instrumentation, not further CSV work.** The quantities that would
settle it -- `self.params[:,2]` per horizon point, the solved `a` trajectory, `lead_xv_0` -- are not
in the log and must be captured by re-running the planner over the route.

## 50. The lead-trajectory path switches mid-entry, but not differently on radar. Fourth consecutive null; CSV inference is closed.

Item 49 closed with a stated limitation: the obstacle residual was reconstructed at horizon index 0
only, so a mechanism reshaping the *interior* of the extrapolated trajectory would be invisible.
Reconstructing all 13 points is pure numpy -- everything from `radarState` to `params[:,2]`
(`process_lead` -> `extrapolate_lead` -> stopped equivalence -> cruise obstacle -> `np.min`) runs
without acados; only `run()` needs the solver. So the interior looked answerable from the existing
CSVs. It is not, and the reason is worth recording.

**`extrapolate_lead` is not always what builds the trajectory.** `build_model_lead_trajectory`
(`long_mpc.py:169`) replaces it with the model's `leadsV3` horizon, anchored to the raw h=0
measurement (`x_lead_traj = raw_d_rel + (model_x - model_x[0])`). It declines, returning `None`, when

```python
raw_lead_brake = max(0.0, -aLeadK)
closing_speed  = max(0.0, v_ego - raw_v_lead)
ttc            = raw_d_rel / max(closing_speed, 1e-3) if closing_speed > 0.1 else inf
if (raw_lead_brake > MODEL_LEAD_TRAJECTORY_MAX_LEAD_BRAKE or        # 0.5
    (closing_speed > 0.75 and ttc < MODEL_LEAD_TRAJECTORY_MAX_CLOSING_TTC)):   # 7.0
  return None
```

with the comment "keep the legacy raw-lead path so an optimistic model horizon cannot delay the first
braking response." That guard reads like it must fire on every brake entry. It does not.

**Measured: 14 of 39 entry ramps contain at least one model-path frame** -- 242 8/25, 251 1/3,
24f 5/11 -- and the per-ramp live fraction ranges 0.067 to 1.000, so on those ramps the obstacle
*switches definition mid-entry*. Whole-route model-path occupancy is 77-82% of engaged frames on all
three routes; the guard is a brake-entry exception, not the normal case.

**This figure is an upper bound, not a measurement.** The checks the CSV cannot model -- `model_x` /
`model_v` shape against `LEAD_T_IDXS_MODEL`, and the `isfinite` guards -- can only make the function
return `None` *more* often. A frame counted as legacy is therefore definitely legacy; a frame counted
as model-path is only *possibly* live. Tightening 14/39 requires `modelV2.leadsV3` from the rlog and
cannot be done from the dumped CSV. (Field note: `model_lead.prob` is the CSV's `mlProb`, not
`mProb1` -- the latter is the radar track's own modelProb.)

**But the switching does not separate radar from no-radar.** Per entry ramp, over the `aEgo` ramp
`[onset, peak]` as item 46's `rate()` defines it, pooled radar (251 + 24f, n=14) vs no-radar
(242, n=25), Mann-Whitney with radar first:

| metric | radar p50 | no-radar p50 | U | z | p |
|---|---|---|---|---|---|
| model-path live fraction | 0.0000 | 0.0000 | 190.0 | +0.51 | 0.609 |
| path flips per frame | 0.0000 | 0.0000 | 189.5 | +0.51 | 0.613 |

Flips-per-frame p50 is 0.0000 on all three routes (max 0.2143 on 242, 0.0909 on 251, 0.0769 on 24f).

**The tail is more decisive than the average.** Of the five fastest entries in the corpus, four are
100% legacy with zero flips:

| route | t | entry rate | live fraction | flips | onset -> peak |
|---|---|---|---|---|---|
| 251 | 365.86 | **-5.39** | 0.583 | 1/11 | legacy -> MODEL |
| 242 | 1405.25 | -2.71 | 0.000 | 0/26 | legacy -> legacy |
| 24f | 449.08 | -2.69 | 0.000 | 0/15 | legacy -> legacy |
| 24f | 595.74 | -2.51 | 0.000 | 0/21 | legacy -> legacy |
| 242 | 816.42 | -2.45 | 0.000 | 0/30 | legacy -> legacy |

The one fast entry that touches the model path, 251 t=365.86, **starts on the legacy path and flips to
MODEL at the peak**. The flip is downstream of the brake onset, not upstream of it, so it cannot have
caused an entry that was already underway.

**This is the fourth consecutive null, and the interior measurement was abandoned rather than run.**
Item 47 ruled out track handover (the three handover episodes are the three slowest). Item 48 ruled
out the BLoTv3 dials (jerk softening appears only in the six slowest ramps). Item 49 ruled out the
obstacle inputs, with `jag V` significant in the *opposite* direction (p=0.028, radar smoother). This
item rules out the path switch. The 13-point interior comparison was *not* run: 25 of the 39 ramps are
fully legacy and reconstructible, but that subset excludes 251 t=365.86 -- the exemplar the whole
investigation rests on -- and drops n to 8 vs 17, below the already-underpowered 14 vs 25. Measuring a
weaker question on a sample chosen to exclude the phenomenon is not evidence; it is the confirmation
trap in a new shape. The plan that specified this work
(`docs/superpowers/plans/2026-09-22-obstacle-trajectory-interior.md`, commit `1206eacbe`)
pre-registered that fork and it was taken as written.

**What this does NOT establish.**
- It does not explain the abruptness. Item 47's finding stands -- the asymmetry is in `aTarget`, so it
  is generated inside the planner -- and no mechanism has been found.
- 14/39 is an upper bound on model-path involvement, so "the path switches mid-entry" is established
  but its true frequency is not. Two of the four unmodelled guards could in principle void every
  model-path frame counted here.
- p=0.609 and p=0.613 at n=14 vs 25 are "no evidence of a difference", not "evidence of no
  difference". The tail argument above carries more weight than either p-value.
- The interior of the extrapolated trajectory is still unmeasured. This item declines to measure it
  on a biased subset; it does not show it would be null.
- Confounds unchanged from 43-49: different drives, traffic, and 251 contributing only 3 episodes.
- No road evidence, no replay, no code changed.

**The remaining suspect is the cost function, not the inputs.** Every input to the MPC lead cost has
now been checked at frame 0 and found equal or smoother on radar, while the *output* (`aTarget`)
separates more strongly than `aEgo` does (item 47). If clean inputs produce a steeper plan, the
mapping is what differs -- the lead cost at `long_mpc.py:374` and the danger term at `:389`, whose
`1/(v_ego + 10.)` normalisation and `danger_factor` weighting are the only remaining unexamined
transforms between the obstacle and the solved acceleration. That requires acados and a replay
harness; it cannot be read from a log. A fifth CSV probe would be the confirmation trap.

**Addendum to 50: the danger factor is a constant and is ruled out.** Before escalating to a replay
harness, the last logged input to the cost function was checked, because the danger slack constraint at
`long_mpc.py:389` scales the whole of `desired_dist_comfort` by `lead_danger_factor` and that product
is tens of metres at highway speed. The CSV's `danger` column is **identically 0.75 on all 30,156
engaged frames of all three routes** -- one distinct value, zero range on every entry ramp, radar and
no-radar Mann-Whitney U=175.0 z=+0.00 p=1.000 at onset, at peak and on range. It never moves, so it
cannot differentiate anything. `LEAD_DANGER_FACTOR` is doing no work here.

That leaves the lead cost at `:374`, `((x_obstacle - x_ego) - desired_dist_comfort) / (v_ego + 10.)`,
whose numerator is item 49's residual `e` (measured, radar not steeper) and whose denominator depends
only on `v_ego`. Both terms are therefore accounted for, which sharpens the open question rather than
widening it: if the inputs, the residual, the danger factor and the trajectory path are all equal or
smoother on radar while `aTarget` separates, the difference is in how the **solver** responds -- the
runtime cost weights set in `set_weights()`, the slack penalties, or conditioning -- not in any
quantity the log contains. Nothing short of running acados over both routes will separate those.

## 51. The follow policy's matched-regime exit is real in one replay window but absent from the corpus. Fifth null.

**Method (replay + offline, no code changed).** A whole-planner replay of 251 segments 5+6 (mode
`x0+src+pa+oat`) traced every line after which `update()`'s local `output_a_target` changed inside
t 365.5–366.3. While `lead_follow_policy._matched` is True, `_matched_brake_floor` lifts the MPC's
≈ −2.8 request to ≈ −0.10. `_transition_target` is then allowed, because −0.10 > −0.25 =
`FOLLOW_TRANSITION_MAX_BRAKE`, and it emits `prev + up_step` (0.07–0.13 per frame), replacing the MPC's
braking. At t 365.89 `_matched` flips False, and the replayed output steps −2.582 → −1.820. The
replay reproduces the matched-phase follow-policy output to 0.000. `action_t` was also checked:
it is a constant 0.550 on the tinygrad `acc` branch with no e2e blend, so it is exonerated.

The replay's negative control still fails: p99 0.159 against a gate of 0.05, and the residual is
100% trajectory (dtraj). So the +0.76 step at the flip is **replay-only**. In the log, the largest
`aTarget` step at that flip is 0.13.

**Corpus test** (`flip_entry.py`, calling the real `lfp._matched` on the logged lead1 fields and
`tFollow`). It counts True→False flips in [onset − 0.5 s, peak] of every aEgo entry ramp:

| route | engaged frames matched | entries | entries with a flip | any matched on ramp |
|---|---|---|---|---|
| 242 no radar | 7.1% | 25 | 0 | 1 |
| 251 radar | 34.4% | 3 | 1 (t 365.86) | 1 |
| 24f radar | 14.3% | 11 | 0 | 0 |

The premise holds: matched is 2–5× more frequent on radar. The mechanism does not follow from it.
Only 1 of 14 radar entries has a flip; incidence is 0/25 vs 1/14, MWU p=0.71. With that episode
removed, radar entry rate is p50 −1.44 against 242's −0.65, so the steepness difference survives
without any flip. The matched regime belongs to steady follow, not to brake entries. It explains at
most the single 251 t 365.86 event and not the "too reactive with radar" feel.

**Still open:** the dtraj control failure (the replay brakes harder than the car in 365.70–365.80,
choosing `cruise` where the car chose `lead1`). It points at solver-side terms — `set_weights()`,
slack penalties, constraints — as item 50 concluded. Replay evidence only; nothing here is
road-validated.

## 52. The replay control failure is mostly a harness gap: BLoTv3 was on in the car, off in the replay

**Finding (replay, 251 segs 5+6, mode `x0+src+pa+oat`).** `LongitudinalPlanner._blotv3_active()`
(`longitudinal_planner.py:583`) reads `Params().get_bool("BlotV3")`. The docker replay has an empty
param store, so it returned False and `_blotv3_policy` was None, meaning jerk_scale 1.0 and no BLoTv3
t_follow. `initData` shows `BlotV3 = 1` on **all three** comparison routes (242, 24f, 251), so the
car ran BLoTv3 every time. Forcing it on in the harness (`BLOT=1`, which sets
`planner._blotv3_active = lambda: True`) gives:

| mode | p50 | p90 | p99 | max |
|---|---|---|---|---|
| before (BLoTv3 off) | 0.00094 | 0.0308 | 0.1587 | 0.1725 |
| BLOT=1 | 0.00024 | 0.0136 | 0.1063 | 0.1557 |

During 365.0–365.9 the replay's jerk_scale runs 0.900 down to 0.825, which lowers acceleration_jerk.
The gate (p99 < 0.05) **still FAILS**. The new worst frame is 364.50 (replay −0.077, logged −0.233),
in a different place from before. The 0.10629 p99 is also the floor that several weight sweeps below
hit, so it is probably a separate residual.

**Sensitivity tests that led here** (all replay-only, harness env overrides on `set_weights`):
- `uncertainty` is unlogged, from `uncert_slow`, and runs 0.484–0.514 in the window. It is **not the cause**. In-band
  values 0.50/0.52 give p99 0.159/0.157, and forcing it out of the rescale band (0.44 or 0.60) makes
  p99 **worse**, at 0.177.
- `lead_dist` = 20 gives p99 0.160, and `panic_bypass` forced False gives 0.1587. Neither is the cause.
- `speed_jerk` ×2/×3/×5 gives p99 0.135/0.106/0.106. `acceleration_jerk` ×0.5/×0.33 gives 0.106, and ×3 gives 0.180,
  but every one of these raises p50 by 2–6×. The car behaved as if acceleration_jerk were lower
  than logged, and BLoTv3's jerk_scale is exactly that term.
- An earlier batch of these runs was a silent no-op. zsh does not word-split an unquoted `$1`, so
  `-e UNC_FORCE=0.44` reached docker as one argument and set an env var named ` UNC_FORCE`. Only
  the `${=1}` reruns are counted above.

**Consequence.** Every replay-based measurement in items 46–51 ran with BLoTv3 off. The
live-log and CSV corpus statistics are unaffected, including the matched-flip corpus count in 51.
Replay-derived claims should be re-checked under `BLOT=1`, especially STATUS 51's +0.76 m/s² replay-only step at 365.89.
This does **not** confound the 242 vs radar comparison, because BLoTv3 was on for all three.

Replay evidence only; nothing here is road-validated. No planner code was changed.

## 53. The replay now nearly reproduces the car: the 364.50 miss was stale radar input, not BLoTv3

**Method (replay only, no code changed).** Same harness and window as item 52 (251 segments 5+6,
mode `x0+src+pa+oat`, `BLOT=1`).

1. **`desFollow` cannot check the BLoTv3 pad.** It is `int(desired_follow_distance(v_ego, vLead,
   self.t_follow))` from `starpilot_following.py:123`, in metres, built from StarPilot's own
   `t_follow` before BLoTv3 adds its pad. Neither `tFollow` nor `desFollow` records the pad.
2. **BLoTv3 timing is not the 364.50 cause.** Harness overrides changed each BLoTv3 output on its own:
   jerk_scale lag of 1 and 3 frames, offsets of ±0.1, pad ×0 and ×2, and jerk_scale forced to 1.
   None moved 364.50: the replay stayed at −0.076 to −0.082 against −0.233 logged.
3. **The cause is input alignment.** At 364.50 the car's `leadTrajectoryX0[0]` is 42.67 m. That is
   the replay's value one frame later, at 364.55. The replay itself had 43.23 m. The frames either
   side match to 0.00, so the car's planner had already drained the next `radarState`.
   - The harness now accepts `LPCUT=1`, which gives each modelV2 frame every message logged before
     the car's own `longitudinalPlan` for that frame. This emulates the SubMaster drain at the
     observed cut-off.

| run | p50 | p90 | p99 | max | worst frame |
|---|---|---|---|---|---|
| BLoTv3 off (item 51) | 0.00094 | – | 0.159 | – | – |
| `BLOT=1` (item 52) | 0.00024 | 0.0136 | 0.106 | 0.156 | 364.50 |
| `BLOT=1 LPCUT=1` | 0.00028 | 0.0038 | **0.059** | **0.060** | 366.45 |
| `LPCUT=1`, BLoTv3 off | 0.00097 | 0.0302 | 0.159 | 0.173 | 366.00 |

- The MPC source mismatch is now 0 of 141 frames. The item 51 open point ("the replay chose
  `cruise` where the car chose `lead1`, 365.70–365.80") is closed: it was the same stale input.
- **Still failing:** the gate is p99 < 0.05, and the remaining error is 366.20–366.60. The replay
  recovers too fast, by up to +0.060, during cruise, while BLoTv3's pursuit tail holds jerk_scale at 0.30.
  - The error depends on the unlogged BLoTv3 state. JS_OFF=+0.1 gives p99 0.0502, max 0.061, and moves
    the worst frame to 366.05. JS_OFF=+0.3 makes it worse (0.113).
  - Without BLoTv3 state in cereal, this is as close as the replay gets.
4. **The 365.89 step from item 51 re-checked.** With inputs aligned it reproduces to −0.003. It is
   the follow policy's output diverging from the planner's internal `a_desired`, which drops
   −2.86 → −1.82. The logged `aTarget` moved only 0.13, so item 51's corpus conclusion stands.

**Consequence.** Items 46–51 used a replay with BLoTv3 off and stale inputs. Their log-only and corpus
conclusions are unaffected. Replay-derived numbers from those items should be re-derived with
`BLOT=1 LPCUT=1` before anyone relies on them.

**No mechanism that separates radar from no-radar has been found.** No code change is proposed.
The next test the harness can now support: replay a 251/24f brake entry with the radar lead replaced
by the vision lead, keeping the planner fixed, and check whether the entry rate falls to 242's
level. If it does, the cause is in the input; if not, it is in the planner.
Replay evidence only; nothing here is road-validated.

## 54. Swapping the radar lead for the vision lead moves the 251 brake entry by about one frame, not its shape

**Method (replay only, no code changed).** Harness env `VISLEAD` rewrites every `radarState` fed to
the planner:
- `VISLEAD=1` replaces leadOne and leadTwo with radard's own vision-only lead, as
  `get_RadarState_from_vision` builds it (radard.py:663): prob > 0.35, dRel = x − 1.52, the
  model-relative vRel, the 0.8/0.2 aLeadK blend, and radar=False.
- `VISLEAD=2` keeps the radar flags, to separate the planner's radar-only branches from the value
  change.

The planner is unchanged. The run uses 251 segments 5+6 with `BLOT=1 LPCUT=1` and the one-step x0
anchor, so every frame starts from the car's real state. This is the per-frame command the
planner would have issued, not a free-running trajectory.

| t | logged | radar (base) | vision (1) |
|---|---|---|---|
| 364.50 | −0.233 | −0.236 | −0.062 |
| 365.00 | −1.006 | −1.006 | −0.746 |
| 365.20 | −2.314 | −2.314 | −1.852 |
| 365.50 | −3.198 | −3.198 | −3.078 |
| 366.00 | −1.393 | −1.393 | −1.710 |
| 366.30 | −0.458 | −0.409 | −0.714 |

- **Onset:** the radar lead starts braking about 0.1 s earlier. It crosses −1.0 and −2.0 about
  0.06 s (one frame) before the vision lead does.
- **Peak and slope:** the peak is the same (−3.20 against −3.14), and so is the slope. On the release
  the vision lead brakes harder, not softer.
- **Radar-only planner branches:** `VISLEAD=2` matches `VISLEAD=1` to within 0.003 except at one frame
  (366.30), so the radar-only branches play almost no part here.

**Reading.** In this entry the radar input advances the braking by about one frame; it does not make
it harsher. That does not explain a "too reactive" feel. The limits are that this is one entry, a
per-frame test rather than a free run, and replay only. The 11 entries on route 24f are the next
place to test.

## 55. On route 24f, the vision lead brakes as deep as the radar lead and often more steeply

This repeats the item 54 test on all 11 of 24f's brake entries. The harness and settings are
unchanged: `BLOT=1 LPCUT=1`, mode `x0+src+pa+oat`, and a vision run with `VISLEAD=1`. The route
was split into seven groups of 2–3 segments, and each group was warmed about 10 s before its
first entry.

**The harness reproduces the car on 24f.** Six of the seven groups pass the negative-control gate
(p99 below 0.05):

| group | A | B | C | D | E | F | G |
|---|---|---|---|---|---|---|---|
| p99 | 0.016 | 0.002 | 0.006 | 0.005 | 0.001 | 0.032 | 0.001 |

Group A has a single frame at 0.59, during warm-up. On every entry, the radar (base) metrics match
the logged ones to 0.03.

The table covers the window [peak − 4 s, peak + 1 s]:

- **min:** the lowest aTarget in the window.
- **t−1, t−2:** the first times aTarget reaches −1 and −2.
- **slope:** the steepest aTarget change over 0.25 s, in m/s³.

| peak | radar min | vision min | radar t−1 / t−2 | vision t−1 / t−2 | radar slope | vision slope |
|---|---|---|---|---|---|---|
| 99.0 | −1.57 | −1.57 | 97.96 / – | 97.96 / – | −0.65 | −0.65 |
| 100.0 | −1.59 | −1.59 | 97.96 / – | 97.96 / – | −0.65 | −0.65 |
| 151.2 | −0.91 | −0.91 | – | – | −4.70 | −4.59 |
| 170.2 | −3.47 | −3.45 | 169.31 / 169.56 | 168.16 / 169.36 | −4.54 | −5.13 |
| 174.6 | −2.47 | −2.50 | 170.62 / 170.62 | 170.62 / 170.62 | −1.66 | −3.53 |
| 272.7 | −3.45 | −3.50 | 271.52 / 272.07 | 270.72 / 272.02 | −6.53 | −7.47 |
| 367.4 | −3.47 | −3.45 | 363.42 / 363.42 | 363.42 / 363.42 | −3.89 | −4.12 |
| 449.1 | −2.09 | −2.58 | 448.33 / 448.78 | 448.13 / 448.33 | −2.86 | −5.12 |
| 595.7 | −2.47 | −2.42 | 594.48 / 595.03 | 594.43 / 594.83 | −2.09 | −5.72 |
| 699.4 | −1.25 | −1.20 | 698.74 / – | 698.64 / – | −0.67 | −3.62 |
| 731.9 | −3.46 | −3.50 | 731.24 / 731.64 | 730.44 / 731.39 | −6.89 | −5.94 |

- **Depth:** the depth is the same (median difference 0.00). Four entries bottom out near −3.5
  either way, which is the brake limit, so the lead source does not set how deep they go.
- **Timing:** the vision lead reaches −1 and −2 at the same time or earlier. For −2 its median
  lead is 0.20 s, and at 170.2 it reaches −1 1.15 s earlier. On 251 the radar lead was about
  one frame earlier.
- **Steepness:** the vision lead is steeper in 7 of the 11 entries, gentler in 1 (731.9), and
  equal in 3. The median radar − vision slope is +0.60 m/s³.

**Reading (replay only).** Across 12 entries on two radar routes, 251 and 24f, the radar lead
never makes the planner brake harder, earlier or more steeply than the vision lead would have on
the same drive. That rules out one explanation: the lead measurement that radar supplies is not what
makes these entries abrupt. On 24f the vision lead is the more reactive of the two.

This matters for the "too reactive" symptom when compared with 242 (no radar; aTarget entry-rate
p50 −0.41, against −1.04 on 24f). That gap is not caused by the planner reacting to a radar
lead in place of a vision one. The likely difference is the situations each route contains:
closing speed, cut-ins and headway at onset. It could also come from something that differs
between the drives but not the lead source, such as personality, the BLoTv3 pad or matched-follow
exits.

Limits:
- The test is per-frame, anchored to x0, so the car's actual response is not simulated.
- The vision lead uses radard's vision-only estimate, not a real no-radar drive.
- The evidence is replay only.

**Next.** Match entries between 242 and the radar routes by their situation at onset (vEgo,
closing speed, headway, whether a lead was newly acquired), then compare the entry rates within
matched pairs. Do not propose a planner fix until a matched comparison shows a gap that remains.

## 56. 242 and the radar routes were driven with the same settings; the current code reproduces 242 exactly, and the gap comes from the situations

Three tests followed item 55. All are replay or offline.

**1. Matching entries by their situation at onset.** Script: `match_entries.py` in the harness
directory.

Each aTarget brake entry is described at onset by:
- vEgo;
- closing speed from the 1 s range slope;
- headway (d1 / vEgo);
- whether the lead is new (hasLead false, or a d1 jump of more than 4 m, within the previous 2 s).

Nearest-neighbour matching found only 4 pairs within distance 1 going from 242 to radar, and 6 of
14 going from radar to 242. In both directions the radar entry is steeper: a median difference of
−0.29 (4 of 4 pairs) and −0.80.

Before matching, the two sets differ a lot:
- 242 has 8 new-lead entries out of 25; its far vision acquisitions at headway above 4 s are
  gentle, with rates from −0.14 to −0.27. The radar routes have 1 out of 14.
- Among existing-lead entries, the closing speed at onset has a median of 1.29 m/s on 242 and
  2.89 m/s on the radar routes.

The existing-lead entry-rate p50 is −0.49 on 242 and −1.25 on the radar routes. The overall
−0.41 that STATUS used to quote mixes in the gentle new-lead entries.

**2. Software and settings.** The routes differ only in software:

| | 242 | 251 and 24f |
|---|---|---|
| commit | 4d5ba0b | 3505708 |
| BoschARadar | 0 | 1 |

`BlotV3`, `LongitudinalPersonality`, `ExperimentalMode` and the personality profiles are the same.
Two planner changes landed between those commits:
- the far-lead brake limit was removed (1434176b3). It only acted on leads with `radar=True`, so
  it was inert on 242;
- BLoTv3 now reads the MPC target instead of the arbitrated output (552798ab / 084a9d56).

The route 242 entries were replayed on the current code (which includes 3505708) with
`BLOT=1 LPCUT=1`, in seven groups H–N. Six of the seven pass p99 < 0.05. Group M has an excursion
of about 1.0 and group K a p99 of 0.064. Across 11 existing-lead entries, the median difference
between logged and replayed min, slope, rate, t−1 and t−2 is 0.00. Only 1200.5 differs, by −0.24
on min, and it lies inside the group M excursion. **The code change between the drives does not
explain why 242 is gentler.**

**3. Jitter during steady following.** Frames with hasLead, vEgo > 8, |vRel| < 1 and
|aTarget| < 0.8:

| | 242 | 251 | 24f |
|---|---|---|---|
| minutes | 6.8 | 1.3 | 4.0 |
| aTarget jerk p50 / p90 / p99 | 0.07 / 0.24 / 1.01 | 0.03 / 0.19 / 0.61 | 0.04 / 0.31 / 1.81 |
| slope reversals per minute | 43.8 | 26.7 | 41.8 |
| \|ΔvRel\| per frame, p50 | 0.126 | 0.016 | 0.016 |

The radar vRel is about eight times cleaner, and steady following is no jumpier with radar. The
exception is the p99 tail on 24f.

**Reading (replay only).** Put together:
- identical settings;
- the current code reproducing 242 (this item);
- the vision-lead counterfactual not softening the radar entries (items 54 and 55);
- steady following no noisier.

These rule out a planner or radar-input mechanism as the cause of the gap in entry rates. The
radar drives contain harder situations at onset: higher closing speeds and fewer gentle far
acquisitions. When the situations are matched, the pairs that remain still lean steeper on
radar. But n is 4 to 6, and 242's closing speeds come from vision range, which understates
closing at distance (see memory "vision isn't closing-speed truth"). So that residual cannot yet
be separated from measurement bias.

**No fix is proposed.** No change here has evidence that it would reduce a real over-reaction
without also blunting correct braking.

**Next.** What would settle it is road data with situations the analysis can compare: a radar
drive and a no-radar drive over the same road and in similar traffic. Alternatively, the driver
could timestamp the moments that felt too reactive (bookmarks), so the analysis starts from the
felt events rather than from every brake entry.

**Correction to "Next" (2026-09-22).** That paragraph is wrong. 24f and 251 already carry driver
bookmarks, analysed in item 42: every rail-pinned (-3.5) brake on both routes is bookmarked and
nothing else is. The felt events are known; item 57 starts from them.

## 57. Removing the ECO output floor does not soften the bookmarked 24f brakes; it makes them earlier and sometimes harder

Replay only (harness `replay.py`, `BLOT=1 LPCUT=1`, new env `MINACC_FORCE` overriding the fed
`starpilotPlan.minAcceleration`), route 24f, the five bookmarked brakes that fall inside the
replayed groups (170.2, 272.7, 367.4, 449.1, 595.7; the ~625 mark is past the end of group F).
Baseline replays pass the gate (p99 0.0008-0.032). No repo code changed.

Times are seconds relative to the bookmarked peak; rate is the steepest 0.5 s drop of aTarget.

| event | floor | first < -0.3 | first < -1.5 | peak | rate |
|---|---|---|---|---|---|
| 170.2 | ECO -0.5 (as driven) | -2.89 | -0.74 | -3.47 | -4.17 |
| | -1.0 (STANDARD) | -2.89 | -0.74 | -3.47 | -4.17 |
| | -3.5 (none) | -2.89 | -0.99 | -3.50 | -4.92 |
| 272.7 | ECO | -4.38 | -0.78 | -3.45 | -4.25 |
| | STANDARD | -4.38 | -0.78 | -3.45 | -4.25 |
| | none | -4.38 | -0.93 | -3.50 | -4.59 |
| 367.4 | ECO | -4.98 | -4.98 | -3.47 | -2.46 |
| | none | -4.98 | -4.98 | -3.50 | -2.70 |
| 449.1 | ECO | -3.92 | -0.62 | -2.09 | -2.55 |
| | STANDARD | -3.92 | -0.62 | -2.09 | -1.93 |
| | none | -3.92 | -1.17 | **-3.50** | **-6.10** |
| 595.7 | ECO | -1.72 | -0.97 | -2.47 | -1.94 |
| | none | -1.72 | -1.17 | -2.52 | -2.50 |

**Reading.** Braking onset (< -0.3) is identical in every case: the ECO floor does not delay the
start of braking. Removing it moves the hard part 0.15-0.55 s earlier but leaves peak and slope
the same or steeper, and on 449.1 it turns a -2.09 brake into a -3.50 rail brake at -6.1 m/s3.
The STANDARD floor (-1.0) is indistinguishable from ECO except for 449.1, where it is gentler.
So the hold-then-rail shape of items 43/45 is the MPC's own solution arriving late and hard; the
output floor only trims its first ~0.2 s. **Relaxing or removing the ECO floor is not a fix** and
would make at least one bookmarked brake harsher (replay evidence).

**Next.** The lateness is upstream of the output clip. The candidate already on record is the
follow policy (`lead_follow_policy.apply()`, via `longitudinal_planner.py:3094`): while `_matched`
is true it replaces the MPC's braking with `prev + up_step` and a brake floor, then releases in one
frame (251 t=365.89: -2.58 -> -1.82). Next replay counterfactual: disable the matched floor and
transition target on the same five events, and see whether braking starts earlier and gentler.

## 58. The follow policy and BLoTv3 are not active in the bookmarked 24f brakes

Replay only, same five bookmarked 24f events and metrics as item 57. Two harness counterfactuals,
no repo code changed:

- `FP_OFF=1` disables `lead_follow_policy._matched_brake_floor`, `_transition_target` and
  `_steady_follow_deadband`. It does change the replay elsewhere: 12/24/4/0/82 frames in groups
  B-F, max 1.16 m/s2, all at t = 120-138, 226-245 and 509-591. So the switch works. **Inside every
  bookmarked window the output is bit-identical to baseline.** The policy is not engaged in these
  brakes. The 251 t=365.89 release step (item 3/LEAD f) is real but is not what the driver marked
  on 24f.
- BLoTv3 off (no `BLOT=1`): identical to baseline to within one 50 ms frame on all five events.

**Conclusion (replay).** The ECO floor (item 57), the matched-follow policy and BLoTv3 are all
ruled out for the bookmarked 24f brakes. The late-then-rail shape is the MPC's own solution to the
lead input it is given. What remains: the MPC's lead cost / `t_follow` / jerk weights against a
fast-closing lead, and whether the lead state fed to it (dRel, vLead, aLeadK) at the onset
understates the closing rate that the range slope shows.

## 59. The harness pins planner state to the log, so items 55, 57 and 58 measured one-frame effects only

Replay only; no repo code changed.

**Harness caveat (supersedes the conclusions of 55, 57 and 58).** Mode `x0+src+pa+oat`
(`replay.py`) re-anchors the MPC initial state (`x0`), the previous accel (`pa`) and the previous
published output (`oat`) to the LOGGED values every frame. That is right for reproducing the log
(the p99 gate), but in a counterfactual nothing the change does can accumulate. The planner's state
is reset to what the car actually did on every frame. Items 55 (vision lead), 57 (ECO floor) and
58 (follow policy, BLoTv3) therefore measured only the instantaneous one-frame response. Their
"no effect" results are not evidence that those layers don't shape the brake. They have to be
re-run with the anchors removed (mode `src` or `free`). Ego speed and accel still come from the
log, so even that is planner-open-loop, not vehicle closed-loop.

**What is still valid from this item (the instantaneous response under the logged state).**
- Line trace at 24f 272.7 (`w271.4:272.4`): the target before the final clip (`L2568`, MPC at
  action_t) goes -1.0 -> -4.17 in 0.7 s. The published output (`L3115`) is held above it by
  `output_accel_min = min(ECO -0.5, a_desired + 0.05)` (`:2202`, item 45). So in ECO the output
  tracks the one-step `a_desired`, not the MPC's lookahead demand: 272.07 raw -4.04 vs out -2.03.
- With the lead's vRel/vLead/aLeadK taken from 1.0 s later (`LEAD_ADV=1.0`, non-causal), the
  pre-clip target reaches -5.1 at 271.12, 1 s earlier than baseline. Earlier lead information does
  make the MPC ask earlier. Under the anchors the output can't show it.
- Q1, lead lag: over the five events the fed `vRel1` lags the range derivative by about 0.3, 1.35,
  1.45 (rms 3.4, unreliable), 0.65 and 0.30 s. Route-wide 5 s windows give a median of 0.25 s
  (p75 0.70). The fits are noisy. It suggests, but does not show, extra lag at brake onsets.
- In all five events the lead itself braked hard (aLeadK -3.7 to -4.4, range slope -3.5 to -5.2)
  on a single track (no handover). The rail brakes answer real hard lead braking.
- Q2 (`TF_ADD=0.3`): no instantaneous effect. It has to be re-run un-anchored.

New harness envs: `LEAD_ADV=<s>`, `TF_ADD=<s>` (backup `replay.py.bak-pre-leadadv`); scripts
`vq2.sh`, `q1lead.py`, `q1lag.py`; traces `trC.txt`, `trCadv.txt`.

## 60. Un-anchored re-run: the vision lead is much softer, the ECO floor and pre-brake are not a fix, and no car-specific cap fires

*Replay evidence only, route 24f, groups B-F, harness mode `src`.* In this mode the planner runs
on its own state, and only the source label is pinned. Ego speed still comes from the log, so the
gap does not respond to a change in braking. That understates how much braking a delayed
response would need later.

**Control:** the un-anchored base reproduces the logged aTarget (group C p99 0.004 m/s2, PASS).
The metrics in the table match the log to 0.01. Counterfactual runs "FAIL" the gate by design,
because they diverge from the log.

| variant | effect on the 5 bookmarked brakes (peak / steepest 0.5 s rate, base -1.9 to -4.25) |
|---|---|
| ECO floor at -1.0 (`MINACC_FORCE=-1.0`) | removes the -0.5 hold (0.6-1.05 s -> 0-0.1 s); same peak; rate about the same |
| ECO floor off (-3.5) | hard part 0.15-0.6 s earlier; 449.1 -2.09 -> **-3.50**, rate -2.55 -> -6.10. Harsher |
| follow policy off (`FP_OFF`) | no change (max rate diff 0.11) |
| tFollow +0.3 s (`TF_ADD`) | small, mixed (170.2 rate -4.17 -> -3.40, others within 0.2) |
| lead 0.5 s earlier (`LEAD_ADV`) | t<-1.5 0.45-0.65 s earlier; same peak and rate |
| vision lead (`VISLEAD=1`) | **peak -0.9 to -2.9, rate -0.9 to -1.9**. About half the radar response in every event |
| anticipatory pre-brake off (`PB_OFF`, L2529-2540) | peak 0.25-0.6 s later; rate softer at 170.2/595.7, **steeper** at 272.7/367.4 |

**Item 55 is reversed.** Its "vision is as deep" result was an anchoring artefact. Free-running, the
vision lead gives about half the braking. This matches 242 feeling smoother. It does not show
that radar is wrong, though: the lead braked at -3.7 to -4.4 in all five events (item 59), and
vision is known to understate fast closing.

**Cap census:** a line trace of `self.a_desired` and `output_a_target` over pk-4.5 .. pk+0.5 for
all five events. Only these lines change the value:
- L2526, the MPC interpolation;
- L2540, the anticipatory pre-brake, 22-58 frames per event, up to -0.06 per frame, fed back as the
  next x0;
- L2544, the deadzone;
- L3091/3093, the follow policy, almost always lifting (softening);
- L2758, `close_lead_brake_cap`, 2 frames at 449.1 and 595.7;
- L3115, the output clip (the ECO floor, lifting).

**None of the RAV4, Sienna, Accord, CR-V, vision-only or pretracking caps fire.** Trimming them
would not change these brakes.

**Conclusion (replay):** the planner's extra layers are not what makes these brakes harsh. The
harshness follows the radar-measured lead braking. The -0.5 hold is ECO, and a -1.0 floor removes
the hold without deepening the peak. The pre-brake is a two-sided trade-off, not a clean trim.
The harness also cannot show the closed-loop cost of braking later.

## 61. ECO soft floor drops to -1.0 while a lead is closing (branch `claude/eco-lead-floor`); closed loop keeps more gap

*Replay evidence only (open and closed loop), route 24f, groups B-F. Not road-driven.*

**Change** (`longitudinal_planner.py`, acc mode): when `radarState.leadOne` has status and
(vRel < -0.3 or aLeadK < -0.25), `accel_limits[0] = min(minAcceleration, A_CRUISE_MIN)`. The ECO (-0.5)
and traffic (-0.35) profiles are unchanged with no lead, or with a steady lead.

**Open loop** (mode `src`, ego speed from the log): the -0.5 hold goes from 0.60 to 0.05 s at 170.2,
0.20 to 0.00 at 449.1, and 0.20 to 0.00 at 595.7. 367.4 had no hold. 272.7 goes 1.05 -> 0.85 s: the
remaining 268.9-269.7 is steady-follow ECO (vRel ~0, aLeadK -0.1..-0.2), 3 s before the brake, and is
left alone on purpose. Peaks are unchanged.

**Closed loop** (new harness env `CL_T`). From CL_T = pk-6 the ego acceleration follows
`output_a_target` through a first-order lag (tau 0.35 s), and vEgo/aEgo are overridden. The lead
position and speed come from the log, so the lead does not react to us, and radarState dRel/vRel are
recomputed against the simulated ego. modelV2 and aLeadK stay as logged. Metrics cover pk-6 .. pk+8.
The first 3 s of every run match between variants, which is the sanity check.

| brake | variant | min gap m | min TTC s | peak cmd | steepest 0.5 s rate of a | t(a<-1.5) s |
|---|---|---|---|---|---|---|
| 170.2 | base / **(a)** / PB_OFF | 18.0 / **19.2** / 17.8 | 5.16 / **5.73** / 5.05 | -3.45 / -3.44 / -3.30 | -3.14 / **-2.84** / -2.88 | 3.30 / 3.10 / 3.35 |
| 272.7 | base / **(a)** / PB_OFF | 18.9 / **20.6** / 18.4 | 5.99 / **6.31** / 5.78 | -3.48 / -3.48 / -3.45 | -3.28 / -3.26 / -3.56 | 2.40 / 2.36 / 2.39 |
| 367.4 | base / **(a)** / PB_OFF | 28.8 / **29.5** / 27.4 | 4.84 / 4.87 / 4.54 | -3.48 / -3.47 / -3.45 | -1.62 / -1.64 / -1.78 | 6.10 / 5.80 / 6.10 |
| 449.1 | base / **(a)** / PB_OFF | 6.5 / **9.2** / 6.5 | 3.07 / **4.36** / 3.07 | -1.74 / -1.72 / -1.74 | -1.36 / **-1.18** / -1.36 | 0.39 / 0.45 / 0.39 |
| 595.7 | base / **(a)** / PB_OFF | 42.1 / 42.5 / 41.0 | 8.86 / 9.32 / 8.62 | -2.29 / -2.10 / -2.11 | -1.58 / -1.62 / -1.26 | 1.25 / 1.26 / 1.00 |

(a) + PB_OFF sits between the two in every event (vcl.sh `uapb`).

**Reading (replay).** (a) keeps more gap and TTC in all five brakes, with the same peak, and the jerk
is equal or softer (170.2 and 449.1 soften by about 0.2-0.3). It starts braking 0.2-0.6 s earlier at
about -0.6..-1.0 instead of waiting at -0.5. PB_OFF gives less gap in every event and a softer rate
in only 2 of 5. It is **not** a trim candidate: the pre-brake is buying distance. Harness limits:
- the lag model is not the car's actuator;
- the lead is replayed, not reactive;
- the model and aLeadK inputs are not re-simulated.

**Tests:** `selfdrive/controls/tests` gives "3 failed, 1263 passed" both with and without the change,
the same three pre-existing failures:
- `test_latcontrol`: bolt low-speed center output;
- `test_latcontrol`: palisade center taper;
- `test_starpilot_planner::test_force_stop_jerk_scale_is_platform_specific`.

No unit test is added yet. Nothing is pushed to `ns-bosch-radar-testing`.

## 62. FrogPilot "human following" = model lead path with no guard; softer brakes, less gap (closed-loop replay)

*Replay evidence only (closed loop as in item 61), route 24f, groups B-F. FrogPilot-Testing 728f65472
(2026-09-14), fetched as `refs/remotes/frog/testing`.*

**What FrogPilot has now.**
- `HumanFollowing` (`long_mpc.py process_lead`): when the model lead prob exceeds the threshold and the
  radar lead is valid, the MPC lead path is radar dRel/vLead plus the model's future x/v deltas, instead
  of the constant-aLeadK exponential extrapolation. There is **no** braking or TTC bail-out.
- `HumanAcceleration` only shapes max accel (low-speed and ramp-off) and launch. It does not touch
  braking. StarPilot already removed it (`test_human_acceleration_param_is_removed`).
- The FrogPilot ECO floor goes to ACCEL_MIN whenever `tracking_lead` (a stronger form of item 61).

**StarPilot** already has the same path (`build_model_lead_trajectory`, long_mpc.py:159). It
falls back to the raw extrapolation when aLeadK < -0.5 or (closing > 0.75 m/s and TTC < 7 s): see
`MODEL_LEAD_TRAJECTORY_MAX_LEAD_BRAKE/MAX_CLOSING_TTC`, commits 24f482966 and 7f2bab7be. There is no
evidence entry, only the comment "optimistic model horizon cannot delay the first braking response".
Every bookmarked brake trips that guard, so the harsh path is the raw aLeadK extrapolation. The item-60
"vision lead is half as hard" result is the same effect.

Harness env `MLT_FROG=1` sets both guard constants to never trip.

| brake | min gap m: base / (a) / MLT / (a)+MLT | min TTC s: same order | peak cmd: same order | steepest 0.5 s rate: same order |
|---|---|---|---|---|
| 170.2 | 18.0 / 19.2 / 13.4 / 13.7 | 5.16 / 5.73 / 4.19 / 4.36 | -3.45 / -3.44 / -2.05 / -1.93 | -3.14 / -2.84 / -1.26 / -1.04 |
| 272.7 | 18.9 / 20.6 / 16.5 / 17.2 | 5.99 / 6.31 / 5.65 / 6.03 | -3.48 / -3.48 / -2.36 / -2.26 | -3.28 / -3.26 / -1.26 / -1.18 |
| 367.4 | 28.8 / 29.5 / 23.6 / 27.7 | 4.84 / 4.87 / 3.70 / 4.31 | -3.48 / -3.47 / -2.34 / -2.28 | -1.62 / -1.64 / -1.72 / -1.40 |
| 449.1 | 6.5 / 9.2 / 6.3 / 8.7 | 3.07 / 4.36 / 3.21 / 4.25 | -1.74 / -1.72 / -1.10 / -1.15 | -1.36 / -1.18 / -0.50 / -0.84 |
| 595.7 | 42.1 / 42.5 / 34.9 / 37.8 | 8.86 / 9.32 / 8.24 / 8.93 | -2.29 / -2.10 / -0.92 / -1.00 | -1.58 / -1.62 / -0.96 / -1.68 |

Full FrogPilot braking (MLT + ECO floor to ACCEL_MIN, tag `frog`) keeps the most gap but restores
the peak (-2.9 to -3.5) and hits a rate of -4.0 at 449.1.

**Reading (replay).** The guard is what makes these brakes feel "reactive". With it off, the peak
falls by about a third and the jerk by about half. The cost is 1-7 m of gap and up to 1.1 s of TTC
(367.4: 4.84 -> 3.70), and (a) wins back part of that. Limits:
- the model lead deltas are replayed from the log, so they do not react to the simulated ego;
- the lead is not reactive.

This is a safety-margin trade and it is not decided here. No code change is made.

## 63. Model lead path keeps only a 3 s closing-TTC guard (closer to FrogPilot HumanFollowing); replay only

Following 62, and at the user's request (goal: FrogPilot-like following, fewer guards that cause abrupt
braking), `build_model_lead_trajectory` (`long_mpc.py`) no longer falls back to the raw aLeadK
extrapolation because the lead is braking (`MODEL_LEAD_TRAJECTORY_MAX_LEAD_BRAKE` 0.5 is removed). The
closing-TTC fallback is reduced from 7 s to 3 s (`MODEL_LEAD_TRAJECTORY_MAX_CLOSING_TTC`); it still needs
closing > 0.75 m/s.

Closed-loop sweep on the five bookmarked 24f brakes, with the 61 ECO floor in place (CL_TAU 0.35):

| guard (lead-brake / TTC)  | peak cmd (m/s^2)                      | min TTC (s)                  |
|---------------------------|----------------------------------------|------------------------------|
| 0.5 / 7 (old)             | -3.44 / -3.48 / -3.47 / -1.72 / -2.10  | 5.73 / 6.31 / 4.87 / 4.36 / 9.32 |
| 1.5 or 2.5 / 7            | about the same as old                  | -                            |
| 4 / 3                     | partial improvement                    | -                            |
| none / 3 (this change)    | -1.93 / -2.26 / -2.28 / -1.15 / -1.00  | 4.36 / 6.03 / 4.31 / 4.25 / 8.93 |

Limits:
- **The 3 s TTC guard never trips in these five events, so its effect is not tested by replay.**
- The model lead deltas and the lead are replayed from the log and do not react to the simulated ego.
- Min TTC is lower than with the old guards on three of the five brakes. This trades margin for comfort.
- Replay evidence only. Nothing is road-validated. On the next drive, watch hard lead brakes, cut-ins
  and late stopped-car approaches. The concern is braking that starts later than before.

Tests: `test_model_lead_trajectory_used_for_braking_lead_with_long_ttc` is added. The urgent-fallback
case now uses a closing lead at a 2 s TTC. The `selfdrive/controls/tests` run gives 1264 passed and 3
failed, and all 3 failures were already failing before this change (latcontrol bolt, latcontrol
palisade, `test_force_stop_jerk_scale_is_platform_specific`).

## 64. Guard trim batch 1: three inert highway caps deleted; batch 2 not exercised; 254 vs 257 drive A/B; replay + limited road evidence

Goal: trim longitudinal guards that do not measurably buy gap or TTC, so the MPC and model decide more
(closer to FrogPilot). Scope is the longitudinal planner only. Method: each guard was given a temporary
env kill switch, and the five bookmarked brake events of route `0000024f--8c147bae2e` (B 140 s, C 262 s,
D 357 s, E 440 s, F 585 s) were replayed closed-loop (CL_TAU 0.35, lead non-reactive) with each guard
off, one at a time, against the same tree. A guard is kept only if removing it loses gap or TTC.

### Batch 1 (highway braking caps) — closed-loop replay, minGap m / minTTC s

| Event | base | no close_lead_brake_cap | no inside_gap cap | no raw_close_lead_needs_control | no matched floor | no transition limiter |
|---|---|---|---|---|---|---|
| B | 13.70 / 4.36 | 13.71 / 4.35 | 13.75 / 4.37 | = | = | = |
| C | 17.20 / 6.03 | 17.09 / 6.03 | 17.19 / 6.03 | = | = | 17.19 / 6.03 |
| D | 27.68 / 4.31 | **24.37 / 3.98** | = | 27.83 / 4.32 | = | = |
| E | 8.69 / 4.25 | **7.47 / 3.72** | = | **5.08 / 1.28** | = | = |
| F | 37.81 / 8.93 | 35.20 / 8.21 | 37.72 / 8.89 | = | 38.82 / 8.93 | 37.35 / 8.92 |

KEPT: `get_close_lead_brake_cap` (D loses 3.3 m, E loses 1.2 m, and the brake starts later and ends
harder: D peak −2.28 → −2.39, 0.5 s rate −1.40 → −1.70) and `raw_close_lead_needs_control` (E TTC
4.25 → 1.28). DELETED, all inert (peak command moved ≤ 0.03 m/s² on every event):
`get_inside_gap_closing_lead_accel_cap` and its `INSIDE_GAP_CLOSING_*` constants,
`lead_follow_policy._matched_brake_floor`, `lead_follow_policy._transition_target` and the
`FOLLOW_TRANSITION_*` / `FOLLOW_SIGN_CROSS_STEP` / `FOLLOW_MAX_CLOSING` constants, and
`MATCHED_FOLLOW_TRANSITION_MIN_SPEED`. `FollowResult` is now `(lead, accel_cap, target)`; nothing
outside the policy consumed `brake_floor`. `get_lead_geometry_required_accel` is telemetry only and
was left alone. Batch 1 does not explain the abrupt-braking complaint; the replay harness at
`$T/vcl.sh` / `vclcmp.py` now mounts the branch planner, policy and `long_mpc.py` together.

### Batch 2 (vision-lead caps) — not exercised by the corpus

`get_vision_lead_approach_cap`, `get_vision_untracked_slow_lead_cap`,
`get_vision_untracked_approach_lift_cap`, `get_vision_slow_stopped_lead_cap`,
`get_tracked_vision_model_brake_floor` / `_cap` and `tracked_vision_lead_approach_needs_immediate_brake`
were swept the same way. Every tag produced byte-identical numbers on all five events in every column.
All five events are radar-tracked leads and every batch-2 guard is a vision-lead path, so replay cannot
distinguish harmless from untested here. Decision: keep batch 2 and defer until a vision-only brake
event is replayed (route `00000257--50424c1a3a` at 348.7 s, `radar=0`, peak −2.55, is a candidate).
Batch 3 (standstill and depart holds) is not started and rests on road testing; replay covers stop-go poorly.

### Drive A/B on the STATUS 63 change: `00000254--8afa97025c` (5d7e1512f) vs `00000257--50424c1a3a` (8c9f5dc66)

Both builds are confirmed from initData; the only planner difference is STATUS 63 (model lead path keeps
only the 3 s TTC guard). Per-frame scan (`/routes/an2/scan_<r>.csv`, summary `$T/smooth.py`),
longActive frames only:

| | 254 before | 257 after |
|---|---|---|
| longActive frames | 4843 | 7795 |
| aTarget p1 / p5 | −3.45 / −1.97 | −2.13 / −1.00 |
| measured aEgo p1 | −3.87 | −2.42 |
| worst 0.5 s drop of aTarget | −3.86 | −1.72 |
| 0.5 s drops steeper than −1.5 m/s² | 37 | 8 |
| braking episodes (aTarget < −1.0), peak | 5, all at −3.45 (the clamp) | 6, −1.28 … −2.82 |
| driver brake while enabled / disengagements | 0 / 9 | 1 / 9 |

Every "before" episode pinned at the −3.45 floor and four of the five had command drops of −1.9 to
−3.9 m/s² inside half a second. After the change the hardest event (415.8 s, lead 75 m closing at
16.7 m/s) peaked at −2.82 with a −0.80 drop. This is one drive each on different roads: limited road
evidence, not a controlled comparison. The one driver brake press in 257 (127 s, lead 72 m closing
7 m/s, planner asked −1.57) is a candidate "too soft" case to look at.

Tests: `selfdrive/controls/tests` gives 1256 passed, 4 skipped, 3 failed; the 3 failures are the same
pre-existing ones (latcontrol bolt, latcontrol palisade, `test_force_stop_jerk_scale_is_platform_specific`).
Five tests for the deleted guards were removed. The 9 `ruff` findings on the planner and its test file
all lie outside the diff hunks and are pre-existing. Not road-validated.

## 65. Guard trim batch 2 (vision-lead caps) replayed on three vision-only brake events; only the untracked slow-lead cap buys gap; replay only

Batch 2 was deferred in STATUS 64 because route 24f has radar leads everywhere. Three vision-only
events were replayed closed-loop (CL_TAU 0.35, lead non-reactive, branch planner at 820c58ca3
mounted over the built tree), with each guard turned off one at a time through a replay-harness
monkeypatch (the branch has no kill switches):

| Event | Route | Build | What happens |
|---|---|---|---|
| V 348.7 s | `00000257--50424c1a3a` | 8c9f5dc66 | vision lead 44 m closing 7.2 m/s at 12.5 m/s, radar 0; logged peak aTarget −2.55 |
| W 787.7 s | `0000020c--4712c9cce5` | d8604bd20 | untracked vision lead 70–90 m closing 6–9 m/s at 13.5 m/s, `src=cruise` throughout |
| X 221.0 s | `0000020c--4712c9cce5` | d8604bd20 | vision lead 78 → 45 m at 21.6 m/s, tracked as `lead1`/`lead0` |

Route 20c has no radar lead in any of its 5792 longActive lead frames. Closed loop, minGap m / minTTC s / peak cmd:

| Guard off | V | W | X |
|---|---|---|---|
| base | 12.24 / 4.13 / −2.41 | 41.28 / 6.56 / −0.85 | 22.07 / 3.71 / −3.50 |
| `get_vision_lead_approach_cap` | 12.06 / 4.09 / −2.42 | = | = |
| `get_vision_untracked_slow_lead_cap` | = | **17.22 / 5.55 / −1.06** | 21.99 / 3.68 / −3.50 |
| `get_vision_untracked_approach_lift_cap` | = | = | = |
| `get_vision_slow_stopped_lead_cap` | = | = | = |
| `get_tracked_vision_model_brake_floor` + `_cap` | = | = | = |
| `tracked_vision_lead_approach_needs_immediate_brake` | 12.06 / 4.09 / −2.42 | = | = |
| all six off | 12.06 / 4.09 / −2.42 | 17.21 / 5.55 / −0.71 | 21.99 / 3.68 / −3.50 |

"=" means the closed-loop trace is byte-identical to base.

KEEP `get_vision_untracked_slow_lead_cap`: on W it is the only thing that brakes. With it off the
planner holds 11.5–12.3 m/s while the untracked lead closes from 70 m to 34 m, minimum gap 41 → 17 m
and TTC 6.6 → 5.6 s. MPC never took that lead (`src=cruise` for the whole event), so nothing else
would have.

Near-inert: `get_vision_lead_approach_cap` and `tracked_vision_lead_approach_needs_immediate_brake`
(same mechanism) advance the brake onset by one frame on V (cmd −1.30 instead of −1.00 for 0.1 s at
348.7 s); 0.18 m of gap, 0.04 s of TTC, then identical. Not exercised on W or X.

Not exercised on any of the three events: `get_vision_untracked_approach_lift_cap`,
`get_vision_slow_stopped_lead_cap`, `get_tracked_vision_model_brake_floor` / `_cap`. None of the
events had a stopped vision lead, so the slow-stopped cap in particular has still not been tested.

No code changed in this section. Deletion of the near-inert or unexercised guards waits for the
user's decision. Harness: `~/.claude/jobs/3c03581d/tmp/{vcl257.sh,vcl20c.sh,replay.py}` with
`GT_VLA/VUS/VUL/VSS/TVM/TVI=0` env hooks; `EVG="V:347.5:353.5" python3 vclcmp.py ...`.

## 66. Guard trim batch 2: vision lead approach cap and immediate-brake confirm bypass deleted (near-inert per STATUS 65); replay only

Per the user's decision on STATUS 65, the near-inert pair is gone from
`selfdrive/controls/lib/longitudinal_planner.py`: `get_vision_lead_approach_cap`,
`tracked_vision_lead_approach_needs_immediate_brake`, their call site in `update()`, the
`vision_lead_approach_confirm_t` state and the `VISION_LEAD_APPROACH_*` constants they alone used
(−112 lines). `VISION_LEAD_APPROACH_MIN_MODEL_PROB` / `_FULL_MODEL_PROB` stay because the untracked
approach-lift cap and `get_dynamic_t_follow` still read them.

Seven tests removed from `test_longitudinal_planner.py` (−133 lines): the four unit tests of the
cap, the two acc-mode/persistence tests that drove it, and
`test_acc_mode_tracked_vision_close_or_braking_lead_bypasses_persistence`, which asserted the
one-frame-earlier onset (first-frame command below −1.3) that STATUS 65 measured as the pair's
whole effect. Controls suite in docker: 1237 passed, 4 skipped, 3 failed, all three pre-existing
and unrelated (two latcontrol, `test_force_stop_jerk_scale_is_platform_specific`).

Closed-loop replay of event V (`00000257--50424c1a3a` 348.7 s, CL_TAU 0.35) with the deleted-pair
planner is byte-identical to the STATUS 65 "cap off" row: minGap 12.06 m, minTTC 4.09 s, peak
command −2.42 m/s², versus 12.24 / 4.13 / −2.41 with the pair. Events W and X were already
unaffected by these two guards.

Kept from batch 2: `get_vision_untracked_slow_lead_cap` (buys 24 m on 20c W),
`get_vision_untracked_approach_lift_cap`, `get_vision_slow_stopped_lead_cap`,
`get_tracked_vision_model_brake_floor` / `_cap` (unexercised on any replayed event, no stopped
vision-lead event exists yet). Replay evidence only; no road data on this build. Pushed to
`ns-bosch-radar-testing` for road testing; new routes to be analysed against this build.

## 67. First road route on `41abc1f36`: route 258. The hard brakes are real leads; the one worst brake followed a lead dropout; the car over-brakes its own command. Limited road evidence, no replay.

Route `11c8fa231c0499ed/00000258--626242f48b`, segments 39-67 (0-38 idle). `initData`: `gitCommit
41abc1f36`, `ns-bosch-radar-testing`, not dirty -- the STATUS 66 build. ECO decel
(`DecelerationProfile 1`), conditional experimental (experimental on 22.6% of engaged frames),
aggressive personality (tFollow 1.25-1.45). 22.9 engaged minutes, 5 user bookmarks. Analysis is
CSV-only (`scan_00000258--626242f48b.csv` in the `oprad-routes` volume, 20 Hz); the rlogs were
already cleaned from the fetch dir, so there is no `liveTracks` and no replay here.

**1. The planner is passed through unchanged; the extra braking past about −3 happens in the car.** `outAccel == aTarget` within 0.05
on 99% of engaged frames. Car response 0.35 s later, by commanded bin, median `aEgo - outAccel`:
−0.08 (−1..−0.5), −0.11, −0.16, −0.13, −0.04, **−0.34 (−3.6..−3.0)**, p10 down to −1.12. The
wheel-speed derivative (`dv/dt` over 0.2 s) agrees (−0.30 in the last bin), so this is not `aEgo`
noise. Worst cases: `aEgo` −4.76 against a −3.50 command (66:36), and −2.79 against −1.60 at 7 m/s
in stop-and-go (63:53.8). Every one of the 9 episodes reaching `aEgo` ≤ −3 had the command
saturated at or near `ACCEL_MIN` −3.5 and the ECO floor `minAcc` −0.50 overridden (the
`a2ed92d29` lead-closing floor). Road grade is not logged in the CSV and is not excluded.

**2. The hard brakes are real lead decelerations, not phantoms.** 42:59 (lead 24.4→6.4 m/s,
`aLeadK` −7.6, model agrees 17→2 m/s), 44:41 (radar 14.9→6.1, model 15→4.3), 51:33 (radar and range
slope agree at −7 to −8 m/s; the model lags by ~3 m/s), 43:49, 53:31, 55:27. Onset is driven by
`leadGeometryRequiredAccel` (2.7-5.9) with `closeLeadBrakeCap` equal to `aTarget`. On 51:33 and
55:27 the radar `aLeadK` spikes (−6, and +8.2 / +13.8 elsewhere) are implausible as accelerations but
the ranges confirm the closing.

**3. 66:36, the hardest brake on the route, followed a 2.5 s lead dropout.** Radar track `tid 20`
was the lead at 63 m, 11 m/s, `y` −3.2→−5.4 (curve), with `modelProb` 0.90→0.65. From 66:32.8 to
66:35.3 **no lead at all** was published (`nat_d1` NaN too) while the model's own lead probability
fell to 0.01-0.05 and its `mlX` walked 69→35 m. The lead came back at 66:35.3 as vision (27 m),
then **the same `tid 20`** at 29.4 m with `vLead` −1.4 (range slope confirms ~1 m/s), TTC ~2.4 s:
`aTarget` −3.50 within 0.5 s, `aEgo` −4.76. The same track id on both sides suggests the radar
kept the object and the *lead selection* dropped it when model probability collapsed -- the D-041/D-042
failure shape (a withheld point, not a degraded one). **Not proven:** without `liveTracks` it
cannot be shown that `tid 20` was continuously present during the gap. Re-fetch segment 66 to
settle it.

**4. Bookmarks.**
- 55:43 -- radar `tid 46` `vRel` frozen at −5.7 (and `aLeadK` 2.07) for ~4 s while its own range
  closed at only ~1.2 m/s; the planner braked to −1.0 twice and the driver overrode with gas twice.
  A stuck or over-closing radar velocity, the same class as the census "U11 over-closes future
  slope" (3 onsets on this route, 8/h).
- 63:10 -- a steady stop behind a decelerating lead, command −1.35, `aEgo` −1.5..−1.8. Looks
  proportionate; the only oddity is the car over-braking its command by 0.2-0.4.
- 63:57 / 64:07 -- stop-and-go. Command −1.0→−1.6 inside the desired gap (13.6 m at 7.8 m/s);
  `aEgo` −2.79. The jolt is the car over-braking its command by 1.2 at low speed. Driver gas at 64:04.
- 66:24 -- cut-in: new radar track `tid 18` at `y` +3.8, 23 m, `aLeadK` −6.4 → command −2.22 at
  engagement. Plausibly correct.

**What this does NOT establish.** One route, 22.9 min, CSV only, no replay, no `liveTracks`. The
over-brake could be Honda Bosch brake tracking or unlogged grade; it is outside the planner either
way, but its cause is not identified. Item 3's mechanism is inferred from the lead fields.

## 68. HumanAcceleration ported from FrogPilot (default off); HumanFollowing becomes a toggle (default on). Static and unit evidence only.

**HumanAcceleration** (`HumanAcceleration`, default **off**; FrogPilot defaults it on). Ported from
FrogPilot-Testing `728f65472` (`frogpilot_acceleration.py`, same math as the Nov 2025 snapshot
`202543c86`). Only `max_accel` changes, in `StarPilotAcceleration.update`: after the profile is chosen
and before the weather reduction. Braking and `min_accel` are not touched.
- The set speed scales max accel: 1/4 of it at a 0 set speed, 1/2 at 12.5 m/s, full from 25 m/s.
- Ramp-off near the set speed: 0 at the set speed, 0.5 m/s² at 1 m/s below it, full at 5 m/s below.
- FrogPilot's other half (in the starting state, longcontrol outputs `a_target` instead of
  `startAccel`) is **not ported**. StarPilot's starting state already clips `a_target` to
  `[0, startAccel]`.

**HumanFollowing** (`HumanFollowing`, default **on**). The model lead path with the 3 s closing-TTC
guard from STATUS 62/63 used to be unconditional. It is now gated in `human_following_model()`
(`longitudinal_planner.py`): with the toggle off, the MPC gets no `modelV2` and every lead
falls back to the `aLeadK` extrapolation. On by default, so driving is unchanged unless the
driver switches it off. The Nov 2025 FrogPilot follow-gap HumanFollowing is **not** ported.

**Where the settings live.** Both toggles follow FrogPilot's gating: they sit under
Longitudinal Tuning (`parent_key LongitudinalTune`, on-device panel rows between Deceleration
Profile and Human-Like Lane Changes) and read as off when `LongitudinalTune` is off
(`get_value(..., condition=longitudinal_tuning)`). Both keys came out of
`STARPILOT_REMOVED_PARAM_KEYS`; they were in that list, which deleted them on every boot.
Both are also in the safe-mode key list. Galaxy's `HIDDEN_SETTING_KEYS` is now empty.

**Artifacts.** `common/libcommon.a` and `common/params_pyx.so` were rebuilt using the larch64 Docker
recipe (Cython 3.1.4, `SP_FORCE_TICI=1`, sconsign cleared first). The key count went from 822 to
824, and `Params(memory=True)` returns defaults HumanAcceleration=False, HumanFollowing=True.

**Tests.** New unit tests (`test_starpilot_acceleration.py`, `test_longitudinal_planner.py`,
galaxy layout) pass. 5 tests fail: 4 fail identically on a clean HEAD export (`test_latcontrol`
×2, `test_force_stop_jerk_scale_is_platform_specific`, one `test_starpilot_card` case). The 5th,
`test_every_galaxy_toggle_key_exists_in_the_committed_device_params_binary`, reads the committed
`.so` and needs the commit. Not replayed, not driven.

## 69. Route 258 re-examined with rlogs: the 43:49 FCW was a real lead hidden for 12.7 s by a self-latching Bosch-A vRel rate check; the trim batches are not implicated. Replay (parser) plus limited road evidence.

Route `11c8fa231c0499ed/00000258--626242f48b` is still the newest on Konik (68 segments). Segments
42-44 and 66 were re-fetched; `initData` on 43: `gitCommit 41abc1f36`, `RangeDerivedVrel 1`,
`HondaBoschARadar 1`.

**1. The FCW (first frame route t 2628.88 = 43:48.9; Peter noted it as 43:41) is the planner FCW, not stock.**
`longitudinalPlan.fcw` (MPC crash count > 2), `carState.stockFcw` never set, driver never braked.
The lead is real: radar `tid 13`, model prob 1.00, braking 24 → 3 m/s; min gap ~7 m at 3 m/s,
`aEgo` −3.9. STATUS 67 item 2 already listed 43:49 as a real lead; what it missed is below.

**2. For 12.7 s before the FCW the published closing speed was wrong by up to 8 m/s.** From 43:29.2
to 43:46.0 `tid 13` was published `measured=False` with `vRel` held at −0.45 (the last trusted
value) while its range fell 124 → 31 m. The radar was right: raw U11 went −3.3 → −8.7 m/s and
matches the range slope. With `vLead` ≈ `vEgo` the planner stayed in `cruise` until d = 42 m and
reached −1 m/s² only at d = 31 m, when U11 was re-admitted (−2.9 s). `aLeadK` sat frozen at +13.75
and `vLeadK` at 17.1 throughout, because the lead KF is not stepped on coasts.

**3. Mechanism: replay of the real `RadarInterface` with an instrumented copy (not committed).**
Every sweep in that window hit `vrel_inconsistent` (the one-sided multi-sweep rate check, D-054):
U11 < fitted range rate − 3.0. The fit uses `track.samples`, and **`samples` is only appended on
accepted or rejoin-held sweeps, never on a `vrel_inconsistent` coast** — even though the range
itself had passed the innovation gate (the comment above the check says the accepted range "is the
gate's baseline from here on, whatever happens to vRel"). The 8 samples froze at 43:12.2-43:18.7,
while the lead was pulling away (+2.8 m/s), so the fit kept a stale opening rate and rejected
every correct closing U11 that followed. The gate latches itself until the single fresh point drags
the frozen fit far enough — here 12.7 s and 90 m of closing. This is a lockout of the kind
`lockcensus.py` looks for, triggered by a real opening → closing reversal of the lead.

**4. Neither safety net saw it.** RadarD's D-053 range assist clears on a coast and appends to
`range_hist` only on measured updates, so `vRelRangeDerived` was NaN for the whole window. The
model lead read 19-21 m/s (closer to truth), but the HumanFollowing path anchors on the radar
`vLead` and takes only the model's future deltas.

**5. Segment 66 (STATUS 67 item 3) is settled: a withheld point, not a missing one.** `tid 20` was
in `liveTracks`, `measured=True`, on every sweep of the 66:32.75-66:35.25 gap (range 60 → 35 m,
`y` −6.7 → −8.6 on a curve, `vRel` −2.5 → −13.5, range slope ~12 m/s confirms). Lead selection
dropped it when the model lead probability collapsed to 0.02-0.24. D-041/D-042 failure shape.

**6. Trim batches (STATUS 64, 66): neutral, not measurable.** Neither route-258 event involves a
deleted guard: the vision-lead approach cap returned early for any `lead.radar` lead (43:49 was
radar throughout) and there was no lead at all during the 66:33 gap. Census (`hb.py`, aTarget < −1.5
onsets per engaged hour): 254 (pre-trim) 74/h over 4 min, 257 (pre-trim) 18/h over 6 min, 258
(post-trim) 21/h over 23 min. The baselines are 4 and 6 engaged minutes; no rate difference is
resolvable. No evidence the trim hurt; none that it helped.

**Next, in priority order (none implemented):**
1. Rate-check lockout: append the innovation-gated range to `track.samples` on a
   `vrel_inconsistent` coast too, so the fit tracks the object. Must be A/B'd with the `ab3.py`
   harness (lost / gained / newly-measured vRel over-close vs the next-1 s range slope against ref)
   on 258 and the cached routes before promotion — D-056 shows a gate relaxation can re-admit
   over-closing U11. Add a `lockcensus.py` count of inconsistent runs > 2 s per route.
2. Let D-053 range assist use coasted-but-gated ranges, so a long coast gets a range-derived vRel.
3. Lead selection must not drop a measured, continuously tracked lead on a model-probability
   collapse alone (segment 66).

## 70. D-062 implemented: a lasting clean run of rate-check coasts re-roots the Bosch-A vRel fit. Static and replay evidence; not driven.

Fixes the STATUS 69 lockout (route 258 43:29-43:46). Mechanism and rule in D-062;
`radar_interface.py` `inconsistent_run`, tests `TestRateCheckCoastReRoots` in
`test_bosch_a_radar.py`.

**Replay harness.** `ab9.py` (not committed, in the `oprad-routes` volume) runs the current parser
(M) and D-062 (R) side by side over each route's rlogs. The routes are the 24 cached rlog routes
(`0000020c`, `231`-`258`).

**1. Coverage.** R loses 0 radar points and 0 lead points against M. It turns 711 coasted sweeps into
measured ones (113 on the lead) and 16 measured sweeps into coasts (3 on the lead).

**2. The one-sided score overstates the cost.** Counting only R's newly measured vRel, 15.4%
over-close the next-1 s range slope by more than 3 m/s, against 4.7% for M's own measured vRel. The
score never looks at the coasted value M published on the same sweep, and that value is often far
worse. On 251 `tid 60`, M held −13.50 for 3 s while the range closed at about −4 to −6. R re-rooted
0.9 s earlier at −8.0 and converged to −6.3; it scores as over-closing but is about 5 m/s nearer.

**3. Paired score.** For every sweep that R measures and M coasts, compare each value with the
next-1 s range slope (at least 10 range points over at least 0.7 s). A side wins when it is nearer
by more than 0.5 m/s.

| | sweeps | R nearer | M coast nearer | summed abs error R / M (m/s) |
|---|---|---|---|---|
| all | 421 | 235 | 79 | 815 / 1,627 |
| lead | 113 | 73 | 35 | — |

By route: 258 is 94 to 4 (lead 71 to 0; M error 709, R 269). 251 is 51 to 17, 24b 20 to 7 and 232
30 to 1. **Against R:** 241 is 0 to 20 (lead 0 to 16) and 24f is 7 to 19 (lead 2 to 19); 237 is
0 to 9, a near tie (median error 1.55 vs 1.48). R over-closes (by more than 3 m/s) where M does not
on 43 sweeps, 31 of them on 258.

**4. The lead losses, by hand.**
- 241 `tid 33`: M coasts −6.67 until 414.7 s; R re-roots at 413.1 with −9.27 and eases to −7.59;
  they agree from 414.73. R is nearer at first and farther later. It errs by 1-2.6 m/s toward
  closing, and neither value is unsafe.
- 24f `tid 18`: both parsers are stale for about 2.5 s, and R is about 1.4 m/s more closing.
- The one hand-checked real R over-close is 239 `tid 13`, not a lead.

**5. Not tightened.** At 3 m/s, the trailing range slope does not separate the good re-roots from
the bad. Three lead routes are too few to re-tune the D-057 thresholds (rule 5). On the lead, the
expected road symptom is earlier or harder braking behind a car that is closing slowly, not a late
brake.

**Tests (static).** All 260 Honda radar tests pass in docker. Controls suite in docker: 1246 passed, 4 skipped, 3 failed. These are the same three failures as STATUS 66 (two in latcontrol, `test_force_stop_jerk_scale_is_platform_specific`); none is new.

**Next.**
1. Drive it. Watch for brakes that are early or harder than the gap needs behind slow-closing leads.
2. Done: the qlog sweep follow-up is below. Still open: replay route 252 (`00000252--69505eb434`),
   segments 15 and 16, once its rlogs reach Konik.
3. STATUS 69 next items 2 (D-053 range assist on gated coasts) and 3 (a lead dropped on a
   model-probability collapse) are still open.

**Follow-up (2026-09-23): qlog sweep and 24d.**

*Correction.* The table in point 3 covers 23 routes, not 24. `ab9.py` crashed on `0000024d` on a
point that only R publishes (M has no value to pair against). With that guard added, 24d gives:
R loses 0 points and 0 lead points, 69 coasted sweeps become measured (none on the lead), and R
publishes 31 point-sweeps that M did not (none on the lead). The paired score goes 2 to 21 against R
(median error 1.69 vs 0.79 m/s, none on the lead, neither side over-closes). With 24d the all-sweeps
row becomes 461 sweeps, 237 R nearer, 100 M nearer, summed error 878 / 1,659. The lead row is
unchanged.

*qlog sweep.* 254 radar-era routes, 198 with engagement, 43.7 engaged hours, 0 segment errors.
Flags: FCW 64, hard brake 765, sign reversal 446, and `frozen_lead` 13 on 7 routes (lead vRel and
aLeadK bit-identical for at least 2 s while its range moves at least 1 m). qlogs only found the
candidates. Every check below is an rlog replay.

| route, t (s) | frozen | M rate-check run in rlog? | lead / engaged | D-062 re-roots it? |
|---|---|---|---|---|
| 252 262.7, 992.3 | 12.2 s, 9.5 s (992: closing 68→36 m) | **rlogs not on Konik yet** | — | not replayed |
| 24d 660.2 | 11.8 s | yes, tid 23, 12.5 s, agrees with U11 | 0.94 / 0.94 | no |
| 24d 828.5 | 4.7 s | yes, tid 47, 7.6 s, 104→56 m, U11 over-closes | 1.0 / 0.26 | no |
| 24d 1506.3 | 4.0 s | yes, tid 47, 5.2 s, agrees | 1.0 / 1.0 | no |
| 23f 1240.6 | 6.0 s | **no run of 2 s or more** | — | — |
| 23f 1640.4 | — | yes, tid 37, 3.9 s, agrees | 0.61 / 1.0 | no |
| 24f 367.4 | 5.5 s, closing 53→37 m | yes, tid 18, 5.8 s, U11 over-closes | 1.0 / 0.45 | **yes**, 1.2 s from 372.8 (point 4) |
| 24f 799.2 | — | yes, tid 23/33, about 2.6 s | 0 and 1.0 / 0 | no |
| 23e 875.7 | 2.5 s | yes, tid 28, 9.4 s, agrees | 0.31 / 1.0 | no |
| 251 490.9 | — | yes, tid 39, 3.8 s, opening | 0.76 / 1.0 | no |
| 251 499.7 | — | yes, tid 30, 2.3 s, U11 over-closes | 0.97 / 0.57 | no |
| 257 169.2 | — | yes, tid 61, 2.2 s, opening | 1.0 / 1.0 | no |

Replay findings:
- The qlog `frozen_lead` signature finds the M rate-check coast in 10 of 11 events that could be
  replayed.
- 23f 1240.6 has no rate-check run behind it. That freeze is unexplained, and the qlog decimation
  may be the cause.
- D-062 re-roots only one flagged lead, 24f `tid 18`. That is the loss already listed in point 4.
- On the others, R has no lead run of 1 s or more that differs from M.
- Those runs are also `degraded` for their whole length (24f `tid 18` only half of it). This is an
  observation, not a tested cause.
- The 258-style lockout (a clean, lasting run of rate-check coasts on the lead while closing) does
  not show up in any other replayable route.
- 252 at 992 s is the one open candidate, and it is closing. Its rlogs upload only on WiFi. When
  they do, fetch segments 15 and 16 and replay them with `ab9.py`.

## 71. The car over-brakes its own command, and the cause is not in our code: the Honda Bosch brake ECU overshoots fast brake onsets. It happens on all 7 routes checked. Replay (log decode) evidence only.

Follows STATUS 67 item 1. Scripts `ob1.py` (per-route bins and episodes) and `ob2.py` (timelines) are in
the `oprad-routes` volume at `/routes/an2` and are not committed. They were run on the rlogs of 258,
257, 254, 251, 24f, 241 and 237.

**1. We send exactly what the planner asks for.** On every route, `ACC_CONTROL.ACCEL_COMMAND` (0x1DF,
decoded from `sendcan`) equals `carControl.actuators.accel`: median difference 0.000, p1/p99 within
±0.03 m/s². On Bosch, `carcontroller.py` only clips to [−3.5, 2.0], and the hill term reaches only
the gas path. Honda Bosch has no longitudinal PID (`kiV` is set only for non-Bosch), so the car's
own ECU closes the loop on `ACCEL_COMMAND`.

**2. Grade is not the cause.** Median pitch is 0.1-1.4° per bin. Correcting `aEgo` by g·sin(pitch)
moves the bin medians by at most about 0.25 and does not remove the saturated-bin offset.

**3. Steady tracking is good; only the saturated bin over-brakes.** Median `aEgo(t+0.35) − cmd` is
within ±0.35 on routes with enough samples for every bin from −0.5 to −3.0 m/s². The −3.0 to −3.6 bin is
−0.25 to −0.65 on every route that reaches it (258 −0.34, 237 −0.33, 241 −0.65, 24f −0.25, 251 −0.29,
254 −0.26).

**4. The large overshoots are short transients at fast brake onsets.** There are 28 episodes across
7 routes where `aEgo` stays more than 0.8 below the command for at least 0.3 s. All have
`BRAKE_REQUEST` set, most last 0.3-0.7 s, and in every one the minimum `aEgo` is more than 0.8
below the minimum command, so reaction lag does not explain them.
- About half peak at a saturated command: 237 cmd −3.42 → `aEgo` −5.98; 254 −3.46 → −5.21;
  258 51:30 −3.39 → −4.04 (`dv/dt` agrees), then the brake releases about 0.3 s behind the command.
- Low speed after a gas-to-brake flip, the STATUS 67 stop-and-go jolt: 258 63:50 at 7.7 m/s went
  from +0.8 to −1.5 in 1 s and `aEgo` reached −2.8. 3 of the 28 episodes are below 10 m/s.
- The routes predate and postdate the guard trims (237/241 against 258), so this is car behaviour,
  not a regression.

**Superseded by item 72:** over-brake scales with brake depth, not onset rate, so the "fast brake
onsets" wording in this heading and in point 4 is wrong.

**What this does not establish.** Whether ECU overshoot depends on the command's rate (jerk) rather
than its level. The data suggests rate, but no replay has varied it. On the road, a brake onset
overshoot is conservative for collision but is felt as a jolt.

**Options (none implemented; any change that lowers a brake command needs a decision first):**
1. Leave it. It errs toward more braking, and a mid-range command is tracked within about 0.2.
2. Limit command jerk at brake onset, below 10 m/s and on gas-to-brake flips only. This targets the
   stop-and-go jolt and does not change the peak deceleration available.
3. Do not scale down saturated commands to cancel the overshoot. It would cut peak braking in exactly
   the emergencies where it is needed.

## 72. Brake over-delivery scales with how hard openpilot asks, not how fast the brake builds; the close-lead cap is not the jolt's cause. Parked, open to revisit. Replay (log decode and closed-loop planner) evidence only.

Follows item 71. Peter asked (1) whether a StarPilot planner guard causes the fast brake onset and
could simply be deleted, as with the item 64-66 trims, and (2) for option 2 of item 71, an onset
jerk limit. Result: no guard to delete, and option 2 is not supported. **Nothing changed in code.**

**1. The 258 63:50 stop-and-go jolt runs through `get_close_lead_brake_cap`, but the cap is not
the problem.** From 49.72 to 51.8 s `longitudinalPlan.aTarget` equals `closeLeadBrakeCap` on every
frame. The first frame replaces the MPC's +0.45 with −0.81, because the cap arrives at full strength
when projected TTC crosses `CLOSE_LEAD_BRAKE_CAP_MAX_TTC` (10 s). Its value is mostly
`0.7 * aLeadK`, and here the lead really was braking (aLeadK to −2.7). A closed-loop replay from
49.0 s (CL_TAU 0.35, lead not reactive) compares three runs:

| Run | reaches −1.0 | peak cmd | min gap |
|---|---|---|---|
| cap on (as driven) | 49.81 s | −1.57 | 9.83 m |
| cap off | 49.96 s | −1.75 | 9.89 m |
| cap off below 10 m/s | same as off | −1.75 | 9.89 m |

Without the cap, the MPC reaches −1.0 only 0.15 s later and then brakes harder. The cap was also kept
by item 64 on highway gap evidence. It stays. The MPC's own swing is +0.45 to −1.0 in about 0.65 s.
The car's `aEgo` of −3.0 against a −1.5 command came about 0.8 s later, while the command was steady.

**2. All 79 brake onsets on the 7 item-71 routes.** An onset is the command crossing −1.0 from above
−0.2, with none in the previous 4 s, followed by 3 s with long control engaged and no pedal.
Over-brake is min `aEgo(t+0.35)` minus min command over those 3 s, pitch-corrected; negative means
the car braked harder than asked.

| onset rate (m/s³) | n | median over-brake | share worse than −0.8 |
|---|---|---|---|
| < 1 | 49 | −0.42 | 14% |
| 1-2 | 8 | −0.37 | 0% |
| 2-4 | 3 | −1.31 | 67% |
| ≥ 4 | 19 | −0.11 | 5% |

- Faster onsets over-brake **less**: the correlation of rate with over-brake is +0.46, where positive
  means less over-brake.
- The least-squares fit is `over = −0.13 + 0.019·rate + 0.268·cmd_min + 0.004·v`. Over-brake is about
  27% of the requested depth, and rate and speed add nothing.
- The worst case is 237 at 762 s: −3.5 requested, −5.98 delivered, at an onset of only 0.7 m/s³.
- Gas-to-brake onsets (previous command ≥ +0.3), n = 16, have the same median as the rest (−0.40
  against −0.38).

**3. The one group that looks different is too small to act on.** The 2-4 m/s³ band has 3 onsets,
2 of them 258's low-speed stop-and-go jolts:
- 63:50 at 7.7 m/s: −1.6 requested, −3.04 delivered;
- 2882 s at 2.4 m/s: −1.0 requested, −2.58 delivered.

There are 6 onsets below 10 m/s in total. Rule 5 applies: do not tune on a handful of points.

**Decision (Peter, 2026-09-23): leave it for now; it is a topic to revisit.** Onset-jerk limiting
(item 71 option 2) is not supported by this data. Compensating for the ECU's depth-proportional
over-delivery would lower brake commands and needs Peter's decision. If it is ever tried, it should
be behind a toggle that defaults off, after replay, and never applied to saturated or emergency
commands (item 71 option 3).

**To revisit:** collect low-speed stop-and-go rlogs until there are about 10 or more onsets below
10 m/s, then rerun `ob5.py` and `ob5sum.py` (in the `oprad-routes` volume at `/routes/an2`, not
committed). The question is whether low-speed gas-to-brake flips over-brake beyond the 27% depth
trend. If they do, retry option 2 there only.

Scripts (not committed, in the `/routes/an2` volume):
- `ob4.py`: plan, guard, lead and command trace.
- `ob5.py`, `ob5sum.py`: the onset census.
- The closed-loop replay is the item-64 `replay.py` plus a `CLC` env switch (`0` = cap off,
  `v<thr>` = off below thr m/s).

## 73. ICBM (formerly Redneck Cruise) on Honda Bosch stock long: Curve Speed Control now lowers the set speed, and the feature is renamed. Static and unit evidence only; not driven.

**What changed (d9b0aa313).** Ported the intent of sunnypilot's ICBM: under stock long with the
`RedneckCruise` param on, openpilot sends SCM_BUTTONS RES_ACCEL / DECEL_SET (0x296, ≤20 Hz) so
the car's own ACC set speed follows openpilot's target. Stock ACC still does all following and braking.

- `starpilot/common/starpilot_variables.py`: `honda_icbm_active()` (available ∧ not pcmCruiseSpeed ∧
  not openpilot long) and `curve_speed_controller_available()` (openpilot long **or** ICBM). CSC was
  previously hidden under stock long.
- `starpilot/controls/lib/starpilot_vcruise.py`: `csc_long_control_active()`. controlsd only sets
  `longActive` under openpilot long, so CSC used `controls_enabled ∧ ICBM` instead. This feeds only `csc_available`.
- UI label "Redneck Cruise" became "ICBM" and the description names Honda Bosch as well as Hyundai.
  **The param key `RedneckCruise` is unchanged**, which avoids a migration.

**Lead-aware target (existing behaviour, documented here).** `select_redneck_target_speed` in
`selfdrive/car/redneck_cruise.py` runs on Honda with no separate toggle. With a lead it holds the set speed while following,
coasts early when closing within 4 s headway (1 mph plus up to 3 mph, scaled by headway), and
steps up 1.25–3 mph when the lead departs. It is capped at the set speed, with a floor of 25 mph on Honda. It needs
`longitudinalPlan.hasLead` plus `radarState.leadOne`. Without radar dRel/vRel it only holds.

**Tests.** The new unit tests pass (`-n0`). 4 pre-existing failures (test_starpilot_card
`test_very_long_press…` and three in test_wheel_controlsd) fail identically on the pre-change tree.
The UI tests need pyray and were not run. ruff count unchanged at 63.

**Open, needs a stock-long drive.** (1) Confirm lead dRel/vRel reach radarState under stock long.
(2) Measure SCM_BUTTONS during a real driver long-press on +/− (5 mph steps) before any
hold-mode emulation is written. Injected frames interleave with the car's own frames, so they may register as taps.
