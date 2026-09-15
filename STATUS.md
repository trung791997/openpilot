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

**`FarLeadBrakeLimit` (`ebf4c20`) is a TEST feature, default OFF.** It bounds braking demanded
for a lead far away in both time and distance, and it is the only behavioural change in this
work. Corpus replay: 15,082 frames, 2,350 in regime, **8 frames altered — all 8 in the
flagged segment**, max reduction 0.89 m/s²; zero frames altered across the other 15 segments.
Its evidence base is **one positive example**, which is why it is off by default and labelled
TEST. Enabling it is a deliberate act: `Params().put_bool("FarLeadBrakeLimit", True)`.

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

See **Next** at the end of this file. The short version: the Konik path has never been run
against a live server from anywhere, so no route has reached an agent session by any route
other than a manual file upload; `test_leads.py` has never been run; and D-048's validation
gate is unmet, which is why the far-lead limit ships off.

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
- **`test_leads.py` has not been run** in this environment (process-replay harness).
- **No Konik route has been fetched or analysed from an agent session.** The network policy
  denied `konik.ai` for the whole of 2026-09-15; `tools/konik_preflight.py` has been
  exercised against its failure paths only, never against a live server.
- The checked-in `.so` files remain **aarch64**, and a local build still overwrites 53 of
  them. The SessionStart hook rebuilds and masks them, but a session that skips the hook
  will hit both.
- 🔴 **The recorded corpus numbers are still not reproduced.** The harness that can
  reproduce them now exists (`tools/bosch_a_corpus_report.py`, added below), but it has never
  been run against a route, so every figure in the 6- and 16-segment sections remains a
  number no one can currently re-derive. The original defect: the 6- and 16-segment results
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

1. **First route queued for Bosch-A analysis, not yet fetched or analysed.**
   Device `11c8fa231c0499ed`, route `11c8fa231c0499ed|00000231--5782493b00` (supplied
   2026-09-15). Nothing is known about its contents yet — whether it carries Bosch-A object
   frames at all, and whether any of them are real targets rather than the no-target
   sentinels every prior clean replay contained, is exactly what
   `tools/konik_preflight.py` is for. Do **not** cite this route as evidence of anything
   until that has run.
   ```bash
   python tools/konik_preflight.py \
     --dongle-id 11c8fa231c0499ed \
     --route '11c8fa231c0499ed|00000231--5782493b00'
   ```
   Requires a Konik token (`$KONIK_TOKEN`) and an environment whose network policy permits
   `api.konik.ai` **and** the storage host the signed segment URLs point at.
2. Get `test_leads.py` running in a real process-replay environment; it is the only radar
   suite still unexercised.
3. Re-run the `radard`/planner suites against the Alpha Long PR branches now that the
   architecture blocker is understood — both PR records list them as skipped for that reason.
4. **Done 2026-09-15** — a real-target route exists; see the section above. The follow-on
   is the shadow `vRelRange` channel (D-044): this segment carries 77 railed samples and an
   ~11 m vision/radar range disagreement, which is exactly the material that channel was
   published to be judged against. Compare `vRelRange` with U11 on this route before anything
   is wired into control.
5. **Done 2026-09-15** — a track-ID reuse does reset the lead Kalman filter, by object
   absence rather than by the incarnation boundary the parser computes, and the reset is
   carried by a single `liveTracks` message. See *Track-ID reuse and the lead Kalman filter*
   above and D-049. Two follow-ons, both needing route data: how often lifecycle breaks
   actually occur, and whether a **seamless** ID reuse (no lifecycle break at all) is possible
   — if it is, nothing resets anywhere.
6. **Highest priority: characterise the t≈11 s range-drift false brake.** It produced a
   −2.89 m/s² command and a driver override, and it passed the innovation gate, the
   one-sided rate check and the gross-distance gate. Needed before any fix: the distribution
   of radar-versus-vision range disagreement across routes where the radar is right, so a
   tightened `HONDA_BOSCH_A_GROSS_DISTANCE_M` can be justified rather than guessed (D-042).
   More real-target routes are the blocker.
7. **Run `tools/bosch_a_corpus_report.py` against `00000231--5782493b00` and compare it with
   the figures recorded above.** The harness was written 2026-09-15 and covers every quantity
   the 6- and 16-segment sections quote, but it has never seen a route, so the recorded numbers
   are still unreproduced. This is D-048's validation-gate item 2 and D-049's item 3; both
   designs stay blocked until it runs. Three outcomes, and the third is the valuable one:
   it reproduces them (the corpus becomes evidence again), it disagrees (**neither side wins
   automatically** — the recorded numbers came from code that no longer exists, and the harness
   has never been validated, so the disagreement is the finding), or it will not run on real
   data at all, which is itself worth knowing before anyone depends on it.
