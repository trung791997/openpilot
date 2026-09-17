# Status

**As of: 2026-09-16**

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

**Lockout census** (proposed fix D-054): locked object-time 6.5 / 7.3 / 14.1 / 6.7 / 9.6 % and
followed-lead-lost ≥1 s episodes 8 / 8 / 9 / 1 / 2 on 232 / 236 / 237 / 239 / 23a.

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

17. **Open: D-054, the innovation-baseline lockout.** Proposed only. Replay K/tolerance on the lock
    episodes (census in the D-053 rework section), with the single-sweep range resets as the negative
    control.

18. **Open: `00000239` 10:33.7 phantom hard brake.** Looks like a lead association fault (lead
    track yRel −0.9 → −3.7 m while range fell 74 → 61.5 m, U11 +1.5 → −7.5, vision held 69–75 m).
    Neither the rework nor D-054 touches it. Needs a raw-track look at 625–635 s.
