# StarPilot 7.0 progress log

The progress log for `night-star7-longradar`, the default branch since 2026-10-24.

The old `STATUS.md` and `DECISIONS.md` on `iq-nav-on-pr10` / `ns-bosch-radar-testing` are **retired history**:
- STATUS entries 1–265 live there.
- D-numbers are still cited from there.

Do not add 7.0 work to them. Add new entries here, at the top of **Log**, newest first.

**Evidence words.** Every claim says *static*, *replay* or *limited road evidence*. Nothing below is road-validated unless an entry says so.

## Where things stand (2026-10-24)

| Change | longradar | V-ASM (`night-star7-vasm-jetson`) | Evidence |
|---|---|---|---|
| Civic brake-onset relief back in (smaller gaps accepted for smoother brakes) | 093fd404d | a553e4157 | replay |
| Force Stop: a red light seen late brakes steadily | 7f9bdb4e7 | 6701db95f | replay + static tests |
| Force Stop: a gas tap keeps Force Stop out for its full 10 s | 2e869b83d | e384f2b12 | static tests |
| Civic stop-and-go: fewer brake on/off cycles, no gas surge after release (now `BoschABrakeEdge`) | 7c8e5eee1, 71f3cc035 | 5b7c73e20, 2af60a652 | replay (route 1f) + static tests |
| Brake-onset relief + stop-and-go fix follow the Bosch-A harness (Civic + CR-V 5G), not Civic only | 44d05a2db | f361d7d4e | static (CR-V); Civic unchanged |
| Curve brake fix: a radar lead whose accel flips from + to - within 1.5 s gets less braking authority (entry 7) | 8a36e513e | 30e08d987 | replay (route 30) + static tests |

**The comma** runs V-ASM 30e08d98 (2026-10-26; has the Bosch-A harness brake fixes and the curve brake fix). `BoschAStockBrakeFeel` is 0 on it.

## Open items

- **Curve brake fix (entry 7).** On the comma since 2026-10-26. Check the next drives for curve brakes and for a real brake that came out weaker than it should.
- **Far brake taps, routes 34 and 37 (entry 9).** The weak-lead bound (camera-contradicted unconfirmed lead, or camera passed) is on branch `night-star7-longradar-weakbound`, waiting for Job's replay. It is not on the comma.
- **Stop-and-go fix.** Judge it on Peter's next stop-and-go drive: brake toggles per minute, short holds, gas surge after release.
- **D-048 radar-lead corroboration is shelved.**
  - The unshipped code is on branch `d048-corroboration` at a0d0bf865.
  - The reason is in the D-048 addendum and STATUS 264 on `iq-nav-on-pr10` (5ae8485d3).
  - Ideas not yet tried:
    - the radar point's lateral position against the model's lane lines;
    - driver overrides as a label, over many more hours.
- **Route 20 T285–287.** The hard slowdown looks like the radar locked onto the wrong object (STATUS 263). This is still ambiguous on the stills.
- **D-097 is not on this branch** (the gentle gas ramp after a coast, 6d9579ce5). `CivicBrakeEdge` covers the release-surge part on the Civic.

## Log

### 9 — 2026-10-27 — Far brake taps (routes 34 and 37): a radar lead the camera contradicts gets less braking authority; side branch, replay pending

**Problem.** Two brake taps behind a far car that was holding its speed:
- route 34 T479: -1.9 for about 2 s, car about 60 m ahead (entry 8);
- route 37 T903: -1.8 for about 1.5 s at 30 m/s, an SUV moving out of the lane (entry 8 addendum).

Every rule in entry 8 read the radar lead's own motion, and every one also fired on real slowdowns.

**New data** (Job, 2026-10-27):
- 20 Hz packs with every radar track and the raw radar CAN for both taps and the four real far brakes (2f T736, 10 T188, 32 T1248, 0f T2190);
- 10 Hz CSVs and stills for all 39 engaged brakes on route 37;
- a census of the decoded F0 STATUS over 106 routes (18.5 engaged h; 46 of the 55 post-rebase routes) and 747 engaged radar-lead brakes below -1.0;
- 20 Hz CSVs and stills for 19 more brakes: 14 on a STATUS-9 lead, 5 where the camera had moved past the radar lead ("ev19").

**What the taps have and the real brakes don't.**
- **Route 34: the camera's range contradicts the radar's closing.** Track 55 swept in from 88 m to 52 m in 3 s. Over the last 1.5 s its vRel read -8 to -10.6 and its own range fell 10-19 m/s. The model lead held 61-66 m: its range fell only 0.7-1.9 m/s, within 3 m/s of its own closing speed (1.4-3.8 m/s), with a ≈ 0. The track also read STATUS 9 (not confirmed) until +1.48 s.
- **STATUS 9 alone is not a tell.** It covers 8.07 % of engaged lead0 time and sits under 39 of the 747 brakes. On the stills, 3 of the 14 STATUS-9 brakes were real: 2ed T338.6 (a truck), 368 T657.1 (stopped traffic) and 0c T342.7 (a car braking at 21 m). 2 more lean real: 37 T1606.2 and 313 T2330.2. The first version of this fix (c9fecd01a) weighted STATUS 9 alone and would have softened all five. It is replaced.
- **Route 37: the camera's lead had moved past the radar's and was not slowing.** Track 32 read 7. From the tap's start it held lead0 only through radard's relaxed gate: its range (45 m) was more than the strict window (0.25 × range, at least 5 m) short of the model lead's (59-72 m). The model lead was closing at -0.2 to +1.5 m/s, with a ≈ 0.
- **The range gap alone is not a tell.** On the same drive (T273-278) a pickup slowing to turn left held lead0 at 82 m while the model lead had moved on to 110 m. The gap reached the window's edge, and the plan went to -2.21 for a real slowdown. There the model lead itself was closing at 11.7 m/s. In the four real far brakes it closed at 4.5 m/s or more (median 9-13), and the gap stayed under 0.65 of the window.

**Fix** (`WeakLeadBound`, `starpilot/car/honda/radar_lead_bound.py`, default ON for Bosch-A). Each published radar lead gets a weight, the larger of:
- **Contradicted:** 0.6 × a ramp from 6 to 9 m/s of contradiction, only while its track's STATUS is 9.
  - The contradiction is how much faster the radar lead closes than the model lead's range falls. It is the smaller of two gaps: the camera's range slope minus vRel, and the camera's range slope minus the track's own range slope. Both slopes are least-squares fits over the last 1.5 s.
  - It is 0 when the model lead slows harder than -0.5 m/s².
  - It is 0 when the camera's range slope and its own closing speed disagree by more than 4 m/s, because then it has jumped to another car.
  - It is 0 when either slope has fewer than 8 samples or covers less than 0.6 s. Model ranges with prob < 0.5 are left out.
- **Camera passed:** 0.6 × a ramp from 0.75 to 1.0 of the strict window, only while the model lead is past the radar lead, closing at ≤ 2 m/s and slowing no harder than -0.5 m/s².

The lead's vLead/vLeadK/vRel and aLeadK move that fraction of the way toward the model lead, through the curve brake fix's blend (entry 7):
- only toward the less severe side;
- only while the model lead's prob is ≥ 0.35;
- never while the lead is urgent (dRel < 30 m or TTC < 4 s);
- dRel is never moved and the point is always published (D-041/D-042).

A new track takes its weight at once. Otherwise the weight rises over 0.25 s and releases over 1 s.

**STATUS transport.** car.capnp is pinned on 7.0. So card sends the decoder's STATUS rows in `starpilotCarState.radarPointStatus` (custom.capnp fields 38 and 39, additive), right after the radarTracks message they describe, stamped with its logMonoTime. radard pairs them within 250 ms. Missing, stale or unpaired rows read as empty, which turns the contradicted tell off. Routes recorded before this change have no rows.

**Evidence** (log emulation with the module's own functions, not a replay). Contradiction is the peak from -2 to +1 s around the brake.

| Case | On the stills | Contradiction | Weight |
|---|---|---|---|
| r34 T479 (tap) | not slowing | 9.9 | 0.05 at its lead0 start (-0.40 s, plan +0.05); 0.57 at 0 (-0.37); 0.60 from +0.19 to +0.70 (-0.71 → -1.58); releases to 0.43 at +1.0 |
| r37 T903 (tap) | SUV moving out | STATUS 7 | camera passed: 0.06 → 0.60 within 0.3 s, while the plan went -0.35 → -1.12 (low -1.84) |
| 2f T736, 10 T188, 32 T1248, 0f T2190 | real | 0, also with STATUS taken as 9 | 0 |
| 2ed T338.6 | real: a truck the camera saw late | 5.8 | 0 from the contradiction; the camera-passed tell gives 0.12 at +0.85, moving vLead ≤ 0.6 m/s |
| 368 T657.1, 0c T342.7 | real | 0 (the model lead was slowing, then jumped to a farther car) | 0 |
| 37 T1606.2, 313 T2330.2 | lean real | 0, 0.9 | 0 |
| 312 T814.0 | not slowing (a one-frame vLead dip) | 9.1 | 0.59 |
| 20 T285.6 | ambiguous | 9.8 | 0.60 |
| 2f8 T721.6, 313 T3291.5 / 313 T336.2 | not slowing / ambiguous | ≤ 2.7 | camera passed: 0.60 |
| 312 T1412.2, 312 T1980.7 / 20 T302.3, 2f7 T688.5 | not slowing / ambiguous | ≤ 5.6 | 0 |
| The five camera-passed brakes in ev19 | 0 real, 2 not slowing, 3 ambiguous | STATUS 7 | camera passed: 0.29-0.60. At 0c T469.1 (0.30) the radar was right about a nearer pickup; vLead is not moved, aLeadK by ≤ 0.29 |
| Route 37: the six real slowdowns on the stills, T273 pickup included | real | not run (10 Hz, no STATUS) | camera passed: 0 |
| Route 37 T323.6: SUV turning right | ambiguous | not run | camera passed: at most 0.27, around its -1.12 |

**Tried and dropped on the way.**
- STATUS 9 alone (c9fecd01a): it softened the three real and two lean-real STATUS-9 brakes.
- The track's range slope against its own vRel: it disagreed by as much on the real 10 T188, 2f T736 and 37 T1606.2.
- The camera's range against vRel alone, without the camera self-check and the track's range slope: it touched 368 T657.1 and 313 T2330.2.
- The range gap without the camera check: it gave the T273 pickup 0.52 at its -2.21.
- The radar's own velocity uncertainty (AUX): it rose on route 37 (78 → 260), but it also rose on the real 32 T1248 brake (194 → 452).

**Remaining risk.**
- *Contradicted tell.* A real lead the radar has not confirmed, which the camera sees late by more than 6 m/s while its range stays consistent with its own speed. 2ed T338.6 came within 0.2 m/s of that (5.84). Above 6 the weight grows 0.2 per m/s, and it ends when the lead becomes urgent (30 m or TTC 4 s). 25 of the census's 39 STATUS-9 brakes have no stills yet; the replay covers them.
- *Camera-passed tell.* The camera's range is long by more than about 19 % (0.75 of the 25 % window) while a far lead really brakes and the camera does not yet see it closing. The bound could then soften that early brake until the lead becomes urgent. Nothing like that is in the data so far.

**Status.**
- Static tests:
  - 14 tests in `tests/test_radar_lead_bound.py` pass:
    - the tells, the slopes, and the contradiction on logged values;
    - the slew, the urgent bypass and the transport;
    - route 34's and route 37's shapes run through radard, plus an unconfirmed lead the camera backs, which is left alone.
  - The Honda radar and Bosch-A decoder tests: 80 pass. 8 fail on `No module named 'tinygrad'` (card startup imports modeld), which is the environment only.
- On branch `night-star7-longradar-weakbound`, not yet on longradar or V-ASM.
- Next: Job's replay, with STATUS rebuilt from CAN because older routes have no rows. It covers:
  - both taps, the four real far brakes, and route 37's slowdowns and turn-offs;
  - the ev19 brakes;
  - the census's 747 brakes open loop, and its 149 post-rebase brakes below -1.5 closed loop.

  It ships only if the real brakes stay the same.

### 8 — 2026-10-26 — Brake tap on route 34: a new radar track's range settling read as a braking car; no fix

**Problem.** On route 34 (11c8fa231c0499ed/00000034--010fbd6343, T479–481) the car braked to -1.9 for about 2 s behind a car about 60 m ahead that was holding its speed. The stills show the gap steady.

**Cause.** Radar track 55 first reported the car at 76 m. The model had it at 64 m, at 13.8 m/s. In about 1 s the range settled to 55 m. The radar's speed filter read that settling as a car doing 6–8 m/s and braking at -3 m/s². The range fell about 20 m/s while the track's own vRel said about 10 m/s. The build had the curve brake fix (entry 7), but there was no + → − reversal, so it correctly did nothing.

**Check** (Job, log level, 25 routes, 4.36 engaged h). The flag: a new lead0 track whose range rate disagrees with its own vRel by more than 5 m/s for at least 0.3 s, in its first 2 s.
- It flags 21 of 403 new tracks, about 4.8/h. Only 5 were followed by aTarget < -1.0.
- 3 of the 79 engaged brakes below -1.5 came within 3 s of a flag, not counting route 34 itself (2f t736, 10 t188, 0f t2190). Job counts all 3 as real early braking, but no stills were checked.
- None of those 3 is under 30 m or under 4 s TTC, so the urgent bypass would not protect them.
- At other thresholds (3 / 8 m/s, windows 1.5–3 s), true hits and real-brake hits rise and fall together.

**Decision.** Not built. On its own, the flag would also soften real early brakes, the same reason D-048 was shelved (STATUS 264). Watch for more route-34-shaped taps. If more appear, try the flag together with a model speed disagreement, and check stills on 2f, 10 and 0f.

**Addendum — route 37 (11c8fa231c0499ed/00000037--34e868f7ba, T903–905), a different shape.**
- **What happened:** at 30 m/s on the highway, a white SUV about 45–50 m ahead was moving out of the lane to the right (stills at T901, T903 and T904). Radar track 32 had been the lead for a while. Its vLead dipped 28.7 → 24.8 in 0.6 s, with aLeadK -2.9, then went back up to 31 (aLeadK +3.7) as the SUV left. The model lead held v 29 and a 0, and then jumped to the next car at 69 m.
- **Effect:** the plan dipped to -1.8 for about 1.5 s, and the car slowed 29.8 → 28.2 m/s.
- **Why nothing caught it:** this is − then +, so the entry 7 latch correctly did nothing. The brake comes before any reversal could be seen. The track was old, so the route 34 new-track flag would not fire either.
- **Rest of the scan:** Job checked the other 29 engaged brakes in dirs 4–20. None was route-34 shaped, and none tripped the mismatch flag.

**Addendum — lead-leaving-lane scan and a combined rule** (Job, log level, 14 routes, 2.95 engaged h; 2026-10-27).
- **Cut-out flag** (lead's radar offset from our path > 1.0 m and moving outward > 0.3 m/s for ≥ 0.3 s): about 23/h. Of the 8 brakes below -1.0 while flagged, only route 37 T903 was a car leaving with no brake. About 4 were cars turning off at intersections that really slowed, one with brake lights on (o31 T451.7). Those need their braking.
- **Combined rule** (cut-out or new-track mismatch; model lead a > -0.5; radar and model speed differ by > 3 m/s; dRel ≥ 30; TTC ≥ 4): about 9.5/h.
  - It catches route 34.
  - It catches route 37 only 0.6 s after the plan was already at -1.84, so it would not have prevented that tap.
  - Of the 64 engaged lead0 brakes below -1.5, it also hits 0f T2091 (camera ambiguous) and 32 T1248. On 32 T1248 the radar saw vLead fall 15.8 → 6.6 while the model lead did not see it. The stills are at night and inconclusive.
  - About 5 of its other flags were turn-offs or exits that really slowed.

**Decision.** Neither rule is built. Both fire on real slowdowns, mostly cars turning off, and the combined rule is too late for route 37. Both taps (34 and 37) stay open, logged as radar artefacts with no bound that spares real slowdowns yet.

**Addendum — far-lead brake onset ramp** (Job, replay, 2026-10-27).
- **Rule tried:** for a new brake behind a radar lead at ≥ 30 m with TTC ≥ 4 s, braking may deepen only from -0.3 at J m/s³ for the first 2 s.
- **Fidelity is low.** Across 67 events the median aEgo corr is 0.35, but the first 5 s track well (planner vs command corr 0.95, vEgo RMSE 0.30). Read the base-vs-J deltas, not the absolute numbers. Route 34 is not reproduced: the sim planner never goes below -1.

| J (m/s³) | r37 time below -1.0 / min aEgo | Real brakes losing > 3 m of gap or TTC < 3 s |
|---|---|---|
| base | 1.15 s / -1.70 | — |
| 0.5 | 0.25 s / -1.06 | 4 (c0f 719 -5.3 m, c0f 2092 -4.2 m, o2b 558 -3.0 m with TTC 2.94, o32 1250 -3.1 m) |
| 0.8 | 0.75 s / -1.31 | 1 (c0f 2092 -3.3 m; its camera call was itself ambiguous) |
| 1.2 | 1.00 s / -1.60 | 0 |

**Decision.** Not built. Where the ramp helps the tap, it gives up gap on real brakes. Where it gives nothing up, it barely changes the tap. The fidelity is too low to pick J = 0.8 on one tap.

### 7 — 2026-10-26 — Curve brake fix: a radar lead that flips from speeding up to braking within 1.5 s gets less braking authority

**Problem.** On route 30 (11c8fa231c0499ed/00000030--70508f00bd, T1421–1424) the Civic braked hard to -2.87 while following a car through an S-curve. The car ahead was not slowing. Radar track 60 read the lead as speeding up at +4 m/s², then braking at -3 m/s², all within 1 s. A vision-only drive would not have braked: the model kept the lead at 47–52 m and about 21–22 m/s.

**Change.** `ReversalBound` in `openpilot/starpilot/car/honda/radar_reversal_bound.py`, called from `BoschALeadPolicy.leads()`. It is Bosch-A only and default ON.
- **Trigger:** the published lead's aLeadK on one radar track goes above +2.0, then below -2.0, within 1.5 s. Only that order counts.
- **Latch:** that track is distrusted for 4 s. A new reversal extends the latch.
- **Bound:** while latched, vLead/vLeadK/vRel and aLeadK move 60 % of the way toward the model lead. They only move when the model lead is the less severe one, and only when the model lead prob is at least 0.35.
- **What is never changed:** dRel. The point is always published (D-041/D-042).
- **Bypass:** none of this applies when the radar range is under 30 m or its TTC is under 4 s.

**Evidence** (limited road evidence, log level):
- **Positive case:** route 30 is the only one found. Across 18 radar routes, radar leads had 18 reversals and the model lead had 0. Only route 30's reversal was followed by lead0 braking the plan below -1.0.
- **Real brakes:** 0 of the 58 engaged lead0 brakes below -1.5 had a reversal in the 4 s before them.
- **Route 30 estimate:** about -2.87 → -1.3. This comes from a regression on that route's rows, **not a replay**. At T1422.3, Job's blended fields were vLead 15.67 → 19.16 and aLeadK -3.0 → -1.42.
- **Tests (static):** 11 new tests in `tests/test_radar_reversal_bound.py`. The Honda radar suite (22) and radard `test_leads.py` pass.

**Replay** (Job, 7.0 planner, bound applied to the logged leads, not radard rerun from CAN):

| Case | Base | Fix |
|---|---|---|
| Route 30 curve brake, aTarget min (log -2.87) | -2.20 | -0.72 |
| 13 real engaged hard brakes (aT -2.0 to -3.5, dRel 8.7–61.9 m) | — | identical, max change 0.0 |
| Other reversals where it changed anything | 30_23 T167.7 -1.68 → -0.92; 30_24 T203.9 -0.65 → -0.10; 0d_5 T307.8 -0.30 → -0.05 | never braked harder |

- **Fidelity** (route 30 seg 23, base run): aTarget corr 0.992, RMSE 0.20; aEgo corr 0.945. The sim brakes less than the log at the event (-2.20 vs -2.87).
- On 30_23 T167.7 the sim base itself is off: the log reads -0.03.
- A 14th control (0f/38) clipped to one frame and is excluded.
- Only per-event fidelity on route 30 was checked.

**Shipped:** V-ASM 30e08d987, on the comma 2026-10-26. **Not verified:** not driven.

### 6 — 2026-10-26 — Curve brake on route 30: radar misread the lead in an S-curve; no fix yet

**Event (Peter).** `11c8fa231c0499ed/00000030--70508f00bd`, about T1422: a semi-hard brake while following a lead through a curve.

**What happened (log + stills, limited road evidence).**
- The road is a two-lane S-curve with guardrails on both sides. The lead was 4–6 m off-centre (yRel -4.35 to +6.4).
- Radar track 60 is the same ID throughout. Its range swung from 45 to 54 m, then down to 39 m, over about 5 s. Over the same time the model lead held at 47–52 m with a flat a, and in the stills the lead is not closing.
- Radar vLead went 22 → 27.6 → 16.1. aLeadK went +4.25 to -3.0 (clamped) in about 0.8 s.
- The negative half took the plan (lead0) to aTarget -2.87. aEgo reached about -3.0, from 22.5 to 18.2 m/s. There was no override.
- Not involved: curve speed, Conditional, Force Stop, a track swap, or a lane change.
- This is the same class as D-048: the radar's numbers agree with each other but describe the wrong thing.

**Candidate discriminator screened (log, Job).** Rule: on the same track, aLeadK goes above +2 and then below -2 within 1.5 s (or the mirror case).
- Data: 12 radar routes (0d–2a, 30), 1.87 h engaged with a radar lead.
- 16 radar hits, against 0 for the model lead under the same rule.
- Outside route 30 it fires at about 2/h, mostly harmless. The route 20 T282 fault is one of those hits.
- Adding |yRel| >= 3 m and curvature > 0.003 isolates route 30, but misses 20 T282. That would be fitted to one road.

**Not built.** If more curve brakes happen on other roads, rerun the rule on them (Job: `rq3/hits.csv`). Any fix would only bound the radar lead's braking authority, never drop the point (D-041/D-042).

**All of Sunday Oct 25 (log + stills, limited road evidence).** Routes 2b, 2c, 2e, 2f, 31 and 32. 2d is not on the device.
- Settings were the same on all six: radar on, Stock Brake Feel 1, Experimental 100 %.
- Driving time: 1.27 h long-active, 0.63 h with a radar lead.
- **Brakes:** 32 brakes below -1.5, 8 of them below -2.5. None is a route-30-type radar misread.
- **The closest case,** 31 T354.6 (-2.87 at 9.4 m/s): radar range was 12 m shorter than the model's. The stills show a real car close ahead at a stop-sign intersection, so the radar range looks right.
- **Route 32's three hard brakes** (-2.8 to -3.05): model-only, with no radar lead. They were red-light stops.
- **The other hard lead brakes:** radar and model agree.
- **The reversal rule** fires about 6 distinct times per hour on these routes. None of those hits lines up with a brake below -1.5.
- So route 30 is still the only example, and the rule still does not separate it.

**Also checked:** route `0000002e--efb7a0c7da`, the "strange stop and creep". Cruise was off and Peter was driving (seg 2 T143–178, which is seg 3 on a 1-based count), so openpilot did not act.

### 5 — 2026-10-25 — Tonight's drives after the brake fixes: calmer on radar, one hard brake at a red light

**Question (Peter).** How is long now on tonight's drives after the fixes? There was no stop-and-go.

**Routes and settings** (dongle 11c8fa231c0499ed; build and settings from each route's initData):
- All four routes ran a V-ASM build that includes f361d7d4e, with Experimental on.
- **24:** vision-only ACC (radar off), personality 0, 0.14 h long-active.
- **25:** radar, personality 0, 0.31 h.
- **27:** radar, personality 1, 0.22 h.
- **2a:** radar, personality 1, 0.02 h, so no usable lead frames.
- Stock Brake Feel was not read from these logs. It is 1 on the comma now.

**Results (log, limited road evidence; Job's definitions; radar routes 25 + 27 + 2a pooled).** This is 0.55 h, against 1.74 h for the radar baseline (routes 0d–20).

| Measure | Tonight, radar | Radar baseline 0d–20 | Route 23 (vision-only) |
|---|---|---|---|
| Lead-slowing jerk p50 / p95 | 0.13 / 0.67 | 0.15 / 1.13 | 0.13 / 0.65 |
| Coasting while the lead slows | 59 % | 48 % | 48 % |
| Brake onsets per hour | 3.6 | 17.8 | 4.8 |
| Hard brakes (below -3.0) per hour | 1.8 (1 event) | 2.9 | 4.8 |
| Driver overrides per hour | 9.1 | 14.4 | 4.8 |

- **The one hard brake:** route 27 at 2094 s, -3.16 at 10 m/s, no lead. The stills show a red light on an empty road, and the car stopped at the line. It was a correct stop that braked firmly late in the approach.
- **Brake on/off outside stop-and-go:** unchanged. Toggles per minute 16.1 against 15.7; runs under 1.5 s 55 % against 54 %. The brake-edge fix only targets the stop-and-go pattern.
- **Gas after a brake release:** p90 over the next 1 s is 0.17, against 0.26 in the baseline.
- **Jerk spikes (above 8):** three windows. One is a 0.07 s blip during a flicker between radar and model lead (27 seg29). One has no lead. One is a step from another plan source.
- **Overrides on route 24 (vision-only):** six in 0.14 h. They are not radar, and were not looked at further.

**Caveats.**
- About half an hour of radar driving, with 3 brake onsets and 1 hard brake.
- Personality and route mix differ from the baseline.
- Brake-request rate was approximated by onset rate.
- Nothing was replayed. Nothing is built from this.

### 4 — 2026-10-25 — Smoothing the radar lead's braking estimate: no gain, not shipped

**Question (Peter).** Route 23 (`11c8fa231c0499ed/00000023--9d20e00d2f`, vision-only ACC, Experimental) felt very smooth. Can it improve Bosch-A radar long tuning?

**Answer: not with what we tried.** Log and replay (limited road evidence).

**What route 23 showed (log).**
- Radar was off for the whole drive: 0 radar-sourced lead frames. It was 0.21 h of long-active driving.
- Behind a slowing lead, command jerk p95 was 0.65 on route 23, against 1.13 on the radar routes 0d–20. Median jerk and the share of time coasting (48 %) were the same.
- Confounds: Stock Brake Feel was 0 on route 23 and 1 on the radar routes, and route 23 is short.
- Radar's lead-acceleration estimate (aLeadK) is noisier than the model's: high-pass noise 0.104 against 0.072.
- The roughest frames were the handovers from a radar lead to a model lead: jerk p95 1.04.

**Variants screened on the signal (log, all bound-only, no radar point removed).**
- V1 (use model accel when it agrees with radar): rougher. Dropped.
- V2 (low-pass aLeadK, tau 0.5 s): fails the onset rule (more than 0.2 s late at -1.0). Dropped.
- V2b (same filter, but drops below -1.0 pass straight through within 0.1 s): passes, and cuts aLeadK jitter about 45 %. It is up to 2 s late crossing -0.5 on the D-048 faults.
- V2c (pass-through at -0.6), and HO (blend the radar-to-model handover over 0.5 s).

**Planner replay (7.0 planner, each route's own params and code, x86).**
- Fidelity is too low for absolutes:
  - route 23: 90.4 % within 0.05 overall, lead frames 58–70 %;
  - route 13: 67.7 % overall, lead0 37.3 %;
  - route 0f: 83.6 % overall, radar-lead frames 43.3 %.
- Under replay, plannerd skips the conditional/traffic host. The error runs both ways, so it looks like state and timing rather than a gain.
- As relative deltas against BASE on routes 13 and 0f, the variants are near no-ops:
  - jerk p95 moves by -0.13 to +0.04;
  - coast % and brake rate are unchanged or slightly worse;
  - onset delay at -1.0 is +0.00 s for every variant.

**Not shipped, and Stage B (closed loop) not run.** The far hard brakes, the 2ae/2f8 cases and the 31 onset events were not replayed at plan level.

**Where the roughest radar-drive jerk comes from (log, 4 worst windows, stills checked).** The radar drives' worst slowdowns peak at 11.7–16.7 m/s³ of aTarget jerk, against 1–5 on route 23. In each window the stills show a real in-lane lead slowing toward a light or through a work zone.
- **0d s4 @43.2:** a two-frame step, -0.35 to -1.44. The lead flickers between radar track 38 at 44 m and a far model lead at 60 m. The step lands right after a flip. The track ID does not change.
- **0d s20 @10.2 and 0f s7 @22.8:** the step is in the e2e (Experimental) output. The radar lead's fields are smooth, the track ID is constant, and modelProb is 1.0.
- **0d s21 @11.9:** no lead at all. It is a cruise-source release oscillation.
- Net: the jerk is mostly the Experimental e2e output, not radar's lead numbers. That is why filtering aLeadK did nothing. The one radar-related case is a flicker between radar and model lead, a single example. Nothing is built on it.

**Reopen** if a replay reaches about 80 % or better fidelity on radar-lead frames, or if a drive shows rough braking that traces to aLeadK jitter or a radar-to-model handover.

### 3 — 2026-10-24 — Weak-Lead Cap Ease not ported: 7.0 launches don't have its jolt

**Question (Peter).** Weak-Lead Cap Ease from 6.7.9 (`WEAK_LEAD_CAP_EASE_JERK` on ns-bosch-radar-testing) smoothed launches behind a car pulling away. Should it be ported into Stock Brake Feel if it helps stop-and-go?

**Answer: no, not on the data we have.**
- **Route 1f:** the ease only shapes *positive* acceleration requests. On 1f openpilot's request stayed between -0.1 and -0.3; the jolt there came from the car's brake and gas handling, which entry 1 fixes.
- **7.0 code:** 7.0 has no weak-lead cap at all, so there is nothing for the ease to attach to.

**Job's scan.** Log-based (limited road evidence), dongle 11c8fa231c0499ed, routes 0d/0e/0f/10/13/1a/1f/20: 72 launches at under 5 m/s behind a lead pulling away.
- **Cap shape** (request held low, then a jump of at least 0.5): 0 of 72.
- **Rises:** the largest rise within 0.25 s has a median of about 0.2 m/s². The worst low-to-positive crossing is 0d 32198, -0.21 -> +0.55, gradual.
- **Jerk:** command jerk p95 has a median of about 0.7 m/s³.
- **Outliers:** the jerk outliers are near-zero-speed stop/start handoffs, not rises.
- **Newborn radar lead:** 13 of 72 launches had one; none clamped.
- **6.7.9 reference:** on route 332 the same detector flags the cap shape on 00/02/05, with jerk p95 3.3-8.4.

**Caveats.**
- About 1.5 h of long-active driving, almost all Experimental Mode.
- No camera stills, because there was no step to look at.
- The scan shows there is nothing to fix on 7.0. It does not show that the ease couldn't help.

**Reopen** if a 7.0 launch shows a held-low request followed by a step.

### 2 — 2026-10-24 — Brake fixes are a Bosch-A harness fix

**Problem.** The brake-onset relief and the stop-and-go brake fix were tied to the Civic. Peter wants them on every car that runs the Bosch-A radar.

**Change.** `bosch_a_harness(cp)` in `bosch_longitudinal.py` uses the same car gate as `BoschARadar`:
- a verified Bosch-A car (Civic Bosch, CR-V 5G);
- with its radar available;
- with no radarless, CAN-FD or alt-radar flag.

Both fixes run when it passes. They are always on, with no toggle. `CivicBrakeEdge` is renamed `BoschABrakeEdge`, and it now also runs in the shared (non-Civic) path. That path's gas force also includes the wind road load, so the -0.20 threshold means the same thing there.

**Not taken.** Job's patch 109f9e5fd would have put both fixes behind the Stock Brake Feel toggle. That toggle is off on the comma, so the patch would have switched both fixes off on the Civic.

**Evidence.**
- **Civic:** behaviour unchanged.
- **CR-V 5G:** static only. The fits are from the Civic, and the CR-V has not been replayed or driven.
- **Tests:** the honda suite shows the same 190 environment failures before and after; the 2 new tests pass.

### 1 — 2026-10-24 — Stop-and-go brake cycling on the Civic (carried over from STATUS 265)

**Problem.** In stop-and-go traffic the Civic kept switching the brake on and off. Each time the brake let go, the gas jumped.

**Cause.** The brake request turned on at -0.12 and off at -0.02, measured on gas force plus road load. That band is so narrow that it limit-cycled on route 1f. On release, the gas was allowed to rise straight to the lookup value.

**Change.** `CivicBrakeEdge` in `opendbc_repo/opendbc/car/honda/bosch_longitudinal.py`. It is Civic only and default ON.
- The brake request turns on at -0.20.
- Once on, the brake holds for at least 1.5 s. It can escape early if the force goes above +0.2.
- After release, gas starts at 35 and ramps to the lookup value over 1 s.

**Replay results** (Job's harness, real class, route 1f, base → fix):

| Measure | Base | Fix |
|---|---|---|
| Brake toggles per minute | 3.69 | 3.17 |
| Brake holds shorter than 1.5 s | 27.6 % | 16.7 % |
| Gas at release | 60 | 35 |
| Mean gas over the 1 s after release | 121 | 81 |

The first commit message (5b7c73e20) claimed 2.11 toggles/min and 0 % short holds. Those numbers came from a simplified simulation that left out the long-control-off resets and the escape. The figures above replace them.

**Not verified:** the change has not been driven.
