# DECISIONS.md (night-star7-vasm-jetson)

Settled decisions for this branch, started fresh on 2026-11-06. Numbering starts at C-001, so these
numbers never collide with the D-numbers on other branches.

## C-001: Brake-onset relief and stop-and-go fix follow the Bosch-A harness, not the Civic alone (2026-10-24)
Decision: `bosch_a_harness(cp)` gates both fixes onto every verified Bosch-A Honda (Civic, CR-V 5G) with its radar available, always on, no toggle. Job's patch gating both fixes behind the Stock Brake Feel toggle was rejected, because that toggle is off on the comma and would have turned both fixes off on the Civic.
Evidence: commit f361d7d4e; PROGRESS entry 2. CR-V behavior is static only, not replayed or driven.
Revisit if: a CR-V 5G route is replayed or driven and shows different brake-onset or stop-and-go behavior than the Civic.

## C-002: Weak-Lead Cap Ease (6.7.9 port) not ported into 7.0 (2026-10-24)
Decision: Do not port the 6.7.9 Weak-Lead Cap Ease into Stock Brake Feel. 7.0 has no weak-lead cap for it to attach to, and a scan of 72 low-speed launches found none of the held-low-then-jump shape it was built to fix.
Evidence: PROGRESS entry 3; Job's scan, dongle 11c8fa231c0499ed, routes 0d/0e/0f/10/13/1a/1f/20.
Revisit if: a 7.0 launch shows a held-low request followed by a step.

## C-003: Radar lead-acceleration smoothing not shipped (2026-10-25)
Decision: None of the screened variants (V1, V2, V2b, V2c, HO) for smoothing the radar lead's acceleration estimate are shipped. Planner-replay fidelity was too low (58-90 %) to trust, and every variant was a near no-op against baseline on the deltas that could be trusted.
Evidence: PROGRESS entry 4; planner replay on routes 23, 13, 0f.
Revisit if: a replay reaches about 80 % fidelity on radar-lead frames, or a drive shows rough braking that traces to aLeadK jitter or a radar-to-model handover.

## C-004: Route 34/37 brake taps get camera corroboration, not a radar-only rule (2026-10-27)
Decision: No radar-only rule (new-track range-rate mismatch, cut-out flag, combined rule, or far-lead onset-ramp variants) is built for the route 34/37 brake taps, because every one also softened real slowdowns. Instead, `WeakLeadBound` bounds a radar lead's authority only when the camera contradicts it (an unconfirmed track closing faster than the camera sees, or a camera lead that has already passed it), shipped default ON for Bosch-A.
Evidence: PROGRESS entries 6, 8, 9; commit 90826c671 (synced); replay on the 19 real brakes and the 747-brake census (all moves softer, none a real in-lane slowdown on stills or the driver's call).
Revisit if: more route-34/37-shaped taps appear that `WeakLeadBound` doesn't catch.

## C-005: CameraSlowBound's camera-match window tightened to max(8 m, 10 %) (2026-10-30)
Decision: `CAM_SLOW_RANGE_M` 10 -> 8 and `CAM_SLOW_RANGE_FRAC` 0.2 -> 0.1, so a camera lead only counts as the radar lead within max(8 m, 10 %). Peter approved after Job's census found 2 false triggers and 1 leaning-false move, all with leads 80-95 m out.
Evidence: PROGRESS entry 12; replay re-run of the 16 movers from the original census.
Revisit if: a new census shows false triggers inside the tightened window, or a real slowdown is missed because of it.

## C-006: Dash map work tracked separately and done in a worktree (2026-10-31)
Decision: The Civic cluster map (`tools/head_unit/`) is worked in the git worktree `/tmp/hu_feed` on this branch, not in the shared `~/openpilot` checkout, as an explicit exception to AGENTS.md §7 approved by Peter. State, open items and resume steps live in `tools/head_unit/aaroad/DASH_MAP_LOG.md`, not in this branch's STATUS/DECISIONS files.
Evidence: PROGRESS entry 13.
Revisit if: another agent needs to edit the same head-unit files from the shared checkout, or the exception is withdrawn.

## C-007: Stock Brake Feel's far cap set to -0.6, gated to a 5 s time gap (2026-11-06)
Decision: The far-lead brake cap (TTC >= 25 s) is raised from -0.35 to -0.6, but only while the nearest lead is within 5 s of time gap; otherwise it stays at -0.35. The owner drove the ungated -0.6 on route 99, found it fine, then asked for the gated version after replay showed it also adding far-lead brakes (-1.58 at an 8 s gap, -1.04 at about 4 mph with the car far ahead).
Evidence: PROGRESS entries 18-19; commits cd4b892e3, 0161545c8; replay, 2.46 h pooled.
Revisit if: a drive shows the gated cap still braking for a car with a 5 s+ time gap, or shows the old light-plateau problem returning inside 5 s.

## C-008: Set Speed Assist removed from the forks (2026-11-06)
Decision: Set Speed Assist is removed from this branch, with its module, tests and both planner hooks deleted. Owner's call after replay: cruise jerk RMS was 0.208 off vs 0.476 on, and the car's own logs on 14 SSA-on units read 0.466 (limited road evidence). The best smoothing attempt, arm R4, recovered only part of the catch-up gain (0.235, keeping 71 % of it) and still braked deeper on slow traffic after an open-road stretch (p78 c78_4: -3.09 vs -2.33 off).
Evidence: PROGRESS entry 15; commit eb9a7e584; sunnypilot #2083, still open upstream.
Revisit if: sunnypilot #2083 lands upstream with a smoothing approach that keeps more of the catch-up gain without the added jerk.

## C-009: Low-speed brake assist (K05N05R1) shipped default ON for Bosch-A (2026-11-06)
Decision: mvl-boston's Bosch low-speed brake integrator is added for Bosch-A harness cars, default ON, to stop the Civic's stop ending in a slow crawl (it delivers 0.2-0.4 m/s^2 less braking than asked below about 2 m/s). Unlike the mvl/replay version, this stopping guard deliberately never makes the output shallower than the planner's command — it rate-limits only the extra brake, not the whole command, because an existing brake-onset test caught the whole-command version cutting a -1.5 request down to -0.002.
Evidence: commit 9e3c9eaaa; replay only (arm K05N05R1): 9b_S0 crawl at 0.5-2 m/s, 12.7 -> 7.2 s; the replay plant does not reproduce the final crawl, so the on-road gain is unmeasured.
Revisit if: a road drive shows no improvement in the final-crawl stop, or shows the extra brake arriving too abruptly.

## C-010: Keep an in-path closing radar lead when the camera's lead jumps away (2026-11-07)
Decision: When both the strict and relaxed camera matches fail, the previous radar lead is kept, not swapped for the camera-only lead, while it has |yRel| < 1.2 m, is closer than the camera's lead, is closing (vRel < 0), and has vLead > -1 m/s. While it is kept, the gross-distance and challenger staleness checks do not retire it. Default ON. The first draft used 1.5 m and had no speed floor. Camera stills showed two next-lane false keeps at |y| 1.44-1.48 and two cross-traffic false keeps while stopped (vLead -6.6 and -9.0). The 1.2 m bound also drops four real leads back to the old behaviour.
Evidence: route 9e seg 24 bookmark (owner drive, build fd3b0ec99d); Job's radard replay; a 46-event census (32 survive) with stills; closed loop on 8 events, where real leads in 82, 84 and 9b are braked for 0.5-1.5 s longer. Replay only.
Revisit if: a road drive shows a brake for a car already out of our lane, at an intersection, or from a glitched radar speed on a kept track.

## C-010: Camera cap for far radar leads the camera contradicts, no position gate (2026-11-08)
Decision: Branch `bob-camera-cap` (not on the car branch yet) adds `CameraCap` after `CameraSlowBound`: a far (>= 50 m) radar lead's vLead/vLeadK/vRel may be raised, never lowered, toward a confident, self-consistent camera lead at the same range (within 30 %) when the camera closes more than 3 m/s slower. It only removes braking, never past the camera's read; dRel, aLeadK and the point are untouched (D-041: publish a bound, never delete; D-042: the 8 m/s^2 slew in and out, no step). A gate on where the camera lead sits against the radar's (at/past it, or <= 5 m past) was tried and dropped: 71 (gap/dRel 0.15-0.32) and b0 17:13 (up to 0.17) overlap, and either gate lost one of them. No radar-only tell separates b0 17:13 (its range slope matched its doppler), which is why the camera is the witness.
Evidence: commit f8b9f99509; replay on the 93b pool, probes 71/8a/b0 and routes a2/a3 (numbers in STATUS and the module docstring); Peter's stills for b0/8a/71; branch code reproduces the prototype exactly (max |X-C| = 0.0000 on 23 groups). Replay only. Cost accepted: 80 seg19 second pulse 0.12 harder.
Revisit if: a road drive shows a real far slowdown braked late because the camera read it slow, or variants A/B/C (aLeadK lift, gap hold < 1, 40 m/s^2 slew-in; branch `bob-camera-cap-variants`) change the residuals on 71, 8a and b0 without new harder brakes.

## C-011: Camera cap also lifts aLeadK toward the camera lead (2026-11-08)
Decision: While the C-010 cap is active, `CameraCap` also raises aLeadK toward the camera lead's a (floored at CAMERA_DECEL_MIN -0.5), never lowering it, slewed at CAP_DA_RATE 10 m/s^2 per second in and out and restarted at 0 on a new track (D-042: no step). dRel and the point stay untouched (D-041). Variants B (hold until gap < 1) and C (40 m/s^2 slew-in) were not taken: they soften four further events (9b seg10, 99 seg15, 84 seg8, a2 seg10) that are not yet judged on stills, and slow the onset of 84 seg4, a real hard stop.
Safer B variants were also tried and not taken (Peter, 2026-11-08): D (hold limited to 1.0 s per engagement) and E (held target tapered from the full gap at 3 m/s to 0 at 1 m/s), alone and combined. No logged feature or simple pair (camera/radar range offset, gap, cam vRel/a/prob, raw vRel, range slope minus vRel, aLeadK, track age, vEgo, dRel) separates the phantom rows (71 s12, 8a s2, b0 s17) from the real ones (a2 s10/s11, 99 s15, 84 s4), so a hold can only be bounded, not targeted. D+E never braked harder than VA, but still softened real far slowdowns by up to 0.5 for up to 2 s (99 s15 -0.60 -> -0.24 over 1.95 s, 84 s4 lead-in -2.79 -> -2.63, a2 s11 -0.62 -> -0.26); its gains were 8a s2 -0.72 -> -0.58 and two events not judged phantom (a2 s22 is a police car pulling off ahead, 9b s10 unseen).
Evidence: replay by Job, 93b pool plus routes a2/a3, default config: 71 seg12 -1.35 -> -0.93, 8a seg2 -0.82 -> -0.72, b0 seg17 t5.7 -0.59 -> -0.11; b0 t51.6 unchanged at -3.40. Nothing brakes harder than the cap alone except 80 seg19's second pulse (+0.20). Full-route fidelity moved <= 0.006 in correlation. Static tests 13/13. Replay only.
Revisit if: a road drive shows a real far slowdown braked late while the camera lead read a > -0.5, or the 80 seg19 second-pulse cost shows up on the road.

## C-012: Camera cap mode setting, VA (default) or D+E (2026-11-09)
Decision: New boolean param `BoschACameraCapHold` (default "0"), a row beside the other Bosch-A toggles on the Long Planner page. Off is VA (C-010 + C-011, unchanged and byte-identical). On is D+E: the VA cap plus a limited tapered hold. With the gap floor dropped, a gap above CAP_GAP_MIN (3 m/s) targets the gap as before; while dv > 0, the gap is above CAP_GAP_HOLD (1 m/s) and at most CAP_HOLD_T (1.0 s) of hold has been used, the target is gap * (gap - 1) / (3 - 1); otherwise 0. hold_t resets on a new track and whenever dv <= 0, and deliberately not when the gap re-crosses 3 while dv > 0. The dv/da slews and the aLeadK lift are unchanged. radard reads the param once at construction (only an exact saved "1" is on; missing or malformed is VA), so a change applies on the next drive.
Update 2026-11-13: toggle retired on bob-unify-boostgate, D+E is the only mode (matches car d2cf07ba40). `BoschANeedFloorMinGap` retired the same way (always on, matches car ad641ed9ec).
Evidence: replay by Job (variants D+E, see C-011): never brakes harder than VA; softens real far slowdowns by up to about 0.4 m/s^2 for up to about 2 s (99 s15 -0.60 -> -0.24 over 1.95 s, a2 s11 -0.62 -> -0.26, 84 s4 -2.79 -> -2.63); cuts more residual phantoms (8a s2 -0.72 -> -0.58). Static tests only for the setting; not road-validated.
Revisit if: a road drive with D+E on shows a real far slowdown braked late, or D+E removes no phantom that VA leaves on the road.

## C-013: Held-vRel range-slope bound for far unmeasured radar leads (2026-11-11)
Decision: On `bob-unify` (replay branch, not on the car), `BoschALeadPolicy.leads()` bounds a radar lead's published vRel/vLead/vLeadK to the track's own range slope minus HELD_VREL_MARGIN (1 m/s) when the point is not measured (`track.direct` False), dRel >= 50 m, vRel <= -4, the range slope is >= -1 and disagrees with the held vRel by 8 m/s+, for 0.3 s+, and the lead is not urgent (`urgent()`: under 30 m or TTC under 4 s, using the last direct vRel). It only raises the speed (removes braking), never deletes the point, and leaves dRel, aLeadK and `track.vRel` raw (D-041: publish a bound, coast nothing). It runs before the camera witness. Cause it answers: ep26 (71 seg 9), a track at 82-85 m held vRel -9.51 unmeasured for 1.8 s while its range opened at +2 m/s. The earlier softening of ep26 by an un-rate-limited camera blend was dropped because it also softened ep292 (REAL, measured, slope agreeing with vRel); range and TTC do not separate the two, the measured flag and the slope disagreement do.
Evidence: commit b7468b5c2c; radard trace ep292 0 of 168 frames differ, ep26 41 frames; Job's scorecard vs `replay_unify6` (36 routes / 1141 episodes + 280-episode second set): 0 harder, 0 later, 3 softer (ep26, ep909, ep922, all Peter-labelled PHANTOM from stills, brake removed), ep1133 (REAL_MILD) onset and peak unchanged, no REAL row moved, second set row-identical; 9 static tests. Replay only (open-loop). Peter took it 2026-11-11.
Revisit if: a road drive shows a real far slowdown braked late while the radar point was unmeasured, or ep157/158-type phantoms (not reached by this bound) need the same treatment.

## Entry format
```
## C-NNN: <title> (YYYY-MM-DD)
Decision: ...
Evidence: ...
Revisit if: ...
```
