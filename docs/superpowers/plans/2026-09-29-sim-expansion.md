# Sim expansion plan (draft for Kevin + James review)

2026-09-29, John (Metadrive Sim). Requested by Peter. Sim evidence only; nothing here changes device behaviour.

Order (Kevin's and James's answers folded in): 1 (with takeovers), 2 crawl, 2b override capabilities,
then 3, 4, 5. Starts after the PR 10 verdict. Driver override stays the main line of work; this plan uses sim time between
override chains and never runs two sims at once. Each phase gets its own prereg before it produces numbers.

## Phase 1: real-drive calibration (sim fidelity check)

Goal: score Peter's real drives with the same metrics as the sim, so every sim result can say how far the sim
sits from the car.

- Input: Jacob's per-drive `lat_pid_sim.npz` export, built offline from rlogs in tools/drive_plots (pending
  Peter's OK to Jacob). 100 Hz; keys t, angle, des_angle, cc_torque, v, eps_torque (= carState.steeringTorque),
  active, pressed, plus sr/stiff/offset/roll/des_curv.
- Scorer: a no-lane-truth mode in tools/sim/pr10_metrics.py, as its own commit; the frozen PR 10 copy
  (4b8bf07a) is untouched. Skips lane_off/inside_off and sim-step gap masks; controls-gap masks stay.
- Match: for each 296 route piece, score the real drive's same stretch and the sim's A-arm seeds. Report per band
  real vs sim median (rms_err, wig, rough, step_p99). Route IDs only; no coordinates.
- Output: a per-metric "sim offset" table. Metrics where sim and car disagree by more than the seed spread get
  flagged as not trusted for verdicts until explained.
- Takeover episodes too (Kevin): same 600/300/0.3 s episode rule on road and sim; compare t90, rate_pk (0.1 s LS
  slope), |cc| at release, release gap, re-press 0.3-0.8 s. Every override verdict rests on sim takeback behaving
  like the car; today there is one point (t90 road ~0.72 s vs sim ~0.77 s).
- Matching set: 296 pieces plus 294 and 297 hands-off chunks (same controller and params; 297 has the densest
  turn-fight / flicker episodes). 287/289 report only, kept out of the offset table (different params; James).
  Only where Jacob's npz exists; no new fetches.
- Plant-ID check (James): replay each real stretch's logged torque open loop through the sim plant and compare
  angle and yaw rate, so a gap can be put on the plant or on the controller context.
- Flag dt-dependent metrics: PR 10's FF assumes a fixed dt and real drives have controls gaps.
- Logging: steeringPressed and the EPS fault/flicker state as the car reports them (297's no-torque faults were
  all while pressed).
- Determinism: find what makes twin runs repeat (fixed seeds, traffic off); exit rows were voided on twin
  spread 0.63 m.

## Phase 2: crawl speeds (2-5 m/s)

Goal: close the "crawl gate untested in sim" gap (PR 10 Amendment 4 note; handsoff_12 / turn_12 voids).

- First find why runs under ~4 m/s never reach the scored span (cruise floor after engage, bridge speed
  control, or the model).
- If fixable in the bridge: crawl rows (straight pull-away, 90 deg turn from a stop, parking-lot S-curve)
  at 2, 3, 4 m/s. Metric: 0.4-3 Hz wheel rms (the 286 wobble), turn error for |des| > 20.
- If not fixable: record it as a sim limit and leave the crawl gate on road evidence only.

## Phase 2b: driver-override capabilities (Kevin, priority order)

a. EPS fault-flicker injection: STEER_STATUS=2 (NO_TORQUE_ALERT_1) -> steerFaultTemporary while pressed, so
   latActive drops, the ClarityEps core resets and the limiter re-seeds (turn-fight cause b; needed to test the
   parked ride-through idea). Road templates: 290 s11, 294 s14, 297 seg56 28-40 s.
b. Plan-tighter-than-wheel crawl turn (cause a): intersection turn with the plan 25-50 deg tighter than the
   driver's line at 11-20 mph; scripted plan offset if the sim model does not produce it.
c. Driver-hand model calibrated from road episodes: hold-torque distribution (incl. just under 1800), release
   shape, re-press timing. The scripted hand's "+150" result showed verdicts are sensitive to hand reaction.

## Phase 3: wind and road crown

Magnitudes from liveParameters, 294/296/297, one segment in three (James; Kevin adding the 297 rate of change):
- Roll median 2.25-2.33 deg on all three routes, p5-p95 ~0.05-3.5 deg (~4 % crown, p95 ~6 %). Levels 0, 2, 4, 6 %
  both directions. The roll estimate may be biased; rank against the Phase 1 calibration.
- angleOffset average -0.66 to -0.90 deg, fast value +-0.5 deg around it (p5 -1.34, p95 -0.18): a 0.5 deg step in
  the plant's offset; check that the learner and integrator absorb it.
- Wind: no direct measure; side force sized to the lateral accel of 1-2 % crown.

Goal: test angle-offset and roll handling, which the flat, calm sim never exercises.

- Constant lateral force (wind) and a banked straight (crown 1-3 %), each in both directions.
- Metric: lane offset drift on straights, time to settle after onset, and whether liveParameters' angleOffset
  and roll follow in the sim.

## Phase 4: learned-parameter mismatch

Goal: how forgiving the controller is when learned params lag reality.

- Steer ratio barely moves on the car (15.0-15.45, ~+-1.5 %): +-5 % is enough.
- Stiffness moves: ~1.00 on 294/296, 297 median 1.10, p95 1.43. Plant at -10, +10, +25, +45 % while the
  controller keeps the learned value, plus a slow ramp of learned stiffness within one run (297 drifted in a
  single drive).
- Driver-torque injection hook (pulses, hold, release), shared with Phase 2b; reuse the override one if it fits.
- Crawl-gate row once Phase 2 holds 2-5 m/s; until then the PR 10 crawl gate stays road evidence only.
- Metric: rms_err and wig per band against the matched run, same seeds.

## Phase 5: more routes, lane changes

- Convert more of Peter's routes into sim maps, preferring stretches with disengages or takeovers
  (route-by-log rule; route IDs only).
- Scripted lane changes and merges, using the recorder's existing lane_change field.

## Later, only if Peter wants it: longitudinal

Today the bridge has no lead cars, traffic or radar. A scripted lead car in MetaDrive (brake, cut-in,
slow for a curve) would test the planner through the model's lead. Synthetic radar into radard is a much
bigger job and would not be realistic about noise, ghosts or dropouts, which is where the radar work lives;
replay data stays the right tool for that.

## Questions for Kevin and James

1. Order: Phase 1 first (it tells us which sim metrics to trust)? Anything you would put ahead of it?
2. Phase 1 match: 296 route pieces enough, or do you want the 287/289/294 hands-off chunks too?
3. Phase 3/4: which magnitudes match what you see on the car (wind, crown, param lag)?
4. Anything from driver-override or VFN Shadow work that needs a sim capability not listed here?
