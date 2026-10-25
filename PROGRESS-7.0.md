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

**The comma** runs V-ASM b95b393 (has the Bosch-A harness brake fixes, f361d7d4e). `BoschAStockBrakeFeel` is 0 on it.

## Open items

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
