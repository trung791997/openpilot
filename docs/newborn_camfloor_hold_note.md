# Newborn camera floor (178958a350): why Bob is holding it (2026-11-17)

Case: route 000000ed--fc469136cd seg 6, 384.8-387.1 s (the SUV). A newborn radar track closed 65 -> 50 m. The quadratic range fit (_range_decel) read fake braking (aLeadK -2.75). The camera saw a calm car (a -0.2..-0.4). The plan went to -3.40 and tripped the stock-feel emergency. The patch fixes this case in replay. It is held, not rejected, for the reasons below.

## What the 9-route replay census found (e3 e5 e6 d1..d6 ed; replay only)
1. **It trimmed real braking.** e3 s12 tid52 is a real far fast closer. Base gives aT -1.62; the patch gives -1.24. Camera agreement does not tell a calm car from one that is really braking hard far away.
2. **Step changes.** When the camera match drops, or the lead turns urgent, the floor is released in one frame. aLeadK stepped 1.4-1.9 m/s^2 in one frame (e5 s1 tid5, e6 s14 tid59). Base never did that there.
3. **The floor value sits on a cliff.** -0.95 is just above STOCK_FEEL_LEAD_STOP_A (-1.0). The fix hinges on 0.05 m/s^2, tuned on one event.
4. **Concerns from the code read (Jason's review, confirmed):**
   - After the newborn window ends, the held floor slews down at 0.5 m/s^2 per s without re-checking the camera. A far lead that really brakes can be under-believed for several seconds.
   - The Bosch-A object list is itself camera-sourced. modelV2 agreeing with it is not fully independent corroboration.

## What Bob tried on top (bob-newborn-camfloor 7be0a9f349, replay only)
- Slew the floor (0.25 m/s^2 per frame toward more braking). The steps are gone.
- Lift only below TTC 8 s. e3 s12 is back to -1.62.
- ed s6 still lifted: aT min -2.27, no emergency latch.
- Still weak: the 8 s cut rests on two cases, and 7 phantom windows still differ from base.

## Direction now
Peter wants this folded into the existing ghost let-off (bosch_a_ghost.py, already on bob-unify-car and scored clean in replay). It should not be a separate newborn-only mechanism. The shared idea: braking that outlives its evidence is bounded by physics (ghost_a_req), not by a hand-set floor. A camera-only veto is not used.

Evidence level throughout: replay only. Nothing here has road evidence.
