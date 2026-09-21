# The Bosch-A radar work, in plain language

This is the story version of what `STATUS.md` and `DECISIONS.md` record formally. Those two files
are the evidence and the rulings; this one is the narrative you can hand to someone who has not been
living inside the repo. Where a claim matters, the item or decision number is cited so you can go
read the rigorous version.

**One caveat up front, and it applies to everything below.** This is longitudinal and lateral control
on a car that actually gets driven. Almost all of it is *replay* evidence — recorded drives pushed
back through the real code — plus a few hours of limited road evidence. Nothing here has been
validated on the road against real radar targets in the way that phrase normally implies.

---

## The car, the radar, and the one hardware fact that explains most of this

The Honda Bosch-A radar reports a lead's closing speed in a field called U11. That field saturates.
Above about 13.5 m/s of closing speed it simply pins — raw `0` at one end, raw `1728` at the other —
and the true value is gone. It is not noisy, not uncertain: it is **clipped**, and nothing downstream
can recover what it was.

Roughly a third of the work in this repo is consequences of that one sentence.

The first consequence is counterintuitive. When a stopped car is approached at 30 mph, the radar says
"closing at 13.5 m/s" when the truth is 19 or 25. So the car *understates* how urgent the situation
is. The tempting fix — treat a pinned reading as a broken measurement and ignore it — was tried, and
it was a safety regression. On one route, two stopped cars pinned the rail on all 88 frames; the code
discarded the reading, waited for a good one that by definition was never coming, eventually **deleted
the radar target entirely**, fell back to the camera, and commanded *zero braking* while closing on
stopped traffic at 76 m. The driver had to intervene.

That produced the rule the rest of the project is built on:

> **Deleting a radar point is more dangerous than publishing a degraded one.** When a check is
> uncertain, publish a bound and coast nothing. (D-041)

An understated closing speed still brakes. A deleted object does not brake at all.

---

## The bug everyone actually noticed: braking for the wrong car

The reported symptom was the car braking hard for a vehicle in the *adjacent* lane, usually on a
curve. Chasing it took three wrong turns, and the wrong turns are the interesting part.

### Wrong turn 1: "the braking is too aggressive"

The obvious read is that −3.50 m/s² is an overreaction. It is not. −3.50 is the configured floor,
and given the inputs the planner received, it is **correct arithmetic** (item 34). An off-path car on
a curve, seen by a radar whose speed field is pinned, looks like something nearly stationary directly
ahead. Any sane planner brakes for that.

So there is no "braking bug" to fix. The bug is upstream: the car should never have been told that
object was the lead. **All of the leverage is in lead selection**, not in the braking response.

### Wrong turn 2: a feature that could never have worked

There was a `FarLeadBrakeLimit` toggle — a cap on how hard the car may brake for a distant lead. It
sounds exactly right for this symptom. It was replayed with the cap forced ON and forced OFF across
four windows, about 3,900 planner cycles, including both of the worst far-lead hard brakes on record.

**It differed on zero cycles. It had never once engaged.**

The reason is a small masterpiece of unintended consequence. The cap stood itself down whenever
time-to-collision was under 10 seconds, on the theory that a genuinely urgent situation should not be
capped. But time-to-collision is computed from closing speed — the pinned one. A 90 m lead at a
pinned 13.5 m/s reads as 6.7 seconds to impact, so the cap politely excused itself on precisely the
frames it existed for.

> **The rail that causes the fault is what makes the cap blind to it.**

It could not be tuned out, either: to see past the rail the threshold would have to drop below the
rail's own apparent TTC of 3–7 s, which means capping real emergency braking. So it was removed
rather than retuned (D-060). It had shipped default-OFF, so removing it changes nothing on the road.

### Wrong turn 3: the fix that made things worse

With selection identified as the real problem, the natural fix is a geometric one. Compute `dyPath` —
how far a lead sits laterally from the path the camera predicts — and reject a radar lead that is far
off-path while the camera holds its own lead that is on-path.

Scored frame-by-frame this looked excellent: it touched only 3 of 113 hard-braking episodes fleet-wide,
and two of the three were exactly the reported symptom.

Then it was run through the real planner, and on one route it turned a **−1.55 m/s² brake into −3.50**.
Worse, not better, by a wide margin.

The mechanism is worth understanding because it generalises. Rejecting the radar lead did not leave
the car with nothing — it freed the slot, and the slot was immediately taken by the *camera's* lead,
which happened to be **nearer and closing twice as fast**. The gate fired on a single frame and
produced eleven frames of divergence, because the handoff outlived the condition that triggered it.

That is D-041 all over again, now with a number attached: deleting a radar point cost 2.3 m/s² of
extra braking on an episode where the radar was the gentler, more correct answer.

### What the fix has to look like instead

Two structural facts constrain it:

1. **There are two published lead slots**, and the planner takes the more urgent of the two. An
   earlier version of the gate covered only the first slot, which is why it barely helped — the
   offending track simply reappeared in slot two. Covering both cut time-in-hard-braking by 57%
   instead of 13%.
2. **Anything that changes *which* object is the lead can trigger a handoff**, and the handoff can
   cost more than the original fault.

So the design now on the table (**D-061, proposed, not implemented**) keeps the geometry signal and
throws away the rejection. The lead stays published and keeps its slot; what gets bounded is how much
braking authority it is allowed to command. No selection change means no handoff. It includes a clamp
that mathematically forbids the change from ever making braking *more* urgent than it would have been
— which turns the bad route from "happened not to regress" into "cannot regress".

It shortens the false brake. It does not eliminate it, and it should not be described as if it does.

---

## The lesson that cost two features

Both `FarLeadBrakeLimit` and the first draft of the `dyPath` fix died the same death: a
**time-to-collision threshold computed from a sensor channel that saturates**. On this platform TTC
does not measure urgency, it measures how pinned the radar is.

This is now a standing rule in `DECISIONS.md`: any TTC or closing-speed term added anywhere on this
platform must state, in its comment, which channel it reads and what it does when that channel is
railed. Two features have been lost to the unstated version of that question; there should not be a
third.

---

## Things that turned out to be true, and things that turned out to be wrong

Some findings that reversed earlier beliefs, kept here because the reversals are the useful part:

- **Radar and camera disagreeing about distance is normal, not a fault signal.** Beyond 60 m they
  differ by 10 m or more on about a third of frames, and those frames are *not* where the bad braking
  is. Every attempt to build a detector on the size of that gap, or on how fast it changes, fired
  constantly during ordinary driving (D-048).
- **Tuning a threshold on offline error statistics caused a measured on-road regression.** A radar
  quality threshold was lowered from 511 to 128 because the error statistics supported it. But that
  quality signal rises during genuine hard braking too, so the tighter threshold preferentially threw
  away *real* emergency manoeuvres. It was restored (D-042). This is why the constants in
  `radar_interface.py` carry their evidence in a comment block, and why you read the block before
  changing the number.
- **A saturated lifecycle counter is not a new object.** Keep publishing it (D-052).
- **A colleague's fix to the braking supervisor is correct but unexercised.** It only matters when
  the final commanded braking is much harsher than what the lead alone justifies — a curve limiter,
  a red light, a forced stop. None of the recorded windows contain that, so it reads as zero-effect
  in replay. That is "not yet tested", not "does not work", and it is reported that way deliberately.

---

## Where this leaves things

Removed: one feature that never ran. Proposed but not written: the bounded lead fix. Blocking it is a
prerequisite that is easy to miss — **`dyPath` does not exist in the running code at all.** Every
number quoted about it was computed offline in analysis scripts. Before any of it ships, the online
version has to be built and proven to reproduce the offline one, or the entire body of evidence is
about a quantity that never shipped.

The honest summary is that a lot of this work is *subtraction*: a feature removed for never having
run, a fix rejected for making things worse, several detectors refuted before being written. That is
slower than shipping, and on a system that brakes a car it is the correct trade.
