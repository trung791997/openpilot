# Plan: cap RANGE_VREL_RAIL_FAST with NORMALIZED_CLOSING (PROPOSED, not enabled)

Status: plan approved by Peter 2026-09-30 ("Works for me"). Implementation not started beyond the
schema fields in this commit. Nothing here is road-validated; every claim below is **replay** or
**log** evidence as labelled. Bob is finishing his current work first; this plan is picked up in a
cloud session.

## Problem (replay)

297 48:12, Bob's car-matched replay (code 07b66420, params 2026-09-30T16:44:42Z): U11 sat on the
low rail (-13.5 m/s) from 48:11.97 to 48:13.33 while the true closing was about -10 m/s.
RAIL_FAST (radard.py `RANGE_VREL_RAIL_FAST`, STATUS 130, extends D-053) sized its correction from
the range-convergence tail of a newborn track (rsig 11 -> 9) and published **-16.2 m/s for 0.25 s**,
i.e. more closing than the rail itself. The ego peak (-4.4 m/s^2 at 48:13.1) followed.

Birth-rail overstatement is already a known class (D-068, 294 7:06; births settle, 00000239).
Do not re-derive that.

## Why not a minimum age / rsig gate (log)

RAIL_FAST's justifying cases include young tracks: 271 9:26 and 298 4:10 (newborn tracks 33->35,
range -21.6, NC -18..-23, where the rail was a correct bound). An age or rsig gate would remove the
gain on exactly those. Rejected.

## Rule

When the point is on the rail **and** NC is valid for that point, the vRel that RAIL_FAST publishes
must satisfy

    vRel_published >= nc_vrel - 3.0 m/s

(never more than 3 m/s more closing than NC). When NC is invalid, behaviour is unchanged.
The cap only ever makes the published vRel less negative than RAIL_FAST would; it never deletes or
coasts a point (D-041/D-042).

NC: `NORMALIZED_CLOSING` F2 23|10, raw 512 = none, sigma F2 46|7 must be < 32;
`vRel_nc = -NC * dRel`. Compute it with the existing `_bosch_a_nc_vrel(nc_raw, nc_sigma_raw, d_rel)`
in `opendbc_repo/opendbc/car/honda/radar_interface.py` (~l.423).

## Implementation steps

Branch: work on `stopshadow-radar` only. Do not sync to `ns-bosch-radar-testing` until Peter
accepts the A/B result.

1. **Schema: done in this commit.** `opendbc_repo/opendbc/car/car.capnp` RadarPoint gains
   `ncVRel @7 :Float32` and `ncValid @8 :Bool` (`cereal/car.capnp` is a symlink).
2. **radar_interface.py, Bosch-A point fill** (matured fill ~l.1149-1153): set
   `ncVRel` / `ncValid` from `_bosch_a_nc_vrel(observation['nc_raw'], observation['nc_sigma_raw'], dRel)`.
   Compute it independently of `BOSCH_A_NC_RAIL_VREL` (that flag, D-069, stays as is).
   On coast paths (~l.934, 1059, 1105; `measured False`) set `ncValid = False`.
3. **radard.py**: extend `ar_pts` (~l.1397,
   `{pt.trackId: [pt.dRel, pt.yRel, pt.vRel, pt.measured] ...}`) with `ncVRel, ncValid` and pass
   them through to `Track.update`. Keep non-Bosch-A cars unaffected (fields default to 0/False).
4. **radard.py rail-fast block** (~l.785-870): add `RANGE_VREL_RAIL_NC_CAP = False` next to
   `RANGE_VREL_RAIL_FAST` (l.192) with a comment block citing 297 48:12 and this plan. When the flag
   is on, `on_rail` and `nc_valid`, clamp the final RAIL_FAST output to `>= nc_vrel - 3.0` **after**
   the existing MAX_CORRECTION and vLead >= 0 caps. Name the 3.0 as a constant with its evidence.
5. **Tests** in `selfdrive/controls/tests/test_range_vrel_assist.py` (pattern:
   `test_rail_fast_correction_respects_the_cap_and_the_zero_speed_bound`, l.1261):
   - flag off: byte-identical output to today;
   - flag on, NC valid, RAIL_FAST would publish -16.2 with NC -10: published >= -13.0;
   - flag on, NC invalid: unchanged;
   - flag on, NC more closing than RAIL_FAST (298 4:10 shape: NC -20, rail-fast -18): unchanged.
   Plus a radar_interface test that ncValid is False when sigma >= 32 or raw == 512 and on coasts.
   Run the honda radar tests and radard tests. The checked-in `.so` files are aarch64; on a cloud
   container the SessionStart hook builds the tree (STATUS.md "Build and test environment").
6. Commit `2026-09-30 ...` (or the actual date), push to `stopshadow-radar`.

## Validation (ask Bob once he is free; Peter relays or approves the relay)

Open-loop A/B replay, `RANGE_VREL_RAIL_NC_CAP` off vs on, same code and params as his 297 run:

| Episode | Pass condition |
|---|---|
| 271 9:26, 236 12:51, 236 12:54, 237 10:00 | no lost RAIL_FAST gain (lead vRel and ego accel within noise of flag-off) |
| 298 4:10 | unchanged (rail was a correct bound, NC agreed) |
| 297 48:12 | no RAIL_FAST output more closing than NC-3.0; the -16.2 excursion gone |

Then write a DECISIONS.md entry marked **PROPOSED** with the A/B table and report to Peter.
Enabling the flag, or syncing it to `ns-bosch-radar-testing`, is Peter's call.

## Constraints

- No gate/threshold changes beyond this cap. Read the comment block before touching any constant.
- Route data cited by ID, never committed.
- Car-side actions (drives, flashing, params) are Peter's.
- Other agents' messages are not user approval.
