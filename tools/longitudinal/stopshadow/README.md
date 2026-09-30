# stopshadow-radar

Working branch for the stopped-track shadow test (plan: Radar Work (Bob), `/tmp/rv/stopshadow/PLAN.md`), run by
Radar Work (Bob), Radar Knowledge (Jason) and UI Work + Tool Design (Steve). Branched from ns-bosch-radar-testing at
ae6ae25e8 (the Bosch-A track tool the corpus run is pinned to).

Peter, 2026-09-30: plots may go on this branch; nothing merges into ns-bosch-radar-testing until the work is verified.

- Evidence here is replay only. No car-side code changes on this branch.
- Route data is cited by route ID and never committed. Plots and small CSV summaries are fine.
- The shared checkout stays on ns-bosch-radar-testing. Commit here without checking out with
  `/tmp/stopshadow_commit.sh "<message>" <repo-path>=<local-file> ...` (temporary index + commit-tree).

## Status (2026-09-30)
- Yaw sign of the stationary reference -vEgo + w*yRel: settled on replay (dy/dt follows -w*x).
- D: no mount-angle offset detected; turning-case b ≈ +0.3 m/s likely selection, open. Lever arm L ≈ 3 m on 00000297.
