# Obstacle Trajectory Interior Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Reconstruct the full 13-point obstacle trajectory `params[:,2]` that the longitudinal MPC solved against on each brake entry, and test whether its *interior shape* differs radar vs no-radar — the one thing STATUS 49 could not see.

**Architecture:** No solver and no replay. Everything from `radarState` to `params[:,2]` in `long_mpc.py` is pure numpy (`process_lead` → `extrapolate_lead` → stopped-equivalence → cruise obstacle → `np.min`); only `run()` needs acados. So the harness re-implements that pure prefix in a job-local script, drives it from the existing CSVs, and validates it against two independent anchors already proven or already logged. Only if Task 1 shows the model-lead path is live on entry ramps does this need rlog and modelV2.

**Tech Stack:** host `python3` only, no docker, no acados. Reuses `$CLAUDE_JOB_DIR/tmp/{phase,entry_dump,onset,obst_entry}.py`.

**Spec:** `STATUS.md` items 46–49 (the candidate list and its elimination). Item 49 states the limitation this plan closes.

## Global Constraints

- **NO EDIT to any control-path file is authorised.** The user said "take a look at fixing," which is not approval of a change. Every file this plan creates lives in `$CLAUDE_JOB_DIR/tmp/` (`/Users/peternguyen/.claude/jobs/3c03581d/tmp/`) and is job-local, never committed. The only repo file this plan writes is `STATUS.md`.
- This is longitudinal control on a car that gets driven. No result may be called safe or working without the word *static*, *replay*, or *limited road evidence*. Nothing here is road evidence.
- **Never `cd` out of the worktree in Bash** — it resets the primary working directory. This has happened three times. Use absolute paths, or `cd <worktree> && …` in one command.
- All 16 routes are ECO (`DecelerationProfile = 1`). Do not change the setting; it breaks corpus comparability.
- Do not re-open: phase/lag (dead, item 46), track handover (item 47), the BLoTv3 dials (item 48), the obstacle *inputs* (item 49). Do not quote the 0.3 s onset-jerk figure (p=0.208) or the withdrawn `desFollow − tFollow` pad figures.
- Importing `phase`, `entry_dump`, `onset` or `pad_entry` runs module-level reports. Wrap every import in `contextlib.redirect_stdout(io.StringIO())`.
- CSV `t` is raw logMonoTime in **nanoseconds**; `route_time = (t − min(seg_t0.values()))/1e9`.
- Verbatim constants from `long_mpc.py`, to be used and not re-derived: `COMFORT_BRAKE = 2.5`, `STOP_DISTANCE = 6.0`, `LEAD_ACCEL_TAU = 1.5`, `N = 12` (so the horizon is **13 points**), `ACCEL_MIN = -3.5` (`opendbc_repo/opendbc/car/honda/../interfaces.py:40`), `MODEL_LEAD_TRAJECTORY_MAX_LEAD_BRAKE = 0.5`, `MODEL_LEAD_TRAJECTORY_MAX_CLOSING_TTC = 7.0`, `LEAD_FILTER_TIME_LOW` → `current_filter_time` interpolated over `[47, 65]` mph.

---

## File Structure

| File | Responsibility |
|---|---|
| `$CLAUDE_JOB_DIR/tmp/modelgate.py` | **Task 1.** Decides, per entry-ramp frame, whether `build_model_lead_trajectory` returns `None`. Pure CSV. Gates the whole plan. |
| `$CLAUDE_JOB_DIR/tmp/obstraj.py` | **Task 2.** Reconstructs the 13-point `lead_0_obstacle`, `cruise_obstacle` and `params[:,2]`, plus the endpoint validation. |
| `$CLAUDE_JOB_DIR/tmp/interior.py` | **Task 3.** Measures interior shape radar vs no-radar and runs the statistics. |
| `STATUS.md` | **Task 3 Step 6.** Item 50. The only repo file touched. |

---

### Task 1: Establish which lead-trajectory path is live on brake entries

If `build_model_lead_trajectory` returns a trajectory, `extrapolate_lead` never runs and the obstacle interior is the *model's* prediction — which is not in the CSV, making Tasks 2–3 impossible without rlog. Its own guard suggests it bows out exactly on brake entries, but that must be measured, not assumed. This task is cheap and decides everything after it.

**Files:**
- Create: `$CLAUDE_JOB_DIR/tmp/modelgate.py`
- Read only: `selfdrive/controls/lib/longitudinal_mpc_lib/long_mpc.py` (the `build_model_lead_trajectory` guard, already quoted below)

**Interfaces:**
- Consumes: `obst_entry.load(pfx)`, `entry_dump.rate(rows, i, jx, key)`, `phase.episodes(eng)`, `phase.truthy`, `phase.pct`.
- Produces: `model_path_live(row) -> bool | None` — `True` if the model path would win on that frame, `False` if the legacy raw path runs, `None` if the CSV cannot decide (no lead). Task 2 imports this to assert the legacy path holds.

- [ ] **Step 1: Write the gate, transcribing the guard from source**

The guard, verbatim from `long_mpc.py`:

```python
raw_lead_brake = max(0.0, -float(getattr(radar_lead, "aLeadK", 0.0)))
closing_speed = max(0.0, float(v_ego) - raw_v_lead)
ttc = raw_d_rel / max(closing_speed, 1e-3) if closing_speed > 0.1 else float("inf")
if (raw_lead_brake > MODEL_LEAD_TRAJECTORY_MAX_LEAD_BRAKE or
    (closing_speed > 0.75 and ttc < MODEL_LEAD_TRAJECTORY_MAX_CLOSING_TTC)):
  return None
```

Write `$CLAUDE_JOB_DIR/tmp/modelgate.py`:

```python
"""Task 1: does build_model_lead_trajectory() bow out on brake entries?

If it returns None the legacy extrapolate_lead() path runs and the obstacle interior is
reconstructible from the CSV. If it returns a trajectory, the interior is the model's
leadsV3 prediction, which the CSV does not carry, and this plan needs rlog instead.

The guard is transcribed verbatim from long_mpc.py. modelProb (`mProb1`) covers the
`prob <= 0.5` early return; the x/v shape and finiteness checks cannot fail on logged
data and are not modelled -- recorded as a limitation, not silently assumed away.
"""
import contextlib, io, sys

sys.path.insert(0, '/Users/peternguyen/.claude/jobs/3c03581d/tmp')
with contextlib.redirect_stdout(io.StringIO()):
  from phase import truthy, episodes, pct
  from entry_dump import rate
  from obst_entry import load

MAX_LEAD_BRAKE = 0.5
MAX_CLOSING_TTC = 7.0


def model_path_live(r):
  """True if build_model_lead_trajectory() would return a trajectory on this frame."""
  d, vl, ve = r.get('d1'), r.get('vLead1'), r.get('vEgo')
  if d is None or vl is None or ve is None:
    return None
  if not truthy(r.get('hasLead')):
    return False                      # radar_lead.status False -> early None
  prob = r.get('mProb1')
  if prob is not None and prob <= 0.5:
    return False
  brake = max(0.0, -(r.get('aLeadK1') or 0.0))
  closing = max(0.0, ve - vl)
  ttc = d / max(closing, 1e-3) if closing > 0.1 else float('inf')
  if brake > MAX_LEAD_BRAKE or (closing > 0.75 and ttc < MAX_CLOSING_TTC):
    return False                      # legacy raw path
  return True


def report(name, pfx):
  _, rows = load(pfx)
  eng = [r for r in rows if truthy(r['longActive']) and r.get('vEgo') is not None
         and r['vEgo'] > 5.0 and r.get('aEgo') is not None]
  ramp_live, ramp_tot, frac = 0, 0, []
  for (i, jx) in episodes(eng):
    ra = rate(eng, i, jx, 'aEgo')
    if ra is None:
      continue
    k0, ka = ra[3], ra[4]
    vs = [model_path_live(eng[k]) for k in range(k0, ka + 1)]
    vs = [v for v in vs if v is not None]
    if not vs:
      continue
    ramp_tot += 1
    f = sum(vs) / len(vs)
    frac.append(f)
    if f > 0.0:
      ramp_live += 1
  allf = [model_path_live(r) for r in eng]
  allf = [v for v in allf if v is not None]
  print(f'{name:14} entry ramps with ANY model-path frame: {ramp_live}/{ramp_tot}   '
        f'per-ramp live fraction p50 {pct(frac,50):.3f} p90 {pct(frac,90):.3f}   '
        f'| whole route {100.0*sum(allf)/len(allf):5.1f}% of {len(allf)} engaged frames')
  return ramp_live, ramp_tot


if __name__ == '__main__':
  tot = [0, 0]
  for nm, p in (('242 NO RADAR', 's_00000242'), ('251 RADAR', 's_00000251'),
                ('24f RADAR', 's_0000024f')):
    a, b = report(nm, p)
    tot[0] += a; tot[1] += b
  print(f'\nVERDICT: {tot[0]}/{tot[1]} entry ramps contain a model-path frame.')
  print('  0/39      -> legacy path is universal on entries; Task 2 proceeds CSV-only.')
  print('  otherwise -> STOP. Those ramps need modelV2.leadsV3 from rlog; report and ask.')
```

- [ ] **Step 2: Run it**

```bash
cd /Users/peternguyen/openpilot-radar/.claude/worktrees/repo-state-assessment-6969b6 && \
  python3 /Users/peternguyen/.claude/jobs/3c03581d/tmp/modelgate.py
```

Expected: a verdict line. The guard's own comment ("keep the legacy raw-lead path so an optimistic model horizon cannot delay the first braking response") predicts `0/39`, but a non-zero count is a real possible outcome, not a bug.

- [ ] **Step 3: Branch on the verdict — this is a decision point, not a formality**

- `0/39` → record it as Task 2's precondition and continue.
- Any ramp with a model-path frame → **stop and report to the user.** Do not silently switch to rlog: that is a much larger job (modelV2 decode, docker with the repo at `/src/openpilot`) and it is the user's call whether it is worth it. State how many ramps and on which routes.
- Mixed within a ramp (`0 < fraction < 1`) → worse than either, because the obstacle *switches definition* mid-entry. That is itself a publishable finding and possibly the mechanism. Report it as such.

---

### Task 2: Reconstruct the 13-point obstacle trajectory and validate it

**Files:**
- Create: `$CLAUDE_JOB_DIR/tmp/obstraj.py`
- Read only: `long_mpc.py` `extrapolate_lead` (`:~915`), `index_function`, `common/filter_simple.py`

**Interfaces:**
- Consumes: `modelgate.model_path_live`, `obst_entry.load`, `entry_dump.rate`, `phase.episodes/truthy/pct/mean`.
- Produces: `t_idxs() -> np.ndarray` (13 points); `extrapolate(x_lead, v_lead, a_lead, a_lead_tau, v_ego) -> np.ndarray` shape `(13, 2)`; `obstacle(row, v_lead_f, a_lead_f) -> np.ndarray` shape `(13,)`; `replay_filters(eng) -> list[tuple[float, float]]` giving the filtered `(v_lead, a_lead)` per engaged frame.

- [ ] **Step 1: Write the failing validation test first**

The reconstruction has two independent anchors, and the test asserts both before any measurement is trusted. Anchor A is proven in STATUS 49: `obstacle[0] − safe_distance[0] == d1 − desFollow` to within ~0.5 m. Anchor B is the CSV's `leadX0_end` / `leadV0_end`, which are almost certainly `lead_xv_0[-1]` — **but they come from the external dumper in the `oprad-routes` volume, not from repo code, so the test must treat a mismatch as "anchor B unproven", not as a reconstruction bug.**

```python
def test_anchor_a_frame0_identity():
  """obstacle[0] - safe_obstacle_distance[0] must equal d1 - desFollow (STATUS 49)."""
  res = anchor_a_residuals('s_0000024f')
  assert abs(pct(res, 50)) < 1.0, f'frame-0 identity broken: p50 {pct(res,50)}'


def test_anchor_b_endpoint():
  """lead_xv_0[-1] should match the logged leadX0_end / leadV0_end."""
  dx, dv = anchor_b_residuals('s_0000024f')
  assert abs(pct(dx, 50)) < 2.0 and abs(pct(dv, 50)) < 1.0, (
    f'endpoint mismatch: dx p50 {pct(dx,50)}, dv p50 {pct(dv,50)} -- '
    'either the reconstruction is wrong or leadX0_end is not lead_xv_0[-1]')
```

- [ ] **Step 2: Run the test to verify it fails**

```bash
cd /Users/peternguyen/openpilot-radar/.claude/worktrees/repo-state-assessment-6969b6 && \
  python3 -m pytest /Users/peternguyen/.claude/jobs/3c03581d/tmp/obstraj.py -v
```

Expected: FAIL — `anchor_a_residuals` not defined.

- [ ] **Step 3: Implement the reconstruction**

Transcribe `extrapolate_lead` exactly. Note the `exp_weight` speed blend (`[0, 20, 35]` mph → `[1.0, 1.0, 0.0]`): at highway speed the exponential term is fully off and the trajectory is pure constant-acceleration, which is a substantive fact about what the interior even *is*.

```python
import numpy as np
from openpilot.selfdrive.modeld.constants import index_function   # if importable; else inline

MAX_T, N = 10.0, 12
T_IDXS = np.array([index_function(i, max_val=MAX_T, max_idx=N) for i in range(N + 1)])
T_DIFFS = np.diff(T_IDXS, prepend=[0.])
CB2, STOP_D, MS_TO_MPH = 5.0, 6.0, 2.23694


def extrapolate(x_lead, v_lead, a_lead, a_lead_tau, v_ego=0.0):
  exp_weight = np.interp(v_ego * MS_TO_MPH, [0, 20, 35], [1.0, 1.0, 0.0])
  if exp_weight > 0:
    a_exp = a_lead * np.exp(-a_lead_tau * (T_IDXS ** 2) / 2.)
    v_exp = np.clip(v_lead + np.cumsum(T_DIFFS * a_exp), 0.0, 1e8)
    x_exp = x_lead + np.cumsum(T_DIFFS * v_exp)
  else:
    x_exp = np.zeros_like(T_IDXS); v_exp = np.zeros_like(T_IDXS)
  v_const = np.clip(v_lead + a_lead * T_IDXS, 0.0, 1e8)
  x_const = x_lead + v_lead * T_IDXS + 0.5 * a_lead * T_IDXS ** 2
  return np.column_stack((exp_weight * x_exp + (1 - exp_weight) * x_const,
                          exp_weight * v_exp + (1 - exp_weight) * v_const))


def obstacle(x_lead, v_lead, a_lead, a_lead_tau, v_ego):
  """lead_0_obstacle over the horizon: lead_xv[:,0] + v**2/(2*COMFORT_BRAKE)."""
  xv = extrapolate(x_lead, v_lead, a_lead, a_lead_tau, v_ego)
  return xv[:, 0] + xv[:, 1] ** 2 / CB2, xv
```

`process_lead` clips before extrapolating and filters `v_lead`/`a_lead` through `FirstOrderFilter`s whose time constant moves with speed, so `replay_filters` must walk the route **sequentially from the first engaged frame** — the filter state is history-dependent and cannot be evaluated per-episode. Apply the clips in source order: `min_x_lead` first, then `v_lead >= 0`, then `a_lead` into `[-10, 5]`.

- [ ] **Step 4: Run the tests to verify they pass**

```bash
cd /Users/peternguyen/openpilot-radar/.claude/worktrees/repo-state-assessment-6969b6 && \
  python3 -m pytest /Users/peternguyen/.claude/jobs/3c03581d/tmp/obstraj.py -v
```

Expected: `test_anchor_a_frame0_identity` PASS. If `test_anchor_b_endpoint` fails, do **not** loosen the threshold to make it green — instead record that `leadX0_end` is not `lead_xv_0[-1]`, drop anchor B, and carry "interior validated at frame 0 only" as an explicit limitation into Task 3 and STATUS 50. A single validated anchor plus a source-exact transcription is weaker than two anchors and must be reported that way.

- [ ] **Step 5: Commit**

Nothing to commit — `$CLAUDE_JOB_DIR/tmp/` is job-local and deliberately untracked. Record the anchor residuals in the Task 3 notes instead.

---

### Task 3: Measure the interior and write STATUS 50

**Files:**
- Create: `$CLAUDE_JOB_DIR/tmp/interior.py`
- Modify: `STATUS.md` (append item 50)

**Interfaces:**
- Consumes: `obstraj.obstacle`, `obstraj.replay_filters`, `obstraj.T_IDXS`, `onset.mwu`, `entry_dump.rate`, `phase.episodes/pct/mean`.
- Produces: the STATUS 50 table. No downstream consumer.

- [ ] **Step 1: Define the interior metrics before looking at any number**

Committing to these in advance is what keeps the third consecutive negative result honest. Per entry-ramp frame, reconstruct the 13-point `params[:,2]` and compute:

1. `curv` — mean `|second difference|` of the obstacle **along the horizon** (not along time). Measures how bent the predicted trajectory is.
2. `dev` — max `|obstacle − straight_line(obstacle[0], obstacle[-1])|`, the departure from a constant-speed path.
3. `frame_jag` — mean `|second difference|` of the whole 13-vector **between consecutive frames**, i.e. how much the predicted trajectory jumps shape from frame to frame. This is the closest thing to "the planner is reacting to a moving target" and is the metric most likely to carry the effect.
4. `src_flip` — number of horizon points where `argmin(x_obstacles)` differs from the frame-0 source, i.e. the obstacle is lead-limited near and cruise-limited far. A source that flips mid-horizon is a discontinuity the solver sees but no logged scalar shows.

Aggregate p50 per route, pool radar (251 + 24f), and run `mwu(radar, no_radar)` on each. Pre-register the reading: **only `p < 0.05` with radar *larger* supports the hypothesis.** Radar smaller, or `p >= 0.05`, is a negative — and given items 47–49, expect a negative and report it plainly.

- [ ] **Step 2: Run the measurement**

```bash
cd /Users/peternguyen/openpilot-radar/.claude/worktrees/repo-state-assessment-6969b6 && \
  python3 /Users/peternguyen/.claude/jobs/3c03581d/tmp/interior.py
```

- [ ] **Step 3: Sanity-check the instrument before believing the result**

Item 48 shipped a meaningless metric because its premise went unchecked, and two of seven earlier "defects" were harness artifacts. So: pick the single most abrupt episode (251 `t=365.86`, entry −5.39) and the single smoothest (242 `t=139.71`, entry −0.20), print their full 13-point obstacle vectors, and read them. If the abrupt one does not look qualitatively different, the metric is probably measuring nothing — say so rather than reporting its `p`.

- [ ] **Step 4: Write STATUS 50**

Required sections, matching items 47–49: the method and its validation anchors; the pre-registered metrics; the table with n per route; the Mann-Whitney results with U, z and p; and a **"What this does NOT establish"** closing that states at minimum — n=14 vs 25 is underpowered; 251 contributes 3 episodes; the routes differ in drive, traffic and time; the filters were replayed but the solver was not run; no road evidence; no replay; no code changed.

- [ ] **Step 5: Commit STATUS.md only**

```bash
cd /Users/peternguyen/openpilot-radar/.claude/worktrees/repo-state-assessment-6969b6 && \
  git add STATUS.md && git status --short
```

Confirm `STATUS.md` is the *only* staged path, then commit as `2026-09-22 STATUS 50: <result>` with the two required trailers, and push.

- [ ] **Step 6: Report the outcome and the fork**

Two possible endings, and the plan commits to both now:

- **Interior differs on radar** → the mechanism is found. The fix is a rate limit or shape guard where the radar lead enters the trajectory. **That fix is not authorised by this plan.** Present it to the user with the evidence and the proposed diff, and wait.
- **Interior does not differ** → the inputs, the supervisor, the handover and now the trajectory are all cleared, which means the abruptness is in the **cost weights or solver conditioning**, not the inputs. That genuinely requires acados and a replay harness. Say so, give the user the cost-weight reading of `long_mpc.py:374–394` as the next hypothesis, and stop — a fifth CSV probe would be the confirmation trap.

---

## Self-Review

**Spec coverage.** Item 49's stated limitation is "the reconstruction is horizon index 0 only; a mechanism reshaping the interior would be invisible." Task 2 reconstructs all 13 points; Task 3 metrics 1–4 all measure interior structure; metric 4 covers the mid-horizon source flip that no logged scalar exposes. Item 49's second limitation, that reading the *solved* trajectory needs replay, is deliberately **not** covered — it is the Task 3 Step 6 fork, escalated to the user rather than assumed.

**Placeholder scan.** No TBDs. Every code step carries runnable code; the guard, `extrapolate_lead` and all constants are transcribed from source with line references. The one deliberately unwritten body is `replay_filters`, whose algorithm (sequential walk, clip order, speed-dependent time constant) is fully specified in Task 2 Step 3.

**Type consistency.** `model_path_live(row) -> bool | None` is defined in Task 1 and consumed in Task 2 under that name. `extrapolate` returns `(13, 2)`; `obstacle` returns `(array(13,), array(13,2))` and Task 3 consumes both. `T_IDXS` has 13 elements because `N = 12` — used consistently. `mwu` returns `(U, z, p_two_sided)` as in items 47–49.

**One gap I am leaving open deliberately.** If Task 1 returns anything but `0/39`, Tasks 2–3 are invalid as written and the plan stops for a user decision. I have not written the rlog variant, because committing to a docker/modelV2 harness before knowing it is needed would be the larger waste.
