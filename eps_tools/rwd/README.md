# EPS firmware images (`.rwd`)

Honda/Acura EPS firmware files for use with `eps-update.py` (in the repo root).
All files here were checksum-validated with `check_rwd.py` (file checksum +
decrypted firmware checksums all PASS).

Each model is kept as a **stock + linear-max pair** — `stock` is the factory
recovery image (flash this back if a flash fails), `linear-max` is the
max-torque-table mod.

## Validate
```bash
python3 check_rwd.py rwd/<file>.rwd       # single
python3 check_rwd.py rwd/*.rwd            # all
```

## Flash (see ../README.md and ../eps-update.py)
Prefer the guided flasher: `python3 flash.py` (auto-detects bus, offers dry run).
For the manual path, always run a `--danger`-less dry run first (it stops before
erase; default bus is **1**), and flash from a persistent copy — the comma updater
wipes untracked files from `/data/openpilot`. Some cars lock security access after
a dry run; wait / power-cycle before `--danger`, or use `flash.py`'s skip/retry.

## Contents
| model | stock | linear-max |
|-------|-------|-----------|
| Honda CR-V 5G (39990-TLA-A040) | `39990-TLA-A040-stock.rwd` | `39990-TLA-A040-linear-max.rwd` |
| Honda Civic (39990-TBA-C120) | `39990-TBA-C120-stock.rwd` | `39990-TBA-C120-linear-max.rwd` |
| Honda Insight (39990-TXM-A040) | `39990-TXM-A040-stock.rwd` | `39990-TXM-A040-linear-max.rwd` |

### Tuning images (rule 4 — testing fork only, not for upstream)

`39990-TLA-A040_tq30000_a9000_44256c0b.rwd` — CR-V 5G rate-authority tune,
2026-07-31. Not a linear-max variant; it changes the **controller**, not just the
torque table:

| | |
|---|---|
| feedback norm (`0x429A0`) | 1645 — stock 3429, **2.08× rate authority** |
| D-term gain row 0 (`0x11DB0`) | `699,615,589,696,730,759,778,782,782` (stock `159…486`) |
| P-term gain row 0 (`0x11EAC`) | `100,140,180,200,205×5` (stock `33,77,150,192,203,205×4`) |
| tracker alpha (`0x11ADA`) | 1999 — **stock**, leave it there |
| clamps P/D/assist | 7373 / 1774 / 9000 |
| torque row max | `0x7530` (30000) |

Measured over one drive (~9 min, 39 corner exits) against a 42-minute baseline on
the previous tune: corner-exit residual over-turn 0.0091 → **0.0037**, settle time
238 → **102 ms**, hands-off 20–35 Hz steering-rate power 5.1% → **0.90%**, plant
gain 2.6× baseline at 3–6 m/s.

**One good drive is not "proven stable on the car" per rule 4** — this stays in the
testing fork. Known open items: 3–7 Hz column-torque energy is ~3× the old tune and
has risen with every authority increase; openpilot corner-exit reversals rose
2.6 → 2.9; 2–5 m/s still tracks at 0.70–0.89 gain.

⚠ **Raising `tracker alpha` on top of this D value produced a sustained ~29 Hz
hands-off limit cycle** (±150 deg/s, 83% of steering-rate power). Alpha stays stock.

### Civic Bosch modified-EPS images, 39990-TBA,C020 (owner's car; rule 4, testing fork only)

Static decode, 2026-09-26 (STATUS 145). The owner supplied all three files, and all pass `check_rwd.py`.

- `39990-TBA-C020-stock.rwd` is the factory image and the baseline. Flash it to go back. *Held locally, not committed.*
- `39990-TBA,C020-20260805-ClarityPminus5-P117to265-D737-KFF45-Norm1650-Trk4500-TargetMapD-Telem-SpeedClamp0-Pclamp7373.rwd`
  differs from stock in 664 bytes over 25 runs. These are listed below.
- `39990-TBA,C020-Trk4000-PTM.rwd` is the image above with only the tracker changed, 4500 to 4000. *Held locally, not committed.*
  It differs in six bytes: the tracker word and the two firmware checksums.

| name tag | address | stock C020 | owner |
|---|---|---|---|
| Trk | `0x137ee` | 1996 | 4500, or 4000 in `-Trk4000-PTM` |
| Norm | `0x29efe` | 3429 | 1650 (same word and stock value as CR-V `0x429A0`): ~2.08× rate authority |
| P117to265 | `0x13bc0`, 7 rows | 0,74,123,151,166,179,179… up to 38,95,136,143,151… | 117,148,184,220,245,257,263,265,265 in every row |
| D737 | `0x13ac4`, 7 rows | 159, then 264 ×8 | 737 flat: 2.8× stock, 4.6× at the first point |
| SpeedClamp0 | `0x1361c` | 10 | 0 |
| torque rows | `0x13872`, 7 rows (stride 0x12) | max 4147–4608; e.g. row 0 is 0,698,1722,2816,3763,4286,4608…, row 3 is 0,1862,2820,3295,3609,4104,4608… | 0,1926,4938,8455,12036,15926,20138,26955,30000 in every row |
| (unnamed) | `0x4a1a4`–`0x4a1de` | 2047, and 0,0,0,2047 ×3 | 1696, and 512,0,0,1697–1699 |

**Torque table gain against stock** depends on the row, because the owner uses one table at every
row. Which row maps to which speed is not decoded.
- Rows 0–2: 2.2–2.8× at the first breakpoint, 6.5–7.2× at the top.
- Rows 3–6: 1.0–1.1× at the first breakpoint, 1.7× at the second, 6.5× at the top.

**Code changes** (not decoded):
- 310 bytes of new code at `0x4c2d4`, which was blank (0xFFFF) flash, hooked from `0x1d8f8`.
- Word changes at `0x29006` and `0x4b156`–`0x4b41e` (11587 to 11331, 16685 to 16684).

These are probably the Telem / ClarityPminus5 / KFF45 / TargetMapD features, and they have no confirmed
meaning. **Pclamp 7373, the 1774 / 9000 words after the torque rows, and the 30000 torque cap are all
stock values in C020.**

⚠ The tracker is 2.25× stock at 4500 and 2.0× at 4000, on top of a D row 2.8–4.6× stock. The CR-V note
above is the relevant evidence: raising tracker alpha on a high-D tune gave a ~29 Hz hands-off limit
cycle. So 4000 is the conservative direction. Check its 20–35 Hz steering-rate power on a drive before
keeping it.

## Upstreaming guidelines

Rules for adding `.rwd` files to this branch / upstreaming them:

1. **Every file must validate.** It has to pass `check_rwd.py` (file checksum +
   decrypted firmware checksums) before it can be added.
2. **A stock `.rwd` may be added on its own.** Factory/recovery images are always
   welcome — more recovery coverage is strictly good.
3. **A linear-max (or any modified) variant may NOT be added alone.** It must be
   accompanied by a valid, verified **stock** `.rwd` for the same model, so there
   is always a factory image to flash back if a flash fails. No stock → no mod.
4. **Keep experimental work in a testing fork.** Anything still being tested or
   tuned (filter/rate variants, gain experiments, `*-test`, version-bumped
   tuning images, etc.) stays in a testing fork until it is proven stable on the
   car. Only stock images and stabilized linear-max variants belong here.
