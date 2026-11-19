# Set Speed Assist: provenance and license

Set Speed Assist (SSA) is ported from zoompilot's `e2e_set_speed` package
(`openpilot/sunnypilot/selfdrive/controls/lib/e2e_set_speed/`), commits `e6baf0b50`
(Set Speed Assist, lift-only toward chill's answer) and `2e9506a87` (approach pace and plan fade).
The algorithm and every constant are unchanged. Copyright (c) 2026-, Zeph Leggett (zoompilot).

zoompilot's `LICENSE.md` is the sunnypilot Custom MIT License (Copyright (c) 2024, Haibin Wen, SUNNYPILOT LLC),
which requires permission for commercial, for-profit or closed-source use and requires these acknowledgments:

- This software is licensed under a custom license requiring permission for use.
- This project uses software from Haibin Wen and SUNNYPILOT LLC and is licensed under a custom license requiring permission for use.

Contact sunnypilot Support for permission requests.

## Differences from the zoompilot source

- `op_long_cruise_jerk` is a local copy of zoompilot's `smart_cruise_control/limits.py` table
  (`[0, 10, 25, 40] m/s -> [1.6, 1.2, 0.8, 0.6] m/s^3`), the same numbers as this tree's `J_CRUISE_VALS`.
- The `LongitudinalPlanZP` telemetry struct and its `fill()` are dropped (no cereal change).
- The enable flag is the param `SetSpeedAssist` (default off), re-read every second by the controller itself.
- `hook.py` is new: one helper both longitudinal planners call at the Accel Boost site.
