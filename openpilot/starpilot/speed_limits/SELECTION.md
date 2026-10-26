# Source selection contract

`selection.select_limit` chooses an observation for the acceptance reducer. It
does not accept a speed, command a car, request an online provider, read settings
or judge sensor health. There is no production adapter yet.

Each of dashboard, map, vision and online must supply an explicit `Observation`.
Source adapters own normalized m/s, typed observation identity, freshness and eligibility.
For example, vision support and its separate display/control filters must be
resolved before this boundary. A missing producer is unknown; it is not proof
that no limit was observed. The selector invents no expiry period or road identity.
Geographic, producer-episode and session-value identities pass through unchanged.
Selection checks their structure but owns no drive session; the acceptance reducer
must verify session-value applicability against its current session before use.

`SelectionPolicy` has an explicit mode, exactly two primary source slots and an
online-fallback Boolean. A slot may be empty or repeat another slot. Online is
never a primary slot. Saved UI labels need a separately reviewed settings adapter;
this module does not activate or translate preferences.

| Mode | Primary candidates | Choice |
| --- | --- | --- |
| Ordered | Only the configured slots | First eligible valid observation |
| Highest | Dashboard and map; vision only if present in either slot | Highest eligible value |
| Lowest | Dashboard and map; vision only if present in either slot | Lowest eligible value |

Equal extrema choose dashboard, then map, then vision, independent of input map
iteration order. The minimum eligible limit is explicitly 1 m/s, inclusive.
Smaller positive observations are reported in `below_minimum` and excluded;
zero, negative, nonfinite or malformed valid candidates are input errors.

The online observation is considered only if no configured primary is eligible
and fallback is enabled. It cannot override a valid primary. An eligible
secondary or online observation can replace an unavailable primary; all
considered unavailable sources remain in the result. Disabled and unconfigured
sources cannot create uncertainty in that choice.

If no eligible alternative exists, the result retains unknown or stale state.
Unknown takes precedence over stale, which takes precedence over absence; the
full per-source statuses remain available. Explicit absence, exclusion below the
minimum or an empty configured source set produces an absent result. Acceptance
independently applies the saved previous-limit policy without changing those
source statuses. Malformed input produces `invalid_input`, errors and
an unknown observation without selecting a source.

The tests include composition through the acceptance reducer: accepted 65 mph,
proposed 45, driver rejection, then source loss and reconnect. Last accepted
history remains 65 for both longitudinal-only and combined authority. Unknown
or stale sources retain that limit only when fallback is enabled; lateral-only or stock ownership
cannot turn a selected speed into a system longitudinal target. These are
deterministic policy tests, not native producer, planner, CAN or vehicle evidence.

## Online source: the Mapbox route limit (2026-10-26)

The online source is the posted limit Mapbox Directions returns for the route
(`annotations=maxspeed`, one entry per route segment). Navigation publishes the
limit of the segment the car is matched to as `starpilotNavigation.speedLimit`
(m/s, 0 = unknown), and only while guiding and on the route; off the route it is
0. `navigation_observation.py` turns that into the online observation: absent when
not guiding, stale past 3 s, unknown at 0. `online_fallback` is now on, so the
route limit is used only when neither priority slot has an eligible limit, and
the sign shows "NAV". When Vision is in a slot, the route limit is held behind a
sign seen this drive until the route's limit changes from its value at the sign
(the same `SignFirst` rule as the map). Evidence: static tests only
(`tests/test_navigation_limit.py`); not yet replayed on a navigated drive.
