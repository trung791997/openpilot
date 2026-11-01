# Sentry motion events

This is the parked motion-event part of Sentry. Galaxy can list local event
metadata and captured images at `#/cameras/events`. Galaxy supports browser push,
Discord, webhooks, ntfy, and optional Firebase app push through the companion relay. Motion detection has
not yet been qualified on a physical device or vehicle.

The manager runs it only on device hardware when both
the vehicle is parked and a valid enabled `SentryMotionPreferences`
document is present. The same manager-owned
`sensord` supplies acceleration onroad and, under that explicit opt-in, while
parked. Sentry never starts another sensor, camera or power service.

The optional version-1 document contains `enabled`, `sensitivity` (0.005–1.0)
and `warningTimeSeconds` (0.1–10.0). Missing, unreadable or malformed settings
disable observation without replacing the saved bytes. There is no automatic
adoption of historical Sentry settings. Galaxy exposes the saved choices at
`#/cameras/sentry-settings`, through Cameras. Each edit requires a one-time
confirmation and fresh parked evidence, then compares the complete saved source
again before replacement. Invalid readable settings require an explicit reset
to Off and defaults; unreadable settings remain untouched. Saving preferences
does not bypass development opt-in. The shared settings page shows runtime
configuration blocks, a live arming countdown and motion-owner evidence failures.
Status expires after 1.5 seconds, is bound to the exact saved settings and boot,
and never grants sensor, camera or power authority. Heartbeats use namespace-local
shared memory on device hardware, not persistent event storage.

Arming requires 90 seconds of continuously fresh parked, ignition-off, power
and motion evidence. Observation runs at 10 Hz, preserving the magnitude-delta
threshold and warning-count behavior. An alarm requires more than 25 qualifying
deltas and at least 30 seconds within a 60-second motion window. A stale or
repeated sample, sensor loss, owner-loop gap, system resume, settings change,
ignition or shutdown request disarms the policy. Recovery requires rearming.
Hardwared retains battery accounting and shutdown authority; Sentry neither
extends parked runtime nor clears shutdown requests.

Warning and alarm records contain only kind, generated event/session IDs and
monotonic/system-clock timestamps. System time is not proof of an accurate wall
clock. Storage uses a private namespace-specific directory, at most 512 bounded
records, and atomic publication without overwriting an existing event. Full or
unavailable storage stops recording new events; it does not delete old evidence.
There are no image paths, location data or network requests. Authority and saved
settings are checked again immediately before publication. A publication whose
directory sync fails reports uncertain durability and is never blindly retried.

A shake log (`shake_log.py`) helps choose sensitivity and warning time from data,
for example to tell passing-car rumble from a door or a get-in. It is observation
only: it reads the sample and decision the motion loop already has and never
changes detection, arming, alarms, notifications, clips or timing; its errors are
swallowed and its file writes run on their own thread every 10 s without fsync.
For each 1 s window whose largest |magnitude delta| is above 0.02 (parked noise
is below that) it writes one JSON line: max, sum, the count above each of 0.02,
0.03, 0.04, 0.06, 0.08, 0.10, 0.12, 0.15, 0.20 and 0.30, the signed deltas, whether
the policy was armed, its trigger count and any event (whose `monoNs` matches the
stored event's `monoTimeNs`). A heartbeat line every 10 minutes shows Sentry was
running. Files are one per UTC day in `sentry/shake/` beside `sentry/events/`,
private like events, 20 MB in total with the oldest day deleted first. It is on
whenever Sentry runs; there is no separate setting.
`tools/sentry_shake_report.py` summarizes the log per hour and per labeled episode
(a `start,end,label` CSV) and replays candidate sensitivity/warning-time pairs
through `SentryPolicy` itself.

Tests cover continuous arming and thresholds, corrupt preferences, serialized
message timing, resume and power/ignition revocation, process selection, bounded
storage and interrupted writes. They do not establish physical motion sensitivity,
parked battery endurance, device wake behavior or complete Sentry parity.
