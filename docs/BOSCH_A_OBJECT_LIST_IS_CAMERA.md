# Bosch-A: the "radar" object list is the camera's

**Short version.** On Peter's Civic (Honda Bosch-A), the 16-slot object list that openpilot parses as
radar points (`0x280`–`0x2FF`, 80 frames at about 14.9 Hz) is sent by the **windshield camera**, not the
radar. The radar fuses it with its own detections internally, and those detections never reach CAN.
So radard's "radar leads", liveTracks and every "radar vs vision" check on this car are a camera
tracker checked against openpilot's vision.

**Naming rule.** When Peter, an agent or a code comment on this car says *radar*, *radar point*,
*radar lead* or *radar track*, read it as **the Bosch-A camera object list**. New writing should say
"object list" or "camera track". Existing names (`radar`, `radarState`, `HondaBoschARadar`,
`radar_interface.py`) stay as they are; renaming them is not worth the churn.

## Evidence

1. **VFN, 2026-10-04: "The Bosch-A radar object bank is the camera's output".** This is static firmware
   analysis of the camera (36161-TLA-A070, decrypted) and the radar (36802-TBA).
   - The camera's transmit PDU table lists all 80 object-list IDs, and its packers write every field.
   - The camera receives on exactly two buses: F-CAN and the private bus to the radar.
   - The radar sends only status PDUs to the camera.
   - Field semantics are vision quantities: bounding-box edges, angular width, 1/TTC, and the scale
     ratio range(t-1)/range(t).
   - At night the list keeps lit moving cars and loses unlit stationary objects.
   - Full report: `docs/research/pr17-bosch-radar-followup-2026-10-03.md` (Addenda 6-9) in VFN's tree.
2. **Wire check on Peter's logs (Bob, 2026-11-12; log evidence).** The panda sits on the private
   camera↔radar bus; bus 2 is the camera side and bus 0 is the radar side.

   | Data | Received on | Rate |
   |---|---|---|
   | All 80 object-list IDs `0x280`–`0x2FF` | bus 2 only (camera side), then forwarded to bus 0 | 14.9 Hz |
   | `0xE4` steering (a known camera output) | bus 2 | 100 Hz |
   | Everything the radar sends (bus 0) | `0x400` (50 Hz), `0x410` (1 Hz), `0x420` (10 Hz), `0x440` (rare); 4-8 distinct payloads each, status only | |
   | Stock's fused output | `0x1DF` ACC_CONTROL (50 Hz) and `0x39F` RADAR_HUD (10 Hz), on F-CAN | |

   This was identical on all 12 stock-ACC routes (`0000025d`…`00000299`, one segment each) and on the
   alpha-long route `000000c8`.

## What this means for work on this branch

- Decodes stay correct. U11 is 1/72 m/s, range is 1/16 m and azimuth is 1/2048 rad; these are the camera's own encodings.
- **The object list and openpilot's model can be wrong together**, for example in the dark, in glare or
  behind a large vehicle. A "radar agrees with vision" check is not independent confirmation.
- Range corrections should be validated against an independent reference, such as odometry against
  a stopped target or a tape measure, and not against openpilot vision.
- Stock ACC fuses real radar detections that we never see. The only radar-fused values on CAN are stock's
  `0x1DF` and `0x39F` (decode in progress, 2026-11-12).
- The safety rules are unchanged: never delete a point, and treat a rail as a bound. They are about not
  discarding evidence, whatever the sensor.
