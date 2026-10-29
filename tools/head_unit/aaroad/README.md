# aaroad — Android Auto road name on the Civic cluster

Xposed module for the 10th-gen Civic head unit (Mitsubishi Electric ADA, Android 4.2.2,
Honda Hack root). Stock `NavigationFocusManager` keeps the Android Auto next-turn road in
`mRoad` but hardcodes `setGuidancePointName()` to `""` and `GuidanceRoadNameCode/Num` to 0,
so the meter gets arrow + distance only. The module fills them from `mRoad` (charset from
`mCharsetCode`, ≤31 chars, ≤36 bytes — the `CanFormatAnalyze.replyBcanNavGuide4` cap).
Any exception leaves the stock values.

Package name is `cn.autohack.coolui` on purpose. Honda Hack's own XposedBridge
(`/data/data/cn.autohack.hondahack/bin/XposedBridge.jar`, v4.8.17, read off the head unit
2026-10-28) ignores `modules.list` and only loads `assets/xposed_init` from
`/data/app/cn.autohack.hondahack-{1,2}.apk` and `/data/app/cn.autohack.coolui-{1,2}.apk`.
CoolUI is not installed on this head unit, so that slot is free. Modules load in zygote:
the head unit must reboot after install or uninstall. Remove with `pm uninstall cn.autohack.coolui`.

Street filter and one-time resend added 2026-10-28 after a live capture: every send got cluster
reply 2 (not stored, resend) and mRoad was instruction text ("Turn right"). Only street names are
sent now ("onto X" / "on X", or a bare name); the present-location name gets the same filter.

The resend was removed 2026-10-29: Android Auto repeats the turn every second, so it replayed
every second, and each replay carries no distance, which made the stock cluster distance bar
flicker between 0 and the real distance (Peter, on the car). It never got the turn stored.

Live result 2026-10-28 16:48–16:50 (parked, route started): real street names went out
("Westgate Way", "South State Highway 78 / TX 78"), each turn was resent once, and the cluster
still answered every send 2,15 (not stored) and showed no text — same answer as the stock
nameless sends. This cluster does not take guidance text in Android Auto mode; the module has
no visible effect.

Evidence before that: **static** only (smali of the pulled AndroidAuto and Navigation apps, and the
decompiled bridge). Not yet installed or road-tested.

Build (arm64 or x86 host): Android SDK `build-tools;34.0.0` (for `d8`) and
`platforms;android-17` under `/tmp/hu/sdk`, `api-82.jar` at `/tmp/hu/api-82.jar`,
plus `aapt`, `zipalign`, `apksigner` on PATH. `./build.sh` → `out/aaroad.apk`.
The debug keystore is generated locally and not committed.

## Cluster screen cast crop (`Cast.java`, 2026-10-28)

Honda Hack's screen cast sends a 400×240 copy of the whole screen and `meter_civic.xml` draws it
`fitXY` into a 584×191 area, so it is ~1.8× too wide. The module hooks `ImageView.setImageBitmap`,
and for the view named `screen_cast` swaps in a frame cut to the chosen screen regions at their own
aspect, centered on black. Regions are read from `/data/local/tmp/aacast.txt` (re-read every 2 s):
`card` (default), `map`, `both`, `full`, or `x,y,w,h;…` in 800×480 screen pixels. Change it live:
`adb shell "echo both > /data/local/tmp/aacast.txt"`. Evidence: **static** (decompiled Honda Hack,
offline preview from a head-unit screenshot); not yet installed.

## Send Android Auto guidance as built-in navigation (`NavFake.java`, 2026-10-28)

The TBT service (`com.mitsubishielectric.ada.appservice.navigation`) marks the guiding device
in bit 7 of byte 4 of the guidance display message (BCAN `0x12f95955`): Android Auto's
`setBCan*` path sets `0x80`, the built-in navigation's `ITBTInformation` path sets `0`
(`TBTInformation.setGuideDevice`). The hook forces `0` so the cluster sees built-in
navigation, and logs each new cluster guide reply (`aaroad: cluster guide reply …`).

    adb shell "echo phone > /data/local/tmp/aanav.txt"    # back to stock marking, live
    adb shell "echo builtin > /data/local/tmp/aanav.txt"  # default

Evidence: static (smali read). Not yet run on the head unit; whether the cluster then stores
guide points (reply 1) and shows the street name is the open question.

Search for a "navigation present" flag to the cluster (2026-10-28, static): none in the Java
code. The unit type (`mNaviFunction`, unit info 9: `NAVI_DA` vs `NAVI_DA_PLUS`) only picks
who sends the position frame `0x12f8c155` (the NMEA receiver on `NAVI_DA`, the nav app on
`NAVI_DA_PLUS`) and what the steering-wheel nav key does. The guidance CAN ID is picked by
model code, not navigation. The device bit above is the only nav-vs-phone marker the head
unit sends. Do not fake `NAVI_DA_PLUS`: with no nav app it would stop the position frame.
Native code and the CPU-com microcontroller were not searched.

## Honda Hack meter nav panel (`Panel.java`, 2026-10-28)

Honda Hack's custom Civic meter already draws on the cluster (compass, trip data) and has its own
nav panel (`C0268ua`, `meter_civic.xml`: turn_icon, seg_dist, next_road). It listens, with no
permission, for `cn.autohack.hondahack.START_NAVI` / `UPDATE_NAVI_WAZE` / `STOP_NAVI`. The Android
Auto process now sends them: icon from the next-turn event and side (`left`, `right`, `exit_left`,
`u_turn`, `roundabout_N`, `end`, ...), distance in ft (50 ft steps) under 0.1 mi then mi, street
from the same `street()` cut as the cluster name. This does not depend on the Honda cluster storing
the guide point.

`UPDATE_NAVI_WAZE` is dropped while Honda Hack's `enable_screen_cast` is on (`C0268ua.z`), so turn
the screen cast off in Honda Hack's meter settings to see the panel. Logcat: `aaroad: panel start`
/ `panel stop`.

2026-10-28, parked with a Google Maps route (limited road evidence): arrow and "Presidents Blvd"
showed, distance did not. Android Auto repeats `onNextTurn` about once a second and each repeat
sent no `distance`, which makes Honda Hack hide seg_dist. A repeat of the same turn now keeps the
last distance; only a new turn drops it until `onNextTurnDistance`.

`Meter.java` (externaldisplay processes) adds the continuous map (`MapView.java`) behind
`navi_layout` on the first nav text Honda Hack sets. The 2026-10-28 stock-page rearrangement never
ran (seg_dist and next_road sit in a LinearLayout) and is gone. Log: `aaroad: meter map added`.

## Cluster map feed from the comma (`hu_feed.py`, 2026-10-29)

The cluster map (heading-up route, 3D close-up near the turn, carpet in openpilot's mode colour) needs
more than Android Auto's nav channel carries: openpilot's mode, the whole route and the car's position.
The comma sends them from `openpilot/starpilot/system/android_auto/hu_feed.py`, started by the Android
Auto daemon. The comma's `aa0` reaches the head unit only through the AAWireless dongle's USB side, so
there is no IP route to the head unit there; the feed uses the phone hotspot both are on, UDP port
47312. It does nothing when the head unit is on another network.

One JSON object per datagram, `{"v": 1, "t": kind, ...}`, each under 1472 bytes:

| kind | way, rate | body |
|---|---|---|
| `b` beacon | comma broadcast, 1 Hz, every up interface but `lo` and `aa*` | `port` only, no drive data |
| `h` hello | head unit to comma, about 1 Hz | `have`: road tiles held, `["x/y", ...]`; leave it out to get no tiles |
| `s` state | 1 Hz and on change | `mode` (`exp` `chill` `aol` `off`), `metric`, `nav` (null unless a route is guiding) |
| `r` route | on change and every 5 s | `id`, `i`, `n`, `pts` `[[lat, lon], ...]`, 50 per datagram, about 512 in all |
| `p` pose | 20 Hz | `lat`, `lon`, `hdg` (deg, compass), `spd` (m/s), `age` (s): the last GPS fix carried forward, at most 1 s |
| `m` road tile | one tile at a time, 3x3 around the car, nearest first | `k` `"x/y"`, `i`, `n`, `b` base64 of 1000-byte pieces of the zoom-14 file (`navigation/road_tiles.py`) |

`nav`: `type`, `modifier`, `distance` (m), `exit`, `degrees`, `left`, `text`, `then`,
`remaining` (m), `duration` (s), `limit` (m/s). Mode: `exp`/`chill` while openpilot drives both axes,
`aol` while it steers alone, `off` otherwise.

The comma answers only hellos from private addresses, one head unit at a time, and stops 5 s after the
hellos stop; with no head unit asking it sends only the beacon and reads nothing. Any device on the
hotspot can ask for the feed. A tile the hellos still lack 3 s after it went out is sent again. Road
tiles exist only while navigating and carry road class, not lane counts, so the head unit uses
per-class lane defaults. The route is the planner's route cut to 512 points, so turns on a long route
are coarse until the head unit snaps them to the tiles.

Head unit side (`Feed.java`, `Relay.java`, `MapView.java`, 2026-10-29):
- `Feed` answers the beacon with a hello (only while the nav page shows) and keeps the pose, state,
  route and up to 25 tiles. If the meter's process may not open the UDP port, it asks `Relay`, which
  runs in Honda Hack's process (it has INTERNET) and passes datagrams over the local socket
  `aaroad.feed`. Logs: `map feed listening [through Honda Hack]`, `map relay carrying the feed`.
- `MapView` shows only while the nav page is up and the pose is under 2 s old; otherwise Honda Hack's
  page is left as it was. Layout B of the mockup: heading-up tilted map (car at 369,180), route road
  with the carpet in the mode colour, faint cross-street stubs, Peter's Civic (`assets/civic_map.png`,
  read from the module apk), Honda Hack's arrow moved to the top left, distance + 500 ft bar,
  street, speed limit, arrival. Zoom about 20 s ahead (200 to 600 m), held on the turn from 500 ft.
  Logs: `meter map showing` / `hidden`, `map draw N ms average` (it draws less often past 33 ms).
- Static only: a desktop run of the real `MapView` against stub Android drawing on a synthetic street
  grid. Not yet run on the head unit.
Module log: `/data/data/cn.autohack.hondahack/log/error.log` (su).
Comma side: static tests only (`tests/test_hu_feed.py`).
