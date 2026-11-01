# Live Maps source boundary

**2026-10-26: Maps as Vision's fill-in.** Selecting Maps in an SLC priority slot starts the mapd shadow provider (`shadow_lifecycle.enabled`) and plannerd's `MapSource` turns a `matchedLimit` sample on a `current` way into a VALID candidate. With priorities Vision then Maps, `map_observation.SignFirst` keeps a seen sign in force: Vision is VALID only for `SIGN_HOLD_NS` after a sign, so afterwards the map is held UNKNOWN (acceptance keeps the sign's limit) until the map's own limit differs from the one it reported at the sign, which marks a new posted-limit stretch. Before any sign the map fills in directly. A map limit still passes the 3 s automatic hold and the driver's lower/higher confirmation settings. On the comma BOOTTIME and MONOTONIC differed by 10 us (2026-10-26, no suspend). Static evidence only: unit and runtime-to-card command tests; not yet driven or replayed on a route with `mapdOut`.

`plannerd` subscribes to `mapdOut` when SLC transport is available.
`runtime.py` samples the Maps adapter when Maps is configured or a highest/lowest
selection policy considers it. The [controller contract](README.md) owns adoption,
confirmation and car-control behavior.

`map_source.MapTracker` validates v2 wire structure, source ordering and freshness.
It exposes diagnostics, not a valid SLC candidate. `map_observation.MapSource`
projects a fresh `matchedLimit` diagnostic into a valid observation only when its
way-selection type is `current`. Possible, predicted and extended ways, missing
speed metadata, GPS/coverage loss and `matchedNoLimit` remain unknown or stale;
they do not mean regulatory absence. The map provider rejects multiple plausible
loaded-tile road IDs before selecting a current way and applies direction and
one-way checks. Provider and offline-package admission remain with the Maps
lifecycle and storage owners.

Map output uses BOOTTIME nanoseconds on Linux. The runtime samples a paired
MONOTONIC/BOOTTIME clock and passes BOOTTIME to the tracker; the clocks must not
be interchanged after suspend. Apple uses monotonic as the host fallback.
GPS observation, computation and output timestamps stay separate. External GPS
fixes expire after 500 ms, internal fixes after 2 s and car fixes after 2.5 s;
computation and output expire after 250 ms. V1 records remain decodable but cannot
create a live candidate because their GPS clock domain was ambiguous.

Unknown versions, conflicting status/limit fields, stale evidence and reordering
cannot authorize a candidate. Source-generation changes clear held evidence.
A source change without a generation change, generation regression, clock
regression or excessive restarts fails closed until an explicit tracker reset;
late packets from retired producers cannot restore a limit.

The observation identity binds the current drive, vehicle, producer, source
generation, GPS source and OSM way ID. It is not a durable identity for restoring
a limit in a later drive. Freshness and a unique loaded-tile match do not establish
real-road accuracy or current map metadata. Synthetic host tests verify these
software gates; they do not qualify Maps on the vehicle.
