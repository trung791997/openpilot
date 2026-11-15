package com.starpilot.aaroad;

// The turn-signal flash of the dash map's car (2026-11-14): lit for the first half of each PERIOD_MS from the moment the
// side's flag came on, so the first flash starts lit. 666 ms = 1.5 Hz = 90 flashes a minute, 50% duty; Honda's
// signal runs 85-90/min, inside FMVSS 108's 60-120. Pure, so the desktop test (test/BlinkTest) can check the phase.
final class Blink {
  static final long PERIOD_MS = 666;

  // Whether the lamp is lit at now (ms) when the signal came on at onSince (ms, same clock). A clock that reads
  // before onSince counts as the start: lit.
  static boolean lit(long onSince, long now) {
    long d = now - onSince;
    return d < 0 || d % PERIOD_MS < PERIOD_MS / 2;
  }

  private Blink() {}
}
