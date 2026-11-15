package com.starpilot.aaroad;

// The small pure rules of the 2026-11-08 performance batch, with no Android in them so test/PerfTest.java can hold
// them on the desktop: the frame-pacing state machine, the zoomed-out kerb skip, the Douglas-Peucker band, and the
// heap-drop counter the log uses as a GC hint.
final class Perf {
  private Perf() {}

  // Adaptive frame pacing for the GL loop. A frame's cost (draw + swap, ms) is averaged over WINDOW_MS windows. Over
  // SLOW_ENTER_MS on average the cap drops from FAST_MS to SLOW_MS (about 24 fps); it returns once WINDOWS_BACK
  // windows in a row (3 s) average under FAST_BACK_MS. Late frames never run back to back: the next one is due at
  // start + cap, or MIN_GAP_MS after now if that has already passed, so the UI thread gets some CPU.
  static final class Pace {
    static final long FAST_MS = 33, SLOW_MS = 42, WINDOW_MS = 1000, MIN_GAP_MS = 4;
    static final double SLOW_ENTER_MS = 30, FAST_BACK_MS = 22;
    static final int WINDOWS_BACK = 3;

    private long frameMs = FAST_MS, winFrom, winCount;
    private double winSum;
    private int calm;
    int switches;  // times the cap changed, for the log
    double lastAvg;  // the last closed window's average cost, ms

    long frameMs() {
      return frameMs;
    }

    // A moving frame that cost costMs, finished at nowMs.
    void frame(long nowMs, double costMs) {
      if (winCount == 0) winFrom = nowMs;
      winSum += costMs;
      winCount++;
      if (nowMs - winFrom < WINDOW_MS) return;
      double avg = winSum / winCount;
      lastAvg = avg;
      winSum = 0;
      winCount = 0;
      if (frameMs == FAST_MS) {
        calm = 0;
        if (avg > SLOW_ENTER_MS) {
          frameMs = SLOW_MS;
          switches++;
        }
      } else if (avg < FAST_BACK_MS) {
        if (++calm >= WINDOWS_BACK) {
          frameMs = FAST_MS;
          calm = 0;
          switches++;
        }
      } else {
        calm = 0;
      }
    }

    // The time the next frame is due, after one that started at startMs and ended at nowMs.
    long next(long startMs, long nowMs) {
      return Math.max(startMs + frameMs, nowMs + MIN_GAP_MS);
    }

    // The loop went idle or the surface changed: the window starts over, the cap stays.
    void rest() {
      winSum = 0;
      winCount = 0;
      calm = 0;
    }
  }

  // The ground kerb passes are skipped zoomed out (the kerb is a 2 px edge there and doubles the draw calls). Below
  // KERB_IK they are off; they fade in to KERB_FULL_IK (LOD_IK / LOD_FADE_IK's style). The surface is widened by
  // the kerb's width times what is missing of the kerb's alpha, so the silhouette stays what it was.
  static final class KerbLod {
    static final float KERB_IK = 0.15f, KERB_FULL_IK = 0.18f;

    private KerbLod() {}

    static float alpha(float ik) {
      return Math.max(0, Math.min(1, (ik - KERB_IK) / (KERB_FULL_IK - KERB_IK)));
    }

    // extra width the surface pass takes on, display metres, for a kerb allowance of kerb
    static float surfaceExtra(float kerb, float alpha) {
      return kerb * (1 - alpha);
    }
  }

  // The Douglas-Peucker tolerance for the roads packed for the GPU, by zoom: 0 the existing 1 m (FreeView.SIMPLIFY,
  // already applied at build, so nothing to do), 1 two metres, 2 four. next() moves between bands with a margin
  // either side of the boundary so a zoom hovering on it does not repack the scene again and again.
  static final class DpBand {
    static final float[] EDGE_IK = {0.25f, 0.15f};  // boundary between band b and b + 1
    static final double[] TOL_M = {0, 2, 4};
    static final float HYST = 0.08f;
    static final long MIN_REPACK_MS = 3000;

    private DpBand() {}

    static int next(int cur, float ik) {
      int b = Math.max(0, Math.min(2, cur));
      while (b < 2 && ik < EDGE_IK[b] * (1 - HYST)) b++;
      while (b > 0 && ik > EDGE_IK[b - 1] * (1 + HYST)) b--;
      return b;
    }

    static double tolerance(int band) {
      return TOL_M[Math.max(0, Math.min(2, band))];
    }
  }

  // Counts drops of the used heap between samples as "GC happened": Dalvik's own counters (Debug.getGlobalGc...)
  // read zero unless allocation counting is on, which this does not turn on.
  static final class GcWatch {
    static final long DROP = 256 * 1024;
    private long last = -1;
    int drops;
    long worstDrop;

    void sample(long used) {
      if (last >= 0 && last - used >= DROP) {
        drops++;
        worstDrop = Math.max(worstDrop, last - used);
      }
      last = used;
    }

    void clear() {
      drops = 0;
      worstDrop = 0;
    }
  }
}
