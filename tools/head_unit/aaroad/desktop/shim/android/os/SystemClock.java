package android.os;

// A clock the harness can step: real time plus an offset it advances, so a render can "wait" 3 s without waiting.
public class SystemClock {
  private static volatile long offsetNs;
  public static void advance(long ms) { offsetNs += ms * 1000000L; }
  public static long uptimeMillis() { return (System.nanoTime() + offsetNs) / 1000000L; }
  public static long elapsedRealtime() { return uptimeMillis(); }
  public static long elapsedRealtimeNanos() { return System.nanoTime() + offsetNs; }
  public static void sleep(long ms) { try { Thread.sleep(ms); } catch (InterruptedException e) { Thread.currentThread().interrupt(); } }
}
