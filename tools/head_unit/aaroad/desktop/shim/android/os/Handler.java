package android.os;

// Nothing runs from a Handler here: the harness steps the app's tick itself, so posts are dropped.
public class Handler {
  public Handler(Looper l) {}
  public boolean post(Runnable r) { return true; }
  public boolean postDelayed(Runnable r, long ms) { return true; }
  public void removeCallbacks(Runnable r) {}
}
