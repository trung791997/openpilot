package android.os;

public class Looper {
  private static final Looper MAIN = new Looper(Thread.currentThread());
  private final Thread thread;
  private Looper(Thread t) { thread = t; }
  public static Looper getMainLooper() { return MAIN; }
  public Thread getThread() { return thread; }
}
