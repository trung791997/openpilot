package com.starpilot.aaroad;

import java.util.concurrent.ArrayBlockingQueue;

// Log lines from threads that must not touch a file (the GL thread). MapView.say writes to Honda Hack's error.log
// and the rolling event file; here the lines go to one background thread that does it, in order. The queue is
// bounded: if the logger itself is stuck, lines are dropped (counted, said once it recovers) rather than blocking the
// caller. The thread starts on the first line, not at class load (Hook.initZygote loads this package in the zygote).
final class AsyncLog {
  private static final ArrayBlockingQueue<Runnable> Q = new ArrayBlockingQueue<Runnable>(256);
  private static Thread thread;
  private static volatile int dropped;

  static void say(final String line) {
    post(new Runnable() {
      @Override
      public void run() {
        MapView.say(line);
      }
    });
  }

  // Work for the logger thread, which may block (a stack of another thread, a file write).
  static void post(Runnable r) {
    start();
    if (!Q.offer(r)) dropped++;
  }

  private static synchronized void start() {
    if (thread != null) return;
    thread = new Thread(new Runnable() {
      @Override
      public void run() {
        for (;;) {
          try {
            Runnable r = Q.take();
            int d = dropped;
            if (d > 0) {
              dropped = 0;
              MapView.say("aaroad: async log dropped " + d + " lines");
            }
            r.run();
          } catch (InterruptedException e) {
            return;
          } catch (Throwable t) {
            // a log line must never kill the logger
          }
        }
      }
    }, "aaroad-log");
    thread.setDaemon(true);
    thread.start();
  }

  private AsyncLog() {}
}
