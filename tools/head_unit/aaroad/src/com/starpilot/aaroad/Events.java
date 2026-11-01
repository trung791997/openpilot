package com.starpilot.aaroad;

import java.io.File;
import java.io.FileOutputStream;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.BlockingQueue;

// The map's show/hide events and its frame-timing lines, kept in a small rolling file on the head unit (Peter
// 2026-11-01: the cluster map sometimes drops to the Honda Hack view and comes back, and the head unit's logcat
// ring holds only about 18 minutes, so the moment is gone by the time anyone looks).
//
// The file is /sdcard/aaroad/events.log, or <meter process files dir>/aaroad/events.log when the meter's process
// may not write the SD card. At MAX_BYTES it moves to events.log.1, replacing the older copy, so the two hold
// about 512 KB. Lines are queued and written on their own thread, so the UI and GL threads never wait on the disk;
// if the queue is full (the disk stuck), a line is dropped rather than block. Plain Java (test/EventsTest.java).
final class Events {
  static final long MAX_BYTES = 256 * 1024;
  static final String NAME = "events.log";
  private static final BlockingQueue<String> queue = new ArrayBlockingQueue<String>(512);
  private static final SimpleDateFormat TIME = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss.SSS", Locale.US);
  static volatile File[] dirs = {new File("/sdcard/aaroad")};  // tried in order; MapView adds its files dir
  private static volatile File file;                         // the one in use, once found
  private static Thread writer;

  // Queues one line with the time; never blocks.
  static void record(String line) {
    String stamped;
    synchronized (TIME) {
      stamped = TIME.format(new Date()) + " " + line;
    }
    queue.offer(stamped);
    start();
  }

  // The file in use, or null before the first line went in (or none could be written).
  static File where() {
    return file;
  }

  private static synchronized void start() {
    if (writer != null) return;
    writer = new Thread(new Runnable() {
      @Override
      public void run() {
        while (true) {
          try {
            String line = queue.take();
            File f = file;
            if (f == null || !append(f, line, MAX_BYTES)) {
              file = null;
              for (File d : dirs) {
                File c = new File(d, NAME);
                if ((d.isDirectory() || d.mkdirs()) && append(c, line, MAX_BYTES)) {
                  file = c;
                  break;
                }
              }
            }
          } catch (Throwable ignored) {
            // nowhere to say it; the line is lost, the next one tries again
          }
        }
      }
    }, "aaroad-events");
    writer.setDaemon(true);
    writer.start();
  }

  // Appends line and a newline to f, first moving f to f.1 if the line would take it past max. False on failure.
  static boolean append(File f, String line, long max) {
    try {
      byte[] bytes = (line + "\n").getBytes("UTF-8");
      if (f.exists() && f.length() + bytes.length > max) {
        File old = new File(f.getPath() + ".1");
        if (old.exists() && !old.delete()) return false;
        if (!f.renameTo(old)) return false;
      }
      FileOutputStream out = new FileOutputStream(f, true);
      try {
        out.write(bytes);
      } finally {
        out.close();
      }
      return true;
    } catch (Throwable t) {
      return false;
    }
  }

  private Events() {}
}
