package com.starpilot.aaroad;

import java.io.File;
import java.io.FileOutputStream;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.BlockingQueue;

// The map's show/hide events and its frame-timing lines, kept in a small rolling file on the head unit (Peter
// 2026-11-01: the cluster map sometimes drops to the Honda Hack view and comes back, and the head unit's logcat
// ring holds only about 18 minutes, so the moment is gone by the time anyone looks).
//
// The file is /sdcard/aaroad/events.log, else aaroad/events.log under the meter process's files dir, else under its
// cache dir, the first that takes a line (2026-11-01: on the head unit no events.log turned up anywhere, and the
// writer kept quiet about why). Which one it went to, or every directory's reason when none did, is status(): MapView
// puts it in logcat and Honda Hack's error.log once, and again only if it changes. At MAX_BYTES the file moves to
// events.log.1, replacing the older copy, so the two hold about 512 KB. Lines carry the wall-clock time (stamp(),
// which Xlog also puts on the lines that go to error.log), are queued and written on their own thread, so the UI and
// GL threads never wait on the disk; if the queue is full (the disk stuck), a line is dropped rather than block.
// Plain Java (test/EventsTest.java).
final class Events {
  static final long MAX_BYTES = 256 * 1024;
  static final String NAME = "events.log";
  static final String DIR = "aaroad";
  private static final BlockingQueue<String> queue = new ArrayBlockingQueue<String>(512);
  private static final SimpleDateFormat TIME = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss.SSS", Locale.US);
  static volatile File[] dirs = {new File("/sdcard/" + DIR)};  // tried in order; MapView adds the app's own dirs
  private static volatile File file;                           // the one in use, once found
  private static volatile String status;
  private static Thread writer;

  // The directories to try: /sdcard/aaroad, then aaroad under each parent given (files dir, cache dir), in order.
  // A null parent (getFilesDir() fails that way) or a repeat is left out.
  static void use(File... parents) {
    List<File> out = new ArrayList<File>();
    out.add(new File("/sdcard/" + DIR));
    for (File p : parents) {
      if (p == null) continue;
      File d = new File(p, DIR);
      if (!out.contains(d)) out.add(d);
    }
    dirs = out.toArray(new File[0]);
  }

  // The line with the wall-clock time in front, as events.log and error.log carry it.
  static String stamp(String line) {
    synchronized (TIME) {
      return TIME.format(new Date()) + " " + line;
    }
  }

  // Queues one line with the time; never blocks.
  static void record(String line) {
    queue.offer(stamp(line));
    start();
  }

  // The file in use, or null before the first line went in (or none could be written).
  static File where() {
    return file;
  }

  // "aaroad: map events go to <file>", or "aaroad: events.log not written: <dir>: <why>; ..." once every directory
  // failed; null before the first line was tried.
  static String status() {
    return status;
  }

  private static synchronized void start() {
    if (writer != null) return;
    writer = new Thread(new Runnable() {
      @Override
      public void run() {
        while (true) {
          try {
            write(queue.take());
          } catch (Throwable t) {
            status = "aaroad: events.log not written: " + t;  // the line is lost, the next one tries again
          }
        }
      }
    }, "aaroad-events");
    writer.setDaemon(true);
    writer.start();
  }

  // Writes one line to the file in use, or finds the first directory that takes it.
  static void write(String line) {
    File f = file;
    String failed = f == null ? null : appendWhy(f, line, MAX_BYTES);
    if (f != null && failed == null) return;
    StringBuilder why = new StringBuilder();
    if (f != null) why.append(f).append(": ").append(failed);
    file = null;
    for (File d : dirs) {
      File c = new File(d, NAME);
      String no;
      if (d.isDirectory() || d.mkdirs()) {
        no = appendWhy(c, line, MAX_BYTES);
      } else {
        no = d.exists() ? "not a directory" : "mkdirs failed" + (d.getParentFile() != null && !d.getParentFile().canWrite()
            ? " (" + d.getParentFile() + " not writable)" : "");
      }
      if (no == null) {
        file = c;
        status = "aaroad: map events go to " + c;
        return;
      }
      if (why.length() > 0) why.append("; ");
      why.append(d).append(": ").append(no);
    }
    status = "aaroad: events.log not written: " + why;
  }

  // Appends line and a newline to f, first moving f to f.1 if the line would take it past max. False on failure.
  static boolean append(File f, String line, long max) {
    return appendWhy(f, line, max) == null;
  }

  // append(), with why it failed, null when it did not.
  static String appendWhy(File f, String line, long max) {
    try {
      byte[] bytes = (line + "\n").getBytes("UTF-8");
      if (f.exists() && f.length() + bytes.length > max) {
        File old = new File(f.getPath() + ".1");
        if (old.exists() && !old.delete()) return "could not delete " + old;
        if (!f.renameTo(old)) return "could not rename to " + old;
      }
      FileOutputStream out = new FileOutputStream(f, true);
      try {
        out.write(bytes);
      } finally {
        out.close();
      }
      return null;
    } catch (Throwable t) {
      return String.valueOf(t);
    }
  }

  private Events() {}
}
