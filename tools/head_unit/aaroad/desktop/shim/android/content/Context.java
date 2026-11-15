package android.content;

import android.content.res.Resources;
import java.io.File;

public class Context {
  private final File dir;
  public Context(File dir) { this.dir = dir; }
  public Context getApplicationContext() { return this; }
  public File getFilesDir() { return new File(dir, "files"); }
  public File getCacheDir() { return new File(dir, "cache"); }
  public void sendBroadcast(Intent i) {}
  public Resources getResources() { return new Resources(); }
}
