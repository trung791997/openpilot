package android.content.res;

import java.io.ByteArrayInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.zip.ZipEntry;
import java.util.zip.ZipFile;

// What MapView.inter() does through reflection: newInstance(), addAssetPath(zip), then open("fonts/...") for Typeface.
public class AssetManager {
  private ZipFile zip;
  public int addAssetPath(String path) {
    try {
      zip = new ZipFile(path);
      return 1;
    } catch (IOException e) {
      return 0;
    }
  }
  public InputStream open(String name) throws IOException {
    if (zip == null) throw new IOException("no asset path");
    ZipEntry e = zip.getEntry("assets/" + name);
    if (e == null) throw new IOException("no asset " + name);
    try (InputStream in = zip.getInputStream(e)) {
      return new ByteArrayInputStream(in.readAllBytes());
    }
  }
}
