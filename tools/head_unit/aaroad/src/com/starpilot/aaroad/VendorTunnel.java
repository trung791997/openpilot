package com.starpilot.aaroad;

import android.net.LocalServerSocket;
import android.net.LocalSocket;
import de.robv.android.xposed.XC_MethodHook;
import de.robv.android.xposed.XposedHelpers;
import java.io.DataInputStream;
import java.io.DataOutputStream;

// Carries the comma's cluster map feed (hu_feed.py) inside Android Auto's raw VendorExtensionChannel
// (channel 13, service 12) so the comma does not have to send 20 Hz UDP over wlan0 while aa0 streams
// video on the same non-DBS Wi-Fi chip.
//
// Framing on the AA channel: 2-byte prefix {'S', 'P'} (0x5350) followed by the JSON datagram bytes.
// Any frame without that prefix is left untouched for stock libHUVE.so.
final class VendorTunnel {
  static final String SOCKET_NAME = "aaroad.aa_feed";
  private static final String PKG = "com.mitsubishielectric.ada.app.androidauto";
  private static final String CHANNEL_CLASS = PKG + ".VendorExtensionChannel";
  private static final byte MAGIC_0 = 0x53;  // 'S'
  private static final byte MAGIC_1 = 0x50;  // 'P'
  private static final byte[] READY_MSG = "{\"v\":1,\"t\":\"ready\"}".getBytes();

  private static boolean started;
  private static volatile Object activeChannel;
  private static final Object CLIENT_LOCK = new Object();
  private static LocalSocket currentClient;
  private static DataOutputStream clientOut;

  static synchronized void install(String pkg, ClassLoader cl) {
    if (started || !PKG.equals(pkg)) return;
    Class<?> vec = XposedHelpers.findClass(CHANNEL_CLASS, cl);
    started = true;

    XposedHelpers.findAndHookMethod(vec, "onChannelOpened", new XC_MethodHook() {
      @Override
      protected void afterHookedMethod(MethodHookParam param) {
        activeChannel = param.thisObject;
        Xlog.log("aaroad: aa vendor tunnel opened");
        sendToComma(READY_MSG, 0, READY_MSG.length);
      }
    });

    XposedHelpers.findAndHookMethod(vec, "onChannelClosed", new XC_MethodHook() {
      @Override
      protected void afterHookedMethod(MethodHookParam param) {
        if (activeChannel == param.thisObject) activeChannel = null;
        Xlog.log("aaroad: aa vendor tunnel closed");
      }
    });

    XposedHelpers.findAndHookMethod(vec, "onMessage", byte[].class, new XC_MethodHook() {
      @Override
      protected void beforeHookedMethod(MethodHookParam param) {
        byte[] data = (byte[]) param.args[0];
        if (data == null || data.length < 2 || data[0] != MAGIC_0 || data[1] != MAGIC_1) return;
        int n = data.length - 2;
        if (n > 0 && n <= 65535) forwardToMeter(data, 2, n);
        param.setResult(data.length);
      }
    });

    Thread t = new Thread(new Runnable() {
      @Override
      public void run() {
        LocalServerSocket server;
        try {
          server = new LocalServerSocket(SOCKET_NAME);
        } catch (Throwable e) {
          Xlog.log("aaroad: aa vendor tunnel could not listen: " + e);
          return;
        }
        while (true) {
          try {
            serve(server.accept());
          } catch (Throwable e) {
            Xlog.log("aaroad: aa vendor tunnel dropped meter: " + e);
          }
        }
      }
    }, "aaroad-aa-tunnel");
    t.setDaemon(true);
    t.start();
    Xlog.log("aaroad: hooked " + CHANNEL_CLASS);
  }

  private static void forwardToMeter(byte[] data, int off, int len) {
    synchronized (CLIENT_LOCK) {
      if (clientOut == null) return;
      try {
        clientOut.writeShort(len);
        clientOut.write(data, off, len);
        clientOut.flush();
      } catch (Throwable e) {
        closeClientLocked();
      }
    }
  }

  private static void sendToComma(byte[] data, int off, int len) {
    Object ch = activeChannel;
    if (ch == null || len <= 0) return;
    byte[] frame = new byte[2 + len];
    frame[0] = MAGIC_0;
    frame[1] = MAGIC_1;
    System.arraycopy(data, off, frame, 2, len);
    try {
      XposedHelpers.callMethod(ch, "sendData", frame);
    } catch (Throwable e) {
      Xlog.log("aaroad: aa vendor tunnel send failed: " + e);
    }
  }

  private static void closeClientLocked() {
    clientOut = null;
    if (currentClient != null) {
      try {
        currentClient.close();
      } catch (Throwable ignored) {
      }
      currentClient = null;
    }
  }

  private static void serve(LocalSocket client) throws Exception {
    synchronized (CLIENT_LOCK) {
      closeClientLocked();
      currentClient = client;
      clientOut = new DataOutputStream(client.getOutputStream());
    }
    Xlog.log("aaroad: aa vendor tunnel carrying feed for meter");
    if (activeChannel != null) sendToComma(READY_MSG, 0, READY_MSG.length);
    try {
      DataInputStream in = new DataInputStream(client.getInputStream());
      byte[] buf = new byte[65536];
      while (true) {
        int n = in.readUnsignedShort();
        in.readFully(buf, 0, n);
        sendToComma(buf, 0, n);
      }
    } finally {
      synchronized (CLIENT_LOCK) {
        if (currentClient == client) closeClientLocked();
      }
    }
  }

  private VendorTunnel() {}
}
