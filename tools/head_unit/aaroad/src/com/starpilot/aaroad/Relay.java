package com.starpilot.aaroad;

import android.net.LocalServerSocket;
import android.net.LocalSocket;
import de.robv.android.xposed.XposedBridge;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.InetSocketAddress;

// Carries the comma's map feed for the meter's process if that process may not use the network: Honda Hack
// holds the INTERNET permission, so this runs in its process and passes datagrams both ways over a local
// socket, each as [u16 length][datagram]. The UDP port is opened only while the meter is connected, so it
// never sits on the port beside the meter's own socket.
final class Relay {
  static final String NAME = "aaroad.feed";
  static final String PKG = "cn.autohack.hondahack";

  private static boolean started;

  static synchronized void install(String pkg, String process) {
    if (started || !PKG.equals(pkg) || !PKG.equals(process)) return;
    started = true;
    Thread t = new Thread(new Runnable() {
      @Override
      public void run() {
        LocalServerSocket server;
        try {
          server = new LocalServerSocket(NAME);
        } catch (Throwable e) {
          XposedBridge.log("aaroad: map relay could not listen: " + e);
          return;
        }
        while (true) {
          try {
            serve(server.accept());
          } catch (Throwable e) {
            XposedBridge.log("aaroad: map relay dropped the meter: " + e);
          }
        }
      }
    }, "aaroad-relay");
    t.setDaemon(true);
    t.start();
  }

  private static void serve(final LocalSocket client) throws Exception {
    final DatagramSocket udp = new DatagramSocket(null);
    final InetAddress[] comma = {null};
    try {
      udp.setReuseAddress(true);
      udp.setBroadcast(true);
      udp.bind(new InetSocketAddress(Feed.PORT));
      final DataOutputStream out = new DataOutputStream(client.getOutputStream());
      XposedBridge.log("aaroad: map relay carrying the feed for the meter");
      Thread down = new Thread(new Runnable() {
        @Override
        public void run() {
          byte[] buf = new byte[65536];
          try {
            while (true) {
              DatagramPacket p = new DatagramPacket(buf, buf.length);
              udp.receive(p);
              if (p.getLength() <= 0) continue;
              if (Feed.isBeacon(buf, p.getLength())) comma[0] = p.getAddress();
              out.writeShort(p.getLength());
              out.write(buf, 0, p.getLength());
              out.flush();
            }
          } catch (Throwable e) {
            try {
              client.close();
            } catch (Throwable ignored) {
            }
          }
        }
      }, "aaroad-relay-down");
      down.setDaemon(true);
      down.start();
      DataInputStream in = new DataInputStream(client.getInputStream());
      byte[] buf = new byte[65536];
      while (true) {
        int n = in.readUnsignedShort();
        in.readFully(buf, 0, n);
        InetAddress to = comma[0];
        if (to != null) udp.send(new DatagramPacket(buf, n, to, Feed.PORT));
      }
    } finally {
      udp.close();
      client.close();
    }
  }

  private Relay() {}
}
