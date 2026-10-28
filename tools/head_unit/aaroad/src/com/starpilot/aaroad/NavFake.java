package com.starpilot.aaroad;

import de.robv.android.xposed.XC_MethodHook;
import de.robv.android.xposed.XposedBridge;
import de.robv.android.xposed.XposedHelpers;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.Arrays;

// The TBT service marks who is guiding in bit 7 of byte 4 of the cluster's guidance
// display message (BCAN 0x12f95955): TBTInformation.setGuideDevice(0x80) on the
// setBCan* path Android Auto uses, setGuideDevice(0) on the ITBTInformation path the
// built-in navigation uses. The cluster answered every Android Auto guide point on
// 2026-10-28 with reply 2 (not stored), so it shows the arrow and distance only. Send
// Android Auto guidance marked as built-in navigation to see if the cluster then
// stores the guide point and shows its text.
//
// /data/local/tmp/aanav.txt, re-read every 2 s: "builtin" (default) or "phone" (stock).
// Cluster replies are logged when they change: GuidancePointInfoReply 1 means stored.
final class NavFake {
  static final String PKG = "com.mitsubishielectric.ada.appservice.navigation";
  private static final String TBT = "com.honda.displayaudio.system.meter.TBTInformation";
  private static final String CONFIG = "/data/local/tmp/aanav.txt";

  private static boolean builtin = true;
  private static long checkedAt, configTime = -1;
  private static byte[] lastReply;

  static void install(ClassLoader loader) {
    Class<?> tbt = XposedHelpers.findClass(TBT, loader);
    XposedHelpers.findAndHookMethod(tbt, "setGuideDevice", byte.class, new XC_MethodHook() {
      @Override
      protected void beforeHookedMethod(MethodHookParam param) {
        reload();
        if (builtin) param.args[0] = (byte) 0;
      }
    });
    XposedHelpers.findAndHookMethod(tbt, "onBcanGuideInfoReplyTbt",
        "com.mitsubishielectric.ada.framework.cpucomservice.BcanCommonData", new XC_MethodHook() {
      @Override
      protected void beforeHookedMethod(MethodHookParam param) {
        try {
          byte[] data = (byte[]) XposedHelpers.callMethod(param.args[0], "getBcanData");
          if (data == null || Arrays.equals(data, lastReply)) return;
          lastReply = data.clone();
          XposedBridge.log("aaroad: cluster guide reply " + hex(data) + (builtin ? " (sent as built-in)" : " (sent as phone)"));
        } catch (Throwable t) {
          XposedBridge.log("aaroad: reply log failed: " + t);
        }
      }
    });
    XposedBridge.log("aaroad: guidance device hook in " + PKG);
  }

  private static void reload() {
    long now = System.currentTimeMillis();
    if (now - checkedAt < 2000) return;
    checkedAt = now;
    File f = new File(CONFIG);
    long t = f.exists() ? f.lastModified() : 0;
    if (t == configTime) return;
    configTime = t;
    String text = "builtin";
    if (t != 0) {
      try {
        BufferedReader in = new BufferedReader(new FileReader(f));
        try {
          String line = in.readLine();
          if (line != null) text = line.trim();
        } finally {
          in.close();
        }
      } catch (Throwable ignored) {
      }
    }
    builtin = !text.equals("phone");
    XposedBridge.log("aaroad: guidance sent as " + (builtin ? "built-in" : "phone"));
  }

  private static String hex(byte[] data) {
    StringBuilder sb = new StringBuilder();
    for (byte b : data) sb.append(String.format("%02x", b & 0xff));
    return sb.toString();
  }

  private NavFake() {}
}
