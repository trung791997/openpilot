package com.starpilot.aaroad;

import de.robv.android.xposed.IXposedHookLoadPackage;
import de.robv.android.xposed.IXposedHookZygoteInit;
import de.robv.android.xposed.XC_MethodHook;
import de.robv.android.xposed.XposedBridge;
import de.robv.android.xposed.XposedHelpers;
import de.robv.android.xposed.callbacks.XC_LoadPackage.LoadPackageParam;

// The stock Android Auto app stores the next-turn road in mRoad but hardcodes the
// cluster's guidance point name to "" and its road-name code/length to 0, so the
// meter shows the arrow and distance only. Fill them from mRoad. Any failure leaves
// the stock values in place.
//
// mRoad is often Google's instruction text ("Turn right", "Your destination is on the
// right"), which stock also sends as the present-location name. Only a street name is
// sent: "onto X" / "on X" is cut out, other instruction text becomes no name.
// The cluster answers every send with reply 2 (not stored). A replay of the turn on reply 2
// (2026-10-28) never got it stored, and since Android Auto repeats the turn every second it
// replayed every second; each replay carries no distance, so the stock distance bar flickered
// to 0 (Peter, 2026-10-29). Removed.
public class Hook implements IXposedHookLoadPackage, IXposedHookZygoteInit {
  private static final String PKG = "com.mitsubishielectric.ada.app.androidauto";
  private static final String NFM = PKG + ".navigation.NavigationFocusManager";
  private static final int MAX_CHARS = 31;  // 5-bit length field, same cap as present location
  private static final int MAX_BYTES = 36;  // CanFormatAnalyze.replyBcanNavGuide4 truncates here

  @Override
  public void initZygote(StartupParam startupParam) {
    MapView.modulePath = startupParam.modulePath;  // the map's car icon is read from the module apk
  }

  @Override
  public void handleLoadPackage(LoadPackageParam lpparam) throws Throwable {
    try {
      Relay.install(lpparam.packageName, lpparam.processName);
    } catch (Throwable t) {
      XposedBridge.log("aaroad: map relay failed: " + t);
    }
    try {
      Cast.install(lpparam.packageName);
    } catch (Throwable t) {
      XposedBridge.log("aaroad: cast hook failed: " + t);
    }
    try {
      Meter.install(lpparam.processName);
    } catch (Throwable t) {
      XposedBridge.log("aaroad: meter hook failed: " + t);
    }
    if (NavFake.PKG.equals(lpparam.packageName)) {
      try {
        NavFake.install(lpparam.classLoader);
      } catch (Throwable t) {
        XposedBridge.log("aaroad: guidance device hook failed: " + t);
      }
      return;
    }
    if (!PKG.equals(lpparam.packageName)) return;
    Class<?> nfm = XposedHelpers.findClass(NFM, lpparam.classLoader);

    XposedHelpers.findAndHookMethod(nfm, "setGuidancePointName", new XC_MethodHook() {
      @Override
      protected void afterHookedMethod(MethodHookParam param) {
        try {
          String road = road(param.thisObject);
          if (road != null) param.setResult(encode(road, charset(param.thisObject)));
        } catch (Throwable t) {
          XposedBridge.log(t);
        }
      }
    });

    XposedHelpers.findAndHookMethod(nfm, "setNavGuide",
        "com.mitsubishielectric.ada.appservice.navigation.NavGuide", new XC_MethodHook() {
      @Override
      protected void afterHookedMethod(MethodHookParam param) {
        try {
          String road = road(param.thisObject);
          if (road == null) return;
          String cs = charset(param.thisObject);
          int code = "UTF-16BE".equals(cs) ? 1 : "GB18030".equals(cs) ? 2 : 0;
          Object guide = param.args[0];
          XposedHelpers.callMethod(guide, "setGuidanceRoadNameCode", code);
          XposedHelpers.callMethod(guide, "setGuidanceRoadNameNum", fit(road, cs).length());
        } catch (Throwable t) {
          XposedBridge.log(t);
        }
      }
    });
    XC_MethodHook streetOnly = new XC_MethodHook() {
      @Override
      protected void beforeHookedMethod(MethodHookParam param) {
        try {
          String raw = (String) XposedHelpers.getObjectField(param.thisObject, "mRoad");
          param.setObjectExtra("raw", raw);
          XposedHelpers.setObjectField(param.thisObject, "mRoad", road(param.thisObject));
          param.setObjectExtra("swapped", Boolean.TRUE);
        } catch (Throwable t) {
          XposedBridge.log(t);
        }
      }

      @Override
      protected void afterHookedMethod(MethodHookParam param) {
        if (param.getObjectExtra("swapped") == null) return;
        try {
          XposedHelpers.setObjectField(param.thisObject, "mRoad", param.getObjectExtra("raw"));
        } catch (Throwable t) {
          XposedBridge.log(t);
        }
      }
    };
    for (String m : new String[] {"setPresentLocationName", "setPresentLocationNameCharacterNum",
                                  "setPresentLocationNameState"}) {
      XposedHelpers.findAndHookMethod(nfm, m, streetOnly);
    }

    XposedHelpers.findAndHookMethod(nfm, "onNextTurn", String.class, int.class, int.class, byte[].class,
        int.class, int.class, new XC_MethodHook() {
      @Override
      protected void beforeHookedMethod(MethodHookParam param) {
        XposedBridge.log("aaroad: street \"" + street((String) param.args[0]) + "\"");
      }
    });

    try {
      Panel.install(nfm);
    } catch (Throwable t) {
      XposedBridge.log("aaroad: panel hook failed: " + t);
    }
    XposedBridge.log("aaroad: hooked " + NFM);
  }

  private static final String[] INSTRUCTION = {
    "turn ", "continue", "keep ", "take ", "merge", "head ", "make ", "slight ", "sharp ", "exit",
    "enter ", "use ", "your destination", "destination", "at the ", "arrive", "go ", "stay ",
    "follow ", "u-turn", "drive ", "proceed", "bear ", "then ", "in ", "you have arrived"};

  private static String road(Object nfm) {
    return street((String) XposedHelpers.getObjectField(nfm, "mRoad"));
  }

  // Street name in an Android Auto road/instruction string, or null if it has none.
  static String street(String road) {
    if (road == null) return null;
    String s = road.trim();
    while (s.endsWith(".")) s = s.substring(0, s.length() - 1).trim();
    String lower = s.toLowerCase(java.util.Locale.US);
    int i = lower.lastIndexOf(" onto ");
    if (i >= 0) {
      s = s.substring(i + 6);
    } else {
      boolean instruction = false;
      for (String p : INSTRUCTION) instruction |= lower.startsWith(p);
      if (instruction) {
        i = lower.indexOf(" on ");
        if (i < 0 || lower.contains("destination")) return null;
        s = s.substring(i + 4);
      }
    }
    String l = s.toLowerCase(java.util.Locale.US);
    if (l.startsWith("toward ")) {
      s = s.substring(7);
      l = l.substring(7);
    }
    i = l.indexOf(" toward ");
    if (i >= 0) s = s.substring(0, i);
    l = s.toLowerCase(java.util.Locale.US).trim();
    if (l.startsWith("the left") || l.startsWith("the right") || l.equals("left") || l.equals("right")) return null;
    s = s.trim();
    return s.length() == 0 ? null : s;
  }

  private static String charset(Object nfm) {
    String cs = (String) XposedHelpers.getObjectField(nfm, "mCharsetCode");
    return cs == null ? "UTF-8" : cs;
  }

  // Longest prefix within both caps, never splitting a surrogate pair.
  private static String fit(String road, String cs) {
    int end = Math.min(road.length(), MAX_CHARS);
    while (end > 0) {
      if (Character.isHighSurrogate(road.charAt(end - 1))) { end--; continue; }
      try {
        if (road.substring(0, end).getBytes(cs).length <= MAX_BYTES) break;
      } catch (java.io.UnsupportedEncodingException e) {
        return "";
      }
      end--;
    }
    return road.substring(0, end);
  }

  private static byte[] encode(String road, String cs) throws java.io.UnsupportedEncodingException {
    return fit(road, cs).getBytes(cs);
  }
}
