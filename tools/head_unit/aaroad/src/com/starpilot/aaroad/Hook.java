package com.starpilot.aaroad;

import de.robv.android.xposed.IXposedHookLoadPackage;
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
// The cluster answered every send on 2026-10-28 with reply 2 (not stored, resend) and
// nothing resends in Android Auto mode, so the turn is replayed once per new turn.
public class Hook implements IXposedHookLoadPackage {
  private static final String PKG = "com.mitsubishielectric.ada.app.androidauto";
  private static final String NFM = PKG + ".navigation.NavigationFocusManager";
  private static final int MAX_CHARS = 31;  // 5-bit length field, same cap as present location
  private static final int MAX_BYTES = 36;  // CanFormatAnalyze.replyBcanNavGuide4 truncates here

  @Override
  public void handleLoadPackage(LoadPackageParam lpparam) throws Throwable {
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
        if (replaying) return;
        lastTurn = param.args.clone();
        resent = false;
        XposedBridge.log("aaroad: street \"" + street((String) param.args[0]) + "\"");
      }
    });

    XposedHelpers.findAndHookMethod(NFM + "$4$1", lpparam.classLoader, "run", new XC_MethodHook() {
      @Override
      protected void afterHookedMethod(MethodHookParam param) {
        try {
          Object info = XposedHelpers.getObjectField(param.thisObject, "val$info");
          int reply = (Integer) XposedHelpers.callMethod(info, "getGuidancePointInfoReply");
          if (reply != 2 || resent || lastTurn == null) return;
          resent = true;
          Object[] a = lastTurn;
          Object nfmObj = XposedHelpers.getObjectField(XposedHelpers.getObjectField(param.thisObject, "this$1"), "this$0");
          XposedBridge.log("aaroad: cluster reply 2, resending turn once");
          replaying = true;
          try {
            XposedHelpers.callMethod(nfmObj, "onNextTurn", a);
          } finally {
            replaying = false;
          }
        } catch (Throwable t) {
          XposedBridge.log(t);
        }
      }
    });
    XposedBridge.log("aaroad: hooked " + NFM);
  }

  private static volatile Object[] lastTurn;
  private static volatile boolean resent;
  private static volatile boolean replaying;

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
