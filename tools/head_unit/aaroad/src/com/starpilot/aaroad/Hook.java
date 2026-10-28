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
    XposedBridge.log("aaroad: hooked " + NFM);
  }

  private static String road(Object nfm) {
    String road = (String) XposedHelpers.getObjectField(nfm, "mRoad");
    if (road == null) return null;
    road = road.trim();
    return road.length() == 0 ? null : road;
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
