package com.starpilot.aaroad;

import android.app.AndroidAppHelper;
import android.content.Context;
import android.content.Intent;
import de.robv.android.xposed.XC_MethodHook;
import de.robv.android.xposed.XposedBridge;
import de.robv.android.xposed.XposedHelpers;

// Honda Hack's custom Civic meter (C0268ua, meter_civic.xml) already draws on the cluster and has
// a nav panel: turn arrow, distance, street. It is fed by unprotected broadcasts it registers for
// itself (START_NAVI, UPDATE_NAVI_WAZE, STOP_NAVI), so it does not need the Honda cluster to accept
// the guide point. Send Android Auto's next turn there.
//
// UPDATE_NAVI_WAZE is dropped while Honda Hack's "enable_screen_cast" is on (C0268ua.z), so the
// panel only shows with screen cast off.
final class Panel {
  private static final String HH = "cn.autohack.hondahack.";
  private static final double FEET_PER_METER = 3.28084, METERS_PER_MILE = 1609.344;

  private static String icon, street;
  private static int meters = -1;
  private static boolean active;

  static void install(Class<?> nfm) {
    XposedHelpers.findAndHookMethod(nfm, "onNextTurn", String.class, int.class, int.class, byte[].class,
        int.class, int.class, new XC_MethodHook() {
      @Override
      protected void afterHookedMethod(MethodHookParam param) {
        try {
          Object o = param.thisObject;
          String i = icon(String.valueOf(XposedHelpers.getObjectField(o, "mEvent")),
                          String.valueOf(XposedHelpers.getObjectField(o, "mTurnSide")), (Integer) param.args[5]);
          String s = Hook.street((String) param.args[0]);
          if (s == null) s = "";
          // Android Auto repeats the same turn about once a second; keep its distance. A new turn
          // drops it until onNextTurnDistance, since mDistanceMeters still holds the previous turn.
          if (!i.equals(icon) || !s.equals(street)) meters = -1;
          icon = i;
          street = s;
          send();
        } catch (Throwable t) {
          XposedBridge.log("aaroad: panel turn failed: " + t);
        }
      }
    });
    XposedHelpers.findAndHookMethod(nfm, "onNextTurnDistance", int.class, int.class, float.class, int.class,
        new XC_MethodHook() {
      @Override
      protected void afterHookedMethod(MethodHookParam param) {
        try {
          meters = (Integer) param.args[0];
          if (icon != null) send();
        } catch (Throwable t) {
          XposedBridge.log("aaroad: panel distance failed: " + t);
        }
      }
    });
    XposedHelpers.findAndHookMethod(nfm, "onStatus", int.class, new XC_MethodHook() {
      @Override
      protected void afterHookedMethod(MethodHookParam param) {
        try {
          boolean now = "ACTIVE".equals(String.valueOf(XposedHelpers.getObjectField(param.thisObject, "mStatus")));
          if (now == active) return;
          active = now;
          if (!now) {
            icon = street = null;
            meters = -1;
          }
          broadcast(new Intent(HH + (now ? "START_NAVI" : "STOP_NAVI")));
          XposedBridge.log("aaroad: panel " + (now ? "start" : "stop"));
        } catch (Throwable t) {
          XposedBridge.log("aaroad: panel status failed: " + t);
        }
      }
    });
  }

  private static void send() {
    Intent i = new Intent(HH + "UPDATE_NAVI_WAZE");
    i.putExtra("icon", icon);
    i.putExtra("street", street);
    if (meters >= 0) {
      String[] d = distance(meters);
      i.putExtra("distance", d[0]);
      i.putExtra("distUnit", d[1]);
    }
    broadcast(i);
  }

  private static void broadcast(Intent i) {
    Context c = AndroidAppHelper.currentApplication();
    if (c != null) c.sendBroadcast(i);
  }

  // Honda Hack drawable suffix (car_big_trans_direction_<x>, car_big_trans_directions_roundabout*)
  // for Android Auto's NextTurnEnum and TurnSide names.
  static String icon(String event, String side, int turnNumber) {
    String lr = "LEFT".equals(side) ? "left" : "RIGHT".equals(side) ? "right" : null;
    if (event.equals("TURN") || event.equals("SHARP_TURN") || event.equals("SLIGHT_TURN") || event.equals("ON_RAMP")) {
      return lr == null ? "forward" : lr;
    }
    if (event.equals("OFF_RAMP") || event.equals("FORK")) return lr == null ? "forward" : "exit_" + lr;
    if (event.equals("U_TURN")) return "u_turn";
    if (event.startsWith("ROUNDABOUT")) return turnNumber >= 1 && turnNumber <= 8 ? "roundabout_" + turnNumber : "roundabout";
    if (event.equals("DESTINATION")) return "end";
    if (event.equals("DEPART")) return "start";
    return "forward";
  }

  // US display: feet in 50 ft steps under 0.1 mi, then miles (one decimal under 10 mi).
  static String[] distance(int meters) {
    if (meters < 0.1 * METERS_PER_MILE) {
      long ft = Math.round(meters * FEET_PER_METER / 50.0) * 50;
      return new String[] {String.valueOf(Math.max(ft, 50)), "ft"};
    }
    double mi = meters / METERS_PER_MILE;
    return new String[] {mi < 10 ? String.format(java.util.Locale.US, "%.1f", mi) : String.valueOf(Math.round(mi)), "mi"};
  }

  private Panel() {}
}
