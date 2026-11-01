package com.starpilot.aaroad;

import android.graphics.Canvas;
import android.graphics.Paint;
import android.graphics.Path;
import android.graphics.RectF;
import android.graphics.Typeface;

// The traffic signal, stop and railroad crossing icons over the dash map (Signals), drawn here for both the view and
// the GPU layer: the view calls draw at each icon, the GPU layer draws the six sprites once into its texture (GlRoads.signals) and lays
// them one texel to one pixel, centred at the same whole pixel, so the two look the same.
//
// As Google Maps draws them: a traffic light is a dark rounded pill with a light rim and red, yellow and green dots,
// upright over the intersection whichever way the car heads; a stop sign is a red octagon with a white rim. A railroad
// crossing is the US advance warning sign (MUTCD W10-1): a yellow disc with a black rim, a black X and "R R" either
// side of it. The next one on the route is drawn BIG times larger.
final class SignalIcon {
  static final int CELL = 24;  // each sprite's square, in pixels; the icon centred in it
  static final int SPRITES = 6;  // signal, signal big, stop, stop big, crossing, crossing big: sprite(kind, big)
  static final float BIG = 1.25f;
  // a signal is W x H px with its rim, its dots R px with their centres GAP apart; a stop sign STOP_R px to a corner
  static final float W = 7, H = 15, R = 1.5f, GAP = 4, STOP_R = 6.5f;
  static final int BODY = 0xff1f2124, RIM = 0xffdadce0, RED = 0xffea4335, YELLOW = 0xfffbbc04, GREEN = 0xff34a853;
  static final int STOP_RED = 0xffd93025, WHITE = 0xffffffff;
  // a crossing is a disc of radius RR_R px to the outside of its rim, the X's arms RR_X px from the centre
  static final float RR_R = 9, RR_X = 6.3f;
  static final int RR_YELLOW = 0xffffd60a, RR_INK = 0xff111111;

  static int sprite(int kind, boolean big) {
    return (kind == Signals.STOP ? 2 : kind == Signals.CROSSING ? 4 : 0) + (big ? 1 : 0);
  }

  // The icon centred on (cx, cy). p and path are the caller's, so a frame allocates nothing but r.
  static void draw(Canvas c, int sprite, float cx, float cy, Paint p, Path path, RectF r) {
    float s = (sprite & 1) != 0 ? BIG : 1;
    p.setStyle(Paint.Style.FILL);
    if (sprite < 2) {
      float hw = W * s / 2, hh = H * s / 2;
      r.left = cx - hw;
      r.top = cy - hh;
      r.right = cx + hw;
      r.bottom = cy + hh;
      p.setColor(RIM);
      c.drawRoundRect(r, hw, hw, p);
      r.left += 1;
      r.top += 1;
      r.right -= 1;
      r.bottom -= 1;
      p.setColor(BODY);
      c.drawRoundRect(r, hw - 1, hw - 1, p);
      p.setColor(RED);
      c.drawCircle(cx, cy - GAP * s, R * s, p);
      p.setColor(YELLOW);
      c.drawCircle(cx, cy, R * s, p);
      p.setColor(GREEN);
      c.drawCircle(cx, cy + GAP * s, R * s, p);
    } else if (sprite >= 4) {
      p.setColor(RR_INK);
      c.drawCircle(cx, cy, RR_R * s, p);
      p.setColor(RR_YELLOW);
      c.drawCircle(cx, cy, RR_R * s - 1.3f, p);
      p.setColor(RR_INK);
      float a = RR_X * s * 0.7071f;
      path.rewind();
      path.moveTo(cx - a, cy - a);
      path.lineTo(cx + a, cy + a);
      path.moveTo(cx + a, cy - a);
      path.lineTo(cx - a, cy + a);
      p.setStyle(Paint.Style.STROKE);
      p.setStrokeWidth(1.6f * s);
      p.setStrokeCap(Paint.Cap.BUTT);
      c.drawPath(path, p);
      p.setStyle(Paint.Style.FILL);
      // the two R's, left and right of the X where its arms open widest
      p.setTextSize(5.2f * s);
      p.setTextAlign(Paint.Align.CENTER);
      Typeface was = p.getTypeface();
      p.setTypeface(Typeface.DEFAULT_BOLD);
      float ry = cy + 1.9f * s, rx = 5.3f * s;
      c.drawText("R", cx - rx, ry, p);
      c.drawText("R", cx + rx, ry, p);
      p.setTypeface(was);
    } else {
      p.setColor(WHITE);
      octagon(path, cx, cy, STOP_R * s + 1.2f);
      c.drawPath(path, p);
      p.setColor(STOP_RED);
      octagon(path, cx, cy, STOP_R * s);
      c.drawPath(path, p);  // no word, nor a bar for it: at this size a bar reads as "do not enter"
    }
  }

  // A regular octagon with flat top and sides, corners rc from the centre.
  private static void octagon(Path path, float cx, float cy, float rc) {
    path.rewind();
    for (int i = 0; i < 8; i++) {
      double a = Math.PI / 8 + i * Math.PI / 4;
      float x = cx + (float) (rc * Math.cos(a)), y = cy + (float) (rc * Math.sin(a));
      if (i == 0) path.moveTo(x, y);
      else path.lineTo(x, y);
    }
    path.close();
  }
}
