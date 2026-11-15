package com.starpilot.aaroad;

import android.graphics.Canvas;
import android.graphics.Paint;
import android.graphics.Path;
import android.graphics.RectF;

// The turn card's picture for a roundabout: a ring, the entry arm from the bottom, the way round to the exit in white
// with an arrowhead, the other arms grey, and the exit number in the ring (RoundaboutGeom has the angles). Replaces the
// stock arrow PNGs, which cannot show which exit. Drawn in the arrow's box; nothing here allocates a bitmap.
final class RoundaboutIcon {
  static final int ON = 0xffffffff, OFF = 0xff6c7076;
  private static final Paint line = new Paint(Paint.ANTI_ALIAS_FLAG), fill = new Paint(Paint.ANTI_ALIAS_FLAG),
      text = new Paint(Paint.ANTI_ALIAS_FLAG);
  private static final Path path = new Path();
  private static final double[] angles = new double[2 + 360 / 6];
  private static final float[] p = new float[2], q = new float[2], h = new float[6];

  static {
    line.setStyle(Paint.Style.STROKE);
    line.setStrokeCap(Paint.Cap.BUTT);
    line.setStrokeJoin(Paint.Join.ROUND);
    fill.setStyle(Paint.Style.FILL);
    text.setTextAlign(Paint.Align.CENTER);
    text.setFakeBoldText(true);
  }

  // exit: the exit's number (0 when unknown, then no number); degrees and modifier say where it is; left is
  // left-hand traffic (clockwise round the ring).
  static void draw(Canvas c, RectF box, int exit, int degrees, String modifier, boolean left) {
    float s = Math.min(box.width(), box.height()), cx = box.centerX(), cy = box.top + 0.45f * s, r = 0.25f * s;
    float w = Math.max(1.6f, 0.075f * s), bottom = box.bottom, top = box.top + 0.03f * s;
    double a = RoundaboutGeom.exitDegrees(degrees, modifier);
    line.setStrokeWidth(w);
    // the ring and the other arms, grey
    line.setColor(OFF);
    c.drawCircle(cx, cy, r, line);
    for (int i = 1; i <= 3; i++) {  // the arms a roundabout usually has besides the entry and the exit
      double arm = i * 90;
      if (Math.abs(arm - a) < 30 || Math.abs(arm - a - 360) < 30) continue;
      RoundaboutGeom.at(cx, cy, r, arm, p);
      RoundaboutGeom.at(cx, cy, r + 0.2f * s, arm, q);
      c.drawLine(p[0], p[1], q[0], q[1], line);
    }
    // the entry arm from the bottom, and the way round to the exit, white
    line.setColor(ON);
    RoundaboutGeom.at(cx, cy, r, 0, p);
    c.drawLine(cx, bottom, p[0], p[1], line);
    int n = RoundaboutGeom.path(a, left, 6, angles);
    path.rewind();
    for (int i = 0; i < n; i++) {
      RoundaboutGeom.at(cx, cy, r, angles[i], p);
      if (i == 0) path.moveTo(p[0], p[1]);
      else path.lineTo(p[0], p[1]);
    }
    c.drawPath(path, line);
    // the exit arm out to the edge, with its head; outward from the ring along the exit's angle
    RoundaboutGeom.at(cx, cy, r, a, p);
    RoundaboutGeom.at(cx, cy, 1, a, q);
    float dx = q[0] - cx, dy = q[1] - cy;
    float reach = Math.min(0.5f * s, Math.max(0.3f * s, 0.62f * s - r));
    float ex = p[0] + dx * reach, ey = p[1] + dy * reach;
    // keep the tip inside the box
    ex = Math.max(box.left + 0.06f * s, Math.min(box.right - 0.06f * s, ex));
    ey = Math.max(top, Math.min(bottom - 0.06f * s, ey));
    float headLen = 0.24f * s;
    c.drawLine(p[0], p[1], ex - dx * headLen * 0.5f, ey - dy * headLen * 0.5f, line);
    RoundaboutGeom.head(ex, ey, dx, dy, headLen, 0.15f * s, h);
    path.rewind();
    path.moveTo(h[0], h[1]);
    path.lineTo(h[2], h[3]);
    path.lineTo(h[4], h[5]);
    path.close();
    fill.setColor(ON);
    c.drawPath(path, fill);
    if (exit > 0) {
      text.setColor(ON);
      text.setTextSize(r * 1.25f);
      c.drawText(Integer.toString(exit), cx, cy + r * 0.45f, text);
    }
  }

  private RoundaboutIcon() {}
}
