package com.starpilot.aaroad;

// Desktop check of the camera's tilt (CloseUp.pitch, CloseUp.camera): cruising it is exactly today's camera, zoomed in
// it looks further toward the horizon, it glides with the range with no step, and the car stays put on screen.
//   javac -d /tmp/tilt src/com/starpilot/aaroad/CloseUp.java test/TiltTest.java
//   java -cp /tmp/tilt com.starpilot.aaroad.TiltTest
// Exits 1 on a failure.
public class TiltTest {
  static int bad;

  static void check(boolean ok, String what) {
    System.out.println((ok ? "ok   " : "FAIL ") + what);
    if (!ok) bad++;
  }

  // MapView.cameraValues() as it was before the tilt, verbatim
  static float[] old() {
    double CAM_H = 150, CAM_ZC = -50, PITCH = Math.toRadians(55), FOCAL = 260, CX = 369, CY = 102.663;
    double cp = Math.cos(PITCH), sp = Math.sin(PITCH), c0 = CAM_H * sp - CAM_ZC * cp;
    double[] h = {FOCAL, CX * cp, CX * c0, 0, CY * cp - FOCAL * sp, CY * c0 + FOCAL * (CAM_H * cp + CAM_ZC * sp), 0, cp, c0};
    float[] v = new float[9];
    for (int i = 0; i < 9; i++) v[i] = (float) (h[i] / c0);
    return v;
  }

  static float[] at(float[] v, double x, double z) {
    double w = v[6] * x + v[7] * z + v[8];
    return new float[] {(float) ((v[0] * x + v[1] * z + v[2]) / w), (float) ((v[3] * x + v[4] * z + v[5]) / w)};
  }

  // the screen row of the ground point z m ahead of the car
  static float row(float[] v, double z) {
    return at(v, 0, z)[1];
  }

  public static void main(String[] a) {
    float[] o = old(), c = new float[9];
    CloseUp.camera(CloseUp.PITCH, c);
    boolean same = true;
    for (int i = 0; i < 9; i++) same &= Float.floatToIntBits(o[i]) == Float.floatToIntBits(c[i]);
    check(same, "at today's tilt the camera is bit for bit the one before");
    for (double r : new double[] {260, 300, 500, 2000}) {
      check(CloseUp.pitch(r) == CloseUp.PITCH, "range " + r + " m (cruise): today's tilt");
    }
    for (double r : new double[] {60, 120, 180}) {
      check(CloseUp.pitch(r) == CloseUp.CLOSE_PITCH, "range " + r + " m (close-up): the close-up tilt");
    }

    // the glide: the tilt only moves one way as the range grows, and no step between ranges a metre apart
    double prev = CloseUp.pitch(100), step = 0;
    boolean mono = true;
    for (double r = 100; r <= 300; r += 1) {
      double p = CloseUp.pitch(r);
      mono &= p >= prev;
      step = Math.max(step, p - prev);
      prev = p;
    }
    check(mono, "the tilt eases back toward today's as the range grows");
    check(Math.toDegrees(step) < 0.3, "no jump: at most " + String.format("%.3f", Math.toDegrees(step)) + " degrees a metre of range");
    double mid = Math.toDegrees(CloseUp.pitch((CloseUp.TILT_SHOW + CloseUp.TILT_HIDE) / 2));
    check(Math.abs(mid - 51.5) < 0.01, "half way at the band's middle: " + String.format("%.2f", mid) + " degrees");

    // the car stays at (369, 180) and its px per metre across stays the same at every tilt, near, a rounding off
    float[] car0 = at(o, 0, 0);
    double across0 = at(o, 1, 0)[0] - car0[0];
    boolean still = true, scale = true;
    for (double r = 100; r <= 300; r += 5) {
      CloseUp.camera(CloseUp.pitch(r), c);
      float[] car = at(c, 0, 0);
      still &= Math.abs(car[0] - car0[0]) < 1e-3 && Math.abs(car[1] - car0[1]) < 1e-3;
      scale &= Math.abs((at(c, 1, 0)[0] - car[0]) - across0) < 1e-4;
    }
    check(Math.abs(car0[0] - 369) < 1e-3 && Math.abs(car0[1] - 180) < 0.01 && still,
          "the car is drawn at (" + car0[0] + ", " + car0[1] + ") at every tilt");
    check(scale, "and the road is as wide at the car at every tilt");

    // zoomed in the road ahead lies further back: more of it fits, so a point 120 m ahead sits a little lower on the
    // screen, and the screen's top edge sees further ahead (CloseUp's 48 degrees: about 191 m, against 177 m)
    CloseUp.camera(CloseUp.CLOSE_PITCH, c);
    check(row(c, 120) > row(o, 120) + 2, "120 m ahead: row " + row(c, 120) + " zoomed in, " + row(o, 120) + " cruising");
    double zTop = 0;
    while (row(c, zTop) > 0) zTop += 0.5;
    check(zTop > 185 && zTop < 195, "the top edge sees " + zTop + " m ahead zoomed in");
    // the camera's place does not move, so CloseUp.lift() still draws a raised point where the camera sees it: the
    // ray through a point h up meets the ground at the lift's scaled point, at any tilt
    for (double[] p : new double[][] {{30, 60, 20}, {-80, 140, 40}}) {
      double x = p[0], z = p[1], hd = p[2];
      float s = CloseUp.lift((float) hd);
      double gx = x * s, gz = CloseUp.CAM_ZC + (z - CloseUp.CAM_ZC) * s;
      // the true point through the tilted camera: the 3D camera at (0, CAM_H, CAM_ZC) looking down CLOSE_PITCH
      double cp = Math.cos(CloseUp.CLOSE_PITCH), sp = Math.sin(CloseUp.CLOSE_PITCH);
      double dy = CloseUp.CAM_H - hd, dz = z - CloseUp.CAM_ZC;
      double depth = dz * cp + dy * sp, up = dz * sp - dy * cp;
      double c0 = CloseUp.CAM_H * sp - CloseUp.CAM_ZC * cp;
      double f = c[0];  // px per unit depth, over c0: the matrix is divided by c0
      float[] g = at(c, gx, gz);
      // the ground point's screen place, from the 3D one: the same column and row
      double gdy = CloseUp.CAM_H, gdz = gz - CloseUp.CAM_ZC;
      double gdepth = gdz * cp + gdy * sp, gup = gdz * sp - gdy * cp;
      boolean sameRay = Math.abs(x / depth - gx / gdepth) < 1e-6 && Math.abs(up / depth - gup / gdepth) < 1e-6;
      check(sameRay && f > 0 && c0 > 0, "a point " + hd + " m up at (" + x + ", " + z + "): its lifted ground point is on its ray");
    }

    System.out.println(bad == 0 ? "all ok" : bad + " failed");
    System.exit(bad == 0 ? 0 : 1);
  }
}
