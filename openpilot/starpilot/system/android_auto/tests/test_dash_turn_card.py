"""The dash map's turn card (aaroad on the head unit) says what the StarPilot Auto car screen says.

aaroad's Turn.java is the car screen's arrow choice (navigation_icons.icon_name), distance (distance_text) and
instruction text in Java. This compiles it and runs it against the Python for every maneuver Mapbox sends.
"""

import shutil
import subprocess
from pathlib import Path

import pytest

from openpilot.starpilot.ui.navigation_icons import MANIFEST, icon_name
from openpilot.starpilot.ui.navigation_state import distance_text

SRC = Path(__file__).resolve().parents[5] / 'tools/head_unit/aaroad/src/com/starpilot/aaroad/Turn.java'

TYPES = ['turn', 'new name', 'depart', 'arrive', 'merge', 'on ramp', 'off ramp', 'fork', 'end of road', 'continue',
         'roundabout', 'rotary', 'roundabout turn', 'notification', 'exit roundabout', 'exit rotary', 'use lane', '']
MODIFIERS = ['uturn', 'sharp right', 'right', 'slight right', 'straight', 'slight left', 'left', 'sharp left',
             'sharpRight', 'slightRight', 'slightLeft', 'sharpLeft', '']

DRIVER = '''package com.starpilot.aaroad;
import java.io.*;
import java.nio.charset.StandardCharsets;
import java.util.*;
public class TurnDriver {
  static String un(String h) {
    byte[] b = new byte[h.length() / 2];
    for (int i = 0; i < b.length; i++) b[i] = (byte) Integer.parseInt(h.substring(2 * i, 2 * i + 2), 16);
    return new String(b, StandardCharsets.UTF_8);
  }
  public static void main(String[] a) throws IOException {
    BufferedReader in = new BufferedReader(new InputStreamReader(System.in, StandardCharsets.UTF_8));
    Set<String> have = new HashSet<String>(Arrays.asList(in.readLine().split(" ")));
    PrintStream out = new PrintStream(System.out, true, "UTF-8");
    for (String line; (line = in.readLine()) != null; ) {
      String[] f = line.split(" ", -1);
      if (f[0].equals("icon")) out.println(Turn.icon(un(f[1]), un(f[2]), have));
      else if (f[0].equals("dist")) out.println(Turn.distance(Double.parseDouble(f[1]), f[2].equals("1")));
      else out.println(Turn.text(un(f[1])));
    }
  }
}
'''


def _hex(s: str) -> str:
  return s.encode().hex()


@pytest.fixture(scope='module')
def java(tmp_path_factory):
  if shutil.which('javac') is None or shutil.which('java') is None:
    pytest.skip('no JDK')
  d = tmp_path_factory.mktemp('turn')
  (d / 'TurnDriver.java').write_text(DRIVER)
  subprocess.run(['javac', '-d', str(d), str(SRC), str(d / 'TurnDriver.java')], check=True, capture_output=True)

  def run(lines):
    stdin = ' '.join(MANIFEST) + '\n' + ''.join(line + '\n' for line in lines)
    out = subprocess.run(['java', '-cp', str(d), 'com.starpilot.aaroad.TurnDriver'], input=stdin, check=True,
                         capture_output=True, text=True, encoding='utf-8').stdout
    return out.split('\n')[:len(lines)]
  return run


def test_arrow_is_the_car_screens_for_every_maneuver(java):
  cases = [(t, m) for t in TYPES for m in MODIFIERS] + [(None, None), ('turn', None)]
  got = java([f'icon {_hex(t or "")} {_hex(m or "")}' for t, m in cases])
  assert got == [icon_name(t or '', m or '') for t, m in cases]


@pytest.mark.parametrize('metric', [False, True])
def test_distance_matches_across_the_range_and_at_rounding_ties(java, metric):
  meters = [i * 0.1 for i in range(20000)] + [i * 1.0 for i in range(2000, 40000, 7)]
  # exact ties of the step rounding and of the tenths, where Java's usual rounding goes up and Python's goes to even
  meters += [12.5, 37.5, 62.5, 1050.0, 1150.0, 1250.0, 5280 * 1.25 / 3.28084, 5280 * 1.35 / 3.28084]
  got = java([f'dist {m!r} {int(metric)}' for m in meters])
  want = [distance_text(m, metric) for m in meters]
  bad = [(m, g, w) for m, g, w in zip(meters, got, want, strict=True) if g != w]
  assert not bad, bad[:10]


def test_text_is_collapsed_like_the_car_screens(java):
  texts = ['Exit 19A  I-635 East\t/ LBJ Freeway', '  Turn right onto\nMain St  ', '', 'Thác Nĩng Đầu Mùa', 'x ' * 200]
  got = java([f'text {_hex(t)}' for t in texts])
  assert got == [' '.join(t.split())[:240] for t in texts]
