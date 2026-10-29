"""Render the lead-car badge sprites (rear view of the Crystal Black Civic Sport) for the Android Auto car screen.

  uv run --with cairosvg --with pillow python3 tools/head_unit/civic/render_lead_sprites.py

Writes openpilot/starpilot/ui/assets/lead_civic.png (tail lamps dim) and lead_civic_brake.png (brake lamps lit), one
crop for both so they overlay exactly, and prints the body and ground fractions that projection_theme.py uses."""
import io
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
import civic3d as c
import cairosvg
from PIL import Image

ASSETS = Path(__file__).parents[3] / 'openpilot/starpilot/ui/assets'
OUT_W, SS = 256, 4

def render(brake, elev=14.0):
  model = c.model
  def trimmed(*a, **k):   # keep the two inner ground-shadow steps; the outer ones smudge at badge size
    polys, rim, dark = model(*a, **k)
    return [p for p in polys if not (isinstance(p[1], str) and p[1] in ('rgba0,0,0,0.10', 'rgba0,0,0,0.14'))], rim, dark
  c.model = trimmed
  base = c.decals
  if not brake:
    def dim(trim):
      return [(ps, '#8a0e15' if col == '#b8121b' else '#c43c3c' if col == '#ff5a5a' else col, kind) for ps, col, kind in base(trim)]
    c.decals = dim
  else:
    def lit(trim):
      out = []
      for ps, col, kind in base(trim):
        if col == '#b8121b':
          col = '#ff2a1c'
        elif col == '#ff5a5a':
          col = '#fff0e8'
        out.append((ps, col, kind))
      return out
    c.decals = lit
  try:
    W = OUT_W * SS
    svg, h = c.civic3d(W / 2, W, W * .80, paint='Crystal Black', trim='sport', elev=elev)
  finally:
    c.decals = base
    c.model = model
  doc = f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{2 * W}">{svg}</svg>'
  img = Image.open(io.BytesIO(cairosvg.svg2png(bytestring=doc.encode()))).convert('RGBA')
  body = (W / 2 - W * .40, W - h / 2, W / 2 + W * .40, W + h / 2)
  return img, body

imgs = {}
for brake in (False, True):
  img, body = render(brake)
  imgs[brake] = (img, body)
# one crop box for both so they overlay exactly
box = imgs[False][0].getchannel('A').point(lambda a: 255 if a > 6 else 0).getbbox()
x0, y0, x1, y1 = box
pad = 2 * SS
x0, y0, x1, y1 = x0 - pad, y0 - pad, x1 + pad, y1 + pad
for brake, (img, _body) in imgs.items():
  crop = img.crop((x0, y0, x1, y1))
  w = OUT_W
  hgt = round(crop.height * w / crop.width)
  crop.resize((w, hgt), Image.LANCZOS).save(ASSETS / f'lead_civic{"_brake" if brake else ""}.png', optimize=True)
bx0, by0, bx1, by1 = imgs[False][1]
cw, ch = x1 - x0, y1 - y0
print('size', OUT_W, round(ch * OUT_W / cw))
print(f'body x {(bx0 - x0) / cw:.3f}..{(bx1 - x0) / cw:.3f}  y {(by0 - y0) / ch:.3f}..{(by1 - y0) / ch:.3f} (fractions of sprite)')
