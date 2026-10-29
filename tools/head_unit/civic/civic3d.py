# ruff: noqa: E701, E702, E731, B905, B007, UP031  -- dense geometry tables from the head-unit mockups, kept as drawn
# 2016-2021 Civic sedan (10th gen) as a small 3D model for the cluster car icon, built from Peter's
# photos: a 2016 sedan straight from behind (775 px/m) and a top view (360 px/m). Metres: x right,
# y up, z forward from the rear bumper. 4.64 m long, 1.80 m wide, 1.415 m tall.
# The map is heading-up, so the car is always seen from straight behind: the head unit can draw a
# pre-rendered sprite of this model instead of running a 3D renderer.
import math

LEN = 4.64
ST = [  # z, Wb (body half width), Wbelt, Hbelt, Wg (glass top half width), Hg, Ht (roof top)
    (0.00, .55, .50, .70, .30, .74, .75),
    (0.05, .70, .64, .86, .45, .90, .92),
    (0.12, .80, .74, .98, .55, 1.03, 1.05),
    (0.18, .85, .80, 1.03, .62, 1.065, 1.075),
    (0.42, .89, .84, 1.04, .66, 1.08, 1.10),
    (0.70, .90, .85, 1.03, .66, 1.18, 1.21),
    (1.00, .90, .85, 1.02, .65, 1.27, 1.30),
    (1.35, .90, .85, 1.01, .64, 1.355, 1.38),
    (1.70, .90, .86, 1.00, .65, 1.385, 1.41),
    (2.10, .90, .86, .99, .65, 1.39, 1.415),
    (2.55, .90, .86, .98, .64, 1.36, 1.39),
    (2.90, .90, .86, .97, .62, 1.25, 1.28),
    (3.25, .90, .85, .96, .58, 1.12, 1.15),
    (3.55, .89, .84, .95, .55, .99, 1.02),
    (3.90, .88, .82, .92, .55, .93, .96),
    (4.20, .84, .78, .86, .52, .87, .90),
    (4.45, .74, .68, .79, .45, .80, .82),
    (4.60, .55, .50, .70, .32, .71, .72),
    (4.64, .40, .36, .60, .22, .60, .60),
]

def _smooth(st, n=3):           # Catmull-Rom between stations, so the roof and glass shade smoothly
    out = []
    for i in range(len(st) - 1):
        p0, p1, p2, p3 = st[max(i - 1, 0)], st[i], st[i + 1], st[min(i + 2, len(st) - 1)]
        for k in range(n):
            t = k / n
            out.append(tuple(0.5 * (2 * b + (c - a) * t + (2 * a - 5 * b + 4 * c - d) * t * t + (3 * b - a - 3 * c + d) * t ** 3)
                             for a, b, c, d in zip(p0, p1, p2, p3)))
    return out + [st[-1]]
FINE = _smooth(ST)

def ring(Wb, Wbelt, Hbelt, Wg, Hg, Ht):
    h2 = min(.80, Hbelt - .06); h1 = min(.52, h2 - .04)
    R = [(.9 * Wb, .30), (Wb, h1), (.99 * Wb, h2), (Wbelt, Hbelt), (Wg, Hg), (.8 * Wg, Ht - .02), (.4 * Wg, Ht - .005), (0.0, Ht)]
    return R + [(-x, h) for x, h in reversed(R[:-1])]

def params(z):
    z = min(max(z, 0.0), LEN)
    for a, b in zip(FINE, FINE[1:]):
        if z <= b[0]:
            t = (z - a[0]) / ((b[0] - a[0]) or 1)
            return [u + (v - u) * t for u, v in zip(a[1:], b[1:])]
    return list(FINE[-1][1:])

def inside(pt, poly):
    x, y = pt; c = False
    for (x0, y0), (x1, y1) in zip(poly, poly[1:] + poly[:1]):
        if (y0 > y) != (y1 > y) and x < x0 + (x1 - x0) * (y - y0) / (y1 - y0): c = not c
    return c

def surf(X, H):                 # where a point drawn on the rear elevation lands on the body
    H = max(H, .305); lo = None
    for i in range(101):
        z = 2.0 * i / 100
        if inside((X, H), ring(*params(z))):
            hi = z; break
        lo = z
    else:                       # outside the body at every station: pull it in to the surface
        return surf(X * .98, H)
    if lo is None: return (X, H, 0.0)
    for _ in range(14):
        m = (lo + hi) / 2
        if inside((X, H), ring(*params(m))): hi = m
        else: lo = m
    return (X, H, hi)

# ---- vectors
def sub(a, b): return (a[0] - b[0], a[1] - b[1], a[2] - b[2])
def dot(a, b): return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
def cross(a, b): return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])
def norm(a):
    n = math.sqrt(dot(a, a)) or 1; return (a[0] / n, a[1] / n, a[2] / n)
def centroid(ps): return tuple(sum(p[i] for p in ps) / len(ps) for i in range(3))

def normal(ps, inner):          # face normal pointing away from the point `inner`
    n = norm(cross(sub(ps[2], ps[0]), sub(ps[-1], ps[1])) if len(ps) == 4 else cross(sub(ps[1], ps[0]), sub(ps[2], ps[0])))
    return n if dot(n, sub(centroid(ps), inner)) >= 0 else (-n[0], -n[1], -n[2])

# ---- model
PAINT = {
    'Platinum White': '#e8e7e2', 'Lunar Silver': '#adb1b7', 'Sonic Gray': '#868b91', 'Modern Steel': '#5a5f66',
    'Crystal Black': '#1e2024', 'Aegean Blue': '#1f50b2', 'Cosmic Blue': '#22325e', 'Rallye Red': '#c4161c',
    'Molten Lava': '#b4361f',
}
GLASS, TYRE, TRIM = '#1b2330', '#121212', '#18191b'

def body_faces(paint):
    faces = []
    rings = [(s[0], ring(*s[1:])) for s in FINE]
    for (za, ra), (zb, rb) in zip(rings, rings[1:]):
        zm = (za + zb) / 2
        for j in range(len(ra) - 1):
            ps = [(ra[j][0], ra[j][1], za), (ra[j + 1][0], ra[j + 1][1], za), (rb[j + 1][0], rb[j + 1][1], zb), (rb[j][0], rb[j][1], zb)]
            col = paint
            if j in (3, 10) and 1.05 <= zm <= 3.4: col = GLASS
            if 5 <= j <= 8 and (0.42 <= zm <= 1.35 or 2.55 <= zm <= 3.55): col = GLASS
            faces.append((ps, col, normal(ps, (0, .8, zm))))
    for z, r in (rings[0], rings[-1]):
        ps = [(x, h, z) for x, h in r]
        faces.append((ps, paint, (0, 0, -1 if z == 0 else 1)))
    return faces

def box(x0, x1, y0, y1, z0, z1, col, taper=0.0):
    c = [(x, y, z) for x in (x0, x1) for y in (y0, y1) for z in (z0, z1)]
    if taper: c = [(x, y, z + (taper if x == x1 and z == z0 else -taper if x == x1 else 0)) for x, y, z in c]
    q = [(0, 1, 3, 2), (4, 5, 7, 6), (0, 1, 5, 4), (2, 3, 7, 6), (0, 2, 6, 4), (1, 3, 7, 5)]
    mid = ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2)
    return [([c[i] for i in f], col, normal([c[i] for i in f], mid)) for f in q]

def tyre(xc, zc, w=.215, r=.32, n=14):
    out, ang = [], [2 * math.pi * (i + .5) / n for i in range(n)]
    for side in (-1, 1):
        ps = [(xc + side * w / 2, r + r * math.cos(a), zc + r * math.sin(a)) for a in ang]
        out.append((ps, TYRE, (side, 0, 0)))
    for a, b in zip(ang, ang[1:] + ang[:1]):
        ps = [(xc + s * w / 2, r + r * math.cos(t), zc + r * math.sin(t)) for s, t in ((-1, a), (1, a), (1, b), (-1, b))]
        out.append((ps, TYRE, normal(ps, (xc, r, zc))))
    return out

def extras(paint, trim):
    out = []
    for s in (-1, 1):            # mirrors on short arms at the front of the side glass
        out += box(*sorted((s * .84, s * .99)), .88, 1.0, 3.06, 3.24, paint)
    ht = lambda z: params(z)[5]
    z0, z1, zt = 1.22, 1.50, 1.27 # shark fin: tall end at the back
    fin = [(-.025, ht(z0), z0), (.025, ht(z0), z0), (.025, ht(z1), z1), (-.025, ht(z1), z1), (-.012, ht(zt) + .075, zt), (.012, ht(zt) + .075, zt)]
    for f in ((0, 3, 4), (1, 2, 5), (0, 1, 5, 4), (3, 2, 5, 4)):
        ps = [fin[i] for i in f]; out.append((ps, paint, normal(ps, (0, ht(1.3), 1.36))))
    if trim == 'si':             # Si wing on two posts
        out += box(-.76, .76, 1.125, 1.155, .02, .25, paint)
        for s in (-1, 1): out += box(s * .42 - .03, s * .42 + .03, ht(.14) - .02, 1.125, .10, .18, paint)
    return out

# rear decals, drawn on the rear elevation (X right, H up, m) and wrapped onto the body
def mirror(ps): return [(-x, h) for x, h in reversed(ps)]
TAIL = [(-.387, 1.071), (-.665, 1.052), (-.70, 1.04), (-.806, .968), (-.82, .897), (-.806, .806), (-.70, .785), (-.523, .748),
        (-.639, .961), (-.600, 1.013), (-.394, 1.058)]
LENS = [(-.72, .955), (-.648, .955), (-.588, .83), (-.72, .82)]
GLOW = [(-.40, 1.064), (-.66, 1.046), (-.785, .962), (-.800, .897), (-.787, .822), (-.70, .800)]
POCKET = [(-.84, .52), (-.62, .52), (-.58, .47), (-.62, .39), (-.84, .40)]
REFL = [(-.832, .41), (-.785, .41), (-.785, .503), (-.832, .503)]
SHUT = [(-.523, .748), (-.471, .677), (-.419, .645), (.419, .645), (.471, .677), (.523, .748)]
RECESS = [(-.316, .884), (.316, .884), (.445, .768), (.42, .652), (-.42, .652), (-.445, .768)]
PLATE = [(-.158, .703), (.158, .703), (.158, .852), (-.158, .852)]
VALANCE = [(-.58, .40), (-.55, .426), (.55, .426), (.58, .40), (.56, .305), (-.56, .305)]
EXHAUST = [(-.13, .308), (.13, .308), (.15, .37), (-.15, .37)]
BADGE = [(math.cos(a) * .06, .977 + math.sin(a) * .047) for a in [2 * math.pi * i / 16 for i in range(16)]]

def decals(trim):
    d = []
    for side in (lambda p: p, mirror):
        d += [(side(POCKET), TRIM, 0), (side(REFL), '#8a0f15' if trim == 'ex' else '#a5121a', 0),
              (side(TAIL), '#b8121b', .2), (side(LENS), '#d9d6d2' if trim == 'ex' else '#cfcbc6', .2),
              (side(GLOW), '#ff5a5a', 'line')]
    d += [(RECESS, 'shade', 0), (PLATE, '#dedede', 0), (SHUT, '#000000', 'line'), (VALANCE, TRIM, 0), (BADGE, '#e6e6e6', 'ring')]
    if trim in ('sport', 'si'): d.append((EXHAUST, '#2b2c2e', 'tip'))
    return d

def dense2(ps, closed, step=.015):
    out = []
    for a, b in zip(ps, ps[1:] + (ps[:1] if closed else [])):
        n = max(1, int(math.hypot(b[0] - a[0], b[1] - a[1]) / step))
        out += [(a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n) for i in range(n)]
    return out if closed else out + [ps[-1]]

# ---- shading
def rgb(h): return [int(h[i:i + 2], 16) for i in (1, 3, 5)]
def hexc(c): return '#%02x%02x%02x' % tuple(max(0, min(255, int(round(v)))) for v in c)
def shade(col, n, L, Hv, gloss):
    k = 0.50 + 0.50 * max(0.0, dot(n, L))
    s = gloss * max(0.0, dot(n, Hv)) ** 28
    return hexc([v * k + 255 * s for v in rgb(col)])

class Cam:
    def __init__(s, yaw=0.0, elev=25.0, dist=9.0):
        e = math.radians(elev)
        s.yaw, s.dist = math.radians(yaw), dist
        s.T = (0.0, 0.55, LEN / 2)
        s.d = (0.0, math.sin(e), -math.cos(e))            # target -> camera
        s.f, s.r, s.u = (0.0, -math.sin(e), math.cos(e)), (1.0, 0.0, 0.0), (0.0, math.cos(e), math.sin(e))
        s.C = tuple(t + (dist or 1e3) * v for t, v in zip(s.T, s.d))
        s.L = norm((s.d[0] - .45, s.d[1] + .8, s.d[2]))    # light from the camera side, above and left
        s.Hv = norm(tuple(a + b for a, b in zip(s.L, s.d)))
    def rot(s, p):               # the car turned right by yaw
        c, si = math.cos(s.yaw), math.sin(s.yaw); x, y, z = p[0], p[1], p[2] - LEN / 2
        return (x * c + z * si, y, -x * si + z * c + LEN / 2)
    def rotn(s, n):
        c, si = math.cos(s.yaw), math.sin(s.yaw); return (n[0] * c + n[2] * si, n[1], -n[0] * si + n[2] * c)
    def proj(s, p):
        v = sub(s.rot(p), s.C); xc, yc, zc = dot(v, s.r), dot(v, s.u), dot(v, s.f)
        return (xc, -yc) if not s.dist else (s.dist * xc / zc, -s.dist * yc / zc)
    def depth(s, p): return dot(sub(s.rot(p), s.C), s.f)
    def facing(s, ps, n): return dot(s.rotn(n), sub(s.C, s.rot(centroid(ps)))) > 0

def model(paint='Lunar Silver', trim='ex', cam=None):
    """Polygons in raw camera units, back to front: (points2d, fill, stroke_or_None, width_m)."""
    cam = cam or Cam(); body = PAINT.get(paint, paint)
    dark = sum(rgb(body)) < 3 * 0x70
    polys = []
    # soft ground shadow: the footprint grown in a few faint steps
    for g, a in ((.30, .10), (.20, .14), (.10, .18), (.02, .25)):
        fp = [(s * (params(z)[0] + g), 0.0, z) for s in (1,) for z in [i * LEN / 24 for i in range(25)]]
        fp = [(fp[0][0] * 0, 0.0, -g)] + fp + [(0, 0.0, LEN + g)] + [(-x, y, z) for x, y, z in reversed(fp)]
        polys.append(([cam.proj(p) for p in fp], 'rgba0,0,0,%.2f' % a, None, 0))
    faces = []
    for ps, col, n in [f for x in (.78, -.78) for zc in (.98, 3.68) for f in tyre(x, zc)]:
        if cam.facing(ps, n): polys.append(([cam.proj(p) for p in ps], shade(col, cam.rotn(n), cam.L, cam.Hv, .1), 'same', 0))
    for ps, col, n in body_faces(body) + extras(body, trim):
        if cam.facing(ps, n): faces.append((cam.depth(centroid(ps)), ps, col, n))
    faces.sort(key=lambda f: -f[0])
    rim_layer = [([cam.proj(p) for p in ps], 'rim', None, 0) for _, ps, _, _ in faces]
    for _, ps, col, n in faces:
        gloss = .55 if col == GLASS else .25
        polys.append(([cam.proj(p) for p in ps], shade(col, cam.rotn(n), cam.L, cam.Hv, gloss), 'same', 0))
    for ps2, col, kind in decals(trim):
        closed = kind not in ('line',)
        pts3 = [surf(x, h) for x, h in dense2(ps2, closed)]
        mx, mh = sum(x for x, _ in ps2) / len(ps2), sum(h for _, h in ps2) / len(ps2)
        c, a, b = surf(mx, mh), surf(mx + .02, mh), surf(mx, mh + .02)
        n = norm(cross(sub(b, c), sub(a, c)))
        if n[2] > 0: n = (-n[0], -n[1], -n[2])
        if dot(cam.rotn(n), norm(sub(cam.C, cam.rot(c)))) < -0.25: continue  # decals wrap the corners: cull only when clearly turned away
        pp = [cam.proj(p) for p in pts3]
        if col == 'shade': col = hexc([v * .80 for v in rgb(body)])
        if kind == 'line':
            polys.append((pp, None, col if col != '#000000' else hexc([v * .55 for v in rgb(body)]), .014))
        elif kind == 'ring':
            polys.append((pp, None, col, .016))
        elif kind == 'tip':
            polys.append((pp, shade(col, cam.rotn(n), cam.L, cam.Hv, .2), '#c9c9c9', .012))
        else:
            polys.append((pp, shade(col, cam.rotn(n), cam.L, cam.Hv, kind or 0), 'same', 0))
    if trim == 'si':             # the wing sits over the deck: draw it last
        for ps, col, n in box(-.76, .76, 1.125, 1.155, .02, .25, body):
            if cam.facing(ps, n): polys.append(([cam.proj(p) for p in ps], shade(col, cam.rotn(n), cam.L, cam.Hv, .25), 'same', 0))
    return polys, rim_layer, dark

def bbox(polys):
    xs = [x for ps, fill, *_ in polys if fill and not str(fill).startswith('rgba') for x, _ in ps]
    ys = [y for ps, fill, *_ in polys if fill and not str(fill).startswith('rgba') for _, y in ps]
    return min(xs), min(ys), max(xs), max(ys)

def to_svg(polys, rim_layer, dark, place, scale, rim=True):
    out = []
    def pts(ps): return ' '.join('%.2f,%.2f' % place(p) for p in ps)
    for ps, fill, stroke, w in polys:
        if fill and fill.startswith('rgba'):
            out.append(f'<polygon points="{pts(ps)}" fill="#000" fill-opacity="{fill.split(",")[-1]}"/>')
    if rim:
        a = .55 if dark else .30
        out.append(f'<g fill="#d8dee6" stroke="#d8dee6" stroke-width="2.2" stroke-linejoin="round" opacity="{a}">' +
                   ''.join(f'<polygon points="{pts(ps)}"/>' for ps, *_ in rim_layer) + '</g>')
    for ps, fill, stroke, w in polys:
        if fill and fill.startswith('rgba'): continue
        if fill:
            st = f' stroke="{fill}" stroke-width="0.6" stroke-linejoin="round"' if stroke == 'same' else \
                 f' stroke="{stroke}" stroke-width="{max(.5, w * scale):.2f}"' if stroke else ''
            out.append(f'<polygon points="{pts(ps)}" fill="{fill}"{st}/>')
        else:
            tag = 'polygon' if w == .016 else 'polyline'
            out.append(f'<{tag} points="{pts(ps)}" fill="none" stroke="{stroke}" ' +
                       f'stroke-width="{max(.5, w * scale):.2f}" stroke-linejoin="round" stroke-linecap="round"/>')
    return ''.join(out)

def civic3d(cx, cy, width, paint='Lunar Silver', trim='ex', yaw=0.0, elev=25.0, dist=9.0):
    """SVG of the car with its bounding box `width` px wide, centred on (cx, cy). Returns (svg, height)."""
    cam = Cam(yaw, elev, dist)
    polys, rim_layer, dark = model(paint, trim, cam)
    x0, y0, x1, y1 = bbox(polys)
    s = width / (x1 - x0); h = (y1 - y0) * s
    place = lambda p: (cx + (p[0] - (x0 + x1) / 2) * s, cy + (p[1] - (y0 + y1) / 2) * s)
    m_per_unit = (x1 - x0) / 1.98     # raw units per metre across the mirrors, for line widths
    return to_svg(polys, rim_layer, dark, place, s * m_per_unit), h

def aspect(elev=25.0, dist=9.0):
    x0, y0, x1, y1 = bbox(model('Lunar Silver', 'ex', Cam(0, elev, dist))[0]); return (y1 - y0) / (x1 - x0)

if __name__ == '__main__':
    import sys
    FONT = 'font-family="Liberation Sans, Arial, sans-serif"'
    if 'check' in sys.argv:      # outline the model over Peter's reference photos
        rear = Cam(0, 0, None)
        polys, rl, dk = model('Platinum White', 'ex', rear)
        place = lambda p: (965 + 775 * p[0], 1335 + 775 * (p[1] - rear.T[1]))
        open('/tmp/hu/mock/chk_rear.svg', 'w').write(
            f'<svg xmlns="http://www.w3.org/2000/svg" width="1920" height="1440">{to_svg(polys, rl, dk, place, 775, rim=False)}</svg>')
        top = Cam(0, 90, None)
        polys, rl, dk = model('Lunar Silver', 'ex', top)
        place = lambda p: (545 + 360 * (LEN / 2 - p[1]), 560 + 360 * p[0])
        open('/tmp/hu/mock/chk_top.svg', 'w').write(
            f'<svg xmlns="http://www.w3.org/2000/svg" width="2800" height="1260">{to_svg(polys, rl, dk, place, 360, rim=False)}</svg>')
        sys.exit()
    svg = ['<svg xmlns="http://www.w3.org/2000/svg" width="1500" height="900"><rect width="100%" height="100%" fill="#050505"/>']
    for i, e in enumerate((15, 22, 30, 40)):
        x = 190 + i * 370
        g, h = civic3d(x, 170, 230, 'Lunar Silver', 'ex', elev=e); svg.append(g)
        svg.append(f'<text x="{x}" y="330" {FONT} font-size="20" fill="#ccc" text-anchor="middle">{e}° above</text>')
    for i, p in enumerate(PAINT):
        x = 90 + i * 165
        g, h = civic3d(x, 470, 130, p, 'ex'); svg.append(g)
        svg.append(f'<text x="{x}" y="570" {FONT} font-size="15" fill="#ccc" text-anchor="middle">{p}</text>')
    for i, (t, p) in enumerate((('ex', 'Lunar Silver'), ('sport', 'Lunar Silver'), ('si', 'Aegean Blue'), ('sport', 'Crystal Black'))):
        x = 190 + i * 370
        g, h = civic3d(x, 690, 150, p, t); svg.append(g)
        svg.append(f'<text x="{x}" y="800" {FONT} font-size="18" fill="#ccc" text-anchor="middle">{t}</text>')
        for j, w in enumerate((46, 32)):
            g, h = civic3d(x + 110 + j * 52, 690, w, p, t); svg.append(g)
    svg.append('</svg>')
    open('/tmp/hu/mock/civic3d.svg', 'w').write(''.join(svg))
