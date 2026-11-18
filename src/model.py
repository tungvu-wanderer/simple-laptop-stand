"""Parametric X laptop stand preview: geometry + tiny numpy z-buffer renderer."""
import math
import numpy as np
import mapbox_earcut as earcut
from shapely.geometry import Polygon, box
from shapely.geometry.polygon import orient

# ---- parameters (mm / degrees) -------------------------------------------
TH = 6.0            # plate thickness
L = 250.0           # arm length
HF = 30.0           # arm height at the front end (under the slope)
REAR = 100.0        # arm height at the rear end
LIP_H = 12.0        # front lip height above slope, measured along slope normal
LIP_T = 10.0        # lip block depth along the arm
SLOT = 5.80         # slot width (printer: slots print +0.15 wide, Z thickness -0.15)
SLOT_EXTRA = 0.5    # extra depth per slot so plates reach the table
ZB = 1.0            # table-pad protrusion (plates sit this far above z=0)
PAD_D, PAD_T, PAD_SINK = 6.0, 2.0, 1.0
LAP_W, LAP_D, LAP_H, LID_T = 313.0, 221.2, 15.5, 4.0
LEAN = 12.0         # screen back-lean from vertical used for the preview
POCKET_D, POCKET_DEPTH = 6.2, 1.0

TS = (REAR - HF) / L                                           # arm slope (tan)
S = math.atan(TS)
TILT = math.degrees(math.atan(TS / math.sqrt(0.5)))            # laptop plane tilt
HINGE = 90.0 + TILT + LEAN                                     # lid opening angle
NPROF = np.array([-math.sin(S), math.cos(S)])                  # slope normal (u,v)


TOP_PADS_U = (55.0, L - 30.0)
BOT_PADS_U = (25.0, L - 25.0)


LIP_PAD_C = 6.0                    # pad centre along the lip face from the slope
_nu = 1 / math.sqrt(1 + math.cos(S) ** 2) * math.cos(S)       # u-component of the mitered face normal
LIP_POCKET_U = POCKET_DEPTH / _nu                               # u offset giving 1 mm depth along the normal


def top(u):
    return HF + u * TS


HC = top(L / 2)                    # arm height at the crossing
SLOT_DEPTH = HC / 2 + SLOT_EXTRA


def profile(kind, pockets=True):
    p3 = np.array([LIP_T, top(LIP_T)])
    p4 = p3 + LIP_H * NPROF
    outer = Polygon([(0, 0), (L, 0), (L, top(L)), tuple(p3), tuple(p4), (0, p4[1])])
    um, hw = L / 2, SLOT / 2
    if kind == "A":   # slot opens from the bottom
        outer = outer.difference(box(um - hw, -1, um + hw, SLOT_DEPTH))
    else:             # slot opens from the sloped top
        outer = outer.difference(box(um - hw, HC - SLOT_DEPTH, um + hw, HC + 20))
    m = 12.0
    wins = [
        [(LIP_T + 16, m), (um - hw - 14, m), (um - hw - 14, top(um - hw - 14) - m), (LIP_T + 16, top(LIP_T + 16) - m)],
        [(um + hw + 14, m), (L - 14, m), (L - 14, top(L - 14) - m), (um + hw + 14, top(um + hw + 14) - m)],
    ]
    for w in wins:
        outer = outer.difference(Polygon(w).buffer(-4).buffer(4))
    tdir = np.array([math.cos(S), math.sin(S)])
    hp = POCKET_D / 2
    if not pockets:
        return orient(outer, 1.0), p3, p4
    for u in TOP_PADS_U:                                       # slope pockets
        c = np.array([u, top(u)])
        pts = [c - hp * tdir, c + hp * tdir, c + hp * tdir - POCKET_DEPTH * NPROF, c - hp * tdir - POCKET_DEPTH * NPROF]
        outer = outer.difference(Polygon([tuple(p) for p in pts]))
    for u in BOT_PADS_U:                                       # foot pockets
        outer = outer.difference(box(u - hp, -1, u + hp, POCKET_DEPTH))
    a1, a2 = p3 + (LIP_PAD_C - hp) * NPROF, p3 + (LIP_PAD_C + hp) * NPROF
    du = np.array([LIP_POCKET_U, 0.0])                         # lip pocket, floor parallel to the face
    outer = outer.difference(Polygon([tuple(a1), tuple(a2), tuple(a2 - du), tuple(a1 - du)]))
    return orient(outer, 1.0), p3, p4


# ---- plate mesh ----------------------------------------------------------
PLATES = {
    "A": dict(d=np.array([1, 1]) / math.sqrt(2), e=np.array([-1, 1]) / math.sqrt(2), sigma=+1, color=(224, 131, 48)),
    "B": dict(d=np.array([-1, 1]) / math.sqrt(2), e=np.array([-1, -1]) / math.sqrt(2), sigma=-1, color=(52, 132, 168)),
}


def to_world(pl, u, v, w):
    xy = (np.asarray(u)[..., None] - L / 2) * pl["d"] + np.asarray(w)[..., None] * pl["e"]
    xy = xy + np.array([0.0, 0.0])
    return np.concatenate([xy, np.asarray(v)[..., None] + ZB], axis=-1)


def vec_world(pl, nu, nv, nw):
    xy = nu * pl["d"] + nw * pl["e"]
    return np.array([xy[0], xy[1], nv])


def plate_mesh(kind):
    pl = PLATES[kind]
    poly, p3, p4 = profile(kind)
    rings = [np.array(poly.exterior.coords)[:-1]] + [np.array(r.coords)[:-1] for r in poly.interiors]
    seg = p4 - p3
    segl2 = seg @ seg

    def on_face(pt):
        for off in (0.0, LIP_POCKET_U):
            q = p3 - np.array([off, 0.0])
            t = ((pt - q) @ seg) / segl2
            d = np.linalg.norm(q + t * seg - pt)
            if d < 1e-6 and -1e-6 <= t <= 1 + 1e-6:
                return True
        return False

    tris, norms = [], []
    verts2 = np.vstack(rings)
    ends = np.cumsum([len(r) for r in rings]).astype(np.uint32)
    idx = np.array(earcut.triangulate_float64(verts2, ends)).reshape(-1, 3)

    def pos(pt, w):
        u = pt[0] - (pl["sigma"] * w if on_face(pt) else 0.0)
        return to_world(pl, u, pt[1], w)

    for w, nw in ((TH / 2, 1.0), (-TH / 2, -1.0)):
        for tri in idx:
            tris.append([pos(verts2[i], w) for i in tri])
            norms.append(vec_world(pl, 0, 0, nw))
    for ring in rings:
        n = len(ring)
        for i in range(n):
            a, b = ring[i], ring[(i + 1) % n]
            qa, qb = pos(a, TH / 2), pos(b, TH / 2)
            ra, rb = pos(a, -TH / 2), pos(b, -TH / 2)
            out2 = np.array([b[1] - a[1], -(b[0] - a[0])])
            for t in ([qa, qb, rb], [qa, rb, ra]):
                nrm = np.cross(t[1] - t[0], t[2] - t[0])
                if np.linalg.norm(nrm) < 1e-9:
                    continue
                nrm = nrm / np.linalg.norm(nrm)
                if nrm[:2] @ vec_world(pl, out2[0], out2[1], 0)[:2] < 0:
                    nrm = -nrm
                tris.append(t)
                norms.append(nrm)
    return np.array(tris), np.array(norms), pl["color"]


def extrude(shape, O, A1, A2, E, color):
    """Extrude a 2D shapely shape drawn in the (A1, A2) plane at O along E."""
    T, N = [], []
    geoms = shape.geoms if hasattr(shape, "geoms") else [shape]
    nrm = np.cross(A1, A2)
    P = lambda p, k: O + p[0] * A1 + p[1] * A2 + k * E
    for g in geoms:
        if g.area < 1e-6:
            continue
        g = orient(g, 1.0)
        rings = [np.array(g.exterior.coords)[:-1]] + [np.array(r.coords)[:-1] for r in g.interiors]
        verts = np.vstack(rings)
        ends = np.cumsum([len(r) for r in rings]).astype(np.uint32)
        idx = np.array(earcut.triangulate_float64(verts, ends)).reshape(-1, 3)
        for k in (0, 1):
            for tri in idx:
                T.append([P(verts[i], k) for i in tri]); N.append(nrm)
        for ring in rings:
            n = len(ring)
            for i in range(n):
                a, b = ring[i], ring[(i + 1) % n]
                q = [P(a, 0), P(b, 0), P(b, 1), P(a, 1)]
                sn = np.cross(q[1] - q[0], q[3] - q[0])
                if np.linalg.norm(sn) < 1e-9:
                    continue
                sn = sn / np.linalg.norm(sn)
                T += [[q[0], q[1], q[2]], [q[0], q[2], q[3]]]; N += [sn, sn]
    return np.array(T), np.array(N), [color] * len(T)


def corner_fillers():
    """box minus circle: the pocket's remaining curved walls (box = pocket length x plate width)."""
    from shapely.geometry import Point
    hp = POCKET_D / 2
    return box(-hp, -TH / 2, hp, TH / 2).difference(Point(0, 0).buffer(hp, 32))


def filler_mesh(kind):
    pl = PLATES[kind]
    col = pl["color"]
    shape = corner_fillers()
    out = []
    tdir = vec_world(pl, math.cos(S), math.sin(S), 0)
    wdir = vec_world(pl, 0, 0, 1)
    nprof = vec_world(pl, NPROF[0], NPROF[1], 0)
    for u in TOP_PADS_U:
        O = to_world(pl, u, top(u), 0.0)
        out.append(extrude(shape, O, tdir, wdir, -POCKET_DEPTH * nprof, col))
    for u in BOT_PADS_U:
        O = to_world(pl, u, 0.0, 0.0)
        out.append(extrude(shape, O, vec_world(pl, 1, 0, 0), wdir, np.array([0, 0, POCKET_DEPTH]), col))
    # lip pocket: shape lives in the mitered face plane
    poly, p3, p4 = profile(kind)
    sg = pl["sigma"]
    e1 = np.array([*(p4 - p3), 0.0])
    e2 = np.array([-sg * TH, 0.0, TH])
    nl = np.cross(e1, e2)
    nl /= np.linalg.norm(nl)
    if nl[0] < 0:
        nl = -nl
    ctr = p3 + LIP_PAD_C * NPROF
    O = to_world(pl, ctr[0], ctr[1], 0.0)
    A1 = vec_world(pl, NPROF[0], NPROF[1], 0)
    g = np.array([-sg, 0.0, 1.0]) / math.sqrt(2)
    A2 = vec_world(pl, g[0], g[1], g[2])
    Wd = TH * math.sqrt(2)
    hp = POCKET_D / 2
    from shapely.geometry import Point
    lipshape = box(-hp, -Wd / 2, hp, Wd / 2).difference(Point(0, 0).buffer(hp, 32))
    En = -vec_world(pl, nl[0], nl[1], nl[2]) * POCKET_DEPTH
    out.append(extrude(lipshape, O, A1, A2, En, col))
    T = np.concatenate([o[0] for o in out]); N = np.concatenate([o[1] for o in out])
    return T, N, col


def cylinder(center, axis, r=PAD_D / 2, h=PAD_T, n=28):
    axis = axis / np.linalg.norm(axis)
    a = np.cross(axis, [0, 0, 1.0])
    if np.linalg.norm(a) < 1e-6:
        a = np.cross(axis, [1.0, 0, 0])
    a /= np.linalg.norm(a)
    b = np.cross(axis, a)
    tris, norms = [], []
    for i in range(n):
        t0, t1 = 2 * math.pi * i / n, 2 * math.pi * (i + 1) / n
        r0 = r * (math.cos(t0) * a + math.sin(t0) * b)
        r1 = r * (math.cos(t1) * a + math.sin(t1) * b)
        lo, hi = center - axis * h / 2, center + axis * h / 2
        tris += [[lo + r0, lo + r1, hi + r1], [lo + r0, hi + r1, hi + r0], [hi, hi + r0, hi + r1], [lo, lo + r1, lo + r0]]
        mid = (math.cos((t0 + t1) / 2) * a + math.sin((t0 + t1) / 2) * b)
        norms += [mid, mid, axis, -axis]
    return np.array(tris), np.array(norms)


def pads(kind):
    pl = PLATES[kind]
    out = []
    # table side: 2 per arm end
    for u in BOT_PADS_U:
        out.append((to_world(pl, u, 0.0, 0.0), np.array([0, 0, -1.0])))
    # laptop side: 1 per arm on the slope
    for u in TOP_PADS_U:
        c = to_world(pl, u, top(u), 0.0)
        out.append((c, np.array([*(NPROF[0] * pl["d"]), NPROF[1]])))
    # lip pad: on the mitered face
    poly, p3, p4 = profile(kind)
    sg = pl["sigma"]
    a0 = np.array([p3[0], p3[1], 0.0])
    e1 = np.array([*(p4 - p3), 0.0])
    e2 = np.array([-sg * TH, 0.0, TH])
    nl = np.cross(e1, e2)
    nl /= np.linalg.norm(nl)
    if nl[0] < 0:
        nl = -nl
    ctr = p3 + LIP_PAD_C * NPROF
    out.append((to_world(pl, ctr[0], ctr[1], 0.0), vec_world(pl, nl[0], nl[1], nl[2])))
    return out


def stand_mesh():
    T, N, C = [], [], []
    for k in "AB":
        t, n, col = plate_mesh(k)
        T.append(t); N.append(n); C += [col] * len(t)
        ft, fn, fc = filler_mesh(k)
        T.append(ft); N.append(fn); C += [fc] * len(ft)
        for c, ax in pads(k):
            ct, cn = cylinder(c, ax)
            T.append(ct); N.append(cn); C += [(38, 38, 42)] * len(ct)
    return np.concatenate(T), np.concatenate(N), np.array(C, dtype=float)


# ---- laptop ghost --------------------------------------------------------
def laptop_mesh():
    th = math.radians(TILT)
    x = np.array([1.0, 0, 0])
    p = np.array([0, math.cos(th), math.sin(th)])
    n = np.array([0, -math.sin(th), math.cos(th)])
    y_f = -106.07 + LIP_T * math.sqrt(0.5) + 0.5
    zf = ZB + HF + (y_f + 106.07) * math.tan(th) + (PAD_T - PAD_SINK)
    F = np.array([-LAP_W / 2, y_f, zf])

    def slab(o, a, b, c, da, db, dc):
        P = lambda i, j, k: o + a * da * i + b * db * j + c * dc * k
        quads = [
            ([P(0, 0, 0), P(1, 0, 0), P(1, 1, 0), P(0, 1, 0)], -c),
            ([P(0, 0, 1), P(1, 0, 1), P(1, 1, 1), P(0, 1, 1)], c),
            ([P(0, 0, 0), P(1, 0, 0), P(1, 0, 1), P(0, 0, 1)], -b),
            ([P(0, 1, 0), P(1, 1, 0), P(1, 1, 1), P(0, 1, 1)], b),
            ([P(0, 0, 0), P(0, 1, 0), P(0, 1, 1), P(0, 0, 1)], -a),
            ([P(1, 0, 0), P(1, 1, 0), P(1, 1, 1), P(1, 0, 1)], a),
        ]
        T, N = [], []
        for q, nn in quads:
            T += [[q[0], q[1], q[2]], [q[0], q[2], q[3]]]
            N += [nn, nn]
        return T, N

    T, N = slab(F, x, p, n, LAP_W, LAP_D, LAP_H)
    cols = [(170, 174, 182)] * len(T)
    al = math.radians(HINGE)
    l = math.cos(al) * (-p) + math.sin(al) * n
    m = -math.sin(al) * p - math.cos(al) * n
    H = F + LAP_D * p + LAP_H * n
    T2, N2 = slab(H, x, l, -m, LAP_W, LAP_D, LID_T)
    T += T2; N += N2
    cols += [(120, 126, 138)] * len(T2)
    return np.array(T), np.array(N), np.array(cols, dtype=float)


def floor_mesh():
    s = 700.0
    q = [np.array(v, dtype=float) for v in ((-s, -s, 0), (s, -s, 0), (s, s, 0), (-s, s, 0))]
    n = np.array([0, 0, 1.0])
    return np.array([[q[0], q[1], q[2]], [q[0], q[2], q[3]]]), np.array([n, n]), np.array([(236, 232, 224)] * 2, dtype=float)


# ---- renderer ------------------------------------------------------------
def camera(phi, el):
    phi, el = math.radians(phi), math.radians(el)
    c = np.array([math.sin(phi) * math.cos(el), -math.cos(phi) * math.cos(el), math.sin(el)])
    r = np.array([math.cos(phi), math.sin(phi), 0.0])
    u = np.array([-math.sin(phi) * math.sin(el), math.cos(phi) * math.sin(el), math.cos(el)])
    return r, u, c


def render(layers, phi, el, W, H, fit_layers, margin=30, ss=2, bg=(244, 243, 240), floor=True):
    """layers: list of (T,N,C). Returns (rgb, coverage) float arrays at W,H."""
    r, u, c = camera(phi, el)
    light = c * 0.8 + u * 0.55 - r * 0.45
    light /= np.linalg.norm(light)
    fit = np.concatenate([t.reshape(-1, 3) for t, _, _ in fit_layers])
    sx, sy = fit @ r, fit @ u
    sc = min((W - 2 * margin) / (sx.max() - sx.min()), (H - 2 * margin) / (sy.max() - sy.min()))
    ox = W / 2 - sc * (sx.max() + sx.min()) / 2
    oy = H / 2 + sc * (sy.max() + sy.min()) / 2
    Ws, Hs = W * ss, H * ss
    img = np.zeros((Hs, Ws, 3))
    img[:] = bg
    zb = np.full((Hs, Ws), -1e18)
    cov = np.zeros((Hs, Ws), bool)
    for T, N, C in layers:
        P = T.reshape(-1, 3)
        X = (P @ r * sc + ox).reshape(-1, 3) * ss
        Y = (oy - P @ u * sc).reshape(-1, 3) * ss
        Z = (P @ c).reshape(-1, 3)
        shade = 0.38 + 0.62 * np.abs(N @ light)
        for i in range(len(T)):
            x, y, z = X[i], Y[i], Z[i]
            x0, x1 = int(max(math.floor(x.min()), 0)), int(min(math.ceil(x.max()), Ws - 1))
            y0, y1 = int(max(math.floor(y.min()), 0)), int(min(math.ceil(y.max()), Hs - 1))
            if x1 < x0 or y1 < y0:
                continue
            den = (y[1] - y[2]) * (x[0] - x[2]) + (x[2] - x[1]) * (y[0] - y[2])
            if abs(den) < 1e-9:
                continue
            gx, gy = np.meshgrid(np.arange(x0, x1 + 1) + 0.5, np.arange(y0, y1 + 1) + 0.5)
            w0 = ((y[1] - y[2]) * (gx - x[2]) + (x[2] - x[1]) * (gy - y[2])) / den
            w1 = ((y[2] - y[0]) * (gx - x[2]) + (x[0] - x[2]) * (gy - y[2])) / den
            w2 = 1 - w0 - w1
            m = (w0 >= -1e-9) & (w1 >= -1e-9) & (w2 >= -1e-9)
            if not m.any():
                continue
            zz = w0 * z[0] + w1 * z[1] + w2 * z[2]
            sub = zb[y0:y1 + 1, x0:x1 + 1]
            upd = m & (zz > sub)
            sub[upd] = zz[upd]
            img[y0:y1 + 1, x0:x1 + 1][upd] = C[i] * shade[i]
            cov[y0:y1 + 1, x0:x1 + 1] |= upd
    out = img.reshape(H, ss, W, ss, 3).mean(axis=(1, 3))
    cv = cov.reshape(H, ss, W, ss).mean(axis=(1, 3))
    return out, cv
