"""Stand parameters, flat plate outline, laptop ghost and a tiny numpy z-buffer renderer for the previews."""
import math
import numpy as np
from shapely.geometry import Polygon, box
from shapely.geometry.polygon import orient

# ---- parameters (mm / degrees) -------------------------------------------
TH = 6.0            # plate thickness
L = 250.0           # arm length
HF = 30.0           # arm height at the front end (under the slope)
REAR = 130.0        # arm height at the rear end
LIP_H = 15.0        # front lip height above slope, measured along slope normal
LIP_T = 13.0        # lip block depth along the arm
SLOT = 5.80         # slot width (printer: slots print +0.15 wide, Z thickness -0.15)
SLOT_EXTRA = 0.15   # extra depth per slot; the two slot tips leave a 2 x this gap so each plate stands on its own feet
ZB = 1.0            # table-pad protrusion (plates sit this far above z=0)
PAD_D, PAD_T, PAD_SINK = 6.0, 2.0, 1.0
LAP_W, LAP_D, LAP_H, LID_T = 313.0, 221.2, 15.5, 4.0
HINGE = 123.6       # lid opening angle assumed by the preview
POCKET_D, POCKET_DEPTH = 6.2, 1.0

TS = (REAR - HF) / L                                           # arm slope (tan)
S = math.atan(TS)
TILT = math.degrees(math.atan(TS / math.sqrt(0.5)))            # laptop plane tilt
NPROF = np.array([-math.sin(S), math.cos(S)])                  # slope normal (u,v)


TOP_PADS_U = (55.0, L - 30.0)
BOT_PADS_U = (25.0, L - 25.0)


LIP_PAD_C = 9.0                    # pad centre along the lip face from the slope


def top(u):
    return HF + u * TS


HC = top(L / 2)                    # arm height at the crossing
SLOT_DEPTH = HC / 2 + SLOT_EXTRA


def profile(kind):
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
    return orient(outer, 1.0), p3, p4


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


# ---- laptop ghost --------------------------------------------------------
def laptop_mesh():
    th = math.radians(TILT)
    x = np.array([1.0, 0, 0])
    p = np.array([0, math.cos(th), math.sin(th)])
    n = np.array([0, -math.sin(th), math.cos(th)])
    y_f = -L / 2 * math.sqrt(0.5) + LIP_T * math.sqrt(0.5) + 0.5
    zf = ZB + HF + (y_f + L / 2 * math.sqrt(0.5)) * math.tan(th) + (PAD_T - PAD_SINK)
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
