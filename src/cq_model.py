"""CadQuery build of the X stand plates (chamfers, true round pockets, mitered lips)."""
import math
import numpy as np
import cadquery as cq
import model as M

CH = 0.8             # chamfer on plate-face edges
CH_CORNER = 3.0      # chamfer on outer corners seen in profile
CH_LIP = 1.2
WIN_R = 4.0

# plate frame: local X = along arm (u), local Y = height (v), local Z = across thickness (w')
# world rotation: rotate about X by 90 (Y->Z, Z->-Y), then about Z by phi
PL = {
    "A": dict(phi=45.0, sigma=-1, color=(224, 131, 48)),
    "B": dict(phi=135.0, sigma=+1, color=(52, 132, 168)),
}
for k, p in PL.items():
    r = math.radians(p["phi"])
    p["d"] = np.array([math.cos(r), math.sin(r)])
    p["e"] = np.array([math.sin(r), -math.cos(r)])


def to_world(k, u, v, w):
    p = PL[k]
    xy = (u - M.L / 2) * p["d"] + w * p["e"]
    return np.array([xy[0], xy[1], v + M.ZB])


def vec_world(k, nu, nv, nw):
    p = PL[k]
    xy = nu * p["d"] + nw * p["e"]
    return np.array([xy[0], xy[1], nv])


def lip_normal(k):
    """Outward (toward laptop) normal of the mitered lip face, in local (u,v,w')."""
    s, sg = M.S, PL[k]["sigma"]
    n = np.array([math.cos(s), math.sin(s), sg * math.cos(s)])
    return n / np.linalg.norm(n)


def _plane(origin, normal):
    n = np.asarray(normal, float)
    n /= np.linalg.norm(n)
    ref = np.array([0.0, 0.0, 1.0]) if abs(n[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    x = np.cross(n, ref)
    x /= np.linalg.norm(x)
    return cq.Plane(origin=tuple(origin), xDir=tuple(x), normal=tuple(n))


def _poly_prism(pts, z0, z1):
    return cq.Workplane("XY").workplane(offset=z0).polyline(pts).close().extrude(z1 - z0)


def build_plate_local(k, chamfer=True, pockets=True):
    L, TH, hf = M.L, M.TH, M.HF
    top, N = M.top, M.NPROF
    # lip body is built TH/2 deeper than the miter plane needs, then cut back to the mitered face
    u3 = M.LIP_T + TH / 2
    p3s = np.array([u3, top(u3)])
    p4s = p3s + M.LIP_H * N
    pts = [(0, 0), (L, 0), (L, top(L)), tuple(p3s), tuple(p4s), (0, p4s[1])]
    um, hw = L / 2, M.SLOT / 2
    from shapely.geometry import Polygon, box
    poly = Polygon(pts)
    if k == "A":
        poly = poly.difference(box(um - hw, -1, um + hw, M.SLOT_DEPTH))
    else:
        poly = poly.difference(box(um - hw, M.HC - M.SLOT_DEPTH, um + hw, M.HC + 20))
    wp = cq.Workplane("XY").workplane(offset=-TH / 2)
    solid = wp.polyline(list(poly.exterior.coords)[:-1]).close().extrude(TH)

    m = 12.0
    wins = [
        [(M.LIP_T + 16, m), (um - hw - 14, m), (um - hw - 14, top(um - hw - 14) - m), (M.LIP_T + 16, top(M.LIP_T + 16) - m)],
        [(um + hw + 14, m), (L - 14, m), (L - 14, top(L - 14) - m), (um + hw + 14, top(um + hw + 14) - m)],
    ]
    for w in wins:
        cutter = _poly_prism(w, -TH, TH).edges("|Z").fillet(WIN_R)
        solid = solid.cut(cutter)

    # mitered lip face
    nl = lip_normal(k)
    p3 = np.array([M.LIP_T, top(M.LIP_T), 0.0])
    t1 = cq.Workplane(_plane(p3, nl)).rect(400, 400).extrude(200)
    t2 = cq.Workplane(_plane(p3, [N[0], N[1], 0.0])).rect(400, 400).extrude(200)
    solid = solid.cut(t1.intersect(t2))

    shape = solid.val()
    if chamfer:
        shape = _chamfer(shape, k, p3, nl)
    solid = cq.Workplane("XY").add(shape)
    if pockets:
        solid = _pockets(solid, k, p3, nl)
    return solid.val()


def _close(a, b, tol=1e-3):
    return abs(a - b) < tol


def _chamfer(shape, k, p3, nl):
    TH = M.TH
    # 1) corner chamfers on vertical (through-thickness) edges seen in profile
    targets = [((0.0, 0.0), M.CH_C if hasattr(M, "CH_C") else CH_CORNER),
               ((M.L, 0.0), CH_CORNER),
               ((M.L, M.top(M.L)), CH_CORNER),
               ((0.0, M.top(M.LIP_T + TH / 2) + M.LIP_H * M.NPROF[1]), CH_LIP)]
    for (x, y), size in targets:
        es = []
        for e in shape.Edges():
            a, b = e.startPoint().toTuple(), e.endPoint().toTuple()
            if _close(a[0], x) and _close(b[0], x) and _close(a[1], y) and _close(b[1], y) and abs(a[2] - b[2]) > TH - 1e-3:
                es.append(e)
        if es:
            shape = shape.chamfer(size, None, es)

    # 2) chamfer plate-face edges, leaving pad-bearing edges square
    es = []
    for e in shape.Edges():
        a, b = np.array(e.startPoint().toTuple()), np.array(e.endPoint().toTuple())
        if not (_close(abs(a[2]), TH / 2) and _close(abs(b[2]), TH / 2) and a[2] * b[2] > 0):
            continue
        on_slope = _close(a[1], M.top(a[0])) and _close(b[1], M.top(b[0]))
        on_bottom = _close(a[1], 0.0) and _close(b[1], 0.0)
        on_face = _close((a - p3) @ nl, 0.0) and _close((b - p3) @ nl, 0.0)
        if on_slope or on_bottom or on_face:
            continue
        es.append(e)
    return shape.chamfer(CH, None, es)


def _pockets(solid, k, p3, nl):
    r = M.POCKET_D / 2
    N = M.NPROF
    cuts = []
    for u in M.TOP_PADS_U:
        cuts.append(((u, M.top(u), 0.0), (N[0], N[1], 0.0)))
    for u in M.BOT_PADS_U:
        cuts.append(((u, 0.0, 0.0), (0.0, -1.0, 0.0)))
    c = p3 + M.LIP_PAD_C * np.array([N[0], N[1], 0.0])
    cuts.append((tuple(c), tuple(nl)))
    for origin, axis in cuts:
        cyl = cq.Workplane(_plane(origin, axis)).circle(r).extrude(M.POCKET_DEPTH, both=True)
        solid = solid.cut(cyl)
    return solid


def to_world_shape(k, shape):
    p = PL[k]
    s = shape.translate(cq.Vector(-M.L / 2, 0, 0))
    s = s.rotate(cq.Vector(0, 0, 0), cq.Vector(1, 0, 0), 90)
    s = s.rotate(cq.Vector(0, 0, 0), cq.Vector(0, 0, 1), p["phi"])
    return s.translate(cq.Vector(0, 0, M.ZB))


def tessellate(shape, tol=0.04, ang=0.15):
    verts, tris = shape.tessellate(tol, ang)
    V = np.array([v.toTuple() for v in verts])
    T = V[np.array(tris)]
    n = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0])
    ln = np.linalg.norm(n, axis=1)
    ok = ln > 1e-12
    return T[ok], n[ok] / ln[ok, None]


def pad_specs(k):
    out = []
    for u in M.BOT_PADS_U:
        out.append((to_world(k, u, 0.0, 0.0), np.array([0, 0, -1.0])))
    N = M.NPROF
    for u in M.TOP_PADS_U:
        out.append((to_world(k, u, M.top(u), 0.0), vec_world(k, N[0], N[1], 0.0)))
    nl = lip_normal(k)
    p3 = np.array([M.LIP_T, M.top(M.LIP_T)])
    ctr = p3 + M.LIP_PAD_C * N
    out.append((to_world(k, ctr[0], ctr[1], 0.0), vec_world(k, *nl)))
    return out


def stand_world_shapes(**kw):
    return {k: to_world_shape(k, build_plate_local(k, **kw)) for k in "AB"}


def stand_mesh(shapes=None, pads=True):
    shapes = shapes or stand_world_shapes()
    Ts, Ns, Cs = [], [], []
    for k, sh in shapes.items():
        t, n = tessellate(sh)
        Ts.append(t); Ns.append(n); Cs += [PL[k]["color"]] * len(t)
        for c, ax in (pad_specs(k) if pads else []):
            ct, cn = M.cylinder(c, ax)
            Ts.append(ct); Ns.append(cn); Cs += [(38, 38, 42)] * len(ct)
    return np.concatenate(Ts), np.concatenate(Ns), np.array(Cs, float)
