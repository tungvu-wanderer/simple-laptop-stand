import sys, collections, struct
import numpy as np
import cadquery as cq
import cq_model as Q
import model as M

out = sys.argv[1]
TH = M.TH

def flat_for_print(shape):
    bb = shape.BoundingBox()
    return shape.translate(cq.Vector(0, 0, -bb.zmin))   # thickness on the bed, profile in XY

# plates: local frame already has the profile in XY and thickness along Z
for k in "AB":
    s = flat_for_print(Q.build_plate_local(k))
    cq.exporters.export(s, f"{out}/xstand_plate_{k}.stl", tolerance=0.01, angularTolerance=0.1)

# slot-fit coupon: slotted block with +0.2/+0.3/+0.4 mm slots, plus a 6 mm tongue plate
slots = [TH + 0.2, TH + 0.3, TH + 0.4]
W, H, DEPTH, GAP = 15.0 * 3 + 10, 25.0, 15.0, 15.0
blk = cq.Workplane("XY").box(W, H, TH, centered=False)
for i, sw in enumerate(slots):
    cx = 10 + i * 17.5
    blk = blk.cut(cq.Workplane("XY").box(sw, DEPTH + 1, TH + 2, centered=False).translate((cx, -1, -1)))
def chamfer_caps(shape, c=0.8):
    es = [e for e in shape.Edges()
          if (abs(e.startPoint().z) < 1e-6 and abs(e.endPoint().z) < 1e-6)
          or (abs(e.startPoint().z - TH) < 1e-6 and abs(e.endPoint().z - TH) < 1e-6)]
    return shape.chamfer(c, None, es)

tongue = cq.Workplane("XY").box(30.0, 20.0, TH, centered=False).translate((0, H + GAP, 0))
coupon = cq.Compound.makeCompound([chamfer_caps(blk.val()), chamfer_caps(tongue.val())])
cq.exporters.export(coupon, f"{out}/xstand_slot_coupon.stl", tolerance=0.01, angularTolerance=0.1)

# ---- verify exports -------------------------------------------------------
def load(p):
    d = open(p, "rb").read()
    if d[:5] == b"solid":
        import re
        v = np.array(re.findall(rb"vertex\s+(\S+)\s+(\S+)\s+(\S+)", d), dtype=float)
    else:
        n = struct.unpack("<I", d[80:84])[0]
        a = np.frombuffer(d[84:], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]), count=n)
        v = a["v"].reshape(-1, 3).astype(float)
    return v.reshape(-1, 3, 3)

for name in ("xstand_plate_A", "xstand_plate_B", "xstand_slot_coupon"):
    T = load(f"{out}/{name}.stl")
    key = lambda p: tuple(np.round(p, 3))
    ec = collections.Counter()
    for t in T:
        ks = [key(p) for p in t]
        for i in range(3):
            ec[frozenset((ks[i], ks[(i + 1) % 3]))] += 1
    bad = sum(1 for c in ec.values() if c != 2)
    mn, mx = T.reshape(-1, 3).min(0), T.reshape(-1, 3).max(0)
    # signed volume
    vol = np.einsum("ij,ij->i", T[:, 0], np.cross(T[:, 1], T[:, 2])).sum() / 6
    print(f"{name}: {len(T)} tris, size {np.round(mx - mn, 1)}, z {mn[2]:.2f}..{mx[2]:.2f}, open/non-manifold edges {bad}, volume {abs(vol)/1000:.1f} cm3")
