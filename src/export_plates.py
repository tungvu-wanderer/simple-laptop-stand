import sys, collections, struct
import numpy as np
import cadquery as cq
import cq_model as Q
import model as M

out = sys.argv[1]
flat = {}
for k in "AB":
    s = Q.build_plate_local(k)
    flat[k] = s.translate(cq.Vector(0, 0, -s.BoundingBox().zmin))
    cq.exporters.export(flat[k], f"{out}/xstand_plate_{k}.stl", tolerance=0.01, angularTolerance=0.1)
bA, bB = flat["A"].BoundingBox(), flat["B"].BoundingBox()
dy = bA.ymax + 15.0 - bB.ymin
both = cq.Compound.makeCompound([flat["A"], flat["B"].translate(cq.Vector(0, dy, 0))])
cq.exporters.export(both, f"{out}/xstand_plates_AB.stl", tolerance=0.01, angularTolerance=0.1)

# overlap check in the assembled pose
sh = Q.stand_world_shapes()
print("overlap mm3:", sh["A"].intersect(sh["B"]).Volume())


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


for name in ("xstand_plate_A", "xstand_plate_B", "xstand_plates_AB"):
    T = load(f"{out}/{name}.stl")
    key = lambda p: tuple(np.round(p, 3))
    ec = collections.Counter()
    for t in T:
        ks = [key(p) for p in t]
        for i in range(3):
            ec[frozenset((ks[i], ks[(i + 1) % 3]))] += 1
    bad = sum(1 for c in ec.values() if c != 2)
    mn, mx = T.reshape(-1, 3).min(0), T.reshape(-1, 3).max(0)
    print(f"{name}: {len(T)} tris, size {np.round(mx - mn, 1)}, open edges {bad}")
