import sys
import cadquery as cq
import u_model as U

out = sys.argv[1]
plate = U.build_plate()
arm = {k: U.build_arm_local(k) for k in "AB"}
world = {k: U.to_world_arm(k, arm[k]) for k in "AB"}

# printed in place: the open pose, standing on the bottom edges
stand = cq.Compound.makeCompound([plate, world["A"], world["B"]])
cq.exporters.export(stand, f"{out}/ustand.stl", tolerance=0.01, angularTolerance=0.1)
for name, shape in (("plate", plate), ("arm A", world["A"]), ("arm B", world["B"])):
    print(f"{name}: volume {shape.Volume() / 1000:.1f} cm3, valid {shape.isValid()}")
print(f"gaps in the open pose: plate-A {plate.distance(world['A']):.2f}, plate-B {plate.distance(world['B']):.2f}, A-B {world['A'].distance(world['B']):.1f} mm")


def overlap(a, b):
    return a.intersect(b).Volume()


def check(label, *pairs):
    print(label + ": " + ", ".join(f"{n} {overlap(x, y):.2f}" for n, x, y in pairs) + " mm3")


# arm A folds first, then arm B stacks over it
check("open", ("plate-A", plate, world["A"]), ("plate-B", plate, world["B"]), ("A-B", world["A"], world["B"]))
a_done = U.fold("A", world["A"])
for frac in (0.25, 0.5, 0.75):
    a = U.fold("A", world["A"], frac)
    check(f"A swinging {frac:.2f}", ("plate-A", plate, a), ("A-B", a, world["B"]))
for frac in (0.25, 0.5, 0.75, 1.0):
    b = U.fold("B", world["B"], frac)
    check(f"B swinging {frac:.2f}", ("plate-B", plate, b), ("A-B", a_done, b))
# the stop: each arm pushed 4 degrees past open must hit the plate (0.3 mm clearance closes at about 3 degrees)
for k in "AB":
    check(f"stop {k} at -4 deg", ("plate", plate, U.fold(k, world[k], -4 / 90)))

stack = cq.Compound.makeCompound([plate, U.fold("A", world["A"]), U.fold("B", world["B"])]).BoundingBox()
print(f"folded stack: {stack.xlen:.1f} x {stack.zlen:.1f} x {stack.ylen:.1f} mm (width x height x thickness)")
print(f"open: {stand.BoundingBox().xlen:.1f} wide x {stand.BoundingBox().ylen:.1f} deep x {stand.BoundingBox().zlen:.1f} high")

# ---- flat top: every body's top follows the arms' top plane --------------------
def top_z(shape, y):
    slab = cq.Workplane("XY").add(cq.Solid.makeBox(U.W + 40, 0.1, 400, cq.Vector(-20, y - 0.05, 0)))
    hit = shape.intersect(slab.val())
    return hit.BoundingBox().zmax if hit.Volume() > 1e-6 else None


def planar_residual(samples):
    return max(abs(z - U.plane_z(y)) for _, y, z in samples)


samples = []
for name, shape, ys in (("A", world["A"], (-170, -120, -60, -20, 3)), ("B", world["B"], (-170, -120, -60, -20, -3)),
                        ("plate", plate, (-3, 1, 3, 5))):
    for y in ys:
        z = top_z(shape, y)
        if z is not None:
            samples.append((name, y, z))
for name, y, z in samples:
    print(f"  top {name:5s} y={y:5.0f}: z={z:7.2f}, plane {U.plane_z(y):7.2f}")
print(f"flat top: max deviation from the arms' plane {planar_residual(samples):.3f} mm over {len(samples)} samples")
ya, yb = -U.Y0 + U.LIP_T + 1, U.TH + 1
wedge = (cq.Workplane("YZ").workplane(offset=-20)
         .polyline([(ya, U.plane_z(ya) + 0.01), (yb, U.plane_z(yb) + 0.01), (yb, 400), (ya, 400)]).close().extrude(U.W + 40)).val()
print(f"material above the plane behind the lip: {stand.intersect(wedge).Volume():.3f} mm3")

# ---- honeycomb ---------------------------------------------------------------
import math
from shapely.geometry import Polygon
from shapely.geometry.polygon import orient


def hexagon(c):
    return Polygon([(c[0] + U.HEX_S * math.cos(math.radians(60 * k)), c[1] + U.HEX_S * math.sin(math.radians(60 * k)))
                    for k in range(6)])


def ceilings(poly):
    """(angle from horizontal, length) of each void edge with solid above it; the polygon is counter-clockwise, so those run right to left."""
    pts = list(orient(poly, 1.0).exterior.coords)
    return [(math.degrees(math.atan2(abs(by - ay), abs(bx - ax))), math.hypot(bx - ax, by - ay))
            for (ax, ay), (bx, by) in zip(pts, pts[1:]) if bx - ax < -1e-9]


for name, cells, shape, probe in (
        ("plate", U.plate_cells(), plate, lambda c: cq.Vector(c[0], U.TH / 2, c[1])),
        ("arm A", U.arm_cells("A"), world["A"], lambda c: cq.Vector(-U.Z_PIN + U.TH / 2, c[0] - U.Y0, c[1])),
        ("arm B", U.arm_cells("B"), world["B"], lambda c: cq.Vector(U.W + U.Z_PIN - U.TH / 2, c[0] - U.Y0, c[1]))):
    empty = sum(1 for c in cells if not shape.isInside(probe(c)))
    gap = min(hexagon(a).distance(hexagon(b)) for i, a in enumerate(cells) for b in cells[i + 1:])
    print(f"{name}: {len(cells)} hexagons, {empty} probed empty, min rib {gap:.2f} mm, open {len(cells) * hexagon((0, 0)).area * U.TH / 1000:.1f} cm3")
poly = hexagon((0, 0))
print("hexagon ceiling edges (angle from horizontal, length mm):", [(round(a), round(l, 1)) for a, l in ceilings(poly)])
