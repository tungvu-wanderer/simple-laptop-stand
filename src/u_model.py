"""CadQuery build of the U stand: a back plate with two arms hinged at its ends that fold flat onto its front face.

The three parts are printed in place in the open pose, standing on their bottom edges."""
import math
import cadquery as cq
from shapely.geometry import Polygon, box

# ---- parameters (mm / degrees) -------------------------------------------
TH = 6.0            # plate thickness
L = 200.0           # arm length, hinge stub included
W = 200.0           # back plate width, equal to the arm length so a folded arm stays inside the hinge span
HF = 10.0           # arm height at the front end (under the slope)
TILT = 30.0         # laptop tilt, equal to the arm slope
LIP_H = 15.0        # front lip height above slope, measured along slope normal
LIP_T = 13.0        # lip block depth along the arm
LIP_PAD_C = 9.0     # lip pad centre along the lip face from the slope
TOP_PADS_U = (55.0, L - 30.0)
BOT_PADS_U = (25.0, L - 30.0)
POCKET_D, POCKET_DEPTH = 6.2, 1.0
CH, CH_CORNER, CH_LIP = 0.5, 3.0, 1.2

# honeycomb lightening: flat-top hexagons print without supports standing up (horizontal ceiling one side long, other edges at 60 deg)
HEX_S = 6.0         # hexagon side, also the longest ceiling bridge
RIB = 2.0           # rib between neighbouring hexagons
RIM = 3.0           # solid margin from a hexagon to the part outline

# print-in-place hinge: vertical pin on the plate's front-face plane, printed with the plate knuckles and running through the arm knuckles
# 5 knuckles alternating plate / arm / plate / arm / plate
PIN_D = 5.0         # pin diameter
KN_R = 5.5          # knuckle radius
CLR = 0.3           # modelled clearance between moving parts (prints about 0.15 mm wider)
KN_N = 5
KN_Z0, KN_Z1 = 6.0, 105.0
KN_H = (KN_Z1 - KN_Z0 - (KN_N - 1) * CLR) / KN_N
STUB = TH           # arm length behind the pin
TAB_W = 5.0         # plate tab that carries arm B's pin and stop

S = math.radians(TILT)
TS = math.tan(S)
REAR = HF + L * TS                                   # arm height at the rear end, also the plate height
NPROF = (-math.sin(S), math.cos(S))                  # slope normal (u,v)
TANG = (math.cos(S), math.sin(S))                    # slope direction (u,v)
Y0 = L - STUB                                        # arm u of arm A's pin; world y = u - Y0 for both arms
PIN_B = (W, -(TH + CLR))                             # arm B's pin sits one stack layer further forward than arm A's
U_PIN = {"A": Y0, "B": Y0 - (TH + CLR)}              # arm u of each pin, so the two arms' fronts line up
Z_PIN = TH + CLR                                     # pin height above the arm's outer face


def top(u):
    return HF + u * TS


def plane_z(y):
    """Height of the arms' top plane in the assembled stand."""
    return top(y + Y0)


def honeycomb(region):
    """Centres of flat-top hexagons that sit at least RIM inside the shapely region, on the lattice phase that fits the most."""
    ap = HEX_S * math.sqrt(3) / 2 + RIB / 2            # apothem of a cell, hexagon plus half a rib
    dx, dy = math.sqrt(3) * ap, 2 * ap
    x0, y0, x1, y1 = region.bounds
    inner = region.buffer(-RIM)
    best = []
    for px in range(8):
        for py in range(8):
            cells = []
            for i in range(int((x1 - x0) / dx) + 2):
                cx = x0 + (px / 8 - 1) * dx + i * dx
                for j in range(int((y1 - y0) / dy) + 3):
                    cy = y0 + (py / 8 - 1) * dy + j * dy + (i % 2) * ap
                    hexagon = Polygon([(cx + HEX_S * math.cos(math.radians(60 * k)), cy + HEX_S * math.sin(math.radians(60 * k)))
                                       for k in range(6)])
                    if inner.contains(hexagon):
                        cells.append((cx, cy))
            if len(cells) > len(best):
                best = cells
    return best


def arm_outline(k):
    u_end = U_PIN[k] + STUB
    p3 = (LIP_T, top(LIP_T))
    p4 = (p3[0] + LIP_H * NPROF[0], p3[1] + LIP_H * NPROF[1])
    return [(0, 0), (u_end, 0), (u_end, top(u_end)), p3, p4, (0, p4[1])]


def arm_cells(k):
    """Hexagons run from behind the lip to the hinge pocket."""
    u0, u1 = LIP_T + RIM, U_PIN[k] - KN_R - CLR - RIM
    return honeycomb(Polygon(arm_outline(k)).intersection(box(u0, -1, u1, 1000)))


def plate_cells():
    """The plate's top is cut to the arms' plane, which is lowest at the front face."""
    return honeycomb(box(KN_R + CLR + RIM, RIM, W - RIM, plane_z(0.0)))


def _box(x0, x1, y0, y1, z0, z1):
    return cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, cq.Vector(x0, y0, z0))


def _cyl(r, h, x, y, z, axis):
    return cq.Solid.makeCylinder(r, h, cq.Vector(x, y, z), cq.Vector(*axis))


def _knuckles():
    """(lo, hi, owner) per knuckle; even ones belong to the plate, odd ones to the arm."""
    return [(KN_Z0 + i * (KN_H + CLR), KN_Z0 + i * (KN_H + CLR) + KN_H, "plate" if i % 2 == 0 else "arm")
            for i in range(KN_N)]


def build_arm_local(k):
    """Arm in its print pose: outer face on z=0, hinge side up, u along the arm, v along the height."""
    u_end = U_PIN[k] + STUB
    u_p = U_PIN[k]
    pts = arm_outline(k)
    p4 = pts[4]
    solid = cq.Workplane("XY").polyline(pts).close().extrude(TH)

    for (x, y), size in (((0.0, 0.0), CH_CORNER), ((u_end, 0.0), CH_CORNER),
                         ((u_end, top(u_end)), CH_CORNER), ((0.0, p4[1]), CH_LIP)):
        solid = solid.edges("|Z").edges(cq.selectors.NearestToPointSelector((x, y, TH / 2))).chamfer(size)

    solid = solid.faces("<Z or >Z").edges().chamfer(CH)
    cells = arm_cells(k)
    solid = solid.cut(cq.Workplane("XY").workplane(offset=-1).pushPoints(cells).polygon(6, 2 * HEX_S).extrude(TH + 2))

    # hinge: plate knuckles get a pocket in the arm, arm knuckles are discs on the hinge side
    for lo, hi, owner in _knuckles():
        if owner == "plate":
            solid = solid.cut(_box(u_p - KN_R - CLR, u_end + 1, lo - CLR, hi + CLR, -1, TH + 1))
        else:
            solid = solid.union(_cyl(KN_R, hi - lo, u_p, lo, Z_PIN, (0, 1, 0)))
            solid = solid.cut(_cyl(PIN_D / 2 + CLR, hi - lo, u_p, lo, Z_PIN, (0, 1, 0)))

    solid = _pockets_arm(solid)
    return solid.val()


def _pockets_arm(solid):
    r, d = POCKET_D / 2, POCKET_DEPTH
    cuts = []
    for u in TOP_PADS_U:
        cuts.append(((u, top(u)), NPROF))
    for u in BOT_PADS_U:
        cuts.append(((u, 0.0), (0.0, -1.0)))
    p3 = (LIP_T, top(LIP_T))
    cuts.append(((p3[0] + LIP_PAD_C * NPROF[0], p3[1] + LIP_PAD_C * NPROF[1]), TANG))
    for (u, v), (au, av) in cuts:
        solid = solid.cut(_cyl(r, 2 * d, u - au * d, v - av * d, TH / 2, (au, av, 0)))
    return solid


def build_plate():
    """Back plate in world pose: x across, y thickness (front face at y=0), z up."""
    H = REAR
    solid = cq.Workplane("XY").add(_box(0, W, 0, TH, 0, H))
    solid = solid.faces("<Y or >Y").edges().chamfer(CH)
    cells = plate_cells()
    solid = solid.cut(cq.Workplane("XZ").workplane(offset=1).pushPoints(cells).polygon(6, 2 * HEX_S).extrude(-TH - 2))

    # tab on the front face at the right end for arm B's pin and stop
    solid = solid.union(_box(W - TAB_W, W, -(TH + CLR), 0, 0, H))
    for lo, hi, owner in _knuckles():
        if owner == "plate":
            solid = solid.union(_cyl(KN_R, hi - lo, 0.0, 0.0, lo, (0, 0, 1)))
            solid = solid.union(_cyl(KN_R, hi - lo, PIN_B[0], PIN_B[1], lo, (0, 0, 1)))
        else:
            solid = solid.cut(_box(-1, KN_R + CLR, -1, TH + 1, lo - CLR, hi + CLR))
            solid = solid.cut(_box(W - TAB_W - 1, W + 1, -(TH + CLR) - 1, 1, lo - CLR, hi + CLR))
    for x, y in ((0.0, 0.0), PIN_B):
        solid = solid.union(_cyl(PIN_D / 2, KN_Z1 - KN_Z0, x, y, KN_Z0, (0, 0, 1)))

    # the arms' top plane continues across the plate and tab, so the whole stand has one flat top
    ya, yb = -(TH + CLR) - 1, TH + 1
    above = (cq.Workplane("YZ").workplane(offset=-10)
             .polyline([(ya, plane_z(ya)), (yb, plane_z(yb)), (yb, H + 20), (ya, H + 20)]).close().extrude(W + 20))
    solid = solid.cut(above)

    for x in (30.0, W - 30.0):
        solid = solid.cut(_cyl(POCKET_D / 2, 2 * POCKET_DEPTH, x, TH / 2, -POCKET_DEPTH, (0, 0, 1)))
    return solid.val()


def to_world_arm(k, shape):
    """Arm print pose -> open pose in the assembled stand."""
    s = shape.rotate(cq.Vector(0, 0, 0), cq.Vector(1, 1, 1), 120)      # (u,v,z) -> (z,u,v)
    s = s.translate(cq.Vector(-Z_PIN, -Y0, 0))
    return s if k == "A" else s.mirror("YZ", cq.Vector(W / 2, 0, 0))


def pivot(k):
    return (0.0, 0.0) if k == "A" else PIN_B


def fold_angle(k):
    return 90.0 if k == "A" else -90.0


def fold(k, shape, frac=1.0):
    px, py = pivot(k)
    return shape.rotate(cq.Vector(px, py, 0), cq.Vector(px, py, 1), fold_angle(k) * frac)
