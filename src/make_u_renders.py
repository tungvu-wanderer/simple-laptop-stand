import sys, math
import numpy as np
import cadquery as cq
from PIL import Image, ImageDraw, ImageFont
import model as M
import cq_model as Q
import u_model as U

out = sys.argv[1]
plate = U.build_plate()
arms = {k: U.to_world_arm(k, U.build_arm_local(k)) for k in "AB"}
COL = {"plate": (224, 131, 48), "A": (52, 132, 168), "B": (96, 150, 92)}
PAD = (38, 38, 42)
LAP_W, LAP_D, LAP_H, LID_T, LID, HINGE = 313.0, 221.2, 15.5, 4.0, 221.2, 123.6
font = ImageFont.load_default(size=22)


def mesh(parts, pads=True):
    Ts, Ns, Cs = [], [], []
    for name, shape in parts.items():
        t, n = Q.tessellate(shape, tol=0.1, ang=0.4)
        Ts.append(t); Ns.append(n); Cs += [COL[name]] * len(t)
    if pads:
        for c, ax in pad_specs():
            ct, cn = M.cylinder(np.array(c), np.array(ax))
            Ts.append(ct); Ns.append(cn); Cs += [PAD] * len(ct)
    return np.concatenate(Ts), np.concatenate(Ns), np.array(Cs, float)


def pad_specs():
    sp = []
    nu = (0.0, -math.sin(U.S), math.cos(U.S))
    tg = (0.0, math.cos(U.S), math.sin(U.S))
    p3 = (U.LIP_T, U.top(U.LIP_T))
    for k, xc in (("A", U.TH / 2 - U.Z_PIN), ("B", U.W - (U.TH / 2 - U.Z_PIN))):
        for u in U.TOP_PADS_U:
            sp.append(((xc, u - U.Y0, U.top(u)), nu))
        for u in U.BOT_PADS_U:
            sp.append(((xc, u - U.Y0, 0.0), (0, 0, -1.0)))
        sp.append(((xc, p3[0] + U.LIP_PAD_C * U.NPROF[0] - U.Y0, p3[1] + U.LIP_PAD_C * U.NPROF[1]), tg))
    for x in (30.0, U.W - 30.0):
        sp.append(((x, U.TH / 2, 0.0), (0, 0, -1.0)))
    return sp


def laptop():
    base = cq.Solid.makeBox(LAP_W, LAP_D, LAP_H)
    lid = cq.Solid.makeBox(LAP_W, LID_T, LID, cq.Vector(0, -LID_T, 0))
    lid = lid.rotate(cq.Vector(0, 0, 0), cq.Vector(1, 0, 0), -(HINGE - 90)).translate(cq.Vector(0, LAP_D, LAP_H))
    s = cq.Compound.makeCompound([base, lid])
    yf = U.LIP_T - U.Y0
    o = cq.Vector(U.W / 2 - LAP_W / 2, yf - math.sin(U.S), U.top(U.LIP_T) + math.cos(U.S))
    s = s.rotate(cq.Vector(0, 0, 0), cq.Vector(1, 0, 0), U.TILT).translate(o)
    parts = []
    for sol, colr in ((s.Solids()[0], (170, 174, 182)), (s.Solids()[1], (120, 126, 138))):
        t, n = Q.tessellate(sol, tol=0.2, ang=0.5)
        parts.append((t, n, np.array([colr] * len(t), float)))
    return tuple(np.concatenate([p[i] for p in parts]) for i in range(3))


def view(phi, el, layer, ghost=None, W=800, H=560, fit=None):
    fit = fit or [layer] + ([ghost] if ghost else [])
    scene, _ = M.render([M.floor_mesh(), layer], phi, el, W, H, fit)
    if ghost:
        lr, cv = M.render([ghost], phi, el, W, H, fit)
        a = (cv * 0.42)[..., None]
        scene = scene * (1 - a) + lr * a
    return Image.fromarray(np.clip(scene, 0, 255).astype(np.uint8))


def caption(img, text):
    o = Image.new("RGB", (img.width, img.height + 36), (255, 255, 255))
    o.paste(img, (0, 36))
    ImageDraw.Draw(o).text((12, 6), text, fill=(30, 30, 30), font=font)
    return o


openm = mesh({"plate": plate, "A": arms["A"], "B": arms["B"]})
foldm = mesh({"plate": plate, "A": U.fold("A", arms["A"]), "B": U.fold("B", arms["B"])}, pads=False)
lap = laptop()

views = [
    caption(view(-38, 26, openm), "Open, from front-left (hexagon honeycomb, print-in-place hinges)"),
    caption(view(-38, 26, openm, lap), "Open, with 14\" MacBook Pro ghost"),
    caption(view(90, 0, openm, lap, H=620), "Side view: 30 deg tilt on 200 mm arms"),
    caption(view(180, 30, openm, fit=[openm]), "From behind: back plate with hinge knuckles"),
    caption(view(-38, 26, foldm), "Folded: arm A then arm B stacked on the plate, 212 x 125 x 18.6 mm"),
    caption(view(90, 0, foldm, fit=[foldm]), "Folded, edge-on: three 6 mm layers"),
]
w, h = views[0].size
sheet = Image.new("RGB", (2 * w, 3 * max(v.height for v in views)), (255, 255, 255))
rh = max(v.height for v in views)
for i, v in enumerate(views):
    sheet.paste(v, ((i % 2) * w, (i // 2) * rh))
sheet.save(f"{out}/u_renders.png")
print("saved", f"{out}/u_renders.png")
