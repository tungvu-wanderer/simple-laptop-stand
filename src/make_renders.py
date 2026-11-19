import sys, math
import numpy as np
from PIL import Image, ImageDraw, ImageFont
import model as M
import cq_model as Q

out = sys.argv[1]
shapes = Q.stand_world_shapes()
stand = Q.stand_mesh(shapes)
stand_nopad = Q.stand_mesh(shapes, pads=False)
lap = M.laptop_mesh()
floor = M.floor_mesh()
font = ImageFont.load_default(size=22)
small = ImageFont.load_default(size=17)


def view(phi, el, with_laptop, W=900, H=640, fit=None, mesh=None):
    fit = fit or [stand, lap]
    scene, _ = M.render([floor, mesh or stand], phi, el, W, H, fit)
    if with_laptop:
        lr, cv = M.render([lap], phi, el, W, H, fit)
        a = (cv * 0.42)[..., None]
        scene = scene * (1 - a) + lr * a
    return Image.fromarray(np.clip(scene, 0, 255).astype(np.uint8))


def caption(img, text):
    out_ = Image.new("RGB", (img.width, img.height + 36), (255, 255, 255))
    out_.paste(img, (0, 36))
    ImageDraw.Draw(out_).text((12, 6), text, fill=(30, 30, 30), font=font)
    return out_


lipbox = np.array([[[-130, -125, 0], [130, -125, 70], [0, -50, 0]]], float)
v1 = caption(view(-38, 26, False, fit=[stand]), "Stand only, from front-left")
v2 = caption(view(-38, 26, True), "With 14\" MacBook Pro ghost, lid at %.1f deg" % M.HINGE)
v5 = caption(view(180, 35, False, fit=[(lipbox, None, None)]), "Close-up from behind: front lips with mitered pad faces")
v3 = caption(view(90, 0, True, W=900, H=700), "Side view: tilt %.1f deg, hinge %.1f deg, screen leans back %.0f deg" % (M.TILT, M.HINGE, M.HINGE - 90 - M.TILT))
pc = Q.to_world("A", 55.0, M.top(55.0), 0.0)
padbox = np.array([[pc - [10, 10, 7], pc + [10, 10, 7], pc]], float)
v6 = caption(view(-20, 50, False, fit=[(padbox, None, None)], mesh=stand_nopad), "Close-up (pad hidden): round pocket 6.2 x 1 mm, 0.5 mm chamfers")
v4 = caption(view(0, 90, True, W=900, H=700), "Top view: X at 45 deg under the laptop footprint")

sheet = Image.new("RGB", (1800, 676 + 736 + 676), (255, 255, 255))
sheet.paste(v1, (0, 0)); sheet.paste(v2, (900, 0)); sheet.paste(v3, (0, 676)); sheet.paste(v4, (900, 676)); sheet.paste(v5, (0, 676 + 736)); sheet.paste(v6, (900, 676 + 736))
sheet.save(f"{out}/renders.png")

# ---- flat print layout --------------------------------------------------
SC = 3.0
polys = {k: M.profile(k) for k in "AB"}
Hh = int((M.top(M.L) + 10) * SC)
img = Image.new("RGB", (int(M.L * SC) + 80, 2 * (Hh + 70) + 20), (255, 255, 255))
d = ImageDraw.Draw(img)
for i, k in enumerate("AB"):
    poly, p3, p4 = polys[k]
    oy = i * (Hh + 70) + Hh + 50    # baseline y px
    ox = 40
    X = lambda u: ox + u * SC
    Y = lambda v: oy - v * SC
    col = Q.PL[k]["color"]
    d.polygon([(X(a), Y(b)) for a, b in poly.exterior.coords], fill=col)
    for r in poly.interiors:
        d.polygon([(X(a), Y(b)) for a, b in r.coords], fill=(255, 255, 255))
    pr = M.PAD_D / 2 * SC
    for u in M.BOT_PADS_U:
        d.ellipse([X(u) - pr, Y(0) - pr, X(u) + pr, Y(0) + pr], fill=(38, 38, 42))
    for u in M.TOP_PADS_U:
        cx, cy = X(u), Y(M.top(u))
        d.ellipse([cx - pr, cy - pr, cx + pr, cy + pr], fill=(38, 38, 42))
    ctr = p3 + M.LIP_PAD_C * M.NPROF
    d.ellipse([X(ctr[0]) - pr, Y(ctr[1]) - pr, X(ctr[0]) + pr, Y(ctr[1]) + pr], fill=(38, 38, 42))
    slot = "from bottom" if k == "A" else "from sloped top"
    d.text((40, i * (Hh + 70) + 6),
           f"Plate {k}: {M.L:.0f} mm long, {M.top(M.L):.0f} mm tall at rear, {M.HF:.0f} mm at front | slot {M.SLOT:.1f} x {M.SLOT_DEPTH:.1f} mm, {slot}",
           fill=(30, 30, 30), font=small)
img.save(f"{out}/plates_flat.png")
print("rear arm top height", M.top(M.L), "crossing height", M.HC, "slot depth", M.SLOT_DEPTH, "arm slope deg", math.degrees(M.S))
