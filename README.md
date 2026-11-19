# simple-laptop-stand

X-shaped laptop stand for a 14" MacBook Pro: two 6 mm plates that cross at 90 degrees in slots, printed flat on an Elegoo Neptune 3 Plus in PLA. See `TOLERANCES.md` for the measured print bias and slot widths.

## Environment

Managed with [uv](https://docs.astral.sh/uv/):

```sh
uv sync
```

## Generate the STLs

Run the scripts from `src/`; they import each other by module name.

```sh
mkdir -p out
uv run --directory src python export_plates.py ../stl     # plates A and B
uv run --directory src python export_stl.py ../stl        # slot coupon v1
uv run --directory src python make_renders.py ../out      # preview renders
```

The geometry parameters (arm length, heights, slot width, pads) are at the top of `src/model.py`.

## U stand (folding variant)

A 200 mm back plate with two 200 mm arms hinged at its ends. The arms are parallel, so the laptop tilt equals the arm slope (30°, arm height 10 mm at the front and 125.5 mm at the rear). The arms' top plane continues across the plate and hinge tab, so the stand has one flat top. Each arm folds 90° onto the plate's front face. Folded, the stand is about 212 x 125 x 18.6 mm.

```sh
uv run --directory src python export_u.py ../stl    # stl/ustand.stl plus fold and interference checks
uv run --directory src python make_u_renders.py ../out  # out/u_renders.png: open, folded and with a laptop ghost
```

- `ustand.stl` is the whole stand in the open pose, printed in place standing on its bottom edges (about 213 x 200 x 125 mm). Use a brim; the ceilings over the knuckle pockets bridge about 12 mm.
- The plate and arms are lightened with flat-top hexagons (6 mm sides, 2 mm ribs, 3 mm solid rim): a horizontal 6 mm ceiling and 60° edges, so they print without supports standing up.
- Each hinge is a 5 mm pin printed with the plate knuckles, through 5.6 mm bores in the arm knuckles. Modelled clearances are 0.3 mm, which prints about 0.15 mm wider. If the hinges fuse or bind, raise `CLR` in `src/u_model.py`.
- Fold arm A first, then arm B; B's pin sits one layer further forward so it stacks over A.
- Each arm stops against the plate at 90° open. Nothing latches it against folding inward.
- Parameters are at the top of `src/u_model.py`; the X stand code is untouched.
