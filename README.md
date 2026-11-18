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
uv run --directory src python export_stl.py ../stl        # plates and coupon v1
uv run --directory src python make_renders.py ../out      # preview renders
```

The geometry parameters (arm length, heights, slot width, pads) are at the top of `src/model.py`.
