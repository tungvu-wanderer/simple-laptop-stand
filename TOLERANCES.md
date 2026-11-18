# Print tolerances

Printer: Elegoo Neptune 3 Plus, PLA. Measured with calipers on slot coupons.

## Measured bias

| Feature | Modelled | Printed | Error |
|---|---|---|---|
| Plate / tongue thickness (Z) | 6.00 mm | 5.85 mm | -0.15 mm |
| Slot width (XY), modelled 6.20 mm | 6.20 mm | 6.35 mm | +0.15 mm |
| Slot width (XY), modelled 6.30 mm | 6.30 mm | 6.45 mm | +0.15 mm |
| Slot width (XY), modelled 6.40 mm | 6.40 mm | 6.55 mm | +0.15 mm |

The error is the same on every slot, so it is a steady printer or slicer effect. If the Z offset, first-layer height or horizontal expansion changes, these numbers shift and the coupon must be reprinted.

## Rule

Printed slot = modelled slot + 0.15 mm. Printed plate thickness = modelled thickness - 0.15 mm.

Modelled slot = 5.70 mm + wanted printed clearance.

## Coupon v1 (`stl/xstand_slot_coupon.stl`)

Slots modelled at 6.20, 6.30 and 6.40 mm. All printed loose, with 0.50 mm or more of play against the 5.85 mm tongue.

## Coupon v2 (`stl/xstand_slot_coupon_v2.stl`)

Slots modelled at 5.75, 5.80, 5.85 and 5.90 mm, 15 mm deep. All four slid onto the tongue by hand, so none was too tight.

## Plate slot

Modelled at 5.80 mm (`SLOT` in `src/model.py`), which should print at about 5.95 mm and leave about 0.10 mm of play against a 5.85 mm plate.

The plate slot is 40.15 mm deep, longer than the 15 mm coupon slots, so it may feel snugger than the coupon did.

Not yet verified on the printed plates:
- If loose, try 5.75 mm.
- If binding, try 5.85 mm.

## Slot tip gap

Plate A's slot opens from the bottom and plate B's from the top, each `HC / 2 + SLOT_EXTRA` deep. Where the two tips face each other they leave a gap of 2 x `SLOT_EXTRA`, so each plate carries its own weight on its own feet and the crossing only holds them upright.

`SLOT_EXTRA` is 0.15 mm, so the modelled gap is 0.30 mm. The printed gap is not measured; if slot ends grow by half the measured 0.15 mm slot-width bias, it prints at about 0.45 mm. If the crossing sits loose in height, lower `SLOT_EXTRA`. If a plate lifts off the table, raise it.

## Other notes

- Silicone pads are 6 x 2 mm, in round pockets of 6.2 mm diameter and 1 mm depth.
- The two plates overlap by about 79 mm³ in the model, which is expected: the modelled slot is narrower than the modelled plate, and the printer bias cancels the interference.
