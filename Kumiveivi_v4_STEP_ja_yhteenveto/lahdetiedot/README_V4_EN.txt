RUBBER WINDER 1:20 - V4 COMPOUND-GEAR PRINTABILITY + FLAT 32T FACE

USE THE ROOT DIRECTORY STL FILES FOR THE WORKING GEARBOX.

Changes since your V3 ZIP:
- 03 compound 12T+32T: rebuilt BOTH gears in CAD as one solid, upper large
  32T tooth face is fully flat at local Z=16.800 mm, not stepped.
- 04 compound 12T+36T: rebuilt 12T pinion for stronger printable tooth tips.
- The 12T pinions in 03 and 04 have SAME MODULE and tooth count as their mates.
  Addendum 1.15m -> 0.95m, backlash 0.20 mm -> 0.10 mm.
  Result: tips ~0.76 mm wide versus the original ~0.28 mm wide. The teeth
  are now more robustly printable with a 0.4 mm nozzle.
- Rebuilt 32T gear with 1.00m addendum (vs 1.15m) for thicker, fully flat
  top surface teeth. It retains the same 32 teeth, module and axis.
- 30T, 36T and output 12T, case, axle locations, crank, bore sizes unchanged.
- All gearing ratios remain 30/12 * 32/12 * 36/12 = 20:1.
- Updated assembly STEP and STL and updated electronics reference assembly STEP.
- The model tooth angular phases in assembly were rotated for valid meshing.
  Individual shaft rotation can of course be adjusted freely on assembly.

IMPORTANT ON THE REQUESTED +8 MM OUTSIDE DIAMETER:
The current 12T has OD 18.304 mm. Expanding OD by 8 mm to 26.304 mm while
keeping the 12 teeth, other gears and original 5 mm shaft locations cannot
provide mechanically correct engagement: the small teeth penetrate solid
material of the larger gear and jam. The original gears already have overlapping
OD circles; they do not have 2 mm of missing center-distance contact.

For demonstration/testing, the exact +8 mm enlarged 03 and 04 files are saved in
TEST_ONLY_plus8mm_OD_NOT_FUNCTIONAL/. DO NOT print them for the working device.
A physically functional +8 mm pinion change requires resizing/repositioning its
mating gears and shaft holes, and may alter the 20:1 ratio unless fully redesigned.

CHECKS:
- Working gear pair profile simulations 30:12, 32:12 and 36:12 had zero
  2D tooth polygon intersection across a single-tooth rotation cycle (31 steps).
- New standalone STL meshes are watertight and are single continuous solids.
- Shaft bores remain 5.0 mm nominal.

PRINT:
- For 0.4 mm nozzle: 0.16 mm layers, >=4 walls, gear face on build plate,
  100% infill for small pinions recommended.
- Check slicer preview for continuous top surfaces; top surface gaps can arise
  from slicing line width and speed even when STL is flat and watertight.
- Test fit by hand without motor load before winding a rubber motor.
