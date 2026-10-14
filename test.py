from pathlib import Path

from build123d import *
from ocp_vscode import show

# Dimensions (mm)
plate_length = 100
plate_width = 50
plate_thickness = 4
hole_diameter = 5
hole_inset = 10  # hole centre distance from each edge at the corners

output_dir = Path(__file__).parent / "output"

with BuildPart() as plate:
    Box(plate_length, plate_width, plate_thickness)
    with Locations((0, 0, plate_thickness / 2)):
        with GridLocations(
            plate_length - 2 * hole_inset,
            plate_width - 2 * hole_inset,
            2,
            2,
        ):
            Hole(radius=hole_diameter / 2)

part = plate.part

output_dir.mkdir(exist_ok=True)
export_step(part, output_dir / "test.step")
export_stl(part, output_dir / "test.stl")

print(f"Volume: {part.volume:.1f} mm^3")
print(f"Bounding box: {part.bounding_box().size}")

try:
    show(part)
except Exception as e:
    print(f"Viewer not available: {e}")
