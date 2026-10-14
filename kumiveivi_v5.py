"""Kumiveivi v5 — 1:20 rubber-motor winder gearbox, parametric build123d model.

Rebuilt from the v4 STEP package (Kumiveivi_v4_STEP_ja_yhteenveto/) and its
parameters.json. Geometry and dimensions follow v4; the gear tooth phases are
computed from the mesh geometry instead of copied, which removes the v4
collision between the 36T gear (part 04) and the output pinion (part 05).

Gear train: 30T -> 12T | 32T -> 12T | 36T -> 12T  =  (30/12)(32/12)(36/12) = 20
Units: mm. Housing coordinates as in v4 (housing corner at origin, shafts along Z).

Shaft layout (directions) taken from the plywood test panel photo, 2026-10-14;
the whole train is centred in the housing. Housing is a plastic frame open at
top and bottom (Y), shafts Ø6 run in 6 x 12 mm ball bearings pressed into Ø12
holes in the side plates. Input 30T and output 12T have glue collars for the shaft.
The big gears keep their v4 profiles; the 12T pinions are profile-shifted so
the long-addendum big-gear tips no longer dig below the pinion base circle,
and every centre distance is the true working distance for that shift.
"""

from math import acos, atan2, ceil, cos, degrees, dist, pi, radians, sin, tan
from pathlib import Path

from build123d import *
from ocp_vscode import show

# ---------------------------------------------------------------- parameters
module = 1.6  # 1.28 in v4; larger teeth tolerate shaft play better
pressure_angle = 20.0  # deg
dedendum_coeff = 1.35  # "deep teeth", big gears
face_width = 6.4
center_clearance = 0.05  # added to working centre distance (print play)

# big gears: addendum coefficient / backlash (v4 values, unchanged)
std_addendum, std_backlash = 1.15, 0.20  # 30T, 36T
big32_addendum = 1.00  # v4 flat-top 32T of 03

# 12T pinions: redesigned to mesh with the big gears above
pinion_shift = 0.40  # profile shift coefficient x (>= ~0.3 avoids interference)
pinion_backlash = 0.10  # tooth thinning at pitch circle
tip_clearance = 0.25  # radial tip/root clearance, x module

z_input = 30
z_pinion = 12
z_mid = 32
z_last = 36

shaft_d = 6.0  # shaft, gear bores, crank and coupler bores (6 mm bearings)
compound_hub_d = 11.36  # hub between the two rings of 03 and 04

# glue collars around the shaft (input 30T: towards plane B, output 12T: below it)
input_collar_len = 10.0
input_collar_wall = 3.0  # radial; must clear the 32T teeth of part 03
output_collar_len = 10.0
output_collar_wall = 3.0

# ball bearings in the side plates: 6 mm bore, 12 mm OD (e.g. MR126ZZ)
bearing_od = 12.0
bearing_fit = 0.0  # added to the housing hole diameter (raise if prints come out tight)

# housing: plastic frame = two side plates (Z) + two end walls (X), open in Y
gear_margin = 5.0  # free space between gear tips and housing (X: end walls, Y: open edges)
wall = 4.8  # side plate and end wall thickness
inner_width = 28.8  # gap between side plates (Z)
gear_wall_gap = 0.8  # axial gap gear <-> side plate
plane_gap = 4.0  # axial gap between gear planes (filled by hubs)

# shaft centres measured from the test panel photo; input 30T, 12T/32T,
# 12T/36T, output 12T. Only the directions are used — the distances come from
# the gear geometry and the whole train is centred in the housing.
photo_shafts = [(69.4, 72.7), (86.8, 51.5), (65.5, 33.1), (36.6, 43.1)]
layout_angles = tuple(
    degrees(atan2(b[1] - a[1], b[0] - a[0])) for a, b in zip(photo_shafts, photo_shafts[1:])
)  # deg, approx. -50.6, -139.2, 160.9

# crank and output accessories
crank_length, crank_width, crank_thickness = 128.0, 24.0, 10.0
crank_hole_inset = 12.0
# M6 nut sunk into the inner (housing) side of the crank at the shaft hole,
# locks the crank to the shaft bolt; an outer nyloc nut clamps it from outside
crank_nut_af = 10.0  # across flats
crank_nut_fit = 0.2  # added to the pocket across-flats size
crank_nut_depth = 3.0
crank_nut_height = 6.0  # M6 nyloc; the part sticking out sets the crank gap
grip_len, grip_od = 32.0, 13.6
coupler_d, coupler_len = 19.2, 19.2
coupler_cross_z = 12.672
accessory_gap = 0.8  # gap between housing outer face and coupler
crank_gap = crank_nut_height - crank_nut_depth + 1.0  # housing face <-> crank, clears the sunk nut

input_phase = 0.0  # deg, rotation of the input gear; others follow

output_dir = Path(__file__).parent / "output" / "kumiveivi_v5"

# ---------------------------------------------------------------- derived
housing_height = 2 * wall + inner_width  # 38.4
plane_a = wall + gear_wall_gap  # 5.6
plane_b = plane_a + face_width + plane_gap  # 16.0
plane_c = plane_b + face_width + plane_gap  # 26.4


def inv(phi):
    return tan(phi) - phi


def inv_inverse(value):
    phi = 0.5
    for _ in range(50):  # Newton, d(inv)/dphi = tan^2
        phi -= (inv(phi) - value) / tan(phi) ** 2
    return phi


def pinion_mesh(z_big, big_addendum):
    """Working centre distance and pinion tip/root radii for big gear + shifted 12T."""
    alpha = radians(pressure_angle)
    alpha_w = inv_inverse(inv(alpha) + 2 * tan(alpha) * pinion_shift / (z_big + z_pinion))
    a_w = module * (z_big + z_pinion) / 2 * cos(alpha) / cos(alpha_w)
    r1, r2 = module * z_pinion / 2, module * z_big / 2
    c = tip_clearance * module
    ra1 = min(r1 + (1 + pinion_shift) * module, a_w - (r2 - dedendum_coeff * module) - c)
    rf1 = a_w - (r2 + big_addendum * module) - c
    return a_w, ra1, rf1


meshes = {  # big gear teeth -> (working centre distance, pinion ra, pinion rf)
    z_input: pinion_mesh(z_input, std_addendum),
    z_mid: pinion_mesh(z_mid, big32_addendum),
    z_last: pinion_mesh(z_last, std_addendum),
}

shafts = [photo_shafts[0]]
for z_big, ang in zip((z_input, z_mid, z_last), layout_angles):
    x, y = shafts[-1]
    a = meshes[z_big][0] + center_clearance
    shafts.append((x + a * cos(radians(ang)), y + a * sin(radians(ang))))

# outermost radius on each shaft, then centre the train in the housing
reach = [
    module * (z_input / 2 + std_addendum),
    max(module * (z_mid / 2 + big32_addendum), meshes[z_input][1]),
    max(module * (z_last / 2 + std_addendum), meshes[z_mid][1]),
    max(meshes[z_last][1], shaft_d / 2 + output_collar_wall),
]
x_min = min(x - r for (x, _), r in zip(shafts, reach))
x_max = max(x + r for (x, _), r in zip(shafts, reach))
y_min = min(y - r for (_, y), r in zip(shafts, reach))
y_max = max(y + r for (_, y), r in zip(shafts, reach))
housing_length = ceil(x_max - x_min + 2 * (wall + gear_margin))  # X
housing_depth = ceil(y_max - y_min + 2 * gear_margin)  # Y
dx = housing_length / 2 - (x_min + x_max) / 2
dy = housing_depth / 2 - (y_min + y_max) / 2
shafts = [(x + dx, y + dy) for x, y in shafts]


# ---------------------------------------------------------------- gears
def gear_profile(teeth, addendum, backlash, shift=0.0, tip_r=None, root_r=None, flank_points=12):
    """Involute spur gear outline as a Face, first tooth centred on +X.

    shift is the profile shift coefficient; tip_r / root_r override the radii
    given by addendum / dedendum_coeff.
    """
    alpha = radians(pressure_angle)
    r = module * teeth / 2
    rb = r * cos(alpha)
    ra = tip_r if tip_r else r + addendum * module
    rf = root_r if root_r else r - dedendum_coeff * module

    tooth = module * (pi / 2 + 2 * shift * tan(alpha)) - backlash
    half_thickness = tooth / (2 * r)

    def psi(radius):  # half angular tooth thickness at radius
        return half_thickness + inv(alpha) - inv(acos(min(1.0, rb / radius)))

    if psi(ra) <= 0:
        raise ValueError(f"{teeth}T: pointed teeth, reduce addendum")

    r_start = max(rf, rb)
    radii = [r_start + (ra - r_start) * i / flank_points for i in range(flank_points + 1)]
    flank = [(rr, psi(rr)) for rr in radii]
    if rf < rb:  # radial flank below the base circle
        flank.insert(0, (rf, psi(rb)))

    pitch = 2 * pi / teeth
    pts = []
    for k in range(teeth):
        c = k * pitch
        for rr, p in flank:  # rising flank
            pts.append((rr * cos(c - p), rr * sin(c - p)))
        for i in range(1, 3):  # tip land
            t = -psi(ra) + 2 * psi(ra) * i / 3
            pts.append((ra * cos(c + t), ra * sin(c + t)))
        for rr, p in reversed(flank):  # falling flank
            pts.append((rr * cos(c + p), rr * sin(c + p)))
        root_from, root_to = c + flank[0][1], c + pitch - flank[0][1]
        for i in range(1, 4):  # root land
            t = root_from + (root_to - root_from) * i / 4
            pts.append((rf * cos(t), rf * sin(t)))
    return make_face(Polyline(*pts, close=True))


def big_profile(teeth, addendum):
    return gear_profile(teeth, addendum, std_backlash)


def pinion_profile(z_big):
    """12T pinion matched to the big gear it meshes with."""
    _, ra, rf = meshes[z_big]
    return gear_profile(z_pinion, 0, pinion_backlash, pinion_shift, tip_r=ra, root_r=rf)


def gear_ring(profile, z0, rotation=0.0):
    ring = extrude(profile, face_width)
    return ring.rotate(Axis.Z, rotation).moved(Location((0, 0, z0)))


def mesh_phase(c1, c2, z1, rot1, z2):
    """Rotation (deg) for gear 2 so it meshes with gear 1 rotated rot1 (deg).

    Gear 1 has tooth k at rot1 + k*360/z1; gear 2 needs a tooth gap facing
    gear 1 when gear 1 has a tooth on the centre line.
    """
    beta = degrees(atan2(c2[1] - c1[1], c2[0] - c1[0]))
    delta = rot1 - beta  # how far gear 1's tooth is off the centre line
    return beta + 180 + 180 / z2 - delta * z1 / z2


def bored(part, length):
    return part - Cylinder(shaft_d / 2, length, align=(Align.CENTER, Align.CENTER, Align.MIN))


def collar(wall_r, length, z0):
    return Cylinder(shaft_d / 2 + wall_r, length, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(
        Location((0, 0, z0))
    )


# rotations of the driving ring on each shaft
rot1 = input_phase
rot2 = mesh_phase(shafts[0], shafts[1], z_input, rot1, z_pinion)
rot3 = mesh_phase(shafts[1], shafts[2], z_mid, rot2, z_pinion)
rot4 = mesh_phase(shafts[2], shafts[3], z_last, rot3, z_pinion)

# 02: input gear 30T, plane A, glue collar towards plane B
input_gear = bored(
    gear_ring(big_profile(z_input, std_addendum), plane_a, rot1)
    + collar(input_collar_wall, input_collar_len, plane_a + face_width),
    housing_height,
)

# 03: compound 12T (plane A) + hub + 32T (plane B)
compound_1 = bored(
    gear_ring(pinion_profile(z_input), plane_a, rot2)
    + Cylinder(compound_hub_d / 2, plane_gap, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(
        Location((0, 0, plane_a + face_width))
    )
    + gear_ring(big_profile(z_mid, big32_addendum), plane_b, rot2),
    housing_height,
)

# 04: compound 12T (plane B) + hub + 36T (plane C)
compound_2 = bored(
    gear_ring(pinion_profile(z_mid), plane_b, rot3)
    + Cylinder(compound_hub_d / 2, plane_gap, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(
        Location((0, 0, plane_b + face_width))
    )
    + gear_ring(big_profile(z_last, std_addendum), plane_c, rot3),
    housing_height,
)

# 05: output pinion 12T (plane C) with glue collar below it
output_pinion = bored(
    collar(output_collar_wall, output_collar_len, plane_c - output_collar_len)
    + gear_ring(pinion_profile(z_last), plane_c, rot4),
    housing_height,
)

gears = {
    "02_input_30T": (input_gear, shafts[0]),
    "03_compound_12T_32T": (compound_1, shafts[1]),
    "04_compound_12T_36T": (compound_2, shafts[2]),
    "05_output_12T": (output_pinion, shafts[3]),
}

# ---------------------------------------------------------------- housing
with BuildPart() as housing_bp:
    with BuildSketch(Plane.XZ):  # frame profile in X / Z, extruded along Y
        Rectangle(housing_length, housing_height, align=(Align.MIN, Align.MIN))
        with Locations((wall, wall)):
            Rectangle(
                housing_length - 2 * wall, inner_width, align=(Align.MIN, Align.MIN), mode=Mode.SUBTRACT
            )
    extrude(amount=-housing_depth)  # Plane.XZ normal is -Y
    for x, y in shafts:  # bearing holes straight through both side plates
        with Locations((x, y, 0)):
            Cylinder(
                (bearing_od + bearing_fit) / 2,
                housing_height,
                align=(Align.CENTER, Align.CENTER, Align.MIN),
                mode=Mode.SUBTRACT,
            )
housing = housing_bp.part

# ---------------------------------------------------------------- crank and coupler
crank_arm = Box(crank_length, crank_width, crank_thickness, align=(Align.MIN, Align.MIN, Align.MIN))
for hx in (crank_hole_inset, crank_length - crank_hole_inset):
    crank_arm -= Cylinder(shaft_d / 2, crank_thickness, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(
        Location((hx, crank_width / 2, 0))
    )
crank_arm -= extrude(
    RegularPolygon((crank_nut_af + crank_nut_fit) / 2, 6, major_radius=False), crank_nut_depth
).moved(Location((crank_hole_inset, crank_width / 2, crank_thickness - crank_nut_depth)))

grip_sleeve = Cylinder(grip_od / 2, grip_len, align=(Align.CENTER, Align.CENTER, Align.MIN)) - Cylinder(
    shaft_d / 2, grip_len, align=(Align.CENTER, Align.CENTER, Align.MIN)
)

output_coupler = (
    Cylinder(coupler_d / 2, coupler_len, align=(Align.CENTER, Align.CENTER, Align.MIN))
    - Cylinder(shaft_d / 2, coupler_len, align=(Align.CENTER, Align.CENTER, Align.MIN))
    - Cylinder(shaft_d / 2, coupler_d, rotation=(0, 90, 0)).moved(Location((0, 0, coupler_cross_z)))
)

# crank under the bottom side wall on shaft 1, grip pointing away; coupler on top of shaft 4
crank_z = -crank_gap - crank_thickness
crank_loc = Location((shafts[0][0] - crank_hole_inset, shafts[0][1] - crank_width / 2, crank_z))
grip_loc = Location(
    (shafts[0][0] + crank_length - 2 * crank_hole_inset, shafts[0][1], crank_z - grip_len)
)
coupler_loc = Location((shafts[3][0], shafts[3][1], housing_height + accessory_gap))

# ---------------------------------------------------------------- assembly
placed_gears = {name: g.moved(Location((x, y, 0))) for name, (g, (x, y)) in gears.items()}
parts = {
    "01_housing": housing,
    **placed_gears,
    "06_crank_arm": crank_arm.moved(crank_loc),
    "07_grip_sleeve": grip_sleeve.moved(grip_loc),
    "08_output_coupler": output_coupler.moved(coupler_loc),
}
colors = {
    "01_housing": "lightgray",
    "02_input_30T": "royalblue",
    "03_compound_12T_32T": "orange",
    "04_compound_12T_36T": "seagreen",
    "05_output_12T": "crimson",
    "06_crank_arm": "dimgray",
    "07_grip_sleeve": "black",
    "08_output_coupler": "gold",
}
for name, p in parts.items():
    p.label = name
    p.color = Color(colors[name])
assembly = Compound(label="kumiveivi_v5", children=list(parts.values()))

# ---------------------------------------------------------------- checks
names = list(placed_gears)
print("Shaft centres:", [(round(x, 3), round(y, 3)) for x, y in shafts])
for i, a in enumerate(names):
    for b in names[i + 1 :] + ["01_housing"]:
        overlap = (parts[a] & parts[b]).volume
        if overlap > 1e-6:
            print(f"COLLISION {a} <-> {b}: {overlap:.3f} mm^3")
# Ø6 shafts (bolts) straight through the housing must miss every other gear
for (name, (_, (x, y))) in gears.items():
    rod = Cylinder(shaft_d / 2, housing_height, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(
        Location((x, y, 0))
    )
    others = [n for n in placed_gears if n != name]
    gap, nearest = min((rod.distance_to(parts[n]), n) for n in others)
    hit = sum((rod & parts[n]).volume for n in others)
    status = "OK" if hit < 1e-6 else "HITS A GEAR"
    print(f"Shaft of {name}: nearest other gear {nearest} at {gap:.2f} mm  {status}")

bb = housing.bounding_box()
print(f"Housing {bb.size.X:.1f} x {bb.size.Y:.1f} x {bb.size.Z:.1f} mm")
print(
    f"Gear clearance to housing: left {x_min + dx - wall:.1f}, right {housing_length - wall - (x_max + dx):.1f}, "
    f"bottom {y_min + dy:.1f}, top {housing_depth - (y_max + dy):.1f} mm"
)
ratio = (z_input / z_pinion) * (z_mid / z_pinion) * (z_last / z_pinion)
print(f"Ratio 1:{ratio:g}")

# 2D running check: turn each pair through one driver tooth pitch
mesh_steps = 12
pairs = [
    ("30T -> 12T", big_profile(z_input, std_addendum), pinion_profile(z_input), 0, rot1, rot2, z_input),
    ("32T -> 12T", big_profile(z_mid, big32_addendum), pinion_profile(z_mid), 1, rot2, rot3, z_mid),
    ("36T -> 12T", big_profile(z_last, std_addendum), pinion_profile(z_last), 2, rot3, rot4, z_last),
]
for label, big, small, i, r_big, r_small, z_big in pairs:
    (x1, y1), (x2, y2) = shafts[i], shafts[i + 1]
    worst_overlap, min_gap = 0.0, float("inf")
    for k in range(mesh_steps):
        t = 360 / z_big * k / mesh_steps
        g1 = big.rotate(Axis.Z, r_big + t).moved(Location((x1, y1)))
        g2 = small.rotate(Axis.Z, r_small - t * z_big / z_pinion).moved(Location((x2, y2)))
        worst_overlap = max(worst_overlap, (g1 & g2).area)
        min_gap = min(min_gap, g1.distance_to(g2))
    _, ra, rf = meshes[z_big]
    status = "OK" if worst_overlap < 1e-6 else "COLLISION"
    print(
        f"{label}: centre {dist((x1, y1), (x2, y2)):.3f} mm, pinion OD {2 * ra:.2f} root {2 * rf:.2f}, "
        f"overlap {worst_overlap:.4f} mm^2, min flank gap {min_gap:.3f} mm  {status}"
    )

# ---------------------------------------------------------------- export
def print_ready(part, rotation=(0, 0, 0)):
    """Rotate into print orientation, centre on XY origin, bottom at Z=0."""
    p = part.rotate(Axis.X, rotation[0]).rotate(Axis.Y, rotation[1])
    bb = p.bounding_box()
    return p.moved(Location((-bb.center().X, -bb.center().Y, -bb.min.Z)))


hl, hd, hh = round(housing_length), round(housing_depth), round(housing_height)
print_parts = {  # file name: (part, rotation about X / Y in deg)
    f"01_kotelo_{hl}x{hd}x{hh}mm_laakerireiat_12mm": (housing, (90, 0)),  # stands on an open edge
    f"02_sisaantuloratas_{z_input}T_kaulus": (input_gear, (0, 0)),  # teeth down, collar up
    f"03_valiratas_{z_pinion}T-{z_mid}T": (compound_1, (180, 0)),  # big ring down
    f"04_valiratas_{z_pinion}T-{z_last}T": (compound_2, (180, 0)),  # big ring down
    f"05_ulostulopinjoni_{z_pinion}T_kaulus": (output_pinion, (180, 0)),  # teeth down, collar up
    f"06_veivin_varsi_{round(crank_thickness)}mm_mutteriupotus_M6": (crank_arm, (0, 0)),  # pocket up
    "07_veivin_kahva": (grip_sleeve, (0, 0)),
    "08_ulostulon_liitin": (output_coupler, (0, 0)),
}

output_dir.mkdir(parents=True, exist_ok=True)
for old in [*output_dir.glob("*.step"), *output_dir.glob("*.stl")]:  # drop renamed leftovers
    old.unlink()
export_step(assembly, output_dir / "00_kumiveivi_v5_kokoonpano.step")
for name, (part, rotation) in print_parts.items():
    p = print_ready(part, rotation)
    export_step(p, output_dir / f"{name}.step")
    export_stl(p, output_dir / f"{name}.stl", tolerance=0.01, angular_tolerance=0.1)

try:
    show(assembly)
except Exception as e:
    print(f"Viewer not available: {e}")
