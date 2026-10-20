"""Kumiveivi v6 (päädyt) — 1:20 rubber-motor winder gearbox, parametric build123d model.

Based on kumiveivi_v5.py. Changes from v5:
- Housing is a box closed on five sides: two side plates (bearings), two fixed
  end walls and a bottom; open only upwards (+Y). Wall thickness 5 mm.
- All housing holes are Ø12 for 6 x 12 mm ball bearings (e.g. MR126ZZ, 4 mm wide).
- All gear bores Ø6 (M6 bolt as shaft).
- Bigger teeth (module 2.0) and a cleaner tooth system: big gears are standard
  (x = 0, addendum 1.0 m), the 12T pinions are profile shifted (x = +0.4) so they
  are not undercut. Working centre distance, tip/root clearances, contact ratio
  and tip interference are computed and checked for every pair.
- Input 30T and output 12T have a 10 mm long glue collar, 2 mm wall around the bolt.

Gear train: 30T -> 12T | 32T -> 12T | 36T -> 12T  =  (30/12)(32/12)(36/12) = 20
Units: mm. Housing corner at origin, shafts along Z, open side at +Y.
"""

from math import acos, atan2, ceil, cos, degrees, dist, pi, radians, sin, sqrt, tan
from pathlib import Path

from build123d import *
from ocp_vscode import show

# ---------------------------------------------------------------- parameters
module = 2.0  # 1.28 in v4, 1.6 in v5; bigger teeth tolerate print errors and shaft play
pressure_angle = 20.0  # deg
face_width = 8.0  # 6.4 in v4/v5; wider = sturdier teeth
addendum_coeff = 1.0  # standard tooth: addendum 1.0 m
dedendum_coeff = 1.25  # standard tooth: dedendum 1.25 m (0.25 m tip clearance)
tip_clearance = 0.25  # radial tip/root clearance, x module
min_tip_land = 0.25  # minimum tooth thickness at the tip, x module

z_input = 30
z_pinion = 12
z_mid = 32
z_last = 36

pinion_shift = 0.40  # profile shift of the 12T pinions (12T undercuts below x ~0.3)
big_shift = 0.0  # profile shift of the big gears
backlash = 0.25  # total circumferential backlash per mesh at the pitch circle (FDM play)
center_clearance = 0.05  # added to working centre distance (print play)

shaft_d = 6.0  # M6 bolt = shaft; every gear bore, crank and coupler bore

# glue collars on the first and last gear: 10 mm long, 2 mm wall around the bolt
collar_len = 10.0
collar_wall = 2.0

# ball bearings in the side plates: 6 mm bore, 12 mm OD
bearing_od = 12.0
bearing_fit = 0.0  # added to the housing hole diameter (raise if prints come out tight)

# housing: box open only upwards (+Y)
wall = 5.0  # all walls
gear_margin = 5.0  # free space between gear tips and housing walls
gear_wall_gap = 0.8  # axial gap gear <-> side plate
plane_gap = 4.0  # axial gap between gear planes (filled by hubs / collars)

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
# M6 nut sunk into the inner (housing) side of the crank at the shaft hole
crank_nut_af = 10.0  # across flats
crank_nut_fit = 0.2  # added to the pocket across-flats size
crank_nut_depth = 3.0
crank_nut_height = 6.0  # M6 nyloc
grip_len, grip_od = 32.0, 13.6
coupler_d, coupler_len = 19.2, 19.2
coupler_cross_z = 12.672
accessory_gap = 0.8  # gap between housing outer face and coupler
crank_gap = crank_nut_height - crank_nut_depth + 1.0  # housing face <-> crank

input_phase = 0.0  # deg, rotation of the input gear; others follow
mesh_steps = 24  # positions per tooth pitch in the running check
# OCP CAD Viewer port; None = ask/auto-detect. Set it (e.g. 3939) when several
# VS Code windows run a viewer, otherwise show() asks interactively and fails headless.
viewer_port = None

output_dir = Path(__file__).parent / "output" / "kumiveivi_v6_paadyt"

# ---------------------------------------------------------------- derived
inner_width = 2 * gear_wall_gap + 3 * face_width + 2 * plane_gap  # gap between side plates
housing_height = 2 * wall + inner_width  # Z
plane_a = wall + gear_wall_gap
plane_b = plane_a + face_width + plane_gap
plane_c = plane_b + face_width + plane_gap
collar_r = shaft_d / 2 + collar_wall
alpha = radians(pressure_angle)


def inv(phi):
    return tan(phi) - phi


def inv_inverse(value):
    phi = 0.5
    for _ in range(50):  # Newton, d(inv)/dphi = tan^2
        phi -= (inv(phi) - value) / tan(phi) ** 2
    return phi


def half_angle(teeth, shift, thinning, radius):
    """Half angular tooth thickness at radius (rad)."""
    r = module * teeth / 2
    rb = r * cos(alpha)
    s = module * (pi / 2 + 2 * shift * tan(alpha)) - thinning
    return s / (2 * r) + inv(alpha) - inv(acos(min(1.0, rb / radius)))


def limit_tip(teeth, shift, thinning, ra):
    """Lower ra until the tip land is at least min_tip_land * module."""
    while 2 * ra * half_angle(teeth, shift, thinning, ra) < min_tip_land * module:
        ra -= 0.01
    return ra


def design_mesh(z_big):
    """Pinion (1) + big gear (2): working centre distance, radii, quality numbers."""
    z1, z2, x1, x2 = z_pinion, z_big, pinion_shift, big_shift
    r1, r2 = module * z1 / 2, module * z2 / 2
    rb1, rb2 = r1 * cos(alpha), r2 * cos(alpha)
    alpha_w = inv_inverse(inv(alpha) + 2 * tan(alpha) * (x1 + x2) / (z1 + z2))
    a_w = (r1 + r2) * cos(alpha) / cos(alpha_w)  # zero-backlash centre distance
    c = tip_clearance * module
    rf1 = r1 + (x1 - dedendum_coeff) * module
    rf2 = r2 + (x2 - dedendum_coeff) * module
    thin = backlash / 2  # each gear's tooth thinned by half the backlash
    ra1 = limit_tip(z1, x1, thin, min(r1 + (addendum_coeff + x1) * module, a_w - rf2 - c))
    ra2 = limit_tip(z2, x2, thin, min(r2 + (addendum_coeff + x2) * module, a_w - rf1 - c))
    a = a_w + center_clearance
    alpha_a = acos((r1 + r2) * cos(alpha) / a)  # pressure angle at the actual distance
    line = a * sin(alpha_a)  # length of the line of action between the base circles
    g1, g2 = sqrt(ra1**2 - rb1**2), sqrt(ra2**2 - rb2**2)
    return {
        "a_w": a_w,
        "a": a,
        "ra1": ra1,
        "rf1": rf1,
        "ra2": ra2,
        "rf2": rf2,
        "thin": thin,
        "contact_ratio": (g1 + g2 - line) / (pi * module * cos(alpha)),
        # tip of either gear must not reach past the other's base-circle tangent point
        "interference": g1 > line or g2 > line,
        "undercut_limit": 1 - z1 * sin(alpha) ** 2 / 2,
        "backlash_total": backlash + 2 * center_clearance * tan(alpha_a),
    }


meshes = {z: design_mesh(z) for z in (z_input, z_mid, z_last)}

shafts = [photo_shafts[0]]
for z_big, ang in zip((z_input, z_mid, z_last), layout_angles):
    x, y = shafts[-1]
    a = meshes[z_big]["a"]
    shafts.append((x + a * cos(radians(ang)), y + a * sin(radians(ang))))

# outermost radius on each shaft, then centre the train in the housing
reach = [
    meshes[z_input]["ra2"],
    max(meshes[z_mid]["ra2"], meshes[z_input]["ra1"]),
    max(meshes[z_last]["ra2"], meshes[z_mid]["ra1"]),
    max(meshes[z_last]["ra1"], collar_r),
]
x_min = min(x - r for (x, _), r in zip(shafts, reach))
x_max = max(x + r for (x, _), r in zip(shafts, reach))
y_min = min(y - r for (_, y), r in zip(shafts, reach))
y_max = max(y + r for (_, y), r in zip(shafts, reach))
housing_length = ceil(x_max - x_min + 2 * (wall + gear_margin))  # X
housing_depth = ceil(y_max - y_min + wall + 2 * gear_margin)  # Y, bottom wall at Y=0, open at top
dx = housing_length / 2 - (x_min + x_max) / 2
dy = wall + gear_margin - y_min
shafts = [(x + dx, y + dy) for x, y in shafts]


# ---------------------------------------------------------------- gears
def gear_profile(teeth, shift, thinning, ra, rf, flank_points=16):
    """Involute spur gear outline as a Face, first tooth centred on +X."""
    rb = module * teeth / 2 * cos(alpha)

    def psi(radius):
        return half_angle(teeth, shift, thinning, radius)

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


def big_profile(z_big):
    m = meshes[z_big]
    return gear_profile(z_big, big_shift, m["thin"], m["ra2"], m["rf2"])


def pinion_profile(z_big):
    """12T pinion matched to the big gear it meshes with."""
    m = meshes[z_big]
    return gear_profile(z_pinion, pinion_shift, m["thin"], m["ra1"], m["rf1"])


def gear_ring(profile, z0, rotation=0.0):
    ring = extrude(profile, face_width)
    return ring.rotate(Axis.Z, rotation).moved(Location((0, 0, z0)))


def mesh_phase(c1, c2, z1, rot1, z2):
    """Rotation (deg) for gear 2 so it meshes with gear 1 rotated rot1 (deg)."""
    beta = degrees(atan2(c2[1] - c1[1], c2[0] - c1[0]))
    delta = rot1 - beta  # how far gear 1's tooth is off the centre line
    return beta + 180 + 180 / z2 - delta * z1 / z2


def cyl(r, length, z0=0.0):
    return Cylinder(r, length, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Location((0, 0, z0)))


def bored(part):
    return part - cyl(shaft_d / 2, housing_height)


def hub(z_big, z0):
    """Solid hub between the two rings of a compound gear, as wide as the pinion root."""
    return cyl(meshes[z_big]["rf1"], plane_gap, z0)


rot1 = input_phase
rot2 = mesh_phase(shafts[0], shafts[1], z_input, rot1, z_pinion)
rot3 = mesh_phase(shafts[1], shafts[2], z_mid, rot2, z_pinion)
rot4 = mesh_phase(shafts[2], shafts[3], z_last, rot3, z_pinion)

# 02: input gear 30T, plane A, glue collar towards plane B
input_gear = bored(gear_ring(big_profile(z_input), plane_a, rot1) + cyl(collar_r, collar_len, plane_a + face_width))

# 03: compound 12T (plane A) + hub + 32T (plane B)
compound_1 = bored(
    gear_ring(pinion_profile(z_input), plane_a, rot2)
    + hub(z_input, plane_a + face_width)
    + gear_ring(big_profile(z_mid), plane_b, rot2)
)

# 04: compound 12T (plane B) + hub + 36T (plane C)
compound_2 = bored(
    gear_ring(pinion_profile(z_mid), plane_b, rot3)
    + hub(z_mid, plane_b + face_width)
    + gear_ring(big_profile(z_last), plane_c, rot3)
)

# 05: output pinion 12T (plane C) with glue collar below it
output_pinion = bored(cyl(collar_r, collar_len, plane_c - collar_len) + gear_ring(pinion_profile(z_last), plane_c, rot4))

gears = {
    "02_input_30T": (input_gear, shafts[0]),
    "03_compound_12T_32T": (compound_1, shafts[1]),
    "04_compound_12T_36T": (compound_2, shafts[2]),
    "05_output_12T": (output_pinion, shafts[3]),
}

# ---------------------------------------------------------------- housing
# box: side plates (Z), fixed end walls (X), bottom (Y=0); open at the top (+Y)
housing = Box(housing_length, housing_depth, housing_height, align=(Align.MIN, Align.MIN, Align.MIN))
housing -= Box(
    housing_length - 2 * wall, housing_depth - wall, inner_width, align=(Align.MIN, Align.MIN, Align.MIN)
).moved(Location((wall, wall, wall)))
for x, y in shafts:  # bearing holes straight through both side plates
    housing -= cyl((bearing_od + bearing_fit) / 2, housing_height).moved(Location((x, y, 0)))

# ---------------------------------------------------------------- crank and coupler
crank_arm = Box(crank_length, crank_width, crank_thickness, align=(Align.MIN, Align.MIN, Align.MIN))
for hx in (crank_hole_inset, crank_length - crank_hole_inset):
    crank_arm -= cyl(shaft_d / 2, crank_thickness).moved(Location((hx, crank_width / 2, 0)))
crank_arm -= extrude(
    RegularPolygon((crank_nut_af + crank_nut_fit) / 2, 6, major_radius=False), crank_nut_depth
).moved(Location((crank_hole_inset, crank_width / 2, crank_thickness - crank_nut_depth)))

grip_sleeve = cyl(grip_od / 2, grip_len) - cyl(shaft_d / 2, grip_len)

output_coupler = (
    cyl(coupler_d / 2, coupler_len)
    - cyl(shaft_d / 2, coupler_len)
    - Cylinder(shaft_d / 2, coupler_d, rotation=(0, 90, 0)).moved(Location((0, 0, coupler_cross_z)))
)

crank_z = -crank_gap - crank_thickness
crank_loc = Location((shafts[0][0] - crank_hole_inset, shafts[0][1] - crank_width / 2, crank_z))
grip_loc = Location((shafts[0][0] + crank_length - 2 * crank_hole_inset, shafts[0][1], crank_z - grip_len))
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
assembly = Compound(label="kumiveivi_v6_paadyt", children=list(parts.values()))

# ---------------------------------------------------------------- checks
problems = []
print(f"Module {module}, face width {face_width}, ratio 1:{(z_input / z_pinion) * (z_mid / z_pinion) * (z_last / z_pinion):g}")
print("Shaft centres:", [(round(x, 3), round(y, 3)) for x, y in shafts])

for z_big, m in meshes.items():
    if m["contact_ratio"] < 1.2:
        problems.append(f"{z_big}T: contact ratio {m['contact_ratio']:.2f} < 1.2")
    if m["interference"]:
        problems.append(f"{z_big}T: tip interference")
    if pinion_shift < m["undercut_limit"]:
        problems.append(f"12T undercut, shift {pinion_shift} < {m['undercut_limit']:.2f}")
    print(
        f"{z_big}T -> 12T: a_w {m['a_w']:.3f} (+{center_clearance}) mm, contact ratio {m['contact_ratio']:.2f}, "
        f"backlash {m['backlash_total']:.2f} mm, pinion OD {2 * m['ra1']:.2f} root {2 * m['rf1']:.2f}, "
        f"big OD {2 * m['ra2']:.2f} root {2 * m['rf2']:.2f}"
    )

names = list(placed_gears)
for i, a in enumerate(names):
    for b in names[i + 1 :] + ["01_housing"]:
        overlap = (parts[a] & parts[b]).volume
        if overlap > 1e-6:
            problems.append(f"3D collision {a} <-> {b}: {overlap:.3f} mm^3")

# M6 bolts straight through the housing must miss every other gear
for name, (_, (x, y)) in gears.items():
    rod = cyl(shaft_d / 2, housing_height).moved(Location((x, y, 0)))
    others = [n for n in placed_gears if n != name]
    gap, nearest = min((rod.distance_to(parts[n]), n) for n in others)
    if gap < 1.0:
        problems.append(f"bolt of {name} only {gap:.2f} mm from {nearest}")
    print(f"Bolt of {name}: nearest other gear {nearest} at {gap:.2f} mm")

bb = housing.bounding_box()
print(f"Housing {bb.size.X:.1f} x {bb.size.Y:.1f} x {bb.size.Z:.1f} mm, wall {wall}, open at +Y")
print(
    f"Gear tips to walls: left {x_min + dx - wall:.1f}, right {housing_length - wall - (x_max + dx):.1f}, "
    f"bottom {y_min + dy - wall:.1f}, top edge {housing_depth - (y_max + dy):.1f} mm"
)

# 2D running check: turn each pair through one full tooth pitch.
# Overlap must stay 0 (no jamming) and the closest flank gap must stay small at
# every position (teeth always in engagement, never jumping).
pairs = [
    ("30T -> 12T", z_input, 0, rot1, rot2),
    ("32T -> 12T", z_mid, 1, rot2, rot3),
    ("36T -> 12T", z_last, 2, rot3, rot4),
]
for label, z_big, i, r_big, r_small in pairs:
    big, small = big_profile(z_big), pinion_profile(z_big)
    (x1, y1), (x2, y2) = shafts[i], shafts[i + 1]
    worst_overlap, gaps = 0.0, []
    for k in range(mesh_steps):
        t = 360 / z_big * k / mesh_steps
        g1 = big.rotate(Axis.Z, r_big + t).moved(Location((x1, y1)))
        g2 = small.rotate(Axis.Z, r_small - t * z_big / z_pinion).moved(Location((x2, y2)))
        worst_overlap = max(worst_overlap, (g1 & g2).area)
        gaps.append(g1.distance_to(g2))
    max_gap_allowed = meshes[z_big]["backlash_total"]
    if worst_overlap > 1e-6:
        problems.append(f"{label}: teeth overlap {worst_overlap:.4f} mm^2 while turning")
    if max(gaps) > max_gap_allowed:
        problems.append(f"{label}: flank gap {max(gaps):.3f} mm > {max_gap_allowed:.2f} mm")
    print(
        f"{label}: centre {dist((x1, y1), (x2, y2)):.3f} mm, overlap {worst_overlap:.4f} mm^2, "
        f"flank gap {min(gaps):.3f}..{max(gaps):.3f} mm over one tooth pitch"
    )

print("ALL CHECKS OK" if not problems else "PROBLEMS:\n  " + "\n  ".join(problems))


# ---------------------------------------------------------------- export
def print_ready(part, rotation=(0, 0, 0)):
    """Rotate into print orientation, centre on XY origin, bottom at Z=0."""
    p = part.rotate(Axis.X, rotation[0]).rotate(Axis.Y, rotation[1])
    bb = p.bounding_box()
    return p.moved(Location((-bb.center().X, -bb.center().Y, -bb.min.Z)))


hl, hd, hh = round(housing_length), round(housing_depth), round(housing_height)
print_parts = {  # file name: (part, rotation about X / Y in deg)
    f"01_kotelo_{hl}x{hd}x{hh}mm_seina5_laakerireiat_12mm": (housing, (90, 0)),  # bottom on the bed, open up
    f"02_sisaantuloratas_{z_input}T_kaulus": (input_gear, (0, 0)),  # teeth down, collar up
    f"03_valiratas_{z_pinion}T-{z_mid}T": (compound_1, (180, 0)),  # big ring down
    f"04_valiratas_{z_pinion}T-{z_last}T": (compound_2, (180, 0)),  # big ring down
    f"05_ulostulopinjoni_{z_pinion}T_kaulus": (output_pinion, (180, 0)),  # teeth down, collar up
    f"06_veivin_varsi_{round(crank_thickness)}mm_mutteriupotus_M6": (crank_arm, (0, 0)),
    "07_veivin_kahva": (grip_sleeve, (0, 0)),
    "08_ulostulon_liitin": (output_coupler, (0, 0)),
}

output_dir.mkdir(parents=True, exist_ok=True)
for old in [*output_dir.glob("*.step"), *output_dir.glob("*.stl")]:  # drop renamed leftovers
    old.unlink()
export_step(assembly, output_dir / "00_kumiveivi_v6_paadyt_kokoonpano.step")
for name, (part, rotation) in print_parts.items():
    p = print_ready(part, rotation)
    export_step(p, output_dir / f"{name}.step")
    export_stl(p, output_dir / f"{name}.stl", tolerance=0.01, angular_tolerance=0.1)

try:
    show(assembly, **({"port": viewer_port} if viewer_port else {}))
except Exception as e:
    print(f"Viewer not available ({type(e).__name__}): {e} — set viewer_port at the top of the file")
