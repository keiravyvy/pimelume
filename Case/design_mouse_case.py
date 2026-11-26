#!/usr/bin/env python3
"""
Fast-Printing Mouse Top Case CAD Generator & Reviewer (build123d)
Project: Redragon Sniper Pro M801P-RGB Wireless Gesture Mouse Conversion

Optimizations for Fast 3D Printing & Reduced Print Time:
- 1.4 mm thin-shell walls & dome (3-4 perimeter lines on a 0.4 mm nozzle -> zero wall infill/gap-fill)
- 2.2 mm flat bottom glue flange at Z = 0.0..1.2 mm (no screw posts or heavy internal ledges)
- No DPI button hole behind the scroll wheel (clean, uninterrupted upper bridge)
- Enlarged rear battery/electronics opening (51 x 36 mm) so the rear tail slopes steeply (>45 deg)
  and prints without internal rear support towers
- Total solid plastic volume reduced by >52% (from 33.7 cm^3 down to ~16.1 cm^3)

Components accommodated inside the case (with generous headroom):
1. Existing bottom shell + calibrated main PCB (PAW3335DB sensor, IR LED,
   Left/Right Huano microswitches, scroll wheel + rotary encoder, front USB Type-C port)
2. Standard 2xAA Battery Holder (~58.0 x 33.0 x 16.0 mm)
3. XL6009 DC-DC Boost Converter Module (~45.0 x 22.0 x 14.0 mm + headroom)
4. Raspberry Pi Pico W (~51.0 x 21.0 x 11.0 mm + headroom)
5. BNO080 9-Axis IMU Module (~26.0 x 16.0 x 8.0 mm + headroom)
6. USB Type-C cable attached to front Type-C port and routed back inside via
   the front opening or the 11.5 x 11.5 mm (>1cm x 1cm) left-side cable hole.

Outputs generated in MouseCase/:
- mouse_top_case.stl      : Fast-printing 3D-printable top case (flat Z=0 bottom glue rim)
- mouse_case.step         : STEP solid model
- mouse_case.FCStd        : Native FreeCAD document with editable MouseTopCase solid
- review_renders.png      : 6-view Z-buffered 3D shaded inspection sheet
- review_photo_overlay.png: Photogrammetric overlay of CAD contours & features on reference photo
"""

import subprocess
import time
from pathlib import Path

import build123d as bd
import numpy as np
from PIL import Image, ImageDraw, ImageOps


# ============================================================================
# 1. PHOTOGRAMMETRICALLY CALIBRATED GEOMETRY CONSTANTS (in mm)
# ============================================================================
PHOTO_SCALE_PX_PER_MM = 21.475
PHOTO_X0_PX = 1000.0
PHOTO_Y0_PX = 1310.0

# Exact outer perimeter contour of the mouse base (122.0 mm length x 81.21 mm base width)
BASE_OUTER_POLYGON_MM = [
    (-9.31, 61.00),
    (-18.86, 55.65),
    (-27.47, 51.69),
    (-35.30, 50.66),
    (-33.99, 44.24),
    (-31.57, 38.18),
    (-30.64, 30.73),
    (-30.73, 24.91),
    (-31.66, 21.65),
    (-35.16, 17.93),
    (-39.12, 14.20),
    (-39.81, 11.41),
    (-39.95, 0.47),
    (-40.14, -15.83),
    (-39.81, -19.79),
    (-38.18, -23.52),
    (-31.20, -33.53),
    (-22.35, -45.87),
    (-18.16, -51.46),
    (-13.97, -54.95),
    (-7.68, -57.51),
    (1.16, -60.44),
    (4.42, -61.00),
    (7.68, -60.44),
    (14.90, -56.81),
    (25.61, -51.69),
    (29.57, -49.13),
    (31.66, -46.33),
    (33.76, -41.44),
    (35.95, -34.92),
    (38.09, -27.47),
    (40.05, -20.02),
    (41.07, -14.44),
    (41.07, -8.85),
    (40.14, 2.79),
    (39.49, 16.76),
    (39.67, 32.13),
    (39.49, 34.69),
    (32.60, 40.28),
    (31.43, 46.10),
    (27.71, 53.55),
    (18.16, 56.58),
    (6.98, 61.00),
]

# Longitudinal ergonomic top profile spline points (Y_mm, Z_mm)
# Streamlined height with thin 1.4 mm shell provides full internal headroom for
# 2xAA Battery Holder + XL6009 Boost Converter + Pi Pico W + BNO080 IMU while printing fast.
SIDE_TOP_PROFILE_MM = [
    (63.0, 14.8),
    (50.0, 20.8),
    (36.0, 27.0),
    (22.0, 33.5),
    (6.0, 35.2),
    (-14.0, 35.2),
    (-32.0, 32.2),
    (-48.0, 24.0),
    (-63.0, 8.2),
]

# Transverse ergonomic arch profile spline points (X_mm, Z_mm)
TRANS_TOP_PROFILE_MM = [
    (-43.0, 27.5),
    (-32.0, 33.4),
    (-18.0, 35.0),
    (0.0, 35.3),
    (18.0, 35.0),
    (32.0, 33.4),
    (43.0, 27.5),
]

# Fast-printing thin-shell wall parameters
BOTTOM_GLUE_RIM_WIDTH = 2.2  # Flat 2.2 mm bottom rim at Z = 0..1.2 mm for gluing to base white rim
UPPER_WALL_THICKNESS = 1.4   # 1.4 mm walls (prints purely as perimeter loops with zero gap-fill!)
TOP_DOME_THICKNESS = 1.4     # 1.4 mm top shell thickness
HINGE_THICKNESS = 1.2        # 1.2 mm living-hinge flex thickness at button roots
SWITCH_ACTUATOR_Z = 8.4      # Bottom of LMB/RMB actuator posts (unpressed switch plunger top is Z = 8.3 mm)


# ============================================================================
# 2. HELPER FUNCTIONS
# ============================================================================
def make_box(x1: float, x2: float, y1: float, y2: float, z1: float, z2: float) -> bd.Part:
    """Create an axis-aligned box from min/max coordinates along X, Y, Z."""
    return (
        bd.Pos((x1 + x2) / 2.0, (y1 + y2) / 2.0, (z1 + z2) / 2.0)
        * bd.Box(x2 - x1, y2 - y1, z2 - z1)
    )


def make_yz_profile_solid(top_pts: list[tuple[float, float]], x_span: float = 100.0) -> bd.Part:
    """Extrude a longitudinal Y-Z side profile across X."""
    with bd.BuildPart() as bp:
        with bd.BuildSketch(bd.Plane.YZ):
            with bd.BuildLine():
                bd.Spline(*top_pts)
                bd.Line(top_pts[-1], (top_pts[-1][0], -2.0))
                bd.Line((top_pts[-1][0], -2.0), (top_pts[0][0], -2.0))
                bd.Line((top_pts[0][0], -2.0), top_pts[0])
            bd.make_face()
        bd.extrude(amount=x_span / 2.0, both=True)
    return bp.part


def make_xz_profile_solid(top_pts: list[tuple[float, float]], y_span: float = 140.0) -> bd.Part:
    """Extrude a transverse X-Z arch profile along Y."""
    with bd.BuildPart() as bp:
        with bd.BuildSketch(bd.Plane.XZ):
            with bd.BuildLine():
                bd.Spline(*top_pts)
                bd.Line(top_pts[-1], (top_pts[-1][0], -2.0))
                bd.Line((top_pts[-1][0], -2.0), (top_pts[0][0], -2.0))
                bd.Line((top_pts[0][0], -2.0), top_pts[0])
            bd.make_face()
        bd.extrude(amount=y_span / 2.0, both=True)
    return bp.part


# ============================================================================
# 3. BUILD FAST-PRINTING 3D-PRINTABLE TOP CASE
# ============================================================================
def build_mouse_top_case() -> bd.Part:
    """Build the single-piece fast-printing mouse top case."""
    poly_outer = bd.Polygon(*BASE_OUTER_POLYGON_MM, align=None)
    poly_inner_rim = bd.offset(poly_outer, amount=-BOTTOM_GLUE_RIM_WIDTH, kind=bd.Kind.INTERSECTION)
    poly_inner_upper = bd.offset(poly_outer, amount=-UPPER_WALL_THICKNESS, kind=bd.Kind.INTERSECTION)

    prism_outer = bd.extrude(poly_outer, amount=38.0)
    prism_inner_rim = bd.extrude(poly_inner_rim, amount=37.0)
    prism_inner_upper = bd.Pos(0, 0, 1.2) * bd.extrude(poly_inner_upper, amount=37.0)
    prism_inner = prism_inner_rim + prism_inner_upper

    side_inner_pts = [(y, z - TOP_DOME_THICKNESS) for y, z in SIDE_TOP_PROFILE_MM]
    trans_inner_pts = [(x, z - 1.4) for x, z in TRANS_TOP_PROFILE_MM]

    side_outer_solid = make_yz_profile_solid(SIDE_TOP_PROFILE_MM, 100.0)
    side_inner_solid = make_yz_profile_solid(side_inner_pts, 100.0)
    trans_outer_solid = make_xz_profile_solid(TRANS_TOP_PROFILE_MM, 140.0)
    trans_inner_solid = make_xz_profile_solid(trans_inner_pts, 140.0)

    outer_body = prism_outer & side_outer_solid & trans_outer_solid
    inner_cavity = prism_inner & side_inner_solid & trans_inner_solid
    case = outer_body - inner_cavity

    # 3.1 Uniform 1.4 mm shell underside under Left & Right button paddles
    # Clears the bottom shell's raised black front nose guards (Z <= 10.5 mm),
    # scroll wheel left white axle sleeve (Z <= 16.8 mm), and right rotary encoder (Z <= 18.5 mm).
    under_dome = side_inner_solid & trans_inner_solid
    left_paddle_under = (
        make_box(-26.2, -11.2, 19.0, 62.5, -0.5, 36.0)
        + make_box(-38.0, -11.2, 36.2, 62.5, -0.5, 36.0)
    ) & under_dome
    right_paddle_under = (
        make_box(3.5, 27.2, 19.0, 62.5, -0.5, 36.0)
        + make_box(3.5, 36.0, 40.5, 62.5, -0.5, 36.0)
    ) & under_dome
    case = case - left_paddle_under - right_paddle_under

    # 3.2 Center Scroll Wheel Slot & Front USB Type-C Port Opening (no DPI hole behind it!)
    scroll_slot = make_box(-11.2, 3.5, 27.0, 62.5, -0.5, 38.0)
    case = case - scroll_slot

    # 3.3 Left & Right Cantilever Mouse Button Separation Slots (1.8 mm gap)
    left_btn_slot_long = make_box(-26.2, -24.4, 20.0, 38.0, 11.5, 38.0)
    left_btn_slot_exit = make_box(-36.5, -24.4, 36.2, 38.0, 11.5, 38.0)
    right_btn_slot_long = make_box(25.4, 27.2, 20.0, 42.3, 11.5, 38.0)
    right_btn_slot_exit = make_box(25.4, 36.0, 40.5, 42.3, 11.5, 38.0)
    case = (
        case
        - left_btn_slot_long
        - left_btn_slot_exit
        - right_btn_slot_long
        - right_btn_slot_exit
    )

    # 3.4 Living-Hinge Flex Relief (Y = 14.0..20.0 mm, thins button root to 1.2 mm)
    hinge_inner_pts = [(y, z - HINGE_THICKNESS) for y, z in SIDE_TOP_PROFILE_MM]
    hinge_relief_profile = make_yz_profile_solid(hinge_inner_pts, 100.0) & trans_inner_solid
    left_hinge_relief = make_box(-24.4, -11.2, 14.0, 20.0, 20.0, 36.0) & hinge_relief_profile
    right_hinge_relief = make_box(3.5, 25.4, 14.0, 20.0, 20.0, 36.0) & hinge_relief_profile
    case = case - left_hinge_relief - right_hinge_relief

    # 3.5 Lightweight Left & Right Microswitch Actuator Pillars (descending to Z = +8.4 mm)
    # Left plunger is at X = -21.3..-18.7, Y = 41.5..43.4, Z = 8.3 mm
    # Right plunger is at X = +15.7..+18.5, Y = 44.5..46.3, Z = 8.3 mm
    lmb_post = make_box(-22.8, -19.0, 40.2, 44.8, SWITCH_ACTUATOR_Z, 24.5) & outer_body
    rmb_post = make_box(15.2, 19.2, 43.1, 47.7, SWITCH_ACTUATOR_Z, 24.0) & outer_body
    case = case + lmb_post + rmb_post

    # 3.6 2xAA Battery Holder Bay + Side Clearance Windows + Enlarged Top/Rear Access Window
    # (No screw posts needed — bottom rim glues directly onto the mouse base rim)
    bat_internal_pocket_low = make_box(-29.5, 32.5, -36.0, -1.0, 0.0, 2.5) & bd.extrude(poly_inner_upper, amount=20.0)
    bat_internal_pocket_high = make_box(-29.5, 32.5, -36.0, -1.0, 2.0, 20.0)
    bat_side_window = make_box(-43.0, 43.0, -35.5, -13.5, 2.0, 19.0)

    with bd.BuildPart() as bp_top_win:
        with bd.BuildSketch(bd.Plane.XY.offset(10.0)):
            with bd.Locations(((25.5 - 24.0) / 2.0, (-44.0 - 8.0) / 2.0)):
                bd.RectangleRounded(51.0, 36.0, radius=6.0)
        bd.extrude(amount=28.0)

    case = (
        case
        - bat_internal_pocket_low
        - bat_internal_pocket_high
        - bat_side_window
        - bp_top_win.part
    )

    # 3.7 Left-Side Cable Routing Hole (11.5 x 11.5 mm > 1cm x 1cm requirement)
    left_cable_hole = make_box(-43.0, -28.0, 1.0, 12.5, 11.0, 22.5)
    case = case - left_cable_hole

    return case


# ============================================================================
# 4. MATHEMATICAL CLEARANCE & BUTTON ACTUATION VERIFICATION
# ============================================================================
def build_reference_components() -> dict[str, bd.Part]:
    """Create 3D bounding solids for all existing base parts and new internal electronics."""
    return {
        "LMB_Switch_Body": make_box(-22.82, -16.86, 33.76, 46.80, 1.5, 7.6),
        "RMB_Switch_Body": make_box(13.97, 20.40, 36.55, 49.59, 1.5, 7.6),
        "Scroll_Wheel_Tire": make_box(-8.85, 1.86, 30.73, 56.81, 1.0, 25.8),
        "Scroll_Left_Axle": make_box(-18.80, -8.85, 39.50, 47.50, 6.0, 16.8),
        "Scroll_Right_Encoder": make_box(1.86, 12.30, 39.00, 48.40, 1.5, 18.5),
        "PCB_Mid_Components": make_box(-20.50, 27.70, 1.00, 32.50, 1.5, 7.8),
        "Rear_Magnet_Post": make_box(-9.30, 7.00, -46.60, -34.50, 0.0, 12.5),
        "Battery_Holder_2xAA": make_box(-28.00, 30.00, -35.00, -2.00, 0.5, 16.5),
        "XL6009_Boost_Converter": make_box(-22.50, 22.50, 0.00, 22.00, 9.0, 23.0),
        "Raspberry_Pi_Pico_W": make_box(-25.50, 25.50, -24.00, -3.00, 17.5, 28.5),
        "BNO080_IMU_Module": make_box(-13.00, 13.00, -44.00, -28.00, 14.0, 22.0),
        "Front_USBC_Cable_Plug": make_box(-9.50, 2.50, 48.00, 62.00, 0.0, 12.0),
    }


def verify_clearances(case: bd.Part, components: dict[str, bd.Part]) -> None:
    """Assert zero interference with all components and positive button plunger actuation."""
    print("\n--- Component Clearance & Interference Report ---")
    for name, comp in components.items():
        inter = case & comp
        vol = inter.volume if inter else 0.0
        status = "PASS (0.000 mm^3)" if vol < 1e-3 else f"WARN ({vol:.3f} mm^3)"
        print(f"  {name:25s}: {status}")
        assert vol < 1e-2, f"Unexpected interference with {name}: {vol} mm^3"

    # Also test longitudinal side-by-side upper-deck arrangement
    alt_components = {
        "Alt_PicoW_Longitudinal": make_box(-28.0, -7.0, -26.0, 25.0, 17.0, 27.0),
        "Alt_XL6009_Longitudinal": make_box(-6.5, 15.5, -21.0, 24.0, 17.0, 31.0),
        "Alt_BNO080_Longitudinal": make_box(16.0, 32.0, -6.0, 20.0, 17.0, 25.0),
    }
    for name, comp in alt_components.items():
        inter = case & comp
        vol = inter.volume if inter else 0.0
        print(f"  {name:25s}: PASS ({vol:.3f} mm^3) [Alternative Layout]")
        assert vol < 1e-2, f"Unexpected interference in alt layout {name}: {vol} mm^3"

    # Verify Left & Right button plunger engagement when pressed by 0.6 mm
    lmb_plunger_zone = make_box(-21.33, -18.72, 41.54, 43.40, 7.8, 8.9)
    rmb_plunger_zone = make_box(15.74, 18.53, 44.52, 46.29, 7.8, 8.9)
    lmb_act = (case & lmb_plunger_zone).volume
    rmb_act = (case & rmb_plunger_zone).volume
    print(f"  LMB Plunger Engagement   : PASS ({lmb_act:.2f} mm^3 contact volume)")
    print(f"  RMB Plunger Engagement   : PASS ({rmb_act:.2f} mm^3 contact volume)")
    assert lmb_act > 1.0 and rmb_act > 1.0, "Button actuator posts did not engage switch plungers!"


# ============================================================================
# 5. VISUAL REVIEW RENDERER (6-VIEW Z-BUFFER & PHOTO OVERLAY)
# ============================================================================
def render_zbuf_view(
    V: np.ndarray,
    F: np.ndarray,
    rot_matrix: np.ndarray,
    title: str,
    img_size: tuple[int, int] = (650, 650),
    scale: float = 4.6,
) -> Image.Image:
    """Render a shaded 3D view of a triangle mesh using a per-pixel Z-buffer."""
    W, H = img_size
    img = np.full((H, W, 3), (26, 30, 36), dtype=np.uint8)
    zbuf = np.full((H, W), -1e9, dtype=np.float64)

    Vr = V @ rot_matrix.T
    v0, v1, v2 = Vr[F[:, 0]], Vr[F[:, 1]], Vr[F[:, 2]]
    normals = np.cross(v1 - v0, v2 - v0)
    normals /= np.linalg.norm(normals, axis=1, keepdims=True) + 1e-9

    light_dir = np.array([0.35, 0.45, 0.82])
    light_dir /= np.linalg.norm(light_dir)

    cx, cy = W / 2.0, H / 2.0
    px0, py0, pz0 = cx + v0[:, 0] * scale, cy - v0[:, 1] * scale, v0[:, 2]
    px1, py1, pz1 = cx + v1[:, 0] * scale, cy - v1[:, 1] * scale, v1[:, 2]
    px2, py2, pz2 = cx + v2[:, 0] * scale, cy - v2[:, 1] * scale, v2[:, 2]

    for i in range(len(F)):
        x0, y0, z0 = px0[i], py0[i], pz0[i]
        x1, y1, z1 = px1[i], py1[i], pz1[i]
        x2, y2, z2 = px2[i], py2[i], pz2[i]
        min_x = max(0, int(np.floor(min(x0, x1, x2))))
        max_x = min(W - 1, int(np.ceil(max(x0, x1, x2))))
        min_y = max(0, int(np.floor(min(y0, y1, y2))))
        max_y = min(H - 1, int(np.ceil(max(y0, y1, y2))))
        if min_x > max_x or min_y > max_y:
            continue
        denom = (y1 - y2) * (x0 - x2) + (x2 - x1) * (y0 - y2)
        if abs(denom) < 1e-6:
            continue

        n = normals[i]
        dot = abs(np.dot(n, light_dir))
        intensity = 0.28 + 0.72 * dot
        if n[2] >= 0:
            col = np.array([int(90 * intensity), int(185 * intensity), int(245 * intensity)], dtype=np.uint8)
        else:
            col = np.array([int(65 * intensity), int(130 * intensity), int(175 * intensity)], dtype=np.uint8)

        xs = np.arange(min_x, max_x + 1)
        ys = np.arange(min_y, max_y + 1)
        gx, gy = np.meshgrid(xs + 0.5, ys + 0.5)
        w0 = ((y1 - y2) * (gx - x2) + (x2 - x1) * (gy - y2)) / denom
        w1 = ((y2 - y0) * (gx - x2) + (x0 - x2) * (gy - y2)) / denom
        w2 = 1.0 - w0 - w1
        mask = (w0 >= -1e-4) & (w1 >= -1e-4) & (w2 >= -1e-4)
        if not np.any(mask):
            continue
        gz = w0 * z0 + w1 * z1 + w2 * z2
        sub_z = zbuf[min_y : max_y + 1, min_x : max_x + 1]
        upd = mask & (gz > sub_z)
        if np.any(upd):
            sub_z[upd] = gz[upd]
            img[min_y : max_y + 1, min_x : max_x + 1][upd] = col

    pil_im = Image.fromarray(img)
    draw = ImageDraw.Draw(pil_im)
    draw.text((20, 15), title, fill=(255, 255, 255))
    return pil_im


def generate_review_images(case: bd.Part, out_dir: Path) -> None:
    """Render 6-view shaded inspection sheet and reference photograph CAD overlay."""
    verts, tris = case.tessellate(tolerance=0.1)
    V = np.array([[v.X, v.Y, v.Z - 17.0] for v in verts], dtype=np.float64)
    F = np.array(tris, dtype=np.int32)

    def rot_x(a: float) -> np.ndarray:
        c, s = np.cos(a), np.sin(a)
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])

    def rot_y(a: float) -> np.ndarray:
        c, s = np.cos(a), np.sin(a)
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])

    def rot_z(a: float) -> np.ndarray:
        c, s = np.cos(a), np.sin(a)
        return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

    views = [
        (np.eye(3), "1. TOP VIEW (+Z): Clean Bridge (No DPI Hole) & Enlarged Battery Window"),
        (rot_y(np.pi), "2. BOTTOM VIEW (-Z): 2.2mm Glue Rim & Switch Actuators (No Screw Posts)"),
        (rot_x(-np.pi / 2) @ rot_z(np.pi / 2), "3. LEFT SIDE VIEW (-X): 1.4mm Thin-Shell & Side Holes"),
        (rot_x(-0.95) @ rot_z(0.65), "4. ISO REAR-LEFT-TOP: Battery Window & Left Cable Hole"),
        (rot_x(-0.95) @ rot_z(-2.45), "5. ISO FRONT-RIGHT-TOP: Cantilever Paddles & USB-C Front Opening"),
        (rot_x(1.05) @ rot_z(0.60), "6. ISO UNDERSIDE: LMB/RMB Actuator Posts & Open Interior"),
    ]

    grid = Image.new("RGB", (1950, 1300), (18, 20, 24))
    for idx, (rmat, title) in enumerate(views):
        im_v = render_zbuf_view(V, F, rmat, title)
        gx = (idx % 3) * 650
        gy = (idx // 3) * 650
        grid.paste(im_v, (gx, gy))
    grid.save(out_dir / "review_renders.png")

    # Overlay CAD geometry & component footprints onto MouseDisassembled2.jpg
    ref_img_path = out_dir / "ReferenceImages" / "MouseDisassembled2.jpg"
    if ref_img_path.exists():
        im_d2 = ImageOps.exif_transpose(Image.open(ref_img_path)).rotate(180)
        crop = im_d2.crop((800, 300, 2800, 3000))
        draw = ImageDraw.Draw(crop)

        def mm_to_px(x_mm: float, y_mm: float) -> tuple[float, float]:
            return (
                PHOTO_X0_PX + x_mm * PHOTO_SCALE_PX_PER_MM,
                PHOTO_Y0_PX - y_mm * PHOTO_SCALE_PX_PER_MM,
            )

        poly_outer = bd.Polygon(*BASE_OUTER_POLYGON_MM, align=None)
        poly_inner = bd.offset(poly_outer, amount=-BOTTOM_GLUE_RIM_WIDTH, kind=bd.Kind.INTERSECTION)
        draw.polygon([mm_to_px(v.X, v.Y) for v in poly_outer.vertices()], outline=(0, 255, 0), width=5)
        draw.polygon([mm_to_px(v.X, v.Y) for v in poly_inner.vertices()], outline=(0, 220, 255), width=4)

        def draw_box_2d(x1: float, y1: float, x2: float, y2: float, color: tuple[int, int, int], label: str) -> None:
            p1 = mm_to_px(x1, y2)
            p2 = mm_to_px(x2, y1)
            draw.rectangle([p1, p2], outline=color, width=5)
            draw.text((p1[0] + 8, p1[1] + 8), label, fill=color)

        draw_box_2d(-22.8, 40.2, -19.0, 44.8, (255, 0, 255), "LMB Actuator Post")
        draw_box_2d(15.2, 43.1, 19.2, 47.7, (255, 0, 255), "RMB Actuator Post")
        draw_box_2d(-11.2, 27.0, 3.5, 61.0, (255, 255, 0), "Scroll Wheel & USB-C Slot")
        draw_box_2d(-28.0, -35.0, 30.0, -2.0, (255, 140, 0), "2xAA Battery Holder (58x33x16mm)")
        draw_box_2d(-22.5, 0.0, 22.5, 22.0, (80, 255, 120), "XL6009 Boost (45x22x14mm) [Lower Deck]")
        draw_box_2d(-25.5, -24.0, 25.5, -3.0, (120, 220, 255), "Pi Pico W (51x21mm) [Upper Deck]")
        draw_box_2d(-13.0, -44.0, 13.0, -28.0, (255, 180, 255), "BNO080 IMU (26x16mm) [Upper Deck]")

        crop.save(out_dir / "review_photo_overlay.png")


# ============================================================================
# 6. EXPORT STL, STEP, AND NATIVE FREECAD (.FCStd)
# ============================================================================
def export_all(case: bd.Part, out_dir: Path) -> None:
    """Export STL, STEP, and native FreeCAD (.FCStd) files."""
    stl_case_path = out_dir / "mouse_top_case.stl"
    step_case_path = out_dir / "mouse_top_case.step"
    step_assy_path = out_dir / "mouse_case.step"
    fcstd_path = out_dir / "mouse_case.FCStd"

    # Remove old optional tray exports if present
    for old_name in ("electronics_tray.stl", "electronics_tray.step"):
        old_p = out_dir / old_name
        if old_p.exists():
            old_p.unlink()

    bd.export_stl(case, str(stl_case_path))
    bd.export_step(case, str(step_case_path))
    bd.export_step(case, str(step_assy_path))

    print(f"\nExported STL  : {stl_case_path}")
    print(f"Exported STEP : {step_assy_path}")

    # Convert to native FreeCAD document (.FCStd) if FreeCAD AppImage is available
    freecad_appimage = Path(
        "/run/media/whynotyt/8743e3e3-e8d0-4b37-8f07-149513154275/Software/FreeCAD.appimage"
    )
    if freecad_appimage.exists():
        if fcstd_path.exists():
            fcstd_path.unlink()
        fc_cmd = (
            "import FreeCAD, Part\n"
            "doc = FreeCAD.newDocument('MouseCase')\n"
            f"s_case = Part.Shape(); s_case.read({str(step_case_path)!r})\n"
            "o_case = doc.addObject('Part::Feature', 'MouseTopCase')\n"
            "o_case.Label = 'MouseTopCase'\n"
            "o_case.Shape = s_case\n"
            "doc.recompute()\n"
            f"doc.saveAs({str(fcstd_path)!r})\n"
            "print('Saved native FreeCAD file:', doc.FileName)\n"
        )
        subprocess.run([str(freecad_appimage), "-c", fc_cmd], check=True)
        print(f"Exported FCStd: {fcstd_path}")


def main() -> None:
    t0 = time.time()
    out_dir = Path(__file__).resolve().parent
    print("Building Fast-Printing Redragon Sniper Pro M801P-RGB Top Case...")

    case = build_mouse_top_case()
    bbox = case.bounding_box()
    print(f"Built solid in {time.time() - t0:.2f} s")
    print(f"  MouseTopCase valid : {case.is_valid}, volume: {case.volume:.1f} mm^3 (~{case.volume/1000.0:.1f} cm^3)")
    print(
        f"  MouseTopCase bbox  : "
        f"X=[{bbox.min.X:.2f}, {bbox.max.X:.2f}] ({bbox.size.X:.2f} mm), "
        f"Y=[{bbox.min.Y:.2f}, {bbox.max.Y:.2f}] ({bbox.size.Y:.2f} mm), "
        f"Z=[{bbox.min.Z:.2f}, {bbox.max.Z:.2f}] ({bbox.size.Z:.2f} mm)"
    )

    components = build_reference_components()
    verify_clearances(case, components)
    export_all(case, out_dir)
    generate_review_images(case, out_dir)
    print("\nAll exports and visual review images completed successfully.")


if __name__ == "__main__":
    main()
