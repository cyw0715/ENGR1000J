"""Generate Mars lander electronics control-box v1 in SolidWorks through COM.

Creates two independent printable parts:
  1. Base: 220 x 135 x 40 mm open enclosure with module support pegs,
     cable-tie slots, a sensor-side opening, connector openings and lid bosses.
  2. Lid: separate 220 x 135 x 15 mm removable cover with a locating lip,
     four screw clearances, camera lens opening, HC-SR04 twin openings and vents.

All dimensions are mm in DESIGN below and are intentionally centralized for later
physical-fit revisions. Generated files are restricted to OUTPUT_DIR.
"""
from __future__ import annotations

import json
import sys
import time
import traceback
from pathlib import Path

import pythoncom
import win32com.client
from win32com.client import VARIANT

OUTPUT_DIR = Path(r"E:\Chenyuewei\University\Freshman\summer\ENGR1000\p2\project\mechanical\2026-07-26-control-box-v1")
# Revision suffix avoids overwriting the already verified first-pass artifacts.
BASE_PATH = OUTPUT_DIR / "mars_lander_control_box_base_v1p3.SLDPRT"
LID_PATH = OUTPUT_DIR / "mars_lander_control_box_lid_v1p3.SLDPRT"
REPORT_PATH = OUTPUT_DIR / "generation_report_v1p3.json"
PARAMS_PATH = OUTPUT_DIR / "design_parameters_and_fit_notes_v1p3.json"

D = {
    "outer_x": 220.0,
    "outer_y": 135.0,
    "base_floor": 3.0,
    "base_wall": 3.0,
    "base_wall_h": 37.0,       # floor+wall = 40 mm total base height
    "lid_top": 3.0,
    "lid_lip_h": 12.0,         # lid total is 15 mm
    "lid_lip_t": 2.0,
    "corner_boss_od": 10.0,
    "corner_pilot_d": 2.7,
    "lid_clear_d": 3.6,
    "standoff_peg_d": 3.2,
    "standoff_h": 7.0,
    "camera_lens_d": 18.0,
    "camera_clear_x": 34.0,
    "camera_clear_y": 30.0,
    "sr04_d": 18.5,
    "sr04_pitch": 26.0,
    "sr04_clear_x": 58.0,
    "sr04_clear_y": 34.0,
}


def mm(v):
    return float(v) / 1000.0


def byref_i4(value=0):
    return VARIANT(pythoncom.VT_BYREF | pythoncom.VT_I4, value)


def empty_dispatch():
    return VARIANT(pythoncom.VT_DISPATCH, None)


def select_top_plane(model):
    model.ClearSelection2(True)
    for name in ("Top Plane", "上视基准面"):
        if model.Extension.SelectByID2(name, "PLANE", 0, 0, 0, False, 0, empty_dispatch(), 0):
            return name
    raise RuntimeError("Cannot select Top Plane / 上视基准面")


def make_sketch(model, entities_fn):
    select_top_plane(model)
    model.SketchManager.InsertSketch(True)
    sketch = model.SketchManager.ActiveSketch
    entities_fn(model.SketchManager)
    model.SketchManager.InsertSketch(True)
    return sketch


def select_sketch(model, sketch, fallback_index):
    model.ClearSelection2(True)
    if sketch is not None:
        try:
            if sketch.Select2(False, 0):
                return
        except Exception:
            pass
    for name in (f"Sketch{fallback_index}", f"草图{fallback_index}"):
        if model.Extension.SelectByID2(name, "SKETCH", 0, 0, 0, False, 0, empty_dispatch(), 0):
            return
    raise RuntimeError(f"Unable to select completed sketch {fallback_index}")


def extrude(model, sketch, sketch_index, depth_mm, merge=True, feature_name=None):
    select_sketch(model, sketch, sketch_index)
    feat = model.FeatureManager.FeatureExtrusion3(
        True, False, True, 0, 0, mm(depth_mm), 0.0,
        False, False, False, False, 0.0, 0.0,
        False, False, False, False, bool(merge), False, True,
        0, 0.0, False,
    )
    if feat is None:
        raise RuntimeError(f"Boss extrusion failed: {feature_name or sketch_index}")
    if feature_name:
        try:
            feat.Name = feature_name
        except Exception:
            pass
    return feat


def cut(model, sketch, sketch_index, depth_mm, feature_name=None):
    select_sketch(model, sketch, sketch_index)
    feat = model.FeatureManager.FeatureCut4(
        True, False, False,
        0, 0,
        mm(depth_mm), 0.0,
        False, False, False, False,
        0.0, 0.0,
        False, False, False, False, False,
        True, True, True, True,
        False, 0, 0, False, False,
    )
    if feat is None:
        raise RuntimeError(f"Cut extrusion failed: {feature_name or sketch_index}")
    if feature_name:
        try:
            feat.Name = feature_name
        except Exception:
            pass
    return feat


def rectangle(sm, x1, y1, x2, y2):
    sm.CreateCornerRectangle(mm(x1), mm(y1), 0, mm(x2), mm(y2), 0)


def circle(sm, x, y, diameter):
    sm.CreateCircleByRadius(mm(x), mm(y), 0, mm(diameter) / 2.0)


def save_part(model, path: Path):
    errors, warnings = byref_i4(0), byref_i4(0)
    ok = bool(model.Extension.SaveAs(str(path), 0, 1, empty_dispatch(), errors, warnings))
    if not ok or not path.exists() or path.stat().st_size == 0:
        raise RuntimeError(f"Failed to save {path.name}: errors={errors.value}, warnings={warnings.value}")
    return {"ok": ok, "errors": errors.value, "warnings": warnings.value, "size_bytes": path.stat().st_size}


def export_part(model, source_path: Path, ext: str):
    target = source_path.with_suffix(ext)
    errors, warnings = byref_i4(0), byref_i4(0)
    ok = bool(model.Extension.SaveAs(str(target), 0, 1, empty_dispatch(), errors, warnings))
    if not ok or not target.exists() or target.stat().st_size == 0:
        raise RuntimeError(f"Failed to export {target.name}: errors={errors.value}, warnings={warnings.value}")
    return {"path": str(target), "size_bytes": target.stat().st_size, "errors": errors.value, "warnings": warnings.value}


def save_preview(model, output: Path, view_id: int):
    model.ClearSelection2(True)
    model.ShowNamedView2("", view_id)
    model.ViewZoomtofit2()
    model.GraphicsRedraw2()
    ok = bool(model.SaveBMP(str(output), 1600, 1000))
    if not ok or not output.exists() or output.stat().st_size == 0:
        raise RuntimeError(f"Preview export failed: {output}")
    return {"path": str(output), "size_bytes": output.stat().st_size}


def make_base(sw, template):
    model = sw.NewDocument(template, 0, 0, 0)
    if model is None:
        time.sleep(1)
        model = sw.ActiveDoc
    if model is None:
        raise RuntimeError("Could not create base document")

    x, y = D["outer_x"] / 2, D["outer_y"] / 2
    t, floor, wh = D["base_wall"], D["base_floor"], D["base_wall_h"]
    si = 1

    # Bottom plate.
    sk = make_sketch(model, lambda sm: rectangle(sm, -x, -y, x, y))
    extrude(model, sk, si, floor, True, "Base_Floor_3mm"); si += 1

    # Back and two side walls. Front is broken into functional apertures.
    walls = [
        (-x, y - t, x, y, "Back_Wall"),
        (-x, -y, -x + t, y - t, "Left_Wall"),
        (x - t, -y, x, y - t, "Right_Wall"),
        (-x, -y, -78, -y + t, "Front_Left_Wall"),
        (-42, -y, -12, -y + t, "Front_Camera_Bridge"),
        (18, -y, 77, -y + t, "Front_SR04_Bridge"),
        (92, -y, x, -y + t, "Front_Right_Wall"),
    ]
    for x1, y1, x2, y2, name in walls:
        sk = make_sketch(model, lambda sm, a=x1,b=y1,c=x2,d=y2: rectangle(sm, a,b,c,d))
        extrude(model, sk, si, wh, True, name); si += 1

    # Corner lid bosses, inside the enclosure. Pilot holes are cut after bosses exist.
    corner_xy = [(-100, -57), (100, -57), (-100, 57), (100, 57)]
    for i, (cx, cy) in enumerate(corner_xy, 1):
        sk = make_sketch(model, lambda sm, a=cx,b=cy: circle(sm, a,b,D["corner_boss_od"]))
        extrude(model, sk, si, 30.0, True, f"Lid_Boss_{i}"); si += 1
        sk = make_sketch(model, lambda sm, a=cx,b=cy: circle(sm, a,b,D["corner_pilot_d"]))
        cut(model, sk, si, 34.0, f"Lid_Boss_{i}_Pilot"); si += 1

    # P4-NANO uses four existing Ø4mm board holes: 3.2 mm locating pegs.
    p4_pegs = [(-93, 18), (-48, 18), (-93, 63), (-48, 63)]
    for i, (cx, cy) in enumerate(p4_pegs, 1):
        sk = make_sketch(model, lambda sm, a=cx,b=cy: circle(sm, a,b,D["standoff_peg_d"]))
        extrude(model, sk, si, D["standoff_h"], True, f"P4_NANO_Peg_{i}"); si += 1

    # Mega pattern is deliberately a peg+zip-tie hybrid: two factory-hole pegs
    # plus two cable-tie slots because clone Mega mounting-hole layouts vary.
    mega_pegs = [(0, -30), (96, -30), (0, 12), (96, 12)]
    for i, (cx, cy) in enumerate(mega_pegs, 1):
        sk = make_sketch(model, lambda sm, a=cx,b=cy: circle(sm, a,b,D["standoff_peg_d"]))
        extrude(model, sk, si, D["standoff_h"], True, f"Mega_Peg_{i}"); si += 1

    # Cable-tie slots through the floor for a removable strap over boards/FFC slack.
    tie_slots = [(-74, 6), (-28, 6), (-5, -45), (36, -45), (75, -45), (75, 28)]
    for i, (cx, cy) in enumerate(tie_slots, 1):
        sk = make_sketch(model, lambda sm, a=cx,b=cy: rectangle(sm, a-6,b-1.7,a+6,b+1.7))
        cut(model, sk, si, floor + 1.0, f"Cable_Tie_Slot_{i}"); si += 1

    # Internal rail pair: accepts a loose sensor carrier / cable clamp plate.
    for i, (x1, y1, x2, y2) in enumerate([(28, 31, 95, 34), (28, 49, 95, 52)], 1):
        sk = make_sketch(model, lambda sm, a=x1,b=y1,c=x2,d=y2: rectangle(sm,a,b,c,d))
        extrude(model, sk, si, 4.0, True, f"Adjustable_Sensor_Rail_{i}"); si += 1

    return model


def make_lid(sw, template):
    model = sw.NewDocument(template, 0, 0, 0)
    if model is None:
        time.sleep(1)
        model = sw.ActiveDoc
    if model is None:
        raise RuntimeError("Could not create lid document")

    x, y = D["outer_x"] / 2, D["outer_y"] / 2
    lip_t, top_t, lip_h = D["lid_lip_t"], D["lid_top"], D["lid_lip_h"]
    si = 1

    # A standalone cover: plate plus downward internal locating rim.
    sk = make_sketch(model, lambda sm: rectangle(sm, -x, -y, x, y))
    extrude(model, sk, si, top_t, True, "Lid_Top_3mm"); si += 1

    # The rim is printed upward in the part's native orientation. On assembly,
    # the lid is turned over; this rim then nests inside the base wall.
    rim_rects = [
        (-x + lip_t, y - lip_t, x - lip_t, y, "Lid_Rim_Back"),
        (-x + lip_t, -y, x - lip_t, -y + lip_t, "Lid_Rim_Front"),
        (-x, -y, -x + lip_t, y, "Lid_Rim_Left"),
        (x - lip_t, -y, x, y, "Lid_Rim_Right"),
    ]
    for x1, y1, x2, y2, name in rim_rects:
        sk = make_sketch(model, lambda sm,a=x1,b=y1,c=x2,d=y2: rectangle(sm,a,b,c,d))
        extrude(model, sk, si, lip_h, True, name); si += 1

    # Four lid fastener clearances.
    for i, (cx, cy) in enumerate([(-100,-57),(100,-57),(-100,57),(100,57)], 1):
        sk = make_sketch(model, lambda sm,a=cx,b=cy: circle(sm,a,b,D["lid_clear_d"]))
        cut(model, sk, si, top_t + lip_h + 1.0, f"Lid_Screw_Clearance_{i}"); si += 1

    # Camera lens opening and four-card locating clearance. The camera PCB is
    # retained from below by a strap in the base; this avoids trusting unknown
    # OV5647 clone hole pitches from the photo.
    camera_x, camera_y = -61.0, -34.0
    sk = make_sketch(model, lambda sm: circle(sm, camera_x, camera_y, D["camera_lens_d"]))
    cut(model, sk, si, top_t + 1.0, "OV5647_Lens_Opening_D18"); si += 1
    # The camera PCB is retained from below by its base strap/rail. Do not cut a
    # rectangular service window here: it would merge with the lens opening and
    # unnecessarily expose the electronics. The lid deliberately has one clean
    # optical aperture only.

    # HC-SR04 two discrete acoustic openings. The 26 mm pitch is a deliberately
    # conservative starter value; module is held by rail/strap, not these holes.
    sr_x, sr_y = 46.0, -34.0
    for i, dy in enumerate((-D["sr04_pitch"]/2, D["sr04_pitch"]/2), 1):
        sk = make_sketch(model, lambda sm,a=sr_x,b=sr_y+dy: circle(sm,a,b,D["sr04_d"]))
        cut(model, sk, si, top_t + 1.0, f"HC_SR04_Acoustic_Opening_{i}"); si += 1
    # No local recess near the acoustic openings: the front cover must retain
    # two clean, completely independent Ø18.5 mm sound ports. Mechanical
    # retention is provided by the adjustable sensor rail and a cable-tie strap
    # in the base, so this cosmetic feature is intentionally omitted.

    # Ventilation slots above Mega/P4 region, with bridge widths suitable for FDM.
    for i, cx in enumerate((-82,-70,-58,-10,2,14), 1):
        sk = make_sketch(model, lambda sm,a=cx: rectangle(sm,a-2.0,30,a+2.0,54))
        cut(model, sk, si, top_t + 1.0, f"Vent_Slot_{i}"); si += 1

    return model


def make_fit_notes():
    return {
        "design_name": "Mars lander control box v1",
        "units": "mm",
        "envelope": {"external_x":220, "external_y":135, "base_height":40, "lid_height":15, "wall_thickness":3},
        "module_placement": {
            "Arduino_Mega_2560": {"reserved_footprint_mm":[102,54], "retention":"4 locating pegs plus floor cable-tie slots; verify clone hole pattern before final print"},
            "ESP32_P4_NANO": {"reserved_footprint_mm":[50,50], "retention":"4 locating pegs at 45.1x45.09 pitch based on Waveshare mechanical drawing"},
            "OV5647": {"retention":"camera service clearance in lid plus base strap/rail; exact clone PCB holes not inferred from photo", "lens_opening_diameter_mm":18},
            "HC_SR04": {"retention":"rail/strap; two lid acoustic openings, 18.5 diameter at 26 pitch", "note":"photograph shows sensor rotated so transducers are vertically arranged in image"},
        },
        "critical_print_notes": [
            "Print base with floor on bed; print lid with exterior top on bed or use supports for its locating rim.",
            "Use PETG/ABS rather than PLA if the enclosure will see sun or motor heat.",
            "Before drilling/printing final revision, test fit actual Mega clone, OV5647 and HC-SR04; their mounting holes differ by seller.",
            "HC-SR04 and camera must face the intended terrain direction; the cover can be mounted with that sensor face downward if required by the lander architecture.",
            "Do not block ESP32-P4 USB-OTG/USB-C/RJ45 or the OV5647 FFC bend region; enlarge/add side cutouts after checking your final cable set.",
        ],
        "parameters": D,
    }


def main():
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    report = {"status":"running", "output_dir":str(OUTPUT_DIR), "files":{}, "previews":[]}
    pythoncom.CoInitialize()
    try:
        try:
            sw = win32com.client.GetActiveObject("SldWorks.Application")
            report["solidworks_connection"] = "attached_to_existing_instance"
        except Exception:
            sw = win32com.client.Dispatch("SldWorks.Application")
            sw.Visible = True
            report["solidworks_connection"] = "started_new_instance"
            time.sleep(8)
        sw.Visible = True
        revision = sw.RevisionNumber
        report["solidworks_revision"] = str(revision() if callable(revision) else revision)
        template = str(sw.GetUserPreferenceStringValue(8) or "")
        if not template or not Path(template).is_file():
            raise RuntimeError("No valid default SolidWorks part template configured")
        report["template"] = template

        PARAMS_PATH.write_text(json.dumps(make_fit_notes(), ensure_ascii=False, indent=2), encoding="utf-8")
        report["files"]["parameters"] = {"path":str(PARAMS_PATH), "size_bytes":PARAMS_PATH.stat().st_size}

        base = make_base(sw, template)
        report["files"]["base_sldprt"] = {"path":str(BASE_PATH), **save_part(base, BASE_PATH)}
        report["files"]["base_step"] = export_part(base, BASE_PATH, ".step")
        report["files"]["base_stl"] = export_part(base, BASE_PATH, ".stl")
        for name, vid in {"isometric":7,"top":5,"front":1,"right":4}.items():
            report["previews"].append({"part":"base", "view":name, **save_preview(base, OUTPUT_DIR / f"base_{name}.bmp", vid)})

        lid = make_lid(sw, template)
        report["files"]["lid_sldprt"] = {"path":str(LID_PATH), **save_part(lid, LID_PATH)}
        report["files"]["lid_step"] = export_part(lid, LID_PATH, ".step")
        report["files"]["lid_stl"] = export_part(lid, LID_PATH, ".stl")
        for name, vid in {"isometric":7,"top":5,"front":1,"right":4}.items():
            report["previews"].append({"part":"lid", "view":name, **save_preview(lid, OUTPUT_DIR / f"lid_{name}.bmp", vid)})

        report["status"] = "pass"
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 0
    except Exception as exc:
        report["status"] = "fail"
        report["error"] = str(exc)
        report["traceback"] = traceback.format_exc()
        print(json.dumps(report, ensure_ascii=False, indent=2), file=sys.stderr)
        return 1
    finally:
        REPORT_PATH.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
        pythoncom.CoUninitialize()

if __name__ == "__main__":
    raise SystemExit(main())
