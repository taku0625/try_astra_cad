"""Export the editable one-piece, zero-pitch MID-360 fixture from its FreeCAD document."""
import hashlib
import json
import math
import sys
import tempfile
from pathlib import Path

for location in ('/usr/lib/freecad/lib', '/usr/lib/freecad-python3/lib'):
    if Path(location).is_dir():
        sys.path.append(location)

import FreeCAD as App
import Mesh
import MeshPart
import Part

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'output/integrated_flat'
MODEL = OUT / 'fixture_mid360_z90_y0.FCStd'
HISTORICAL_STEP = ROOT / 'output/fixture_mid360_z90_5806d96177.step'
V = App.Vector


def check_camera(doc, fixture, spec):
    sheet = doc.Parameters
    rear_edge = sheet.BaseCX.Value - sheet.BaseL.Value / 2
    base_top = sheet.BaseT.Value
    port_local = V(0, spec['rear_face_y_source'], spec['rear_connector_z_source'])
    forward = App.Rotation(V(0, 0, 1), -90)
    fits = []
    for roll in (0, 180):
        rotation = App.Rotation(V(1, 0, 0), roll).multiply(forward)
        camera = doc.Camera.Shape.copy()
        camera.Placement = App.Placement(doc.Camera.Placement.Base, rotation)
        port = camera.Placement.multVec(port_local)
        plug = Part.makeCylinder(sheet.CableD.Value / 2, port.x - rear_edge + 1,
                                 V(rear_edge - 1, port.y, port.z), V(1, 0, 0))
        plug_overlap = fixture.common(plug).Volume
        assert plug_overlap < 1e-5, ('Blocked plug', roll, plug_overlap)

        overlaps = []
        for dx in (0, 0.25, 1, 3, 6, 10, 20, 40, 80):
            moved = camera.copy()
            moved.translate(V(dx, 0, 0))
            overlaps.append(fixture.common(moved).Volume)
        assert max(overlaps) < 1e-5, ('Camera installation collision', roll, overlaps)

        sole_area = supported = 0.
        rear_holes = []
        for face in camera.Faces:
            if not isinstance(face.Surface, Part.Plane):
                continue
            bounds = face.optimalBoundingBox()
            if abs(bounds.ZMin - base_top) < 1e-5 and abs(bounds.ZMax - base_top) < 1e-5:
                sole_area += face.Area
                supported += face.common(fixture).Area
            if abs(bounds.XMin - port.x) < 1e-5 and abs(bounds.XMax - port.x) < 1e-5:
                for wire in face.Wires:
                    hole = wire.optimalBoundingBox()
                    diameter = spec['rear_hole_entry_diameter_model']
                    if abs(hole.YLength - diameter) < 1e-4 and abs(hole.ZLength - diameter) < 1e-4:
                        rear_holes.append(hole.Center)
        assert sole_area > 0 and supported / sole_area > .8, ('Camera support', roll)
        assert len(rear_holes) == 4, ('Camera rear mounting holes', roll)
        for hole in rear_holes:
            seat = hole - V(sheet.BackT.Value, 0, 0)
            bore = Part.makeCylinder(sheet.CameraHoleD.Value / 2, sheet.BackT.Value, seat, V(1, 0, 0))
            ring = Part.makeCylinder(spec['assumed_screw_head_diameter'] / 2, .2, seat, V(1, 0, 0)).cut(bore)
            assert fixture.common(bore).Volume < 1e-5, ('Blocked camera screw', roll, hole)
            assert abs(fixture.common(ring).Volume - ring.Volume) < 1e-5, ('Incomplete screw seat', roll, hole)
        fits.append(dict(roll_deg=roll, connector_z_mm=port.z, plug_overlap_mm3=plug_overlap,
                         max_installation_overlap_mm3=max(overlaps), support_fraction=supported / sole_area,
                         rear_screw_seats_checked=len(rear_holes)))
    return fits


def export(doc, destination):
    part = doc.PrintableFixture
    shape = part.Shape
    assert shape.isValid() and len(shape.Solids) == 1
    stem = MODEL.stem
    step = destination / (stem + '.step')
    stl = destination / (stem + '.stl')
    assembly = destination / 'assembly.step'
    Part.export([part], str(step))
    Part.export([part, doc.Camera, doc.MID360, doc.CollarEnvelope, doc.CarbonPipeEnvelope], str(assembly))
    mesh = MeshPart.meshFromShape(Shape=shape, LinearDeflection=.06, AngularDeflection=.18, Relative=False)
    mesh.write(str(stl))

    solid = Part.read(str(step))
    printed = Mesh.Mesh(str(stl))
    assert solid.isValid() and len(solid.Solids) == 1 and math.isclose(solid.Volume, shape.Volume, rel_tol=1e-7)
    assert printed.isSolid() and printed.countComponents() == 1
    assert not printed.hasNonManifolds() and not printed.hasNonUniformOrientedFacets()
    assert abs(printed.BoundBox.ZMin) < 1e-5

    # The released part should equal the historical solid cut by the new cable opening.
    original = Part.read(str(HISTORICAL_STEP))
    opening = doc.CableCircularClearance.Shape.multiFuse([
        doc.CableUpperClearance.Shape, doc.CableVerticalExtension.Shape, doc.CableTeardropRoof.Shape])
    expected = original.cut(opening)
    difference = solid.cut(expected).Volume + expected.cut(solid).Volume
    assert difference < 1e-5, ('Geometry differs outside the requested cut', difference)

    sheet = doc.Parameters
    probe_width = sheet.CableD.Value + 2
    wall = doc.BackPlate.Shape.BoundBox
    section = Part.makeBox(.2, probe_width, wall.ZMax - .2, V(wall.Center.x - .1, -probe_width / 2, .1))
    aperture = section.cut(solid)
    assert len(aperture.Solids) == 1
    opening_bounds = aperture.optimalBoundingBox(False, False)
    params = json.loads((ROOT / 'cad/parameters.json').read_text())
    fits = check_camera(doc, solid, params['camera'])
    lidar = doc.MID360.Placement
    scan = lidar.Rotation.multVec(V(0, 1, 0))
    cable = lidar.Rotation.multVec(V(1, 0, 0))
    mount = lidar.multVec(V(0, params['lidar']['bottom_y_source'], 0))
    assert mount.Length < 1e-5, 'LiDAR mounting center'
    assert (scan - V(0, 0, -1)).Length < 1e-6, 'LiDAR must face straight down'
    assert (cable - V(0, -1, 0)).Length < 1e-6, 'LiDAR cable must face -Y'
    assert solid.common(doc.MID360.Shape).Volume < 1e-5

    bounds = solid.optimalBoundingBox(False, False)
    report = {
        'historical_commit': '71f4adf468578672e7c65175cbfeb1a04e145f48',
        'historical_step': HISTORICAL_STEP.name,
        'historical_step_sha256': hashlib.sha256(HISTORICAL_STEP.read_bytes()).hexdigest(),
        'removed_volume_mm3': original.Volume - solid.Volume,
        'difference_from_expected_cut_mm3': difference,
        'dimensions_mm': [bounds.XLength, bounds.YLength, bounds.ZLength],
        'printed_part_count': 1,
        'stl_closed_manifold': True,
        'stl_facets': printed.CountFacets,
        'stl_z_min_mm': printed.BoundBox.ZMin,
        'lidar_mount_rotation_y_deg': 0,
        'lidar_mount_rotation_z_deg': 90,
        'lidar_mount_center_mm': list(mount),
        'lidar_scan_direction': list(scan),
        'lidar_cable_direction': list(cable),
        'cable_opening_width_mm': opening_bounds.YLength,
        'cable_opening_height_mm': opening_bounds.ZLength,
        'cable_opening_z_mm': [opening_bounds.ZMin, opening_bounds.ZMax],
        'camera_mounts': fits,
        'sha256': {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in (MODEL, step, stl, assembly)},
    }
    (destination / 'validation.json').write_text(json.dumps(report, indent=2) + '\n')


def main():
    doc = App.openDocument(str(MODEL))
    try:
        doc.recompute()
        with tempfile.TemporaryDirectory(dir=OUT) as work:
            destination = Path(work)
            export(doc, destination)
            for file in destination.iterdir():
                file.replace(OUT / file.name)
    finally:
        App.closeDocument(doc.Name)
    print(f'Exported the flat one-piece fixture to {OUT}')


if __name__ == '__main__':
    main()
