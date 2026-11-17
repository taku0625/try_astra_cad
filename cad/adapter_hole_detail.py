"""Check the exported adapter's four screw paths and extract a real CAD section.

This does not regenerate or modify the model. Drawing geometry comes from the
exported STEP, and entry centers come from the un-drilled stock in the FCStd.
"""
import hashlib
import json
import math
import sys
from pathlib import Path

for location in ['/usr/lib/freecad/lib', '/usr/lib/freecad-python3/lib']:
    if Path(location).exists():
        sys.path.append(location)
import FreeCAD as App
import Part


def main():
    root = Path(__file__).resolve().parents[1]
    out = root/'output/split_assembly'
    report = json.loads((out/'validation.json').read_text())
    assert report['variant'] == 'split'
    shape = Part.read(str(out/'lidar_adapter.step'))
    assert shape.isValid() and len(shape.Solids) == 1
    doc = App.openDocument(str(out/'assembly.FCStd'))
    try:
        assert math.isclose(shape.Volume, doc.LiDARAdapter.Shape.Volume, rel_tol=1e-7)
        stock = doc.LiDARTiltWedge.Shape
        holes = sorted((h for h in report['holes'] if h['type'] == 'LIDAR'),
                       key=lambda h: (-round(h['xyz'][0], 6), h['xyz'][1]))
        assert len(holes) == 4
        checked = []
        for i, h in enumerate(holes, 1):
            point, seat, axis = [App.Vector(*h[key]) for key in ['xyz', 'seat_xyz', 'axis']]
            bore = Part.makeCylinder(h['diameter']/2-.02, h['seat_thickness']+.02,
                                     point-axis*.01, axis)
            driver = Part.makeCylinder(2.75, 90, seat, axis)
            bore_overlap, driver_overlap = shape.common(bore).Volume, shape.common(driver).Volume
            assert bore_overlap < 1e-5 and driver_overlap < 1e-5, ('Blocked screw path', i)
            # Find where the centerline exits the actual un-drilled adapter.
            path = stock.common(Part.makeLine(seat, seat+axis*100))
            entry = max((v.Point for v in path.Vertexes), key=lambda v: (v-seat).dot(axis))
            bounds = stock.optimalBoundingBox(False, False)
            if abs(entry.z-bounds.ZMax) < 1e-6:
                entry_face = 'top'
            else:
                assert abs(entry.x-bounds.XMin) < 1e-6, ('Unexpected entry face', i)
                entry_face = 'rear (-X)'
            checked.append(dict(id=f'H{i}',mount_center_mm=list(point),seat_center_mm=list(seat),
                                headward_axis=list(axis),entry_center_mm=list(entry),entry_face=entry_face,
                                bore_interference_mm3=bore_overlap,driver_interference_mm3=driver_overlap))
        assert [h['entry_face'] for h in checked] == ['top', 'top', 'rear (-X)', 'rear (-X)']
        # The positive-Y row contains H2/H4. Slicing the exported solid, rather
        # than drawing assumed channels, makes the section show the real voids.
        section_y = checked[1]['mount_center_mm'][1]
        assert math.isclose(section_y, checked[3]['mount_center_mm'][1], abs_tol=1e-6)
        loops = shape.slice(App.Vector(0, 1, 0), section_y)
        assert loops and all(w.isClosed() for w in loops)
        detail = dict(holes=checked,section_y_mm=section_y,
                      section_loops_xz=[[[v.x, v.z] for v in w.discretize(Deflection=.03)] for w in loops])
        (root/'.cache').mkdir(exist_ok=True)
        (root/'.cache/adapter-hole-detail.json').write_text(json.dumps(detail))
        verification = dict(source_step='lidar_adapter.step',
                            source_step_sha256=hashlib.sha256((out/'lidar_adapter.step').read_bytes()).hexdigest(),
                            source_fcstd_sha256=hashlib.sha256((out/'assembly.FCStd').read_bytes()).hexdigest(),
                            result='PASS - four separate through bores and clear M3 head/driver paths in exported STEP',
                            hole_count=4,top_entry_count=2,rear_entry_count=2,holes=checked,
                            section_y_mm=section_y,section_closed_loop_count=len(loops))
        (out/'hole_access_validation.json').write_text(json.dumps(verification,indent=2)+'\n')
        print('Verified adapter: 4 LiDAR holes; 2 top entries and 2 rear entries.')
    finally:
        App.closeDocument(doc.Name)


if __name__ == '__main__':
    main()
