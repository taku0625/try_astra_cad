"""Vertical sensor fixture from supplied STEP geometry. Units mm."""
import hashlib
import json
import math
import os
import zipfile
from pathlib import Path

os.environ.setdefault('MPLCONFIGDIR', '/tmp/stacked-cad-mpl')
os.environ.setdefault('XDG_CACHE_HOME', '/tmp/stacked-cad-cache')
import cadquery as cq
from OCP.BRepAdaptor import BRepAdaptor_Surface
import ezdxf
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, FancyBboxPatch
from matplotlib.backends.backend_pdf import PdfPages
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'output'
P = json.loads((ROOT / 'cad/parameters.json').read_text())
COLORS = {'plate': '#bdccd8', 'post_1': '#718da4', 'post_2': '#718da4',
          'cap': '#526f88', 'top_pad': '#343e46', 'camera': '#d79b46', 'lidar': '#3697a5'}


def box(dx, dy, dz, x, y, z):
    return cq.Workplane('XY').box(dx, dy, dz, centered=(True, True, False)).translate((x, y, z))


def cylinder(x, y, d, z, h):
    return cq.Workplane('XY').center(x, y).circle(d / 2).extrude(h).translate((0, 0, z))


def orient(shape, sensor):
    s = P[sensor]
    if sensor == 'camera':
        return shape.rotate((0, 0, 0), (0, 0, 1), s['rotation_z_deg'])
    return shape.rotate((0, 0, 0), (1, 0, 0), s['rotation_x_deg']).rotate(
        (0, 0, 0), (0, 0, 1), s['rotation_z_deg'])


def rotated_vector(sensor, xyz):
    v = orient(cq.Workplane('XY').newObject([cq.Vertex.makeVertex(*xyz)]), sensor)
    return v.val().Center().toTuple()


def build():
    b, c = P['plate'], P['clamp']
    t = b['thickness']
    cam = P['camera']; lid = P['lidar']
    camera = orient(cq.importers.importStep(str(ROOT/'cad'/cam['reference_file'])), 'camera')
    camera = camera.translate((*cam['origin_xy'], t-cam['original_bottom_z']))
    lidar = orient(cq.importers.importStep(str(ROOT/'cad'/lid['reference_file'])), 'lidar')
    # With +Y mapped to -Z, the original bottom plane becomes the upper mounting face.
    lidar = lidar.translate((*lid['origin_xy'], lid['mounting_face_z']+lid['original_bottom_y']))
    plate = box(b['length_x'], b['width_y'], t, b['center_x'], 0, 0).edges('|Z').fillet(b['corner_radius'])
    holes = [{'xy': [x,y], 'd': b['host_hole_diameter'], 'tag': 'HOST'} for x,y in b['host_holes_xy']]
    lx, ly = lid['origin_xy']
    holes += [{'xy': [lx+x,ly+y], 'd': lid['clearance_diameter'], 'tag': 'LIDAR',
               'cbore_d': b['lidar_counterbore_diameter'], 'cbore_depth': b['lidar_counterbore_depth'], 'cbore_side': 'TOP'}
              for x,y in lid['holes_local_xy']]
    cap = box(c['cap_depth_x'], 2*c['post_offset_y']+c['post_width_y'], c['cap_thickness'],
              c['center_x'], c['center_y'], t+c['post_height'])
    posts = {}
    for i, sign in enumerate([-1,1]):
        x, y = c['center_x'], c['center_y']+sign*c['post_offset_y']
        holes.append({'xy': [x,y], 'd': c['clearance_diameter'], 'tag': 'POST',
                      'cbore_d': b['camera_post_counterbore_diameter'], 'cbore_depth': b['camera_post_counterbore_depth'], 'cbore_side': 'BOTTOM'})
        post = box(c['post_depth_x'], c['post_width_y'], c['post_height'], x,y,t)
        for z in [t,t+c['post_height']-c['tap_depth']]:
            post = post.cut(cylinder(x,y,c['tap_drill_diameter'],z,c['tap_depth']))
        posts[f'post_{i+1}'] = post
        cap = cap.cut(cylinder(x,y,c['clearance_diameter'],t+c['post_height'],c['cap_thickness']))
    for hole in holes:
        x,y = hole['xy']; d = hole['d']; before = plate.val().Volume()
        plate = plate.cut(cylinder(x,y,d,0,t))
        removed = math.pi*(d/2)**2*t
        if 'cbore_d' in hole:
            cd, depth = hole['cbore_d'], hole['cbore_depth']
            assert cd>d and 0<depth<t
            z = t-depth if hole['cbore_side']=='TOP' else 0
            plate = plate.cut(cylinder(x,y,cd,z,depth))
            removed += math.pi*((cd/2)**2-(d/2)**2)*depth
        assert math.isclose(before-plate.val().Volume(), removed, rel_tol=1e-7), 'Overlapping or clipped holes'
    pad = box(c['pad_depth_x'], c['pad_width_y'], c['pad_compressed_thickness'],
              c['center_x'], c['center_y'], t+c['post_height']-c['pad_compressed_thickness'])
    return {'plate': plate, **posts, 'cap': cap, 'top_pad': pad, 'camera': camera, 'lidar': lidar}, holes


def overlap_volume(a, b):
    ab, bb = a.val().BoundingBox(), b.val().BoundingBox()
    if any(min(getattr(ab,k+'max'),getattr(bb,k+'max'))-max(getattr(ab,k+'min'),getattr(bb,k+'min')) < 1e-5 for k in 'xyz'):
        return 0.0
    return sum(s.Volume() for s in a.intersect(b).solids().vals())


def validate(parts, holes):
    b = P['plate']; t = b['thickness']
    for k,v in parts.items():
        assert len(v.solids().vals())>0 and all(s.isValid() for s in v.solids().vals()), k
    cb, lb = parts['camera'].val().BoundingBox(), parts['lidar'].val().BoundingBox()
    assert abs(cb.zmin-t)<5e-6 and abs(lb.zmax)<5e-6
    assert cb.zmin>lb.zmax
    assert P['camera']['origin_xy']==P['lidar']['origin_xy'], 'Sensors must be stacked at the same XY datum'
    axes = {'camera_forward': rotated_vector('camera',(0,1,0)),
            'lidar_scan_axis': rotated_vector('lidar',(0,1,0)),
            'camera_cable_outward': rotated_vector('camera',(0,-1,0)),
            'lidar_cable_outward': rotated_vector('lidar',(1,0,0))}
    for key, expected in {'camera_forward':(1,0,0),'lidar_scan_axis':(0,0,-1),
                          'camera_cable_outward':(-1,0,0),'lidar_cable_outward':(-1,0,0)}.items():
        assert math.dist(axes[key],expected)<1e-9, (key,axes[key])
    connector_centers = {}
    for key in ['camera', 'lidar']:
        s = P[key]
        q = rotated_vector(key, s['connector_axis_point_source'])
        dz = t-s['original_bottom_z'] if key == 'camera' else s['original_bottom_y']
        q = (q[0]+s['origin_xy'][0], q[1]+s['origin_xy'][1], q[2]+dz)
        connector_centers[key] = q
        found = False
        for face in parts[key].faces().vals():
            if face.geomType() != 'CYLINDER':
                continue
            cyl = BRepAdaptor_Surface(face.wrapped).Cylinder()
            location, direction = cyl.Location(), cyl.Axis().Direction()
            if (abs(cyl.Radius()-s['connector_cylinder_radius']) < 1e-5
                    and abs(direction.X()) > .999999
                    and math.dist((location.Y(), location.Z()), q[1:]) < 1e-5):
                found = True
                break
        assert found, f'{key}: transformed cable connector cylinder not found'
    # Match source MID-360 bottom-hole axes to the actual plate clearance holes.
    lid = P['lidar']; mapped = []
    for x in [-24.,24.]:
        for z in [-18.,18.]:
            q = rotated_vector('lidar',(x,lid['original_bottom_y'],z))
            q = (q[0]+lid['origin_xy'][0],q[1]+lid['origin_xy'][1],q[2]+lid['original_bottom_y'])
            assert abs(q[2])<1e-8
            assert min(math.dist(q[:2],h['xy']) for h in holes if h['tag']=='LIDAR')<1e-8
            mapped.append(q)
    collisions = {}; names = list(parts)
    for i,k in enumerate(names):
        for j in names[i+1:]:
            v = overlap_volume(parts[k],parts[j])
            if v>1e-5: collisions[k+' / '+j]=v
    assert not collisions, collisions
    # Verify recessed socket-head envelopes clear the plate AND both sensors.
    e = P['fastener_envelopes']; head_checks = []
    for h in holes:
        if h['tag']=='HOST': continue
        x,y = h['xy']
        if h['tag']=='POST':
            d, height = e['camera_post_head_diameter'], e['camera_post_head_height']
            z = h['cbore_depth']-height
        else:
            d, height = e['lidar_head_diameter'], e['lidar_head_height']
            z = t-h['cbore_depth']
        assert z>=0 and z+height<=t and d<h['cbore_d']
        head = cylinder(x,y,d,z,height)
        assert all(overlap_volume(head,v)<1e-5 for v in parts.values()), (h,z)
        head_checks.append({'tag':h['tag'],'xy':h['xy'],'head_z_range':[z,z+height]})
    return {'status':P['status'],'units':'mm','sensor_XY_datums':{k:P[k]['origin_xy'] for k in ['camera','lidar']},
            'camera_z_range':[cb.zmin,cb.zmax], 'lidar_z_range':[lb.zmin,lb.zmax],
            'sensor_mounting_face_separation_mm':cb.zmin-lb.zmax,
            'directions':axes, 'lidar_hole_alignment':'PASS - source 48 x 36 pattern matches inverted placement',
            'connector_centers_global':connector_centers,
            'connector_geometry_check':'PASS - transformed STEP connector cylinders aligned with X',
            'positive_volume_interferences':collisions,'recessed_head_checks':head_checks,
            'solids':{k:len(v.solids().vals()) for k,v in parts.items()},
            'metal_mass_kg_at_2700_kg_m3':sum(v.val().Volume()*2.7e-6 for k,v in parts.items() if k not in ['camera','lidar','top_pad']),
            'plate_holes':holes,'source_sha256':{k:hashlib.sha256((ROOT/'cad'/P[k]['reference_file']).read_bytes()).hexdigest() for k in ['camera','lidar']},
            'unresolved':P['unresolved']}


def drawing(parts, holes, report):
    b,c = P['plate'],P['clamp'];t=b['thickness']
    fig=plt.figure(figsize=(15,10),facecolor='white')
    fig.suptitle('C2-176DEG + MID-360 | Vertical stack',x=.05,ha='left',fontsize=23,weight='bold')
    fig.text(.05,.925,'CAMERA ABOVE / LiDAR FACING DOWN / BOTH CABLE PORTS TOWARD -X (REAR)',fontsize=11,color='#286678')
    ax=fig.add_subplot(121,projection='3d')
    faces=[];colors=[]
    for k,v in parts.items():
        vv,tt=v.val().tessellate(.35,.25)
        faces.extend([[vv[i].toTuple() for i in tri] for tri in tt]);colors.extend([COLORS[k]]*len(tt))
    ax.add_collection3d(Poly3DCollection(faces,facecolors=colors,edgecolors=colors,linewidths=0,shade=True,rasterized=True))
    ax.set(xlim=(-65,45),ylim=(-60,60),zlim=(-70,70),xlabel='X / forward (mm)',ylabel='Y (mm)',zlabel='Z (mm)')
    ax.set_box_aspect((110,120,140));ax.view_init(elev=18,azim=35)
    ax.set_title('Actual user STEP geometry',fontsize=11)
    # A side view makes the inverted orientation and rear connectors explicit.
    ax=fig.add_subplot(222)
    for k,v in parts.items():
        bb=v.val().BoundingBox()
        ax.add_patch(Rectangle((bb.xmin,bb.zmin),bb.xlen,bb.zlen,facecolor=COLORS[k],edgecolor=COLORS[k],alpha=.5))
    ax.text(0,31,'CAMERA',ha='center',fontsize=10)
    ax.text(0,-30,'MID-360',ha='center',fontsize=10)
    for point in report['connector_centers_global'].values():
        z = point[2]
        ax.annotate('',xy=(-67,z),xytext=(-41,z),arrowprops=dict(arrowstyle='->',color='#234f6d',lw=2))
    ax.text(-68,61,'Both ports rearward',ha='left',fontsize=9,color='#234f6d')
    ax.annotate('',xy=(64,30.5),xytext=(35,30.5),arrowprops=dict(arrowstyle='->',color='#9c641f'))
    ax.text(49.5,40,'Lens +X',ha='center',fontsize=8,color='#9c641f')
    ax.annotate('Scan -Z',xy=(0,-75),xytext=(0,-64),ha='center',fontsize=9,arrowprops=dict(arrowstyle='->',color='#21838e'))
    ax.axhline(0,color='#8498a4',ls=':',lw=.8)
    ax.set(xlim=(-75,80),ylim=(-85,75),title='SIDE | bounding outlines, mm');ax.set_aspect('equal');ax.axis('off')
    ax=fig.add_subplot(224)
    xmin=b['center_x']-b['length_x']/2;hy=b['width_y']/2
    ax.add_patch(FancyBboxPatch((xmin,-hy),b['length_x'],b['width_y'],boxstyle=f"round,pad=0,rounding_size={b['corner_radius']}",facecolor='#edf2f6',edgecolor='#708698'))
    for h in holes:
        x,y=h['xy'];ax.add_patch(Circle((x,y),h['d']/2,facecolor='white',edgecolor='#344e62'))
        if 'cbore_d' in h:
            ax.add_patch(Circle((x,y),h['cbore_d']/2,fill=False,edgecolor='#a16232' if h['cbore_side']=='TOP' else '#436dab',linestyle='-' if h['cbore_side']=='TOP' else '--'))
    ax.plot(0,0,'+',color='#234f6d');ax.text(0,7,'Shared XY datum',ha='center',fontsize=8)
    ax.text(-10,-64,f"{b['length_x']:g} x {b['width_y']:g} x {t:g} mm",ha='center',fontsize=10)
    ax.set(xlim=(-70,55),ylim=(-75,65),title='PLATE | top view, dashed = bottom counterbore');ax.set_aspect('equal');ax.axis('off')
    fig.text(.05,.075,'Camera rests on plate TOP (Z=8). Inverted LiDAR mounts against plate BOTTOM (Z=0).',fontsize=11)
    fig.text(.05,.045,'Review: screw engagement, clamp pressure, mating cables, field of view and vibration. Fasteners not included in CAD.',fontsize=9,color='#a04428')
    fig.subplots_adjust(top=.87,bottom=.14,wspace=.12,hspace=.24)
    fig.savefig(OUT/'assembly_preview.png',dpi=160)
    with PdfPages(OUT/'review_drawing.pdf') as pdf:
        pdf.savefig(fig)
        f2,a2=plt.subplots(figsize=(11.7,8.3));a2.axis('off')
        lines=['VERTICAL FIXTURE | Dimensions and assembly order (mm)','',
               'Axes: camera forward +X; camera cable -X; inverted LiDAR scan -Z, cable -X.',
               f"Plate: {b['length_x']:g}(X) x {b['width_y']:g}(Y) x {t:g}, center ({b['center_x']:g},0), R{b['corner_radius']:g} corners.",
               'Camera bottom Z=8; LiDAR upper mounting face Z=0; common sensor XY datum=(0,0).','',
               'PLATE HOLES:']
        for h in holes:
            x,y=h['xy'];line=f" {h['tag']:5s} ({x:5g},{y:5g}) dia {h['d']:g} THRU"
            if 'cbore_d' in h:line+=f"; counterbore dia {h['cbore_d']:g} x {h['cbore_depth']:g} deep from {h['cbore_side']}"
            lines.append(line)
        lines += ['',f"Posts x2: {c['post_depth_x']:g} x {c['post_width_y']:g} x {c['post_height']:g}; center (-8,+/-26.8).",
                  ' Both ends: dia 3.3 drill depth 8; M4 effective thread depth 6 proposed.',
                  f"Cap: {c['cap_depth_x']:g} x {2*c['post_offset_y']+c['post_width_y']:g} x {c['cap_thickness']:g}; 2 x dia 4.5, pitch 53.6.",
                  ' Cap underside Z=53.2; camera top Z=53; top pad 8 x 30 x 0.2 compressed.',
                  ' Source: camera rotZ=-90; LiDAR rotX=-90 then rotZ=180 (degrees).',
                  ' MID-360 mounting centers from STEP: original X=+/-24, Z=+/-18.',
                  ' STEP bores suggest M3; exact thread and permissible insertion need confirmation.','',
                  'ASSEMBLY ORDER (required because LiDAR covers lower post screw heads):',
                  ' 1. Attach both posts with M4x8 from BOTTOM counterbores (4.5 mm engagement).',
                  ' 2. Attach inverted LiDAR from TOP counterbores; M3x8 gives 3.5 mm insertion.',
                  '    M3x8 is conditional on manufacturer-approved insertion; confirm before use.',
                  ' 3. Place camera; install top pad and cap with 2 x M4x8 (4 mm engagement).',
                  ' 4. Attach host through 4 x dia 5.5 holes; M5 length depends on host.',
                  ' Lower post heads: dia 7 x 4; LiDAR heads: dia 5.5 x 3; both recessed 0.5.',
                  ' Tap geometry and fasteners not modeled. No bottom pads between sensors/plate.',
                  ' Deburr; support face flatness 0.05 and Ra3.2 proposed; validate requirements.',
                  f"Metal mass at 2700 kg/m3: {report['metal_mass_kg_at_2700_kg_m3']:.3f} kg.",
                  ' REVIEW: clamp friction/pressure, inverted operation, plugs/bends, FOV, loads.']
        a2.text(.01,.98,'\n'.join(lines),va='top',family='monospace',fontsize=8.4,linespacing=1.22)
        pdf.savefig(f2);plt.close(f2)
    plt.close(fig)


def main():
    OUT.mkdir(exist_ok=True)
    parts,holes=build();report=validate(parts,holes)
    for k in ['plate','post_1','post_2','cap','top_pad']:
        cq.exporters.export(parts[k],str(OUT/(k+'.step')))
        cq.exporters.export(parts[k],str(OUT/(k+'.stl')),tolerance=.05,angularTolerance=.1)
        rt=cq.importers.importStep(str(OUT/(k+'.step')))
        assert rt.val().isValid() and math.isclose(rt.val().Volume(),parts[k].val().Volume(),rel_tol=1e-7)
    # The plate top and bottom differ: provide one DXF per machined face.
    for name,shape in [('plate',parts['plate'].faces('>Z').wires()),
                       ('plate_bottom',parts['plate'].faces('<Z').wires()),('cap',parts['cap'].faces('<Z').wires())]:
        cq.exporters.export(shape,str(OUT/(name+'.dxf')))
        circles=list(ezdxf.readfile(OUT/(name+'.dxf')).modelspace().query('CIRCLE'))
        assert len(circles)==(2 if name=='cap' else len(holes))
    a=cq.Assembly(name='C2_MID360_VERTICAL_STACK')
    for k,v in parts.items():a.add(v,name=k,color=cq.Color(COLORS[k]))
    a.export(str(OUT/'assembly.step'))
    assembly=cq.importers.importStep(str(OUT/'assembly.step'))
    assert len(assembly.solids().vals())==sum(report['solids'].values())
    report['step_roundtrip']='PASS: 5 generated parts and 13-solid assembly'
    report['dxf_checks']='PASS: top plate 10 contours, bottom plate 10 contours, cap 2 holes; use PDF for drilling depths'
    drawing(parts,holes,report)
    (OUT/'validation.json').write_text(json.dumps(report,indent=2)+'\n')
    with zipfile.ZipFile(ROOT/'fixture_cad.zip','w',compression=zipfile.ZIP_DEFLATED,compresslevel=6) as archive:
        for name in ['README.md','requirements.txt']:
            archive.write(ROOT/name,name)
        for folder in ['cad','output']:
            for path in sorted((ROOT/folder).rglob('*')):
                if path.is_file() and '__pycache__' not in path.parts:archive.write(path,path.relative_to(ROOT))
    with zipfile.ZipFile(ROOT/'fixture_cad.zip') as archive:assert archive.testzip() is None
    print(json.dumps(report,indent=2))


if __name__=='__main__':
    main()
