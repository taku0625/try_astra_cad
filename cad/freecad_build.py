"""Build the native fixture: integrated or a bolted two-part assembly."""
import argparse
import hashlib
import json
import math
import sys
from pathlib import Path

# System Python must match the FreeCAD installation; this is not a CadQuery model.
for location in ['/usr/lib/freecad/lib', '/usr/lib/freecad-python3/lib']:
    if Path(location).exists():
        sys.path.append(location)
import FreeCAD as App
import Part
import Mesh
import MeshPart

ROOT = Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser()
parser.add_argument('--variant',choices=['integrated','split'],default='integrated')
args=parser.parse_args()
SPLIT=args.variant=='split'
OUT = ROOT/'output'/('split_assembly' if SPLIT else '')
CACHE = ROOT/'.cache'/('render-meshes-split.json' if SPLIT else 'render-meshes.json')
P = json.loads((ROOT/'cad/parameters.json').read_text())
V = App.Vector
D = App.newDocument('SplitFixtureAssembly' if SPLIT else 'SinglePrintedFixture')
S = D.addObject('Spreadsheet::Sheet', 'Parameters')
S.Label = 'Editable dimensions (mm)'
B,C,L,R = P['plate'],P['camera'],P['lidar'],P['rear_bracket']
M=P['pipe_mount']
J=P['assembly_joint'] if SPLIT else None
lidar_yaw=App.Rotation(V(0,0,1),L['mount_rotation_z_deg'])
lidar_pitch=App.Rotation(V(0,1,0),L['mount_rotation_y_deg'])
lidar_mount_rotation=lidar_pitch.multiply(lidar_yaw)
lidar_mount_origin=V(0,0,L['mount_center_z'])
lidar_normal=lidar_pitch.multVec(V(0,0,1))
lidar_hole_rotation=App.Rotation(V(0,0,1),lidar_normal)
assert -60 < L['mount_rotation_y_deg'] < 0, 'Wedge supports negative assembly Y pitch between -60 and 0 degrees'
assert L['screw_seat_thickness'] > 0
assert L['mount_center_z']+abs(math.tan(math.radians(L['mount_rotation_y_deg'])))*L['wedge_length_x']/2 < -1, 'Keep tilted mount below base'
camera_roll=C['roll_about_forward_deg']%360
assert camera_roll in (0,180), 'Camera assembly reference roll must be 0 or 180 degrees about +X'
camera_forward_rotation=App.Rotation(V(0,0,1),-90)
camera_rotations={roll:App.Rotation(V(1,0,0),roll).multiply(camera_forward_rotation) for roll in (0,180)}
# Both seating faces are equally far below the camera datum in their mounted pose.
camera_seat_z=-C['seating_plane_z_source']
cable_z=-abs(C['rear_connector_z_source'])-camera_seat_z
values = dict(BaseL=B['length_x'], BaseW=B['width_y'], BaseT=B['thickness'], BaseCX=B['center_x'], Radius=B['corner_radius'],
              CameraX=C['forward_offset'], CameraRear=C['rear_face_y_source'], BackT=R['thickness'], BackW=R['width'],
              BackH=R['height_above_base'], MountPitch=C['rear_mount_pitch'], CameraHoleD=R['camera_clearance_diameter'],
              CableD=R['cable_diameter'], CableUp=R['cable_upward_extension'],
              CameraSeatZ=camera_seat_z, CableZ=cable_z, BraceW=R['brace_width'], BraceD=R['brace_depth'], BraceH=R['brace_height'])
values.update(PipeOD=M['pipe_od'], PipeX=M['center_x'], PipeY=M['center_y'],
              SlotMin=M['slot_pitch_min'], SlotMax=M['slot_pitch_max'], SlotW=M['slot_width'])
for i,(key,val) in enumerate(values.items(),1):
    S.set(f'A{i}',key);S.set(f'B{i}',f'={val} mm');S.setAlias(f'B{i}',key)
S.setColumnWidth('A',180);S.setColumnWidth('B',100)
features=[]

def expr(o, prop, value):
    if isinstance(value,str):o.setExpression(prop,value)
    else:setattr(o,prop,value)

def primitive(kind,name):
    o=D.addObject(kind,name);features.append(o);return o

def place(o,x,y,z,rot=None):
    for key,val in zip('xyz',[x,y,z]):
        if isinstance(val,str):o.setExpression('Placement.Base.'+key,val)
    base=o.Placement.Base
    for key,val in zip('xyz',[x,y,z]):
        if not isinstance(val,str):setattr(base,key,val)
    p=o.Placement;p.Base=base
    if rot is not None:p.Rotation=rot
    o.Placement=p

def box(name,dx,dy,dz,x,y,z):
    o=primitive('Part::Box',name)
    for key,val in [('Length',dx),('Width',dy),('Height',dz)]:expr(o,key,val)
    place(o,x,y,z);return o

def cyl(name,r,h,x,y,z,rot=None):
    o=primitive('Part::Cylinder',name);expr(o,'Radius',r);expr(o,'Height',h);place(o,x,y,z,rot);return o

def fuse(name,items):
    o=primitive('Part::MultiFuse',name);o.Shapes=items;o.Refine=True;return o

# Rounded base constructed with editable native primitives.
a=box('BaseCenterX','Parameters.BaseL-2*Parameters.Radius','Parameters.BaseW','Parameters.BaseT',
      'Parameters.BaseCX-Parameters.BaseL/2+Parameters.Radius','-Parameters.BaseW/2',0)
b=box('BaseCenterY','Parameters.BaseL','Parameters.BaseW-2*Parameters.Radius','Parameters.BaseT',
      'Parameters.BaseCX-Parameters.BaseL/2','-Parameters.BaseW/2+Parameters.Radius',0)
basepieces=[a,b]
for i,(sx,sy) in enumerate([(-1,-1),(-1,1),(1,-1),(1,1)]):
    basepieces.append(cyl(f'BaseCorner{i}','Parameters.Radius','Parameters.BaseT',
                         f'Parameters.BaseCX+{sx}*(Parameters.BaseL/2-Parameters.Radius)',
                         f'{sy}*(Parameters.BaseW/2-Parameters.Radius)',0))
base=fuse('RoundedBase',basepieces)
back=box('BackPlate','Parameters.BackT','Parameters.BackW','Parameters.BackH+1 mm',
         'Parameters.CameraX+Parameters.CameraRear-Parameters.BackT','-Parameters.BackW/2','Parameters.BaseT-1 mm')
braces=[]
for i,y in enumerate(['-Parameters.BackW/2','Parameters.BackW/2-Parameters.BraceW']):
    w=primitive('Part::Wedge',f'RearBrace{i}')
    for key,val in dict(Xmin=0,Xmax='Parameters.BraceW',Ymin=0,Ymax='Parameters.BraceD',Zmin=0,Zmax='Parameters.BraceH',
                        X2min=0,X2max='Parameters.BraceW',Z2min=0,Z2max=0).items():expr(w,key,val)
    place(w,'Parameters.CameraX+Parameters.CameraRear-Parameters.BackT+1 mm',y,'Parameters.BaseT-1 mm',App.Rotation(V(0,0,1),90))
    braces.append(w)
# The enlarged rounded base includes the pipe mount at the same thickness and level.
# Native box/boolean wedge: horizontal top overlaps the base; bottom is the
# LiDAR mating plane. The same rigid transform drives sensor and fasteners.
wedge_bottom=L['mount_center_z']-abs(math.tan(math.radians(L['mount_rotation_y_deg'])))*L['wedge_length_x']/2-1
wedge_stock=box('LiDARWedgeStock',L['wedge_length_x'],L['wedge_width_y'],(0 if SPLIT else 1)-wedge_bottom,
                -L['wedge_length_x']/2,-L['wedge_width_y']/2,wedge_bottom)
if SPLIT:
    wings=[]
    for i,sign in enumerate([-1,1]):
        y=J['wing_inner_y'] if sign>0 else -J['wing_outer_y']
        wings.append(box(f'JointWing{i}',J['wing_x_max']-J['wing_x_min'],
                         J['wing_outer_y']-J['wing_inner_y'],J['flange_thickness'],
                         J['wing_x_min'],y,-J['flange_thickness']))
    wedge_stock=fuse('AdapterStock',[wedge_stock,*wings])
clip_origin=lidar_mount_origin+lidar_pitch.multVec(V(-150,-150,-150))
wedge_clip=box('LiDARPlaneTrim',300,300,150,clip_origin.x,clip_origin.y,clip_origin.z)
wedge_clip.Placement=App.Placement(clip_origin,lidar_pitch)
wedge=primitive('Part::Cut','LiDARTiltWedge');wedge.Base=wedge_stock;wedge.Tool=wedge_clip;wedge.Refine=True
stock=fuse('BaseStock' if SPLIT else 'OnePieceStock',[base,back,*braces]+([] if SPLIT else [wedge]))
tools=[];holes=[]
lidar_tools=[]
tools.append(cyl('PipeClearance','(Parameters.PipeOD+0.6 mm)/2','Parameters.BaseT+2 mm',
                 'Parameters.PipeX','Parameters.PipeY',-1))
# Two radial slots for a provisional axial two-hole attachment. Not certified product dimensions.
for i,sign in enumerate([-1,1]):
    y0='Parameters.PipeY'+(' - Parameters.SlotMax/2' if sign<0 else ' + Parameters.SlotMin/2')
    parts=[box(f'CollarSlotWeb{i}','Parameters.SlotW','(Parameters.SlotMax-Parameters.SlotMin)/2',
               'Parameters.BaseT+2 mm','Parameters.PipeX-Parameters.SlotW/2',y0,-1)]
    for j,pitch in enumerate(['SlotMin','SlotMax']):
        parts.append(cyl(f'CollarSlotEnd{i}_{j}','Parameters.SlotW/2','Parameters.BaseT+2 mm',
                         'Parameters.PipeX',f'Parameters.PipeY+{sign}*Parameters.{pitch}/2',-1))
    tools.append(fuse(f'CollarSlot{i}',parts))
for i,(x,y) in enumerate(B['host_holes_xy']):
    tools.append(cyl(f'HostHole{i}',B['host_hole_diameter']/2,'Parameters.BaseT+2 mm',x,y,-1))
    holes.append(dict(type='HOST',xy=[x,y],diameter=B['host_hole_diameter']))
lidar_hole_centers=[]
for x in [-L['hole_pitch_x']/2,L['hole_pitch_x']/2]:
    for y in [-L['hole_pitch_y']/2,L['hole_pitch_y']/2]:
        pt=lidar_mount_origin+lidar_mount_rotation.multVec(V(x,y,0))
        lidar_hole_centers.append(pt)
for i,pt in enumerate(lidar_hole_centers):
    bore_start=pt-lidar_normal
    seat=pt+lidar_normal*L['screw_seat_thickness']
    lidar_tools.append(cyl(f'LiDARHole{i}',L['clearance_diameter']/2,L['screw_seat_thickness']+2,
                     bore_start.x,bore_start.y,bore_start.z,lidar_hole_rotation))
    # Deep head/driver channel; its flat bottom is perpendicular to the screw.
    lidar_tools.append(cyl(f'LiDARCounterbore{i}',L['counterbore_diameter']/2,100,
                     seat.x,seat.y,seat.z,lidar_hole_rotation))
    holes.append(dict(type='LIDAR',xy=[pt.x,pt.y],xyz=list(pt),axis=list(lidar_normal),
                      diameter=L['clearance_diameter'],counterbore=L['counterbore_diameter'],
                      seat_thickness=L['screw_seat_thickness'],seat_xyz=list(seat)))
# Rear camera fasteners: holes extracted from the source back-face loops.
for i,(sy,sz) in enumerate([(-1,-1),(-1,1),(1,-1),(1,1)]):
    tools.append(cyl(f'CameraRearHole{i}','Parameters.CameraHoleD/2','Parameters.BackT+2 mm',
                     'Parameters.CameraX+Parameters.CameraRear-Parameters.BackT-1 mm',
                     f'{sy}*Parameters.MountPitch/2',f'Parameters.BaseT-Parameters.CameraSeatZ+{sz}*Parameters.MountPitch/2',
                     App.Rotation(V(0,0,1),V(1,0,0))))
# The vertical slot spans both connector positions, with a 45-degree roof for FDM.
# Extend to the rear edge: the low connector requires a 0.5 mm shallow base relief
# so a nominal dia 16 plug can approach the opening horizontally from behind.
cable=cyl('CableCircularClearance','Parameters.CableD/2',
          'Parameters.CameraX+Parameters.CameraRear-Parameters.BaseCX+Parameters.BaseL/2+2 mm',
          'Parameters.BaseCX-Parameters.BaseL/2-1 mm',0,'Parameters.BaseT+Parameters.CableZ',
          App.Rotation(V(0,0,1),V(1,0,0)))
upper_cable=cyl('CableUpperClearance','Parameters.CableD/2','Parameters.BackT+2 mm',
                'Parameters.CameraX+Parameters.CameraRear-Parameters.BackT-1 mm',0,
                'Parameters.BaseT+Parameters.CableZ+Parameters.CableUp',App.Rotation(V(0,0,1),V(1,0,0)))
slot=box('CableVerticalExtension','Parameters.BackT+2 mm','Parameters.CableD','Parameters.CableUp',
         'Parameters.CameraX+Parameters.CameraRear-Parameters.BackT-1 mm','-Parameters.CableD/2',
         'Parameters.BaseT+Parameters.CableZ')
diamond=primitive('Part::Prism','CableTeardropDiamond');diamond.Polygon=4
expr(diamond,'Circumradius','Parameters.CableD/sqrt(2)');expr(diamond,'Height','Parameters.BackT+2 mm')
place(diamond,'Parameters.CameraX+Parameters.CameraRear-Parameters.BackT-1 mm',0,
      'Parameters.BaseT+Parameters.CableZ+Parameters.CableUp',App.Rotation(V(1,1,1),120))
clip=box('CableRoofClip','Parameters.BackT+2 mm','2*Parameters.CableD','Parameters.CableD',
         'Parameters.CameraX+Parameters.CameraRear-Parameters.BackT-1 mm','-Parameters.CableD',
         'Parameters.BaseT+Parameters.CableZ+Parameters.CableUp+Parameters.CableD/(2*sqrt(2))')
roof=primitive('Part::Common','CableTeardropRoof');roof.Base=diamond;roof.Tool=clip;roof.Refine=True
tools.extend([cable,upper_cable,slot,roof])
if SPLIT:
    adapter_tools=list(lidar_tools)
    for i,(x,y) in enumerate(J['holes_xy']):
        tools.append(cyl(f'JointBaseHole{i}',J['clearance_diameter']/2,'Parameters.BaseT+2 mm',x,y,-1))
        adapter_tools.append(cyl(f'JointAdapterHole{i}',J['clearance_diameter']/2,
                                 J['flange_thickness']+2,x,y,-J['flange_thickness']-1))
        pocket=primitive('Part::Prism',f'JointNutPocket{i}');pocket.Polygon=6
        pocket.Circumradius=J['nut_pocket_across_flats']/math.sqrt(3)
        pocket.Height=J['nut_pocket_depth']+1
        place(pocket,x,y,-J['flange_thickness']-1)
        adapter_tools.append(pocket)
        holes.append(dict(type='JOINT',xy=[x,y],diameter=J['clearance_diameter']))
    adapter_cutters=fuse('AdapterOpenings',adapter_tools)
    adapter=primitive('Part::Cut','LiDARAdapter');adapter.Base=wedge;adapter.Tool=adapter_cutters;adapter.Refine=True
    adapter.Label='PRINT 2 - 30 degree LiDAR adapter'
else:
    tools.extend(lidar_tools)
cutters=fuse('ClearanceOpenings',tools)
fixture=primitive('Part::Cut','PrintableBase' if SPLIT else 'PrintableFixture');fixture.Base=stock;fixture.Tool=cutters;fixture.Refine=True
fixture.Label='PRINT 1 - common base' if SPLIT else 'ONE PRINTED PART - PLA'
D.recompute()
print_parts=[('base',fixture),('lidar_adapter',adapter)] if SPLIT else [('fixture',fixture)]
for name,obj in print_parts:
    assert not obj.Shape.isNull() and obj.Shape.isValid() and len(obj.Shape.Solids)==1, (name,'must be one valid solid')
if SPLIT:
    # Tight bounds matter for trimmed, oblique cylinders; OCCT face tolerances
    # can pad a planar bound by 1e-7 mm even though its vertices are at Z=0.
    assert abs(fixture.Shape.optimalBoundingBox().ZMin)<1e-6 and abs(adapter.Shape.optimalBoundingBox().ZMax)<1e-6
    assert fixture.Shape.common(adapter.Shape).Volume<1e-5, 'Printed parts overlap'

# Show the selected camera pose in the assembly and validate both poses below.
# MID-360 yaw is followed by assembly Y pitch, about its mounting center.
refs={}
camera_source=Part.read(str(ROOT/'cad'/C['reference_file']))
for name,key in [('Camera','camera'),('MID360','lidar')]:
    o=D.addObject('Part::Feature',name);o.Label='REFERENCE '+name
    if key=='camera':
        o.Shape=camera_source
        o.Placement=App.Placement(V(C['forward_offset'],0,B['thickness']-camera_seat_z),camera_rotations[camera_roll])
        o.setExpression('Placement.Base.x','Parameters.CameraX')
        o.setExpression('Placement.Base.z','Parameters.BaseT-Parameters.CameraSeatZ')
    else:
        o.Shape=Part.read(str(ROOT/'cad'/P[key]['reference_file']))
        r=lidar_mount_rotation.multiply(App.Rotation(V(0,0,1),180).multiply(App.Rotation(V(1,0,0),-90)))
        o.Placement=App.Placement(lidar_mount_origin+lidar_pitch.multVec(V(0,0,L['bottom_y_source'])),r)
    refs[key]=o
D.recompute()
for o in features:o.Visibility=False
fixture.Visibility=True
if SPLIT:adapter.Visibility=True
for o in refs.values():o.Visibility=True
S.Visibility=False

# Simplified bought-part envelopes; deliberately not labeled as exact manufacturer geometry.
mount_refs=[]
for name,outer,inner,z,height in [
    ('CollarEnvelope',M['collar_od_reference'],M['pipe_od'],-M['collar_height_reference'],M['collar_height_reference']),
    ('CarbonPipeEnvelope',M['pipe_od'],M['pipe_id_reference'],M['pipe_top_z']-M['pipe_length_reference'],M['pipe_length_reference'])]:
    o=D.addObject('Part::Feature',name)
    o.Label='UNVERIFIED ENVELOPE '+name
    origin=V(M['center_x'],M['center_y'],z)
    o.Shape=Part.makeCylinder(outer/2,height,origin).cut(Part.makeCylinder(inner/2,height,origin))
    # These reference envelopes are regenerated from JSON, not spreadsheet expressions.
    o.Visibility=True
    mount_refs.append(o)
D.recompute()

# Native geometry and fabrication checks.
shape=fixture.Shape.fuse(adapter.Shape).removeSplitter() if SPLIT else fixture.Shape
interferences={k:shape.common(o.Shape).Volume for k,o in refs.items()}
for o in mount_refs:
    assert shape.common(o.Shape).Volume<1e-5, ('Mount interference',o.Name)
    for sensor in refs.values():
        assert o.Shape.common(sensor.Shape).Volume<1e-5, ('Reference collision',o.Name,sensor.Name)
# Continuous base retained, with an integral tilted mounting wedge below it.
assert shape.BoundBox.ZMin < L['mount_center_z']
bridge=Part.makeBox(1,10,B['thickness'],V(-60,20,0))
assert abs(shape.common(bridge).Volume-bridge.Volume)<1e-5
assert D.getObject('PipeMountPad') is None
assert M['slot_pitch_min']/2-M['slot_width']/2-(M['pipe_od']+0.6)/2 >= 1.4
# Check both ends of each slot, including a nominal M5 shaft and top-side driver access.
for sign in [-1,1]:
    for pitch in [M['slot_pitch_min'],M['slot_pitch_max']]:
        x,y=M['center_x'],M['center_y']+sign*pitch/2
        probe=Part.makeCylinder(2.5,12,V(x,y,-1))
        assert shape.common(probe).Volume<1e-5
        driver=Part.makeCylinder(5,40,V(x,y,B['thickness']+0.01))
        assert shape.common(driver).Volume<1e-5
        for sensor in refs.values(): assert sensor.Shape.common(driver).Volume<1e-5
assert all(v<1e-5 for v in interferences.values()),interferences
camdir=refs['camera'].Placement.Rotation.multVec(V(0,1,0))
liddir=refs['lidar'].Placement.Rotation.multVec(V(0,1,0))
camport=refs['camera'].Placement.Rotation.multVec(V(0,-1,0))
lidport=refs['lidar'].Placement.Rotation.multVec(V(1,0,0))
cam_source_z=refs['camera'].Placement.Rotation.multVec(V(0,0,1))
assert (camdir-V(1,0,0)).Length<1e-8 and (liddir-lidar_pitch.multVec(V(0,0,-1))).Length<1e-8
assert (camport-V(-1,0,0)).Length<1e-8 and (lidport-lidar_mount_rotation.multVec(V(-1,0,0))).Length<1e-8
plug_low_z=B['thickness']+cable_z
plug_high_z=plug_low_z+R['cable_upward_extension']
plug_radius=R['cable_diameter']/2
back_x=C['forward_offset']+C['rear_face_y_source']
rear_start=B['center_x']-B['length_x']/2-1
probe_length=back_x-rear_start+0.1
plug_probe=Part.makeCylinder(plug_radius-0.02,probe_length,V(rear_start,0,plug_low_z),V(1,0,0))
# Check rear access over the full range of heights, including both connector positions.
upper_probe=plug_probe.copy();upper_probe.translate(V(0,0,R['cable_upward_extension']))
bridge_probe=Part.makeBox(probe_length,R['cable_diameter']-0.04,R['cable_upward_extension'],
                          V(rear_start,-plug_radius+0.02,plug_low_z))
plug_envelope=plug_probe.fuse(upper_probe).fuse(bridge_probe)
assert shape.common(plug_envelope).Volume<1e-5, 'Extended cable approach blocked by base or rear plate'
assert C['screw_diameter']<R['camera_clearance_diameter']<C['assumed_screw_head_diameter']
screw_insertion=C['screw_length_under_head']-R['thickness']
assert 0<screw_insertion<C['rear_hole_axis_first_obstruction_mm'], 'Nominal screw exceeds STEP axial opening'
# Inspect the supplied camera back-face loops, rather than guessing a thread pattern.
rear_holes=[]
for face in camera_source.Faces:
    bb=face.optimalBoundingBox()
    if abs(bb.YMin-C['rear_face_y_source'])<1e-5 and abs(bb.YMax-C['rear_face_y_source'])<1e-5:
        for wire in face.Wires:
            wb=wire.optimalBoundingBox();cx=(wb.XMin+wb.XMax)/2;cz=(wb.ZMin+wb.ZMax)/2
            if abs(abs(cx)-C['rear_mount_pitch']/2)<1e-4 and abs(abs(cz)-C['rear_mount_pitch']/2)<1e-4:
                assert abs(wb.XLength-C['rear_hole_entry_diameter_model'])<1e-4
                rear_holes.append((cx,cz))
assert len(rear_holes)==4,rear_holes
camera_fits={}
lidar_top=refs['lidar'].Shape.optimalBoundingBox().ZMax
for roll,rotation in camera_rotations.items():
    placement=App.Placement(refs['camera'].Placement.Base,rotation)
    camera_shape=camera_source.copy()
    camera_shape.Placement=placement
    camera_bounds=camera_shape.optimalBoundingBox()
    assert camera_bounds.ZMin>lidar_top
    assert abs(camera_bounds.YMin+camera_bounds.YMax)<1e-4
    overlap=interferences['camera'] if roll==camera_roll else shape.common(camera_shape).Volume
    assert overlap<1e-5, ('Camera interferes with fixture',roll,overlap)
    connector=placement.multVec(V(0,C['rear_face_y_source'],C['rear_connector_z_source']))
    assert abs(connector.y)<1e-8 and plug_low_z-1e-8<=connector.z<=plug_high_z+1e-8, (
        'Cable opening does not reach the camera connector',roll,connector.z)

    rear_global=[]
    for x,z in rear_holes:
        pt=placement.multVec(V(x,C['rear_face_y_source'],z))
        rear_global.append(list(pt))
        assert abs(pt.x-back_x)<1e-8
        head_base=V(pt.x-R['thickness'],pt.y,pt.z)
        bore=Part.makeCylinder(R['camera_clearance_diameter']/2-0.02,R['thickness']+0.1,
                              head_base-V(0.1,0,0),V(1,0,0))
        assert shape.common(bore).Volume<1e-5, ('Camera screw bore blocked',roll,pt)
        approach=Part.makeCylinder(2.75,40,head_base-V(40,0,0),V(1,0,0))
        assert shape.common(approach).Volume<1e-5, ('Camera driver access blocked',roll,pt)
        # Preserve the full bearing annulus around each screw beside the slot.
        bearing=Part.makeCylinder(C['assumed_screw_head_diameter']/2,0.2,head_base,V(1,0,0)).cut(
            Part.makeCylinder(R['camera_clearance_diameter']/2,0.2,head_base,V(1,0,0)))
        assert abs(shape.common(bearing).Volume-bearing.Volume)<1e-5, ('Incomplete screw head bearing',roll,pt)
        shaft=Part.makeCylinder(C['screw_diameter']/2,C['screw_length_under_head'],head_base,V(1,0,0))
        assert shape.common(shaft).Volume<1e-5, ('Camera screw shaft blocked',roll,pt)

    # Measure underside contact on the real camera geometry in this orientation.
    full_foot_area=0.
    supported_area=0.
    for face in camera_shape.Faces:
        bb=face.optimalBoundingBox()
        if isinstance(face.Surface,Part.Plane) and abs(bb.ZMin-B['thickness'])<1e-5 and abs(bb.ZMax-B['thickness'])<1e-5:
            full_foot_area+=face.Area
            supported_area+=face.common(shape).Area
    assert full_foot_area>500 and supported_area/full_foot_area>0.80, (roll,full_foot_area,supported_area)
    # Slide in along X with the connector leading through the rear plate.
    for dx in [0.25,1,3,6,10,20,40,80]:
        moved=camera_shape.copy();moved.translate(V(dx,0,0))
        assert shape.common(moved).Volume<1e-5, ('Camera rearward assembly blocked',roll,dx)
    camera_fits[roll]=dict(roll_about_forward_deg=roll,connector_center_global_mm=list(connector),
                           back_hole_centers_global_mm=rear_global,interference_volume_mm3=overlap,
                           foot_area_mm2=full_foot_area,supported_area_mm2=supported_area,
                           support_fraction=supported_area/full_foot_area)
reference_fit=camera_fits[camera_roll]
opening_top_z=plug_high_z+R['cable_diameter']/math.sqrt(2)
# Conservative full-forward-hemisphere check, retaining the previous pupil assumption.
fov_plane=C['forward_offset']+C['lens_barrel_rear_x_source']
margin=fov_plane-shape.BoundBox.XMax
assert margin>=C['minimum_rear_margin'],('Forward clearance',margin)
# Verify actual source mounting face and hole axes after the compound rotation.
lidar_source=Part.read(str(ROOT/'cad'/L['reference_file']))
lidar_source_holes=[]
lidar_support_area=0.;lidar_face_area=0.
for face in lidar_source.Faces:
    bb=face.optimalBoundingBox()
    if isinstance(face.Surface,Part.Plane) and abs(bb.YMin-L['bottom_y_source'])<1e-5 and abs(bb.YMax-L['bottom_y_source'])<1e-5:
        placed=face.copy();placed.Placement=refs['lidar'].Placement
        lidar_face_area+=placed.Area
        lidar_support_area+=placed.common(shape).Area
        for wire in face.Wires:
            wb=wire.optimalBoundingBox()
            if abs(abs(wb.Center.x)-L['hole_pitch_x']/2)<1e-4 and abs(abs(wb.Center.z)-L['hole_pitch_y']/2)<1e-4:
                lidar_source_holes.append(refs['lidar'].Placement.multVec(wb.Center))
assert len(lidar_source_holes)==4, ('LiDAR source mounting holes',len(lidar_source_holes))
assert all(min((pt-q).Length for q in lidar_source_holes)<1e-4 for pt in lidar_hole_centers)
assert lidar_face_area>3000 and lidar_support_area/lidar_face_area>0.95, ('LiDAR mounting face support',lidar_support_area,lidar_face_area)
lidar_insertion=L['screw_length_under_head']-L['screw_seat_thickness']
assert 0 < lidar_insertion <= 3.5, 'Confirm manufacturer thread depth before increasing insertion'
for h in holes:
    if h['type']=='LIDAR':
        pt=V(*h['xyz']);seat=V(*h['seat_xyz'])
        probe=Part.makeCylinder(2.75,90,seat,lidar_normal)
        # On the split version, fasten the LiDAR BEFORE bolting the adapter to
        # the base: the base intentionally covers the angled access channels.
        access_shape=adapter.Shape if SPLIT else shape
        assert access_shape.common(probe).Volume<1e-5,('Blocked angled LiDAR screw access',h)
        shaft=Part.makeCylinder(1.5,L['screw_length_under_head'],seat,-lidar_normal)
        assert shape.common(shaft).Volume<1e-5,('Blocked M3 shaft',h)
        bearing=Part.makeCylinder(2.75,0.2,seat-lidar_normal*0.2,lidar_normal).cut(
            Part.makeCylinder(L['clearance_diameter']/2,0.2,seat-lidar_normal*0.2,lidar_normal))
        assert abs(shape.common(bearing).Volume-bearing.Volume)<1e-5,('Incomplete angled screw bearing',h)
for offset in [0.25,1,3,6,10,20,40,80]:
    moved=refs['lidar'].Shape.copy();moved.translate(-lidar_normal*offset)
    assert shape.common(moved).Volume<1e-5,('LiDAR installation approach blocked',offset)

# Bought fasteners are reference envelopes, not printable parts or thread models.
hardware=[];hardware_groups={}
joint_report=None
if SPLIT:
    def hex_prism(across_flats,height,origin):
        radius=across_flats/math.sqrt(3)
        points=[origin+V(radius*math.cos(i*math.pi/3),radius*math.sin(i*math.pi/3),0) for i in range(6)]
        return Part.Face(Part.makePolygon(points+[points[0]])).extrude(V(0,0,height))

    def hardware_ref(name,group,s):
        o=D.addObject('Part::Feature',name);o.Shape=s;o.Label='BOUGHT - '+name;o.Visibility=True
        hardware.append(o);hardware_groups.setdefault(group,[]).append(o)
        return o

    # A solid/solid boolean common discards zero-thickness contact. Intersect
    # the mating planar faces themselves to measure the real bearing area.
    mating_faces=[]
    for obj in [fixture,adapter]:
        faces=[]
        for face in obj.Shape.Faces:
            fb=face.optimalBoundingBox()
            if isinstance(face.Surface,Part.Plane) and abs(fb.ZMin)<1e-6 and abs(fb.ZMax)<1e-6:
                faces.append(face)
        mating_faces.append(faces)
    joint_contact=sum(a.common(b).Area for a in mating_faces[0] for b in mating_faces[1])
    assert joint_contact>3000, ('Insufficient base/adapter contact',joint_contact)
    assert J['nut_pocket_depth']>J['nut_height']
    assert J['nut_pocket_across_flats']>J['nut_across_flats']
    nut_top=-J['flange_thickness']+J['nut_pocket_depth']
    nut_bottom=nut_top-J['nut_height']
    bolt_seat=B['thickness']+J['washer_thickness']
    bolt_tip=bolt_seat-J['screw_length_under_head']
    protrusion=nut_bottom-bolt_tip
    assert 0.8<=protrusion<=3, ('Joint bolt must fully engage nut',protrusion)
    for i,(x,y) in enumerate(J['holes_xy']):
        washer=Part.makeCylinder(J['washer_od']/2,J['washer_thickness'],V(x,y,B['thickness'])).cut(
            Part.makeCylinder(J['washer_id']/2,J['washer_thickness'],V(x,y,B['thickness'])))
        hardware_ref(f'M4Washer{i}','washers',washer)
        bolt=Part.makeCylinder(J['screw_diameter']/2,J['screw_length_under_head'],V(x,y,bolt_tip)).fuse(
            Part.makeCylinder(J['head_diameter']/2,J['head_height'],V(x,y,bolt_seat)))
        hardware_ref(f'M4Bolt{i}','bolts',bolt)
        nut=hex_prism(J['nut_across_flats'],J['nut_height'],V(x,y,nut_bottom)).cut(
            Part.makeCylinder(J['screw_diameter']/2,J['nut_height'],V(x,y,nut_bottom)))
        hardware_ref(f'M4Nut{i}','nuts',nut)
        # Full washer bearing above the base, and full nut bearing below the
        # pocket roof (a genuine load path rather than only matching holes).
        base_bearing=Part.makeCylinder(J['washer_od']/2,0.2,V(x,y,B['thickness']-.2)).cut(
            Part.makeCylinder(J['clearance_diameter']/2,0.2,V(x,y,B['thickness']-.2)))
        assert abs(fixture.Shape.common(base_bearing).Volume-base_bearing.Volume)<1e-5, ('Washer bearing',i)
        nut_bearing=hex_prism(J['nut_across_flats'],.2,V(x,y,nut_top)).cut(
            Part.makeCylinder(J['clearance_diameter']/2,.2,V(x,y,nut_top)))
        assert abs(adapter.Shape.common(nut_bearing).Volume-nut_bearing.Volume)<1e-5, ('Nut bearing',i)
        driver=Part.makeCylinder(J['head_diameter']/2,60,V(x,y,bolt_seat+J['head_height']))
        assert shape.common(driver).Volume<1e-5, ('Joint driver blocked',i)
        for sensor in refs.values():assert sensor.Shape.common(driver).Volume<1e-5, ('Sensor blocks joint driver',i)
        # Nut insertion from below is possible with the sensor already fitted.
        nut_sweep=hex_prism(J['nut_across_flats'],J['nut_height']+20,V(x,y,nut_bottom-20))
        assert adapter.Shape.common(nut_sweep).Volume<1e-5, ('Nut insertion blocked',i)
        for sensor in refs.values():assert sensor.Shape.common(nut_sweep).Volume<1e-5, ('Sensor blocks nut insertion',i)
        bolt_sweep=Part.makeCylinder(J['head_diameter']/2,40,V(x,y,bolt_seat))
        assert shape.common(bolt_sweep).Volume<1e-5, ('Bolt head insertion blocked',i)
    # Part.makeCompound crashes in this FreeCAD 1.0/Python 3.13 build; the
    # supported Part.Compound constructor preserves the same separate solids.
    m4_hardware=Part.Compound([o.Shape for o in hardware])
    assert shape.common(m4_hardware).Volume<1e-5, 'Joint hardware intersects printed parts'
    for o in [*refs.values(),*mount_refs]:
        assert o.Shape.common(m4_hardware).Volume<1e-5, ('Joint hardware collision',o.Name)
    for i,pt in enumerate(lidar_hole_centers):
        seat=pt+lidar_normal*L['screw_seat_thickness']
        bolt=Part.makeCylinder(1.5,L['screw_length_under_head'],seat,-lidar_normal).fuse(
            Part.makeCylinder(2.75,3,seat,lidar_normal))
        assert shape.common(bolt).Volume<1e-5, ('LiDAR screw collides with base or adapter',i)
        hardware_ref(f'M3LiDARScrew{i}','lidar_screws',bolt)
    # The adapter with the attached LiDAR moves vertically up to the base.
    subassembly=Part.Compound([adapter.Shape,refs['lidar'].Shape,
                                 *[o.Shape for o in hardware_groups['lidar_screws']]])
    for offset in [0.25,1,3,6,10,20,40,80]:
        moved=subassembly.copy();moved.translate(V(0,0,-offset))
        for fixed in [fixture,*mount_refs,refs['camera']]:
            assert fixed.Shape.common(moved).Volume<1e-5, ('Adapter assembly approach blocked',offset,fixed.Name)
    joint_report=dict(J,contact_area_mm2=joint_contact,nut_top_z_mm=nut_top,nut_bottom_z_mm=nut_bottom,
                      screw_tip_z_mm=bolt_tip,screw_protrusion_below_nut_mm=protrusion,
                      bearing_check='PASS - four complete washer and nut seats',
                      hardware_collision='PASS - M4 hardware clear of prints, sensors, collar and pipe',
                      access_check='PASS - top driver and bottom nut insertion with LiDAR fitted',
                      assembly_approach='PASS - eight vertical offsets of adapter plus LiDAR',
                      printed_part_overlap_mm3=fixture.Shape.common(adapter.Shape).Volume)

OUT.mkdir(parents=True,exist_ok=True)
part_reports={}
part_volumes={obj.Name:obj.Shape.Volume for _,obj in print_parts}
for name,obj in print_parts:
    mesh=MeshPart.meshFromShape(Shape=obj.Shape,LinearDeflection=.06,AngularDeflection=.18,Relative=False)
    if SPLIT and name=='lidar_adapter':
        # Put the broad base-contact face on the bed, without changing the CAD assembly.
        mesh.transform(App.Placement(V(0,0,0),App.Rotation(V(1,0,0),180)).toMatrix())
    assert mesh.isSolid() and not mesh.hasNonManifolds() and mesh.countComponents()==1
    assert not mesh.hasNonUniformOrientedFacets()
    if SPLIT:assert abs(mesh.BoundBox.ZMin)<1e-5, (name,'STL must be bed aligned')
    mesh.write(str(OUT/f'{name}.stl'))
    stl_check=Mesh.Mesh(str(OUT/f'{name}.stl'))
    assert stl_check.isSolid() and stl_check.countComponents()==1 and not stl_check.hasNonManifolds()
    assert not stl_check.hasNonUniformOrientedFacets()
    Part.export([obj],str(OUT/f'{name}.step'))
    # Ignore cached tessellation/deflection when reporting CAD dimensions.
    bounds=obj.Shape.optimalBoundingBox(False,False)
    part_reports[name]=dict(volume_mm3=obj.Shape.Volume,stl_facets=mesh.CountFacets,
                           stl_connected_components=1,stl_closed_manifold=True,
                           stl_z_min_mm=mesh.BoundBox.ZMin,stl_z_max_mm=mesh.BoundBox.ZMax,
                           stl_rotation_x_deg=180 if SPLIT and name=='lidar_adapter' else 0,
                           dimensions_mm=[bounds.XLength,bounds.YLength,bounds.ZLength])
Part.export([*[o for _,o in print_parts],*refs.values(),*mount_refs,*hardware],str(OUT/'assembly.step'))
native_path=OUT/('assembly.FCStd' if SPLIT else 'fixture.FCStd')
D.recompute();D.saveAs(str(native_path))
report={'engine':'FreeCAD '+'.'.join(App.Version()[:3]),'status':P['status'],'printed_part_count':len(print_parts),
        'variant':args.variant,'printed_parts':part_reports,'assembly_joint':joint_report,
        'fixture_valid_solid':True,'stl_closed_manifold':True,'stl_connected_components':len(print_parts),
        'stl_facets':sum(item['stl_facets'] for item in part_reports.values()),
        'fixture_bounds_mm':dict(x=[shape.BoundBox.XMin,shape.BoundBox.XMax],y=[shape.BoundBox.YMin,shape.BoundBox.YMax],z=[shape.BoundBox.ZMin,shape.BoundBox.ZMax]),
        'camera_forward_shift_mm':C['forward_offset'],'interference_volume_mm3':interferences,
        'camera_orientation':dict(roll_about_forward_deg=camera_roll,source_positive_z_global=list(cam_source_z),
                                  seating_plane_z_source=camera_seat_z*cam_source_z.z,basis=C['orientation_basis']),
        'camera_mount_orientations':list(camera_fits.values()),
        'camera_connector_center_global_mm':reference_fit['connector_center_global_mm'],
        'cable_opening_height_above_base_mm':cable_z,
        'cable_base_relief_depth_mm':max(0,plug_radius-cable_z),
        'cable_upward_extension_mm':R['cable_upward_extension'],
        'cable_opening_bottom_z_mm':plug_low_z-plug_radius,
        'cable_opening_top_z_mm':opening_top_z,
        'cable_opening_height_mm':opening_top_z-plug_low_z+plug_radius,
        'cable_approach':f"PASS - nominal dia {R['cable_diameter']:g} plug path clear at both camera rolls and between",
        'camera_fastener':dict(diameter_mm=C['screw_diameter'],spec_source=C['screw_spec_source'],hole_diameter_mm=R['camera_clearance_diameter'],
                               length_under_head_mm=C['screw_length_under_head'],through_length_mm=R['thickness'],
                               insertion_without_washer_mm=screw_insertion,assumed_head_diameter_mm=C['assumed_screw_head_diameter'],
                               head_bearing='PASS - full assumed dia 3.0 head annulus at all four holes',
                               actual_insertion_limit_verified=False),
        'camera_assembly_approach':'PASS - exact sensor at 8 approach offsets along X at each camera roll',
        'camera_back_hole_centers_global':reference_fit['back_hole_centers_global_mm'],
        'camera_foot_area_mm2':reference_fit['foot_area_mm2'],'camera_supported_area_mm2':reference_fit['supported_area_mm2'],
        'camera_support_fraction':reference_fit['support_fraction'],
        'lidar_mount_rotation_z_deg':L['mount_rotation_z_deg'],
        'lidar_mount_rotation_y_deg':L['mount_rotation_y_deg'],
        'lidar_mount_center_mm':list(lidar_mount_origin),
        'lidar_mount_hole_centers_xy':[[pt.x,pt.y] for pt in lidar_hole_centers],
        'lidar_mount_hole_centers_xyz':[list(pt) for pt in lidar_hole_centers],
        'lidar_supported_area_mm2':lidar_support_area,'lidar_mount_face_area_mm2':lidar_face_area,
        'lidar_source_hole_alignment':'PASS - all four actual STEP hole centers match rotated bores',
        'lidar_screw_insertion_mm':lidar_insertion,
        'lidar_screw_bearing':'PASS - four perpendicular seats and unobstructed M3 shafts',
        'lidar_assembly_approach':'PASS - eight offsets along tilted mounting normal',
        'lidar_screw_access':('PASS - fasten LiDAR to detached adapter before joining base' if SPLIT
                              else 'PASS - all four screws accessible before camera insertion'),
        'directions':dict(camera_forward=list(camdir),lidar_scan=list(liddir),camera_cable=list(camport),lidar_cable=list(lidport)),
        'fov':dict(assumption='Entrance pupil at or forward of lens-barrel rear plane; all forward rays have nonnegative X direction',
                   pupil_lower_bound_x=fov_plane,fixture_max_x=shape.BoundBox.XMax,clearance_behind_plane_mm=margin,
                   claimed_horizontal_fov_deg=C['horizontal_fov_assumed_deg'],entrance_pupil_verified=False,
                   result='PASS for fixture only under assumed forward hemisphere; tilted LiDAR may enter camera FOV',
                   lidar_max_x_mm=refs['lidar'].Shape.optimalBoundingBox().XMax,
                   full_assembly_fov_verified=False),
        'fixture_volume_mm3':shape.Volume,'pla_solid_mass_g_at_1_24':shape.Volume*0.00124,
        'holes':holes,'printing':P['assembly_printing'] if SPLIT else P['printing'],'unresolved':P['unresolved'],
        'source_sha256':{k:hashlib.sha256((ROOT/'cad'/P[k]['reference_file']).read_bytes()).hexdigest() for k in refs}}
report['pipe_mount']=dict(M, mounting_dimensions_verified=False, pipe_od_verified=False, reference_collision_check='PASS - provisional envelopes only', slot_and_tool_access='PASS - M5 shaft at pitch 34 and 40 mm', minimum_slot_pipe_ligament_mm=M['slot_pitch_min']/2-M['slot_width']/2-(M['pipe_od']+0.6)/2)
report['integrated_base']=('PASS - two separate printed solids bolted across the Z=0 interface' if SPLIT else
                          f"PASS - single solid, continuous base plus integral {L['mount_rotation_y_deg']:g} deg Y LiDAR wedge")
report['parameters_sha256']=hashlib.sha256((ROOT/'cad/parameters.json').read_bytes()).hexdigest()
report['model_script_sha256']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
report['STL_roundtrip']='PASS - each reimported STL is a closed manifold with one connected component'
# Native-file reopen/recompute, including a genuine parameter edit and return to nominal.
part_names=[(name,obj.Name) for name,obj in print_parts]
hardware_names={key:[o.Name for o in objects] for key,objects in hardware_groups.items()}
fixture_name=fixture.Name
App.closeDocument(D.Name)
D=App.openDocument(str(native_path));D.recompute()
for name,obj_name in part_names:
    obj=D.getObject(obj_name)
    assert obj.Shape.isValid() and len(obj.Shape.Solids)==1
    assert abs(obj.Shape.Volume-part_volumes[obj_name])<1e-5
old_back_volume=D.BackPlate.Shape.Volume
cell=D.Parameters.getCellFromAlias('BackT');D.Parameters.set(cell,f"{values['BackT']+1} mm");D.recompute()
assert D.getObject(fixture_name).Shape.isValid() and D.BackPlate.Shape.Volume>old_back_volume
assert abs(D.BackPlate.Length.Value-values['BackT']-1)<1e-6
D.Parameters.set(cell,f"{values['BackT']} mm");D.recompute()
for name,obj_name in part_names:assert abs(D.getObject(obj_name).Shape.Volume-part_volumes[obj_name])<1e-5
report['FCStd_reopen_parameter_edit']='PASS - reopen, change back plate thickness +1 mm, recompute, restore'
for name,obj_name in part_names:
    step_check=Part.read(str(OUT/f'{name}.step'))
    assert step_check.isValid() and len(step_check.Solids)==1
    assert math.isclose(step_check.Volume,part_volumes[obj_name],rel_tol=1e-7)
report['STEP_roundtrip']='PASS - each part is a valid single solid with preserved volume'
(OUT/'validation.json').write_text(json.dumps(report,indent=2)+'\n')
# Meshes for report rendering only. CAD geometry is built exclusively in FreeCAD.
render={}
render_shapes=[(name,D.getObject(obj_name).Shape) for name,obj_name in part_names]
render_shapes += [(key,obj.Shape) for key,obj in [('camera',D.Camera),('lidar',D.MID360),('collar',D.CollarEnvelope),('pipe',D.CarbonPipeEnvelope)]]
render_shapes += [(key,Part.Compound([D.getObject(name).Shape for name in names])) for key,names in hardware_names.items()]
for key,render_shape in render_shapes:
    vs,ts=render_shape.tessellate(.2)
    render[key]={'vertices':[list(v) for v in vs],'triangles':ts}
(ROOT/'.cache').mkdir(exist_ok=True)
CACHE.write_text(json.dumps(render))
App.closeDocument(D.Name)
print(json.dumps(report,indent=2))
