"""Build a native, editable FreeCAD document and the single printable fixture."""
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
OUT = ROOT/'output'
P = json.loads((ROOT/'cad/parameters.json').read_text())
V = App.Vector
D = App.newDocument('SinglePrintedFixture')
S = D.addObject('Spreadsheet::Sheet', 'Parameters')
S.Label = 'Editable dimensions (mm)'
B,C,L,R = P['plate'],P['camera'],P['lidar'],P['rear_bracket']
M=P['pipe_mount']
lidar_yaw=App.Rotation(V(0,0,1),L['mount_rotation_z_deg'])
lidar_pitch=App.Rotation(V(0,1,0),L['mount_rotation_y_deg'])
lidar_mount_rotation=lidar_pitch.multiply(lidar_yaw)
lidar_mount_origin=V(0,0,L['mount_center_z'])
lidar_normal=lidar_pitch.multVec(V(0,0,1))
lidar_hole_rotation=App.Rotation(V(0,0,1),lidar_normal)
assert -60 < L['mount_rotation_y_deg'] < 0, 'Wedge supports negative assembly Y pitch between -60 and 0 degrees'
assert L['screw_seat_thickness'] > 0
assert L['mount_center_z']+abs(math.tan(math.radians(L['mount_rotation_y_deg'])))*L['wedge_length_x']/2 < -1, 'Keep tilted mount below base'
camera_rotation=App.Rotation(V(1,0,0),C['roll_about_forward_deg']).multiply(App.Rotation(V(0,0,1),-90))
camera_seat_z=camera_rotation.multVec(V(0,0,C['seating_plane_z_source'])).z
cable_z=camera_rotation.multVec(V(0,0,C['rear_connector_z_source'])).z-camera_seat_z
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
wedge_stock=box('LiDARWedgeStock',L['wedge_length_x'],L['wedge_width_y'],1-wedge_bottom,
                -L['wedge_length_x']/2,-L['wedge_width_y']/2,wedge_bottom)
clip_origin=lidar_mount_origin+lidar_pitch.multVec(V(-150,-150,-150))
wedge_clip=box('LiDARPlaneTrim',300,300,150,clip_origin.x,clip_origin.y,clip_origin.z)
wedge_clip.Placement=App.Placement(clip_origin,lidar_pitch)
wedge=primitive('Part::Cut','LiDARTiltWedge');wedge.Base=wedge_stock;wedge.Tool=wedge_clip;wedge.Refine=True
stock=fuse('OnePieceStock',[base,back,*braces,wedge])
tools=[];holes=[]
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
    tools.append(cyl(f'LiDARHole{i}',L['clearance_diameter']/2,L['screw_seat_thickness']+2,
                     bore_start.x,bore_start.y,bore_start.z,lidar_hole_rotation))
    # Deep head/driver channel; its flat bottom is perpendicular to the screw.
    tools.append(cyl(f'LiDARCounterbore{i}',L['counterbore_diameter']/2,100,
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
# Cable opening: a vertically extended circle with a 45-degree roof for FDM.
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
cutters=fuse('ClearanceOpenings',tools)
fixture=primitive('Part::Cut','PrintableFixture');fixture.Base=stock;fixture.Tool=cutters;fixture.Refine=True
fixture.Label='ONE PRINTED PART - PLA'
D.recompute()
assert not fixture.Shape.isNull(), 'FreeCAD feature recompute failed'
assert fixture.Shape.isValid() and len(fixture.Shape.Solids)==1, 'Fixture must be one valid solid'

# User STEP geometry: camera rolled 180 degrees about +X, rear connector low.
# MID-360 yaw is followed by assembly Y pitch, about its mounting center.
refs={}
for name,key in [('Camera','camera'),('MID360','lidar')]:
    o=D.addObject('Part::Feature',name);o.Label='REFERENCE '+name
    o.Shape=Part.read(str(ROOT/'cad'/P[key]['reference_file']))
    if key=='camera':
        o.Placement=App.Placement(V(C['forward_offset'],0,B['thickness']-camera_seat_z),camera_rotation)
        o.setExpression('Placement.Base.x','Parameters.CameraX')
        o.setExpression('Placement.Base.z','Parameters.BaseT-Parameters.CameraSeatZ')
    else:
        r=lidar_mount_rotation.multiply(App.Rotation(V(0,0,1),180).multiply(App.Rotation(V(1,0,0),-90)))
        o.Placement=App.Placement(lidar_mount_origin+lidar_pitch.multVec(V(0,0,L['bottom_y_source'])),r)
    refs[key]=o
D.recompute()
for o in features:o.Visibility=False
fixture.Visibility=True
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
shape=fixture.Shape
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
assert refs['camera'].Shape.optimalBoundingBox().ZMin>refs['lidar'].Shape.optimalBoundingBox().ZMax
assert abs(refs['camera'].Shape.optimalBoundingBox().YMin+refs['camera'].Shape.optimalBoundingBox().YMax)<1e-4
camdir=refs['camera'].Placement.Rotation.multVec(V(0,1,0))
liddir=refs['lidar'].Placement.Rotation.multVec(V(0,1,0))
camport=refs['camera'].Placement.Rotation.multVec(V(0,-1,0))
lidport=refs['lidar'].Placement.Rotation.multVec(V(1,0,0))
cam_source_z=refs['camera'].Placement.Rotation.multVec(V(0,0,1))
assert (camdir-V(1,0,0)).Length<1e-8 and (liddir-lidar_pitch.multVec(V(0,0,-1))).Length<1e-8
assert (camport-V(-1,0,0)).Length<1e-8 and (lidport-lidar_mount_rotation.multVec(V(-1,0,0))).Length<1e-8
assert (cam_source_z-V(0,0,-1)).Length<1e-8, 'Camera roll must put source +Z downward'
connector=refs['camera'].Placement.multVec(V(0,C['rear_face_y_source'],C['rear_connector_z_source']))
assert abs(connector.y)<1e-8 and abs(connector.z-(B['thickness']+cable_z))<1e-8
assert connector.z<refs['camera'].Placement.Base.z, 'Rear connector must be below optical axis'
rear_start=B['center_x']-B['length_x']/2-1
plug_probe=Part.makeCylinder(R['cable_diameter']/2-0.02,connector.x-rear_start+0.1,
                             V(rear_start,connector.y,connector.z),V(1,0,0))
# Probe the whole upward extension, including the formerly obstructing upper lip.
upper_probe=plug_probe.copy();upper_probe.translate(V(0,0,R['cable_upward_extension']))
bridge_probe=Part.makeBox(connector.x-rear_start+0.1,R['cable_diameter']-0.04,R['cable_upward_extension'],
                          V(rear_start,-R['cable_diameter']/2+0.02,connector.z))
plug_envelope=plug_probe.fuse(upper_probe).fuse(bridge_probe)
assert shape.common(plug_envelope).Volume<1e-5, 'Extended cable approach blocked by base or rear plate'
assert C['screw_diameter']<R['camera_clearance_diameter']<C['assumed_screw_head_diameter']
screw_insertion=C['screw_length_under_head']-R['thickness']
assert 0<screw_insertion<C['rear_hole_axis_first_obstruction_mm'], 'Nominal screw exceeds STEP axial opening'
# Inspect the supplied camera back-face loops, rather than guessing a thread pattern.
source=Part.read(str(ROOT/'cad'/C['reference_file']))
rear_holes=[]
for face in source.Faces:
    bb=face.optimalBoundingBox()
    if abs(bb.YMin-C['rear_face_y_source'])<1e-5 and abs(bb.YMax-C['rear_face_y_source'])<1e-5:
        for wire in face.Wires:
            wb=wire.optimalBoundingBox();cx=(wb.XMin+wb.XMax)/2;cz=(wb.ZMin+wb.ZMax)/2
            if abs(abs(cx)-C['rear_mount_pitch']/2)<1e-4 and abs(abs(cz)-C['rear_mount_pitch']/2)<1e-4:
                assert abs(wb.XLength-C['rear_hole_entry_diameter_model'])<1e-4
                rear_holes.append((cx,cz))
assert len(rear_holes)==4,rear_holes
rear_global=[]
for x,z in rear_holes:
    pt=refs['camera'].Placement.multVec(V(x,C['rear_face_y_source'],z))
    rear_global.append(list(pt))
    assert abs(pt.x-(C['forward_offset']+C['rear_face_y_source']))<1e-8
    # Clear through-hole to each camera back-face hole.
    bore=Part.makeCylinder(R['camera_clearance_diameter']/2-0.02,R['thickness']+0.1,
                          V(pt.x-R['thickness']-0.1,pt.y,pt.z),V(1,0,0))
    assert shape.common(bore).Volume<1e-5
    # Rear screwdriver/head access is kept clear of the two rear gussets.
    approach=Part.makeCylinder(2.75,40,V(pt.x-R['thickness']-40,pt.y,pt.z),V(1,0,0))
    assert shape.common(approach).Volume<1e-5
    # A complete bearing annulus for an assumed dia 3.0 M1.6 head remains around
    # each small through-hole, even beside the enlarged cable opening.
    head_base=V(pt.x-R['thickness'],pt.y,pt.z)
    bearing=Part.makeCylinder(C['assumed_screw_head_diameter']/2,0.2,head_base,V(1,0,0)).cut(
        Part.makeCylinder(R['camera_clearance_diameter']/2,0.2,head_base,V(1,0,0)))
    assert abs(shape.common(bearing).Volume-bearing.Volume)<1e-5, 'Incomplete screw head bearing surface'
    shaft=Part.makeCylinder(C['screw_diameter']/2,C['screw_length_under_head'],head_base,V(1,0,0))
    assert shape.common(shaft).Volume<1e-5, 'Nominal 8 mm screw shaft blocked by fixture'
# Exact camera underside planes are supported on the base top, minus bolt openings.
full_foot_area=0.;supported_area=0.
for face in refs['camera'].Shape.Faces:
    bb=face.optimalBoundingBox()
    if isinstance(face.Surface,Part.Plane) and abs(bb.ZMin-B['thickness'])<1e-5 and abs(bb.ZMax-B['thickness'])<1e-5:
        full_foot_area+=face.Area
        supported_area+=face.common(shape).Area
assert full_foot_area>500 and supported_area/full_foot_area>0.80,(full_foot_area,supported_area)
# The rear connector crosses the back plate only through the cable opening at the
# final height: install by translating rearward along X, with the connector leading.
# Check real sensor geometry at representative approach positions; a cuboid would
# incorrectly fill the connector clearance and the empty space behind the body.
for dx in [0.25,1,3,6,10,20,40,80]:
    moved=refs['camera'].Shape.copy();moved.translate(V(dx,0,0))
    assert shape.common(moved).Volume<1e-5,('Camera rearward assembly blocked',dx)
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
        assert shape.common(probe).Volume<1e-5,('Blocked angled LiDAR screw access',h)
        shaft=Part.makeCylinder(1.5,L['screw_length_under_head'],seat,-lidar_normal)
        assert shape.common(shaft).Volume<1e-5,('Blocked M3 shaft',h)
        bearing=Part.makeCylinder(2.75,0.2,seat-lidar_normal*0.2,lidar_normal).cut(
            Part.makeCylinder(L['clearance_diameter']/2,0.2,seat-lidar_normal*0.2,lidar_normal))
        assert abs(shape.common(bearing).Volume-bearing.Volume)<1e-5,('Incomplete angled screw bearing',h)
for offset in [0.25,1,3,6,10,20,40,80]:
    moved=refs['lidar'].Shape.copy();moved.translate(-lidar_normal*offset)
    assert shape.common(moved).Volume<1e-5,('LiDAR installation approach blocked',offset)
mesh=MeshPart.meshFromShape(Shape=shape,LinearDeflection=.06,AngularDeflection=.18,Relative=False)
assert mesh.isSolid() and not mesh.hasNonManifolds() and mesh.countComponents()==1
assert not mesh.hasNonUniformOrientedFacets()
OUT.mkdir(exist_ok=True)
mesh.write(str(OUT/'fixture.stl'))
stl_check=Mesh.Mesh(str(OUT/'fixture.stl'))
assert stl_check.isSolid() and stl_check.countComponents()==1 and not stl_check.hasNonManifolds()
Part.export([fixture],str(OUT/'fixture.step'))
Part.export([fixture,*refs.values(),*mount_refs],str(OUT/'assembly.step'))
D.recompute();D.saveAs(str(OUT/'fixture.FCStd'))
report={'engine':'FreeCAD '+'.'.join(App.Version()[:3]),'status':P['status'],'printed_part_count':1,
        'fixture_valid_solid':True,'stl_closed_manifold':True,'stl_connected_components':1,'stl_facets':mesh.CountFacets,
        'fixture_bounds_mm':dict(x=[shape.BoundBox.XMin,shape.BoundBox.XMax],y=[shape.BoundBox.YMin,shape.BoundBox.YMax],z=[shape.BoundBox.ZMin,shape.BoundBox.ZMax]),
        'camera_forward_shift_mm':C['forward_offset'],'interference_volume_mm3':interferences,
        'camera_orientation':dict(roll_about_forward_deg=C['roll_about_forward_deg'],source_positive_z_global=list(cam_source_z),
                                  seating_plane_z_source=C['seating_plane_z_source'],basis=C['orientation_basis']),
        'camera_connector_center_global_mm':list(connector),
        'cable_opening_height_above_base_mm':cable_z,
        'cable_base_relief_depth_mm':max(0,R['cable_diameter']/2-cable_z),
        'cable_upward_extension_mm':R['cable_upward_extension'],
        'cable_opening_top_z_mm':connector.z+R['cable_upward_extension']+R['cable_diameter']/math.sqrt(2),
        'cable_approach':'PASS - nominal dia 16 profile swept upward 6 mm, clear from rear edge through plate; actual plug not supplied',
        'camera_fastener':dict(diameter_mm=C['screw_diameter'],spec_source=C['screw_spec_source'],hole_diameter_mm=R['camera_clearance_diameter'],
                               length_under_head_mm=C['screw_length_under_head'],through_length_mm=R['thickness'],
                               insertion_without_washer_mm=screw_insertion,assumed_head_diameter_mm=C['assumed_screw_head_diameter'],
                               head_bearing='PASS - full assumed dia 3.0 head annulus at all four holes',
                               actual_insertion_limit_verified=False),
        'camera_assembly_approach':'PASS - exact sensor at 8 approach offsets along X; final rear contact and cable clearance',
        'camera_back_hole_centers_global':rear_global,'camera_foot_area_mm2':full_foot_area,
        'camera_supported_area_mm2':supported_area,'camera_support_fraction':supported_area/full_foot_area,
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
        'lidar_screw_access':'PASS - all four screws accessible before camera insertion',
        'directions':dict(camera_forward=list(camdir),lidar_scan=list(liddir),camera_cable=list(camport),lidar_cable=list(lidport)),
        'fov':dict(assumption='Entrance pupil at or forward of lens-barrel rear plane; all forward rays have nonnegative X direction',
                   pupil_lower_bound_x=fov_plane,fixture_max_x=shape.BoundBox.XMax,clearance_behind_plane_mm=margin,
                   claimed_horizontal_fov_deg=C['horizontal_fov_assumed_deg'],entrance_pupil_verified=False,
                   result='PASS for fixture only under assumed forward hemisphere; tilted LiDAR may enter camera FOV',
                   lidar_max_x_mm=refs['lidar'].Shape.optimalBoundingBox().XMax,
                   full_assembly_fov_verified=False),
        'fixture_volume_mm3':shape.Volume,'pla_solid_mass_g_at_1_24':shape.Volume*0.00124,
        'holes':holes,'printing':P['printing'],'unresolved':P['unresolved'],
        'source_sha256':{k:hashlib.sha256((ROOT/'cad'/P[k]['reference_file']).read_bytes()).hexdigest() for k in refs}}
report['pipe_mount']=dict(M, mounting_dimensions_verified=False, pipe_od_verified=False, reference_collision_check='PASS - provisional envelopes only', slot_and_tool_access='PASS - M5 shaft at pitch 34 and 40 mm', minimum_slot_pipe_ligament_mm=M['slot_pitch_min']/2-M['slot_width']/2-(M['pipe_od']+0.6)/2)
report['integrated_base']=f"PASS - single solid, continuous base plus integral {L['mount_rotation_y_deg']:g} deg Y LiDAR wedge"
report['parameters_sha256']=hashlib.sha256((ROOT/'cad/parameters.json').read_bytes()).hexdigest()
report['model_script_sha256']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
report['STL_roundtrip']='PASS - reimported closed manifold, one connected component'
# Native-file reopen/recompute, including a genuine parameter edit and return to nominal.
App.closeDocument(D.Name)
D=App.openDocument(str(OUT/'fixture.FCStd'));D.recompute()
assert D.PrintableFixture.Shape.isValid()
assert abs(D.PrintableFixture.Shape.Volume-shape.Volume)<1e-5
old_back_volume=D.BackPlate.Shape.Volume
cell=D.Parameters.getCellFromAlias('BackT');D.Parameters.set(cell,f"{values['BackT']+1} mm");D.recompute()
assert D.PrintableFixture.Shape.isValid() and D.BackPlate.Shape.Volume>old_back_volume
assert abs(D.BackPlate.Length.Value-values['BackT']-1)<1e-6
D.Parameters.set(cell,f"{values['BackT']} mm");D.recompute()
assert abs(D.PrintableFixture.Shape.Volume-shape.Volume)<1e-5
report['FCStd_reopen_parameter_edit']='PASS - reopen, change back plate thickness +1 mm, recompute, restore'
step_check=Part.read(str(OUT/'fixture.step'))
assert step_check.isValid() and len(step_check.Solids)==1
assert math.isclose(step_check.Volume,shape.Volume,rel_tol=1e-7)
report['STEP_roundtrip']='PASS - valid single solid, volume preserved'
(OUT/'validation.json').write_text(json.dumps(report,indent=2)+'\n')
# Meshes for report rendering only. CAD geometry is built exclusively in FreeCAD.
render={}
for key,obj in [('fixture',D.PrintableFixture),('camera',D.Camera),('lidar',D.MID360),('collar',D.CollarEnvelope),('pipe',D.CarbonPipeEnvelope)]:
    vs,ts=obj.Shape.tessellate(.2)
    render[key]={'vertices':[list(v) for v in vs],'triangles':ts}
(ROOT/'.cache').mkdir(exist_ok=True)
(ROOT/'.cache/render-meshes.json').write_text(json.dumps(render))
App.closeDocument(D.Name)
print(json.dumps(report,indent=2))
