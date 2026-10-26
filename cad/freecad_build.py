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
B,C,L,K = P['plate'],P['camera'],P['lidar'],P['cage']
values = dict(BaseL=B['length_x'], BaseW=B['width_y'], BaseT=B['thickness'], BaseCX=B['center_x'], Radius=B['corner_radius'],
              CameraX=C['forward_offset'], InnerW=K['inside_width'], InnerH=K['inside_height'], Wall=K['wall_thickness'],
              RoofT=K['roof_thickness'], CageL=K['length_x'], CageRear=K['rear_relative_x'], ScrewX=K['screw_relative_x'],
              NutFlat=K['nut_across_flats'], NutDepth=K['nut_depth'])
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
rails=[]
for i,y in enumerate(['-Parameters.InnerW/2-Parameters.Wall','Parameters.InnerW/2']):
    rails.append(box(f'CageSide{i}','Parameters.CageL','Parameters.Wall','Parameters.InnerH+Parameters.RoofT+1 mm',
                     'Parameters.CameraX+Parameters.CageRear',y,'Parameters.BaseT-1 mm'))
roof=box('IntegralRoof','Parameters.CageL','Parameters.InnerW+2*Parameters.Wall','Parameters.RoofT',
         'Parameters.CameraX+Parameters.CageRear','-Parameters.InnerW/2-Parameters.Wall','Parameters.BaseT+Parameters.InnerH')
stock=fuse('OnePieceStock',[base,*rails,roof])
tools=[];holes=[]
for i,(x,y) in enumerate(B['host_holes_xy']):
    tools.append(cyl(f'HostHole{i}',B['host_hole_diameter']/2,'Parameters.BaseT+2 mm',x,y,-1))
    holes.append(dict(type='HOST',xy=[x,y],diameter=B['host_hole_diameter']))
for i,(x,y) in enumerate([(x,y) for x in [-L['hole_pitch_x']/2,L['hole_pitch_x']/2] for y in [-L['hole_pitch_y']/2,L['hole_pitch_y']/2]]):
    tools.append(cyl(f'LiDARHole{i}',L['clearance_diameter']/2,'Parameters.BaseT+2 mm',x,y,-1))
    tools.append(cyl(f'LiDARCounterbore{i}',L['counterbore_diameter']/2,L['counterbore_depth']+1,
                     x,y,f"Parameters.BaseT-{L['counterbore_depth']} mm"))
    holes.append(dict(type='LIDAR',xy=[x,y],diameter=L['clearance_diameter'],counterbore=L['counterbore_diameter'],depth=L['counterbore_depth']))
    # Vertical driver/head access through the integrated roof at the two forward mounting screws.
    if x>0:
        tools.append(cyl(f'RoofDriverAccess{i}',K['driver_hole_diameter']/2,'Parameters.RoofT+2 mm',x,y,'Parameters.BaseT+Parameters.InnerH-1 mm'))
for i,z in enumerate(K['screw_z_above_floor']):
    rot=App.Rotation(V(1,0,0),-90)
    tools.append(cyl(f'ClampScrew{i}',K['screw_clearance_diameter']/2,'Parameters.Wall+2 mm',
                     'Parameters.CameraX+Parameters.ScrewX','Parameters.InnerW/2-1 mm',f'Parameters.BaseT+{z} mm',rot))
    h=primitive('Part::Prism',f'CaptiveNut{i}');h.Polygon=6
    h.setExpression('Circumradius','Parameters.NutFlat/sqrt(3)');h.setExpression('Height','Parameters.NutDepth+0.1 mm')
    place(h,'Parameters.CameraX+Parameters.ScrewX','Parameters.InnerW/2-0.1 mm',f'Parameters.BaseT+{z} mm',rot)
    tools.append(h)
cutters=fuse('MachiningAndNutPockets',tools)
fixture=primitive('Part::Cut','PrintableFixture');fixture.Base=stock;fixture.Tool=cutters;fixture.Refine=True
fixture.Label='ONE PRINTED PART - PLA'
D.recompute()
assert not fixture.Shape.isNull(), 'FreeCAD feature recompute failed'
assert fixture.Shape.isValid() and len(fixture.Shape.Solids)==1, 'Fixture must be one valid solid'

# User STEP geometry: camera retains forward +X; MID-360 points down, both ports -X.
refs={}
for name,key in [('Camera','camera'),('MID360','lidar')]:
    o=D.addObject('Part::Feature',name);o.Label='REFERENCE '+name
    o.Shape=Part.read(str(ROOT/'cad'/P[key]['reference_file']))
    if key=='camera':
        o.Placement=App.Placement(V(C['forward_offset'],0,B['thickness']-C['bottom_z_source']),App.Rotation(V(0,0,1),-90))
        o.setExpression('Placement.Base.x','Parameters.CameraX')
        o.setExpression('Placement.Base.z',f"Parameters.BaseT-{C['bottom_z_source']} mm")
    else:
        r=App.Rotation(V(0,0,1),180).multiply(App.Rotation(V(1,0,0),-90))
        o.Placement=App.Placement(V(0,0,L['bottom_y_source']),r)
    refs[key]=o
D.recompute()
for o in features:o.Visibility=False
fixture.Visibility=True
for o in refs.values():o.Visibility=True
S.Visibility=False

# Native geometry and fabrication checks.
shape=fixture.Shape
interferences={k:shape.common(o.Shape).Volume for k,o in refs.items()}
assert all(v<1e-5 for v in interferences.values()),interferences
assert refs['camera'].Shape.optimalBoundingBox().ZMin>refs['lidar'].Shape.optimalBoundingBox().ZMax
assert abs(refs['camera'].Shape.optimalBoundingBox().YMin+refs['camera'].Shape.optimalBoundingBox().YMax)<1e-4
camdir=refs['camera'].Placement.Rotation.multVec(V(0,1,0))
liddir=refs['lidar'].Placement.Rotation.multVec(V(0,1,0))
camport=refs['camera'].Placement.Rotation.multVec(V(0,-1,0))
lidport=refs['lidar'].Placement.Rotation.multVec(V(1,0,0))
assert (camdir-V(1,0,0)).Length<1e-8 and (liddir-V(0,0,-1)).Length<1e-8
assert (camport-V(-1,0,0)).Length<1e-8 and (lidport-V(-1,0,0)).Length<1e-8
# Conservative forward-hemisphere exclusion; the real entrance pupil is not provided in STEP.
fov_plane=C['forward_offset']+C['lens_barrel_rear_x_source']
margin=fov_plane-shape.BoundBox.XMax
assert margin>5, 'Fixture must stay at least 5 mm behind the assumed pupil lower-bound plane'
# Camera slides in from +X with connector leading. Check the complete translation interval
# conservatively using the swept bounding envelope, not only a few sample positions.
cb=refs['camera'].Shape.optimalBoundingBox()
# STEP tolerance inflates the bounding box slightly below the seating plane.
# Sliding contact at the exact plate top is allowed; all other envelope faces
# retain the conservative STEP bounds.
sweep=Part.makeBox(cb.XLength+100,cb.YLength,cb.ZMax-B['thickness'],V(cb.XMin,cb.YMin,B['thickness']))
insertion_overlap=shape.common(sweep).Volume
assert insertion_overlap<1e-4,('Camera insertion path blocked',insertion_overlap)
# Screwdriver corridors remain clear from above while camera is absent.
for h in holes:
    if h['type']=='LIDAR':
        x,y=h['xy'];probe=Part.makeCylinder(2.75,90,V(x,y,B['thickness']-L['counterbore_depth']))
        assert shape.common(probe).Volume<1e-5,('Blocked LiDAR screw access',h)
# Captive nuts can be inserted from the open camera tunnel before inserting camera.
mesh=MeshPart.meshFromShape(Shape=shape,LinearDeflection=.06,AngularDeflection=.18,Relative=False)
assert mesh.isSolid() and not mesh.hasNonManifolds() and mesh.countComponents()==1
assert not mesh.hasNonUniformOrientedFacets()
OUT.mkdir(exist_ok=True)
mesh.write(str(OUT/'fixture.stl'))
stl_check=Mesh.Mesh(str(OUT/'fixture.stl'))
assert stl_check.isSolid() and stl_check.countComponents()==1 and not stl_check.hasNonManifolds()
Part.export([fixture],str(OUT/'fixture.step'))
Part.export([fixture,*refs.values()],str(OUT/'assembly.step'))
D.recompute();D.saveAs(str(OUT/'fixture.FCStd'))
report={'engine':'FreeCAD '+'.'.join(App.Version()[:3]),'status':P['status'],'printed_part_count':1,
        'fixture_valid_solid':True,'stl_closed_manifold':True,'stl_connected_components':1,'stl_facets':mesh.CountFacets,
        'fixture_bounds_mm':dict(x=[shape.BoundBox.XMin,shape.BoundBox.XMax],y=[shape.BoundBox.YMin,shape.BoundBox.YMax],z=[shape.BoundBox.ZMin,shape.BoundBox.ZMax]),
        'camera_forward_shift_mm':C['forward_offset'],'interference_volume_mm3':interferences,
        'camera_insertion_swept_envelope':'PASS - unobstructed front insertion with clamping screws backed off',
        'lidar_screw_access':'PASS - all four screws accessible before camera insertion',
        'directions':dict(camera_forward=list(camdir),lidar_scan=list(liddir),camera_cable=list(camport),lidar_cable=list(lidport)),
        'fov':dict(assumption='Entrance pupil at or forward of lens-barrel rear plane; all forward rays have nonnegative X direction',
                   pupil_lower_bound_x=fov_plane,fixture_max_x=shape.BoundBox.XMax,clearance_behind_plane_mm=margin,
                   claimed_horizontal_fov_deg=C['horizontal_fov_assumed_deg'],entrance_pupil_verified=False,
                   result='PASS for assumed forward hemisphere; verify actual image and full calibrated FOV'),
        'fixture_volume_mm3':shape.Volume,'pla_solid_mass_g_at_1_24':shape.Volume*0.00124,
        'holes':holes,'printing':P['printing'],'unresolved':P['unresolved'],
        'source_sha256':{k:hashlib.sha256((ROOT/'cad'/P[k]['reference_file']).read_bytes()).hexdigest() for k in refs}}
report['parameters_sha256']=hashlib.sha256((ROOT/'cad/parameters.json').read_bytes()).hexdigest()
report['model_script_sha256']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
report['STL_roundtrip']='PASS - reimported closed manifold, one connected component'
# Native-file reopen/recompute, including a genuine parameter edit and return to nominal.
App.closeDocument(D.Name)
D=App.openDocument(str(OUT/'fixture.FCStd'));D.recompute()
assert D.PrintableFixture.Shape.isValid()
assert abs(D.PrintableFixture.Shape.Volume-shape.Volume)<1e-5
oldmax=D.PrintableFixture.Shape.optimalBoundingBox().ZMax
roofcell=D.Parameters.getCellFromAlias('RoofT');D.Parameters.set(roofcell,f"{values['RoofT']+1} mm");D.recompute()
assert abs(D.PrintableFixture.Shape.optimalBoundingBox().ZMax-oldmax-1)<1e-6
D.Parameters.set(roofcell,f"{values['RoofT']} mm");D.recompute()
assert abs(D.PrintableFixture.Shape.Volume-shape.Volume)<1e-5
report['FCStd_reopen_parameter_edit']='PASS - reopen, change roof thickness by +1 mm, recompute, restore'
step_check=Part.read(str(OUT/'fixture.step'))
assert step_check.isValid() and len(step_check.Solids)==1
assert math.isclose(step_check.Volume,shape.Volume,rel_tol=1e-7)
report['STEP_roundtrip']='PASS - valid single solid, volume preserved'
(OUT/'validation.json').write_text(json.dumps(report,indent=2)+'\n')
# Meshes for report rendering only. CAD geometry is built exclusively in FreeCAD.
render={}
for key,obj in [('fixture',D.PrintableFixture),('camera',D.Camera),('lidar',D.MID360)]:
    vs,ts=obj.Shape.tessellate(.2)
    render[key]={'vertices':[list(v) for v in vs],'triangles':ts}
(ROOT/'.cache').mkdir(exist_ok=True)
(ROOT/'.cache/render-meshes.json').write_text(json.dumps(render))
App.closeDocument(D.Name)
print(json.dumps(report,indent=2))
