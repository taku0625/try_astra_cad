"""Run native FreeCAD modeling, then create review images, PDF and ZIP.
FreeCAD is the CAD engine; Matplotlib only renders the exported meshes.
"""
import argparse
import hashlib
import json
import os
import subprocess
import zipfile
from pathlib import Path
os.environ.setdefault('MPLCONFIGDIR','/tmp/try-astra-cad-mpl')
os.environ.setdefault('XDG_CACHE_HOME','/tmp/try-astra-cad-cache')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.patches import Rectangle
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'output'
COLORS={'fixture':'#628ca5','camera':'#d6a04c','lidar':'#409b98'}


def add_meshes(ax,meshes,keys):
    faces=[];colors=[]
    for key in keys:
        item=meshes[key];verts=item['vertices']
        faces.extend([[verts[i] for i in t] for t in item['triangles']])
        colors.extend([COLORS[key]]*len(item['triangles']))
    ax.add_collection3d(Poly3DCollection(faces,facecolors=colors,edgecolors=colors,linewidths=0,shade=True,rasterized=True))


def render(p,r,meshes):
    b,c,k=p['plate'],p['camera'],p['rear_bracket']
    cx=c['forward_offset'];t=b['thickness'];back=cx+c['rear_face_y_source']
    f=plt.figure(figsize=(15,10),facecolor='white')
    f.suptitle('FreeCAD | Rear screw bracket + camera support base',x=.05,ha='left',fontsize=21,weight='bold')
    f.text(.05,.927,'CAMERA ROLL CORRECTED 180 DEG / REAR CONNECTOR LOW / ONE PLA PRINT',fontsize=11,color='#286678')
    ax=f.add_subplot(121,projection='3d')
    add_meshes(ax,meshes,['fixture','camera','lidar'])
    ax.set(xlim=(-65,65),ylim=(-60,60),zlim=(-70,70),xlabel='X / forward (mm)',ylabel='Y (mm)',zlabel='Z (mm)')
    ax.set_box_aspect((130,120,140));ax.view_init(elev=20,azim=35)
    ax.set_title('Camera above, inverted MID-360 below',fontsize=11)
    ax=f.add_subplot(222,projection='3d')
    add_meshes(ax,meshes,['fixture'])
    ax.set(xlim=(-60,35),ylim=(-55,55),zlim=(0,68),xlabel='X',ylabel='Y',zlabel='Z')
    ax.set_box_aspect((95,110,68));ax.view_init(elev=22,azim=155)
    ax.set_title('Rear view | 4 screw holes + teardrop cable opening',fontsize=10.5)
    ax=f.add_subplot(224)
    plane=r['fov']['pupil_lower_bound_x'];front=r['fov']['fixture_max_x']
    ax.axvspan(plane,75,facecolor='#dceee2',alpha=.75)
    ax.add_patch(Rectangle((b['center_x']-b['length_x']/2,0),b['length_x'],t,color=COLORS['fixture'],alpha=.9))
    ax.add_patch(Rectangle((back-k['thickness'],t),k['thickness'],k['height_above_base'],color=COLORS['fixture'],alpha=.8))
    ax.add_patch(Rectangle((cx-16.4,t),38.1,45,facecolor=COLORS['camera'],edgecolor='#8c672c',alpha=.55))
    ax.add_patch(Rectangle((cx+22.8,t+10.5),6.7,24,facecolor=COLORS['camera']))
    ax.add_patch(Rectangle((cx-28.3,r['camera_connector_center_global_mm'][2]-3.75),11.9,7.5,facecolor=COLORS['camera']))
    ax.annotate('Rear connector LOW',xy=(cx-24,r['camera_connector_center_global_mm'][2]),
                xytext=(-58,68),fontsize=8,color='#8c672c',arrowprops=dict(arrowstyle='->',color='#8c672c'))
    ax.axvline(plane,color='#38784c',ls='--',lw=1.3)
    ax.text(plane+3,66,'Forward rays',color='#38784c',fontsize=9)
    ax.annotate('',xy=(plane,-8),xytext=(front,-8),arrowprops=dict(arrowstyle='<->',color='#38784c'))
    ax.text((plane+front)/2,-18,f"{r['fov']['clearance_behind_plane_mm']:.1f} mm",ha='center',fontsize=9,color='#38784c')
    ax.text(-53,-31,'Dashed plane = assumed pupil lower bound\nActual pupil / full FOV need image verification',fontsize=8,color='#5c6871')
    ax.annotate('+X',xy=(70,30.5),xytext=(58,30.5),fontsize=9,arrowprops=dict(arrowstyle='->'))
    ax.set(xlim=(-65,75),ylim=(-42,77),title='SIDE | base supports camera (schematic)')
    ax.set_aspect('equal');ax.axis('off')
    f.text(.05,.074,f"Camera underside contact: {r['camera_supported_area_mm2']:.1f} / {r['camera_foot_area_mm2']:.1f} mm2 ({100*r['camera_support_fraction']:.1f}%)",fontsize=10.5)
    f.text(.05,.045,'Single PLA part + commercial screws. Confirm thread depth, mating plug, actual image and operating temperature.',fontsize=9,color='#a04428')
    f.subplots_adjust(top=.87,bottom=.14,wspace=.13,hspace=.29)
    f.savefig(OUT/'assembly_preview.png',dpi=160)
    with PdfPages(OUT/'review_drawing.pdf') as pdf:
        pdf.savefig(f)
        f2,ax=plt.subplots(figsize=(11.7,8.3));ax.axis('off')
        lines=[
          'ONE-PIECE REAR BRACKET | Dimensions and assembly (mm)','',
          'FreeCAD native tree and editable Parameters spreadsheet. STL contains fixture only.',
          f"Base: {b['length_x']:g} x {b['width_y']:g} x {t:g}, center ({b['center_x']:g},0), R{b['corner_radius']:g}; front X={front:g}.",
          f"Rear plate: {k['thickness']:g} thick x {k['width']:g} wide x {k['height_above_base']:g} above base; Z top={t+k['height_above_base']:g}.",
          f"Rear plate camera contact face X={back:g}; rear face X={back-k['thickness']:g}.",
          'Camera datum X=22 (8 mm rearward from previous X=30), Y=0; bottom Z=8.',
          'Camera roll: 180 deg about forward +X; source +Z points DOWN; rear connector below lens.',
          'MID-360 datum XY=(0,0), mounting face Z=0, scan downward -Z. Both cables exit -X.','',
          'CAMERA REAR ATTACHMENT:',
          ' 4 x dia 3.6 THRU along X at Y=+/-12.5, Z=18 and 43 (25 x 25 pitch).',
          ' Source STEP rear face Y=-16.4: four entry loops diameter 2.57 at X/Z=+/-12.5.',
          ' M3 thread inferred; axial solid begins 7 mm inside source holes. Verify actual specification.',
          ' M3x10 through 6 mm rear plate gives 4 mm insertion WITHOUT washers (conditional).',
          f" Cable opening Y=0, Z={r['camera_connector_center_global_mm'][2]:g}: dia 16 + 45-degree teardrop roof.",
          f" Base relief below cable: {r['cable_base_relief_depth_mm']:g} deep, extends to rear edge for plug approach.",
          ' Two rear-only gussets: 5 mm wide, 16 mm deep, 26 mm high, with 1 mm root overlaps.','',
          'BASE HOLES:']
        for h in r['holes']:
            x,y=h['xy'];line=f" {h['type']:5s}: ({x:g},{y:g}), dia {h['diameter']:g} THRU"
            if 'counterbore' in h:line+=f"; TOP counterbore dia {h['counterbore']:g} x {h['depth']:g} deep"
            lines.append(line)
        lines += ['',
          'PRINT: PLA; base underside on bed; 0.4 nozzle, 0.2 layers, 5 walls, 6 top/bottom layers.',
          ' 40% gyroid and 5 mm brim are starting values. Inspect overhangs/bridges in slicer.',
          ' Start with supports off: cable opening has a 45-degree roof, horizontal screw holes are small.',
          ' Temperature/speed follow filament and printer profiles. Deburr holes and contact faces.','',
          'ASSEMBLY:',
          '1. Attach inverted LiDAR to base with 4 screws. M3x8 implies 3.5 mm insertion: verify limit.',
          '2. With rear connector LOW, rest camera on base; slide rearward into cable opening.',
          '3. Fasten 4 rear camera screws from -X. Verify thread, insertion and tightening torque.',
          '4. Connect cables. Attach host via 4 M5 clearance holes; keep LiDAR/cable volumes clear.','',
          f"CAMERA SUPPORT: {r['camera_supported_area_mm2']:.1f} of {r['camera_foot_area_mm2']:.1f} mm2 bottom planes contact the base.",
          f"FOV: fixture Xmax={front:g}; assumed pupil lower-bound X={plane:g}; rear clearance {plane-front:.1f}.",
          'All fixture material remains behind this plane. Exact pupil and full FOV absent from STEP.',
          'Verify full-frame images, cable plug clearance, PLA temperature/creep, loads and vibration.',
          f"Solid PLA mass at 1.24 g/cm3: {r['pla_solid_mass_g_at_1_24']:.1f} g; actual print includes infill/support effects."]
        ax.text(.01,.98,'\n'.join(lines),va='top',family='monospace',fontsize=8.1,linespacing=1.18)
        pdf.savefig(f2);plt.close(f2)
    plt.close(f)


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--render-only',action='store_true');args=parser.parse_args()
    if not args.render_only:
        subprocess.run([os.environ.get('FREECAD_PYTHON','/usr/bin/python3'),str(ROOT/'cad/freecad_build.py')],cwd=ROOT,check=True)
    p=json.loads((ROOT/'cad/parameters.json').read_text());r=json.loads((OUT/'validation.json').read_text())
    for filename,key in [('parameters.json','parameters_sha256'),('freecad_build.py','model_script_sha256')]:
        assert hashlib.sha256((ROOT/'cad'/filename).read_bytes()).hexdigest()==r[key], 'CAD cache is stale; rerun without --render-only'
    meshes=json.loads((ROOT/'.cache/render-meshes.json').read_text())
    render(p,r,meshes)
    # Retire only the known generated parts of the former multi-part design.
    for name in ['plate.step','plate.stl','plate.dxf','plate_bottom.dxf','cap.step','cap.stl','cap.dxf',
                 'post_1.step','post_1.stl','post_2.step','post_2.stl','top_pad.step','top_pad.stl']:
        (OUT/name).unlink(missing_ok=True)
    for backup in OUT.glob('*.FCStd1'):backup.unlink()
    (ROOT/'cad/.render-meshes.json').unlink(missing_ok=True)
    with zipfile.ZipFile(ROOT/'fixture_cad.zip','w',compression=zipfile.ZIP_DEFLATED,compresslevel=6) as z:
        for name in ['README.md','requirements.txt']:z.write(ROOT/name,name)
        for folder in ['cad','output']:
            for path in sorted((ROOT/folder).rglob('*')):
                if (path.is_file() and '__pycache__' not in path.parts and not path.name.startswith('.')
                        and path.suffix not in ['.FCBak','.FCStd1']):
                    z.write(path,path.relative_to(ROOT))
    with zipfile.ZipFile(ROOT/'fixture_cad.zip') as z:assert z.testzip() is None
    print('FreeCAD model, single-part STL, report and ZIP complete.')


if __name__=='__main__':main()
