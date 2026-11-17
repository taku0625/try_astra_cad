"""Run native FreeCAD modeling, then create review images, PDF and ZIP.
FreeCAD is the CAD engine; Matplotlib only renders the exported meshes.
"""
import argparse
import hashlib
import json
import os
import subprocess
import shutil
import zipfile
from pathlib import Path
os.environ.setdefault('MPLCONFIGDIR','/tmp/try-astra-cad-mpl')
os.environ.setdefault('XDG_CACHE_HOME','/tmp/try-astra-cad-cache')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.patches import Rectangle, Polygon
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'output'
COLORS={'fixture':'#628ca5','camera':'#d6a04c','lidar':'#409b98','collar':'#999999','pipe':'#303840'}


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
    f.suptitle('FreeCAD | MID-360 tilted mount: Y -30 degrees',x=.05,ha='left',fontsize=21,weight='bold')
    f.text(.05,.927,'25 mm PIPE ASSUMED / ONE COLLAR / PRODUCT MOUNTING DIMENSIONS UNVERIFIED',fontsize=11,color='#286678')
    ax=f.add_subplot(121,projection='3d')
    add_meshes(ax,meshes,['fixture','camera','lidar','collar','pipe'])
    ax.set(xlim=(-120,95),ylim=(-60,60),zlim=(-135,70),xlabel='X / forward (mm)',ylabel='Y (mm)',zlabel='Z (mm)')
    ax.set_box_aspect((215,120,205));ax.view_init(elev=20,azim=35)
    ax.set_title(f"MID-360: Z +{p['lidar']['mount_rotation_z_deg']:g} deg, then Y {p['lidar']['mount_rotation_y_deg']:g} deg",fontsize=11)
    ax=f.add_subplot(222,projection='3d')
    add_meshes(ax,meshes,['fixture'])
    ax.set(xlim=(-120,35),ylim=(-55,55),zlim=(-42,68),xlabel='X',ylabel='Y',zlabel='Z')
    ax.set_box_aspect((155,110,110));ax.view_init(elev=22,azim=155)
    ax.set_title('One part | integral 30-degree LiDAR mount',fontsize=10.5)
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
    l=p['lidar']
    import math
    half=l['wedge_length_x']/2
    slope=-math.tan(math.radians(l['mount_rotation_y_deg']))
    ax.add_patch(Polygon([(-half,0),(half,0),(half,l['mount_center_z']+slope*half),
                          (-half,l['mount_center_z']-slope*half)],color=COLORS['fixture']))
    angle=math.radians(l['mount_rotation_y_deg'])
    # Side silhouette of the sensor envelope, schematic; assembly pane uses real STEP mesh.
    def tilted(x,z):
        return (math.cos(angle)*x+math.sin(angle)*z,l['mount_center_z']-math.sin(angle)*x+math.cos(angle)*z)
    ax.add_patch(Polygon([tilted(x,z) for x,z in [(-32,0),(32,0),(32,-60),(-32,-60)]],
                         facecolor=COLORS['lidar'],alpha=.55))
    ax.annotate('Y -30 deg',xy=tilted(0,-35),xytext=(-55,-75),fontsize=9,
                arrowprops=dict(arrowstyle='->',color=COLORS['lidar']))
    m=p['pipe_mount']
    ax.add_patch(Rectangle((m['center_x']-m['collar_od_reference']/2,-m['collar_height_reference']),m['collar_od_reference'],m['collar_height_reference'],color=COLORS['collar']))
    ax.add_patch(Rectangle((m['center_x']-m['pipe_od']/2,-42),m['pipe_od'],42,color=COLORS['pipe']))
    ax.axvline(plane,color='#38784c',ls='--',lw=1.3)
    ax.text(plane+3,66,'Forward rays',color='#38784c',fontsize=9)
    ax.annotate('',xy=(plane,-8),xytext=(front,-8),arrowprops=dict(arrowstyle='<->',color='#38784c'))
    ax.text((plane+front)/2,-18,f"{r['fov']['clearance_behind_plane_mm']:.1f} mm",ha='center',fontsize=9,color='#38784c')
    ax.text(-53,-31,'Dashed plane = assumed pupil lower bound\nActual pupil / full FOV need image verification',fontsize=8,color='#5c6871')
    ax.annotate('+X',xy=(70,30.5),xytext=(58,30.5),fontsize=9,arrowprops=dict(arrowstyle='->'))
    ax.set(xlim=(-120,75),ylim=(-100,77),title='SIDE | base supports camera (schematic)')
    ax.set_aspect('equal');ax.axis('off')
    f.text(.05,.074,f"Camera underside contact: {r['camera_supported_area_mm2']:.1f} / {r['camera_foot_area_mm2']:.1f} mm2 ({100*r['camera_support_fraction']:.1f}%)",fontsize=10.5)
    f.text(.05,.045,'Single PLA part + commercial screws. Confirm thread depth, mating plug, actual image and operating temperature.',fontsize=9,color='#a04428')
    f.subplots_adjust(top=.87,bottom=.14,wspace=.13,hspace=.29)
    f.savefig(OUT/'assembly_preview.png',dpi=160)
    with PdfPages(OUT/'review_drawing.pdf') as pdf:
        pdf.savefig(f)
        f2,ax=plt.subplots(figsize=(11.7,8.3));ax.axis('off')
        lines=[
          'ONE-PIECE REAR BRACKET + PROVISIONAL PIPE MOUNT (mm)','',
          'FreeCAD native tree and editable Parameters spreadsheet. STL contains fixture only.',
          f"Base: {b['length_x']:g} x {b['width_y']:g} x {t:g}, center ({b['center_x']:g},0), R{b['corner_radius']:g}; front X={front:g}.",
          f"Rear plate: {k['thickness']:g} thick x {k['width']:g} wide x {k['height_above_base']:g} above base; Z top={t+k['height_above_base']:g}.",
          f"Rear plate camera contact face X={back:g}; rear face X={back-k['thickness']:g}.",
          'Camera datum X=22 (8 mm rearward from previous X=30), Y=0; bottom Z=8.',
          'Camera roll: 180 deg about forward +X; source +Z points DOWN; rear connector below lens.',
          f"MID-360 mount center (0,0,{p['lidar']['mount_center_z']:g}); assembly Y pitch {p['lidar']['mount_rotation_y_deg']:g} deg.",
          f"Apply Z +{p['lidar']['mount_rotation_z_deg']:g} deg first, then Y {p['lidar']['mount_rotation_y_deg']:g} deg; LiDAR axis (+0.5,0,-0.866).",'',
          'CAMERA REAR ATTACHMENT:',
          f" 4 x dia {k['camera_clearance_diameter']:g} THRU, length {k['thickness']:g}, along X; Y=+/-12.5, Z=18/43 (25 x 25 pitch).",
          ' Source STEP rear face Y=-16.4: four entry loops diameter 2.57 at X/Z=+/-12.5.',
          ' M1.6 confirmed by user; STEP entry diameter is not a thread specification. Check insertion limit.',
          f" M{c['screw_diameter']:g}x{c['screw_length_under_head']:g} (head excluded) through {k['thickness']:g} plate: {r['camera_fastener']['insertion_without_washer_mm']:g} insertion, no washer.",
          f" Cable opening: dia {k['cable_diameter']:g} at Y=0, Z={r['camera_connector_center_global_mm'][2]:g}, extended UP {k['cable_upward_extension']:g} with 45-degree roof.",
          f" Cable opening top Z={r['cable_opening_top_z_mm']:.2f}; nominal plug envelope checked, actual plug not supplied.",
          f" Base relief below cable: {r['cable_base_relief_depth_mm']:g} deep, extends to rear edge for plug approach.",
          ' Two rear-only gussets: 5 mm wide, 16 mm deep, 26 mm high, with 1 mm root overlaps.','',
          'BASE HOLES:']
        for h in r['holes']:
            x,y=h['xy'];line=f" {h['type']:5s}: ({x:g},{y:g}), dia {h['diameter']:g} THRU"
            if 'counterbore' in h:line+=f"; Z={h['xyz'][2]:.2f}; angled dia {h['counterbore']:g} access, seat {h['seat_thickness']:g} thick"
            lines.append(line)
        lines += ['',
          'PRINT: PLA; tilted mount projects below Z=0. Select side-face orientation and supports in slicer.',
          ' 40% gyroid and 5 mm brim are starting values. Inspect overhangs/bridges in slicer.',
          ' Inspect angled head/driver channels, mount overhangs and small camera holes in slicer.',
          ' Temperature/speed follow filament and printer profiles. Deburr holes and contact faces.','',
          'ASSEMBLY:',
          '1. Attach tilted LiDAR to wedge using 4 angled M3x8 screws, insertion 3.5 mm: verify limit.',
          '2. With rear connector LOW, rest camera on base; slide rearward into cable opening.',
          '3. Fasten 4 rear camera screws from -X. Verify thread, insertion and tightening torque.',
          '4. Connect cables. Attach host via 4 M5 clearance holes; keep LiDAR/cable volumes clear.','',
          f"CAMERA SUPPORT: {r['camera_supported_area_mm2']:.1f} of {r['camera_foot_area_mm2']:.1f} mm2 bottom planes contact the base.",
          f"FOV: fixture Xmax={front:g}; assumed pupil lower-bound X={plane:g}; rear clearance {plane-front:.1f}.",
          'Fixture only is behind the plane; tilted LiDAR protrudes forward and may enter camera FOV.',
          'Verify full-frame images (including LiDAR), cable clearance, PLA temperature/creep and loads.',
          f"Solid PLA mass at 1.24 g/cm3: {r['pla_solid_mass_g_at_1_24']:.1f} g; actual print includes infill/support effects."]
        ax.text(.01,.98,'\n'.join(lines),va='top',family='monospace',fontsize=8.1,linespacing=1.18)
        pdf.savefig(f2);plt.close(f2)
        f3,ax=plt.subplots(figsize=(11.7,8.3));ax.axis('off')
        m=p['pipe_mount']
        notes=[
            'INTEGRATED BASE / PROVISIONAL SCS2512CN2 PIPE INTERFACE',
            'NOT A VERIFIED PRODUCT INTERFACE - CHECK MANUFACTURER DRAWING BEFORE PRINTING',
            '',
            f"Pipe axis: Z; center X={m['center_x']:g}, Y={m['center_y']:g} mm; assumed OD {m['pipe_od']:g} mm.",
            'Pipe top must stay at or below Z=0 to keep the rear cable path clear.',
            'Pipe wall thickness in the assembly is a visualization assumption only.',
            f"One base: {b['length_x']:g} x {b['width_y']:g} x {t:g} mm, bottom Z=0; no stepped extension.",
            'Pipe clearance hole: 25.6 mm. Two radial slots along Y, width 5.5 mm.',
            'Opposed bolt-center pitch adjusts from 34 to 40 mm; nominal M5 shaft clearance.',
            'Collar is BELOW the base. Bolts enter from ABOVE; use suitable washers.',
            'Collar reference envelope: OD45 x H12 with bore25; NOT verified SCS2512CN2 CAD.',
            'Confirm that the real collar has matching AXIAL mounting holes and insertion depth.',
            'Do not use the slit-clamping screw as a fixture attachment screw.',
            'Choose bolt size and length only after checking collar threads and washer thickness.',
            'Check clamp screw/tool access with the actual collar; simplified envelope has no clamp screw.',
            'Check pipe crushing, single-collar slip/rotation and rear cantilever strength with actual hardware.',
            '',
            'Assembly: bolt collar under base, insert pipe from below to Z<=0, then tighten to manufacturer limits.',
            'Printed object includes integral tilted LiDAR wedge; collar and pipe are bought-part references.',
            'CAD checks: one solid; STL/STEP roundtrip; no sensor/envelope collisions; slot/tool access.',
        ]
        ax.text(.02,.96,'\n'.join(notes),va='top',family='monospace',fontsize=10,linespacing=1.5)
        pdf.savefig(f3);plt.close(f3)
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
    # Distinct filenames identify the actual CAD inputs, avoiding ambiguous downloads.
    input_digest=hashlib.sha256((ROOT/'cad/parameters.json').read_bytes()+(ROOT/'cad/freecad_build.py').read_bytes()).hexdigest()[:10]
    stem=f"fixture_mid360_z{p['lidar']['mount_rotation_z_deg']:g}_y{p['lidar']['mount_rotation_y_deg']:g}_{input_digest}"
    manifest={'model_input_id':input_digest,'lidar_mount_rotation_z_deg':p['lidar']['mount_rotation_z_deg'],'lidar_mount_rotation_y_deg':p['lidar']['mount_rotation_y_deg'],'files':{}}
    for suffix in ['stl','step']:
        target=OUT/f'{stem}.{suffix}'
        shutil.copyfile(OUT/f'fixture.{suffix}',target)
        manifest['files'][suffix]={'name':target.name,'sha256':hashlib.sha256(target.read_bytes()).hexdigest()}
    (OUT/'latest_print_files.json').write_text(json.dumps(manifest,indent=2)+'\n')
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
