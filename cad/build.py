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
    b,c,k=p['plate'],p['camera'],p['cage']
    cx=c['forward_offset'];t=b['thickness']
    f=plt.figure(figsize=(15,10),facecolor='white')
    f.suptitle('FreeCAD | Single-piece PLA sensor mount',x=.05,ha='left',fontsize=22,weight='bold')
    f.text(.05,.927,'CAMERA +30 mm FORWARD / ONE PRINTED PART / MID-360 DOWN / BOTH CABLE PORTS REARWARD',fontsize=10.5,color='#286678')
    ax=f.add_subplot(121,projection='3d')
    add_meshes(ax,meshes,['fixture','camera','lidar'])
    ax.set(xlim=(-65,70),ylim=(-60,60),zlim=(-70,70),xlabel='X / forward (mm)',ylabel='Y (mm)',zlabel='Z (mm)')
    ax.set_box_aspect((135,120,140));ax.view_init(elev=20,azim=35)
    ax.set_title('Assembly with supplied sensor STEP models',fontsize=11)
    ax=f.add_subplot(222,projection='3d')
    add_meshes(ax,meshes,['fixture'])
    ax.set(xlim=(-60,40),ylim=(-55,55),zlim=(0,65),xlabel='X',ylabel='Y',zlabel='Z')
    ax.set_box_aspect((100,110,65));ax.view_init(elev=25,azim=45)
    ax.set_title('Print this part only: 90 x 100 x 58.6 mm',fontsize=11)
    ax=f.add_subplot(224)
    plane=r['fov']['pupil_lower_bound_x'];front=r['fov']['fixture_max_x']
    ax.axvspan(plane,85,facecolor='#dceee2',alpha=.75)
    ax.add_patch(Rectangle((b['center_x']-b['length_x']/2,0),b['length_x'],t,color=COLORS['fixture'],alpha=.9))
    # Side projection of rear camera sleeve, housing, barrel and cable connector.
    ax.add_patch(Rectangle((cx+k['rear_relative_x'],t),k['length_x'],k['inside_height']+k['roof_thickness'],color=COLORS['fixture'],alpha=.55))
    ax.add_patch(Rectangle((cx-16.4,t),38.1,45,facecolor=COLORS['camera'],edgecolor='#8c672c',alpha=.55))
    ax.add_patch(Rectangle((cx+22.8,t+10.5),6.7,24,facecolor=COLORS['camera']))
    ax.add_patch(Rectangle((cx-28.3,t+33.75),11.9,7.5,facecolor=COLORS['camera']))
    ax.axvline(plane,color='#38784c',ls='--',lw=1.3)
    ax.text(plane+3,65,'Forward rays',color='#38784c',fontsize=9)
    ax.annotate('',xy=(plane,-8),xytext=(front,-8),arrowprops=dict(arrowstyle='<->',color='#38784c'))
    ax.text((plane+front)/2,-18,f"{r['fov']['clearance_behind_plane_mm']:.1f} mm",ha='center',fontsize=9,color='#38784c')
    ax.text(-53,-31,'Dashed plane = assumed pupil lower bound\nActual pupil / full FOV need image verification',fontsize=8,color='#5c6871')
    ax.annotate('+X',xy=(80,30.5),xytext=(68,30.5),fontsize=9,arrowprops=dict(arrowstyle='->'))
    ax.set(xlim=(-65,85),ylim=(-42,77),title='SIDE | fixture behind lens barrel (schematic)')
    ax.set_aspect('equal');ax.axis('off')
    f.text(.05,.074,'PLA FDM: plate underside on bed | supports required inside sleeve | 0.2 mm layers, 5 walls, 40% infill',fontsize=10.5)
    f.text(.05,.045,'One printed part + commercial screws/nuts. Verify fit, actual image, clamp retention and temperature before use.',fontsize=9,color='#a04428')
    f.subplots_adjust(top=.87,bottom=.14,wspace=.13,hspace=.29)
    f.savefig(OUT/'assembly_preview.png',dpi=160)
    with PdfPages(OUT/'review_drawing.pdf') as pdf:
        pdf.savefig(f)
        f2,ax=plt.subplots(figsize=(11.7,8.3));ax.axis('off')
        lines=[
          'SINGLE-PIECE PLA FIXTURE | Dimensions, printing and assembly (mm)','',
          'FreeCAD native feature tree + editable Parameters spreadsheet; exported STL is fixture only.',
          f"Base: {b['length_x']:g} x {b['width_y']:g} x {t:g}, XY center ({b['center_x']:g},0), R{b['corner_radius']:g}.",
          f"Sleeve: X={cx+k['rear_relative_x']:g} to {cx+k['rear_relative_x']+k['length_x']:g}; internal {k['inside_width']:g}(Y) x {k['inside_height']:g}(Z).",
          f"Walls {k['wall_thickness']:g}; roof {k['roof_thickness']:g}; overall printed height {r['fixture_bounds_mm']['z'][1]:g}.",
          'Camera origin X=30, Y=0; camera bottom Z=8. LiDAR mounting face Z=0, scan -Z.',
          'Camera forward +X. Both cable ports point rearward -X.','',
          'BASE HOLES:']
        for h in r['holes']:
            x,y=h['xy'];s=f" {h['type']:5s}: ({x:g},{y:g}), dia {h['diameter']:g} THRU"
            if 'counterbore' in h:s+=f"; TOP counterbore dia {h['counterbore']:g} x {h['depth']:g} deep"
            lines.append(s)
        lines += ['',
          'Roof driver access: 2 x dia 8 at XY=(24,+/-18), aligned to forward LiDAR screws.',
          'Clamp screw axes: X=25, Z=22.5 and 38.5; direction Y; dia 4.6 through +Y wall.',
          'M4 nut pockets: 7.3 across flats, 3.4 deep from INSIDE of +Y wall.',
          'Use two M4 nylon-tipped screws + two M4 nuts (commercial parts).','',
          'PRINT: PLA; 0.4 nozzle, 0.2 layer, 5 walls, 6 top/bottom layers, 40% gyroid, 5 mm brim.',
          'Orient base underside Z=0 on bed. Supports MUST reach sleeve roof from the base top.',
          'Use support on model/everywhere; remove from open front/rear. Deburr holes/pockets.',
          'Temperature/speed follow the actual PLA and printer profile. No machine G-code supplied.','',
          'ASSEMBLY:',
          '1. Remove supports. Insert 2 M4 nuts from sleeve interior; back off clamp screw tips.',
          '2. Fasten inverted LiDAR: rear screws directly, front screws through roof access holes.',
          '   M3x8 gives 3.5 mm insertion with the modeled counterbores: verify actual allowed depth.',
          '3. Slide camera in from +X, rear connector first. Seat and tighten nylon tips gently.',
          '   M4x12 is a starting choice; verify actual screw tip/head geometry and reach.',
          '4. Use 4 host M5 holes; host must leave the suspended LiDAR and rear cables clear.','',
          f"OPTICS: fixture Xmax={front:g}; assumed pupil lower-bound plane X={plane:g}; rear margin {plane-front:g}.",
          'All fixture material is behind this plane, excluding it from the assumed forward hemisphere.',
          'Exact entrance pupil and full calibrated FOV are absent from STEP: verify actual images.',
          'PLA is a prototype material here: verify sensor heating, creep, vibration and retention.',
          f"Solid PLA mass at 1.24 g/cm3: {r['pla_solid_mass_g_at_1_24']:.1f} g; slicer mass differs with infill/support."]
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
