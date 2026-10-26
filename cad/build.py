"""Regenerate fixture from supplied sensor STEP. Units mm; review fastening before use."""
import os, json, math, hashlib
from pathlib import Path
os.environ.setdefault('MPLCONFIGDIR','/tmp/coplanar-mpl')
os.environ.setdefault('XDG_CACHE_HOME','/tmp/coplanar-cache')
import cadquery as cq
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, FancyBboxPatch
from matplotlib.backends.backend_pdf import PdfPages
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'output'
P=json.loads((ROOT/'cad/parameters.json').read_text())
COLORS={'plate':'#c3ced7','post_1':'#7e95a8','post_2':'#7e95a8','cap':'#658299','top_pad':'#383f43','camera':'#d79b46','lidar':'#459ba5'}
def box(dx,dy,dz,x,y,z):
    return cq.Workplane('XY').box(dx,dy,dz,centered=(True,True,False)).translate((x,y,z))
def bore(s,x,y,d,z,h):
    return s.cut(cq.Workplane('XY').center(x,y).circle(d/2).extrude(h).translate((0,0,z)))
def build():
    b=P['plate'];c=P['clamp'];t=b['thickness']
    camera=cq.importers.importStep(str(ROOT/'cad'/P['camera']['reference_file']))
    camera=camera.rotate((0,0,0),(0,0,1),P['camera']['rotation_z_deg']).translate((*P['camera']['origin_xy'],t-P['camera']['original_bottom_z']))
    lidar=cq.importers.importStep(str(ROOT/'cad'/P['lidar']['reference_file']))
    lidar=lidar.rotate((0,0,0),(1,0,0),P['lidar']['rotation_x_deg']).translate((*P['lidar']['origin_xy'],t-P['lidar']['original_bottom_y']))
    plate=box(b['length_x'],b['width_y'],t,b['center_x'],0,0).edges('|Z').fillet(b['corner_radius'])
    holes=[(x,y,b['host_hole_diameter'],'HOST') for x,y in b['host_holes_xy']]
    lx,ly=P['lidar']['origin_xy']
    holes += [(lx+x,ly+y,P['lidar']['clearance_diameter'],'LIDAR') for x,y in P['lidar']['holes_local_xy']]
    posts={}
    cap=box(c['cap_depth_x'],2*c['post_offset_y']+c['post_width_y'],c['cap_thickness'],c['center_x'],c['center_y'],t+c['post_height'])
    for i,sign in enumerate([-1,1]):
        x=c['center_x'];y=c['center_y']+sign*c['post_offset_y']
        holes.append((x,y,c['clearance_diameter'],'POST'))
        post=box(c['post_depth_x'],c['post_width_y'],c['post_height'],x,y,t)
        post=bore(post,x,y,c['tap_drill_diameter'],t,c['tap_depth'])
        post=bore(post,x,y,c['tap_drill_diameter'],t+c['post_height']-c['tap_depth'],c['tap_depth'])
        posts['post_'+str(i+1)]=post
        cap=bore(cap,x,y,c['clearance_diameter'],t+c['post_height'],c['cap_thickness'])
    for x,y,d,_ in holes:
        before=plate.val().Volume();plate=bore(plate,x,y,d,0,t)
        assert math.isclose(before-plate.val().Volume(),math.pi*(d/2)**2*t,rel_tol=1e-7),'Hole overlap/clipping'
    pad=box(c['pad_depth_x'],c['pad_width_y'],c['pad_compressed_thickness'],c['center_x'],c['center_y'],t+c['post_height']-c['pad_compressed_thickness'])
    return {'plate':plate,**posts,'cap':cap,'top_pad':pad,'camera':camera,'lidar':lidar},holes
def validate(parts,holes):
    t=P['plate']['thickness']
    for k,v in parts.items():
        assert len(v.solids().vals())>0 and all(s.isValid() for s in v.solids().vals()),k
    bottoms={k:parts[k].val().BoundingBox().zmin for k in ['camera','lidar']}
    assert max(abs(v-t) for v in bottoms.values())<5e-6,bottoms
    collisions={};names=list(parts)
    for i,k in enumerate(names):
        for j in names[i+1:]:
            vol=sum(s.Volume() for s in parts[k].intersect(parts[j]).solids().vals())
            if vol>1e-5:collisions[k+' / '+j]=vol
    assert not collisions,collisions
    contacts={}
    for k in ['camera','lidar']:
        area=0
        for f in parts[k].faces().vals():
            b=f.BoundingBox()
            if f.geomType()=='PLANE' and abs(b.zmin-t)<1e-5 and abs(b.zmax-t)<1e-5:area+=f.Area()
        assert area>20,(k,area)
        contacts[k]=area
    return {'status':P['status'],'units':'mm','bottom_z':bottoms,'coplanarity_error_mm':abs(bottoms['camera']-bottoms['lidar']),
      'bottom_planar_face_area_mm2':contacts,'positive_volume_interferences':collisions,
      'solids':{k:len(v.solids().vals()) for k,v in parts.items()},
      'metal_mass_kg_at_2700_kg_m3':sum(v.val().Volume()*2.7e-6 for k,v in parts.items() if k not in ['camera','lidar','top_pad']),
      'plate_holes':[{'xy':[x,y],'diameter':d,'function':tag} for x,y,d,tag in holes],
      'source_sha256':{k:hashlib.sha256((ROOT/'cad'/P[k]['reference_file']).read_bytes()).hexdigest() for k in ['camera','lidar']},
      'unresolved':P['unresolved']}
def dim(ax,a,b,label,offset=(0,0)):
    ax.annotate('',xy=b,xytext=a,arrowprops=dict(arrowstyle='<->',color='#405c72',lw=.8))
    ax.text((a[0]+b[0])/2+offset[0],(a[1]+b[1])/2+offset[1],label,ha='center',va='center',fontsize=8,backgroundcolor='white')
def drawing(parts,holes,report):
    fig=plt.figure(figsize=(15,10),facecolor='white')
    fig.suptitle('C2-176DEG + MID-360 | Coplanar mounting fixture',x=.05,ha='left',fontsize=21,weight='bold')
    fig.text(.05,.925,'USER STEP GEOMETRY / common bottom datum Z=6 mm / mechanical review required',fontsize=11,color='#a04428')
    ax=fig.add_subplot(121,projection='3d')
    all_faces=[]; all_colors=[]
    for k,v in parts.items():
        vv,tt=v.val().tessellate(.35,.25)
        faces=[[vv[i].toTuple() for i in tri] for tri in tt]
        all_faces.extend(faces); all_colors.extend([COLORS[k]]*len(faces))
    ax.add_collection3d(Poly3DCollection(all_faces,facecolors=all_colors,edgecolors=all_colors,linewidths=0,shade=True,rasterized=True))
    ax.set(xlim=(-90,85),ylim=(-120,120),zlim=(0,90),xlabel='X / forward (mm)',ylabel='Y (mm)',zlabel='Z (mm)')
    ax.set_box_aspect((175,240,90));ax.view_init(elev=32,azim=35)
    ax.set_title('Supplied sensor models + machined parts',fontsize=11)
    ax.text(25,-50,70,'C2 camera',fontsize=10,color='#986023');ax.text(-20,50,83,'MID-360',fontsize=10,color='#26707b')
    ax=fig.add_subplot(222)
    b=P['plate']; c=P['clamp']; xmin=b['center_x']-b['length_x']/2; xmax=xmin+b['length_x']; hy=b['width_y']/2
    ax.add_patch(FancyBboxPatch((xmin,-hy),b['length_x'],b['width_y'],boxstyle=f"round,pad=0,rounding_size={b['corner_radius']}",facecolor='#edf2f6',edgecolor='#708698'))
    for x,y,d,tag in holes:ax.add_patch(Circle((x,y),d/2,facecolor='white',edgecolor='#344e62'))
    for k in ['camera','lidar','post_1','post_2','cap']:
        bb=parts[k].val().BoundingBox()
        ax.add_patch(Rectangle((bb.xmin,bb.ymin),bb.xlen,bb.ylen,facecolor=COLORS[k],edgecolor=COLORS[k],alpha=.4))
    ax.text(25,-50,'C2',ha='center',fontsize=10);ax.text(-20,50,'MID-360',ha='center',fontsize=10)
    ax.annotate('+X / camera front',xy=(118,-50),xytext=(78,-50),fontsize=8,arrowprops=dict(arrowstyle='->'))
    dim(ax,(xmin,-hy-13),(xmax,-hy-13),f"{b['length_x']:g}");dim(ax,(xmin-12,-hy),(xmin-12,hy),f"{b['width_y']:g}",(-8,0))
    ax.set(xlim=(-115,155),ylim=(-130,120),title='TOP | mm, bounding outlines');ax.set_aspect('equal');ax.axis('off')
    ax=fig.add_subplot(224)
    for k in parts:
        bb=parts[k].val().BoundingBox()
        ax.add_patch(Rectangle((bb.ymin,bb.zmin),bb.ylen,bb.zlen,facecolor=COLORS[k],edgecolor=COLORS[k],alpha=.6))
    ax.axhline(6,color='#be3c36',ls='--',lw=1.2)
    ax.text(0,83,'Camera bottom = LiDAR bottom = Z 6',ha='center',color='#be3c36',fontsize=10)
    ax.set(xlim=(-120,120),ylim=(-10,90),title='FRONT | common underside datum');ax.set_aspect('equal');ax.axis('off')
    fig.text(.05,.085,f"Base {b['length_x']:g} x {b['width_y']:g} x {b['thickness']:g} mm | A6061-T6 proposed | Camera: top clamp | LiDAR: bottom screws",fontsize=11)
    fig.text(.05,.057,'No bottom pads or height offsets. Thread depth, clamp pressure, cables and FOV need confirmation.',fontsize=10,color='#a04428')
    fig.subplots_adjust(top=.87,bottom=.15,wspace=.1,hspace=.28);fig.savefig(OUT/'assembly_preview.png',dpi=160)
    with PdfPages(OUT/'review_drawing.pdf') as pdf:
        pdf.savefig(fig)
        fig2,ax2=plt.subplots(figsize=(11.7,8.3));ax2.axis('off')
        text=[
          'FIXTURE | Part dimensions and hole coordinates (mm)', '',
          'GLOBAL: +X camera forward, +Z up. Plate bottom Z=0, BOTH sensor bottoms Z=6.',
          f"BASE: {b['length_x']:g}(X) x {b['width_y']:g}(Y) x {b['thickness']:g}, center ({b['center_x']:g},0), R{b['corner_radius']:g}.",
          *[f"  {tag}: dia {d:g} THRU at ({x:g}, {y:g})" for x,y,d,tag in holes], '',
          f"POSTS (quantity 2): {c['post_depth_x']:g}(X) x {c['post_width_y']:g}(Y) x {c['post_height']:g} high.",
          '  Centered diameter 3.3 blind drill depth 8 on BOTH ends; tap M4 proposal.',
          '  Threads are not modeled. Top/bottom blind holes are separate.',
          f"CAP: {c['cap_depth_x']:g}(X) x {2*c['post_offset_y']+c['post_width_y']:g}(Y) x {c['cap_thickness']:g}; 2 x dia {c['clearance_diameter']:g}, pitch {2*c['post_offset_y']:g}.",
          '  Global center (17,-50); underside Z=51.2.',
          'TOP PAD: 8 x 30, compressed thickness 0.2; initial 0.3 proposed.',
          '  Camera top Z=51.0. No pad underneath either sensor.', '',
          'FASTENERS (proposals; verify lengths and threads):',
          '  LiDAR: 4 x M3; length = 6 mm plate + manufacturer approved insertion.',
          '  Clamp: 4 x M4 (2 top, 2 bottom); choose <8 mm thread engagement.',
          '  Host: 4 x M5; host-dependent length, >=5 mm underside head clearance.', '',
          'MODEL EVIDENCE:',
          '  C2 source optical +Y -> global +X; bottom Z=-22.5 -> global Z=6.',
          '  MID source upright +Y -> global +Z; bottom Y=-25.9171 -> global Z=6.',
          '  MID holes source X=+/-24, Z=+/-18, 48 x 36 pitch.',
          '  R1.5 entrances and R1.25 inner bores suggest M3; VERIFY actual threads.', '',
          'REVIEW: cable plugs/bends, clamp pressure, friction, thread depth, FOV, loads.',
          'Plate holes are through clearance holes. Posts require tapping after drilling.',
          'Deburr edges. Mounting face flatness 0.05 and Ra 3.2 proposed; validate needs.',
          f"Estimated metal mass: {report['metal_mass_kg_at_2700_kg_m3']:.3f} kg.",
          'Individual STEP files + coordinate tables define geometry; review before machining.'
        ]
        ax2.text(.01,.98,'\n'.join(text),va='top',family='monospace',fontsize=8.6,linespacing=1.23)
        pdf.savefig(fig2);plt.close(fig2)
    plt.close(fig)
def main():
    OUT.mkdir(exist_ok=True);parts,holes=build();report=validate(parts,holes)
    for k in ['plate','post_1','post_2','cap','top_pad']:
        cq.exporters.export(parts[k],str(OUT/(k+'.step')))
        cq.exporters.export(parts[k],str(OUT/(k+'.stl')),tolerance=.05,angularTolerance=.1)
        rt=cq.importers.importStep(str(OUT/(k+'.step')))
        assert rt.val().isValid() and math.isclose(rt.val().Volume(),parts[k].val().Volume(),rel_tol=1e-7)
    for k in ['plate','cap']:cq.exporters.export(parts[k].faces('<Z').wires(),str(OUT/(k+'.dxf')))
    a=cq.Assembly(name='C2_MID360_COPLANAR_FIXTURE')
    for k,v in parts.items():a.add(v,name=k,color=cq.Color(COLORS[k]))
    a.export(str(OUT/'assembly.step'));report['step_roundtrip']='PASS: all five generated components'
    drawing(parts,holes,report)
    (OUT/'validation.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
if __name__=='__main__':main()
