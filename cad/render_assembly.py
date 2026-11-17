"""Assembly and exploded drawings from the validated native CAD meshes."""
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.collections import PolyCollection
from matplotlib.patches import Patch, Circle, Polygon
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


COLORS = {
    'base': '#5985a1', 'lidar_adapter': '#d7833f',
    'camera': '#d5b65e', 'lidar': '#368e88',
    'collar': '#959ca1', 'pipe': '#393f44',
    'bolts': '#424b58', 'washers': '#a1a9b3', 'nuts': '#737e8c',
    'lidar_screws': '#424b58',
}


def add_parts(ax, meshes, keys, offsets=None):
    faces, colors = [], []
    for key in keys:
        item = meshes[key]
        vertices = np.asarray(item['vertices']) + np.asarray((offsets or {}).get(key, (0, 0, 0)))
        triangles = vertices[np.asarray(item['triangles'])]
        faces.extend(triangles)
        colors.extend([COLORS[key]] * len(triangles))
    ax.add_collection3d(Poly3DCollection(faces, facecolors=colors, edgecolors=colors,
                                      linewidths=0, shade=True, rasterized=True))


def format_axes(ax, limits, aspect, azimuth):
    ax.set(xlim=limits[0], ylim=limits[1], zlim=limits[2], xlabel='X / front (mm)',
           ylabel='Y (mm)', zlabel='Z (mm)')
    ax.set_box_aspect(aspect)
    ax.set_proj_type('ortho')
    ax.view_init(elev=22, azim=azimuth)
    ax.tick_params(labelsize=8)


def adapter_hole_figure(p, meshes, detail):
    """Orthographic mating-face view and a section through the exported CAD."""
    fig = plt.figure(figsize=(14, 8.5), facecolor='white')
    fig.suptitle('Four LiDAR mounting holes | adapter only', fontsize=20, y=.96, weight='bold')
    fig.text(.06,.90,'Screws, base and sensor hidden. Blue rings identify H1-H4; side-wing M4 holes are a separate joint.',fontsize=11)
    ax = fig.add_axes([.065,.23,.41,.58])
    angle = np.deg2rad(p['lidar']['mount_rotation_y_deg'])
    origin = np.array([0,0,p['lidar']['mount_center_z']])
    tangent = np.array([np.cos(angle),0,-np.sin(angle)])
    normal = np.array([np.sin(angle),0,np.cos(angle)])
    item = meshes['lidar_adapter']
    vertices = np.asarray(item['vertices'])-origin
    faces = vertices[np.asarray(item['triangles'])]
    projected = np.stack([faces[:,:,1], faces@tangent],axis=-1)
    # Look from the sensor side (-normal): draw far surfaces first, leaving
    # the true CAD openings visible instead of adding artificial hole circles.
    order = np.argsort((faces@normal).mean(axis=1))[::-1]
    normals = np.cross(faces[:,1]-faces[:,0],faces[:,2]-faces[:,0])
    normals /= np.maximum(np.linalg.norm(normals,axis=1,keepdims=True),1e-12)
    light = .60+.40*np.abs(normals@normal)
    color = np.array([.84,.51,.25])[None,:]*light[:,None]
    ax.add_collection(PolyCollection(projected[order],facecolors=color[order],edgecolors='none'))
    accent = '#125b8f'
    for h in detail['holes']:
        point = np.asarray(h['mount_center_mm'])-origin
        y,u = point[1],point@tangent
        ax.add_patch(Circle((y,u),4.2,fill=False,lw=1.3,edgecolor=accent))
        label_y = y-10 if y<0 else y+10
        label_u = u+9 if u>0 else u-9
        ax.annotate(h['id'],xy=(y,u),xytext=(label_y,label_u),ha='center',va='center',
                    color=accent,fontsize=12,weight='bold',arrowprops=dict(arrowstyle='-',color=accent,lw=.8))
    centers = np.array([h['mount_center_mm'] for h in detail['holes']])-origin
    ymin,ymax = centers[:,1].min(),centers[:,1].max()
    umin,umax = (centers@tangent).min(),(centers@tangent).max()
    ax.annotate('',xy=(ymax,-40),xytext=(ymin,-40),arrowprops=dict(arrowstyle='<->',lw=.8))
    ax.text(0,-44,f'{ymax-ymin:g} mm',ha='center',fontsize=10)
    ax.annotate('',xy=(-57,umax),xytext=(-57,umin),arrowprops=dict(arrowstyle='<->',lw=.8))
    ax.text(-61,0,f'{umax-umin:g} mm',va='center',ha='center',rotation=90,fontsize=10)
    ax.set(xlim=(-65,65),ylim=(-47,43),xlabel='Y (mm)',ylabel='Along slope toward +X (mm)',
           title='LiDAR mating face — viewed square-on')
    ax.set_aspect('equal');ax.tick_params(labelsize=8)

    ax = fig.add_axes([.565,.23,.40,.58])
    for loop in detail['section_loops_xz']:
        ax.add_patch(Polygon(loop,facecolor='#ecd0b5',edgecolor='#543d2d',lw=1))
    row = [h for h in detail['holes'] if abs(h['mount_center_mm'][1]-detail['section_y_mm'])<1e-6]
    for h in row:
        entry = np.asarray(h['entry_center_mm'])
        seat = np.asarray(h['seat_center_mm'])
        point = np.asarray(h['mount_center_mm'])
        start = entry+np.asarray(h['headward_axis'])*12
        ax.plot([seat[0],point[0]],[seat[2],point[2]],ls='--',lw=1,color=accent)
        ax.annotate('',xy=(seat[0],seat[2]),xytext=(start[0],start[2]),
                    arrowprops=dict(arrowstyle='->',color=accent,lw=1.8))
        if h['entry_face']=='top':
            ax.text(start[0]+2,start[2]+3,f"{h['id']}: TOP entry",fontsize=10,color=accent,weight='bold')
        else:
            ax.text(start[0]-10,start[2]+5,f"{h['id']}: REAR entry\n(-X face)",fontsize=10,color=accent,weight='bold')
    ax.annotate('LiDAR mating face',xy=(0,p['lidar']['mount_center_z']),xytext=(-7,-40),
                fontsize=9,ha='center',arrowprops=dict(arrowstyle='-',color='#543d2d'))
    ax.set(xlim=(-48,38),ylim=(-44,20),xlabel='X / front (mm)',ylabel='Z (mm)',
           title=f"Actual CAD section at Y = +{detail['section_y_mm']:g} mm (H2 / H4)")
    ax.set_aspect('equal');ax.tick_params(labelsize=8)
    fig.text(.565,.195,'Arrows: screw / driver insertion. H1 / H3 have the same paths at Y = -24 mm.',fontsize=9)
    fig.text(.06,.13,'4 x dia3.6 through-holes at the LiDAR face; 4 x dia6.8 head / tool channels.',fontsize=12,weight='bold')
    fig.text(.06,.095,'TOP entry: H1 / H2. REAR (-X) entry: H3 / H4. These are four separate screw paths.',fontsize=11)
    fig.text(.06,.06,'Attach the LiDAR with four M3x8 screws while the adapter is detached. Headward axis: (-0.5, 0, +0.866025).',fontsize=10)
    fig.text(.06,.028,'The upper/base-contact face alone does not show all four access openings. No CAD geometry was changed for this drawing.',fontsize=10,color='#586472')
    return fig


def render_assembly(p, report, meshes, out, detail):
    lidar, joint = p['lidar'], p['assembly_joint']
    with PdfPages(out/'assembly_guide.pdf') as pdf:
        fig = plt.figure(figsize=(12, 9), facecolor='white')
        fig.suptitle('C2 / MID-360 | Two-part mounting assembly', fontsize=20, y=.96, weight='bold')
        fig.text(.07, .90, f"Z +{lidar['mount_rotation_z_deg']:g} deg, then assembly Y {lidar['mount_rotation_y_deg']:g} deg | mount center Z = {lidar['mount_center_z']:g} mm", fontsize=11)
        ax = fig.add_subplot(111, projection='3d')
        add_parts(ax, meshes, list(COLORS))
        format_axes(ax, ((-120, 65), (-60, 60), (-125, 75)), (185, 120, 200), 35)
        fig.legend(handles=[Patch(facecolor=COLORS[k], label=t) for k,t in
                           [('base','1. Common base'), ('lidar_adapter','2. 30-degree adapter'),
                            ('lidar','MID-360 reference'), ('bolts','Bought fasteners')]],
                   loc='lower center', bbox_to_anchor=(.5,.07), ncol=4, frameon=False, fontsize=10)
        fig.text(.07, .045, 'Fit the LiDAR to the detached adapter first; then bolt that subassembly to the base.', fontsize=11)
        fig.text(.07, .02, 'Prototype: verify hardware, strength, cable clearance, camera FOV and slicer supports before use.', fontsize=9, color='#985027')
        fig.subplots_adjust(top=.88, bottom=.13, left=.08, right=.95)
        fig.savefig(out/'assembly_preview.png', dpi=150)
        pdf.savefig(fig)
        plt.close(fig)

        fig = plt.figure(figsize=(13, 9), facecolor='white')
        fig.suptitle('Exploded view | Base + removable 30-degree adapter', fontsize=19, y=.96, weight='bold')
        ax = fig.add_axes([.02,.11,.69,.78], projection='3d')
        offsets = {'base':(0,0,20), 'lidar_adapter':(0,0,-40), 'lidar_screws':(0,0,-40),
                   'bolts':(0,0,55), 'washers':(0,0,35), 'nuts':(0,0,-55)}
        add_parts(ax, meshes, list(offsets), offsets)
        for x,y in joint['holes_xy']:
            ax.plot([x,x],[y,y],[-68,72],ls='--',lw=.8,color='#939ca5',alpha=.7)
        format_axes(ax, ((-120,45),(-57,57),(-85,90)),(165,114,175),140)
        notes = [
            ('M4 x 20 / 4 pcs', 'Insert from ABOVE the base.'),
            ('Washers / 4 pcs', 'OD9 x ID4.3 x 0.8 mm.'),
            ('BLUE: common base', 'Flat underside mates at Z=0.'),
            ('ORANGE: adapter', 'Attach LiDAR with 4 x M3x8\nbefore joining the base.'),
            ('M4 hex nuts / 4 pcs', 'Seat in underside hex pockets.\nAF7, height 3.2 mm nominal.'),
        ]
        for i,(title,body) in enumerate(notes):
            y=.83-i*.13
            fig.text(.73,y,title,fontsize=12,weight='bold',color='#263544')
            fig.text(.73,y-.035,body,fontsize=10,va='top',linespacing=1.5)
        fig.text(.07,.055,'Exploded spacing is for illustration. Sensors, pipe and collar are hidden; these are two printed parts, not one STL.',fontsize=10)
        fig.text(.07,.025,'M3 screws are shown installed. M4 threads and drive sockets are simplified reference envelopes.',fontsize=9,color='#586472')
        fig.savefig(out/'exploded_view.png',dpi=150)
        pdf.savefig(fig)
        plt.close(fig)

        fig = adapter_hole_figure(p, meshes, detail)
        fig.savefig(out/'adapter_holes.png',dpi=170)
        pdf.savefig(fig)
        plt.close(fig)

        fig, ax = plt.subplots(figsize=(11.7,8.3))
        ax.axis('off')
        base = report['printed_parts']['base']['dimensions_mm']
        adapter = report['printed_parts']['lidar_adapter']['dimensions_mm']
        jr = report['assembly_joint']
        lines = [
            'TWO-PRINT ASSEMBLY / DIMENSIONS AND FASTENERS (mm)', '',
            f'BASE: {base[0]:.1f} x {base[1]:.1f} x {base[2]:.1f}; 8 mm plate; print underside on bed.',
            f'ADAPTER: {adapter[0]:.1f} x {adapter[1]:.1f} x {adapter[2]:.3f}; 30-degree LiDAR mating face.',
            'Adapter STL is already flipped 180 deg about X, flat contact face down, minimum Z=0.',
            'STEP / FCStd keep both parts at their assembly coordinates. Do not align parts using print STLs.', '',
            'BASE / ADAPTER JOINT:',
            f"4 x dia {joint['clearance_diameter']:g} vertical bores at XY = {joint['holes_xy']}.",
            'Contact plane Z=0. M4x20 cap screw + top washer + captured hex nut (no printed threads).',
            f"Flange {joint['flange_thickness']:g} thick; nut pocket AF{joint['nut_pocket_across_flats']:g}, depth {joint['nut_pocket_depth']:g} from underside.",
            f"Nut AF{joint['nut_across_flats']:g} x {joint['nut_height']:g}; screw tip projects {jr['screw_protrusion_below_nut_mm']:.1f} below nut.",
            f"Validated planar contact area: {jr['contact_area_mm2']:.1f} mm2; complete washer and nut seats.", '',
            'LIDAR / ADAPTER JOINT:',
            '4 x M3x8 into sensor, angled bores dia3.6, head/driver channels dia6.8.',
            'Seat thickness 4.5 along screw axis; insertion 3.5. Confirm actual thread/depth and head size.',
            'Hole centers: (+15.588457,+/-24,-11), (-15.588457,+/-24,-29).',
            'Headward screw axis (-0.5,0,+0.866025). Fit these screws while the adapter is DETACHED.', '',
            'ASSEMBLY ORDER:',
            '1. Print one base and one adapter. Deburr holes; remove supports; dry-fit nuts in pockets.',
            '2. Fit LiDAR to adapter with the four angled M3x8 screws. Verify manufacturer insertion limit.',
            '3. Seat M4 nuts in underside pockets; hold them in place while starting the bolts.',
            '4. Bring the LiDAR+adapter upward to the base and tighten four top M4x20 + washers evenly.',
            '5. Fit the camera with four M1.6x8 rear screws, then collar/pipe/cables as in README.md.',
            'Remove adapter+LiDAR as one unit for service; base blocks the M3 access when assembled.', '',
            'VALIDATION:',
            'Two separate solids; per-part closed STL and STEP roundtrips; FCStd reopen/edit/restore.',
            'Actual LiDAR STEP hole alignment and support; M3 and M4 bearing/tool paths; nut insertion.',
            'No print/print or print/sensor interference; adapter+LiDAR vertical assembly approach.',
            f"LiDAR support: {report['lidar_supported_area_mm2']/report['lidar_mount_face_area_mm2']*100:.2f}%; camera support: {report['camera_support_fraction']*100:.1f}%.", '',
            'NOT VALIDATED: strength, PLA creep/vibration, actual hardware fit/torque, slicer/G-code.',
            'Tilted LiDAR protrudes ahead of the fixture: verify camera images for obstruction.',
            'Pipe OD25 and SCS2512CN2 interface are provisional envelopes, not certified dimensions.',
        ]
        ax.text(.015,.985,'\n'.join(lines),va='top',family='monospace',fontsize=9,linespacing=1.23)
        fig.subplots_adjust(left=.03,right=.98,top=.97,bottom=.02)
        pdf.savefig(fig)
        plt.close(fig)
