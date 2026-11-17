"""Assembly and exploded drawings from the validated native CAD meshes."""
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.patches import Patch
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


def render_assembly(p, report, meshes, out):
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
