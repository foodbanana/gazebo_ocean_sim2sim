#!/usr/bin/env python3
"""config/iris_drone.yaml -> models/iris_075/model.sdf + meshes/collision/hull_*.stl.
Mesh URIs are model://iris_075/... (the SDF is also spawned as a string, so relative paths would not resolve).

The Isaac Sim drone (Iris scaled 0.75), rebuilt from what Isaac Sim exports (scripts/export_isaac_drone_collision.py):
  link body       Isaac Sim body link mass properties, visual = Isaac body mesh + red grasp tape meshes, collision =
                  the PhysX body hulls (or CoACD, config collision.source), contact sensor /drone/contacts
  links rotor0..3 at the Isaac Sim rotor link poses, Isaac rotor mass properties, prop mesh visual, PhysX prop hull
                  collision, contact sensor /drone/contacts
  rotorN_joint    revolute body -> rotorN about the rotor z (thrust) axis, no limits / damping; speed set by the
                  DroneLockstep plugin (prop spin, Isaac rule)
Materials: one plain color per Isaac Sim material (config mdl_materials, materials.yaml), no textures.
Model plugins: DetachableJoint (body held on the world-fixed model "drone_anchor": attach /drone/attach,
detach /drone/detach, state /drone/state), DroneLockstep (PX4 bridge), OdometryPublisher (/drone/odometry).

Needs (only to rebuild, not to run the simulation):  pip install trimesh scipy pyyaml (+ coacd for source coacd)
Run:  python3 ocean_sim2sim_gazebo/scripts/build_iris_drone.py
"""
import glob
import math
import os

import numpy as np
import trimesh
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
MODEL = os.path.join(PKG, 'models', 'iris_075')
URI = 'model://iris_075'


def quat_to_R(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def rpy(R):
    """rotation matrix -> SDF roll pitch yaw (R = Rz(yaw) Ry(pitch) Rx(roll))."""
    pitch = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    return math.atan2(R[2, 1], R[2, 2]), pitch, math.atan2(R[1, 0], R[0, 0])


def body_hulls(c):
    k = c['collision']
    if k['source'] == 'isaac':
        # the hulls Isaac Sim's PhysX simulates (scripts/export_isaac_drone_collision.py), body link frame
        src = sorted(glob.glob(os.path.join(MODEL, k['isaac_dir'], 'hull_*.stl')))
        if not src:
            raise SystemExit(f"no hulls in {k['isaac_dir']}: run export_isaac_drone_collision.py with Isaac Sim first")
        return [trimesh.load(f).convex_hull for f in src]
    if k['source'] == 'coacd':
        import coacd
        body = trimesh.load(os.path.join(MODEL, k['body_mesh']))
        T = np.eye(4)
        cy, sy = math.cos(k['yaw']), math.sin(k['yaw'])
        T[:3, :3] = k['scale'] * np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
        body.apply_transform(T)
        coacd.set_log_level('error')
        parts = coacd.run_coacd(coacd.Mesh(body.vertices, body.faces), threshold=k['threshold'],
                                max_convex_hull=k['max_convex_hull'], max_ch_vertex=k['max_ch_vertex'],
                                decimate=True, preprocess_mode='auto', seed=k['seed'])
        return [trimesh.Trimesh(v, f).convex_hull for v, f in parts]
    raise SystemExit(f"collision.source must be isaac or coacd: {k['source']}")


def material_of(c, mesh):
    """materials.yaml entry -> (diffuse rgb, roughness, metalness, where from)."""
    m = mesh['material']
    if m.get('bound') is None:
        e = m['isaac_renders']['equivalent_preview_surface']
        return e['diffuseColor'], e['roughness'], e['metallic'], 'unbound: Isaac Sim RTX default material'
    for sh in m['shaders']:
        if sh.get('id') == 'UsdPreviewSurface':
            i = sh['inputs']
            return (i.get('diffuseColor', [0.18] * 3), i.get('roughness', 0.5), i.get('metallic', 0.0),
                    f"{m['bound']} (UsdPreviewSurface)")
        if 'mdl_material' in sh:
            name = sh['mdl_material']
            if name not in c['mdl_materials']:
                raise SystemExit(f"MDL material {name} ({mesh['prim']}) missing in config mdl_materials")
            e = c['mdl_materials'][name]
            return e['diffuse'], e['roughness'], e['metalness'], f"{m['bound']} (MDL {name}, plain color)"
    raise SystemExit(f"{mesh['prim']}: unknown material {m}")


def visual_sdf(name, uri, rgb, rough, metal, spec, indent='      '):
    col = ' '.join(f'{v:.6g}' for v in rgb)
    sp = ' '.join(f'{v:g}' for v in spec)
    return f'''{indent}<visual name="{name}">
{indent}  <geometry><mesh><uri>{uri}</uri></mesh></geometry>
{indent}  <material>
{indent}    <ambient>{col} 1</ambient><diffuse>{col} 1</diffuse><specular>{sp} 1</specular>
{indent}    <pbr><metal><roughness>{rough:g}</roughness><metalness>{metal:g}</metalness></metal></pbr>
{indent}  </material>
{indent}</visual>'''


def collision_sdf(name, uri, mu, indent='      '):
    return f'''{indent}<collision name="{name}">
{indent}  <geometry><mesh><uri>{uri}</uri></mesh></geometry>
{indent}  <surface><friction><ode><mu>{mu}</mu><mu2>{mu}</mu2></ode></friction></surface>
{indent}</collision>'''


def inertial_sdf(L, indent='      '):
    c, I = L['com'], np.array(L['inertia_about_com'])
    return f'''{indent}<inertial>
{indent}  <pose>{c[0]:.8f} {c[1]:.8f} {c[2]:.8f} 0 0 0</pose>
{indent}  <mass>{L['mass']:.6f}</mass>
{indent}  <inertia>
{indent}    <ixx>{I[0, 0]:.8e}</ixx><ixy>{I[0, 1]:.8e}</ixy><ixz>{I[0, 2]:.8e}</ixz>
{indent}    <iyy>{I[1, 1]:.8e}</iyy><iyz>{I[1, 2]:.8e}</iyz><izz>{I[2, 2]:.8e}</izz>
{indent}  </inertia>
{indent}</inertial>'''


def contact_sdf(names, indent='      '):
    coll = '\n'.join(f'{indent}    <collision>{n}</collision>' for n in names)
    return f'''{indent}<sensor name="contacts" type="contact">
{indent}  <always_on>true</always_on><update_rate>240</update_rate><topic>/drone/contacts</topic>
{indent}  <contact>
{coll}
{indent}    <topic>/drone/contacts</topic>
{indent}  </contact>
{indent}</sensor>'''


def combined(links, poses):
    """Total mass, COM and inertia about it in the body frame from the per-link values (parallel axis)."""
    M = sum(L['mass'] for L in links.values())
    cw = {n: poses[n][0] @ np.array(L['com']) + poses[n][1] for n, L in links.items()}
    com = sum(L['mass'] * cw[n] for n, L in links.items()) / M
    I = np.zeros((3, 3))
    for n, L in links.items():
        R = poses[n][0]
        r = cw[n] - com
        I += R @ np.array(L['inertia_about_com']) @ R.T + L['mass'] * (r @ r * np.eye(3) - np.outer(r, r))
    return M, com, I


def build(c):
    with open(os.path.join(MODEL, c['isaac_flight'])) as f:
        fl = yaml.safe_load(f)
    with open(os.path.join(MODEL, c['isaac_visual'], 'materials.yaml')) as f:
        vis = yaml.safe_load(f)['meshes']
    links = fl.get('links')
    if not links or 'body' not in links:
        raise SystemExit(f"{c['isaac_flight']} has no links: (re)run export_isaac_drone_collision.py with Isaac Sim")
    rotors = [r['name'] for r in fl['rotors']]
    if set(links) != {'body', *rotors} or len(c['rotor_joints']) != len(rotors):
        raise SystemExit(f'links {sorted(links)} / rotors {rotors} / rotor_joints {c["rotor_joints"]} do not match')
    mu = float(c['collision']['friction'])
    spec = c['specular']

    # ---- body collision hulls -> meshes/collision
    hulls = body_hulls(c)
    for old in glob.glob(os.path.join(MODEL, 'meshes', 'collision', 'hull_*.stl')):
        os.remove(old)
    for i, h in enumerate(hulls):
        h.export(os.path.join(MODEL, 'meshes', 'collision', f'hull_{i:02d}.stl'))

    # ---- visuals per link (Isaac Sim OBJ in the link frame)
    vis_by_link, mat_lines = {}, []
    for obj, m in vis.items():
        rgb, rough, metal, src = material_of(c, m)
        stem = obj[len(m['link']) + 1:-4]                       # <link>_<mesh>.obj -> <mesh>
        vis_by_link.setdefault(m['link'], []).append(
            visual_sdf(stem, f"{URI}/{c['isaac_visual']}/{obj}", rgb, rough, metal, spec))
        mat_lines.append(f'  {obj}: diffuse {[round(float(v), 6) for v in rgb]}, roughness {rough}, metalness {metal} <- {src}')

    # ---- body link
    names = [f'hull_{i:02d}' for i in range(len(hulls))]
    body = f'''    <link name="body">
{inertial_sdf(links['body'])}
{chr(10).join(vis_by_link.get('body', []))}
{chr(10).join(collision_sdf(n, f"{URI}/meshes/collision/{n}.stl", mu) for n in names)}
      <!-- contacts of all body hulls (grasp judge: finger forces, docs/grasp_demo.md) -->
{contact_sdf(names)}
    </link>'''

    # ---- rotor links + joints
    poses = {'body': (np.eye(3), np.zeros(3))}
    parts = []
    for rn, jn in zip(rotors, c['rotor_joints']):
        L = links[rn]
        p = np.array(L['pose_in_body']['position'])
        R = quat_to_R(L['pose_in_body']['orientation_wxyz'])
        poses[rn] = (R, p)
        r, pi, y = rpy(R)
        src = sorted(glob.glob(os.path.join(MODEL, c['collision']['isaac_dir'], f'{rn}_hull_*.stl')))
        if not src:
            raise SystemExit(f"no {rn}_hull_*.stl in {c['collision']['isaac_dir']}: run export_isaac_drone_collision.py")
        cn = [os.path.basename(s)[:-4] for s in src]
        parts.append(f'''    <link name="{rn}">
      <pose relative_to="body">{p[0]:.7f} {p[1]:.7f} {p[2]:.7f} {r:.7f} {pi:.7f} {y:.7f}</pose>
{inertial_sdf(L)}
{chr(10).join(vis_by_link.get(rn, []))}
{chr(10).join(collision_sdf(n, f"{URI}/{c['collision']['isaac_dir']}/{n}.stl", mu) for n in cn)}
{contact_sdf(cn)}
    </link>
    <joint name="{jn}" type="revolute">
      <parent>body</parent>
      <child>{rn}</child>
      <axis>
        <xyz>0 0 1</xyz>
        <limit><lower>-1e16</lower><upper>1e16</upper><effort>-1</effort><velocity>-1</velocity></limit>
        <dynamics><damping>0</damping><friction>0</friction><spring_stiffness>0</spring_stiffness></dynamics>
      </axis>
    </joint>''')

    sdf = f'''<?xml version="1.0"?>
<!-- Generated by ocean_sim2sim_gazebo/scripts/build_iris_drone.py from config/iris_drone.yaml. Do not edit. -->
<sdf version="1.9">
  <model name="iris_075">
{body}
{chr(10).join(parts)}

    <!-- world-fixed hover: hold the body on the model "drone_anchor" (welded to the world) until /drone/detach -->
    <plugin filename="gz-sim-detachable-joint-system" name="gz::sim::systems::DetachableJoint">
      <parent_link>body</parent_link>
      <child_model>drone_anchor</child_model>
      <child_link>link</child_link>
      <attach_topic>/drone/attach</attach_topic>
      <detach_topic>/drone/detach</detach_topic>
      <output_topic>/drone/state</output_topic>
    </plugin>
    <!-- PX4 drone: lockstep with the bridge node (ocean_sim2sim_drone px4_bridge_node), rotor wrench on the body at
         120 Hz + prop joint speeds (Isaac Sim prop spin rule). Idle (no force, props held) while no bridge is
         connected: the drone just hangs on its anchor (docs/drone_px4.md) -->
    <plugin filename="OceanSim2simDroneLockstep" name="ocean_sim2sim::DroneLockstep">
      <link_name>body</link_name>
      <rotor_joints>{' '.join(c['rotor_joints'])}</rotor_joints>
      <port>4570</port>
      <rate>120</rate>
      <timeout>10</timeout>
    </plugin>
    <!-- drone body pose (world) for the grasp demo / judge -->
    <plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">
      <odom_frame>world</odom_frame>
      <robot_base_frame>drone</robot_base_frame>
      <odom_publish_frequency>60</odom_publish_frequency>
      <odom_topic>/drone/odometry</odom_topic>
      <dimensions>3</dimensions>
    </plugin>
  </model>
</sdf>
'''
    with open(os.path.join(MODEL, 'model.sdf'), 'w') as f:
        f.write(sdf)

    # ---- report: mass check against the Isaac Sim whole-drone values
    M, com, I = combined(links, poses)
    Mi, ci, Ii = float(fl['mass']), np.array(fl['com']), np.array(fl['inertia_about_com'])
    waist = np.concatenate([s.vertices[:, 1] for s in (h.section(plane_origin=[0, 0, 0], plane_normal=[1, 0, 0]) for h in hulls)
                            if s is not None])
    print(f'[OK] {os.path.relpath(MODEL, os.getcwd())}/model.sdf')
    print(f'  body: {len(hulls)} hulls ({c["collision"]["source"]}), max {max(len(h.vertices) for h in hulls)} vertices, '
          f'waist (x = 0) {(waist.max() - waist.min()) * 1000:.1f} mm')
    print('  materials:\n' + '\n'.join(mat_lines))
    print(f'  links: ' + ', '.join(f'{n} {L["mass"]:g} kg' for n, L in links.items()))
    print(f'  combined from the links: mass {M:.6f} kg, COM {np.round(com * 1000, 4).tolist()} mm')
    print(f'  Isaac Sim whole drone:   mass {Mi:.6f} kg, COM {np.round(ci * 1000, 4).tolist()} mm')
    print(f'  inertia about COM (links) [kg m2]:\n{np.array2string(I, precision=8)}')
    print(f'  inertia about COM (Isaac Sim):\n{np.array2string(Ii, precision=8)}')
    dI = np.abs(I - Ii).max() / np.abs(Ii).max()
    print(f'  max |dI| / max |I| = {dI:.2e}, |dCOM| = {np.linalg.norm(com - ci) * 1e6:.2f} um, dM = {M - Mi:+.2e} kg')
    if abs(M - Mi) > 1e-5 or np.linalg.norm(com - ci) > 1e-5 or dI > 1e-3:
        raise SystemExit('link mass properties do not add up to the Isaac Sim whole-drone values')


def main():
    with open(os.path.join(PKG, 'config', 'iris_drone.yaml')) as f:
        build(yaml.safe_load(f))


if __name__ == '__main__':
    main()
