#!/usr/bin/env python3
"""config/iris_drone.yaml -> models/iris_075/model.sdf + meshes/collision/hull_*.stl (one rigid body).
Mesh URIs are model://iris_075/... (the SDF is also spawned as a string, so relative paths would not resolve).

Iris scaled 0.75 like the Isaac Sim drone: body mesh visual, body collision = CoACD convex hulls of the
scaled + rotated mesh, 4 fixed rotor visuals, mass 0.78 (body, uniform density over the hulls) + 4 x 0.005
(rotor discs). The model also has a DetachableJoint that holds it on a model "drone_anchor" (world-fixed hover):
attach /drone/attach, detach /drone/detach, state /drone/state (gz topics).

Body collision (config collision.source):
  isaac: the convex hulls Isaac Sim's PhysX cooks for its drone (exported once with Isaac Sim by
         scripts/export_isaac_drone_collision.py into meshes/collision_isaac/) = the same collision input as Isaac Sim
  coacd: CoACD decomposition of the mesh made here (no Isaac Sim needed)
Needs (only to rebuild, not to run the simulation):  pip install trimesh scipy (+ coacd for source coacd)
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


def transform(scale, yaw):
    T = np.eye(4)
    c, s = math.cos(yaw), math.sin(yaw)
    T[:3, :3] = scale * np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    return T


def build(c):
    scale, yaw = float(c['scale']), float(c['yaw'])
    T = transform(scale, yaw)

    # ---- body collision
    body = trimesh.load(os.path.join(MODEL, c['body']['mesh']))
    body.apply_transform(T)
    k = c['collision']
    if k['source'] == 'isaac':
        # the hulls Isaac Sim's PhysX simulates (scripts/export_isaac_drone_collision.py), drone body frame
        src = sorted(glob.glob(os.path.join(MODEL, k['isaac_dir'], 'hull_*.stl')))
        if not src:
            raise SystemExit(f"no hulls in {k['isaac_dir']}: run export_isaac_drone_collision.py with Isaac Sim first")
        hulls = [trimesh.load(f).convex_hull for f in src]
    elif k['source'] == 'coacd':
        import coacd
        coacd.set_log_level('error')
        parts = coacd.run_coacd(coacd.Mesh(body.vertices, body.faces), threshold=k['threshold'],
                                max_convex_hull=k['max_convex_hull'], max_ch_vertex=k['max_ch_vertex'],
                                decimate=True, preprocess_mode='auto', seed=k['seed'])
        hulls = [trimesh.Trimesh(v, f).convex_hull for v, f in parts]
    else:
        raise SystemExit(f"collision.source must be isaac or coacd: {k['source']}")
    for old in glob.glob(os.path.join(MODEL, 'meshes', 'collision', 'hull_*.stl')):
        os.remove(old)
    for i, h in enumerate(hulls):
        h.export(os.path.join(MODEL, 'meshes', 'collision', f'hull_{i:02d}.stl'))

    # ---- mass properties: body mass spread uniformly over the hulls + rotor discs
    vol = sum(h.volume for h in hulls)
    rho = c['body']['mass'] / vol
    items = []                                     # (mass, com, inertia about own com in drone frame)
    for h in hulls:
        h.density = rho
        items.append((h.mass, h.center_mass, h.moment_inertia))
    r = c['rotors']
    rotors = []
    for rot in r['list']:
        p = (T @ np.array([*rot['pos'], 1.0]))[:3]
        rotors.append((p, rot['mesh']))
        rr, th, m = r['radius'] * scale, r['thickness'] * scale, r['mass']
        I = np.diag([m * (3 * rr ** 2 + th ** 2) / 12] * 2 + [m * rr ** 2 / 2])
        items.append((m, p, I))
    M = sum(m for m, _, _ in items)
    com = sum(m * p for m, p, _ in items) / M
    I = np.zeros((3, 3))
    for m, p, Ic in items:
        d = p - com
        I += Ic + m * (np.dot(d, d) * np.eye(3) - np.outer(d, d))

    # ---- flight values from Isaac Sim (mass_properties: isaac): the PhysX mass, COM, inertia and rotor positions of the
    #      flying drone (scripts/export_isaac_drone_collision.py -> isaac_flight.yaml), so it flies like the Isaac Sim one
    if c.get('mass_properties', 'computed') == 'isaac':
        with open(os.path.join(MODEL, c['isaac_flight'])) as f:
            fl = yaml.safe_load(f)
        M, com, I = float(fl['mass']), np.array(fl['com']), np.array(fl['inertia_about_com'])
        mesh_by_dir = {d: m for d, m in c['rotors']['mesh_by_direction'].items()}
        rotors = [(np.array(r['position']), mesh_by_dir[int(r['direction'])]) for r in fl['rotors']]

    # ---- SDF
    rgb = ' '.join(f'{v:g}' for v in c['body']['color'])
    yaw_s = f'{yaw:.6f}'
    mu = float(k['friction'])
    coll = '\n'.join(f'''      <collision name="hull_{i:02d}">
        <geometry><mesh><uri>model://iris_075/meshes/collision/hull_{i:02d}.stl</uri></mesh></geometry>
        <surface><friction><ode><mu>{mu}</mu><mu2>{mu}</mu2></ode></friction></surface>
      </collision>''' for i in range(len(hulls)))
    coll_names = '\n'.join(f'          <collision>hull_{i:02d}</collision>' for i in range(len(hulls)))
    rot_vis = '\n'.join(f'''      <visual name="rotor_{i}">
        <pose>{p[0]:.6f} {p[1]:.6f} {p[2]:.6f} 0 0 {yaw_s}</pose>
        <geometry><mesh><uri>model://iris_075/{mesh}</uri><scale>{scale} {scale} {scale}</scale></mesh></geometry>
      </visual>''' for i, (p, mesh) in enumerate(rotors))
    sdf = f'''<?xml version="1.0"?>
<!-- Generated by ocean_sim2sim_gazebo/scripts/build_iris_drone.py from config/iris_drone.yaml. Do not edit. -->
<sdf version="1.9">
  <model name="iris_075">
    <link name="body">
      <inertial>
        <pose>{com[0]:.6f} {com[1]:.6f} {com[2]:.6f} 0 0 0</pose>
        <mass>{M:.6f}</mass>
        <inertia>
          <ixx>{I[0, 0]:.6e}</ixx><ixy>{I[0, 1]:.6e}</ixy><ixz>{I[0, 2]:.6e}</ixz>
          <iyy>{I[1, 1]:.6e}</iyy><iyz>{I[1, 2]:.6e}</iyz><izz>{I[2, 2]:.6e}</izz>
        </inertia>
      </inertial>
      <visual name="body">
        <pose>0 0 0 0 0 {yaw_s}</pose>
        <geometry><mesh><uri>model://iris_075/{c['body']['mesh']}</uri><scale>{scale} {scale} {scale}</scale></mesh></geometry>
        <material><ambient>{rgb} 1</ambient><diffuse>{rgb} 1</diffuse></material>
      </visual>
{rot_vis}
{coll}
      <!-- contacts of all hulls (grasp judge: finger forces, docs/grasp_demo.md) -->
      <sensor name="contacts" type="contact">
        <always_on>true</always_on><update_rate>240</update_rate><topic>/drone/contacts</topic>
        <contact>
{coll_names}
          <topic>/drone/contacts</topic>
        </contact>
      </sensor>
    </link>

    <!-- world-fixed hover: hold the body on the model "drone_anchor" (welded to the world) until /drone/detach -->
    <plugin filename="gz-sim-detachable-joint-system" name="gz::sim::systems::DetachableJoint">
      <parent_link>body</parent_link>
      <child_model>drone_anchor</child_model>
      <child_link>link</child_link>
      <attach_topic>/drone/attach</attach_topic>
      <detach_topic>/drone/detach</detach_topic>
      <output_topic>/drone/state</output_topic>
    </plugin>
    <!-- PX4 drone: lockstep with the bridge node (ocean_sim2sim_drone px4_bridge_node), rotor wrench at 120 Hz.
         Idle (no force) while no bridge is connected: the drone just hangs on its anchor (docs/drone_px4.md) -->
    <plugin filename="OceanSim2simDroneLockstep" name="ocean_sim2sim::DroneLockstep">
      <link_name>body</link_name>
      <port>4570</port>
      <rate>120</rate>
      <timeout>10</timeout>
    </plugin>
    <!-- drone body pose (world) for the grasp demo / judge -->
    <plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">
      <odom_frame>world</odom_frame>
      <robot_base_frame>drone</robot_base_frame>
      <odom_topic>/drone/odometry</odom_topic>
      <odom_publish_frequency>60</odom_publish_frequency>
      <dimensions>3</dimensions>
    </plugin>
  </model>
</sdf>
'''
    with open(os.path.join(MODEL, 'model.sdf'), 'w') as f:
        f.write(sdf)

    # ---- report
    waist = np.concatenate([s.vertices[:, 1] for s in (h.section(plane_origin=[0, 0, 0], plane_normal=[1, 0, 0]) for h in hulls)
                            if s is not None])
    mw = body.section(plane_origin=[0, 0, 0], plane_normal=[1, 0, 0]).vertices[:, 1]
    b = body.bounds * 1000
    print(f'[OK] {MODEL}/model.sdf')
    print(f'  outline x {b[0, 0]:.0f}~{b[1, 0]:.0f}, y {b[0, 1]:.0f}~{b[1, 1]:.0f}, z {b[0, 2]:.0f}~{b[1, 2]:.0f} mm')
    print(f'  waist (x = 0): mesh {(mw.max() - mw.min()) * 1000:.1f} mm, collision hulls {(waist.max() - waist.min()) * 1000:.1f} mm')
    print(f'  {len(hulls)} hulls, max {max(len(h.vertices) for h in hulls)} vertices; mass {M:.3f} kg, '
          f'COM ({com[0] * 1000:.1f}, {com[1] * 1000:.1f}, {com[2] * 1000:.1f}) mm')


def main():
    with open(os.path.join(PKG, 'config', 'iris_drone.yaml')) as f:
        build(yaml.safe_load(f))


if __name__ == '__main__':
    main()
