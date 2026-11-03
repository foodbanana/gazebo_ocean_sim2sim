#!/usr/bin/env python3
"""Export the drone collision shapes exactly as Isaac Sim's PhysX cooks them (convex decomposition hulls) -> STL files
in the drone body frame, for the Gazebo drone (config/iris_drone.yaml collision.source: isaac). docs/scene.md

Runs with Isaac Sim's Python, using the Isaac Sim project's own drone loader (same USD, same scene-layer overrides):
    ~/isaacsim/python.sh ocean_sim2sim_gazebo/scripts/export_isaac_drone_collision.py \
        --isaac-project ~/data_collection_ur5_gripper [--out ocean_sim2sim_gazebo/models/iris_075/meshes/collision_isaac]

PhysX API: omni.physx get_physx_cooking_interface().request_convex_collision_representation (the hulls PhysX simulates).
Also writes report.txt (hull count, vertices, waist width) and raw.stl (the source collision mesh, same frame) to compare.
"""
import argparse
import os
import sys

p = argparse.ArgumentParser()
p.add_argument('--isaac-project', required=True, help='data_collection_ur5_gripper directory (has isaacsim/scripts)')
p.add_argument('--out', default=os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                             'models', 'iris_075', 'meshes', 'collision_isaac'))
p.add_argument('--drone-config', default=None, help='Isaac Sim drone yaml (default: its drone_iris.yaml)')
args, _ = p.parse_known_args()
scripts = os.path.join(os.path.expanduser(args.isaac_project), 'isaacsim', 'scripts')
sys.path.insert(0, scripts)

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({'headless': True})

import numpy as np  # noqa: E402
import trimesh  # noqa: E402
import isaacsim.core.experimental.utils.app as app_utils  # noqa: E402
import isaacsim.core.experimental.utils.stage as stage_utils  # noqa: E402
import yaml  # noqa: E402
from isaacsim.core.experimental.prims import RigidPrim  # noqa: E402
from isaacsim.core.simulation_manager import SimulationManager  # noqa: E402
from omni.physx import get_physx_cooking_interface  # noqa: E402
from pxr import PhysicsSchemaTools, Usd, UsdGeom, UsdPhysics, UsdUtils  # noqa: E402

import collision_geom as cg  # noqa: E402
import drone as dr  # noqa: E402
import test_scene as ts  # noqa: E402


def main():
    cfg = dr.load_drone_config(args.drone_config or dr.DEFAULT_DRONE_CONFIG)
    config = ts.load_config(ts.DEFAULT_CONFIG)
    stage_utils.create_new_stage()
    stage_utils.set_stage_up_axis('Z')
    stage_utils.set_stage_units(meters_per_unit=1.0, kilograms_per_unit=1.0)
    stage = stage_utils.get_current_stage(backend='usd')
    stage_utils.define_prim('/World', 'Xform')
    if not SimulationManager.switch_physics_engine('physx'):
        raise RuntimeError('could not switch to PhysX')
    SimulationManager.setup_simulation(dt=ts.PHYSICS_DT)
    ts.define_physics_material(stage, ts.OBJECT_MATERIAL_PATH, config['contact']['object_material'])
    info = dr.add_drone(stage, cfg, position=(0.0, 0.0, 1.0))
    body = info['body_path']
    Rb, tb = dr.drone_frame(stage, body)           # body link frame (scale removed) = the frame the grasp plan uses
    for _ in range(5):
        app.update()

    stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
    cooking = get_physx_cooking_interface()
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    os.makedirs(args.out, exist_ok=True)
    for f in os.listdir(args.out):
        if f.endswith('.stl'):
            os.remove(os.path.join(args.out, f))
    hulls, raws, lines = [], [], []
    for prim in dr.collision_prims(stage, info['path']):
        if dr._owner(prim) != body:
            continue
        approx = UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get() if prim.HasAPI(UsdPhysics.MeshCollisionAPI) else None
        got = {}

        def on_result(result, convexes, got=got):
            got['result'], got['convexes'] = result, convexes

        cooking.request_convex_collision_representation(stage_id=stage_id,
                                                        collision_prim_id=PhysicsSchemaTools.sdfPathToInt(prim.GetPath()),
                                                        run_asynchronously=False, on_result=on_result)
        for _ in range(200):
            if got:
                break
            app.update()
        if not got or str(got['result']).split('.')[-1] != 'RESULT_VALID':
            raise RuntimeError(f'{prim.GetPath()}: no convex representation ({got.get("result")})')
        M = cg.mat_np(cache.GetLocalToWorldTransform(prim))        # prim local -> world (row vectors, includes scale)
        pts, faces, is_mesh = cg._shape(prim)
        raw_w = (np.c_[pts, np.ones(len(pts))] @ M)[:, :3]
        raws.append(trimesh.Trimesh((raw_w - tb) @ Rb, faces, process=False) if is_mesh else trimesh.convex.convex_hull((raw_w - tb) @ Rb))
        for c in got['convexes']:
            v = np.array([[q.x, q.y, q.z] for q in c.vertices])
            w = (np.c_[v, np.ones(len(v))] @ M)[:, :3]                # convex vertices are in the prim's local space
            hulls.append(trimesh.convex.convex_hull((w - tb) @ Rb))
        lines.append(f'{prim.GetPath()}: approximation {approx}, {len(got["convexes"])} convex(es), '
                     f'{sum(len(c.vertices) for c in got["convexes"])} vertices')
    if not hulls:
        raise RuntimeError('no body collision hulls')
    for i, h in enumerate(hulls):
        h.export(os.path.join(args.out, f'hull_{i:02d}.stl'))
    raw = trimesh.util.concatenate(raws)
    raw.export(os.path.join(args.out, 'raw_reference.stl'))
    H = trimesh.util.concatenate(hulls)

    def waist(mesh, z, half=0.013):
        s = trimesh.sample.sample_surface(mesh, 300000)[0]
        s = s[(np.abs(s[:, 0]) < half) & (np.abs(s[:, 2] - z) < 0.001)]
        return (s[:, 1].max() - s[:, 1].min()) * 1000 if len(s) else float('nan')

    lines.append(f'{len(hulls)} hulls, max {max(len(h.vertices) for h in hulls)} vertices; bounds hulls '
                 f'{np.round(H.bounds * 1000, 1).tolist()} mm, raw {np.round(raw.bounds * 1000, 1).tolist()} mm')
    for z in (0.0, -0.01, -0.02, -0.03):
        lines.append(f'  width in the finger slab (|x| < 13 mm) at z {z * 1000:+.0f} mm: hulls {waist(H, z):.1f}, raw {waist(raw, z):.1f} mm')
    lines += mass_properties(cfg, info, Rb, tb)
    with open(os.path.join(args.out, 'report.txt'), 'w') as f:
        f.write('Exported by ocean_sim2sim_gazebo/scripts/export_isaac_drone_collision.py (Isaac Sim PhysX cooked hulls)\n'
                f'drone config {os.path.relpath(cfg["path"], os.path.expanduser(args.isaac_project))}, USD '
                f'{os.path.relpath(cfg["usd_abs"], os.path.expanduser(args.isaac_project))}, scale {cfg["scale"]}, body frame {body}\n'
                + '\n'.join(lines) + '\n')
    print('\n'.join(lines))
    print(f'[OK] {args.out}')


def mass_properties(cfg, info, Rb, tb):
    """What PhysX simulates for the flying drone (drone_flight._mass_properties): total mass, COM and inertia about it in
    the body frame (links + parallel axis), rotor positions / thrust axes in the body frame, rotor directions and the
    thrust model -> isaac_flight.yaml (Gazebo drone: same values)."""
    app_utils.play()
    app.update()
    links = RigidPrim(info['bodies'])
    m = links.get_masses().numpy().reshape(-1)
    pos, quat = (x.numpy() for x in links.get_world_poses())
    com_l = links.get_coms()[0].numpy().reshape(-1, 3)
    Il = links.get_inertias().numpy().reshape(-1, 3, 3)
    k = info['bodies'].index(info['body_path'])
    Rbody = cg.quat_to_R(quat[k])
    Rs = [cg.quat_to_R(q) for q in quat]
    com_w = np.array([p + R @ c for p, R, c in zip(pos, Rs, com_l)])
    c_tot = (m[:, None] * com_w).sum(axis=0) / m.sum()
    I = np.zeros((3, 3))
    for mi, R, Ii, cw in zip(m, Rs, Il, com_w):
        Rr = Rbody.T @ R
        r = Rbody.T @ (cw - c_tot)
        I += Rr @ Ii @ Rr.T + mi * (r @ r * np.eye(3) - np.outer(r, r))
    fl = cfg['flight']
    rot = RigidPrim([dr._sub(info['path'], r) for r in fl['rotors']])
    rp, rq = (x.numpy() for x in rot.get_world_poses())
    rotors = [{'name': n, 'position': np.round(Rbody.T @ (p - pos[k]), 6).tolist(),
               'thrust_axis': np.round(Rbody.T @ cg.quat_to_R(q)[:, 2], 6).tolist(), 'direction': int(d)}
              for n, p, q, d in zip(fl['rotors'], rp, rq, fl['rot_dir'])]
    out = {'source': 'Isaac Sim PhysX (export_isaac_drone_collision.py), drone body frame (FLU), SI units',
           'drone_config': os.path.relpath(cfg['path'], os.path.expanduser(args.isaac_project)),
           'mass': round(float(m.sum()), 6),
           'com': np.round(Rbody.T @ (c_tot - pos[k]), 6).tolist(),
           'inertia_about_com': np.round(I, 8).tolist(),
           'link_masses': {n.split('/')[-1]: round(float(v), 6) for n, v in zip(info['bodies'], m)},
           'rotors': rotors,
           'rotor_constant': fl['rotor_constant'], 'rolling_moment_coefficient': fl['rolling_moment_coefficient'],
           'max_rotor_velocity': fl['max_rotor_velocity'], 'linear_drag': fl['linear_drag'],
           'px4': cfg.get('px4')}
    with open(os.path.join(args.out, 'isaac_flight.yaml'), 'w') as f:
        f.write('# Generated by export_isaac_drone_collision.py from Isaac Sim (PhysX values of the flying drone). Do not edit.\n')
        yaml.safe_dump(out, f, sort_keys=False, default_flow_style=None)
    return [f'mass {out["mass"]} kg, COM {out["com"]} m, inertia diag {np.round(np.diag(I), 6).tolist()} kg m2',
            *[f'  {r["name"]}: {r["position"]} axis {r["thrust_axis"]} dir {r["direction"]}' for r in rotors]]


try:
    main()
finally:
    app.close()
