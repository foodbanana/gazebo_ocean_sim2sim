#!/usr/bin/env python3
"""Export the drone collision shapes exactly as Isaac Sim's PhysX cooks them (convex decomposition hulls) -> STL files
in the drone body frame, for the Gazebo drone (config/iris_drone.yaml collision.source: isaac). docs/scene.md

Runs with Isaac Sim's Python, using the Isaac Sim project's own drone loader (same USD, same scene-layer overrides):
    ~/isaacsim/python.sh ocean_sim2sim_gazebo/scripts/export_isaac_drone_collision.py \
        --isaac-project ~/data_collection_ur5_gripper [--out ocean_sim2sim_gazebo/models/iris_075/meshes/collision_isaac]

PhysX API: omni.physx get_physx_cooking_interface().request_convex_collision_representation (the hulls PhysX simulates).
Also writes report.txt (hull count, vertices, waist width) and raw_reference.stl (the source collision mesh, same frame).

Rigid bodies (Isaac Sim): body and rotor0..3 (props, fixed to the body by FixedJoint unless prop spin). Written to --out:
  hull_*.stl           body collision hulls, body link frame
  rotorN_hull_*.stl    rotor (prop) collision hulls (PhysX cooked convexHull), rotorN link frame
  isaac_flight.yaml    whole-drone mass properties + thrust model (as before) and `links:` per rigid body: PhysX mass,
                       COM and inertia about the COM (link frame), link pose in the body frame
and to --visual-out (default: sibling folder visual_isaac/):
  <link>_<mesh>.obj    every visual mesh (positions + normals, triangulated) in its own rigid-body link frame (m)
  materials.yaml       per visual mesh: bound material and all its authored shader inputs, or what Isaac renders if unbound
Link frames = the USD rigid-body prim frames with the 0.75 drone scale removed (= PhysX actor frames).
"""
import argparse
import copy
import os
import sys

p = argparse.ArgumentParser()
p.add_argument('--isaac-project', required=True, help='data_collection_ur5_gripper directory (has isaacsim/scripts)')
p.add_argument('--out', default=os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                             'models', 'iris_075', 'meshes', 'collision_isaac'))
p.add_argument('--visual-out', default=None, help='visual meshes + materials.yaml (default: <out>/../visual_isaac)')
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
from pxr import PhysicsSchemaTools, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, UsdUtils  # noqa: E402

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

    def cooked(prim, R, t):
        """PhysX cooked convex hulls of a collision prim -> (hull trimeshes, raw trimesh, report line) in the frame (R, t)."""
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
        raw = trimesh.Trimesh((raw_w - t) @ R, faces, process=False) if is_mesh else trimesh.convex.convex_hull((raw_w - t) @ R)
        out = []
        for c in got['convexes']:
            v = np.array([[q.x, q.y, q.z] for q in c.vertices])
            w = (np.c_[v, np.ones(len(v))] @ M)[:, :3]                # convex vertices are in the prim's local space
            out.append(trimesh.convex.convex_hull((w - t) @ R))
        return out, raw, (f'{prim.GetPath()}: approximation {approx}, {len(got["convexes"])} convex(es), '
                          f'{sum(len(c.vertices) for c in got["convexes"])} vertices')

    hulls, raws, lines = [], [], []
    rotor_hulls = {}
    for prim in dr.collision_prims(stage, info['path']):
        owner = dr._owner(prim)
        if owner == body:
            h, r, line = cooked(prim, Rb, tb)
            hulls += h
            raws.append(r)
        else:                                                         # rotor (prop) link: hulls in the rotor link frame
            Rr, tr = dr.drone_frame(stage, owner)
            h, _, line = cooked(prim, Rr, tr)
            rotor_hulls.setdefault(owner.split('/')[-1], []).extend(h)
        lines.append(line)
    if not hulls:
        raise RuntimeError('no body collision hulls')
    for i, h in enumerate(hulls):
        h.export(os.path.join(args.out, f'hull_{i:02d}.stl'))
    for name, hs in sorted(rotor_hulls.items()):
        for i, h in enumerate(hs):
            h.export(os.path.join(args.out, f'{name}_hull_{i:02d}.stl'))
        lines.append(f'{name}: {len(hs)} hull(s), {sum(len(h.vertices) for h in hs)} vertices, bounds '
                     f'{np.round(trimesh.util.concatenate(hs).bounds * 1000, 1).tolist()} mm (rotor link frame)')
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
    lines += export_visuals(stage, info)
    lines += mass_properties(cfg, info, Rb, tb)
    with open(os.path.join(args.out, 'report.txt'), 'w') as f:
        f.write('Exported by ocean_sim2sim_gazebo/scripts/export_isaac_drone_collision.py (Isaac Sim PhysX cooked hulls)\n'
                f'drone config {os.path.relpath(cfg["path"], os.path.expanduser(args.isaac_project))}, USD '
                f'{os.path.relpath(cfg["usd_abs"], os.path.expanduser(args.isaac_project))}, scale {cfg["scale"]}, body frame {body}\n'
                + '\n'.join(lines) + '\n')
    print('\n'.join(lines))
    print(f'[OK] {args.out}')


# What Isaac Sim (RTX) renders for a mesh with no bound material and no displayColor: its default material, a neutral
# gray. Measured with a render probe (2026-11-03, dome light 1000, Isaac Sim rgb annotator): an unbound quad matches a
# UsdPreviewSurface quad of diffuse 0.215 gray, roughness 0.5 (OmniPBR with default inputs, diffuse 0.2: 2 % darker)
UNBOUND_RENDER = {'kind': 'RTX default material (no material bound, no displayColor)',
                  'equivalent_preview_surface': {'diffuseColor': [0.215, 0.215, 0.215], 'roughness': 0.5, 'metallic': 0.0}}


def _value(v):
    """USD attribute value -> yaml-friendly python (assets as their authored path / URL, never a resolved local path)."""
    if isinstance(v, Sdf.AssetPath):
        return v.path
    if isinstance(v, (bool, int, str)) or v is None:
        return v
    if isinstance(v, float):
        return round(v, 7)
    try:
        return [_value(x) for x in v]
    except TypeError:
        return str(v)


def _material(prim):
    """Bound material of a mesh -> dict (path, kind, MDL source, all authored shader inputs) or the unbound render."""
    mat, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()
    if not mat:
        return {'bound': None, 'isaac_renders': copy.deepcopy(UNBOUND_RENDER)}
    out = {'bound': str(mat.GetPath()).replace('/World/Drone/', ''), 'shaders': []}
    for name in [a.GetName() for a in mat.GetPrim().GetAttributes() if a.GetName().startswith('inputs:') and a.HasAuthoredValue()]:
        out.setdefault('material_inputs', {})[name[7:]] = _value(mat.GetPrim().GetAttribute(name).Get())
    for sp in Usd.PrimRange(mat.GetPrim()):
        if not sp.IsA(UsdShade.Shader):
            continue
        sh = UsdShade.Shader(sp)
        d = {'shader': sp.GetName(), 'id': sh.GetIdAttr().Get()}
        src = sp.GetAttribute('info:mdl:sourceAsset')
        if src and src.HasAuthoredValue():
            d['mdl_source'] = src.Get().path
            d['mdl_material'] = sp.GetAttribute('info:mdl:sourceAsset:subIdentifier').Get()
        d['inputs'] = {i.GetBaseName(): _value(i.Get()) for i in sh.GetInputs() if i.GetAttr().HasAuthoredValue()}
        out['shaders'].append(d)
    return out


def _mesh_link_frame(prim, R, t, cache):
    """USD mesh -> (vertices (n, 3), triangles (m, 3), per-corner normals (m, 3, 3)) in the link frame (R, t).
    Polygons fan-triangulated; leftHanded orientation / mirroring transforms flip the winding so faces stay
    counter-clockwise seen from outside. Authored normals (constant / uniform / vertex / faceVarying) are carried over;
    without them smooth area-weighted vertex normals are computed (what Hydra / RTX does for subdivisionScheme none)."""
    m = UsdGeom.Mesh(prim)
    pts = np.array(m.GetPointsAttr().Get(), dtype=float)
    counts = np.array(m.GetFaceVertexCountsAttr().Get())
    idx = np.array(m.GetFaceVertexIndicesAttr().Get())
    M = cg.mat_np(cache.GetLocalToWorldTransform(prim))
    A = M[:3, :3]
    V = ((np.c_[pts, np.ones(len(pts))] @ M)[:, :3] - t) @ R
    flip = (m.GetOrientationAttr().Get() == UsdGeom.Tokens.leftHanded) != (np.linalg.det(A) < 0)
    normals = m.GetNormalsAttr().Get() if m.GetNormalsAttr().HasAuthoredValue() else None
    interp = m.GetNormalsInterpolation() if normals is not None else None
    if normals is not None:
        N = np.array(normals, dtype=float) @ np.linalg.inv(A).T @ R   # covariant transform, then into the link frame
        N /= np.linalg.norm(N, axis=1, keepdims=True)
    tris, corner = [], []                                             # corner = face-vertex index of each triangle corner
    k = 0
    for f, c in enumerate(counts):
        for j in range(1, c - 1):
            cs = [k, k + j, k + j + 1]
            if flip:
                cs = cs[::-1]
            corner.append(cs)
        k += c
    corner = np.array(corner)
    face_of = np.repeat(np.arange(len(counts)), counts)              # face of each face-vertex
    tris = idx[corner]
    if normals is None:
        fn = np.cross(V[tris[:, 1]] - V[tris[:, 0]], V[tris[:, 2]] - V[tris[:, 0]])   # area weighted
        vn = np.zeros_like(V)
        for j in range(3):
            np.add.at(vn, tris[:, j], fn)
        vn /= np.maximum(np.linalg.norm(vn, axis=1, keepdims=True), 1e-12)
        TN = vn[tris]
    elif interp == UsdGeom.Tokens.faceVarying:
        TN = N[corner]
    elif interp in (UsdGeom.Tokens.vertex, UsdGeom.Tokens.varying):
        TN = N[tris]
    elif interp == UsdGeom.Tokens.uniform:
        TN = N[face_of[corner]]
    else:                                                             # constant
        TN = np.broadcast_to(N[0], (*tris.shape, 3)).copy()
    return V, tris, TN


def _write_obj(path, V, tris, TN, header):
    Nu, inv = np.unique(np.round(TN.reshape(-1, 3), 5), axis=0, return_inverse=True)
    inv = inv.reshape(-1, 3) + 1
    with open(path, 'w') as f:
        f.write(header)
        f.write(''.join(f'v {x:.6f} {y:.6f} {z:.6f}\n' for x, y, z in V))
        f.write(''.join(f'vn {x:.5f} {y:.5f} {z:.5f}\n' for x, y, z in Nu))
        f.write(''.join(f'f {a + 1}//{na} {b + 1}//{nb} {c + 1}//{nc}\n'
                        for (a, b, c), (na, nb, nc) in zip(tris, inv)))


def export_visuals(stage, info):
    """Every visual mesh of the drone -> OBJ in its rigid-body link frame + materials.yaml."""
    vout = args.visual_out or os.path.join(os.path.dirname(os.path.abspath(args.out)), 'visual_isaac')
    os.makedirs(vout, exist_ok=True)
    for f in os.listdir(vout):
        if f.endswith('.obj'):
            os.remove(os.path.join(vout, f))
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    meshes, lines = {}, []
    for prim in Usd.PrimRange(stage.GetPrimAtPath(info['path'])):
        if not prim.IsA(UsdGeom.Mesh) or UsdGeom.Imageable(prim).ComputeVisibility() == UsdGeom.Tokens.invisible \
                or UsdGeom.Imageable(prim).ComputePurpose() not in (UsdGeom.Tokens.default_, UsdGeom.Tokens.render):
            continue
        owner = dr._owner(prim)
        link = owner.split('/')[-1]
        R, t = dr.drone_frame(stage, owner)
        V, tris, TN = _mesh_link_frame(prim, R, t, cache)
        # authored normals agree with the winding? (Gazebo culls back faces by winding; Isaac shades with the normals)
        gn = np.cross(V[tris[:, 1]] - V[tris[:, 0]], V[tris[:, 2]] - V[tris[:, 0]])
        agree = float(np.mean(np.einsum('ij,ij->i', gn, TN.mean(axis=1)) > 0))
        name = f'{link}_{prim.GetName()}.obj'
        rel = str(prim.GetPath()).replace(info['path'] + '/', '')
        _write_obj(os.path.join(vout, name), V, tris, TN,
                   f'# Generated by export_isaac_drone_collision.py from Isaac Sim: {rel}, {link} link frame, m. Do not edit.\n')
        m = UsdGeom.Mesh(prim)
        pv = {p.GetPrimvarName(): _value(p.Get()) for p in UsdGeom.PrimvarsAPI(prim).GetPrimvars()
              if p.GetAttr().HasAuthoredValue()}
        uv = [n for n, p in pv.items() if n in ('st', 'st0', 'uv')]
        meshes[name] = {'prim': rel, 'link': link, 'vertices': len(V), 'triangles': len(tris),
                        'normals': m.GetNormalsInterpolation() if m.GetNormalsAttr().HasAuthoredValue() else 'computed (smooth)',
                        'double_sided': bool(m.GetDoubleSidedAttr().Get()),
                        'uv': uv or None,
                        'primvars': {n: v for n, v in pv.items() if n not in ('st', 'st0', 'uv')} or None,
                        'material': _material(prim)}
        if not uv and any('mdl_source' in d for d in meshes[name]['material'].get('shaders', [])):
            meshes[name]['note'] = ('no UV primvar: OmniPBR (project_uvw false) samples every texture at uv (0, 0) -> '
                                    'one constant color per material (bilinear, wrap = mean of the 4 corner texels)')
        lines.append(f'visual {name}: {len(V)} vertices, {len(tris)} triangles, normals agree with winding '
                     f'{agree * 100:.1f} %, material {meshes[name]["material"].get("bound")}')
    with open(os.path.join(vout, 'materials.yaml'), 'w') as f:
        f.write('# Generated by export_isaac_drone_collision.py from Isaac Sim (visual meshes and their materials). Do not edit.\n')
        yaml.safe_dump({'source': 'Isaac Sim drone USD as loaded by drone.add_drone (scene-layer overrides included)',
                        'frames': 'each OBJ is in its rigid-body link frame (scale removed), m', 'meshes': meshes},
                       f, sort_keys=False, default_flow_style=None, width=120)
    return lines


def mass_properties(cfg, info, Rb, tb):
    """What PhysX simulates for the flying drone (drone_flight._mass_properties): total mass, COM and inertia about it in
    the body frame (links + parallel axis), rotor positions / thrust axes in the body frame, rotor directions and the
    thrust model -> isaac_flight.yaml (Gazebo drone: same values)."""
    app_utils.play()
    app.update()
    links = RigidPrim(info['bodies'])
    m = links.get_masses().numpy().reshape(-1)
    pos, quat = (x.numpy() for x in links.get_world_poses())
    com_l, com_q = (x.numpy().reshape(len(m), -1) for x in links.get_coms())
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
           'px4': cfg.get('px4'),
           # per rigid body (Gazebo links): PhysX mass, COM and inertia about the COM in the link frame (PhysX tensor API
           # get_coms / get_inertias, same values summed above), link pose in the body frame (position, quaternion wxyz)
           'links': {}}
    for i, n in enumerate(info['bodies']):
        Rl = Rbody.T @ Rs[i]
        out['links'][n.split('/')[-1]] = {
            'mass': round(float(m[i]), 6),
            'com': np.round(com_l[i].astype(float), 8).tolist(),
            'inertia_about_com': np.round(Il[i].astype(float), 11).tolist(),
            'pose_in_body': {'position': np.round((Rbody.T @ (pos[i] - pos[k])).astype(float), 7).tolist(),
                             'orientation_wxyz': np.round(_R_to_quat(Rl.astype(float)), 7).tolist()},
            'physx_com_orientation_wxyz': np.round(com_q[i].astype(float), 7).tolist()}
    with open(os.path.join(args.out, 'isaac_flight.yaml'), 'w') as f:
        f.write('# Generated by export_isaac_drone_collision.py from Isaac Sim (PhysX values of the flying drone). Do not edit.\n')
        yaml.safe_dump(out, f, sort_keys=False, default_flow_style=None)
    return [f'mass {out["mass"]} kg, COM {out["com"]} m, inertia diag {np.round(np.diag(I), 6).tolist()} kg m2',
            *[f'  {r["name"]}: {r["position"]} axis {r["thrust_axis"]} dir {r["direction"]}' for r in rotors],
            *[f'link {n}: mass {v["mass"]}, COM {v["com"]}, inertia {v["inertia_about_com"]}, pose in body '
              f'{v["pose_in_body"]}, PhysX COM frame {v["physx_com_orientation_wxyz"]}' for n, v in out['links'].items()]]


def _R_to_quat(R):
    """rotation matrix -> quaternion (w, x, y, z), w >= 0."""
    w = np.sqrt(max(0.0, 1.0 + np.trace(R))) / 2
    x = np.copysign(np.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = np.copysign(np.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = np.copysign(np.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    q = np.array([w, x, y, z])
    return q / np.linalg.norm(q)


try:
    main()
finally:
    app.close()
