#!/usr/bin/env python3
"""Simulate original main scenes and measure settled visual object/table gaps.

Run using Isaac python.sh. This checks real PhysX results, including cooked
convex decomposition, rather than only checking authored SDF coordinates.
"""
from pathlib import Path
import argparse
import json
import sys
import traceback

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/x_bot/isaac_sim'))
from isaacsim import SimulationApp

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--world', choices=('manipulation_test','simple_room'), default='manipulation_test')
args = parser.parse_args()
app = SimulationApp({'headless':True, 'renderer':'RayTracedLighting'})
code = 0
try:
    import numpy as np
    from pxr import Gf, Usd, UsdGeom, UsdPhysics
    from isaacsim.core.simulation_manager import SimulationManager
    from isaacsim.core.experimental.prims import RigidPrim
    import isaacsim.core.experimental.utils.app as app_utils
    import isaacsim.core.experimental.utils.stage as stage_utils
    from scene_builder import build_scene
    from scene_contacts import visible_surface_height

    stage = stage_utils.create_new_stage()
    build_scene(stage, args.world, ROOT/'src/x_bot')
    SimulationManager.setup_simulation(dt=.005, device='cpu')
    SimulationManager.initialize_physics()
    cache = UsdGeom.XformCache()
    environment = stage.GetPrimAtPath('/World/Scene/Environment')
    if args.world == 'manipulation_test':
        names = ('kitchen_table','coke_can','coffee_mug','book')
    else:
        names = ('KitchenTable_kitchen','Coke_table_2','CoffeeCup_table_1')
    bodies = {}
    for name in names:
        model = stage.GetPrimAtPath(str(environment.GetPath())+'/'+name)
        assert model.IsValid(), name
        body_prim = next((p for p in Usd.PrimRange(model) if p.HasAPI(UsdPhysics.RigidBodyAPI)), model)
        rigid = RigidPrim(str(body_prim.GetPath())) if body_prim.HasAPI(UsdPhysics.RigidBodyAPI) else None
        matrix = cache.GetLocalToWorldTransform(body_prim)
        mesh_data = []
        for p in Usd.PrimRange(body_prim):
            if p.IsA(UsdGeom.Mesh) and '/visual_' in str(p.GetPath()):
                mesh = UsdGeom.Mesh(p)
                local = cache.GetLocalToWorldTransform(p)*matrix.GetInverse()
                points = [local.Transform(Gf.Vec3d(*v)) for v in mesh.GetPointsAttr().Get()]
                mesh_data.append((points, list(mesh.GetFaceVertexCountsAttr().Get()), list(mesh.GetFaceVertexIndicesAttr().Get())))
        assert mesh_data, name
        bodies[name] = (rigid,matrix,mesh_data)

    app_utils.play()
    while SimulationManager.get_simulation_time() < 4:
        app.update()

    def world_meshes(name):
        rigid, matrix, meshes = bodies[name]
        if rigid:
            positions, orientations = rigid.get_world_poses()
            pos, q = positions.numpy()[0], orientations.numpy()[0]
            matrix = Gf.Matrix4d().SetRotate(Gf.Quatd(float(q[0]),Gf.Vec3d(*map(float,q[1:]))))
            matrix.SetTranslateOnly(Gf.Vec3d(*map(float,pos)))
        return matrix, [(np.asarray([matrix.Transform(Gf.Vec3d(*v)) for v in points]),counts,indices) for points,counts,indices in meshes]

    _, table_meshes = world_meshes(names[0])
    measurements = {}
    for name in names[1:]:
        matrix, meshes = world_meshes(name)
        position = matrix.ExtractTranslation()
        tops = [visible_surface_height(points,counts,indices,position[0],position[1]) for points,counts,indices in table_meshes]
        top = max(v for v in tops if v is not None)
        bottom = min(float(points[:,2].min()) for points,_,_ in meshes)
        gap = bottom-top
        measurements[name] = {'visible_tabletop_z_m':top,'settled_visual_bottom_z_m':bottom,'gap_mm':gap*1000}
        print(f'CONTACT {args.world}/{name}: gap={gap*1000:.3f} mm',flush=True)
        assert -.005 <= gap <= .003, f'{name}: floated or penetrated {gap*1000:.3f} mm'
    output = ROOT/'docs/validation/gazebo_scene_migration'
    output.mkdir(parents=True,exist_ok=True)
    (output/f'{args.world}_contacts.json').write_text(json.dumps({'world':args.world,'physics_hz':200,'settle_simulation_seconds':4,'measurements':measurements},indent=2))
    print('CONTACT CHECK PASSED',flush=True)
except Exception:
    traceback.print_exc();sys.stderr.flush();code=1
finally:
    app.close(exit_code=code)
