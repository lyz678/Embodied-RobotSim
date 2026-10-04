#!/usr/bin/env python3
"""Check migrated SDF transforms/materials and render two reference scenes.

Run using Isaac python.sh. Does not start ROS or move the robot.
"""
from pathlib import Path
import json
import os
import sys
import traceback

REPOSITORY = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY / 'src/x_bot/isaac_sim'))
from sdf_scene import SdfAssets, WORLDS, numbers, safe_name, sdf_bool
from isaacsim import SimulationApp

app = SimulationApp({'headless': True, 'renderer': 'RayTracedLighting'})
exit_code = 0
try:
    import numpy as np
    from PIL import Image
    from pxr import Gf, Usd, UsdGeom, UsdPhysics, UsdShade
    import omni.usd
    import omni.replicator.core as rep
    import carb
    from scene_builder import spawn_floor_height
    carb.settings.get_settings().set('/rtx/post/dlss/execMode', 2)
    destination = Path(os.environ.get('ISAAC_ASSETS_PATH', str(Path.home() / 'isaacsim_assets/6.1'))) / 'GazeboMain'
    assets = SdfAssets(destination / 'source')
    results = {}
    for name in WORLDS:
        source = assets.world(name)
        stage = Usd.Stage.Open(str(destination / f'{name}.usd'))
        checked = 0

        def check(element, parent, usd_name=None, inherited_static=False):
            global checked
            path = parent + '/' + (usd_name or safe_name(element.get('name')))
            prim = stage.GetPrimAtPath(path)
            assert prim.IsValid(), path
            matrix = UsdGeom.Xformable(prim).GetOrderedXformOps()[0].Get()
            values = numbers(element.findtext('pose'), [0]*6)
            angles = values[3:]
            cr, cp, cy = np.cos(angles); sr, sp, sy = np.sin(angles)
            # Independent column-vector Rz Ry Rx formula from SDF convention.
            rotation = np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
                                 [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],
                                 [-sp,cp*sr,cp*cr]])
            np.testing.assert_allclose(np.asarray(matrix)[:3,:3], rotation.T, atol=1e-9)
            np.testing.assert_allclose(matrix.ExtractTranslation(), values[:3], atol=1e-9)
            checked += 1
            static = inherited_static or sdf_bool(element.findtext('static'))
            for link in element.findall('link'):
                check(link, path, inherited_static=static)
                body = stage.GetPrimAtPath(path + '/' + safe_name(link.get('name')))
                assert body.HasAPI(UsdPhysics.RigidBodyAPI) == (not static), str(body.GetPath())
            for child in element.findall('model'):
                check(child, path, inherited_static=static)
            for tag in ('visual', 'collision'):
                for index, child in enumerate(element.findall(tag)):
                    check(child, path, f'{tag}_{index}')
                    assert any(p.IsA(UsdGeom.Gprim) for p in Usd.PrimRange(stage.GetPrimAtPath(path + f'/{tag}_{index}/shape'))), path

        for model in source.findall('model'):
            check(model, '/Scene')
        visual_meshes = textured = 0
        for prim in stage.Traverse():
            if not prim.IsA(UsdGeom.Mesh) or UsdGeom.Imageable(prim).ComputePurpose() == 'guide':
                continue
            visual_meshes += 1
            material, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()
            if material:
                for shader in Usd.PrimRange(material.GetPrim()):
                    if shader.IsA(UsdShade.Shader) and shader.GetAttribute('info:id').Get() in ('UsdUVTexture','OmniPBR'):
                        textured += 1
                        assert UsdGeom.PrimvarsAPI(prim).GetPrimvar('st').HasValue(), str(prim.GetPath())
                        break
        floor = spawn_floor_height(stage, 0, 0)
        if name == 'manipulation_test':
            # The source puts the carpet's collider below the ground plane;
            # its visual surface is raised, but physics support remains z=0.
            np.testing.assert_allclose(floor, 0, atol=1e-7)
        results[name] = {'hierarchical_transforms_checked': checked, 'visual_meshes': visual_meshes,
                         'spawn_floor_height_m': floor,
                         'textured_meshes': textured, 'manifest': json.loads((destination/f'{name}.json').read_text())}
        print(f'CHECKED {name}: transforms={checked}, meshes={visual_meshes}, textured={textured}', flush=True)

    output = REPOSITORY / 'docs/validation/gazebo_scene_migration'
    output.mkdir(parents=True, exist_ok=True)
    for name, views in {
        'simple_room': [('overview',(10,-14,13),(0,0,.2)), ('kitchen',(3.3,1.1,2.2),(4.8,2.7,.8))],
        'manipulation_test': [('table',(-1.2,-2.2,1.8),(.6,0,.6))],
    }.items():
        omni.usd.get_context().open_stage(str(destination / f'{name}.usd'))
        stage = omni.usd.get_context().get_stage()
        camera = UsdGeom.Camera.Define(stage, '/ValidationCamera')
        camera.CreateFocalLengthAttr(22)
        camera.CreateClippingRangeAttr(Gf.Vec2f(.01,100))
        transform = camera.AddTransformOp()
        render = rep.create.render_product('/ValidationCamera', (1280,720))
        rgb = rep.AnnotatorRegistry.get_annotator('rgb')
        rgb.attach([render])
        for label, eye, target in views:
            transform.Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye),Gf.Vec3d(*target),Gf.Vec3d(0,0,1)).GetInverse())
            for _ in range(40):
                app.update()
            rep.orchestrator.step(rt_subframes=8)
            pixels = rgb.get_data()
            assert pixels.size and pixels[:,:,:3].std() > 10, 'Empty render'
            file = output / f'{name}_{label}.png'
            Image.fromarray(pixels[:,:,:3]).save(file)
            print(f'RENDER {file}', flush=True)
        rgb.detach([render]); render.destroy()
    (output / 'report.json').write_text(json.dumps(results, indent=2))
except Exception:
    traceback.print_exc()
    sys.stderr.flush()
    exit_code = 1
finally:
    app.close(exit_code=exit_code)
