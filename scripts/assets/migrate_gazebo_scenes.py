#!/usr/bin/env python3
"""Convert the main branch's original worlds and textured models to local USD.

Run with Isaac python.sh. Original sources/licences stay beside the converted
assets; scene entrypoints only reference completed, verified USD files.
"""
from pathlib import Path
import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys
import tarfile
import traceback

REPOSITORY = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPOSITORY / 'src/x_bot/isaac_sim'))
from sdf_scene import SdfAssets, WORLDS, numbers, safe_name, sdf_bool


from scene_sources import SOURCE_REVISION, extract_sources


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--revision', default=SOURCE_REVISION)
    parser.add_argument('--worlds', nargs='+', choices=WORLDS, default=list(WORLDS))
    parser.add_argument('--force', action='store_true')
    args = parser.parse_args()
    destination = Path(os.environ.get('ISAAC_ASSETS_PATH', str(Path.home() / 'isaacsim_assets/6.1'))) / 'GazeboMain'
    source, revision = extract_sources(destination, args.revision)
    assets = SdfAssets(source)
    worlds = {name: assets.world(name) for name in args.worlds}
    meshes = sorted(set(mesh for world in worlds.values() for mesh in assets.meshes(world)))
    from isaacsim import SimulationApp
    app = SimulationApp({'headless': True})
    exit_code = 0
    try:
        import isaacsim.core.experimental.utils.app as app_utils
        app_utils.enable_extension('omni.kit.asset_converter')
        import omni.kit.asset_converter as converter
        from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade, PhysxSchema

        converted = {}
        for index, mesh in enumerate(meshes):
            digest = hashlib.sha256((revision + str(mesh.relative_to(source))).encode()).hexdigest()[:16]
            output = destination / 'meshes' / digest / 'mesh.usd'
            output.parent.mkdir(parents=True, exist_ok=True)
            if args.force or not output.is_file():
                context = converter.AssetConverterContext()
                context.ignore_materials = False
                context.export_preview_surface = True
                context.use_meter_as_world_unit = True
                # OBJ scan vertices are already Z-up metres. DAE metadata
                # supplies units/up-axis; let the converter transform those.
                context.convert_stage_up_z = mesh.suffix.lower() != '.obj'
                context.use_double_precision_to_usd_transform_op = True
                if mesh.suffix.lower() == '.dae':
                    from collada_asset import convert_collada
                    convert_collada(mesh, output)
                else:
                    task = converter.get_instance().create_converter_task(str(mesh), str(output), None, context)
                    if not app.run_coroutine(task.wait_until_finished()):
                        raise RuntimeError(f'Cannot convert {mesh}: {task.get_error_message()}')
                stage = Usd.Stage.Open(str(output))
                UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
                UsdGeom.SetStageMetersPerUnit(stage, 1.0)
                stage.GetRootLayer().Save()
            converted[mesh] = output
            print(f'MESH {index+1}/{len(meshes)} {mesh.name}', flush=True)

        def pose(prim, element):
            xyzrpy = numbers(element.findtext('pose'), [0]*6)
            rotation = Gf.Matrix4d(1)
            for axis, angle in zip((Gf.Vec3d.XAxis(), Gf.Vec3d.YAxis(), Gf.Vec3d.ZAxis()), xyzrpy[3:]):
                rotation = rotation * Gf.Matrix4d().SetRotate(Gf.Rotation(axis, angle * 180 / 3.141592653589793))
            rotation.SetTranslateOnly(Gf.Vec3d(*xyzrpy[:3]))
            UsdGeom.Xformable(prim).AddTransformOp().Set(rotation)

        for name, world in worlds.items():
            output = destination / f'{name}.usd'
            temporary = destination / f'{name}.building.usda'
            stage = Usd.Stage.CreateNew(str(temporary))
            UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
            UsdGeom.SetStageMetersPerUnit(stage, 1.0)
            root = UsdGeom.Xform.Define(stage, '/Scene').GetPrim()
            stage.SetDefaultPrim(root)
            counts = {'models': 0, 'links': 0, 'visuals': 0, 'collisions': 0}
            placements = []

            def geometry(element, path, base, collision, dynamic):
                node = UsdGeom.Xform.Define(stage, path).GetPrim()
                pose(node, element)
                geom = element.find('geometry')
                kind = next(iter(geom)).tag
                p = path + '/shape'
                if kind == 'mesh':
                    mesh = geom.find('mesh')
                    referenced = UsdGeom.Xform.Define(stage, p).GetPrim()
                    referenced.GetReferences().AddReference(str(converted[assets.resolve(mesh.findtext('uri'), base)]))
                    UsdGeom.Xformable(node).AddScaleOp(opSuffix='sdfScale').Set(Gf.Vec3f(*numbers(mesh.findtext('scale'), [1]*3)))
                elif kind == 'box':
                    shape = UsdGeom.Cube.Define(stage, p)
                    shape.CreateSizeAttr(1)
                    UsdGeom.Xformable(shape).AddScaleOp().Set(Gf.Vec3f(*numbers(geom.findtext('box/size'), [1]*3)))
                elif kind == 'cylinder':
                    shape = UsdGeom.Cylinder.Define(stage, p)
                    shape.CreateAxisAttr('Z')
                    shape.CreateRadiusAttr(float(geom.findtext('cylinder/radius')))
                    shape.CreateHeightAttr(float(geom.findtext('cylinder/length')))
                elif kind == 'sphere':
                    UsdGeom.Sphere.Define(stage, p).CreateRadiusAttr(float(geom.findtext('sphere/radius')))
                elif kind == 'plane':
                    # Gazebo planes collide as infinite planes regardless of
                    # visual size. A large static ground slab approximates it.
                    sx, sy = (100, 100) if collision else numbers(geom.findtext('plane/size'), [100,100])
                    normal = Gf.Vec3d(*numbers(geom.findtext('plane/normal'), [0,0,1]))
                    if collision:
                        # A box with its top at z=0 avoids oversized
                        # triangle cooking/contact warnings in PhysX.
                        shape = UsdGeom.Cube.Define(stage, p)
                        shape.CreateSizeAttr(1)
                        shape.AddOrientOp().Set(Gf.Quatf(Gf.Rotation(Gf.Vec3d.ZAxis(), normal).GetQuat()))
                        shape.AddTranslateOp().Set(Gf.Vec3d(0,0,-.05))
                        shape.AddScaleOp().Set(Gf.Vec3f(sx,sy,.1))
                    else:
                        shape = UsdGeom.Mesh.Define(stage, p)
                        shape.CreatePointsAttr([(-sx/2,-sy/2,0),(sx/2,-sy/2,0),(sx/2,sy/2,0),(-sx/2,sy/2,0)])
                        shape.CreateFaceVertexCountsAttr([4]); shape.CreateFaceVertexIndicesAttr([0,1,2,3])
                        shape.CreateSubdivisionSchemeAttr('none')
                        shape.AddOrientOp().Set(Gf.Quatf(Gf.Rotation(Gf.Vec3d.ZAxis(), normal).GetQuat()))
                else:
                    raise ValueError(f'Unsupported geometry {kind}: {path}')
                if collision:
                    UsdGeom.Imageable(node).CreatePurposeAttr('guide')
                    for prim in Usd.PrimRange(node):
                        if prim.IsA(UsdGeom.Gprim):
                            UsdPhysics.CollisionAPI.Apply(prim)
                            contact = PhysxSchema.PhysxCollisionAPI.Apply(prim)
                            contact.CreateContactOffsetAttr(.002)
                            contact.CreateRestOffsetAttr(0.0)
                            if prim.IsA(UsdGeom.Mesh):
                                approximation = 'convexDecomposition' if dynamic else 'none'
                                UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr(approximation)
                                if dynamic:
                                    # Voxel hulls without shrink-wrap can extend
                                    # centimetres beyond a thin tabletop and
                                    # hold grasp objects visibly in the air.
                                    decomposition = PhysxSchema.PhysxConvexDecompositionCollisionAPI.Apply(prim)
                                    decomposition.CreateShrinkWrapAttr(True)
                                    decomposition.CreateErrorPercentageAttr(1.0)
                                    decomposition.CreateHullVertexLimitAttr(128)
                                    decomposition.CreateMaxConvexHullsAttr(64)
                                    decomposition.CreateMinThicknessAttr(.001)
                elif kind != 'mesh' or element.find('material/pbr') is not None:
                    rgba = numbers(element.findtext('material/diffuse'), [.75,.75,.75,1])
                    material = UsdShade.Material.Define(stage, path + '/Material')
                    shader = UsdShade.Shader.Define(stage, path + '/Material/Shader')
                    shader.CreateIdAttr('UsdPreviewSurface')
                    shader.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgba[:3]))
                    shader.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(.7)
                    pbr = element.find('material/pbr/metal')
                    if pbr is not None:
                        reader = UsdShade.Shader.Define(stage, path + '/Material/UV')
                        reader.CreateIdAttr('UsdPrimvarReader_float2')
                        reader.CreateInput('varname', Sdf.ValueTypeNames.String).Set('st')
                        reader.CreateOutput('result', Sdf.ValueTypeNames.Float2)
                        for tag, input_name, channel in (('albedo_map','diffuseColor','rgb'), ('normal_map','normal','rgb'), ('roughness_map','roughness','r'), ('metalness_map','metallic','r')):
                            uri = pbr.findtext(tag)
                            if not uri:
                                continue
                            texture = UsdShade.Shader.Define(stage, path + '/Material/' + tag)
                            texture.CreateIdAttr('UsdUVTexture')
                            texture.CreateInput('file', Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(str(assets.resolve(uri, base))))
                            texture.CreateInput('sourceColorSpace', Sdf.ValueTypeNames.Token).Set('sRGB' if tag == 'albedo_map' else 'raw')
                            texture.CreateInput('st', Sdf.ValueTypeNames.Float2).ConnectToSource(reader.ConnectableAPI(), 'result')
                            if tag == 'normal_map':
                                texture.CreateInput('scale', Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(2,2,2,1))
                                texture.CreateInput('bias', Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(-1,-1,-1,0))
                            texture.CreateOutput(channel, Sdf.ValueTypeNames.Float3 if channel == 'rgb' else Sdf.ValueTypeNames.Float)
                            shader.CreateInput(input_name, Sdf.ValueTypeNames.Color3f if input_name == 'diffuseColor' else Sdf.ValueTypeNames.Normal3f if input_name == 'normal' else Sdf.ValueTypeNames.Float).ConnectToSource(texture.ConnectableAPI(), channel)
                    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), 'surface')
                    for prim in Usd.PrimRange(stage.GetPrimAtPath(p)):
                        if prim.IsA(UsdGeom.Gprim):
                            UsdShade.MaterialBindingAPI.Apply(prim).Bind(material)

            def model(element, parent, inherited_static=None):
                path = parent + '/' + safe_name(element.get('name'))
                node = UsdGeom.Xform.Define(stage, path).GetPrim()
                pose(node, element)
                node.CreateAttribute('sdf:name', Sdf.ValueTypeNames.String).Set(element.get('name'))
                counts['models'] += 1
                value = element.findtext('static')
                static = inherited_static is True or sdf_bool(value)
                base = element.get('_asset_base')
                for link in element.findall('link'):
                    link_path = path + '/' + safe_name(link.get('name'))
                    body = UsdGeom.Xform.Define(stage, link_path).GetPrim()
                    pose(body, link)
                    counts['links'] += 1
                    if not static:
                        UsdPhysics.RigidBodyAPI.Apply(body)
                        mass = float(link.findtext('inertial/mass') or '1')
                        UsdPhysics.MassAPI.Apply(body).CreateMassAttr(mass)
                    for tag in ('visual','collision'):
                        for index, element_geom in enumerate(link.findall(tag)):
                            geometry(element_geom, link_path + '/' + tag + '_' + str(index), base, tag == 'collision', not static)
                            counts[tag+'s'] += 1
                for child in element.findall('model'):
                    model(child, path, True if static else None)

            for element in world.findall('model'):
                model(element, '/Scene')
                placements.append({'name': element.get('name'), 'pose': numbers(element.findtext('pose'), [0]*6)})
            lights = UsdGeom.Xform.Define(stage, '/Scene/Lights')
            ambient = UsdLux.DomeLight.Define(stage, '/Scene/Lights/Ambient')
            ambient.CreateIntensityAttr(700)
            ambient.CreateColorAttr(Gf.Vec3f(*numbers(world.findtext('scene/ambient'), [.5,.5,.5,1])[:3]))
            ambient.GetPrim().CreateAttribute('inputs:shadow:enable', Sdf.ValueTypeNames.Bool).Set(False)
            for light in world.findall('light'):
                path = '/Scene/Lights/' + safe_name(light.get('name'))
                obj = UsdLux.DistantLight.Define(stage, path) if light.get('type') == 'directional' else UsdLux.SphereLight.Define(stage, path)
                pose(obj.GetPrim(), light)
                obj.CreateIntensityAttr(1000 * float(light.findtext('intensity') or '1'))
                obj.CreateColorAttr(Gf.Vec3f(*numbers(light.findtext('diffuse'), [1,1,1,1])[:3]))
                obj.GetPrim().CreateAttribute('inputs:shadow:enable', Sdf.ValueTypeNames.Bool).Set(sdf_bool(light.findtext('cast_shadows'), True))
            stage.GetRootLayer().Save()
            stage.GetRootLayer().Export(str(output))
            # Catch unresolved texture references before publishing a manifest.
            textures = set()
            for prim in stage.Traverse():
                for attribute in prim.GetAttributes():
                    value = attribute.Get()
                    if isinstance(value, Sdf.AssetPath) and value.path and not value.path.endswith('.mdl'):
                        if not value.resolvedPath or not Path(value.resolvedPath).is_file():
                            raise FileNotFoundError(f'Unresolved texture {value.path}: {prim.GetPath()}')
                        textures.add(value.resolvedPath)
            manifest = {'revision': revision, 'world': name, 'counts': counts,
                        'textures': len(textures), 'placements': placements,
                        'notes': ['SDF positions/rotations/scales retained; original collision geometry retained.',
                                  'Gazebo lights mapped to USD lights; brightness units differ.',
                                  'PhysX dynamic meshes use shrink-wrapped convex decomposition (1% error, 128 vertices, up to 64 hulls); static meshes use triangles.',
                                  'Infinite SDF ground planes use a 100 m wide box with its top at the plane; collision contact offset is 2 mm.']}
            output.with_suffix('.json').write_text(json.dumps(manifest, indent=2))
            print(f'WORLD {name}: {counts}, textures={len(textures)} -> {output}', flush=True)
    except Exception:
        traceback.print_exc()
        sys.stderr.flush()
        exit_code = 1
    finally:
        app.close(exit_code=exit_code)


if __name__ == '__main__':
    main()
