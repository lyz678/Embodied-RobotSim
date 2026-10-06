"""COLLADA -> USD for the project's static Gazebo furniture assets.

Supports their mesh primitives, independent normal/UV indices, material
bindings and nested scene matrices. Fails on unsupported geometry rather than
silently replacing it with boxes. Animation is not used by these SDF props.
"""
from pathlib import Path
from urllib.parse import unquote
import xml.etree.ElementTree as ET

from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade
from x_bot_scene_assets.sdf_scene import safe_name


def convert_collada(source, output):
    source, output = Path(source), Path(output)
    document = ET.parse(source).getroot()
    for element in document.iter():
        element.tag = element.tag.split('}')[-1]
    ids = {element.get('id'): element for element in document.iter() if element.get('id')}
    stage = Usd.Stage.CreateNew(str(output))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1)
    root = UsdGeom.Xform.Define(stage, '/Asset')
    stage.SetDefaultPrim(root.GetPrim())
    unit = float(document.find('asset/unit').get('meter', '1'))
    root.AddScaleOp().Set(Gf.Vec3f(unit))
    axis = document.findtext('asset/up_axis') or 'Y_UP'
    if axis == 'Y_UP':
        root.AddRotateXOp().Set(90)
    elif axis != 'Z_UP':
        raise ValueError(f'Unsupported COLLADA axis {axis}: {source}')
    materials = {}
    for mat in document.findall('library_materials/material'):
        effect = ids[mat.find('instance_effect').get('url').lstrip('#')]
        technique = effect.find('profile_COMMON/technique')
        model = next(iter(technique))
        material_path = '/Asset/Materials/' + safe_name(mat.get('id'))
        material = UsdShade.Material.Define(stage, material_path)
        shader = UsdShade.Shader.Define(stage, material_path + '/Surface')
        shader.CreateIdAttr('UsdPreviewSurface')
        shader.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(.7)
        diffuse = model.find('diffuse')
        color = diffuse.find('color') if diffuse is not None else None
        if color is not None:
            shader.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*[float(v) for v in color.text.split()][:3]))
        texture = diffuse.find('texture') if diffuse is not None else None
        if texture is not None:
            image_id = texture.get('texture')
            profile = effect.find('profile_COMMON')
            # Resolve sampler -> surface -> image for standard COLLADA;
            # this project's FBX exports usually reference image IDs directly.
            params = {p.get('sid'): p for p in profile.findall('newparam')}
            if image_id in params:
                image_id = params[image_id].findtext('sampler2D/source') or image_id
            if image_id in params:
                image_id = params[image_id].findtext('surface/init_from') or image_id
            image = ids.get(image_id)
            if image is None:
                raise ValueError(f'Unknown COLLADA texture {image_id}: {source}')
            path = (source.parent / unquote(image.findtext('init_from'))).resolve()
            if not path.is_file():
                matches = list(source.parent.parent.rglob(path.name))
                if len(matches) != 1:
                    raise FileNotFoundError(path)
                path = matches[0].resolve()
            reader = UsdShade.Shader.Define(stage, material_path + '/UV')
            reader.CreateIdAttr('UsdPrimvarReader_float2')
            reader.CreateInput('varname', Sdf.ValueTypeNames.String).Set('st')
            reader.CreateOutput('result', Sdf.ValueTypeNames.Float2)
            image_shader = UsdShade.Shader.Define(stage, material_path + '/Texture')
            image_shader.CreateIdAttr('UsdUVTexture')
            image_shader.CreateInput('file', Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(str(path)))
            image_shader.CreateInput('sourceColorSpace', Sdf.ValueTypeNames.Token).Set('sRGB')
            image_shader.CreateInput('st', Sdf.ValueTypeNames.Float2).ConnectToSource(reader.ConnectableAPI(), 'result')
            image_shader.CreateOutput('rgb', Sdf.ValueTypeNames.Float3)
            shader.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).ConnectToSource(image_shader.ConnectableAPI(), 'rgb')
        shader.CreateOutput('surface', Sdf.ValueTypeNames.Token)
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), 'surface')
        materials[mat.get('id')] = material

    def mesh(instance, parent):
        geometry = ids[instance.get('url').lstrip('#')].find('mesh')
        if geometry is None:
            raise ValueError(f'Unsupported COLLADA non-mesh geometry: {source}')
        arrays = {}
        for src in geometry.findall('source'):
            accessor = src.find('technique_common/accessor')
            floats = [float(v) for v in src.find('float_array').text.split()]
            stride = int(accessor.get('stride', '1'))
            offset = int(accessor.get('offset', '0'))
            count = int(accessor.get('count'))
            arrays[src.get('id')] = [floats[offset+i*stride:offset+(i+1)*stride] for i in range(count)]
        vertices = {v.get('id'): v.find("input[@semantic='POSITION']").get('source').lstrip('#') for v in geometry.findall('vertices')}
        bindings = {m.get('symbol'): m.get('target').lstrip('#') for m in instance.findall('bind_material/technique_common/instance_material')}
        index = 0
        for primitive in geometry:
            if primitive.tag in ('source', 'vertices', 'extra'):
                continue
            if primitive.tag not in ('triangles', 'polylist'):
                raise ValueError(f'Unsupported COLLADA {primitive.tag}: {source}')
            inputs = primitive.findall('input')
            stride = max(int(i.get('offset', '0')) for i in inputs) + 1
            packed = [int(v) for v in primitive.findtext('p').split()]
            counts = [3]*int(primitive.get('count')) if primitive.tag == 'triangles' else [int(v) for v in primitive.findtext('vcount').split()]
            if len(packed) != sum(counts)*stride:
                raise ValueError(f'Invalid COLLADA indices: {source}')
            position_input = next(i for i in inputs if i.get('semantic') == 'VERTEX')
            positions = arrays[vertices[position_input.get('source').lstrip('#')]]
            shape = UsdGeom.Mesh.Define(stage, parent + '/mesh_' + str(index))
            shape.CreatePointsAttr([Gf.Vec3f(*v[:3]) for v in positions])
            shape.CreateFaceVertexCountsAttr(counts)
            shape.CreateFaceVertexIndicesAttr(packed[int(position_input.get('offset','0'))::stride])
            shape.CreateSubdivisionSchemeAttr('none')
            shape.CreateDoubleSidedAttr(True)
            for entry in inputs:
                semantic = entry.get('semantic')
                if semantic not in ('NORMAL', 'TEXCOORD') or (semantic == 'TEXCOORD' and entry.get('set', '0') != '0'):
                    continue
                data = arrays[entry.get('source').lstrip('#')]
                indices = packed[int(entry.get('offset','0'))::stride]
                if semantic == 'NORMAL':
                    shape.CreateNormalsAttr([Gf.Vec3f(*data[i][:3]) for i in indices])
                    shape.SetNormalsInterpolation('faceVarying')
                else:
                    uv = UsdGeom.PrimvarsAPI(shape).CreatePrimvar('st', Sdf.ValueTypeNames.TexCoord2fArray, 'faceVarying')
                    uv.Set([Gf.Vec2f(*data[i][:2]) for i in indices])
            material = bindings.get(primitive.get('material'))
            if material:
                UsdShade.MaterialBindingAPI.Apply(shape.GetPrim()).Bind(materials[material])
            index += 1

    def node(element, parent, index):
        path = parent + '/' + safe_name(element.get('id') or element.get('name') or f'node_{index}')
        obj = UsdGeom.Xform.Define(stage, path)
        matrix = Gf.Matrix4d(1)
        for child in element:
            values = [float(v) for v in (child.text or '').split()] if child.tag in ('matrix','translate','rotate','scale') else []
            if child.tag == 'matrix':
                transform = Gf.Matrix4d(*values).GetTranspose()
            elif child.tag == 'translate':
                transform = Gf.Matrix4d().SetTranslate(Gf.Vec3d(*values))
            elif child.tag == 'rotate':
                transform = Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(*values[:3]), values[3]))
            elif child.tag == 'scale':
                transform = Gf.Matrix4d().SetScale(Gf.Vec3d(*values))
            else:
                continue
            matrix = transform * matrix
        obj.AddTransformOp().Set(matrix)
        for i, instance in enumerate(element.findall('instance_geometry')):
            mesh(instance, path + '/geometry_' + str(i))
        for i, child in enumerate(element.findall('node')):
            node(child, path, i)

    scene_id = document.find('scene/instance_visual_scene').get('url').lstrip('#')
    # Several FBX exports reuse the scene's ID for its first node; resolve
    # within library_visual_scenes rather than the document-wide ID table.
    visual_scene = next(s for s in document.findall('library_visual_scenes/visual_scene') if s.get('id') == scene_id)
    for index, element in enumerate(visual_scene.findall('node')):
        node(element, '/Asset', index)
    if not any(prim.IsA(UsdGeom.Mesh) for prim in stage.Traverse()):
        raise ValueError(f'COLLADA conversion produced no meshes: {source}')
    stage.GetRootLayer().Save()
