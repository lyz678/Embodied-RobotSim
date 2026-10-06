"""Read the project's original SDF assets without a Gazebo installation.

Model/link/geometry poses remain separate; USD composes their rotations and
translations instead of adding Euler angles or world positions.
"""
from __future__ import annotations

import copy
from pathlib import Path
import re
import xml.etree.ElementTree as ET


WORLDS = ('simple_room', 'manipulation_test', 'small_house', 'ware_house',
          'obstacle_avoidance_test', 'empty')


def numbers(text, default):
    return [float(v) for v in text.split()] if text else list(default)


def sdf_bool(text, default=False):
    if text is None:
        return default
    value = text.strip().lower()
    if value in ('true', '1'):
        return True
    if value in ('false', '0'):
        return False
    raise ValueError(f'Invalid SDF boolean: {text!r}')


def safe_name(name):
    value = re.sub(r'[^A-Za-z0-9_]', '_', name)
    return value if value and not value[0].isdigit() else 'object_' + value


class SdfAssets:
    def __init__(self, source):
        self.source = Path(source)
        self.models = {p.parent.name: p.parent for p in (self.source / 'models').rglob('model.sdf')}

    def resolve(self, uri, base):
        if uri.startswith('model://'):
            relative = Path(uri[8:])
            direct = self.source / 'models' / relative
            if direct.exists():
                return direct
            for index, part in enumerate(relative.parts):
                if part in self.models:
                    candidate = self.models[part].joinpath(*relative.parts[index+1:])
                    if candidate.exists():
                        return candidate
        else:
            candidate = Path(base) / uri.removeprefix('file://')
            if candidate.exists():
                return candidate
        raise FileNotFoundError(f'Unresolved SDF asset {uri!r} in {base}')

    def include(self, include, base, stack=()):
        folder = self.resolve(include.findtext('uri'), base)
        sdf = folder / 'model.sdf'
        if sdf in stack:
            raise ValueError(f'Cyclic SDF include: {sdf}')
        model = copy.deepcopy(ET.parse(sdf).getroot().find('model'))
        for tag in ('name', 'pose', 'static'):
            value = include.find(tag)
            if value is None:
                continue
            if tag == 'name':
                model.set('name', value.text)
            else:
                old = model.find(tag)
                if old is not None:
                    model.remove(old)
                model.append(copy.deepcopy(value))
        return self.expand(model, folder, stack + (sdf,))

    def expand(self, model, base, stack=()):
        model.set('_asset_base', str(base))
        for include in list(model.findall('include')):
            model.append(self.include(include, base, stack))
            model.remove(include)
        for child in model.findall('model'):
            if not child.get('_asset_base'):
                self.expand(child, base, stack)
        for pose in model.findall('.//pose'):
            if pose.get('relative_to'):
                raise ValueError('SDF relative_to frames require explicit resolution')
        if model.findall('joint'):
            raise ValueError('Scene articulation joints are not supported by this static-prop importer')
        return model

    def world(self, name):
        path = self.source / 'worlds' / f'{name}.sdf'
        world = ET.parse(path).getroot().find('world')
        for include in list(world.findall('include')):
            world.append(self.include(include, path.parent))
            world.remove(include)
        for model in world.findall('model'):
            if not model.get('_asset_base'):
                self.expand(model, path.parent)
        return world

    def meshes(self, world):
        return sorted({self.resolve(mesh.findtext('uri'), model.get('_asset_base'))
                       for model in world.iter('model') for mesh in model.findall('./link/*/geometry/mesh')})
