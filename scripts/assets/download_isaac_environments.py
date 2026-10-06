#!/usr/bin/env python3
"""Run with Isaac python.sh to collect Office and Simple Room for offline use."""
import json
import subprocess
from pathlib import Path
import os
from scene_sources import SOURCE_REVISION
from isaacsim import SimulationApp

app = SimulationApp({'headless': True})
from omni.kit.usd.collect import Collector

asset_root = 'https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.1'
destination = Path(os.environ.get('ISAAC_ASSETS_PATH', str(Path.home() / 'isaacsim_assets/6.1')))
try:
    for folder, filename in [('Office', 'office.usd'), ('Simple_Room', 'simple_room.usd')]:
        output = destination / folder
        output.mkdir(parents=True, exist_ok=True)
        if (output / 'download.json').is_file() and (output / filename).is_file():
            print(f'Already collected: {output / filename}', flush=True)
            continue
        source = f'{asset_root}/Isaac/Environments/{folder}/{filename}'
        print(f'Collecting {source} -> {output}', flush=True)
        collector = Collector(usd_path=source, collect_dir=str(output))
        success, path = app.run_coroutine(collector.collect())
        if not success:
            raise RuntimeError(f'Asset collection failed: {source}')
        (output / 'download.json').write_text(json.dumps({'source': source, 'usd': str(path)}, indent=2))
        print(f'COLLECTED {path}', flush=True)
    # Reuse the exact textured objects from the main-branch grasping test.
    import isaacsim.core.experimental.utils.app as app_utils
    app_utils.enable_extension('omni.kit.asset_converter')
    import omni.kit.asset_converter as converter
    repository = Path(__file__).resolve().parents[2]
    objects = {
        'coke': ('Coke', 'coke.obj'),
        'cup': ('ACE_Coffee_Mug_Kristen_16_oz_cup', 'model.obj'),
        'book': ('Eat_to_Live_The_Amazing_NutrientRich_Program_for_Fast_and_Sustained_Weight_Loss_Revised_Edition_Book', 'model.obj'),
    }
    for name, (model, mesh) in objects.items():
        prefix = f'src/x_bot/models/simple_house/{model}'
        source_dir = destination / 'PickObjects/source' / model
        paths = subprocess.check_output(['git', 'ls-tree', '-r', '--name-only', SOURCE_REVISION, prefix], cwd=repository, text=True).splitlines()
        if not paths:
            raise RuntimeError(f'Missing main-branch asset: {model}')
        for path in paths:
            if '/thumbnails/' in path:
                continue
            target = source_dir / Path(path).relative_to(prefix)
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(subprocess.check_output(['git', 'show', f'{SOURCE_REVISION}:{path}'], cwd=repository))
        output = destination / 'PickObjects' / f'{name}.usd'
        context = converter.AssetConverterContext()
        context.ignore_materials = False
        context.use_meter_as_world_unit = True
        task = converter.get_instance().create_converter_task(str(source_dir / 'meshes' / mesh), str(output), None, context)
        if not app.run_coroutine(task.wait_until_finished()):
            raise RuntimeError(f'Cannot convert {name}: {task.get_error_message()}')
        from pxr import Usd, UsdGeom
        converted = Usd.Stage.Open(str(output))
        # Original OBJ vertices use Z-up metres; referencing does not rotate them.
        UsdGeom.SetStageUpAxis(converted, UsdGeom.Tokens.z)
        converted.GetRootLayer().Save()
        print(f'CONVERTED {output}', flush=True)
finally:
    app.close()
