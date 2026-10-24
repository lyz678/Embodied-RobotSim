import gzip
import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/embodied/semantic_navigation'))
from storage import geometry_signature,load_state,save_state


class SemanticStorageTests(unittest.TestCase):
    def test_mapping_summary_transfers_to_embodied_and_compressed_only_restore(self):
        with tempfile.TemporaryDirectory() as temp:
            bundle=Path(temp)/'map';bundle.mkdir()
            for name in ('bundle.json','map.yaml','map.pgm','map.pcd'):
                (bundle/name).write_text(name)
            signature=geometry_signature(bundle)
            path=Path(str(bundle)+'.semantic.json')
            state=dict(frame='map',geometry_signature=signature,rooms=[dict(name='厨房')],objects=[dict(position=[-2,3,.8])])
            save_state(path,state)
            self.assertEqual(load_state(path,geometry_signature(bundle)),state)
            compressed=Path(str(path)+'.gz')
            self.assertEqual(json.loads(gzip.decompress(compressed.read_bytes())),state)
            path.unlink()
            self.assertEqual(load_state(path,signature),state)
            (bundle/'map.pcd').write_text('new point cloud')
            self.assertEqual(load_state(path,geometry_signature(bundle)),{})
            self.assertEqual(load_state(path,None),{})

    def test_missing_or_unbound_legacy_summary_is_not_navigation_evidence(self):
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/'world.json'
            self.assertEqual(load_state(path,'geometry'),{})
            save_state(path,dict(frame='map',rooms=[dict(name='厨房')]))
            self.assertEqual(load_state(path,'geometry'),{})


if __name__=='__main__':unittest.main()
