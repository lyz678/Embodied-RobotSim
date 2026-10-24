"""Mapping completion exports summaries and requests the global 3D snapshot."""
import json
import sys
import tempfile
import threading
import time
import unittest
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/embodied/semantic_navigation'))
from semantic_world import SemanticWorld
from world import Grid
from storage import geometry_signature,load_state


class HandoffTests(unittest.TestCase):
    def test_missing_grid_still_publishes_health_without_overwriting_saved_summary(self):
        node=NS(grid=None,status='complete',pub=NS(publish=Mock()))
        SemanticWorld.tick(node)
        data=json.loads(node.pub.publish.call_args.args[0].data)
        self.assertEqual(data['semantic_status'],'waiting_map')
        self.assertIn('/map',data['last_error'])
        self.assertEqual(data['rooms'],[])

    def test_geometry_completion_saves_structured_state_and_voxel_services(self):
        with tempfile.TemporaryDirectory() as temp:
            bundle=Path(temp)/'house';bundle.mkdir()
            for name in ('bundle.json','map.yaml','map.pgm','map.pcd'):(bundle/name).write_text(name)
            clients={}
            for scope in ('global',):
                future=Future();future.set_result(NS(success=True,message='OK'))
                clients[scope]=NS(service_is_ready=lambda:True,call_async=Mock(return_value=future))
            node=NS(grid=Grid(np.zeros((20,20)),20,20,.1,[0,0]),status='complete',snapshot_pending={},snapshots={},snapshot_attempt=-100.,snapshot_clients=clients,
                    objects=[],observations=[],image=None,pending=None,last_map_reasoning=time.monotonic(),last_vision=0,proposals=[],lock=threading.RLock(),error=None,
                    bundle=bundle,path=Path(str(bundle)+'.semantic.json'),pub=NS(publish=Mock()))
            SemanticWorld.tick(node)
            for client in clients.values():client.call_async.assert_called_once()
            SemanticWorld.tick(node)
            data=load_state(node.path,geometry_signature(bundle))
            self.assertEqual(data['schema'],3)
            self.assertEqual(data['semantic_status'],'unconfirmed')
            self.assertEqual(data['exploration_status'],'complete')
            self.assertTrue(all(row['saved'] for row in data['voxel_snapshots'].values()))
            self.assertEqual(data['voxel_snapshots']['global']['frame'],'odom')
            self.assertTrue(Path(str(node.path)+'.gz').exists())
            self.assertEqual(json.loads(node.pub.publish.call_args.args[0].data),data)

if __name__=='__main__':unittest.main()
