import sys,math,unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/embodied/semantic_navigation'))
from world import Grid,build_rooms,region_at,candidates
class SemanticWorldTests(unittest.TestCase):
 def scene(self):
  a=np.zeros((30,60),np.int16);a[[0,-1],:]=100;a[:,[0,-1]]=100;a[:,30]=100;a[13:17,30]=0
  return Grid(a,60,30,.1,[0.,0.])
 def test_door_separates_two_regions_and_requires_repeated_visual_votes(self):
  grid=self.scene();obs=[dict(position=[1,1],room='厨房',confidence=.9),dict(position=[1.5,1.5],room='厨房',confidence=.8),dict(position=[4,1],room='客厅',confidence=.9),dict(position=[4.5,1.5],room='客厅',confidence=.9)]
  rooms,labels,stride=build_rooms(grid,obs);self.assertEqual({r['name'] for r in rooms},{'厨房','客厅'});self.assertNotEqual(region_at(grid,labels,stride,[1,1]),region_at(grid,labels,stride,[4,1]))
  rooms,_,_=build_rooms(grid,obs[:1]);self.assertTrue(all(r['name']=='未知' for r in rooms))
 def test_candidates_are_derived_from_object_and_exclude_obstacles_unknown_and_edges(self):
  grid=self.scene();grid.a[2:8,2:8]=-1
  result=candidates(grid,[1.5,1.5],[1,1]);self.assertTrue(result)
  for goal in result:
   self.assertTrue(grid.clear(goal['x'],goal['y']));self.assertTrue(.67<=math.dist([goal['x'],goal['y']],[1.5,1.5])<=.79)
  self.assertFalse(grid.clear(.3,.3));self.assertFalse(grid.clear(3,1))
 def test_rotated_map_origin_roundtrip(self):
  grid=Grid(np.zeros((20,20)),20,20,.1,[5,-3],math.pi/2)
  self.assertEqual(grid.cell(*grid.point(7,4)),(7,4))

class VLMRegionValidationTests(unittest.TestCase):
 def test_overlapping_rooms_are_rejected_independent_of_proposal_order(self):
  from world import regions_from_proposals
  grid=Grid(np.zeros((40,40)),40,40,.1,[0,0]);objects=[dict(id='a',position=[1,1,.8]),dict(id='b',position=[2,2,.8])]
  proposals=[dict(name='厨房',bounds_xy=[[.5,.5],[2.5,2.5]],object_ids=['a'],confidence=.9),dict(name='卧室',bounds_xy=[[1.5,1.5],[3,3]],object_ids=['b'],confidence=.9)]
  for rows in (proposals,proposals[::-1]):
   rooms,labels,_=regions_from_proposals(grid,rows,objects);self.assertEqual(rooms,[]);self.assertFalse(labels.any())

 def test_only_observed_objects_can_support_regions_and_obstacles_stay_excluded(self):
  from world import regions_from_proposals
  a=np.zeros((30,30),np.int16);a[10:15,10:15]=100;grid=Grid(a,30,30,.1,[0,0]);objects=[dict(id='table_1',position=[1.,1.,.8])]
  p=dict(name='厨房',bounds_xy=[[.5,.5],[2,2]],object_ids=['invented'],confidence=.9)
  rooms,_,_=regions_from_proposals(grid,[p],objects);self.assertEqual(rooms,[])
  p['object_ids']=['table_1'];rooms,labels,stride=regions_from_proposals(grid,[p],objects);self.assertEqual(len(rooms),1);self.assertEqual(labels[5,5],0)
  p['confidence']=.3;self.assertEqual(regions_from_proposals(grid,[p],objects)[0],[])

class MapReasoningTests(unittest.TestCase):
 def test_request_constrains_names_and_known_ids_and_retains_raw_evidence(self):
  from types import SimpleNamespace
  from map_reasoning import infer_regions
  calls=[]
  def create(**kwargs):
   calls.append(kwargs)
   return SimpleNamespace(choices=[SimpleNamespace(message=SimpleNamespace(content='{"regions":[]}'))])
  client=SimpleNamespace(chat=SimpleNamespace(completions=SimpleNamespace(create=create)))
  cfg=SimpleNamespace(vision_model='local',completion_options=lambda:dict(max_tokens=512))
  grid=Grid(np.zeros((10,10)),10,10,.1,[0,0])
  objects=[dict(id='cup_1',class_id=4,class_name='bottle',position=[.5,.5,.8],observations=3)]
  regions,png,raw=infer_regions(client,cfg,grid,objects)
  self.assertEqual(regions,[]);self.assertTrue(png.startswith(b'\x89PNG'));self.assertEqual(raw,'{"regions":[]}')
  schema=calls[0]['response_format']['json_schema']['schema']['properties']['regions']['items']['properties']
  self.assertNotIn('书房/卧室',schema['name']['enum']);self.assertEqual(schema['object_ids']['items']['enum'],['cup_1'])
  self.assertEqual(calls[0]['max_tokens'],1200)
  prompt=calls[0]['messages'][0]['content'][1]['text']
  self.assertIn('"class_name": "bottle"',prompt)
  self.assertNotIn('"class_name": "table"',prompt)
  self.assertIn('可包含负数',prompt)

if __name__=='__main__':unittest.main()
