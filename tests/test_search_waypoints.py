import json
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/embodied/semantic_navigation'))
from world import Grid
from waypoint_reasoning import search_waypoints,infer_navigation_hints,search_region


class SearchWaypointTests(unittest.TestCase):
    def test_model_selects_a_real_free_point_even_when_room_boundaries_are_unknown(self):
        grid=Grid(np.zeros((60,60)),60,60,.1,[0,0]);grid.a[25:35,25:35]=100
        candidates=search_waypoints(grid,[],[])
        self.assertTrue(candidates)
        self.assertTrue(all(grid.clear(*p['xy'],radius=.48) for p in candidates))
        calls=[]
        def create(**kwargs):
            calls.append(kwargs)
            return NS(choices=[NS(message=NS(content=json.dumps(dict(hints=[dict(name='厨房',waypoint_id=candidates[0]['id'],confidence=.3,reason='粗定位')]))))])
        client=NS(chat=NS(completions=NS(create=create)));cfg=NS(vision_model='local',completion_options=lambda:{})
        hints,png,_=infer_navigation_hints(client,cfg,grid,[],[])
        self.assertEqual(hints[0]['position'],candidates[0]['xy']);self.assertEqual(hints[0]['confidence'],.3)
        self.assertTrue(png.startswith(b'\x89PNG'))
        fields=calls[0]['response_format']['json_schema']['schema']['properties']['hints']['items']['properties']
        self.assertEqual(fields['waypoint_id']['enum'],[p['id'] for p in candidates])
        self.assertNotIn('bounds_xy',fields)

    def test_search_area_does_not_cross_a_closed_wall_or_accept_an_obstacle_goal(self):
        grid=Grid(np.zeros((60,60)),60,60,.1,[0,0]);grid.a[:,30]=100
        hint=dict(name='书房',position=[1.5,2.],confidence=.4,search_radius=3.)
        region=search_region(grid,hint)
        self.assertTrue(region)
        self.assertTrue(all(grid.point(row*2,end*2)[0]<3. for row,start,end in region['cell_runs']))
        hint['position']=[3.,2.];self.assertIsNone(search_region(grid,hint))

if __name__=='__main__':unittest.main()
