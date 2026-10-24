"""Run with the ROS environment and Agent Python dependencies."""
import asyncio
import sys
import time
import unittest
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import AsyncMock, Mock, patch
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/embodied/llm_agent'))
sys.path.insert(0,str(ROOT/'src/embodied/semantic_navigation'))
from navigator import SemanticNavigator
from world import Grid,regions_from_proposals


class NavigationTests(unittest.IsolatedAsyncioTestCase):
    def test_current_map_pose_composes_fresh_odometry_with_bounded_delayed_correction(self):
        import math
        from navigator import base
        def transform(x,y,yaw,stamp):
            return NS(header=NS(stamp=NS(sec=99,nanosec=int(stamp*1e9))),transform=NS(translation=NS(x=x,y=y,z=0.),rotation=NS(x=0.,y=0.,z=math.sin(yaw/2),w=math.cos(yaw/2))))
        odom=transform(2.,0.,0.,.9);correction=transform(5.,1.,math.pi/2,.4)
        node=NS(tf_buffer=NS(lookup_transform=lambda target,source,*a:odom if target=='odom' else correction),get_clock=lambda:NS(now=lambda:NS(nanoseconds=100*10**9)))
        self.assertTrue(np.allclose(base(node),[5,3,math.pi/2]))
        odom.header.stamp.nanosec=500000000
        with self.assertRaises(ValueError):base(node)

    async def test_vlm_guess_dispatches_navigation_then_authorizes_search_without_room_boundary(self):
        robot,_=self.setup_robot(False)
        robot._node.semantic_world['navigation_hints']=[dict(name='厨房',position=[2.,2.],confidence=.4,reason='相机看到橱柜',search_radius=3.)]
        nav=SemanticNavigator(robot)
        with patch('navigator.base',return_value=(2,2,0)),patch('navigator.classify') as classify:
            ok,message=await nav.navigate('厨房')
        self.assertTrue(ok,message);self.assertIn('估计',message)
        robot._go_pose.assert_awaited_once();classify.assert_not_called()
        self.assertEqual(nav.room['boundary_source'],'vlm_navigation_hint')
        self.assertEqual(robot._node.current_scene,'未知')
        self.assertEqual(robot._node.semantic_phase,'room_search_ready')

    async def test_missing_semantic_publisher_or_inactive_map_fails_without_long_exploration_wait(self):
        for missing in ('semantic','map'):
            robot,_=self.setup_robot();node=robot._node;node._nav_cancel_requested=False
            if missing=='semantic':node.semantic_world={};node.semantic_received=0.
            else:node.semantic_grid=None
            clock=[100.];node.semantic_received=100. if missing=='map' else 0.
            async def advance(seconds):clock[0]+=seconds
            with patch('navigator.time.monotonic',side_effect=lambda:clock[0]),patch('navigator.asyncio.sleep',side_effect=advance):
                ok,message=await SemanticNavigator(robot).navigate('厨房')
            self.assertFalse(ok);self.assertIn('未发送导航目标',message)
            self.assertLess(clock[0],111.)
            robot._go_pose.assert_not_awaited()
            self.assertEqual(node.semantic_phase,'room_navigation_failed')

    def setup_robot(self,rooms=True):
        grid=Grid(np.zeros((60,60)),60,60,.1,[0,0])
        regions,_,_=regions_from_proposals(grid,[dict(name='厨房',bounds_xy=[[1,1],[3,3]],object_ids=['table'],confidence=.9)],[dict(id='table',position=[2,2,.8])])
        node=NS(semantic_grid=grid,semantic_received=time.monotonic(),semantic_world=dict(exploration_status='complete',rooms=regions if rooms else [],objects=[]),current_scene='厨房',get_logger=lambda:NS(info=Mock()))
        robot=NS(_node=node,_go_pose=AsyncMock(return_value=(True,'到达')),vision_client=object(),_loop=asyncio.get_running_loop())
        return robot,regions[0]

    async def test_unknown_kitchen_never_dispatches_goals_or_reuses_previous_room(self):
        robot,room=self.setup_robot(False);nav=SemanticNavigator(robot);nav.room=room
        ok,msg=await nav.navigate('厨房')
        self.assertFalse(ok);self.assertIn('未发送导航目标',msg)
        robot._go_pose.assert_not_awaited();self.assertIsNone(nav.room)
        self.assertEqual(robot._node.current_scene,'未知')
        self.assertEqual(robot._node.semantic_phase,'room_navigation_failed')

    async def test_successful_nav2_outside_room_still_fails(self):
        robot,_=self.setup_robot();nav=SemanticNavigator(robot)
        with patch('navigator.base',return_value=(4,4,0)):
            ok,msg=await nav.navigate('厨房')
        self.assertFalse(ok);self.assertIn('不在目标房间',msg);self.assertIsNone(nav.room)

    async def test_visual_failure_cannot_authorize_pick_and_success_can(self):
        for observed,expected in (('卧室',False),('厨房',True)):
            robot,_=self.setup_robot();nav=SemanticNavigator(robot)
            robot._node.get_camera_base64=Mock(return_value='image')
            with ThreadPoolExecutor(max_workers=1) as executor:
                robot._exec=executor
                with patch('navigator.base',return_value=(2,2,0)),patch('navigator.classify',return_value=dict(room=observed,confidence=.9)):
                    ok,_=await nav.navigate('厨房')
            self.assertEqual(ok,expected);self.assertEqual(nav.room is not None,expected)
            if not expected:
                self.assertFalse((await nav.approach(9))[0])

if __name__=='__main__':unittest.main()
