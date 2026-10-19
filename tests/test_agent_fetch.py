"""Fetch ordering, noisy/stale grasp handling, and failure propagation."""
import sys
import copy
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import AsyncMock,Mock,patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/embodied/llm_agent'))
from agent_server import Qwen3Agent,LLMAgentNode,LOCATIONS,OBJECTS
from fetch_policy import explicit_fetch,stable_grasp

def detection(x=1.,stamp=1,q=(0,0,0,1)):
    pose=NS(position=NS(x=x,y=0.,z=.8),orientation=NS(**dict(zip(('x','y','z','w'),q))))
    return NS(header=NS(frame_id='odom',stamp=NS(sec=stamp,nanosec=0)),results=[NS(pose=NS(pose=pose),hypothesis=NS(class_id='9',score=.8))])

class FetchPolicyTests(unittest.IsolatedAsyncioTestCase):
    def test_settling_uses_pose_window_and_requires_fresh_data(self):
        for moving,fresh,expected in ((False,True,True),(True,True,False),(False,False,False)):
            tick=[0.]
            node=NS(_base_pose=(0.,0.,0.),_base_speed=(.025,.01),_odom_received=0.,get_logger=lambda:NS(warn=Mock()))
            def advance(dt):
                tick[0]+=dt
                if fresh:node._odom_received=tick[0]
                node._base_pose=(tick[0]*.05 if moving else 0.,0.,0.)
            with patch('agent_server.time.monotonic',side_effect=lambda:tick[0]),patch('agent_server.time.sleep',side_effect=advance):
                self.assertEqual(LLMAgentNode.base_stopped(node,2.),expected)

    def test_arrival_rejects_far_stale_or_turning_base(self):
        def check(x=3.76, stamp=100, stopped=True):
            import math
            q=NS(x=0.,y=0.,z=math.sin(math.radians(3)),w=math.cos(math.radians(3)))
            tf=NS(header=NS(stamp=NS(sec=stamp,nanosec=0)),transform=NS(rotation=q,translation=NS(x=x,y=2.22)))
            node=NS(pub_vel=NS(publish=Mock()),base_stopped=lambda:stopped,tf_buffer=NS(lookup_transform=lambda *a:tf),
                get_clock=lambda:NS(now=lambda:NS(nanoseconds=100*10**9)),get_logger=lambda:NS(info=Mock()))
            return LLMAgentNode.verify_arrival(node,dict(x=3.76,y=2.22,yaw_deg=6.))[0]
        self.assertTrue(check())
        self.assertFalse(check(x=3.55))
        self.assertFalse(check(stamp=99))
        self.assertFalse(check(stopped=False))

    def test_explicit_intent_and_negation(self):
        self.assertEqual(explicit_fetch('到厨房拿可乐',LOCATIONS,OBJECTS),('厨房','可乐'))
        self.assertEqual(explicit_fetch('帮我去厨房拿瓶可乐',LOCATIONS,OBJECTS),('厨房','可乐'))
        self.assertEqual(explicit_fetch('🥤 厨房拿可乐',LOCATIONS,OBJECTS),('厨房','可乐'))
        self.assertEqual(explicit_fetch('请帮我去厨房拿一罐可乐！',LOCATIONS,OBJECTS),('厨房','可乐'))
        for text in ('不要到厨房拿可乐','到厨房拿可乐再去客厅','到厨房拿可乐吗？','如果到厨房拿可乐会怎样'):
            self.assertIsNone(explicit_fetch(text,LOCATIONS,OBJECTS))

    def test_cluster_rejects_singleton_and_outlier(self):
        self.assertIsNone(stable_grasp([detection()]))
        rows=[detection(1+i*.001,i) for i in range(3)]+[detection(3,4)]
        self.assertLess(stable_grasp(rows).results[0].pose.pose.position.x,1.01)
        self.assertIsNone(stable_grasp([detection(1,1),detection(2,2),detection(3,3)]))

    def test_old_detection_and_repeated_single_frame_cannot_form_cluster(self):
        for received,source_stamp in ((-10.,100),(0.,1),(0.,100)):
            row=detection(stamp=source_stamp)
            node=NS(_detection_lock=threading.Lock(),_detections_received=received,_detection_generation=1,
                _latest_detections=[row],get_clock=lambda:NS(now=lambda:NS(nanoseconds=100*10**9)),
                get_logger=lambda:NS(info=Mock()))
            ticks=iter(i*.01 for i in range(1000))
            with patch('agent_server.time.monotonic',side_effect=lambda:next(ticks)),patch('agent_server.time.sleep'):
                self.assertIsNone(LLMAgentNode.get_best_detection(node,9,.3))

    async def test_navigation_failure_never_picks(self):
        robot=NS(_node=NS(localization_ready=lambda:True),navigate=AsyncMock(return_value=(False,'停车失败')),pick=AsyncMock())
        agent=object.__new__(Qwen3Agent);agent._robot=robot;agent._client=None
        await agent.chat('到厨房拿可乐',[],AsyncMock())
        robot.pick.assert_not_called()

    async def test_explicit_fetch_needs_no_scene_or_llm(self):
        robot=NS(_node=NS(localization_ready=lambda:True,current_scene='未知'),navigate=AsyncMock(return_value=(True,'到达')),
                 pick=AsyncMock(return_value=(True,'接触检查通过')),recognize_scene=AsyncMock())
        agent=object.__new__(Qwen3Agent);agent._robot=robot;agent._client=None
        await agent.chat('到厨房拿可乐',[],AsyncMock())
        robot.navigate.assert_awaited_once_with('厨房');self.assertEqual(robot.pick.call_args.args[0],'可乐')
        robot.recognize_scene.assert_not_called()

    def run_pick(self,home=True,motions=None,contact=True):
        pose=detection().results[0].pose.pose
        node=NS(collection_event=Mock(),pub_vel=NS(publish=Mock()),pub_cloud_filter=NS(publish=Mock()),pub_collision_obj=NS(publish=Mock()),
                get_logger=lambda:NS(info=Mock(),error=Mock()),base_stopped=lambda:True,
                go_home_client=object(),call_trigger=Mock(return_value=home),gripper_sync=Mock(return_value=True),
                set_inference=Mock(),get_best_detection=lambda *a:NS(results=[NS(pose=NS(pose=pose))]),
                arm_pose_sync=Mock(side_effect=motions or [True,True,True]),grasp_contact=lambda:contact)
        ticks=iter(i*.1 for i in range(1000))
        with patch('agent_server.time.sleep'),patch('agent_server.time.monotonic',side_effect=lambda:next(ticks)):
            result=LLMAgentNode.pick_sequence(node,9)
        return node,result

    def test_home_failure_stops_and_restores_perception(self):
        n,r=self.run_pick(home=False);self.assertFalse(r[0]);n.gripper_sync.assert_not_called();n.set_inference.assert_called_with(True)

    def test_lift_failure_is_not_reported_success(self):
        n,r=self.run_pick(motions=[True,True,False]);self.assertFalse(r[0]);self.assertIn('lift',r[1]);self.assertEqual(n.call_trigger.call_count,1)
        self.assertTrue(n.arm_pose_sync.call_args.kwargs['cartesian'])

    def test_empty_close_stops_before_lift(self):
        n,r=self.run_pick(contact=False);self.assertFalse(r[0]);self.assertEqual(n.arm_pose_sync.call_count,2)

    def test_lift_is_vertical_and_return_checked(self):
        n,r=self.run_pick();self.assertTrue(r[0]);approach=n.arm_pose_sync.call_args_list[1].args;lift=n.arm_pose_sync.call_args_list[2].args
        self.assertEqual(approach[:2],lift[:2]);self.assertAlmostEqual(lift[2]-approach[2],.12);self.assertEqual(n.call_trigger.call_count,2)

if __name__=='__main__':unittest.main()
