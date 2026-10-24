"""Async controller checks with moving robot/visual feedback, without a simulator."""
import asyncio
from collections import deque
from concurrent.futures import ThreadPoolExecutor
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock,patch
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/embodied/semantic_navigation'))
from world import Grid
from waypoint_reasoning import search_region
from visual_servo import Observation,ServoConfig
from servo_navigation import run,image_projection


class ServoLoopTests(unittest.IsolatedAsyncioTestCase):
    def test_current_projection_uses_fresh_latest_tf_without_requesting_future_image_tf(self):
        tf=NS(header=NS(stamp=NS(sec=99,nanosec=900000000)),transform=NS(translation=NS(x=0.,y=0.,z=0.),rotation=NS(x=0.,y=0.,z=0.,w=1.)))
        n=NS(servo_camera_info=NS(k=[100,0,50,0,100,50,0,0,1],width=100,height=100,header=NS(frame_id='camera')),
             _camera_image=NS(header=NS(frame_id='camera',stamp=NS(sec=100,nanosec=0))),
             get_clock=lambda:NS(now=lambda:NS(nanoseconds=100*10**9)),tf_buffer=NS(lookup_transform=Mock(return_value=tf)))
        self.assertEqual(image_projection(n,np.array([0,0,1.])),(.5,.5))
        self.assertEqual(n.tf_buffer.lookup_transform.call_args.args[2].nanoseconds,0)
        tf.header.stamp.sec=99;tf.header.stamp.nanosec=0
        with self.assertRaises(ValueError):image_projection(n,np.array([0,0,1.]))

    async def exercise(self,mode):
        clock=[100.];pose=np.array([0.,0.,0.]);target=np.array([1.3,.12,.8]);published=[]
        grid=Grid(np.zeros((120,120)),120,120,.1,[-6,-6])
        room=search_region(grid,dict(name='厨房',position=[0.,0.],confidence=.8,search_radius=4.))
        def publish(msg):published.append((clock[0],msg.linear.x,msg.angular.z))
        n=NS(_nav_cancel_requested=False,_nav_handle=None,semantic_grid=grid,target_tracks=deque(maxlen=100),localization_ready=lambda:True,
             pub_servo_vel=NS(publish=publish),get_logger=lambda:NS(info=Mock()),get_clock=lambda:NS(now=lambda:NS(nanoseconds=int(clock[0]*1e9))),base_stopped=lambda:True)
        def update():
            if mode=='lost' and clock[0]>101.:return
            for age in (0.,.1,.2):n.target_tracks.append(Observation(9,clock[0]-age,clock[0]-age,target.copy(),'can1'))
        def relative(point,frame,stamp=None):
            if frame=='map':return np.asarray(point)
            dx,dy=point[:2]-pose[:2];co,si=math.cos(pose[2]),math.sin(pose[2])
            return np.array([co*dx+si*dy,-si*dx+co*dy,.4 if frame=='fr3_link1' else .8])
        async def advance(dt):
            if published:
                _,v,w=published[-1];pose[:2]+=dt*v*np.array([math.cos(pose[2]),math.sin(pose[2])]);pose[2]+=dt*w
            clock[0]+=dt;update()
            if mode=='cancel' and clock[0]>100.5:n._nav_cancel_requested=True
        update()
        with ThreadPoolExecutor(max_workers=1) as executor:
            robot=NS(_node=n,_loop=asyncio.get_running_loop(),_exec=executor)
            with patch('navigator.base',side_effect=lambda node,frame='map':tuple(pose)),patch('servo_navigation.point_in_frame',side_effect=lambda node,p,frame,stamp=None:relative(p,frame,stamp)),patch('servo_navigation.image_projection',return_value=(.5,.5)):
                result=await run(robot,room,9,config=ServoConfig(timeout=30.,lost_timeout=3.),clock=lambda:clock[0],sleep=advance)
        return result,published,n,pose

    async def test_visual_feedback_converges_and_finishes_with_stop(self):
        result,published,node,pose=await self.exercise('normal')
        self.assertTrue(result[0],result[1]);self.assertEqual(node.semantic_phase,'visual_servo_ready')
        self.assertEqual(published[-1][1:],(0.,0.));self.assertTrue(any(row[1]>.01 for row in published))

    async def test_detection_loss_stops_translation_and_does_not_use_memory(self):
        result,published,node,_=await self.exercise('lost')
        self.assertFalse(result[0]);self.assertIn('重新观测',result[1])
        self.assertTrue(all(abs(row[1])<1e-8 for row in published if row[0]>101.4))
        self.assertEqual(published[-1][1:],(0.,0.))

    async def test_cancel_stops_without_an_extra_forward_command(self):
        result,published,_,_=await self.exercise('cancel')
        self.assertFalse(result[0]);self.assertIn('取消',result[1]);self.assertEqual(published[-1][1:],(0.,0.))

if __name__=='__main__':unittest.main()
