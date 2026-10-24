"""Visual feedback convergence, identity preservation and stale-frame rejection."""
import math
import sys
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/embodied/semantic_navigation'))
from visual_servo import Observation,TargetLock,ServoConfig,command


class VisualServoTests(unittest.TestCase):
    def samples(self,p,now=10.,identity='can1'):
        return [Observation(9,now-i*.1,now-i*.1,np.asarray(p,float),identity) for i in (0,1,2)]

    def test_lock_never_switches_to_a_far_historical_or_other_live_instance(self):
        lock=TargetLock();estimate=lock.update(self.samples([1.,0.,.8]),9,10.,10.,[0,0])
        self.assertIsNotNone(estimate)
        self.assertIsNone(lock.update(self.samples([3.,0.,.8],10.2,'can2'),9,10.2,10.2,[0,0]))
        self.assertTrue(np.allclose(lock.last.position,[1,0,.8]))
        self.assertIsNotNone(lock.update(self.samples([1.02,0.,.8],10.4,'new_id'),9,10.4,10.4,[0,0]))

    def test_one_frame_repeats_or_stale_clock_cannot_authorize_motion(self):
        lock=TargetLock();sample=Observation(9,10.,10.,np.array([1.,0.,.8]),'can1')
        self.assertIsNone(lock.update([sample]*10,9,10.,10.,[0,0]))
        self.assertIsNone(lock.update(self.samples([1,0,.8]),9,10.6,10.6,[0,0]))
        self.assertIsNone(lock.update(self.samples([1,0,.8]),9,10.1,11.,[0,0]))

    def test_control_turns_before_driving_and_requires_visibility_and_arm_reach(self):
        c=ServoConfig()
        out=command([1.,1.,.8],[1.,1.,.4],[.4,.5],c)
        self.assertEqual(out.linear,0.);self.assertGreater(out.angular,0.)
        out=command([.5,0,.8],[.5,0,.4],[.5,.5],c)
        self.assertLess(out.linear,0.);self.assertFalse(out.ready)
        self.assertFalse(command([.69,0,.8],[.69,0,1.],[.5,.5],c).ready)
        self.assertFalse(command([.69,0,.8],[.69,0,.2],None,c).ready)
        out=command([.69,0,.8],[.69,0,.2],[.5,.5],c)
        self.assertTrue(out.ready);self.assertEqual((out.linear,out.angular),(0.,0.))

    def test_closed_loop_reaches_visible_grasp_region_from_lateral_offset(self):
        c=ServoConfig();pose=np.array([0.,0.,0.]);target=np.array([1.8,.5,.8]);history=[]
        for _ in range(1000):
            dx,dy=target[:2]-pose[:2];co,si=math.cos(pose[2]),math.sin(pose[2])
            point=np.array([co*dx+si*dy,-si*dx+co*dy,.8])
            uv=(.5-.7*point[1]/max(.1,point[0]-.3),.5)
            out=command(point,[point[0],point[1],.4],uv,c);history.append(out)
            if out.ready:break
            pose[:2]+=.1*out.linear*np.array([co,si]);pose[2]+=.1*out.angular
        self.assertTrue(history[-1].ready)
        self.assertTrue(all(-c.max_reverse<=v.linear<=c.max_forward and abs(v.angular)<=c.max_angular for v in history))
        self.assertLess(len(history),800)

if __name__=='__main__':unittest.main()
