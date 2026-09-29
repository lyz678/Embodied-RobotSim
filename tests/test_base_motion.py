"""Offline command/geometry regressions, not a Gazebo/Isaac dynamics test."""
import math
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/x_bot/isaac_sim'))
from base_motion import BaseMotion, WHEEL_NAMES, wheel_velocities, RADIUS, TRACK


class BaseMotionTests(unittest.TestCase):
    def test_straight_and_reverse_have_no_yaw(self):
        for v in (.3, -.15):
            wheels=wheel_velocities(v,0)
            self.assertEqual(len(set(wheels)),1)
            self.assertAlmostEqual(wheels[0]*RADIUS,v)

    def test_turn_sign_and_axle_consistency(self):
        for w in (-.5,.5):
            left,right,rear_left,rear_right=wheel_velocities(.2,w)
            self.assertEqual((left,right),(rear_left,rear_right))
            self.assertAlmostEqual(RADIUS*(right-left)/TRACK,w)
            self.assertAlmostEqual(RADIUS*(right+left)/2,.2)

    def test_explicit_joint_mapping(self):
        targets=dict(zip(WHEEL_NAMES,wheel_velocities(0,.5)))
        self.assertLess(targets['front_left_wheel_joint'],0)
        self.assertGreater(targets['front_right_wheel_joint'],0)
        self.assertEqual(targets['front_left_wheel_joint'],targets['back_left_wheel_joint'])

    def test_acceleration_and_curvature(self):
        motion=BaseMotion()
        for _ in range(200):
            old_v,old_w=motion.linear,motion.angular
            motion.advance(.4,.6,.005)
            self.assertLessEqual(abs(motion.linear-old_v),.0025+1e-12)
            self.assertLessEqual(abs(motion.angular-old_w),.005+1e-12)
            self.assertAlmostEqual(motion.angular/motion.linear,1.5)
        self.assertAlmostEqual(motion.linear,.4)
        self.assertAlmostEqual(motion.angular,.6)

    def test_saturation_preserves_curvature(self):
        motion=BaseMotion()
        for _ in range(300): motion.advance(1.,1.,.005)
        self.assertAlmostEqual(motion.linear,.5)
        self.assertAlmostEqual(motion.angular,.5)

    def test_immediate_stop_and_timeout(self):
        motion=BaseMotion()
        motion.advance(.4,.7,.05)
        self.assertEqual(motion.advance(0,0,.005),[0]*4)
        motion.advance(.4,.7,.05)
        self.assertEqual(motion.advance(.4,.7,.005,enabled=False),[0]*4)

    def test_invalid_and_rewind_fail_closed(self):
        for cmd in [(math.nan,0,.01),(0,math.inf,.01),(.1,.1,-.1)]:
            motion=BaseMotion(); motion.advance(.4,.7,.05)
            self.assertEqual(motion.advance(*cmd),[0]*4)

    def test_stall_does_not_jump(self):
        motion=BaseMotion(); motion.advance(.5,1.,10.)
        self.assertLessEqual(motion.linear,.025)
        self.assertLessEqual(motion.angular,.05)

    def test_single_wheel_writer(self):
        source=(ROOT/'src/x_bot/isaac_sim/ros_bridge.py').read_text()
        self.assertEqual(source.count('isaacsim.core.nodes.IsaacArticulationController'),1)
        self.assertIn('list(WHEEL_NAMES)',source)
        self.assertNotIn('FrontDifferential',source)


if __name__ == '__main__': unittest.main()
