"""Offline command/geometry regressions, not a Gazebo/Isaac dynamics test."""
import math
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/x_bot/isaac_sim'))
from base_motion import (BaseMotion, WHEEL_NAMES, wheel_velocities, RADIUS, TRACK,
                         MAX_ANGULAR, ANGULAR_ACCEL, MAX_WHEEL_YAW_DEMAND,
                         WHEEL_YAW_ACCEL)


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
            self.assertLessEqual(abs(motion.linear-old_v),.005+1e-12)
            self.assertLessEqual(abs(motion.angular-old_w),ANGULAR_ACCEL*.005+1e-12)
            self.assertAlmostEqual(motion.angular/motion.linear,1.5)
        self.assertAlmostEqual(motion.linear,.4)
        self.assertAlmostEqual(motion.angular,.6)

    def test_saturation_preserves_curvature(self):
        motion=BaseMotion()
        for _ in range(500): motion.advance(4.,1.,.005)
        self.assertAlmostEqual(motion.linear,1.)
        self.assertAlmostEqual(motion.angular,.25)

    def test_immediate_stop_and_timeout(self):
        motion=BaseMotion()
        motion.advance(.4,.7,.05)
        self.assertEqual(motion.advance(0,0,.005),[0]*4)
        motion.advance(.4,.7,.05)
        self.assertEqual(motion.advance(.4,.7,.005,enabled=False),[0]*4)

    def test_forward_speed_limit_and_braking(self):
        motion = BaseMotion()
        for _ in range(400):
            wheels = motion.advance(2., 0., .005)
        self.assertAlmostEqual(motion.linear, 1.)
        for velocity in wheels:
            self.assertAlmostEqual(velocity * RADIUS, 1.)
        motion.advance(.1, 0., .005)
        self.assertAlmostEqual(motion.linear, .99)

    def test_invalid_and_rewind_fail_closed(self):
        for cmd in [(math.nan,0,.01),(0,math.inf,.01),(.1,.1,-.1)]:
            motion=BaseMotion(); motion.advance(.4,.7,.05)
            self.assertEqual(motion.advance(*cmd),[0]*4)

    def test_stall_does_not_jump(self):
        motion=BaseMotion(); motion.advance(.5,1.,10.)
        self.assertLessEqual(motion.linear,.05)
        self.assertLessEqual(motion.angular,ANGULAR_ACCEL*.05)

    def test_yaw_reversal_and_straight_command_unload_old_integral(self):
        motion = BaseMotion()
        motion.angular = .5
        motion.yaw_integral = .8
        motion.advance(0., -.3, .005, measured_angular=.5)
        self.assertGreater(motion.yaw_integral, 0.)
        self.assertLess(motion.yaw_integral, .8)
        motion.yaw_integral = .8
        motion.advance(.2, 0., .005, measured_angular=.3)
        for _ in range(100):
            motion.advance(.2, 0., .005, measured_angular=0.)
        self.assertLess(motion.yaw_integral, .002)

    def test_turn_braking_clears_torque_that_would_cause_overshoot(self):
        for target, measured in [(.2, .9), (.8, 1.2)]:
            motion = BaseMotion()
            motion.angular = 1.
            motion.yaw_integral = 1.
            wheels = motion.advance(0., target, .005, measured_angular=measured)
            self.assertLess(motion.yaw_integral, 1.)
            wheel_yaw = RADIUS*(wheels[1]-wheels[0])/TRACK
            self.assertLess(wheel_yaw, motion.angular + 1.)

    def test_small_nav2_corrections_keep_body_yaw_tracking(self):
        # A 20 Hz planner fluctuates slightly while a 200 Hz skid loop drives
        # a first-order plant. Discarding compensation on each reduction
        # previously lowered actual yaw to .445 for a .59 average request.
        for direction in (-1., 1.):
            motion = BaseMotion()
            actual = 0.
            samples = []
            for i in range(4000):
                target = direction * (.6 if (i//10)%2 == 0 else .58)
                wheels = motion.advance(.4, target, .005, measured_angular=actual)
                demand = RADIUS*(wheels[1]-wheels[0])/TRACK
                actual += (.35*demand-actual)*.005/.1
                if i > 2000:
                    samples.append(actual)
            self.assertAlmostEqual(sum(samples)/len(samples), direction*.59, delta=.015)
            self.assertLess(max(samples)-min(samples), .03)

    def test_feedback_and_braking_do_not_jump_wheel_targets(self):
        motion = BaseMotion()
        previous = 0.
        for i in range(600):
            target = .6 if i < 300 else -.3
            measured = .5 if i % 2 else .7
            wheels = motion.advance(.3, target, .005, measured_angular=measured)
            demand = RADIUS*(wheels[1]-wheels[0])/TRACK
            self.assertLessEqual(abs(demand-previous), WHEEL_YAW_ACCEL*.005+1e-12)
            previous = demand
        self.assertEqual(motion.advance(.3, -.3, .005, enabled=False), [0.]*4)
        self.assertEqual(motion.wheel_angular, 0.)

    def test_yaw_limit_and_bounded_skid_compensation(self):
        motion = BaseMotion()
        for _ in range(2000):
            wheels = motion.advance(0., 2., .005, measured_angular=0.)
            self.assertLessEqual(abs(motion.angular), MAX_ANGULAR)
            self.assertLessEqual(abs(RADIUS*(wheels[1]-wheels[0])/TRACK), MAX_WHEEL_YAW_DEMAND)

    def test_yaw_feedback_overcomes_skid_and_resets_on_stop(self):
        # A slipping plant only realizes 35% of ideal differential yaw, with
        # first-order response. Test tracking, reversal and stopped state.
        motion=BaseMotion(); actual=0.
        for target in (.5, -.5):
            for _ in range(2000):
                left,right,_,_=motion.advance(.1,target,.005,measured_angular=actual)
                demand=RADIUS*(right-left)/TRACK
                actual += (.35*demand-actual)*.005/.1
            self.assertAlmostEqual(actual,target,delta=.02)
        self.assertEqual(motion.advance(0,0,.005,measured_angular=actual),[0]*4)
        self.assertEqual(motion.yaw_integral,0.)
        self.assertEqual(motion.advance(.1,.5,.005,measured_angular=math.nan),[0]*4)

    def test_single_wheel_writer(self):
        source=(ROOT/'src/x_bot/isaac_sim/ros_bridge.py').read_text()
        self.assertEqual(source.count('isaacsim.core.nodes.IsaacArticulationController'),1)
        self.assertIn('list(WHEEL_NAMES)',source)
        self.assertNotIn('FrontDifferential',source)


if __name__ == '__main__': unittest.main()
