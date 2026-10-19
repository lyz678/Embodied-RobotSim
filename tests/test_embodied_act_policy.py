import sys,threading,unittest
from pathlib import Path
from collections import deque
from types import SimpleNamespace as NS
import numpy as np
sys.path[:0]=['/home/lyz/ACT',str(Path(__file__).resolve().parents[1]/'src/embodied/act_policy')]
from deploy import PolicyObservations,contact_and_home,HOME
class PolicyTests(unittest.TestCase):
 def test_initial_home_empty_close_and_no_departure_are_not_success(self):
  q=np.r_[HOME,.03]
  self.assertFalse(contact_and_home(q,False,True))
  self.assertFalse(contact_and_home(q,True,False))
  self.assertFalse(contact_and_home(np.r_[HOME,.005],True,True))
  self.assertTrue(contact_and_home(q,True,True))
 def test_policy_observation_requires_fresh_rgb_and_bracket_without_truth(self):
  o=object.__new__(PolicyObservations);o.lock=threading.RLock();o.cfg={'visual_mode':'rgb'};o.sensor_error=None
  o.node=NS(get_clock=lambda:NS(now=lambda:NS(nanoseconds=10*10**9)))
  o.images=deque([(9.9,np.zeros((320,320,3),np.uint8))]);o.joint_history=deque([(9.895,np.zeros(8),np.zeros(8)),(9.905,np.zeros(8),np.zeros(8))])
  row=o.snapshot();self.assertIsNotNone(row);self.assertNotIn('truth',row)
  o.images=deque([(9.,np.zeros((320,320,3),np.uint8))]);self.assertIsNone(o.snapshot())
  o.images=deque([(9.9,np.zeros((320,320,3),np.uint8))]);o.joint_history=deque([(9.88,np.zeros(8),np.zeros(8)),(9.92,np.zeros(8),np.zeros(8))]);self.assertIsNone(o.snapshot())
if __name__=='__main__':unittest.main()
